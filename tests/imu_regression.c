/* Hardware-independent regression cases, executed as ARM machine code by Unicorn. */
#include "g354.h"
#include "attitude.h"

#include <math.h>
#include <string.h>

static bool near(float a, float b, float tolerance) {
    return isfinite(a) && fabsf(a - b) < tolerance;
}

static void checksum(uint16_t words[G354_BURST_WORDS]) {
    uint16_t sum = 0;
    for (unsigned i = 0; i < G354_BURST_WORDS - 1; ++i) {
        sum = (uint16_t)(sum + words[i]);
    }
    words[G354_BURST_WORDS - 1] = sum;
}

static void put_i32(uint16_t *words, int32_t value) {
    words[0] = (uint16_t)((uint32_t)value >> 16);
    words[1] = (uint16_t)value;
}

static void make_frame(uint16_t words[G354_BURST_WORDS]) {
    memset(words, 0, G354_BURST_WORDS * sizeof(*words));
    words[0] = 0xFE00;
    put_i32(&words[1], 172621824); /* 25 C. */
    put_i32(&words[3], 368640000); /* +90 dps. */
    put_i32(&words[5], -184320000); /* -45 dps. */
    put_i32(&words[7], 4096); /* +0.001 dps (tests lower 16 bits). */
    put_i32(&words[9], -81920000); /* -0.25 g. */
    put_i32(&words[11], 163840000); /* +0.5 g. */
    put_i32(&words[13], 327680000); /* +1 g. */
    words[15] = 0xFFFC;
    checksum(words);
}

static bool packet_cases(void) {
    uint16_t words[G354_BURST_WORDS];
    make_frame(words);
    g354_sample_t sample;
    if (g354_decode_burst(words, &sample) != G354_OK ||
        !near(sample.temperature_c, 25.0f, 0.0001f) ||
        !near(sample.gyroscope_dps[0], 90.0f, 0.0001f) ||
        !near(sample.gyroscope_dps[1], -45.0f, 0.0001f) ||
        !near(sample.gyroscope_dps[2], 0.001f, 0.000001f) ||
        !near(sample.accelerometer_g[0], -0.25f, 0.00001f) ||
        !near(sample.accelerometer_g[1], 0.5f, 0.00001f) ||
        !near(sample.accelerometer_g[2], 1.0f, 0.00001f) || sample.counter != 0xFFFC) {
        return false;
    }
    const g354_sample_t previous = sample;
    words[7] ^= 1;
    if (g354_decode_burst(words, &sample) != G354_ERROR_CHECKSUM ||
        memcmp(&sample, &previous, sizeof(sample)) != 0) {
        return false;
    }
    make_frame(words);
    words[0] |= 1;
    checksum(words);
    if (g354_decode_burst(words, &sample) != G354_ERROR_DIAGNOSTIC) {
        return false;
    }
    words[0] = 0;
    checksum(words);
    if (g354_decode_burst(words, &sample) != G354_ERROR_NOT_READY) {
        return false;
    }
    make_frame(words);
    put_i32(&words[3], INT32_MIN);
    put_i32(&words[5], INT32_MAX);
    put_i32(&words[1], -65536);
    checksum(words);
    return g354_decode_burst(words, &sample) == G354_OK &&
           near(sample.gyroscope_dps[0], -524.288f, 0.001f) &&
           near(sample.gyroscope_dps[1], 524.288f, 0.001f) &&
           near(sample.temperature_c, 25.0f + 2635.0f * 0.0037918f, 0.0001f);
}

typedef struct {
    uint16_t registers[2][64];
    uint16_t frame[G354_BURST_WORDS];
    uint16_t pending;
    uint8_t window;
    unsigned burst_index;
    bool selected;
    bool burst;
    bool fail_transfer;
    bool corrupt_configuration;
    bool protocol_error;
    uint32_t us;
    uint32_t previous_word_us;
    uint32_t reset_us;
    unsigned reset_edges;
} fake_bus_t;

static void fake_select(void *context, bool active) {
    fake_bus_t *fake = context;
    fake->selected = active;
    if (!active) {
        fake->burst = false;
    }
}

static bool fake_transfer(void *context, uint16_t tx, uint16_t *rx) {
    fake_bus_t *fake = context;
    if (!fake->selected) {
        fake->protocol_error = true;
    }
    if (fake->fail_transfer) {
        return false;
    }
    if (fake->burst) {
        const uint32_t required = fake->burst_index == 0 ? 70U : 32U;
        if (fake->us - fake->previous_word_us < required || fake->burst_index >= G354_BURST_WORDS) {
            fake->protocol_error = true;
        }
        *rx = fake->frame[fake->burst_index++];
    } else {
        if (fake->previous_word_us != 0 && fake->us - fake->previous_word_us < 40U) {
            fake->protocol_error = true;
        }
        *rx = fake->pending;
        const uint8_t address = (uint8_t)((tx >> 8) & 0x7F);
        if ((tx & 0x8000) != 0) {
            if (address == 0x7E) {
                fake->window = (uint8_t)tx & 1U;
            } else if (fake->window == 0 && address == 0) {
                fake->burst = true;
                fake->burst_index = 0;
            } else if (fake->window == 0 && address == 3) {
                fake->registers[0][1] = (tx & 0xFF) == 1 ? 0 : 0x0400;
            } else {
                uint16_t *reg = &fake->registers[fake->window][address / 2];
                if ((address & 1) != 0) {
                    *reg = (uint16_t)((*reg & 0x00FF) | ((tx & 0xFF) << 8));
                } else {
                    *reg = (uint16_t)((*reg & 0xFF00) | (tx & 0xFF));
                }
            }
        } else {
            fake->pending = fake->registers[fake->window][address / 2];
            if (fake->window == 1 && address == 0x0C && fake->corrupt_configuration) {
                fake->pending ^= 0x1000;
            }
        }
    }
    fake->previous_word_us = fake->us;
    fake->us += 25; /* 16 bits at approximately the configured 656 kHz. */
    return true;
}

static void fake_reset(void *context, bool asserted) {
    fake_bus_t *fake = context;
    ++fake->reset_edges;
    if (asserted) {
        fake->reset_us = fake->us;
        memset(fake->registers, 0, sizeof(fake->registers));
        fake->registers[0][1] = 0x0400;
        fake->registers[1][0x6A / 2] = 0x3347;
        fake->registers[1][0x6C / 2] = 0x3435;
        fake->registers[1][0x6E / 2] = 0x4450;
        fake->registers[1][0x70 / 2] = 0x3048;
        fake->registers[1][0x72 / 2] = 0x1234;
        fake->window = 0;
        fake->pending = 0;
    } else if (fake->us - fake->reset_us < 100000U) {
        fake->protocol_error = true;
    }
}

static void fake_delay_us(void *context, uint32_t us) {
    ((fake_bus_t *)context)->us += us;
}

static void fake_delay_ms(void *context, uint32_t ms) {
    ((fake_bus_t *)context)->us += ms * 1000U;
}

static uint32_t fake_millis(void *context) {
    return ((fake_bus_t *)context)->us / 1000U;
}

static bool driver_cases(void) {
    static fake_bus_t fake;
    g354_bus_t bus = {
        .context = &fake, .select = fake_select, .transfer = fake_transfer,
        .reset = fake_reset, .delay_us = fake_delay_us, .delay_ms = fake_delay_ms,
        .millis = fake_millis
    };
    g354_t device;
    if (g354_initialise(&device, &bus) != G354_OK || !device.configured ||
        strcmp(device.product_id, "G354PDH0") != 0 || device.firmware_version != 0x1234 ||
        fake.reset_edges != 2 || fake.us < 900000U || fake.selected || fake.protocol_error ||
        fake.registers[1][0x04 / 2] != 0x0200 ||
        fake.registers[1][0x02 / 2] != 0x0006 ||
        fake.registers[1][0x06 / 2] != 0x0002 ||
        fake.registers[1][0x0C / 2] != 0xF003 ||
        fake.registers[1][0x0E / 2] != 0x7000 || fake.window != 0) {
        return false;
    }
    make_frame(fake.frame);
    g354_sample_t sample;
    if (g354_start(&device) != G354_OK || !device.sampling ||
        g354_read_sample(&device, &sample) != G354_OK || fake.protocol_error || fake.selected ||
        !near(sample.gyroscope_dps[0], 90, 0.001f) ||
        g354_stop(&device) != G354_OK || device.sampling) {
        return false;
    }
    fake.corrupt_configuration = true;
    if (g354_initialise(&device, &bus) != G354_ERROR_CONFIGURATION || device.configured || fake.selected) {
        return false;
    }
    fake.corrupt_configuration = false;
    fake.fail_transfer = true;
    return g354_initialise(&device, &bus) == G354_ERROR_IO && !fake.selected && !device.configured;
}

static bool attitude_cases(void) {
    attitude_t attitude;
    attitude_output_t output;
    const float bias[3] = {0.1f, -0.05f, 0.02f};
    /* Gravity measured at known roll=30 deg, pitch=20 deg, yaw=0. */
    const float tilted[3] = {-0.3420201433f, 0.4698463104f, 0.8137976813f};
    if (!attitude_initialise(&attitude, 500, bias, tilted)) {
        return false;
    }
    for (unsigned i = 0; i < 2000; ++i) {
        if (!attitude_update(&attitude, bias, tilted, &output)) {
            return false;
        }
    }
    if (!near(output.roll_deg, 30, 0.02f) || !near(output.pitch_deg, 20, 0.02f) ||
        !near(output.yaw_deg, 0, 0.02f)) {
        return false;
    }
    const float level[3] = {0, 0, 1};
    const float turning[3] = {0.1f, -0.05f, 90.02f};
    if (!attitude_initialise(&attitude, 500, bias, level)) {
        return false;
    }
    for (unsigned i = 0; i < 500; ++i) {
        if (!attitude_update(&attitude, turning, level, &output)) {
            return false;
        }
    }
    if (!near(output.yaw_deg, 90, 0.02f) || !near(output.roll_deg, 0, 0.02f) ||
        !near(output.pitch_deg, 0, 0.02f)) {
        return false;
    }
    const float accelerated[3] = {1, 0, 1};
    if (!attitude_update(&attitude, bias, accelerated, &output) || !output.accelerometer_ignored) {
        return false;
    }
    const float overrange[3] = {0.1f, -0.05f, 445.02f};
    if (!attitude_update(&attitude, overrange, level, &output) ||
        !output.recovering || output.yaw_deg < 89.0f) {
        return false; /* Overrange recovery must not silently zero accumulated heading. */
    }
    const float invalid[3] = {NAN, 0, 1};
    return !attitude_update(&attitude, bias, invalid, &output) &&
           !attitude_initialise(&attitude, 500, bias, (const float[3]){0, 0, 0});
}

unsigned int imu_regression_main(void) {
    unsigned int failures = 0;
    if (!packet_cases()) failures |= 1;
    if (!driver_cases()) failures |= 2;
    if (!attitude_cases()) failures |= 4;
    return failures;
}
