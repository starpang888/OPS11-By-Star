#include "g354.h"

#include <string.h>

/* M-G354PDH0 Rev.20160212, sections 6-8. Register writes are byte-wide. */
enum {
    REG_MODE_CTRL = 0x02,
    REG_DIAG_STAT = 0x04,
    REG_SIG_CTRL = 0x00,
    REG_MSC_CTRL = 0x02,
    REG_SMPL_CTRL = 0x04,
    REG_FILTER_CTRL = 0x06,
    REG_UART_CTRL = 0x08,
    REG_GLOB_CMD = 0x0A,
    REG_BURST_CTRL1 = 0x0C,
    REG_BURST_CTRL2 = 0x0E,
    REG_POL_CTRL = 0x10,
    REG_PROD_ID = 0x6A,
    REG_VERSION = 0x72,
    REG_WIN_CTRL = 0x7E,
    NORMAL_STALL_US = 40,
    BURST_START_US = 45,
    BURST_STALL_US = 16
};

static g354_status_t write_byte(g354_t *device, uint8_t address, uint8_t value) {
    uint16_t unused;
    const g354_bus_t *bus = &device->bus;
    bus->select(bus->context, true);
    bus->delay_us(bus->context, 1);
    const bool ok = bus->transfer(bus->context, (uint16_t)(((address | 0x80U) << 8) | value), &unused);
    bus->delay_us(bus->context, NORMAL_STALL_US);
    bus->select(bus->context, false);
    bus->delay_us(bus->context, 1);
    return ok ? G354_OK : G354_ERROR_IO;
}

static g354_status_t select_window(g354_t *device, uint8_t window) {
    return write_byte(device, REG_WIN_CTRL, window);
}

static g354_status_t read_word(g354_t *device, uint8_t address, uint16_t *value) {
    uint16_t unused;
    const g354_bus_t *bus = &device->bus;
    bus->select(bus->context, true);
    bus->delay_us(bus->context, 1);
    bool ok = bus->transfer(bus->context, (uint16_t)(address << 8), &unused);
    bus->delay_us(bus->context, NORMAL_STALL_US);
    if (ok) {
        ok = bus->transfer(bus->context, 0, value); /* Pipelined response to the preceding command. */
        bus->delay_us(bus->context, NORMAL_STALL_US);
    }
    bus->select(bus->context, false);
    bus->delay_us(bus->context, 1);
    return ok ? G354_OK : G354_ERROR_IO;
}

static g354_status_t poll_clear(g354_t *device, uint8_t address, uint16_t mask, uint32_t timeout_ms) {
    const g354_bus_t *bus = &device->bus;
    const uint32_t start = bus->millis(bus->context);
    do {
        uint16_t value;
        const g354_status_t status = read_word(device, address, &value);
        if (status != G354_OK) {
            return status;
        }
        if ((value & mask) == 0) {
            return G354_OK;
        }
        bus->delay_ms(bus->context, 1);
    } while ((uint32_t)(bus->millis(bus->context) - start) < timeout_ms);
    return G354_ERROR_TIMEOUT;
}

static g354_status_t check_word(g354_t *device, uint8_t address, uint16_t mask, uint16_t expected) {
    uint16_t value;
    const g354_status_t status = read_word(device, address, &value);
    if (status != G354_OK) {
        return status;
    }
    return (value & mask) == expected ? G354_OK : G354_ERROR_CONFIGURATION;
}

static g354_status_t set_mode(g354_t *device, bool sampling) {
    g354_status_t status = select_window(device, 0);
    if (status != G354_OK) {
        return status;
    }
    status = write_byte(device, REG_MODE_CTRL + 1, sampling ? 0x01 : 0x02);
    if (status != G354_OK) {
        return status;
    }
    const uint32_t start = device->bus.millis(device->bus.context);
    do {
        uint16_t mode;
        status = read_word(device, REG_MODE_CTRL, &mode);
        if (status != G354_OK) {
            return status;
        }
        if (((mode & 0x0300U) == 0) && (((mode & 0x0400U) == 0) == sampling)) {
            device->sampling = sampling;
            return G354_OK;
        }
        device->bus.delay_ms(device->bus.context, 1);
    } while ((uint32_t)(device->bus.millis(device->bus.context) - start) < 100U);
    return G354_ERROR_TIMEOUT;
}

/* Return immediately on a failed bus operation; never continue configuring an unknown state. */
#define CHECK(operation) do { const g354_status_t result = (operation); if (result != G354_OK) return result; } while (0)

g354_status_t g354_initialise(g354_t *device, const g354_bus_t *bus) {
    if ((device == NULL) || (bus == NULL) || (bus->select == NULL) || (bus->transfer == NULL) ||
        (bus->reset == NULL) || (bus->delay_us == NULL) || (bus->delay_ms == NULL) || (bus->millis == NULL)) {
        return G354_ERROR_ARGUMENT;
    }
    memset(device, 0, sizeof(*device));
    device->bus = *bus;
    bus->select(bus->context, false);
    bus->reset(bus->context, true);
    bus->delay_ms(bus->context, 100); /* /RST low pulse minimum: 100 ms. */
    bus->reset(bus->context, false);
    bus->delay_ms(bus->context, 800);

    CHECK(select_window(device, 1));
    CHECK(poll_clear(device, REG_GLOB_CMD, 0x0400, 1000));
    CHECK(set_mode(device, false)); /* Also handles a persisted UART AUTO_START setting. */
    CHECK(read_word(device, REG_DIAG_STAT, &device->diagnostic));
    if ((device->diagnostic & 0x7B7FU) != 0) {
        return G354_ERROR_DIAGNOSTIC;
    }

    CHECK(select_window(device, 1));
    const uint16_t expected_id[4] = {0x3347, 0x3435, 0x4450, 0x3048};
    for (size_t i = 0; i < 4; ++i) {
        uint16_t id;
        CHECK(read_word(device, (uint8_t)(REG_PROD_ID + 2 * i), &id));
        device->product_id[2 * i] = (char)(id & 0xFF);
        device->product_id[2 * i + 1] = (char)(id >> 8);
        if (id != expected_id[i]) {
            return G354_ERROR_PRODUCT_ID;
        }
    }
    CHECK(read_word(device, REG_VERSION, &device->firmware_version));

    /* Explicit configuration overrides saved settings without changing IMU non-volatile memory. */
    CHECK(write_byte(device, REG_UART_CTRL, 0)); /* SPI requires UART_AUTO=0; AUTO_START=0. */
    CHECK(write_byte(device, REG_UART_CTRL + 1, 0));
    CHECK(write_byte(device, REG_SIG_CTRL, 0));
    CHECK(write_byte(device, REG_SIG_CTRL + 1, 0xFE)); /* Temperature and six-axis new-data flags. */
    CHECK(write_byte(device, REG_MSC_CTRL, 0x06)); /* DRDY enabled/active high; no external trigger. */
    CHECK(write_byte(device, REG_SMPL_CTRL + 1, 0x02)); /* 500 samples/s. */
    CHECK(write_byte(device, REG_FILTER_CTRL, 0x02)); /* Moving average, 4 taps. */
    bus->delay_ms(bus->context, 1);
    CHECK(poll_clear(device, REG_FILTER_CTRL, 0x0020, 100));
    CHECK(write_byte(device, REG_BURST_CTRL1, 0x03)); /* COUNT + CHECKSUM, GPIO excluded. */
    CHECK(write_byte(device, REG_BURST_CTRL1 + 1, 0xF0)); /* FLAG, TEMP, GYRO, ACCL. */
    CHECK(write_byte(device, REG_BURST_CTRL2 + 1, 0x70)); /* All sensor values are 32-bit. */
    CHECK(write_byte(device, REG_POL_CTRL, 0)); /* Apply mounting transforms in software. */

    CHECK(check_word(device, REG_UART_CTRL, 0x0303, 0));
    CHECK(check_word(device, REG_SIG_CTRL, 0xFEFC, 0xFE00));
    CHECK(check_word(device, REG_MSC_CTRL, 0x00E6, 0x0006));
    CHECK(check_word(device, REG_SMPL_CTRL, 0xFF00, 0x0200));
    CHECK(check_word(device, REG_FILTER_CTRL, 0x003F, 0x0002));
    CHECK(check_word(device, REG_BURST_CTRL1, 0xFC07, 0xF003));
    CHECK(check_word(device, REG_BURST_CTRL2, 0x7C00, 0x7000));
    CHECK(check_word(device, REG_POL_CTRL, 0x007E, 0));
    CHECK(select_window(device, 0)); /* All sampling access remains in window 0. */
    device->configured = true;
    return G354_OK;
}

g354_status_t g354_start(g354_t *device) {
    if ((device == NULL) || !device->configured) {
        return G354_ERROR_ARGUMENT;
    }
    return set_mode(device, true);
}

g354_status_t g354_stop(g354_t *device) {
    if ((device == NULL) || !device->configured) {
        return G354_ERROR_ARGUMENT;
    }
    return set_mode(device, false);
}

static int32_t signed_sensor_word(uint16_t high, uint16_t low) {
    /* Avoid relying on unsigned-to-signed overflow conversion for negative sensor values. */
    const uint32_t bits = ((uint32_t)high << 16) | low;
    return (bits & 0x80000000U) ? -1 - (int32_t)(~bits) : (int32_t)bits;
}

g354_status_t g354_decode_burst(const uint16_t words[G354_BURST_WORDS], g354_sample_t *sample) {
    if ((words == NULL) || (sample == NULL)) {
        return G354_ERROR_ARGUMENT;
    }
    uint16_t sum = 0;
    for (size_t i = 0; i < G354_BURST_WORDS - 1; ++i) {
        sum = (uint16_t)(sum + words[i]);
    }
    if (sum != words[G354_BURST_WORDS - 1]) {
        return G354_ERROR_CHECKSUM;
    }
    if ((words[0] & 1U) != 0) {
        return G354_ERROR_DIAGNOSTIC;
    }
    if ((words[0] & 0xFE00U) != 0xFE00U) {
        return G354_ERROR_NOT_READY;
    }
    g354_sample_t decoded = {0};
    decoded.flags = words[0];
    decoded.counter = words[15];
    /* Compute the temperature offset before converting to float to retain low bits near 25 C. */
    const int32_t temp = signed_sensor_word(words[1], words[2]);
    decoded.temperature_c = 25.0f + (-0.0037918f / 65536.0f) * (float)((int64_t)temp - 172621824);
    for (size_t axis = 0; axis < 3; ++axis) {
        decoded.gyroscope_dps[axis] = (float)signed_sensor_word(words[3 + 2 * axis], words[4 + 2 * axis]) * (0.016f / 65536.0f);
        decoded.accelerometer_g[axis] = (float)signed_sensor_word(words[9 + 2 * axis], words[10 + 2 * axis]) * (0.0002f / 65536.0f);
    }
    *sample = decoded;
    return G354_OK;
}

g354_status_t g354_read_sample(g354_t *device, g354_sample_t *sample) {
    if ((device == NULL) || (sample == NULL) || !device->sampling) {
        return G354_ERROR_ARGUMENT;
    }
    const g354_bus_t *bus = &device->bus;
    uint16_t words[G354_BURST_WORDS];
    uint16_t unused;
    bus->select(bus->context, true);
    bus->delay_us(bus->context, 1);
    bool ok = bus->transfer(bus->context, 0x8000, &unused);
    bus->delay_us(bus->context, BURST_START_US);
    for (size_t i = 0; ok && (i < G354_BURST_WORDS); ++i) {
        ok = bus->transfer(bus->context, 0, &words[i]);
        /* At SCLK <= 1 MHz, 16 us + a 16-bit transfer also satisfies tREADRATE2 >= 32 us. */
        bus->delay_us(bus->context, BURST_STALL_US);
    }
    bus->select(bus->context, false);
    bus->delay_us(bus->context, NORMAL_STALL_US);
    if (!ok) {
        return G354_ERROR_IO;
    }
    const g354_status_t status = g354_decode_burst(words, sample);
    if (status == G354_ERROR_DIAGNOSTIC) {
        /* Preserve the destructive diagnostic read for the application/debugger. */
        CHECK(read_word(device, REG_DIAG_STAT, &device->diagnostic));
    }
    return status;
}
