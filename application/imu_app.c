#include "imu_app.h"

#include "attitude.h"
#include "bsp_g354.h"
#include "imu_config.h"
#include "odometry_app.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

_Static_assert(IMU_CALIBRATION_SAMPLES > 1, "Calibration requires a variance estimate");
_Static_assert(IMU_BODY_X_AXIS >= 0 && IMU_BODY_X_AXIS < 3 &&
               IMU_BODY_Y_AXIS >= 0 && IMU_BODY_Y_AXIS < 3 &&
               IMU_BODY_Z_AXIS >= 0 && IMU_BODY_Z_AXIS < 3, "Axis indices must be 0, 1, or 2");
_Static_assert(IMU_BODY_X_AXIS != IMU_BODY_Y_AXIS && IMU_BODY_X_AXIS != IMU_BODY_Z_AXIS &&
               IMU_BODY_Y_AXIS != IMU_BODY_Z_AXIS, "Mounting axes must be a permutation");
_Static_assert((IMU_BODY_X_SIGN == 1 || IMU_BODY_X_SIGN == -1) &&
               (IMU_BODY_Y_SIGN == 1 || IMU_BODY_Y_SIGN == -1) &&
               (IMU_BODY_Z_SIGN == 1 || IMU_BODY_Z_SIGN == -1), "Axis signs must be +/-1");
#define IMU_PERMUTATION_SIGN (((IMU_BODY_X_AXIS == 0 && IMU_BODY_Y_AXIS == 1) || \
                               (IMU_BODY_X_AXIS == 1 && IMU_BODY_Y_AXIS == 2) || \
                               (IMU_BODY_X_AXIS == 2 && IMU_BODY_Y_AXIS == 0)) ? 1 : -1)
_Static_assert(IMU_PERMUTATION_SIGN * IMU_BODY_X_SIGN * IMU_BODY_Y_SIGN * IMU_BODY_Z_SIGN == 1,
               "Mounting transform must preserve the right-handed coordinate frame");

typedef struct {
    uint32_t count;
    float gyro_mean[3];
    float gyro_m2[3];
    float accel_mean[3];
    float accel_m2[3];
} calibration_t;

volatile imu_output_t g_imu;

static void publish(const imu_output_t *output) {
    odometry_app_submit_imu(output->drdy_timestamp_us, output->yaw_deg, output->valid);
    taskENTER_CRITICAL();
    g_imu = *output;
    taskEXIT_CRITICAL();
}

bool imu_app_get_output(imu_output_t *output) {
    if (output == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    *output = g_imu;
    taskEXIT_CRITICAL();
    /* Callers must not act on a snapshot that outlived several 2 ms sample periods. */
    output->valid = output->valid && ((uint32_t)(bsp_g354_micros() - output->timestamp_us) < 10000U);
    return output->valid;
}

static void remap_axes(const float sensor[3], float body[3]) {
    body[0] = IMU_BODY_X_SIGN * sensor[IMU_BODY_X_AXIS];
    body[1] = IMU_BODY_Y_SIGN * sensor[IMU_BODY_Y_AXIS];
    body[2] = IMU_BODY_Z_SIGN * sensor[IMU_BODY_Z_AXIS];
}

static void reset_calibration(calibration_t *calibration, imu_output_t *output) {
    memset(calibration, 0, sizeof(*calibration));
    output->calibration_samples = 0;
    ++output->calibration_restarts;
}

static bool accumulate_calibration(calibration_t *calibration, imu_output_t *output) {
    const float *gyro = output->gyroscope_dps;
    const float *accel = output->accelerometer_g;
    const float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (fabsf(norm - 1.0f) > IMU_CALIBRATION_ACCEL_TOLERANCE_G) {
        reset_calibration(calibration, output);
        return false;
    }
    for (size_t i = 0; i < 3; ++i) {
        if (fabsf(gyro[i]) > IMU_CALIBRATION_GYRO_LIMIT_DPS ||
            ((calibration->count > 0) && (fabsf(accel[i] - calibration->accel_mean[i]) > 0.03f))) {
            reset_calibration(calibration, output);
            return false;
        }
    }
    ++calibration->count;
    for (size_t i = 0; i < 3; ++i) {
        const float gyro_delta = gyro[i] - calibration->gyro_mean[i];
        calibration->gyro_mean[i] += gyro_delta / (float)calibration->count;
        calibration->gyro_m2[i] += gyro_delta * (gyro[i] - calibration->gyro_mean[i]);
        const float accel_delta = accel[i] - calibration->accel_mean[i];
        calibration->accel_mean[i] += accel_delta / (float)calibration->count;
        calibration->accel_m2[i] += accel_delta * (accel[i] - calibration->accel_mean[i]);
    }
    output->calibration_samples = calibration->count;
    if (calibration->count < IMU_CALIBRATION_SAMPLES) {
        return false;
    }
    for (size_t i = 0; i < 3; ++i) {
        const float divisor = (float)(calibration->count - 1U);
        if ((calibration->gyro_m2[i] / divisor > IMU_CALIBRATION_GYRO_STD_DPS * IMU_CALIBRATION_GYRO_STD_DPS) ||
            (calibration->accel_m2[i] / divisor > IMU_CALIBRATION_ACCEL_STD_G * IMU_CALIBRATION_ACCEL_STD_G)) {
            reset_calibration(calibration, output);
            return false;
        }
    }
    return true;
}

static void fail(imu_output_t *output, g354_t *device, imu_error_t error) {
    bsp_g354_disarm_drdy();
    output->state = IMU_STATE_ERROR;
    output->error = error;
    output->valid = false;
    output->diagnostic = device->diagnostic;
    output->spi_error = bsp_g354_spi_error();
    publish(output);
    if (device->sampling) {
        (void)g354_stop(device); /* Best effort, one attempt; keep the original failure visible. */
    }
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void imu_app_task(void) {
    /* Static storage keeps the 1024-word task stack available for HAL/math calls. */
    static imu_output_t output;
    static g354_t device;
    static attitude_t attitude;
    static calibration_t calibration;
    g354_bus_t bus;
    output.state = IMU_STATE_INITIALISING;
    output.sample_rate_hz = G354_SAMPLE_RATE_HZ;
    output.quaternion[0] = 1.0f;
    publish(&output);
    if (!bsp_g354_initialise(&bus)) {
        fail(&output, &device, IMU_ERROR_BSP);
    }
    output.sensor_status = g354_initialise(&device, &bus);
    memcpy(output.product_id, device.product_id, sizeof(output.product_id));
    output.firmware_version = device.firmware_version;
    output.diagnostic = device.diagnostic;
    if (output.sensor_status != G354_OK) {
        fail(&output, &device, IMU_ERROR_SENSOR);
    }
    bsp_g354_arm_drdy();
    output.sensor_status = g354_start(&device);
    if (output.sensor_status != G354_OK) {
        fail(&output, &device, IMU_ERROR_SENSOR);
    }
    output.state = IMU_STATE_CALIBRATING;
    publish(&output);

    bool have_counter = false;
    uint16_t previous_counter = 0;
    uint32_t previous_sample_us = 0;
    uint32_t settling = IMU_SETTLING_SAMPLES;
    uint32_t consecutive_errors = 0;
    uint32_t consecutive_timeouts = 0;
    for (;;) {
        if (!bsp_g354_wait_drdy(IMU_DRDY_TIMEOUT_MS)) {
            ++output.drdy_timeouts;
            output.valid = false;
            if (output.state == IMU_STATE_RUNNING) {
                output.tracking_continuous = false;
            } else {
                reset_calibration(&calibration, &output);
            }
            publish(&output);
            if (++consecutive_timeouts >= 3U) {
                fail(&output, &device, IMU_ERROR_DRDY_TIMEOUT);
            }
            continue;
        }
        consecutive_timeouts = 0;
        taskENTER_CRITICAL();
        const uint32_t edges_before = bsp_g354_drdy_count();
        const uint32_t edge_time = bsp_g354_last_drdy_us();
        taskEXIT_CRITICAL();
        const uint32_t read_start = bsp_g354_micros();
        g354_sample_t sample;
        output.sensor_status = g354_read_sample(&device, &sample);
        const uint32_t read_end = bsp_g354_micros();
        const uint32_t read_time = read_end - read_start;
        if (read_time > output.max_read_time_us) {
            output.max_read_time_us = read_time;
        }
        output.drdy_interrupts = bsp_g354_drdy_count();
        output.drdy_timestamp_us = edge_time;
        if (output.sensor_status != G354_OK) {
            output.valid = false;
            switch (output.sensor_status) {
                case G354_ERROR_CHECKSUM: ++output.checksum_errors; break;
                case G354_ERROR_NOT_READY: ++output.not_ready_errors; break;
                default: ++output.io_errors; break;
            }
            if (output.state == IMU_STATE_CALIBRATING) {
                reset_calibration(&calibration, &output);
            }
            output.spi_error = bsp_g354_spi_error();
            publish(&output);
            if ((output.sensor_status == G354_ERROR_DIAGNOSTIC) ||
                (++consecutive_errors >= IMU_MAX_CONSECUTIVE_ERRORS)) {
                fail(&output, &device, IMU_ERROR_SENSOR);
            }
            continue;
        }
        /* A new edge during the burst or an overly long transfer risks mixing sampling epochs. */
        if ((edges_before != bsp_g354_drdy_count()) || (read_time >= 2000U)) {
            ++output.timing_errors;
            output.valid = false;
            if (output.state == IMU_STATE_RUNNING) {
                output.tracking_continuous = false;
            } else {
                reset_calibration(&calibration, &output);
            }
            publish(&output);
            if (++consecutive_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
                output.sensor_status = G354_ERROR_TIMEOUT;
                fail(&output, &device, IMU_ERROR_SENSOR);
            }
            continue;
        }
        consecutive_errors = 0;
        bool contiguous = true;
        if (have_counter) {
            const uint16_t delta = (uint16_t)(sample.counter - previous_counter);
            if (delta == 0) {
                ++output.duplicate_samples;
                output.valid = false;
                publish(&output);
                continue;
            }
            if ((delta % G354_COUNTER_STEP != 0) || ((uint32_t)(read_end - previous_sample_us) >= 32768000U)) {
                ++output.timing_errors;
                contiguous = false;
            } else if (delta != G354_COUNTER_STEP) {
                output.dropped_samples += delta / G354_COUNTER_STEP - 1U;
                contiguous = false;
            }
        }
        previous_counter = sample.counter;
        previous_sample_us = read_end;
        have_counter = true;
        ++output.samples;
        output.timestamp_us = read_end;
        output.sample_counter = sample.counter;
        output.sample_flags = sample.flags;
        output.temperature_c = sample.temperature_c;
        remap_axes(sample.gyroscope_dps, output.gyroscope_dps);
        remap_axes(sample.accelerometer_g, output.accelerometer_g);
        if (!contiguous) {
            output.valid = false;
            if (output.state == IMU_STATE_RUNNING) {
                /* No FIFO: missing motion cannot be reconstructed from the newest sample. */
                output.tracking_continuous = false;
            } else {
                reset_calibration(&calibration, &output);
            }
            publish(&output);
            continue; /* Never multiply a fresh gyro value by the whole missing interval. */
        }
        if (output.state == IMU_STATE_CALIBRATING) {
            if (settling > 0) {
                --settling;
                publish(&output);
                continue;
            }
            if (!accumulate_calibration(&calibration, &output)) {
                publish(&output);
                continue;
            }
            if (!attitude_initialise(&attitude, (float)G354_SAMPLE_RATE_HZ,
                                      calibration.gyro_mean, calibration.accel_mean)) {
                fail(&output, &device, IMU_ERROR_ATTITUDE);
            }
            output.state = IMU_STATE_RUNNING;
            output.tracking_continuous = true;
        }
        attitude_output_t pose;
        const uint32_t fusion_start = bsp_g354_micros();
        if (!attitude_update(&attitude, output.gyroscope_dps, output.accelerometer_g, &pose)) {
            fail(&output, &device, IMU_ERROR_ATTITUDE);
        }
        const uint32_t fusion_time = bsp_g354_micros() - fusion_start;
        if (fusion_time > output.max_fusion_time_us) {
            output.max_fusion_time_us = fusion_time;
        }
        ++output.attitude_updates;
        memcpy(output.quaternion, pose.quaternion, sizeof(output.quaternion));
        memcpy(output.corrected_gyroscope_dps, pose.gyroscope_dps, sizeof(output.corrected_gyroscope_dps));
        memcpy(output.gyroscope_bias_dps, pose.bias_dps, sizeof(output.gyroscope_bias_dps));
        output.roll_deg = pose.roll_deg;
        output.pitch_deg = pose.pitch_deg;
        output.yaw_deg = pose.yaw_deg;
        output.accelerometer_ignored = pose.accelerometer_ignored;
        output.gyroscope_overrange = pose.gyroscope_overrange;
        if (pose.gyroscope_overrange) {
            output.tracking_continuous = false;
        }
        output.recovering = pose.recovering;
        output.valid = output.tracking_continuous && !pose.recovering;
        publish(&output);
    }
}
