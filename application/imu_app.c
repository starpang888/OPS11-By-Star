/** @file imu_app.c
 * @brief application 层 IMU 任务：启动标定、DRDY 读取、采样连续性检查、姿态计算及结果发布。
 *        运行流程：原始数据轴映射→健康检查→零偏标定/KF-RK2→提交里程计公共端点。
 */
#include "imu_app.h"

#include "attitude.h"
#include "bsp_g354.h"
#include "imu_config.h"
#include "odometry_app.h"
#include "encoder_app.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

_Static_assert(IMU_CALIBRATION_SAMPLES > 1, "Calibration requires a variance estimate");
_Static_assert(IMU_TEMPERATURE_POINTS_MAX <= GYRO_TEMPERATURE_MAX_POINTS,
               "Temperature point storage exceeded");
_Static_assert(IMU_TEMPERATURE_POINT_SAMPLES > 0, "Temperature mean needs samples");
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

/* Welford 在线均值/方差状态；不保存全部 1000 点，减少 RAM 占用。 */
typedef struct {
    uint32_t count;
    float gyro_mean[3];
    float gyro_m2[3];
    float accel_mean[3];
    float accel_m2[3];
    float temperature_mean;
} calibration_t;

volatile imu_output_t g_imu;
volatile uint32_t g_imu_command;
static attitude_t attitude;
static gyro_temperature_t temperature_calibration;

bool imu_app_request_command(imu_command_t command) {
    if (command <= IMU_COMMAND_NONE || command > IMU_COMMAND_TEMPERATURE_STOP) return false;
    taskENTER_CRITICAL();
    const bool accepted = g_imu.state == IMU_STATE_RUNNING && g_imu_command == IMU_COMMAND_NONE;
    if (accepted) g_imu_command = (uint32_t)command;
    taskEXIT_CRITICAL();
    return accepted;
}

static void process_command(imu_output_t *output) {
    /* 命令读取后立即清空邮箱，每条温漂命令只执行一次，结果留在输出结构体。 */
    taskENTER_CRITICAL();
    const uint32_t command = g_imu_command;
    g_imu_command = IMU_COMMAND_NONE;
    taskEXIT_CRITICAL();
    if (command == IMU_COMMAND_NONE) return;
    output->last_command = command;
    output->command_succeeded = false;
    switch (command) {
        case IMU_COMMAND_TEMPERATURE_START:
            if (output->valid) {
                gyro_temperature_start(&temperature_calibration);
                output->command_succeeded = true;
            }
            break;
        case IMU_COMMAND_TEMPERATURE_FIT:
            output->command_succeeded = gyro_temperature_fit(&temperature_calibration,
                IMU_TEMPERATURE_FIT_MIN_POINTS, IMU_TEMPERATURE_FIT_MIN_SPAN_C);
            break;
        case IMU_COMMAND_TEMPERATURE_APPLY:
            if (temperature_calibration.output.fit_valid && output->valid) {
                output->command_succeeded = attitude_set_temperature_model(&attitude,
                    temperature_calibration.output.slope, temperature_calibration.output.min_c,
                    temperature_calibration.output.max_c, output->temperature_c);
                temperature_calibration.output.applied = output->command_succeeded;
            }
            break;
        case IMU_COMMAND_TEMPERATURE_STOP:
            temperature_calibration.output.collecting = false;
            temperature_calibration.output.pending_samples = 0;
            output->command_succeeded = true;
            break;
        default: break;
    }
}

static void publish(imu_output_t *output) {
    /* 输入无效时不能沿用上一帧的滤波/静止历史，也不能把旧航向作为新健康样本提交。 */
    if (output->state == IMU_STATE_RUNNING && !output->valid) {
        attitude_discard_history(&attitude);
        temperature_calibration.output.pending_samples = 0;
        output->stationary = false;
        output->stationary_time_s = 0.0f;
    }
    output->temperature_calibration = temperature_calibration.output;
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
    /* 样本周期为 2 ms；超过 10 ms 没有新数据时，即使旧 valid 为真也要拒绝。 */
    output->valid = output->valid && ((uint32_t)(bsp_g354_micros() - output->timestamp_us) < 10000U);
    return output->valid;
}

static void remap_axes(const float sensor[3], float body[3]) {
    /* 加速度和角速度使用同一个右手系安装变换，保证静止判断和姿态方向一致。 */
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
    /* 只有持续静止且方差合格的整段数据才能用于启动零偏；中途移动会重新积累。 */
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
    calibration->temperature_mean += (output->temperature_c - calibration->temperature_mean) /
                                      (float)calibration->count;
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
    /* 严重错误保留首因，注销 DRDY 接收者并停止解算，不循环复位掩盖现场。 */
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
    /* 大结构体使用静态存储，为 1024 word 任务栈保留 HAL 和数学调用空间。 */
    static imu_output_t output;
    static g354_t device;
    static calibration_t calibration;
    g354_bus_t bus;
    output.state = IMU_STATE_INITIALISING;
    output.sample_rate_hz = G354_SAMPLE_RATE_HZ;
    output.algorithm = 1U;
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
            const attitude_config_t config = {
                .sample_rate_hz = (float)G354_SAMPLE_RATE_HZ,
                .gyro_limit_dps = IMU_GYRO_LIMIT_DPS,
                .stationary_threshold_dps = IMU_STATIONARY_THRESHOLD_DPS,
                .stationary_period_s = IMU_STATIONARY_PERIOD_S,
                .accel_tolerance_g = IMU_STATIONARY_ACCEL_TOLERANCE_G,
                .bias_slew_dps_per_s = IMU_BIAS_SLEW_DPS_PER_S,
                .deadband_dps = IMU_GYRO_DEADBAND_DPS,
                .temperature_model_valid = IMU_TEMPERATURE_MODEL_VALID != 0,
                .temperature_slope = {IMU_TEMPERATURE_SLOPE_X, IMU_TEMPERATURE_SLOPE_Y,
                                      IMU_TEMPERATURE_SLOPE_Z},
                .temperature_min_c = IMU_TEMPERATURE_MIN_C,
                .temperature_max_c = IMU_TEMPERATURE_MAX_C
            };
            for (unsigned i = 0; i < 3; ++i) {
                output.kalman_noise_variance[i] = calibration.gyro_m2[i] /
                    (float)(calibration.count - 1U);
                output.startup_gyroscope_bias_dps[i] = calibration.gyro_mean[i];
            }
            output.startup_temperature_c = calibration.temperature_mean;
            if (!attitude_initialise(&attitude, &config, calibration.gyro_mean,
                                      output.kalman_noise_variance, calibration.temperature_mean)) {
                fail(&output, &device, IMU_ERROR_ATTITUDE);
            }
            output.state = IMU_STATE_RUNNING;
            output.tracking_continuous = true;
        }
        process_command(&output);
        attitude_output_t pose;
        const uint32_t attitude_start = bsp_g354_micros();
        const uint8_t wheel_mask = encoder_app_stationary_block_mask(attitude_start);
        if (!attitude_update(&attitude, output.gyroscope_dps, output.accelerometer_g,
                              output.temperature_c, wheel_mask, &pose)) {
            fail(&output, &device, IMU_ERROR_ATTITUDE);
        }
        gyro_temperature_collect(&temperature_calibration,
            pose.stationary && output.tracking_continuous, output.temperature_c, pose.filtered_dps,
            IMU_TEMPERATURE_BIN_C, IMU_TEMPERATURE_POINT_SAMPLES, IMU_TEMPERATURE_POINTS_MAX);
        const uint32_t attitude_time = bsp_g354_micros() - attitude_start;
        if (attitude_time > output.max_fusion_time_us) {
            output.max_fusion_time_us = attitude_time;
        }
        ++output.attitude_updates;
        memcpy(output.quaternion, pose.quaternion, sizeof(output.quaternion));
        memcpy(output.corrected_gyroscope_dps, pose.gyroscope_dps, sizeof(output.corrected_gyroscope_dps));
        memcpy(output.gyroscope_bias_dps, pose.bias_dps, sizeof(output.gyroscope_bias_dps));
        memcpy(output.filtered_gyroscope_dps, pose.filtered_dps, sizeof(pose.filtered_dps));
        memcpy(output.compensated_gyroscope_dps, pose.compensated_dps, sizeof(pose.compensated_dps));
        memcpy(output.temperature_bias_dps, pose.temperature_bias_dps, sizeof(pose.temperature_bias_dps));
        memcpy(output.residual_bias_dps, pose.residual_bias_dps, sizeof(pose.residual_bias_dps));
        memcpy(output.kalman_gain, pose.kalman_gain, sizeof(pose.kalman_gain));
        output.stationary = pose.stationary;
        output.stationary_time_s = pose.stationary_elapsed_s;
        output.stationary_block_mask = pose.stationary_block_mask;
        output.deadband_mask = pose.deadband_mask;
        output.bias_updates = pose.bias_updates;
        output.temperature_compensated = pose.temperature_compensated;
        output.temperature_out_of_range = pose.temperature_out_of_range;
        output.temperature_calibration = temperature_calibration.output;
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
