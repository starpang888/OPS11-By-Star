#ifndef ODOMETRY_APP_H
#define ODOMETRY_APP_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ODOMETRY_STATE_WAITING = 0,
    ODOMETRY_STATE_RUNNING,
    ODOMETRY_STATE_TRACKING_LOST,
    ODOMETRY_STATE_ERROR
} odometry_state_t;
typedef enum {
    ODOMETRY_ERROR_NONE = 0,
    ODOMETRY_ERROR_TASK,
    ODOMETRY_ERROR_CONFIG,
    ODOMETRY_ERROR_TIMEBASE,
    ODOMETRY_ERROR_INPUT,
    ODOMETRY_ERROR_SYNC,
    ODOMETRY_ERROR_GEOMETRY
} odometry_error_t;
typedef enum {
    ODOMETRY_WAIT_NONE = 0,
    ODOMETRY_WAIT_IMU,
    ODOMETRY_WAIT_ENCODER_1,
    ODOMETRY_WAIT_ENCODER_2,
    ODOMETRY_WAIT_SYNC
} odometry_wait_t;
typedef enum {
    ODOMETRY_SYNC_NONE = 0,
    ODOMETRY_SYNC_NO_IMU,
    ODOMETRY_SYNC_NO_BRACKET,
    ODOMETRY_SYNC_BRACKET_TOO_WIDE,
    ODOMETRY_SYNC_GENERATION,
    ODOMETRY_SYNC_COUNT_DELTA,
    ODOMETRY_SYNC_TOO_OLD,
    ODOMETRY_SYNC_IMU_INTERVAL
} odometry_sync_failure_t;
typedef struct {
    odometry_state_t state;
    odometry_error_t error;
    bool valid;
    bool tracking_continuous; /* Latched false after lost motion, until reset. */
    double x_mm, y_mm;
    float yaw_deg; /* Relative to initial heading, [-180,180]. */
    uint32_t timestamp_us; /* Common sample endpoint, TIM2; not publication time. */
    uint32_t latency_us;
    uint32_t encoder_bracket_us[2];
    uint32_t updates;
    uint32_t resets;
    uint32_t sync_errors;
    uint32_t input_errors;
    uint32_t imu_queue_overruns;
    float measured_update_rate_hz;
    odometry_wait_t wait_reason;
    uint32_t task_cycles; /* Changes even while waiting for the first origin. */
    uint8_t lost_input_mask; /* First loss: bit0=SPI1, bit1=SPI2, bit2=IMU. */
    odometry_sync_failure_t sync_failure;
    uint8_t sync_input_mask; /* SPI1=1, SPI2=2, IMU=4; synchronization failure. */
    uint32_t failure_age_us;
    uint32_t failure_interval_us;
    uint32_t failure_bracket_us[2]; /* Zero when no bracket exists. */
} odometry_output_t;
extern volatile odometry_output_t g_odometry;
/* Start once before osKernelStart, after peripheral initialisation. */
bool odometry_app_start(void);
/* Task context only. Coherent snapshot with freshness check. */
bool odometry_app_get_output(odometry_output_t *output);
/* Queue a new pose origin. Does not reset sensors or repair IMU lost tracking.
   Stationary reset is recommended. Wait for valid before using coordinates. */
bool odometry_app_reset(void);

/* Application producer hooks. Task context only; fixed storage, no waiting.
   The encoder hook must run before diagnostic transactions overwrite CS time. */
void odometry_app_submit_encoder(unsigned id, uint32_t timestamp_us,
                                 int64_t counts, bool valid, bool origin_reset);
void odometry_app_submit_imu(uint32_t timestamp_us, float yaw_deg, bool valid);
#endif
