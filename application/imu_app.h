#ifndef IMU_APP_H
#define IMU_APP_H

#include "g354.h"

typedef enum {
    IMU_STATE_INITIALISING = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_RUNNING,
    IMU_STATE_ERROR
} imu_state_t;

typedef enum {
    IMU_ERROR_NONE = 0,
    IMU_ERROR_BSP,
    IMU_ERROR_SENSOR,
    IMU_ERROR_DRDY_TIMEOUT,
    IMU_ERROR_ATTITUDE
} imu_error_t;

typedef struct {
    imu_state_t state;
    imu_error_t error;
    g354_status_t sensor_status;
    bool valid; /* False during calibration, stale data, recovery, or lost orientation continuity. */
    bool tracking_continuous; /* Sticky false after a data gap; restart to establish a new reference. */
    bool accelerometer_ignored;
    bool gyroscope_overrange;
    bool recovering;
    char product_id[9];
    uint16_t firmware_version;
    uint16_t diagnostic;
    uint16_t sample_counter;
    uint16_t sample_flags;
    uint32_t sample_rate_hz;
    uint32_t timestamp_us; /* Read completion, TIM2, wraps after about 71.6 minutes. */
    uint32_t drdy_timestamp_us;
    uint32_t drdy_interrupts;
    uint32_t samples;
    uint32_t attitude_updates;
    uint32_t calibration_samples;
    uint32_t calibration_restarts;
    uint32_t dropped_samples;
    uint32_t duplicate_samples;
    uint32_t checksum_errors;
    uint32_t io_errors;
    uint32_t not_ready_errors;
    uint32_t timing_errors;
    uint32_t drdy_timeouts;
    uint32_t spi_error;
    uint32_t max_read_time_us;
    uint32_t max_fusion_time_us;
    float temperature_c;
    float gyroscope_dps[3];
    float corrected_gyroscope_dps[3];
    float accelerometer_g[3];
    float gyroscope_bias_dps[3];
    float quaternion[4]; /* [w,x,y,z], body to world, +Z up. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg; /* Relative heading in [-180,180]; no absolute heading observation. */
} imu_output_t;

/* Convenient debugger watch target. Firmware consumers should use the coherent copy API. */
extern volatile imu_output_t g_imu;
bool imu_app_get_output(imu_output_t *output);
void imu_app_task(void);

#endif
