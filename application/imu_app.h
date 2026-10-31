/** @file imu_app.h
 * @brief application 层 IMU 接口：任务调度、开机标定、姿态发布及温漂标定命令。
 *        调试器可看 g_imu，其他任务通过 imu_app_get_output 获取一致快照。
 */
#ifndef IMU_APP_H
#define IMU_APP_H

#include "g354.h"
#include "gyro_temperature.h"

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
    bool valid; /* 标定、恢复或姿态连续性丢失时无效；读取接口另检查数据是否过期。 */
    bool tracking_continuous; /* 采样中断后锁存为 false，需重启重新建立姿态参考。 */
    bool accelerometer_ignored;
    bool gyroscope_overrange;
    bool recovering;
    char product_id[9];
    uint16_t firmware_version;
    uint16_t diagnostic;
    uint16_t sample_counter;
    uint16_t sample_flags;
    uint32_t sample_rate_hz;
    uint32_t timestamp_us; /* 读取完成时的 TIM2 微秒时间，约 71.6 分钟回绕。 */
    uint32_t drdy_timestamp_us; /* 本次有效样本对应的 DRDY 上升沿时间，用于里程计同步。 */
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
    float temperature_c; /* 传感器温度，摄氏度。 */
    float gyroscope_dps[3]; /* 完成安装轴映射的原始角速度，度/秒。 */
    float corrected_gyroscope_dps[3]; /* 补偿和死区后的角速度，实际参与积分，度/秒。 */
    float accelerometer_g[3]; /* 完成安装轴映射后的加速度，g。 */
    float gyroscope_bias_dps[3]; /* 启动、温漂和剩余零偏的总和，度/秒。 */
    float quaternion[4]; /* [w,x,y,z]，机体系到启动时定义的导航系。 */
    float roll_deg; /* Z-X-Y 中间 X 角，[-90,90] 度，相对开机姿态。 */
    float pitch_deg; /* Z-X-Y 的 Y 角，[-180,180] 度，相对开机姿态。 */
    float yaw_deg; /* 相对航向，[-180,180] 度，无绝对北向观测。 */
    uint32_t algorithm; /* 1=论文自适应 KF+Heun RK2；当前不运行 Fusion。 */
    bool stationary;
    uint8_t stationary_block_mask; /* 静止阻塞位可叠加：角速度=1、加速度=2、轮1=4、轮2=8。 */
    uint8_t deadband_mask; /* bit0/1/2 分别表示 X/Y/Z 角速度被死区归零。 */
    float stationary_time_s;
    uint32_t bias_updates;
    float filtered_gyroscope_dps[3];
    float compensated_gyroscope_dps[3]; /* 已补偿、尚未应用死区；当前配置死区为 0.3 度/秒。 */
    float startup_gyroscope_bias_dps[3];
    float temperature_bias_dps[3];
    float residual_bias_dps[3];
    float kalman_noise_variance[3];
    float kalman_gain[3];
    float startup_temperature_c;
    bool temperature_compensated;
    bool temperature_out_of_range;
    gyro_temperature_output_t temperature_calibration;
    uint32_t last_command;
    bool command_succeeded;
} imu_output_t;

typedef enum {
    IMU_COMMAND_NONE = 0,
    IMU_COMMAND_TEMPERATURE_START = 1,
    IMU_COMMAND_TEMPERATURE_FIT = 2,
    IMU_COMMAND_TEMPERATURE_APPLY = 3,
    IMU_COMMAND_TEMPERATURE_STOP = 4
} imu_command_t;

/* 一次性命令邮箱，由 IMU 任务消费；命令不写 Flash。
   可在调试器中设置，或由其他任务通过下方接口提交。 */
extern volatile uint32_t g_imu_command;
bool imu_app_request_command(imu_command_t command);

/* 全局结构体供调试器观察；固件任务读取时使用快照接口，避免跨一次发布读到混合字段。 */
extern volatile imu_output_t g_imu;
bool imu_app_get_output(imu_output_t *output);
void imu_app_task(void);

#endif
