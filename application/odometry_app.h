/** @file odometry_app.h
 * @brief application 层里程计接口：缓存三路历史、对齐公共端点并发布底盘中心 X/Y/航向。
 */
#ifndef ODOMETRY_APP_H
#define ODOMETRY_APP_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ODOMETRY_STATE_WAITING = 0, /* 启动等待或有界同步恢复等待，结合 valid/error 区分。 */
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
    bool tracking_continuous; /* 真正丢失运动信息后锁存为 false，显式重建原点后才能重新开始。 */
    double x_mm, y_mm; /* 底盘中心相对初始原点的世界坐标，毫米。 */
    float yaw_deg; /* 相对初始朝向的航向，[-180,180] 度。 */
    uint32_t timestamp_us; /* 公共采样端点的 TIM2 微秒时间，不是发布或发送时间。 */
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
    uint8_t lost_input_mask; /* 首次输入失效来源：bit0=SPI1、bit1=SPI2、bit2=IMU，可叠加。 */
    odometry_sync_failure_t sync_failure;
    uint8_t sync_input_mask; /* SPI1=1, SPI2=2, IMU=4; synchronization failure. */
    uint32_t failure_age_us;
    uint32_t failure_interval_us;
    uint32_t failure_bracket_us[2]; /* Zero when no bracket exists. */
    uint32_t skipped_sync_samples; /* 被拒绝的同步端点数，不表示丢弃了编码器累计位移。 */
    uint32_t sync_recoveries; /* 在后续连续且合格的公共端点上成功续接的次数。 */
} odometry_output_t;
extern volatile odometry_output_t g_odometry;
/* 外设初始化后、osKernelStart 前启动一次，重复调用不会重复创建任务。 */
bool odometry_app_start(void);
/* 任务上下文获取一致快照，统一检查运行状态、轨迹连续性和数据新鲜度。 */
bool odometry_app_get_output(odometry_output_t *output);
/* 请求新的位姿原点，不重置传感器，也不能修复 IMU 已丢失的姿态连续性。
   建议静止调用，等待 valid 恢复后再使用新坐标。 */
bool odometry_app_reset(void);

/* 三路采样任务提交健康结果及时间戳；固定容量存储，不阻塞等待对齐。
   编码器提交必须在诊断访问覆盖片选时间前完成；这些接口不允许 ISR 调用。 */
void odometry_app_submit_encoder(unsigned id, uint32_t timestamp_us,
                                 int64_t counts, bool valid, bool origin_reset);
void odometry_app_submit_imu(uint32_t timestamp_us, float yaw_deg, bool valid);
#endif
