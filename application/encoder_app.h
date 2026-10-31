/** @file encoder_app.h
 * @brief application 层双编码器接口：两路独立采样任务、角度/多圈位移/转速结果及诊断状态。
 */
#ifndef ENCODER_APP_H
#define ENCODER_APP_H

#include "mt6826s.h"
#include "encoder_motion.h"
#include "bsp_mt6826s.h"

typedef enum {
    ENCODER_1 = 0, /* SPI1 / PA4, g_encoder. */
    ENCODER_2,     /* SPI2 / PB12, g_encoder2. */
    ENCODER_COUNT
} encoder_id_t;

typedef enum {
    ENCODER_STATE_INITIALISING = 0,
    ENCODER_STATE_RUNNING,
    ENCODER_STATE_ERROR
} encoder_state_t;

typedef enum {
    ENCODER_ERROR_NONE = 0,
    ENCODER_ERROR_TASK,
    ENCODER_ERROR_BSP,
    ENCODER_ERROR_SENSOR,
    ENCODER_ERROR_MOTION
} encoder_error_t;

typedef struct {
    mt6826s_status_t single_status;
    mt6826s_status_t burst_status;
    uint8_t single_frame[MT6826S_FRAME_BYTES];
    uint8_t burst_frame[MT6826S_FRAME_BYTES];
    bool single_crc_valid; /* Includes the fixed-bit format check. */
    bool burst_crc_valid;
    bool nonzero_pair; /* Reject an all-zero response as evidence of communication. */
    bool passed; /* Both reads checked, nonzero data, circular angle difference <= tolerance. */
    float single_angle_deg;
    float burst_angle_deg;
    float angle_difference_deg; /* Signed circular difference; use only if both reads checked. */
    uint32_t checks;
    uint32_t passes;
    uint32_t timestamp_ms;
} encoder_register_check_t;

typedef struct {
    encoder_state_t state;
    encoder_error_t error;
    mt6826s_status_t sensor_status;
    bool valid;
    bool overspeed;
    bool weak_magnet;
    bool undervoltage;
    uint8_t alarms; /* Trust alarm fields only when crc_valid is true. */
    bool crc_valid;
    uint8_t raw_frame[MT6826S_FRAME_BYTES];
    uint8_t crc_received;
    uint8_t crc_calculated;
    uint16_t angle_raw; /* Last healthy sample, 0..32767. */
    float angle_deg; /* Last healthy sample, [0,360). No software zero offset. */
    uint32_t sample_rate_hz; /* 配置的标称采样频率，Hz。 */
    float measured_read_rate_hz; /* 每秒实际周期尝试次数，约每秒更新，不含额外诊断访问。 */
    uint32_t last_attempt_ms;
    uint32_t timestamp_ms; /* 最近健康角度的锁存请求时刻，HAL 毫秒时基。 */
    uint32_t timestamp_us; /* 同一锁存请求的公共 TIM2 微秒时间，按 uint32_t 回绕。 */
    uint32_t sample_interval_us; /* 相邻健康样本的真实采样间隔，微秒。 */
    uint32_t max_sample_interval_us;
    uint32_t attempts;
    uint32_t samples;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t io_errors;
    uint32_t alarm_samples;
    uint32_t consecutive_errors;
    uint32_t spi_error;
    uint32_t overspeed_samples;
    uint32_t weak_magnet_samples;
    uint32_t undervoltage_samples;
    uint8_t last_alarm_bits; /* Latched from the most recent CRC/format-checked alarm frame. */
    uint8_t last_alarm_frame[MT6826S_FRAME_BYTES];
    float last_alarm_angle_deg; /* Diagnostic only: this frame was rejected for control. */
    uint32_t last_alarm_ms;
    mt6826s_diagnostics_t diagnostics;
    mt6826s_status_t diagnostics_status;
    bool diagnostics_read_ok; /* Transport success only; single-register reads have no CRC. */
    uint32_t diagnostics_timestamp_ms;
    uint32_t diagnostics_io_errors;
    encoder_register_check_t register_check; /* 默认关闭开发核对时为零，不表示通信故障。 */
    bool bandwidth_change_evaluated; /* One attempt after startup register reads succeed. */
    mt6826s_status_t bandwidth_change_status; /* Inspect only when evaluated is true. */
    mt6826s_bandwidth_change_t bandwidth_change;
    encoder_motion_output_t motion;
    bsp_mt6826s_dma_stats_t dma;
} encoder_output_t;

extern volatile encoder_output_t g_encoder;
extern volatile encoder_output_t g_encoder2;

/* 外设初始化后、osKernelStart 前启动两路任务；两路均创建成功才返回 true。
   一路失败不会阻止另一路继续运行，各路错误保存在自己的输出结构体。 */
bool encoder_app_start(void);
/* 仅任务上下文调用；复制一致快照并检查数据是否过期。 */
bool encoder_app_get_output(encoder_output_t *output);
/* Select either encoder. The original getter above selects ENCODER_1. */
bool encoder_app_get_output_for(encoder_id_t id, encoder_output_t *output);
/* 提供给 IMU 的静止门限快照：bit2/bit3 分别表示轮1/轮2阻止零偏学习。
   拒绝样本、重置、采样间隔异常或数据过期都会阻止学习。 */
uint8_t encoder_app_stationary_block_mask(uint32_t now_us);
/* 在任务中请求重新建立累计位移原点，由后续健康样本执行。
   不修改编码器硬件零位，也不修改单圈 angle_deg。 */
bool encoder_app_reset_motion(encoder_id_t id);

#endif
