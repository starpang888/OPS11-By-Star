/** @file encoder_motion.h
 * @brief modules 层编码器运动接口：单圈角度展开为有符号多圈位移、窗口转速及滤波转速。
 */
#ifndef ENCODER_MOTION_H
#define ENCODER_MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t counts_per_revolution;
    uint32_t max_gap_ms;
    uint32_t speed_window_ms;
    uint32_t speed_filter_tau_ms; /* 转速一阶滤波时间常数，毫秒；为 0 时禁用滤波。 */
} encoder_motion_config_t;

typedef struct {
    bool initialized;
    bool tracking_continuous; /* 数据中断后锁存为 false，重置后才重新建立连续轨迹。 */
    bool valid; /* 当前累计位置是否从第一个健康样本连续积分而来。 */
    bool speed_valid; /* 是否已形成完整且连续的健康测速窗口。 */
    int64_t total_counts; /* 相对第一个健康样本的精确有符号累计计数。 */
    int64_t turns; /* 净转过的完整圈数，向零截断。 */
    double total_angle_deg; /* 有符号累计角位移，单位度，不是转子绝对单圈角度。 */
    float speed_raw_rpm; /* 未滤波的窗口平均转速，转/分钟。 */
    float speed_rpm; /* 滤波转速；原始角度增加方向为正，单位转/分钟。 */
    uint32_t discontinuities; /* 丢失连续性的片段数，不是坏帧总数。 */
    uint32_t speed_updates;
} encoder_motion_output_t;

/* 每只编码器对应一个实例，由其采样任务独占；不依赖 HAL/RTOS。 */
typedef struct {
    encoder_motion_config_t config;
    encoder_motion_output_t output;
    bool have_previous;
    uint16_t previous_raw;
    uint32_t previous_ms;
    int64_t window_counts;
    uint32_t window_ms;
} encoder_motion_t;

bool encoder_motion_init(encoder_motion_t *motion, const encoder_motion_config_t *config);
void encoder_motion_reset(encoder_motion_t *motion);
/* 每个健康周期样本调用一次；相邻样本实际转动必须严格小于半圈。
   超过半圈可能产生无法从单圈角度检测出的方向混叠。 */
void encoder_motion_update(encoder_motion_t *motion, uint16_t raw, uint32_t timestamp_ms);
/* 拒绝样本时调用；保留已有累计位移，丢弃测速历史，不跨未知区间继续积分。 */
void encoder_motion_invalidate(encoder_motion_t *motion);

#endif
