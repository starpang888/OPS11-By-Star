/** @file encoder_motion.c
 * @brief modules 层多圈展开与测速实现。处理 0/360 度跨越，使用真实毫秒间隔计算转速。
 */
#include "encoder_motion.h"

#include <float.h>
#include <stddef.h>

bool encoder_motion_init(encoder_motion_t *motion, const encoder_motion_config_t *config) {
    if ((motion == NULL) || (config == NULL) ||
        (config->counts_per_revolution < 2U) || (config->counts_per_revolution > 65536U) ||
        ((config->counts_per_revolution & 1U) != 0U) || (config->max_gap_ms == 0U) ||
        (config->max_gap_ms >= UINT32_MAX / 2U) || (config->speed_window_ms == 0U) ||
        (config->speed_window_ms >= UINT32_MAX / 2U)) {
        return false;
    }
    *motion = (encoder_motion_t) { .config = *config };
    return true;
}

void encoder_motion_reset(encoder_motion_t *motion) {
    const encoder_motion_config_t config = motion->config;
    *motion = (encoder_motion_t) { .config = config };
}

void encoder_motion_invalidate(encoder_motion_t *motion) {
    if (motion->output.initialized) {
        if (motion->have_previous) {
            ++motion->output.discontinuities;
        }
        motion->output.tracking_continuous = false;
    }
    motion->have_previous = false;
    motion->output.valid = false;
    motion->output.speed_valid = false;
    motion->output.speed_raw_rpm = 0.0f;
    motion->output.speed_rpm = 0.0f;
}

static void anchor(encoder_motion_t *motion, uint16_t raw, uint32_t timestamp_ms) {
    motion->have_previous = true;
    motion->previous_raw = raw;
    motion->previous_ms = timestamp_ms;
    motion->window_counts = motion->output.total_counts;
    motion->window_ms = timestamp_ms;
}

void encoder_motion_update(encoder_motion_t *motion, uint16_t raw, uint32_t timestamp_ms) {
    const uint32_t counts_per_rev = motion->config.counts_per_revolution;
    if (raw >= counts_per_rev) {
        encoder_motion_invalidate(motion);
        return;
    }
    if (!motion->output.initialized) {
        motion->output.initialized = true;
        motion->output.tracking_continuous = true;
        motion->output.valid = true;
        anchor(motion, raw, timestamp_ms);
        return;
    }
    if (!motion->have_previous) {
        anchor(motion, raw, timestamp_ms);
        return;
    }
    const uint32_t elapsed_ms = (uint32_t)(timestamp_ms - motion->previous_ms);
    int32_t delta = (int32_t)raw - (int32_t)motion->previous_raw;
    const int32_t half_rev = (int32_t)counts_per_rev / 2;
    /* 恰好半圈无法确定方向；间隔过长时，即使端点角度接近也不能保证未跨多圈。 */
    if ((elapsed_ms > motion->config.max_gap_ms) || (delta == half_rev) || (delta == -half_rev)) {
        encoder_motion_invalidate(motion);
        anchor(motion, raw, timestamp_ms);
        return;
    }
    if (delta > half_rev) {
        delta -= (int32_t)counts_per_rev;
    } else if (delta < -half_rev) {
        delta += (int32_t)counts_per_rev;
    }
    if (((delta > 0) && (motion->output.total_counts > INT64_MAX - delta)) ||
        ((delta < 0) && (motion->output.total_counts < INT64_MIN - delta))) {
        encoder_motion_invalidate(motion);
        anchor(motion, raw, timestamp_ms);
        return;
    }
    motion->output.total_counts += delta;
    motion->output.turns = motion->output.total_counts / (int64_t)counts_per_rev;
    motion->output.total_angle_deg = (double)motion->output.total_counts * (360.0 / counts_per_rev);
    motion->output.valid = motion->output.tracking_continuous;
    motion->previous_raw = raw;
    motion->previous_ms = timestamp_ms;

    const uint32_t window_elapsed_ms = (uint32_t)(timestamp_ms - motion->window_ms);
    if (window_elapsed_ms >= motion->config.speed_window_ms) {
        const int64_t counts = motion->output.total_counts - motion->window_counts;
        const float rpm = (float)counts * (60000.0f / counts_per_rev) / (float)window_elapsed_ms;
        const float alpha = (float)window_elapsed_ms /
                            ((float)motion->config.speed_filter_tau_ms + (float)window_elapsed_ms);
        motion->output.speed_raw_rpm = rpm;
        /* 重置或失效后滤波器从零起步，首个测速窗口不视为已稳定的测量值。 */
        motion->output.speed_rpm += alpha * (rpm - motion->output.speed_rpm);
        /* 零输入下滤波尾部可能停在极小的非规格化浮点数，例如 1.4013e-45。
           仅清除此数值尾巴，不对真实小转速添加人为死区。 */
        if ((rpm == 0.0f) && (motion->output.speed_rpm > -FLT_MIN) &&
            (motion->output.speed_rpm < FLT_MIN)) {
            motion->output.speed_rpm = 0.0f;
        }
        motion->output.speed_valid = true;
        ++motion->output.speed_updates;
        motion->window_counts = motion->output.total_counts;
        motion->window_ms = timestamp_ms;
    }
}
