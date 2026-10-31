/** @file attitude_wheel_gate.c
 * @brief modules 层轮子静止窗口实现：先检查瞬时位移，再检查至少 100 ms 的计数范围。
 */
#include "attitude_wheel_gate.h"

void attitude_wheel_gate_update(attitude_wheel_gate_t *g, uint32_t now,
                                 int64_t counts, bool valid, bool reset,
                                 uint32_t step_limit, uint32_t range_limit) {
    if (reset || !valid || (g->valid && (uint32_t)(now - g->last_sample_us) > 5000U)) {
        g->size = 0;
        g->head = 0;
        g->stationary = false;
    }
    g->last_sample_us = now;
    g->valid = valid;
    if (!valid) {
        return;
    }
    const unsigned previous = (g->head + ATTITUDE_WHEEL_POINTS - 1U) % ATTITUDE_WHEEL_POINTS;
    if (g->size > 0 && (uint32_t)(now - g->time[previous]) < 5000U) {
        /* 即使尚未到下一个 5 ms 端点，也检查往返运动，避免快速拨动后回到原位
           被窗口端点采样遗漏，导致错误学习陀螺仪零偏。 */
        const int64_t delta = counts - g->counts[previous];
        if (delta > (int64_t)step_limit || delta < -(int64_t)step_limit) {
            g->stationary = false;
            g->size = 0;
            g->head = 0;
        } else {
            return;
        }
    }
    if (g->size > 0) {
        const int64_t delta = counts - g->counts[previous];
        if (delta > (int64_t)step_limit || delta < -(int64_t)step_limit ||
            (uint32_t)(now - g->time[previous]) > 10000U) {
            g->size = 0;
            g->head = 0;
        }
    }
    g->counts[g->head] = counts;
    g->time[g->head] = now;
    g->head = (g->head + 1U) % ATTITUDE_WHEEL_POINTS;
    if (g->size < ATTITUDE_WHEEL_POINTS) {
        ++g->size;
    }
    g->stationary = false;
    if (g->size == ATTITUDE_WHEEL_POINTS) {
        int64_t minimum = g->counts[0], maximum = minimum;
        for (unsigned i = 1; i < ATTITUDE_WHEEL_POINTS; ++i) {
            if (g->counts[i] < minimum) minimum = g->counts[i];
            if (g->counts[i] > maximum) maximum = g->counts[i];
        }
        g->stationary = (maximum - minimum <= (int64_t)range_limit) &&
            (uint32_t)(now - g->time[g->head]) >= 100000U;
    }
}
