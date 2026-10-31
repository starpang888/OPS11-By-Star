/** @file attitude_wheel_gate.h
 * @brief modules 层随动轮静止判据：通过累计计数的短步长和长窗口范围辅助 IMU 零偏学习。
 */
#ifndef ATTITUDE_WHEEL_GATE_H
#define ATTITUDE_WHEEL_GATE_H

#include <stdbool.h>
#include <stdint.h>

/* 21 个端点覆盖 20×5 ms 时间段，不依赖编码器与 IMU 任务恰好同步执行。 */
#define ATTITUDE_WHEEL_POINTS 21U
typedef struct {
    int64_t counts[ATTITUDE_WHEEL_POINTS];
    uint32_t time[ATTITUDE_WHEEL_POINTS];
    uint32_t last_sample_us;
    unsigned head, size;
    bool valid, stationary;
} attitude_wheel_gate_t;

/* time_us 为公共微秒时基；counts 为累计计数；两个阈值的单位均为编码器计数。
   无效样本、计数重置或采样间隔异常都会清除静止窗口。 */
void attitude_wheel_gate_update(attitude_wheel_gate_t *gate, uint32_t time_us,
                                 int64_t counts, bool valid, bool reset,
                                 uint32_t step_limit, uint32_t range_limit);

#endif
