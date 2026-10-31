/** @file odometry_app.c
 * @brief application 层里程计同步任务。用 IMU 时间点查找两轮前后样本，插值后做平面积分。
 *        短时同步拒绝允许有界续接；计数或姿态真正不连续时保留坐标并报告跟踪丢失。
 */
#include "odometry_app.h"
#include "odometry_config.h"
#include "odometry.h"
#include "bsp_time.h"
#include "mt6826s.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

_Static_assert(ODOMETRY_ENCODER_HISTORY >= 4 && ODOMETRY_IMU_QUEUE >= 4,
               "Synchronization needs bounded sample history");
_Static_assert(pdMS_TO_TICKS(1) > 0, "Odometry requires a 1 ms or faster RTOS tick");

typedef struct {
    uint32_t time, generation;
    int64_t counts;
} wheel_sample_t;
typedef struct {
    wheel_sample_t data[ODOMETRY_ENCODER_HISTORY];
    unsigned head, size;
} wheel_history_t;
typedef struct {
    uint32_t time, generation;
    float yaw;
} yaw_sample_t;
typedef struct {
    yaw_sample_t yaw;
    wheel_sample_t lower[2], upper[2];
    uint32_t generation[3];
    uint32_t overruns;
    unsigned wheel_samples[2];
    bool have_yaw, bracket[2];
} input_t;

volatile odometry_output_t g_odometry;
static wheel_history_t wheels[2];
static yaw_sample_t yaw_queue[ODOMETRY_IMU_QUEUE];
static unsigned yaw_head, yaw_size;
static uint32_t generations[3], overruns;
static osThreadId task_handle;
static bool reset_requested;

void odometry_app_submit_encoder(unsigned id, uint32_t timestamp_us,
                                 int64_t counts, bool valid, bool origin_reset) {
    if (id >= 2) {
        return;
    }
    taskENTER_CRITICAL();
    if (origin_reset || !valid) {
        ++generations[id];
    }
    if (valid) {
        wheel_history_t *h = &wheels[id];
        h->data[h->head] = (wheel_sample_t){ timestamp_us, generations[id], counts };
        h->head = (h->head + 1U) % ODOMETRY_ENCODER_HISTORY;
        if (h->size < ODOMETRY_ENCODER_HISTORY) {
            ++h->size;
        }
    }
    taskEXIT_CRITICAL();
}

void odometry_app_submit_imu(uint32_t timestamp_us, float yaw_deg, bool valid) {
    taskENTER_CRITICAL();
    if (!valid) {
        ++generations[2];
    } else {
        if (yaw_size == ODOMETRY_IMU_QUEUE) {
            ++overruns;
            ++generations[2];
            /* 队列满时丢最旧端点并改变代次；已建立的定位据此判定连续性丢失。 */
            --yaw_size;
        }
        yaw_queue[yaw_head] = (yaw_sample_t){ timestamp_us, generations[2], yaw_deg };
        yaw_head = (yaw_head + 1U) % ODOMETRY_IMU_QUEUE;
        ++yaw_size;
    }
    taskEXIT_CRITICAL();
}

static void publish(const odometry_output_t *output) {
    taskENTER_CRITICAL();
    g_odometry = *output;
    taskEXIT_CRITICAL();
}

bool odometry_app_get_output(odometry_output_t *output) {
    if (output == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    *output = g_odometry;
    taskEXIT_CRITICAL();
    output->valid = output->valid && output->tracking_continuous &&
        output->state == ODOMETRY_STATE_RUNNING && output->error == ODOMETRY_ERROR_NONE &&
        (uint32_t)(bsp_time_micros() - output->timestamp_us) < ODOMETRY_STALE_AFTER_US;
    return output->valid;
}

bool odometry_app_reset(void) {
    taskENTER_CRITICAL();
    const bool started = task_handle != NULL;
    reset_requested = started;
    taskEXIT_CRITICAL();
    return started;
}

/* 只比较短历史内的时间戳；无符号相减和有符号先后判断可跨越 TIM2 的 uint32_t 回绕。 */
static void read_input(input_t *input) {
    memset(input, 0, sizeof(*input));
    taskENTER_CRITICAL();
    memcpy(input->generation, generations, sizeof(generations));
    input->overruns = overruns;
    for (unsigned i = 0; i < 2; ++i) {
        input->wheel_samples[i] = wheels[i].size;
    }
    if (yaw_size != 0) {
        input->have_yaw = true;
        input->yaw = yaw_queue[(yaw_head + ODOMETRY_IMU_QUEUE - yaw_size) % ODOMETRY_IMU_QUEUE];
        for (unsigned id = 0; id < 2; ++id) {
            const wheel_history_t *h = &wheels[id];
            const unsigned oldest = (h->head + ODOMETRY_ENCODER_HISTORY - h->size) % ODOMETRY_ENCODER_HISTORY;
            bool have_lower = false;
            for (unsigned j = 0; j < h->size; ++j) {
                const wheel_sample_t sample = h->data[(oldest + j) % ODOMETRY_ENCODER_HISTORY];
                const int32_t delta = (int32_t)(sample.time - input->yaw.time);
                if (delta <= 0) {
                    input->lower[id] = sample;
                    have_lower = true;
                }
                if (delta >= 0 && have_lower) {
                    input->upper[id] = sample;
                    input->bracket[id] = true;
                    break;
                }
            }
        }
    }
    taskEXIT_CRITICAL();
}

static void pop_yaw(uint32_t time) {
    taskENTER_CRITICAL();
    /* 计算期间生产者可能因溢出替换最旧端点；只弹出同一时间戳，不能误删替换后的端点。
       下一次代次检查会识别该次连续性丢失。 */
    if (yaw_size != 0 &&
        yaw_queue[(yaw_head + ODOMETRY_IMU_QUEUE - yaw_size) % ODOMETRY_IMU_QUEUE].time == time) {
        --yaw_size;
    }
    taskEXIT_CRITICAL();
}

static void lose_tracking(odometry_output_t *out, odometry_error_t error) {
    out->valid = false;
    out->tracking_continuous = false;
    out->state = ODOMETRY_STATE_TRACKING_LOST;
    out->error = error;
    if (error == ODOMETRY_ERROR_SYNC) {
        ++out->sync_errors;
    } else {
        ++out->input_errors;
    }
}

static odometry_sync_failure_t interpolate(const input_t *input, odometry_position_t positions[2],
                                           uint32_t gaps[2], uint8_t *mask) {
    /* 每路必须有真实前后样本，且代次一致、时间跨度合格、计数差无半圈歧义。 */
    *mask = 0;
    for (unsigned i = 0; i < 2; ++i) {
        gaps[i] = input->bracket[i] ? input->upper[i].time - input->lower[i].time : 0;
    }
    for (unsigned i = 0; i < 2; ++i) {
        *mask = (uint8_t)(1U << i);
        if (!input->bracket[i]) {
            return ODOMETRY_SYNC_NO_BRACKET;
        }
        if (input->lower[i].generation != input->generation[i] ||
            input->upper[i].generation != input->generation[i]) {
            return ODOMETRY_SYNC_GENERATION;
        }
        const wheel_sample_t *a = &input->lower[i], *b = &input->upper[i];
        const uint32_t span = b->time - a->time;
        gaps[i] = span;
        if (span > ODOMETRY_ENCODER_MAX_BRACKET_US) {
            return ODOMETRY_SYNC_BRACKET_TOO_WIDE;
        }
        const uint64_t magnitude = b->counts >= a->counts ?
            (uint64_t)b->counts - (uint64_t)a->counts : (uint64_t)a->counts - (uint64_t)b->counts;
        if (magnitude >= MT6826S_COUNTS_PER_REVOLUTION / 2U) {
            return ODOMETRY_SYNC_COUNT_DELTA;
        }
        const double delta = b->counts >= a->counts ? (double)magnitude : -(double)magnitude;
        positions[i].counts = a->counts;
        positions[i].fraction = span == 0 ? 0 : delta * (double)(uint32_t)(input->yaw.time - a->time) / span;
    }
    if (input->yaw.generation != input->generation[2]) {
        *mask = 4U;
        return ODOMETRY_SYNC_GENERATION;
    }
    *mask = 0;
    return ODOMETRY_SYNC_NONE;
}

static void record_sync_detail(odometry_output_t *out, odometry_sync_failure_t reason,
                               uint8_t mask, uint32_t age, uint32_t interval,
                               const uint32_t gaps[2]) {
    out->sync_failure = reason;
    out->sync_input_mask = mask;
    out->failure_age_us = age;
    out->failure_interval_us = interval;
    out->failure_bracket_us[0] = gaps == NULL ? 0 : gaps[0];
    out->failure_bracket_us[1] = gaps == NULL ? 0 : gaps[1];
}

static void record_sync_failure(odometry_output_t *out, odometry_sync_failure_t reason,
                                uint8_t mask, uint32_t age, uint32_t interval,
                                const uint32_t gaps[2]) {
    record_sync_detail(out, reason, mask, age, interval, gaps);
    lose_tracking(out, ODOMETRY_ERROR_SYNC);
}

/* 只拒绝该插值端点，不重置原点，也不外推。
   下一成功更新仍从上一成功位姿计算累计计数差和连续航向差，因此不丢弃中间位移。 */
static void skip_sync_endpoint(odometry_output_t *out, bool *recovering,
                               odometry_sync_failure_t reason, uint8_t mask,
                               uint32_t age, uint32_t interval, const uint32_t gaps[2]) {
    record_sync_detail(out, reason, mask, age, interval, gaps);
    out->valid = false;
    out->state = ODOMETRY_STATE_WAITING;
    out->error = ODOMETRY_ERROR_SYNC;
    out->wait_reason = ODOMETRY_WAIT_SYNC;
    ++out->skipped_sync_samples;
    *recovering = true;
}

static void odometry_task(void const *argument) {
    (void)argument;
    static odometry_t odometry;
    odometry_output_t out = {0};
    const odometry_config_t config = {
        .wheel_diameter_mm = { ODOMETRY_WHEEL_1_DIAMETER_MM, ODOMETRY_WHEEL_2_DIAMETER_MM },
        .counts_per_revolution = MT6826S_COUNTS_PER_REVOLUTION,
        .encoder_revolutions_per_wheel_revolution = { ODOMETRY_WHEEL_1_GEAR_RATIO, ODOMETRY_WHEEL_2_GEAR_RATIO },
        .direction = { ODOMETRY_WHEEL_1_DIRECTION, ODOMETRY_WHEEL_2_DIRECTION },
        .rolling_angle_deg = { ODOMETRY_WHEEL_1_ANGLE_DEG, ODOMETRY_WHEEL_2_ANGLE_DEG },
        .intersection_x_mm = ODOMETRY_INTERSECTION_X_MM,
        .intersection_y_mm = ODOMETRY_INTERSECTION_Y_MM
    };
    if (!bsp_time_initialise()) {
        out.state = ODOMETRY_STATE_ERROR;
        out.error = ODOMETRY_ERROR_TIMEBASE;
    } else if (!odometry_init(&odometry, &config)) {
        out.state = ODOMETRY_STATE_ERROR;
        out.error = ODOMETRY_ERROR_CONFIG;
    }
    uint32_t reference_generation[3] = {0};
    bool recovering_sync = false;
    uint32_t rate_start = bsp_time_micros(), rate_updates = 0;
    for (;;) {
        ++out.task_cycles;
        taskENTER_CRITICAL();
        const bool reset = reset_requested;
        reset_requested = false;
        if (reset) {
            memset(wheels, 0, sizeof(wheels));
            yaw_size = 0;
        }
        taskEXIT_CRITICAL();
        if (reset && out.state != ODOMETRY_STATE_ERROR) {
            const uint32_t resets = out.resets + 1U;
            out = (odometry_output_t){ .resets = resets };
            recovering_sync = false;
            (void)odometry_init(&odometry, &config);
            rate_start = bsp_time_micros();
            rate_updates = 0;
        }
        /* 每次唤醒处理量有上限，正常情况下约消费一个 500 Hz IMU 端点。 */
        for (unsigned work = 0; work < ODOMETRY_IMU_QUEUE; ++work) {
            if (out.state == ODOMETRY_STATE_ERROR || out.state == ODOMETRY_STATE_TRACKING_LOST) {
                break;
            }
            input_t input;
            read_input(&input);
            out.imu_queue_overruns = input.overruns;
            out.wait_reason = ODOMETRY_WAIT_NONE;
            const uint32_t now = bsp_time_micros();
            if (odometry.initialized && memcmp(reference_generation, input.generation, sizeof(reference_generation)) != 0) {
                for (unsigned i = 0; i < 3; ++i) {
                    if (reference_generation[i] != input.generation[i]) {
                        out.lost_input_mask |= (uint8_t)(1U << i);
                    }
                }
                lose_tracking(&out, ODOMETRY_ERROR_INPUT);
                break;
            }
            if (!input.have_yaw) {
                out.wait_reason = !odometry.initialized ? ODOMETRY_WAIT_IMU :
                    recovering_sync ? ODOMETRY_WAIT_SYNC : ODOMETRY_WAIT_NONE;
                if (odometry.initialized && (uint32_t)(now - out.timestamp_us) >= ODOMETRY_STALE_AFTER_US) {
                    record_sync_failure(&out, ODOMETRY_SYNC_NO_IMU, 4U,
                                        now - out.timestamp_us, 0, NULL);
                }
                break;
            }
            const uint32_t age = now - input.yaw.time;
            odometry_position_t positions[2];
            uint32_t gaps[2];
            uint8_t sync_mask;
            const odometry_sync_failure_t sync = interpolate(&input, positions, gaps, &sync_mask);
            if (sync != ODOMETRY_SYNC_NONE) {
                out.wait_reason = input.wheel_samples[0] == 0 ? ODOMETRY_WAIT_ENCODER_1 :
                    input.wheel_samples[1] == 0 ? ODOMETRY_WAIT_ENCODER_2 : ODOMETRY_WAIT_SYNC;
                if (sync == ODOMETRY_SYNC_NO_BRACKET && age < ODOMETRY_SYNC_TIMEOUT_US) {
                    break; /* 等实际后侧编码器样本到达，不用外推代替测量。 */
                }
                pop_yaw(input.yaw.time);
                if (odometry.initialized) {
                    if (sync == ODOMETRY_SYNC_NO_BRACKET ||
                        sync == ODOMETRY_SYNC_BRACKET_TOO_WIDE) {
                        skip_sync_endpoint(&out, &recovering_sync, sync, sync_mask, age,
                                           input.yaw.time - out.timestamp_us, gaps);
                    } else {
                        record_sync_failure(&out, sync, sync_mask, age,
                                            input.yaw.time - out.timestamp_us, gaps);
                        break;
                    }
                }
                continue; /* 启动时可继续寻找后续被两轮历史支持的公共原点。 */
            }
            const uint32_t interval = input.yaw.time - out.timestamp_us;
            if (odometry.initialized &&
                (interval == 0U || interval > ODOMETRY_IMU_MAX_INTERVAL_US)) {
                pop_yaw(input.yaw.time);
                record_sync_failure(&out, ODOMETRY_SYNC_IMU_INTERVAL, 4U, age, interval, gaps);
                break;
            }
            if (age >= ODOMETRY_SYNC_TIMEOUT_US) {
                pop_yaw(input.yaw.time);
                if (odometry.initialized) {
                    skip_sync_endpoint(&out, &recovering_sync, ODOMETRY_SYNC_TOO_OLD,
                                       0U, age, interval, gaps);
                }
                continue;
            }
            const bool establishing = !odometry.initialized;
            if (!odometry_update(&odometry, positions, input.yaw.yaw)) {
                lose_tracking(&out, ODOMETRY_ERROR_GEOMETRY);
                break;
            }
            if (establishing) {
                memcpy(reference_generation, input.generation, sizeof(reference_generation));
            }
            pop_yaw(input.yaw.time);
            out.x_mm = odometry.x_mm;
            out.y_mm = odometry.y_mm;
            out.yaw_deg = (float)odometry_yaw_deg(&odometry);
            out.timestamp_us = input.yaw.time;
            out.latency_us = age;
            memcpy(out.encoder_bracket_us, gaps, sizeof(gaps));
            out.state = ODOMETRY_STATE_RUNNING;
            out.error = ODOMETRY_ERROR_NONE;
            out.valid = true;
            out.tracking_continuous = true;
            out.wait_reason = ODOMETRY_WAIT_NONE;
            if (recovering_sync) {
                ++out.sync_recoveries;
                recovering_sync = false;
            }
            ++out.updates;
        }
        if (recovering_sync && out.state == ODOMETRY_STATE_WAITING &&
            (uint32_t)(bsp_time_micros() - out.timestamp_us) >= ODOMETRY_STALE_AFTER_US) {
            /* 超过新鲜度窗口仍未恢复：保留同步首因，锁存跟踪丢失并维持坐标无效。 */
            lose_tracking(&out, ODOMETRY_ERROR_SYNC);
        }
        const uint32_t elapsed = bsp_time_micros() - rate_start;
        if (elapsed >= 1000000U) {
            out.measured_update_rate_hz = (float)(uint32_t)(out.updates - rate_updates) * 1000000.0f / elapsed;
            rate_updates = out.updates;
            rate_start = bsp_time_micros();
        }
        publish(&out);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool odometry_app_start(void) {
    if (task_handle != NULL) {
        return true;
    }
    /* 编码器采样优先于定位插值/积分，DMA 唤醒采样任务时可及时抢占计算任务。 */
    osThreadDef(odometryTask, odometry_task, osPriorityBelowNormal, 0, ODOMETRY_TASK_STACK_WORDS);
    task_handle = osThreadCreate(osThread(odometryTask), NULL);
    if (task_handle == NULL) {
        g_odometry.state = ODOMETRY_STATE_ERROR;
        g_odometry.error = ODOMETRY_ERROR_TASK;
        return false;
    }
    return true;
}
