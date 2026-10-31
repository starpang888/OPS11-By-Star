#include "encoder_app.h"

#include "bsp_mt6826s.h"
#include "encoder_config.h"
#include "odometry_app.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

_Static_assert(ENCODER_SAMPLE_PERIOD_MS > 0, "Encoder sample period must be positive");
_Static_assert(pdMS_TO_TICKS(ENCODER_SAMPLE_PERIOD_MS) > 0, "Encoder period is shorter than one tick");
_Static_assert(ENCODER_BANDWIDTH_TARGET <= 7U, "Encoder BW must fit three bits");
_Static_assert(ENCODER_MOTION_MAX_GAP_MS >= ENCODER_SAMPLE_PERIOD_MS, "Motion gap shorter than sample period");
_Static_assert(ENCODER_SPEED_WINDOW_MS > 0U, "Speed window must be positive");

volatile encoder_output_t g_encoder;
volatile encoder_output_t g_encoder2;

typedef struct {
    bsp_mt6826s_id_t bsp_id;
    volatile encoder_output_t *published;
    encoder_output_t output;
    osThreadId task_handle;
    encoder_motion_t motion;
    bool reset_motion_requested; /* Access only under the task critical section. */
} encoder_instance_t;

static encoder_instance_t encoder_instances[ENCODER_COUNT] = {
    { .bsp_id = BSP_MT6826S_1, .published = &g_encoder },
    { .bsp_id = BSP_MT6826S_2, .published = &g_encoder2 }
};

static void publish(encoder_instance_t *instance) {
    (void)bsp_mt6826s_get_dma_stats(instance->bsp_id, &instance->output.dma);
    taskENTER_CRITICAL();
    *instance->published = instance->output;
    taskEXIT_CRITICAL();
}

bool encoder_app_get_output(encoder_output_t *output) {
    return encoder_app_get_output_for(ENCODER_1, output);
}

bool encoder_app_get_output_for(encoder_id_t id, encoder_output_t *output) {
    if ((output == NULL) || ((unsigned)id >= ENCODER_COUNT)) {
        return false;
    }
    taskENTER_CRITICAL();
    *output = *encoder_instances[id].published;
    taskEXIT_CRITICAL();
    output->valid = output->valid && ((uint32_t)(HAL_GetTick() - output->timestamp_ms) < ENCODER_STALE_AFTER_MS);
    output->motion.valid = output->motion.valid && output->valid;
    output->motion.speed_valid = output->motion.speed_valid && output->valid;
    return output->valid;
}

bool encoder_app_reset_motion(encoder_id_t id) {
    if ((unsigned)id >= ENCODER_COUNT) {
        return false;
    }
    taskENTER_CRITICAL();
    encoder_instance_t *instance = &encoder_instances[id];
    const bool started = instance->task_handle != NULL;
    if (started) {
        instance->reset_motion_requested = true;
    }
    taskEXIT_CRITICAL();
    return started;
}

static bool checked_frame(mt6826s_status_t status) {
    return (status == MT6826S_OK) || (status == MT6826S_ERROR_ALARM);
}

static void check_register_reads(const mt6826s_bus_t *bus, encoder_register_check_t *check) {
    mt6826s_sample_t single;
    mt6826s_sample_t burst;
    check->single_status = mt6826s_read_via_registers(bus, &single);
    check->burst_status = mt6826s_read(bus, &burst);
    memcpy(check->single_frame, single.raw, sizeof(check->single_frame));
    memcpy(check->burst_frame, burst.raw, sizeof(check->burst_frame));
    check->single_crc_valid = checked_frame(check->single_status);
    check->burst_crc_valid = checked_frame(check->burst_status);
    check->single_angle_deg = single.angle_deg;
    check->burst_angle_deg = burst.angle_deg;
    check->nonzero_pair = (single.angle_raw != 0 || single.alarms != 0) &&
                          (burst.angle_raw != 0 || burst.alarms != 0);
    check->passed = false;
    check->angle_difference_deg = 0;
    if (check->single_crc_valid && check->burst_crc_valid) {
        int32_t delta = (int32_t)single.angle_raw - (int32_t)burst.angle_raw;
        if (delta > (int32_t)MT6826S_COUNTS_PER_REVOLUTION / 2) {
            delta -= (int32_t)MT6826S_COUNTS_PER_REVOLUTION;
        } else if (delta < -(int32_t)MT6826S_COUNTS_PER_REVOLUTION / 2) {
            delta += (int32_t)MT6826S_COUNTS_PER_REVOLUTION;
        }
        check->angle_difference_deg = (float)delta * (360.0f / MT6826S_COUNTS_PER_REVOLUTION);
        check->passed = check->nonzero_pair &&
                        (check->angle_difference_deg >= -ENCODER_REGISTER_CHECK_TOLERANCE_DEG) &&
                        (check->angle_difference_deg <= ENCODER_REGISTER_CHECK_TOLERANCE_DEG);
    }
    ++check->checks;
    if (check->passed) {
        ++check->passes;
    }
    check->timestamp_ms = HAL_GetTick();
}

static void read_diagnostics(const mt6826s_bus_t *bus, encoder_output_t *output) {
    output->diagnostics_status = mt6826s_read_diagnostics(bus, &output->diagnostics);
    output->diagnostics_read_ok = output->diagnostics_status == MT6826S_OK;
    if (output->diagnostics_read_ok) {
        output->diagnostics_timestamp_ms = HAL_GetTick();
    } else {
        ++output->diagnostics_io_errors;
    }
    check_register_reads(bus, &output->register_check);
    if (ENCODER_BANDWIDTH_CHANGE_ENABLED && !output->bandwidth_change_evaluated &&
        output->diagnostics_read_ok && output->register_check.passed) {
        /* A fresh read inside the driver checks BW again before any write.
           Do not retry an uncertain write or a failed read-back this boot. */
        output->bandwidth_change_evaluated = true;
        output->bandwidth_change_status = mt6826s_set_bandwidth(
            bus, 0U, ENCODER_BANDWIDTH_TARGET, &output->bandwidth_change);
        if (output->bandwidth_change.verified) {
            output->diagnostics.bandwidth_register = output->bandwidth_change.after_register;
            output->diagnostics.bandwidth = output->bandwidth_change.after_register & 0x07U;
        }
    }
}

static void encoder_task(void const *argument) {
    encoder_instance_t *instance = (encoder_instance_t *)argument;
    encoder_output_t *output = &instance->output;
    mt6826s_bus_t bus;
    output->sample_rate_hz = 1000U / ENCODER_SAMPLE_PERIOD_MS;
    publish(instance);
    const encoder_motion_config_t motion_config = {
        .counts_per_revolution = MT6826S_COUNTS_PER_REVOLUTION,
        .max_gap_ms = ENCODER_MOTION_MAX_GAP_MS,
        .speed_window_ms = ENCODER_SPEED_WINDOW_MS,
        .speed_filter_tau_ms = ENCODER_SPEED_FILTER_TAU_MS
    };
    if (!encoder_motion_init(&instance->motion, &motion_config)) {
        output->state = ENCODER_STATE_ERROR;
        output->error = ENCODER_ERROR_MOTION;
        publish(instance);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!bsp_mt6826s_initialise(instance->bsp_id, &bus)) {
        output->state = ENCODER_STATE_ERROR;
        output->error = ENCODER_ERROR_BSP;
        publish(instance);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    /* Power-up preparation is typically 3 ms. Allow 10 ms plus tick alignment. */
    vTaskDelay(pdMS_TO_TICKS(ENCODER_STARTUP_DELAY_MS) + 1U);
    read_diagnostics(&bus, output);
    uint32_t last_diagnostics_ms = HAL_GetTick();
    uint32_t rate_window_ms = HAL_GetTick();
    uint32_t rate_window_attempts = output->attempts;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        taskENTER_CRITICAL();
        const bool reset_requested = instance->reset_motion_requested;
        instance->reset_motion_requested = false;
        taskEXIT_CRITICAL();
        if (reset_requested) {
            encoder_motion_reset(&instance->motion);
        }
        const uint32_t discontinuities_before = instance->motion.output.discontinuities;
        mt6826s_sample_t sample;
        output->sensor_status = mt6826s_read(&bus, &sample);
        const uint32_t latch_ms = bsp_mt6826s_last_latch_ms(instance->bsp_id);
        const uint32_t latch_us = bsp_mt6826s_last_latch_us(instance->bsp_id);
        output->last_attempt_ms = latch_ms;
        ++output->attempts;
        output->spi_error = bsp_mt6826s_spi_error(instance->bsp_id);
        output->valid = false;
        output->crc_valid = false;
        output->alarms = 0;
        output->overspeed = false;
        output->weak_magnet = false;
        output->undervoltage = false;
        if ((output->sensor_status != MT6826S_ERROR_IO) && (output->sensor_status != MT6826S_ERROR_ARGUMENT)) {
            memcpy(output->raw_frame, sample.raw, sizeof(output->raw_frame));
            output->crc_received = sample.raw[3];
            output->crc_calculated = sample.crc_calculated;
            output->crc_valid = (output->crc_received == output->crc_calculated);
            if (output->crc_valid && (output->sensor_status != MT6826S_ERROR_FORMAT)) {
                output->alarms = sample.alarms;
                output->overspeed = (sample.alarms & MT6826S_ALARM_OVERSPEED) != 0;
                output->weak_magnet = (sample.alarms & MT6826S_ALARM_WEAK_MAGNET) != 0;
                output->undervoltage = (sample.alarms & MT6826S_ALARM_UNDERVOLTAGE) != 0;
            }
        }
        if (output->sensor_status == MT6826S_OK) {
            output->state = ENCODER_STATE_RUNNING;
            output->error = ENCODER_ERROR_NONE;
            output->valid = true;
            output->consecutive_errors = 0;
            output->angle_raw = sample.angle_raw;
            output->angle_deg = sample.angle_deg;
            if (output->samples != 0) {
                output->sample_interval_us = latch_us - output->timestamp_us;
                if (output->sample_interval_us > output->max_sample_interval_us) {
                    output->max_sample_interval_us = output->sample_interval_us;
                }
            }
            output->timestamp_ms = latch_ms;
            output->timestamp_us = latch_us;
            ++output->samples;
            encoder_motion_update(&instance->motion, sample.angle_raw, latch_ms);
        } else {
            encoder_motion_invalidate(&instance->motion);
            output->state = ENCODER_STATE_ERROR;
            output->error = ENCODER_ERROR_SENSOR;
            ++output->consecutive_errors;
            switch (output->sensor_status) {
                case MT6826S_ERROR_CRC: ++output->crc_errors; break;
                case MT6826S_ERROR_FORMAT: ++output->format_errors; break;
                case MT6826S_ERROR_ALARM:
                    ++output->alarm_samples;
                    if (output->overspeed) {
                        ++output->overspeed_samples;
                    }
                    if (output->weak_magnet) {
                        ++output->weak_magnet_samples;
                    }
                    if (output->undervoltage) {
                        ++output->undervoltage_samples;
                    }
                    output->last_alarm_bits = sample.alarms;
                    memcpy(output->last_alarm_frame, sample.raw, sizeof(output->last_alarm_frame));
                    output->last_alarm_angle_deg = sample.angle_deg;
                    output->last_alarm_ms = latch_ms;
                    break;
                default: ++output->io_errors; break;
            }
        }
        output->motion = instance->motion.output;
        /* motion.valid describes the ENTIRE accumulated path and stays false
           after any past loss. Odometry has its own origin and may start from
           a later healthy segment. Notify each segment boundary explicitly,
           so an already established odometry origin still latches loss. */
        const bool segment_changed = reset_requested ||
            output->motion.discontinuities != discontinuities_before;
        odometry_app_submit_encoder((unsigned)instance->bsp_id, latch_us,
            output->motion.total_counts,
            output->valid && output->motion.initialized && instance->motion.have_previous,
            segment_changed);
        if ((uint32_t)(HAL_GetTick() - last_diagnostics_ms) >= ENCODER_DIAGNOSTICS_PERIOD_MS) {
            last_diagnostics_ms = HAL_GetTick();
            read_diagnostics(&bus, output);
        }
        output->motion = instance->motion.output;
        const uint32_t rate_elapsed_ms = (uint32_t)(HAL_GetTick() - rate_window_ms);
        if (rate_elapsed_ms >= 1000U) {
            output->measured_read_rate_hz = (float)(uint32_t)(output->attempts - rate_window_attempts) *
                                           1000.0f / (float)rate_elapsed_ms;
            rate_window_ms = HAL_GetTick();
            rate_window_attempts = output->attempts;
        }
        publish(instance);
        /* At exactly the next deadline, delay-until must return immediately.
           Adding another period there halves acquisition to 500 Hz. On a
           genuinely missed deadline, acquire once now and rebase the schedule
           to now; do not emit a burst of catch-up transfers. */
        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_wake) > pdMS_TO_TICKS(ENCODER_SAMPLE_PERIOD_MS)) {
            last_wake = now - pdMS_TO_TICKS(ENCODER_SAMPLE_PERIOD_MS);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_SAMPLE_PERIOD_MS));
    }
}

bool encoder_app_start(void) {
    /* CMSIS-RTOS v1's FreeRTOS adapter interprets stacksize as StackType_t words. */
    osThreadDef(encoderTask, encoder_task, osPriorityNormal, 0, ENCODER_TASK_STACK_WORDS);
    osThreadDef(encoder2Task, encoder_task, osPriorityNormal, 0, ENCODER_TASK_STACK_WORDS);
    const osThreadDef_t *definitions[ENCODER_COUNT] = {
        osThread(encoderTask), osThread(encoder2Task)
    };
    bool all_started = true;
    for (unsigned i = 0; i < ENCODER_COUNT; ++i) {
        encoder_instance_t *instance = &encoder_instances[i];
        if (instance->task_handle != NULL) {
            continue;
        }
        instance->output = (encoder_output_t) {0};
        instance->task_handle = osThreadCreate(definitions[i], instance);
        if (instance->task_handle == NULL) {
            /* Called before the scheduler starts, so no concurrent reader yet. */
            instance->output.state = ENCODER_STATE_ERROR;
            instance->output.error = ENCODER_ERROR_TASK;
            *instance->published = instance->output;
            all_started = false;
        }
    }
    return all_started;
}
