#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

/* Each encoder has its own task; both use this period (nominally 1 kHz each). */
#define ENCODER_SAMPLE_PERIOD_MS 1U
#define ENCODER_STARTUP_DELAY_MS 10U
#define ENCODER_STALE_AFTER_MS 10U
#define ENCODER_TASK_STACK_WORDS 512U
#define ENCODER_DIAGNOSTICS_PERIOD_MS 1000U
/* Millisecond timestamps; speed averages actual displacement over >=10 ms. */
#define ENCODER_MOTION_MAX_GAP_MS 5U
#define ENCODER_SPEED_WINDOW_MS 10U
#define ENCODER_SPEED_FILTER_TAU_MS 20U
/* Ordinary and burst reads occur at different times. Assess agreement while stationary. */
#define ENCODER_REGISTER_CHECK_TOLERANCE_DEG 1.0f
/* Temporary BW=0 vs BW=5 comparison; applies once per MCU boot, no EEPROM burn. */
#define ENCODER_BANDWIDTH_CHANGE_ENABLED 1U
#define ENCODER_BANDWIDTH_TARGET 5U

#endif
