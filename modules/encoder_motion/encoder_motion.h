#ifndef ENCODER_MOTION_H
#define ENCODER_MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t counts_per_revolution;
    uint32_t max_gap_ms;
    uint32_t speed_window_ms;
    uint32_t speed_filter_tau_ms; /* 0 disables speed filtering. */
} encoder_motion_config_t;

typedef struct {
    bool initialized;
    bool tracking_continuous; /* Latched false after a gap until reset. */
    bool valid; /* Current position is continuous from the first sample. */
    bool speed_valid; /* A complete window of consecutive healthy samples exists. */
    int64_t total_counts; /* Exact signed displacement from first healthy sample. */
    int64_t turns; /* Completed net revolutions, truncated toward zero. */
    double total_angle_deg; /* Signed displacement, not absolute rotor angle. */
    float speed_raw_rpm; /* Window average before filtering. */
    float speed_rpm; /* Filtered window average, positive for increasing raw angle. */
    uint32_t discontinuities; /* Lost segments, rather than number of bad frames. */
    uint32_t speed_updates;
} encoder_motion_output_t;

/* One instance per encoder, owned by its sampling task. No HAL/RTOS dependency. */
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
/* Call for every healthy periodic sample. Assumes actual travel is strictly less
   than half a revolution between samples; larger travel can alias undetectably. */
void encoder_motion_update(encoder_motion_t *motion, uint16_t raw, uint32_t timestamp_ms);
/* Call for every rejected sample. Keeps accumulated displacement, drops speed
   history and never integrates across an unknown interval. */
void encoder_motion_invalidate(encoder_motion_t *motion);

#endif
