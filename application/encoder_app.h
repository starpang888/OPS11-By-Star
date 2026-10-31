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
    uint32_t sample_rate_hz; /* Configured nominal frequency. */
    float measured_read_rate_hz; /* Actual periodic attempts per second, updated about once a second. */
    uint32_t last_attempt_ms;
    uint32_t timestamp_ms; /* Latch request time of the last healthy sample, HAL tick. */
    uint32_t timestamp_us; /* Same latch request, shared TIM2, wraps at 2^32 us. */
    uint32_t sample_interval_us; /* Interval between healthy acquisitions. */
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
    encoder_register_check_t register_check;
    bool bandwidth_change_evaluated; /* One attempt after communication checks pass. */
    mt6826s_status_t bandwidth_change_status; /* Inspect only when evaluated is true. */
    mt6826s_bandwidth_change_t bandwidth_change;
    encoder_motion_output_t motion;
    bsp_mt6826s_dma_stats_t dma;
} encoder_output_t;

extern volatile encoder_output_t g_encoder;
extern volatile encoder_output_t g_encoder2;

/* Starts both tasks before osKernelStart, after peripheral initialisation.
   Returns true if both were created; a successful task survives failure of the other. */
bool encoder_app_start(void);
/* Task context only. Copies coherently and checks the age of the sample. */
bool encoder_app_get_output(encoder_output_t *output);
/* Select either encoder. The original getter above selects ENCODER_1. */
bool encoder_app_get_output_for(encoder_id_t id, encoder_output_t *output);
/* Task context only. Queues a new displacement origin at the next healthy
   sample; does not change the encoder's hardware zero or angle_deg. */
bool encoder_app_reset_motion(encoder_id_t id);

#endif
