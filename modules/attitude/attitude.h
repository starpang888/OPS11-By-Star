#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "FusionAhrs.h"
#include "FusionBias.h"

typedef struct {
    FusionAhrs ahrs;
    FusionBias bias;
    float sample_period_s;
    bool bias_tracking;
} attitude_t;

typedef struct {
    float quaternion[4]; /* [w,x,y,z], body to world. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg; /* ZYX Euler yaw in [-180,180], relative to initial heading. */
    float gyroscope_dps[3];
    float bias_dps[3];
    bool accelerometer_ignored;
    bool gyroscope_overrange;
    bool recovering;
} attitude_output_t;

bool attitude_initialise(attitude_t *attitude, float sample_rate_hz,
                         const float initial_bias_dps[3], const float gravity_g[3]);
bool attitude_update(attitude_t *attitude, const float gyroscope_dps[3],
                     const float accelerometer_g[3], attitude_output_t *output);

#endif
