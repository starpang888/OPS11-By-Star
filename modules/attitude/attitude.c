#include "attitude.h"

#include <math.h>
#include <stddef.h>

static FusionVector vector(const float xyz[3]) {
    return (FusionVector) {.array = {xyz[0], xyz[1], xyz[2]}};
}

static bool finite_vector(const float xyz[3]) {
    return isfinite(xyz[0]) && isfinite(xyz[1]) && isfinite(xyz[2]);
}

static void initialise_bias(attitude_t *attitude, float sample_rate_hz, FusionVector offset) {
    FusionBiasInitialise(&attitude->bias);
    const FusionBiasSettings settings = {
        .sampleRate = sample_rate_hz,
        .stationaryThreshold = 0.05f, /* After startup calibration; much stricter than upstream 3 dps. */
        .stationaryPeriod = 3.0f
    };
    FusionBiasSetSettings(&attitude->bias, &settings);
    FusionBiasSetOffset(&attitude->bias, offset);
}

bool attitude_initialise(attitude_t *attitude, float sample_rate_hz,
                         const float initial_bias_dps[3], const float gravity_g[3]) {
    if ((attitude == NULL) || (initial_bias_dps == NULL) || (gravity_g == NULL) ||
        !isfinite(sample_rate_hz) || (sample_rate_hz < 1.0f) ||
        !finite_vector(initial_bias_dps) || !finite_vector(gravity_g)) {
        return false;
    }
    const float norm = FusionVectorNorm(vector(gravity_g));
    if ((norm < 0.5f) || (norm > 1.5f)) {
        return false;
    }
    attitude->sample_period_s = 1.0f / sample_rate_hz;
    attitude->bias_tracking = false;
    initialise_bias(attitude, sample_rate_hz, vector(initial_bias_dps));
    FusionAhrsInitialise(&attitude->ahrs);
    const FusionAhrsSettings settings = {
        .sampleRate = sample_rate_hz,
        .convention = FusionConventionNwu, /* World +Z is up; heading is relative, not geographic north. */
        .gain = 0.5f,
        .gyroscopeRange = 450.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 0.0f,
        .rejectionTimeout = 5.0f
    };
    FusionAhrsSetSettings(&attitude->ahrs, &settings);

    /* Initialise inclination from the stationary mean; heading is defined as zero. */
    const float roll = atan2f(gravity_g[1], gravity_g[2]);
    const float pitch = atan2f(-gravity_g[0], sqrtf(gravity_g[1] * gravity_g[1] + gravity_g[2] * gravity_g[2]));
    const float sr = sinf(0.5f * roll), cr = cosf(0.5f * roll);
    const float sp = sinf(0.5f * pitch), cp = cosf(0.5f * pitch);
    const FusionQuaternion quaternion = {.array = {cr * cp, sr * cp, cr * sp, -sr * sp}};
    FusionAhrsSetQuaternion(&attitude->ahrs, quaternion);
    FusionAhrsSkipStartup(&attitude->ahrs);
    return true;
}

bool attitude_update(attitude_t *attitude, const float gyroscope_dps[3],
                     const float accelerometer_g[3], attitude_output_t *output) {
    if ((attitude == NULL) || (gyroscope_dps == NULL) || (accelerometer_g == NULL) ||
        (output == NULL) || !finite_vector(gyroscope_dps) || !finite_vector(accelerometer_g)) {
        return false;
    }
    const FusionVector gyro = vector(gyroscope_dps);
    const FusionVector accel = vector(accelerometer_g);
    FusionVector corrected;
    /* Do not learn a bias when acceleration itself indicates motion/freefall. */
    const float accel_norm = FusionVectorNorm(accel);
    const bool bias_allowed = fabsf(accel_norm - 1.0f) < 0.03f;
    if (!bias_allowed && attitude->bias_tracking) {
        initialise_bias(attitude, 1.0f / attitude->sample_period_s, FusionBiasGetOffset(&attitude->bias));
    }
    attitude->bias_tracking = bias_allowed;
    if (bias_allowed) {
        corrected = FusionBiasUpdate(&attitude->bias, gyro);
    } else {
        corrected = FusionVectorSubtract(gyro, FusionBiasGetOffset(&attitude->bias));
    }

    /* Six-axis update with a zero magnetometer. Unlike UpdateNoMagnetometer(), this
       preserves relative yaw if gyro overrange triggers the upstream startup recovery. */
    FusionAhrsUpdate(&attitude->ahrs, corrected, accel, FUSION_VECTOR_ZERO);
    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&attitude->ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);
    const FusionVector bias = FusionBiasGetOffset(&attitude->bias);
    const FusionAhrsFlags flags = FusionAhrsGetFlags(&attitude->ahrs);
    const FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&attitude->ahrs);
    for (size_t i = 0; i < 4; ++i) {
        if (!isfinite(quaternion.array[i])) {
            return false;
        }
        output->quaternion[i] = quaternion.array[i];
    }
    output->roll_deg = euler.angle.roll;
    output->pitch_deg = euler.angle.pitch;
    output->yaw_deg = euler.angle.yaw;
    for (size_t i = 0; i < 3; ++i) {
        output->gyroscope_dps[i] = corrected.array[i];
        output->bias_dps[i] = bias.array[i];
    }
    output->accelerometer_ignored = states.accelerometerIgnored;
    output->gyroscope_overrange = flags.overrangeRecovery;
    output->recovering = flags.startup || flags.overrangeRecovery || flags.accelerationRecovery;
    return isfinite(output->roll_deg) && isfinite(output->pitch_deg) && isfinite(output->yaw_deg);
}
