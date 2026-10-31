/** @file attitude.c
 * @brief modules 层论文姿态算法。先滤波与补偿，再联合判断静止，最后应用死区并积分。
 *        航向以开机姿态为参考，无绝对北向观测；加速度不参与姿态反馈校正。
 */
#include "attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232088f

static bool finite_vector(const float xyz[3]) {
    return isfinite(xyz[0]) && isfinite(xyz[1]) && isfinite(xyz[2]);
}

static float clamp(float value, float minimum, float maximum) {
    return fminf(fmaxf(value, minimum), maximum);
}

bool attitude_initialise(attitude_t *a, const attitude_config_t *c,
                         const float bias[3], const float variance[3], float temperature) {
    if (!a || !c || !bias || !variance || !finite_vector(bias) || !finite_vector(variance) ||
        !isfinite(temperature) || !isfinite(c->sample_rate_hz) || c->sample_rate_hz < 1.0f ||
        !isfinite(c->gyro_limit_dps) || c->gyro_limit_dps <= 0.0f ||
        !isfinite(c->stationary_threshold_dps) || c->stationary_threshold_dps <= 0.0f ||
        !isfinite(c->stationary_period_s) || c->stationary_period_s <= 0.0f ||
        !isfinite(c->accel_tolerance_g) || c->accel_tolerance_g <= 0.0f ||
        !isfinite(c->bias_slew_dps_per_s) || c->bias_slew_dps_per_s < 0.0f ||
        !isfinite(c->deadband_dps) || c->deadband_dps < 0.0f ||
        !finite_vector(c->temperature_slope) || !isfinite(c->temperature_min_c) ||
        !isfinite(c->temperature_max_c) ||
        (c->temperature_model_valid && c->temperature_min_c >= c->temperature_max_c)) {
        return false;
    }
    for (unsigned i = 0; i < 3; ++i) {
        if (variance[i] < 0.0f) {
            return false;
        }
    }
    memset(a, 0, sizeof(*a));
    a->config = *c;
    a->initial_temperature_c = temperature;
    a->quaternion[0] = 1.0f; /* 论文式 4.4：初始机体系与导航系重合。 */
    for (unsigned i = 0; i < 3; ++i) {
        a->initial_bias[i] = bias[i];
        attitude_kalman_t *f = &a->filter[i];
        f->x = bias[i];
        f->r = fmaxf(variance[i], 1.0e-8f);
        f->p = f->r;
        f->q = f->r;
    }
    return true;
}

static float kalman_update(attitude_kalman_t *f, float measured, float *gain) {
    /* 对应论文式 3.5～3.12：预测、计算增益、修正，再调整下一次使用的 Q。 */
    const float prediction_p = f->p + f->q;
    const float innovation = measured - f->x;
    *gain = prediction_p / (prediction_p + f->r);
    f->x += *gain * innovation;
    f->p = fmaxf((1.0f - *gain) * prediction_p, f->r * 1.0e-6f);
    f->innovations[f->head] = innovation * innovation;
    f->head = (f->head + 1U) % ATTITUDE_INNOVATIONS;
    if (f->size < ATTITUDE_INNOVATIONS) {
        ++f->size;
    }
    float power = 0.0f;
    for (unsigned i = 0; i < f->size; ++i) {
        power += f->innovations[i];
    }
    /* 为 Q 保留正下限，避免滤波增益长期趋零；此处不承担运动分类功能。 */
    f->q = clamp(*gain * *gain * power / (float)f->size, f->r * 0.01f, 1.0e6f);
    return f->x;
}

static float temperature_bias(const attitude_t *a, unsigned axis, float temperature) {
    if (!a->config.temperature_model_valid) {
        return 0.0f;
    }
    /* 只在实测温度范围内使用模型，不向范围外外推。
       启动零偏已包含初始温度的偏置，因此只补偿相对初始温度的变化。 */
    const float current = clamp(temperature, a->config.temperature_min_c, a->config.temperature_max_c);
    const float initial = clamp(a->initial_temperature_c,
                                a->config.temperature_min_c, a->config.temperature_max_c);
    return a->config.temperature_slope[axis] * (current - initial);
}

bool attitude_set_temperature_model(attitude_t *a, const float slope[3],
                                    float min_c, float max_c, float temperature_c) {
    if (!a || !slope || !finite_vector(slope) || !isfinite(min_c) || !isfinite(max_c) ||
        min_c >= max_c || !isfinite(temperature_c)) {
        return false;
    }
    float old[3];
    for (unsigned i = 0; i < 3; ++i) {
        old[i] = temperature_bias(a, i, temperature_c);
    }
    a->config.temperature_model_valid = true;
    a->config.temperature_min_c = min_c;
    a->config.temperature_max_c = max_c;
    memcpy(a->config.temperature_slope, slope, sizeof(a->config.temperature_slope));
    for (unsigned i = 0; i < 3; ++i) {
        a->residual_bias[i] += old[i] - temperature_bias(a, i, temperature_c);
    }
    return true;
}

static void derivative(const float q[4], const float w[3], float dq[4]) {
    /* Hamilton 四元数导数：q_dot = 0.5 * q ⊗ [0,wx,wy,wz]，机体系转导航系。 */
    dq[0] = 0.5f * (-q[1]*w[0] - q[2]*w[1] - q[3]*w[2]);
    dq[1] = 0.5f * ( q[0]*w[0] + q[2]*w[2] - q[3]*w[1]);
    dq[2] = 0.5f * ( q[0]*w[1] - q[1]*w[2] + q[3]*w[0]);
    dq[3] = 0.5f * ( q[0]*w[2] + q[1]*w[1] - q[2]*w[0]);
}

static bool integrate(attitude_t *a, const float rate_dps[3]) {
    float rate[3];
    for (unsigned i = 0; i < 3; ++i) {
        rate[i] = rate_dps[i] * DEG_TO_RAD;
    }
    if (a->have_previous_rate) {
        const float dt = 1.0f / a->config.sample_rate_hz;
        float k1[4], k2[4], predicted[4];
        derivative(a->quaternion, a->previous_rate, k1);
        for (unsigned i = 0; i < 4; ++i) {
            predicted[i] = a->quaternion[i] + dt * k1[i];
        }
        derivative(predicted, rate, k2);
        float norm_squared = 0.0f;
        for (unsigned i = 0; i < 4; ++i) {
            /* Heun 二阶 RK：取前后斜率平均；补上论文式 4.9 缺少的 1/2 系数。 */
            a->quaternion[i] += 0.5f * dt * (k1[i] + k2[i]);
            norm_squared += a->quaternion[i] * a->quaternion[i];
        }
        if (!isfinite(norm_squared) || norm_squared < 1.0e-12f) {
            return false;
        }
        const float inverse_norm = 1.0f / sqrtf(norm_squared);
        for (unsigned i = 0; i < 4; ++i) {
            a->quaternion[i] *= inverse_norm;
        }
    }
    memcpy(a->previous_rate, rate, sizeof(rate));
    a->have_previous_rate = true;
    return true;
}

void attitude_discard_history(attitude_t *a) {
    a->have_previous_rate = false;
    a->prime_filter = true;
    a->stationary_elapsed_s = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
        a->filter[i].size = 0;
        a->filter[i].head = 0;
        a->filter[i].p = a->filter[i].r;
        a->filter[i].q = a->filter[i].r;
    }
}

bool attitude_update(attitude_t *a, const float gyroscope_dps[3],
                     const float accelerometer_g[3], float temperature_c,
                     uint8_t wheel_block_mask, attitude_output_t *o) {
    if (!a || !gyroscope_dps || !accelerometer_g || !o ||
        !finite_vector(gyroscope_dps) || !finite_vector(accelerometer_g) ||
        !isfinite(temperature_c) || a->config.sample_rate_hz < 1.0f) {
        return false;
    }
    memset(o, 0, sizeof(*o));
    o->stationary_block_mask = wheel_block_mask & 12U;
    const float dt = 1.0f / a->config.sample_rate_hz;
    float remaining[3];
    for (unsigned i = 0; i < 3; ++i) {
        if (fabsf(gyroscope_dps[i]) >= a->config.gyro_limit_dps) {
            o->gyroscope_overrange = true;
        }
        if (a->prime_filter) a->filter[i].x = gyroscope_dps[i];
        o->filtered_dps[i] = kalman_update(&a->filter[i], gyroscope_dps[i], &o->kalman_gain[i]);
        o->temperature_bias_dps[i] = temperature_bias(a, i, temperature_c);
        remaining[i] = o->filtered_dps[i] - a->initial_bias[i] - o->temperature_bias_dps[i];
        if (fabsf(remaining[i] - a->residual_bias[i]) > a->config.stationary_threshold_dps ||
            /* 同时检查原始角速度，避免 KF 滞后把刚发生的运动误判为静止。 */
            fabsf(gyroscope_dps[i] - a->initial_bias[i] - o->temperature_bias_dps[i] -
                  a->residual_bias[i]) > 0.5f) {
            o->stationary_block_mask |= 1U;
        }
    }
    a->prime_filter = false;
    const float accel_norm = sqrtf(accelerometer_g[0]*accelerometer_g[0] +
                                   accelerometer_g[1]*accelerometer_g[1] +
                                   accelerometer_g[2]*accelerometer_g[2]);
    if (fabsf(accel_norm - 1.0f) > a->config.accel_tolerance_g) {
        o->stationary_block_mask |= 2U;
    }
    if (o->gyroscope_overrange) {
        o->stationary_block_mask |= 1U;
    }
    if (o->stationary_block_mask == 0U) {
        a->stationary_elapsed_s = fminf(a->stationary_elapsed_s + dt, a->config.stationary_period_s);
    } else {
        a->stationary_elapsed_s = 0.0f;
    }
    o->stationary = a->stationary_elapsed_s >= a->config.stationary_period_s;
    /* 静止时限速学习剩余零偏；使用死区前数据，避免死区掩盖真实运动。 */
    if (o->stationary) {
        const float step = a->config.bias_slew_dps_per_s * dt;
        for (unsigned i = 0; i < 3; ++i) {
            a->residual_bias[i] += clamp(remaining[i] - a->residual_bias[i], -step, step);
        }
        ++a->bias_updates;
    }
    for (unsigned i = 0; i < 3; ++i) {
        o->residual_bias_dps[i] = a->residual_bias[i];
        o->bias_dps[i] = a->initial_bias[i] + o->temperature_bias_dps[i] + a->residual_bias[i];
        o->compensated_dps[i] = o->filtered_dps[i] - o->bias_dps[i];
        o->gyroscope_dps[i] = o->compensated_dps[i];
        if (fabsf(o->gyroscope_dps[i]) < a->config.deadband_dps) {
            o->gyroscope_dps[i] = 0.0f;
            o->deadband_mask |= (uint8_t)(1U << i);
        }
    }
    if (o->gyroscope_overrange) {
        attitude_discard_history(a);
    } else if (!integrate(a, o->gyroscope_dps)) {
        return false;
    }
    memcpy(o->quaternion, a->quaternion, sizeof(o->quaternion));
    const float w = a->quaternion[0], x = a->quaternion[1];
    const float y = a->quaternion[2], z = a->quaternion[3];
    /* 采用 R = Rz(yaw) Rx(roll) Ry(pitch)，atan2 保留完整象限。
       符号与机体转导航的定义一致，俯视逆时针航向为正。 */
    o->roll_deg = asinf(clamp(2.0f*(w*x + y*z), -1.0f, 1.0f)) * RAD_TO_DEG;
    o->pitch_deg = atan2f(2.0f*(w*y - x*z), w*w - x*x - y*y + z*z) * RAD_TO_DEG;
    o->yaw_deg = atan2f(2.0f*(w*z - x*y), w*w - x*x + y*y - z*z) * RAD_TO_DEG;
    o->stationary_elapsed_s = a->stationary_elapsed_s;
    o->bias_updates = a->bias_updates;
    o->accelerometer_ignored = true;
    o->temperature_compensated = a->config.temperature_model_valid;
    o->temperature_out_of_range = a->config.temperature_model_valid &&
        (temperature_c < a->config.temperature_min_c || temperature_c > a->config.temperature_max_c ||
         a->initial_temperature_c < a->config.temperature_min_c ||
         a->initial_temperature_c > a->config.temperature_max_c);
    return isfinite(o->roll_deg) && isfinite(o->pitch_deg) && isfinite(o->yaw_deg);
}
