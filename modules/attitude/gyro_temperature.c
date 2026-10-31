/** @file gyro_temperature.c
 * @brief modules 层温漂采集实现：同一温度箱只保留一个静止均值点，再按轴做最小二乘拟合。
 */
#include "gyro_temperature.h"

#include <math.h>
#include <string.h>

void gyro_temperature_start(gyro_temperature_t *c) {
    memset(c, 0, sizeof(*c));
    c->output.collecting = true;
}

void gyro_temperature_collect(gyro_temperature_t *c, bool stationary,
                               float temperature, const float gyro[3],
                               float bin_c, uint32_t samples_per_point, uint32_t max_points) {
    if (!c->output.collecting) return;
    if (!stationary || !isfinite(temperature) || !isfinite(bin_c) || bin_c <= 0.0f ||
        samples_per_point == 0U || max_points == 0U || !isfinite(gyro[0]) ||
        !isfinite(gyro[1]) || !isfinite(gyro[2])) {
        c->output.pending_samples = 0;
        return;
    }
    /* 用温度箱去重，防止长时间停留在一个温度时产生大量重复拟合点。 */
    const int32_t bin = (int32_t)floorf(temperature / bin_c);
    if (c->output.points >= max_points || c->output.points >= GYRO_TEMPERATURE_MAX_POINTS) return;
    for (unsigned i = 0; i < c->output.points; ++i) {
        if (c->point[i].bin == bin) {
            c->output.pending_samples = 0;
            return;
        }
    }
    if (c->output.pending_samples == 0 || bin != c->pending_bin) {
        c->output.pending_samples = 0;
        c->pending_bin = bin;
        c->temperature_mean = 0.0f;
        memset(c->bias_mean, 0, sizeof(c->bias_mean));
    }
    const float n = (float)++c->output.pending_samples;
    c->temperature_mean += (temperature - c->temperature_mean) / n;
    for (unsigned i = 0; i < 3; ++i) {
        c->bias_mean[i] += (gyro[i] - c->bias_mean[i]) / n;
    }
    if (c->output.pending_samples < samples_per_point) return;
    gyro_temperature_point_t *p = &c->point[c->output.points];
    p->temperature_c = c->temperature_mean;
    p->bin = bin;
    memcpy(p->bias_dps, c->bias_mean, sizeof(p->bias_dps));
    if (c->output.points == 0 || p->temperature_c < c->output.min_c) c->output.min_c = p->temperature_c;
    if (c->output.points == 0 || p->temperature_c > c->output.max_c) c->output.max_c = p->temperature_c;
    ++c->output.points;
    c->output.pending_samples = 0;
    c->output.fit_valid = false;
    c->output.applied = false;
}

bool gyro_temperature_fit(gyro_temperature_t *c, uint32_t minimum_points, float minimum_span_c) {
    c->output.fit_valid = false;
    c->output.applied = false;
    const unsigned n = c->output.points;
    if (n < minimum_points || n < 2 || n > GYRO_TEMPERATURE_MAX_POINTS ||
        !isfinite(minimum_span_c) || minimum_span_c <= 0.0f ||
        c->output.max_c - c->output.min_c < minimum_span_c) return false;
    /* 先中心化再用双精度最小二乘，减轻温度平方相减导致的精度损失。
       仅在显式拟合命令下执行，不进入每次 500 Hz 解算的常规路径。 */
    double mean_t = 0.0, mean_b[3] = {0};
    for (unsigned j = 0; j < n; ++j) {
        mean_t += c->point[j].temperature_c / (double)n;
        for (unsigned i = 0; i < 3; ++i) mean_b[i] += c->point[j].bias_dps[i] / (double)n;
    }
    double tt = 0.0, tb[3] = {0};
    for (unsigned j = 0; j < n; ++j) {
        const double dt = c->point[j].temperature_c - mean_t;
        tt += dt * dt;
        for (unsigned i = 0; i < 3; ++i) tb[i] += dt * (c->point[j].bias_dps[i] - mean_b[i]);
    }
    if (tt < 1.0e-6) return false;
    for (unsigned i = 0; i < 3; ++i) {
        c->output.slope[i] = (float)(tb[i] / tt);
        c->output.intercept[i] = (float)(mean_b[i] - tb[i] / tt * mean_t);
        double squared = 0.0;
        for (unsigned j = 0; j < n; ++j) {
            const double residual = c->point[j].bias_dps[i] - mean_b[i] -
                tb[i] / tt * (c->point[j].temperature_c - mean_t);
            squared += residual * residual;
        }
        c->output.rms_dps[i] = (float)sqrt(squared / (double)n);
        if (!isfinite(c->output.slope[i]) || !isfinite(c->output.intercept[i]) ||
            !isfinite(c->output.rms_dps[i])) return false;
    }
    c->output.collecting = false;
    c->output.fit_valid = true;
    return true;
}
