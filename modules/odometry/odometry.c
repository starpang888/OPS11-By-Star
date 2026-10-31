/** @file odometry.c
 * @brief modules 层平面位姿积分。累计计数差转轮子位移，补偿安装偏置后转入初始世界坐标。
 */
#include "odometry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#define ODOMETRY_PI 3.14159265358979323846

static double wrap(double radians) {
    return remainder(radians, 2.0 * ODOMETRY_PI);
}

bool odometry_init(odometry_t *o, const odometry_config_t *c) {
    if (o == NULL || c == NULL || !isfinite(c->counts_per_revolution) ||
        c->counts_per_revolution <= 0 || !isfinite(c->intersection_x_mm) ||
        !isfinite(c->intersection_y_mm)) {
        return false;
    }
    double u[2][2], scale[2];
    for (unsigned i = 0; i < 2; ++i) {
        if (!isfinite(c->wheel_diameter_mm[i]) || c->wheel_diameter_mm[i] <= 0 ||
            !isfinite(c->encoder_revolutions_per_wheel_revolution[i]) ||
            c->encoder_revolutions_per_wheel_revolution[i] <= 0 ||
            !isfinite(c->rolling_angle_deg[i]) ||
            (c->direction[i] != 1 && c->direction[i] != -1)) {
            return false;
        }
        const double angle = c->rolling_angle_deg[i] * ODOMETRY_PI / 180.0;
        u[i][0] = cos(angle);
        u[i][1] = sin(angle);
        scale[i] = c->direction[i] * ODOMETRY_PI * c->wheel_diameter_mm[i] /
            (c->counts_per_revolution * c->encoder_revolutions_per_wheel_revolution[i]);
        if (!isfinite(scale[i]) || scale[i] == 0) {
            return false;
        }
    }
    const double determinant = u[0][0] * u[1][1] - u[0][1] * u[1][0];
    if (fabs(determinant) < 0.1) { /* 拒绝接近平行的滚动方向，避免矩阵求逆放大误差。 */
        return false;
    }
    memset(o, 0, sizeof(*o));
    o->config = *c;
    o->inverse[0][0] = u[1][1] / determinant;
    o->inverse[0][1] = -u[0][1] / determinant;
    o->inverse[1][0] = -u[1][0] / determinant;
    o->inverse[1][1] = u[0][0] / determinant;
    memcpy(o->mm_per_count, scale, sizeof(scale));
    return true;
}

bool odometry_update(odometry_t *o, const odometry_position_t p[2], double yaw_deg) {
    if (o == NULL || p == NULL || !isfinite(yaw_deg) ||
        !isfinite(p[0].fraction) || !isfinite(p[1].fraction)) {
        return false;
    }
    const double yaw = wrap(yaw_deg * ODOMETRY_PI / 180.0);
    if (!o->initialized) {
        memcpy(o->previous, p, sizeof(o->previous));
        o->previous_yaw_rad = yaw;
        o->initialized = true;
        return true;
    }
    double distance[2];
    for (unsigned i = 0; i < 2; ++i) {
        /* 相邻更新的整数锚点差不能超过一圈；先算无符号幅值，也避免更换原点时
           直接进行有符号减法引起溢出。 */
        const uint64_t magnitude = p[i].counts >= o->previous[i].counts ?
            (uint64_t)p[i].counts - (uint64_t)o->previous[i].counts :
            (uint64_t)o->previous[i].counts - (uint64_t)p[i].counts;
        if ((double)magnitude > o->config.counts_per_revolution) {
            return false;
        }
        const double delta = p[i].counts >= o->previous[i].counts ?
            (double)magnitude : -(double)magnitude;
        distance[i] = (delta + p[i].fraction - o->previous[i].fraction) * o->mm_per_count[i];
    }
    const double dtheta = wrap(yaw - o->previous_yaw_rad);
    /* 交点 C 的平移换算到底盘中心 O：dx_O=dx_C+cy*dtheta，dy_O=dy_C-cx*dtheta。 */
    const double dx = o->inverse[0][0] * distance[0] + o->inverse[0][1] * distance[1] +
        o->config.intersection_y_mm * dtheta;
    const double dy = o->inverse[1][0] * distance[0] + o->inverse[1][1] * distance[1] -
        o->config.intersection_x_mm * dtheta;
    /* 用区间中点航向将机体系位移旋转到开机世界坐标系。 */
    const double midpoint = o->yaw_rad + 0.5 * dtheta;
    const double x = o->x_mm + cos(midpoint) * dx - sin(midpoint) * dy;
    const double y = o->y_mm + sin(midpoint) * dx + cos(midpoint) * dy;
    if (!isfinite(x) || !isfinite(y)) {
        return false;
    }
    o->x_mm = x;
    o->y_mm = y;
    o->yaw_rad = wrap(o->yaw_rad + dtheta);
    o->previous_yaw_rad = yaw;
    memcpy(o->previous, p, sizeof(o->previous));
    return true;
}

double odometry_yaw_deg(const odometry_t *o) {
    return o->yaw_rad * 180.0 / ODOMETRY_PI;
}
