/** @file odometry.h
 * @brief modules 层双随动轮定位几何：滚动方向矩阵、中心偏置补偿和中点航向积分。
 *        坐标为毫米，输入航向为度；+X 向前、+Y 向左、逆时针为正。
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double wheel_diameter_mm[2]; /* 两轮有效滚动直径，毫米。 */
    double counts_per_revolution;
    double encoder_revolutions_per_wheel_revolution[2];
    int direction[2]; /* 编码器计数正方向修正，只能取 +1 或 -1。 */
    double rolling_angle_deg[2]; /* 两轮滚动正方向相对机体 +X 的角度，单位度。 */
    double intersection_x_mm, intersection_y_mm; /* 两条滚动直线交点相对底盘中心的偏置，毫米。 */
} odometry_config_t;

/* 整数累计计数与不足一个计数的插值余量分开保存，避免长时间累计后损失细分精度。 */
typedef struct {
    int64_t counts;
    double fraction;
} odometry_position_t;

typedef struct {
    odometry_config_t config;
    double inverse[2][2], mm_per_count[2];
    odometry_position_t previous[2];
    double previous_yaw_rad;
    double x_mm, y_mm, yaw_rad;
    bool initialized;
} odometry_t;

/* 纯几何算法，无 HAL/RTOS 依赖；第一次更新把底盘中心设为零点。
   后续更新必须使用连续轨迹上的公共采样端点，由应用层保证同步和有效性。 */
bool odometry_init(odometry_t *odometry, const odometry_config_t *config);
bool odometry_update(odometry_t *odometry, const odometry_position_t positions[2], double yaw_deg);
double odometry_yaw_deg(const odometry_t *odometry);
#endif
