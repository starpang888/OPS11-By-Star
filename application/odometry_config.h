#ifndef ODOMETRY_CONFIG_H
#define ODOMETRY_CONFIG_H
/* Top view: +X forward, +Y left, positive yaw counterclockwise.
   Wheel 1: SPI1/PA4, front/back. Wheel 2: SPI2/PB12, left/right. */
#define ODOMETRY_WHEEL_1_DIAMETER_MM 58.0
#define ODOMETRY_WHEEL_2_DIAMETER_MM 58.0
#define ODOMETRY_WHEEL_1_ANGLE_DEG 0.0
#define ODOMETRY_WHEEL_2_ANGLE_DEG 90.0
/* Provisional: verify that forward/left travel produces positive counts. */
#define ODOMETRY_WHEEL_1_DIRECTION 1
#define ODOMETRY_WHEEL_2_DIRECTION 1
#define ODOMETRY_WHEEL_1_GEAR_RATIO 1.0
#define ODOMETRY_WHEEL_2_GEAR_RATIO 1.0
/* Intersection of ROLLING direction lines, relative to chassis centre. */
#define ODOMETRY_INTERSECTION_X_MM (-20.0)
#define ODOMETRY_INTERSECTION_Y_MM 20.0
#define ODOMETRY_ENCODER_HISTORY 32U
#define ODOMETRY_IMU_QUEUE 16U
#define ODOMETRY_ENCODER_MAX_BRACKET_US 4000U
#define ODOMETRY_IMU_MAX_INTERVAL_US 6000U
#define ODOMETRY_SYNC_TIMEOUT_US 8000U
#define ODOMETRY_STALE_AFTER_US 10000U
#define ODOMETRY_TASK_STACK_WORDS 768U
#endif
