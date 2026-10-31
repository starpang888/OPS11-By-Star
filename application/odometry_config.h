/** @file odometry_config.h
 * @brief application 层里程计参数：两轮安装几何、计数方向和公共端点同步边界。
 */
#ifndef ODOMETRY_CONFIG_H
#define ODOMETRY_CONFIG_H
/* 俯视机体坐标：+X 向前、+Y 向左、逆时针航向为正。
   轮1 SPI1/PA4 测前后，轮2 SPI2/PB12 测左右；直径单位毫米，方向角单位度。 */
#define ODOMETRY_WHEEL_1_DIAMETER_MM 58.0
#define ODOMETRY_WHEEL_2_DIAMETER_MM 58.0
#define ODOMETRY_WHEEL_1_ANGLE_DEG 0.0
#define ODOMETRY_WHEEL_2_ANGLE_DEG 90.0
/* 暂定方向符号：装车后确认向前/向左运动对应计数增加，否则将对应值改为 -1。 */
#define ODOMETRY_WHEEL_1_DIRECTION 1
#define ODOMETRY_WHEEL_2_DIRECTION 1
#define ODOMETRY_WHEEL_1_GEAR_RATIO 1.0
#define ODOMETRY_WHEEL_2_GEAR_RATIO 1.0
/* 两条滚动方向直线的交点相对底盘中心的位置，毫米：后方 20、左方 20。 */
#define ODOMETRY_INTERSECTION_X_MM (-20.0)
#define ODOMETRY_INTERSECTION_Y_MM 20.0
#define ODOMETRY_ENCODER_HISTORY 32U
#define ODOMETRY_IMU_QUEUE 16U
#define ODOMETRY_ENCODER_MAX_BRACKET_US 4000U
/* 相邻成功公共端点的最大间隔，包含跳过端点所占时间，单位微秒。
   IMU 原始采样连续性由 imu_app 独立检查；这里限制定位积分的时间跨度。 */
#define ODOMETRY_IMU_MAX_INTERVAL_US 6000U
#define ODOMETRY_SYNC_TIMEOUT_US 8000U
#define ODOMETRY_STALE_AFTER_US 10000U
#define ODOMETRY_TASK_STACK_WORDS 768U
#endif
