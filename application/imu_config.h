/** @file imu_config.h
 * @brief application 层 IMU 参数：开机标定、静止判断、零偏更新、死区及安装轴映射。
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* G354 输出 500 Hz；启动时保持设备静止，先丢弃过渡样本，再收集 1000 点约 2 秒标定。 */
#define IMU_SETTLING_SAMPLES 50U
#define IMU_CALIBRATION_SAMPLES 1000U
#define IMU_CALIBRATION_GYRO_LIMIT_DPS 0.5f
/* 500 Hz、4 点平均下实测静止角速度标准差约 0.031 度/秒，门限留出噪声余量。
   同时保留角速度幅值、加速度模长和方差检查，防止运动中建立零偏。 */
#define IMU_CALIBRATION_GYRO_STD_DPS 0.05f
#define IMU_CALIBRATION_ACCEL_TOLERANCE_G 0.05f
#define IMU_CALIBRATION_ACCEL_STD_G 0.01f
#define IMU_DRDY_TIMEOUT_MS 100U
#define IMU_MAX_CONSECUTIVE_ERRORS 10U

/* 论文第 3/4 章方法：角速度自适应 KF、联合静止检测、Heun RK2。
   静止判断使用死区前角速度，不能让死区掩盖真实低速运动。 */
#define IMU_STATIONARY_THRESHOLD_DPS 0.04f
#define IMU_STATIONARY_PERIOD_S 0.1f
#define IMU_STATIONARY_ACCEL_TOLERANCE_G 0.03f
/* 将论文未明确单位的 0.0001 步长按 200 Hz 下每次 0.0001 度/秒解释，
   换算成每秒最多调整 0.02 度/秒，再按本工程的 500 Hz 周期缩放。 */
#define IMU_BIAS_SLEW_DPS_PER_S 0.02f
/* 用户选定的暂用死区，待有转台再标定；三轴分别应用，低于 0.3 度/秒的真实转动也会被抑制。 */
#define IMU_GYRO_DEADBAND_DPS 0.3f
#define IMU_GYRO_LIMIT_DPS 450.0f

/* 温漂系数必须由本机 G354 实测得到，默认禁用，不套用论文中其他芯片的系数。
   斜率单位为 (度/秒)/摄氏度；截距已被启动零偏抵消。 */
#define IMU_TEMPERATURE_MODEL_VALID 0
#define IMU_TEMPERATURE_SLOPE_X 0.0f
#define IMU_TEMPERATURE_SLOPE_Y 0.0f
#define IMU_TEMPERATURE_SLOPE_Z 0.0f
#define IMU_TEMPERATURE_MIN_C 0.0f
#define IMU_TEMPERATURE_MAX_C 0.0f

/* 温漂标定仅在显式命令开启后采集静止样本；箱宽为摄氏度，样本数按 500 Hz 计。 */
#define IMU_TEMPERATURE_BIN_C 0.5f
#define IMU_TEMPERATURE_POINTS_MAX 32U
#define IMU_TEMPERATURE_POINT_SAMPLES 500U
#define IMU_TEMPERATURE_FIT_MIN_POINTS 7U
#define IMU_TEMPERATURE_FIT_MIN_SPAN_C 3.0f

/* 默认机体系与手册图 3.1 的传感器右手系一致。
   实际安装不同时修改轴序和符号，角速度与加速度必须使用同一右手坐标变换。 */
#define IMU_BODY_X_AXIS 0
#define IMU_BODY_Y_AXIS 1
#define IMU_BODY_Z_AXIS 2
#define IMU_BODY_X_SIGN 1
#define IMU_BODY_Y_SIGN 1
#define IMU_BODY_Z_SIGN 1

#endif
