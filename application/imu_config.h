#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* The G354 driver configures 500 Hz. Keep the unit still during startup calibration. */
#define IMU_SETTLING_SAMPLES 50U
#define IMU_CALIBRATION_SAMPLES 1000U
#define IMU_CALIBRATION_GYRO_LIMIT_DPS 0.5f
/* At 500 Hz / 4 taps, observed stationary gyro std is about 0.031 dps.
   Allow noise margin while retaining the mean-rate and acceleration checks. */
#define IMU_CALIBRATION_GYRO_STD_DPS 0.05f
#define IMU_CALIBRATION_ACCEL_TOLERANCE_G 0.05f
#define IMU_CALIBRATION_ACCEL_STD_G 0.01f
#define IMU_DRDY_TIMEOUT_MS 100U
#define IMU_MAX_CONSECUTIVE_ERRORS 10U

/* Default body frame = the right-handed sensor frame in datasheet Figure 3.1.
   Change this permutation/sign mapping to match the actual mounting. The same
   proper rotation must be applied to both gyro and accelerometer vectors. */
#define IMU_BODY_X_AXIS 0
#define IMU_BODY_Y_AXIS 1
#define IMU_BODY_Z_AXIS 2
#define IMU_BODY_X_SIGN 1
#define IMU_BODY_Y_SIGN 1
#define IMU_BODY_Z_SIGN 1

#endif
