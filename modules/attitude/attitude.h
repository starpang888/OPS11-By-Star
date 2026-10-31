/** @file attitude.h
 * @brief modules 层姿态接口：自适应角速度 KF、联合静止零偏修正和 RK2 四元数积分。
 *        输入角速度为度/秒、加速度为 g、温度为摄氏度；本模块不依赖 HAL/RTOS。
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#define ATTITUDE_INNOVATIONS 10U

typedef struct {
    float sample_rate_hz;
    float gyro_limit_dps;
    float stationary_threshold_dps;
    float stationary_period_s;
    float accel_tolerance_g;
    float bias_slew_dps_per_s;
    float deadband_dps; /* 角速度死区，度/秒；在静止识别及零偏学习之后应用。 */
    bool temperature_model_valid;
    float temperature_slope[3]; /* 机体系三轴温漂斜率，单位 (度/秒)/摄氏度。 */
    float temperature_min_c;
    float temperature_max_c;
} attitude_config_t;

typedef struct {
    float x, p, q, r; /* 状态估计、估计方差、过程噪声方差、测量噪声方差。 */
    float innovations[ATTITUDE_INNOVATIONS]; /* 更新前新息的平方，用于调整下一次 Q。 */
    unsigned head, size;
} attitude_kalman_t;

typedef struct {
    attitude_config_t config;
    attitude_kalman_t filter[3];
    float quaternion[4];
    float initial_bias[3], residual_bias[3];
    float initial_temperature_c;
    float previous_rate[3]; /* 上一个有效样本的角速度，弧度/秒，供 RK2 两端求斜率。 */
    float stationary_elapsed_s;
    uint32_t bias_updates;
    bool have_previous_rate;
    bool prime_filter;
} attitude_t;

typedef struct {
    float quaternion[4]; /* [w,x,y,z]，表示机体系到初始导航系的旋转。 */
    float roll_deg, pitch_deg, yaw_deg; /* Z-X-Y 右手系欧拉角，单位度；Yaw 范围 [-180,180]。 */
    float gyroscope_dps[3]; /* 死区后角速度，实际参与 RK2 积分。 */
    float filtered_dps[3]; /* 原始角速度经自适应 KF 后的结果。 */
    float compensated_dps[3]; /* 已减零偏、尚未应用死区的角速度。 */
    float bias_dps[3]; /* 总零偏 = 启动零偏 + 相对温漂 + 剩余零偏。 */
    float temperature_bias_dps[3], residual_bias_dps[3];
    float kalman_gain[3];
    float stationary_elapsed_s;
    uint32_t bias_updates;
    uint8_t stationary_block_mask; /* 静止阻塞位：角速度=1、加速度=2、轮1=4、轮2=8。 */
    uint8_t deadband_mask;
    bool stationary;
    bool temperature_compensated;
    bool temperature_out_of_range;
    bool accelerometer_ignored; /* 恒为 true：加速度仅辅助静止判断，不持续修正姿态。 */
    bool gyroscope_overrange;
    bool recovering;
} attitude_output_t;

/* 使用启动静止标定的零偏、噪声方差和温度建立初始姿态，初始四元数为单位四元数。 */
bool attitude_initialise(attitude_t *attitude, const attitude_config_t *config,
                         const float initial_bias_dps[3], const float noise_variance[3],
                         float initial_temperature_c);
/* 每个健康连续样本调用一次；wheel_block_mask 由两路编码器静止判据提供。 */
bool attitude_update(attitude_t *attitude, const float gyroscope_dps[3],
                     const float accelerometer_g[3], float temperature_c,
                     uint8_t wheel_block_mask, attitude_output_t *output);
/* 输入中断后丢弃积分/滤波历史；保留的姿态和零偏仅用于诊断，不代表轨迹连续。 */
void attitude_discard_history(attitude_t *attitude);
/* 应用实测温漂模型，并同步调整剩余零偏，避免切换模型时角速度突变。 */
bool attitude_set_temperature_model(attitude_t *attitude, const float slope[3],
                                    float min_c, float max_c, float temperature_c);

#endif
