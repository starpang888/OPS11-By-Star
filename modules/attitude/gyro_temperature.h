/** @file gyro_temperature.h
 * @brief modules 层陀螺仪温漂标定：静止温度分箱采集、三轴线性拟合，数据仅保存在 RAM。
 */
#ifndef GYRO_TEMPERATURE_H
#define GYRO_TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>

#define GYRO_TEMPERATURE_MAX_POINTS 32U
typedef struct {
    float temperature_c;
    float bias_dps[3];
    int32_t bin;
} gyro_temperature_point_t;

typedef struct {
    bool collecting;
    bool fit_valid;
    bool applied;
    uint32_t points;
    uint32_t pending_samples;
    float min_c, max_c;
    float slope[3]; /* 三轴温漂斜率，单位 (度/秒)/摄氏度。 */
    float intercept[3]; /* 拟合截距，度/秒；原始零偏 = slope*T + intercept。 */
    float rms_dps[3]; /* 三轴拟合残差均方根，度/秒。 */
} gyro_temperature_output_t;

typedef struct {
    gyro_temperature_output_t output;
    gyro_temperature_point_t point[GYRO_TEMPERATURE_MAX_POINTS];
    float temperature_mean, bias_mean[3];
    int32_t pending_bin;
} gyro_temperature_t;

/* 开始新采集会清除上一组 RAM 标定点，不写 Flash。 */
void gyro_temperature_start(gyro_temperature_t *calibration);
/* 仅在确认静止时采集滤波后的原始角速度；每个温度箱积累足量样本后生成一个点。 */
void gyro_temperature_collect(gyro_temperature_t *calibration, bool stationary,
                               float temperature_c, const float filtered_raw_dps[3],
                               float bin_c, uint32_t samples_per_point, uint32_t max_points);
/* 仅在点数和温度跨度满足要求时拟合；返回 true 仍需应用层显式启用该模型。 */
bool gyro_temperature_fit(gyro_temperature_t *calibration, uint32_t minimum_points,
                          float minimum_span_c);

#endif
