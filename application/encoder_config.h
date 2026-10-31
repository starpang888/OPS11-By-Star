/** @file encoder_config.h
 * @brief application 层双编码器参数：采样周期、测速窗口、静止门限及开发诊断开关。
 */
#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

/* 每只编码器一个独立任务；共用 1 ms 采样周期，各路标称 1 kHz。 */
#define ENCODER_SAMPLE_PERIOD_MS 1U
#define ENCODER_STARTUP_DELAY_MS 10U
#define ENCODER_STALE_AFTER_MS 10U
#define ENCODER_TASK_STACK_WORDS 512U
/* 开发诊断开关：收尾配置默认关闭；启动后仅做角度、报警和 CRC 采样。
   排查协议或器件配置时才改为 1，并重新编译。 */
#ifndef ENCODER_STARTUP_REGISTER_CHECK_ENABLED
#define ENCODER_STARTUP_REGISTER_CHECK_ENABLED 0
#endif
#ifndef ENCODER_RUNTIME_DIAGNOSTICS_ENABLED
#define ENCODER_RUNTIME_DIAGNOSTICS_ENABLED 0
#endif
#define ENCODER_DIAGNOSTICS_PERIOD_MS 1000U
/* 运动计算使用毫秒时间；转速由至少 10 ms 窗口内的实际位移求平均。 */
#define ENCODER_MOTION_MAX_GAP_MS 5U
#define ENCODER_SPEED_WINDOW_MS 10U
#define ENCODER_SPEED_FILTER_TAU_MS 20U
/* 论文 AS5045B 为 4096 计数/圈，MT6826S 分辨率为其 8 倍，故门限对应放大 8 倍。
   静止判断检查真实 5 ms 端点位移及至少 100 ms 的计数范围，不直接套用每 1 ms 差值。 */
#define ENCODER_STATIONARY_STEP_COUNTS 8U
#define ENCODER_STATIONARY_RANGE_COUNTS 40U
#define ENCODER_STATIONARY_STALE_US 5000U
/* 普通读与 Burst 读取时刻不同，读法对比应在轮子静止时判断；容差单位为度。 */
#define ENCODER_REGISTER_CHECK_TOLERANCE_DEG 1.0f
/* 启动时一次性确认/设置已选定的 BW=5，只改运行寄存器，不烧写 EEPROM。 */
#define ENCODER_BANDWIDTH_CHANGE_ENABLED 1U
#define ENCODER_BANDWIDTH_TARGET 5U

#endif
