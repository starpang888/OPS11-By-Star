/** @file telemetry_config.h
 * @brief application 层串口发送参数：10 ms 一帧即标称 100 Hz，任务栈以 word 为单位。
 */
#ifndef TELEMETRY_CONFIG_H
#define TELEMETRY_CONFIG_H
#define TELEMETRY_PERIOD_MS 10U
#define TELEMETRY_TASK_STACK_WORDS 512U
#endif
