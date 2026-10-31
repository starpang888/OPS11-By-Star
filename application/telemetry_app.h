/** @file telemetry_app.h
 * @brief application 层位姿串口任务接口：周期获取有效性快照，编码报文后交给 BSP DMA 发送。
 */
#ifndef TELEMETRY_APP_H
#define TELEMETRY_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "bsp_telemetry.h"

typedef enum {
    TELEMETRY_ERROR_NONE = 0,
    TELEMETRY_ERROR_TASK,
    TELEMETRY_ERROR_CONFIG,
    TELEMETRY_ERROR_TRANSPORT
} telemetry_error_t;
typedef struct {
    bool running;
    telemetry_error_t error;
    uint32_t cycles, sent, skipped_busy, format_errors; /* 周期数、提交帧数、忙碌跳过数、编码错误数。 */
    uint32_t sequence, sample_timestamp_us;
    uint16_t last_frame_bytes;
    bool pose_valid;
    bsp_telemetry_status_t tx;
} telemetry_output_t;
extern volatile telemetry_output_t g_telemetry;
/* UART/DMA 初始化后、osKernelStart 前启动一次；g_telemetry 供调试器观察。 */
bool telemetry_app_start(void);
#endif
