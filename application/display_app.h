/** @file display_app.h
 * @brief application 层 OLED 显示任务接口，g_display 用于观察连接及刷新状态。
 */
#ifndef DISPLAY_APP_H
#define DISPLAY_APP_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DISPLAY_ERROR_NONE = 0,
    DISPLAY_ERROR_TASK,
    DISPLAY_ERROR_CONFIG,
    DISPLAY_ERROR_NOT_FOUND,
    DISPLAY_ERROR_IO
} display_error_t;
typedef struct {
    bool ready;
    bool pose_valid;
    uint8_t address_7bit;
    display_error_t error;
    uint32_t frames;
    uint32_t io_errors;
    uint32_t last_i2c_error;
    uint32_t sample_timestamp_us;
    uint32_t max_refresh_ms;
} display_output_t;
extern volatile display_output_t g_display;
/* 外设初始化后、osKernelStart 前调用；显示故障不会停止传感器或串口任务。 */
bool display_app_start(void);
#endif
