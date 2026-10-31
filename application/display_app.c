/** @file display_app.c
 * @brief application 层 OLED 位姿显示。低优先级读取一致快照，仅显示数据，不参与解算。
 */
#include "display_app.h"
#include "display_config.h"
#include "bsp_oled.h"
#include "odometry_app.h"
#include "oled.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile display_output_t g_display;
static osThreadId task_handle;
static oled_t screen; /* 1 KB 帧缓存放静态 RAM，不占用显示任务栈。 */
_Static_assert(pdMS_TO_TICKS(DISPLAY_PERIOD_MS) > 0U, "Display period is shorter than one tick");
_Static_assert(DISPLAY_ADDRESS_7BIT == 0U || DISPLAY_ADDRESS_7BIT < 0x80U,
               "Display uses an unshifted seven-bit address");

static void publish(const display_output_t *output) {
    taskENTER_CRITICAL();
    g_display = *output;
    taskEXIT_CRITICAL();
}

static void draw_value(unsigned row, const char *name, double value, const char *unit, bool valid) {
    char number[16];
    if (!valid || !isfinite(value)) {
        (void)snprintf(number, sizeof(number), "--");
    } else if (fabs(value) > 999999999.9) {
        (void)snprintf(number, sizeof(number), "RANGE");
    } else {
        /* 显示一位小数，不启用 printf 浮点支持；四舍五入后再拆整数与小数。 */
        const uint64_t scaled = (uint64_t)(fabs(value) * 10.0 + 0.5);
        (void)snprintf(number, sizeof(number), "%s%lu.%u",
            value < 0.0 && scaled != 0U ? "-" : "",
            (unsigned long)(scaled / 10U), (unsigned)(scaled % 10U));
    }
    /* 标签占 4 列、数值占 8 列并右对齐，剩余列显示单位；超宽提示而不截断数值。 */
    if (strlen(number) > 8U) (void)snprintf(number, sizeof(number), "RANGE");
    char label[5];
    char text[OLED_TEXT_COLUMNS + 1U];
    (void)snprintf(label, sizeof(label), "%s:", name);
    (void)snprintf(text, sizeof(text), "%-4.4s%8.8s %.3s", label, number, unit);
    oled_text_line(&screen, row, text);
}

static bool connect_screen(display_output_t *output) {
    uint8_t address = DISPLAY_ADDRESS_7BIT;
    if (address == 0U) {
        if (bsp_oled_probe(0x3CU)) address = 0x3CU;
        else if (bsp_oled_probe(0x3DU)) address = 0x3DU;
    } else if (!bsp_oled_probe(address)) {
        address = 0U;
    }
    output->address_7bit = address;
    if (address == 0U) {
        output->error = DISPLAY_ERROR_NOT_FOUND;
        output->last_i2c_error = bsp_oled_i2c_error();
        return false;
    }
    bsp_oled_select_address(address);
    const oled_bus_t bus = {NULL, bsp_oled_write, bsp_oled_yield};
    if (!oled_initialise(&screen, &bus, DISPLAY_CONTROLLER)) {
        output->error = DISPLAY_ERROR_IO;
        output->last_i2c_error = bsp_oled_i2c_error();
        ++output->io_errors;
        return false;
    }
    output->error = DISPLAY_ERROR_NONE;
    return true;
}

static void display_task(void const *argument) {
    (void)argument;
    display_output_t output = {0};
    if (!bsp_oled_initialise()) {
        output.error = DISPLAY_ERROR_CONFIG;
        publish(&output);
        vTaskSuspend(NULL);
    }
    /* 等待模块供电稳定，不阻塞其他任务启动。 */
    vTaskDelay(pdMS_TO_TICKS(100U));
    for (;;) {
        if (!output.ready) {
            output.ready = connect_screen(&output);
            publish(&output);
            if (!output.ready) {
                vTaskDelay(pdMS_TO_TICKS(DISPLAY_RETRY_MS));
                continue;
            }
        }
        const TickType_t start = xTaskGetTickCount();
        odometry_output_t pose;
        const bool valid = odometry_app_get_output(&pose);
        output.pose_valid = valid;
        output.sample_timestamp_us = pose.timestamp_us;
        const char *state = valid ? "RUN" :
            pose.state == ODOMETRY_STATE_TRACKING_LOST ? "LOST" :
            pose.state == ODOMETRY_STATE_ERROR ? "ERROR" : "WAIT";
        char heading[OLED_TEXT_COLUMNS + 1U];
        (void)snprintf(heading, sizeof(heading), "ODOM %s E%u", state, (unsigned)pose.error);
        oled_text_line(&screen, 0U, heading);
        draw_value(1U, "X", pose.x_mm, "MM", valid);
        draw_value(2U, "Y", pose.y_mm, "MM", valid);
        draw_value(3U, "YAW", pose.yaw_deg, "DEG", valid);
        if (oled_refresh(&screen)) {
            ++output.frames;
        } else {
            output.ready = false;
            output.error = DISPLAY_ERROR_IO;
            output.last_i2c_error = bsp_oled_i2c_error();
            ++output.io_errors;
        }
        const TickType_t elapsed = xTaskGetTickCount() - start;
        const uint32_t elapsed_ms = elapsed * portTICK_PERIOD_MS;
        if (elapsed_ms > output.max_refresh_ms) output.max_refresh_ms = elapsed_ms;
        publish(&output);
        /* 耗时已经超过周期时也至少让出一拍，不突发补刷旧画面。 */
        const TickType_t period = pdMS_TO_TICKS(DISPLAY_PERIOD_MS);
        vTaskDelay(elapsed < period ? period - elapsed : 1U);
    }
}

bool display_app_start(void) {
    if (task_handle != NULL) return true;
    osThreadDef(displayTask, display_task, osPriorityLow, 0, DISPLAY_TASK_STACK_WORDS);
    task_handle = osThreadCreate(osThread(displayTask), NULL);
    if (task_handle == NULL) {
        g_display.error = DISPLAY_ERROR_TASK;
        return false;
    }
    return true;
}
