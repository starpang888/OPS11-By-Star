/** @file telemetry_app.c
 * @brief application 层低优先级串口任务：每 10 ms 获取位姿快照、编码并启动 DMA。
 *        忙碌时跳过本周期，之后发送最新位姿，不排队积压旧坐标。
 */
#include "telemetry_app.h"
#include "telemetry_config.h"
#include "odometry_app.h"
#include "odometry_protocol.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

volatile telemetry_output_t g_telemetry;
static osThreadId task_handle;
_Static_assert(ODOMETRY_PROTOCOL_CAPACITY <= BSP_TELEMETRY_CAPACITY,
               "Protocol must fit DMA buffer");
_Static_assert(ODOMETRY_PROTOCOL_CAPACITY * 10U * 1000U <= 115200U * TELEMETRY_PERIOD_MS,
               "Worst-case ASCII frame exceeds UART bandwidth");

static void telemetry_task(void const *argument) {
    (void)argument;
    telemetry_output_t output = {0};
    if (!bsp_telemetry_initialise() || pdMS_TO_TICKS(TELEMETRY_PERIOD_MS) == 0U) {
        output.error = TELEMETRY_ERROR_CONFIG;
        g_telemetry = output;
        vTaskSuspend(NULL);
    }
    output.running = true;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        ++output.cycles;
        bsp_telemetry_service();
        bsp_telemetry_get_status(&output.tx);
        if (!output.tx.ready) {
            output.error = TELEMETRY_ERROR_TRANSPORT;
            output.running = false;
        } else if (output.tx.busy) {
            ++output.skipped_busy;
        } else {
            odometry_output_t pose;
            /* 位姿有效性由里程计接口统一判断；启动等待和故障状态也继续发送状态帧。 */
            const bool valid = odometry_app_get_output(&pose);
            odometry_packet_t packet = {
                .sequence = output.sequence++,
                .timestamp_us = pose.timestamp_us,
                .x_mm = pose.x_mm, .y_mm = pose.y_mm, .yaw_deg = pose.yaw_deg,
                .valid = valid,
                .tracking_continuous = pose.tracking_continuous,
                .state = (uint8_t)pose.state, .error = (uint8_t)pose.error
            };
            /* BSP 发送前会复制到静态 DMA 缓冲区，因此此处可以使用任务栈上的报文。 */
            char frame[ODOMETRY_PROTOCOL_CAPACITY];
            const size_t length = odometry_protocol_encode(frame, sizeof(frame), &packet);
            output.pose_valid = packet.valid;
            output.sample_timestamp_us = packet.timestamp_us;
            output.last_frame_bytes = (uint16_t)length;
            if (length == 0U) {
                ++output.format_errors;
            } else if (bsp_telemetry_send(frame, length)) {
                ++output.sent;
            }
        }
        bsp_telemetry_get_status(&output.tx);
        /* 调试器镜像；固件任务需要一致快照时应使用各模块提供的读取接口。 */
        taskENTER_CRITICAL();
        g_telemetry = output;
        taskEXIT_CRITICAL();
        const TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_wake) > pdMS_TO_TICKS(TELEMETRY_PERIOD_MS)) {
            last_wake = now;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

bool telemetry_app_start(void) {
    if (task_handle != NULL) {
        return true;
    }
    osThreadDef(telemetryTask, telemetry_task, osPriorityLow, 0,
                TELEMETRY_TASK_STACK_WORDS);
    task_handle = osThreadCreate(osThread(telemetryTask), NULL);
    if (task_handle == NULL) {
        g_telemetry.error = TELEMETRY_ERROR_TASK;
        return false;
    }
    return true;
}
