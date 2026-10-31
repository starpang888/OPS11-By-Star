/** @file bsp_telemetry.h
 * @brief BSP 层 USART2 发送接口：非阻塞 DMA、独立发送缓冲区及传输统计。
 */
#ifndef BSP_TELEMETRY_H
#define BSP_TELEMETRY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_TELEMETRY_CAPACITY 112U
typedef struct {
    bool ready;
    bool busy;
    uint32_t started, completed, errors, timeouts, recoveries;
    uint32_t last_hal_error;
} bsp_telemetry_status_t;

/* USART2 由串口任务独占；UART/DMA 初始化后由单个任务调用这些接口。
   send 返回前将报文复制到静态 SRAM，调用者随后可以复用自己的报文缓冲区。 */
bool bsp_telemetry_initialise(void);
/* 每个发送周期先处理完成、错误或超时，再判断是否可以发送下一帧。 */
void bsp_telemetry_service(void);
bool bsp_telemetry_send(const char *data, size_t length);
void bsp_telemetry_get_status(bsp_telemetry_status_t *status);
#endif
