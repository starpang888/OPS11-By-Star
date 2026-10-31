/** @file bsp_telemetry.c
 * @brief BSP 层串口 DMA 生命周期管理：回调记录事件，任务负责恢复和缓冲区复用。
 */
#include "bsp_telemetry.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
static uint8_t tx_buffer[BSP_TELEMETRY_CAPACITY];
static bsp_telemetry_status_t status;
static volatile bool tx_done, tx_error;
static volatile uint32_t irq_error;
static uint32_t started_ms;

bool bsp_telemetry_initialise(void) {
    const DMA_HandleTypeDef *dma = huart2.hdmatx;
    status.ready = huart2.Instance == USART2 &&
        huart2.Init.BaudRate == 115200U &&
        huart2.Init.WordLength == UART_WORDLENGTH_8B &&
        huart2.Init.StopBits == UART_STOPBITS_1 &&
        huart2.Init.Parity == UART_PARITY_NONE &&
        huart2.Init.HwFlowCtl == UART_HWCONTROL_NONE &&
        (huart2.Init.Mode & UART_MODE_TX) != 0U &&
        huart2.gState == HAL_UART_STATE_READY && dma != NULL &&
        dma->Instance == DMA1_Stream6 && dma->Init.Channel == DMA_CHANNEL_4 &&
        dma->Init.Direction == DMA_MEMORY_TO_PERIPH &&
        dma->Init.Mode == DMA_NORMAL && dma->Init.MemInc == DMA_MINC_ENABLE &&
        dma->Init.PeriphInc == DMA_PINC_DISABLE &&
        dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
        NVIC_GetEnableIRQ(USART2_IRQn) && NVIC_GetEnableIRQ(DMA1_Stream6_IRQn);
    return status.ready;
}

/* 仅在任务中恢复：HAL 中止操作可能等待 tick，不能在长临界区或中断里执行。
   中断只报告事件，不能释放或覆盖正在被 DMA 读取的缓冲区。 */
static void recover(void) {
    taskENTER_CRITICAL();
    CLEAR_BIT(huart2.Instance->CR3, USART_CR3_DMAT);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
    __HAL_DMA_DISABLE_IT(huart2.hdmatx,
        DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(huart2.hdmatx, DMA_IT_FE);
    taskEXIT_CRITICAL();

    const HAL_StatusTypeDef uart_result = HAL_UART_AbortTransmit(&huart2);
    /* UART 错误路径可能提前清除 DMAT，因此中止 UART 后仍需确认 DMA 已停止。 */
    if (huart2.hdmatx->State != HAL_DMA_STATE_READY ||
        (huart2.hdmatx->Instance->CR & DMA_SxCR_EN) != 0U) {
        (void)HAL_DMA_Abort(huart2.hdmatx);
    }
    taskENTER_CRITICAL();
    status.ready = uart_result == HAL_OK &&
        huart2.hdmatx->State == HAL_DMA_STATE_READY &&
        (huart2.hdmatx->Instance->CR & DMA_SxCR_EN) == 0U &&
        huart2.gState == HAL_UART_STATE_READY;
    if (status.ready) {
        __HAL_DMA_CLEAR_FLAG(huart2.hdmatx,
            __HAL_DMA_GET_TC_FLAG_INDEX(huart2.hdmatx) |
            __HAL_DMA_GET_HT_FLAG_INDEX(huart2.hdmatx) |
            __HAL_DMA_GET_TE_FLAG_INDEX(huart2.hdmatx) |
            __HAL_DMA_GET_DME_FLAG_INDEX(huart2.hdmatx) |
            __HAL_DMA_GET_FE_FLAG_INDEX(huart2.hdmatx));
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
        HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
        status.busy = false;
        ++status.recoveries;
    }
    tx_done = tx_error = false;
    taskEXIT_CRITICAL();
}

void bsp_telemetry_service(void) {
    if (!status.ready) {
        return;
    }
    taskENTER_CRITICAL();
    const bool error = tx_error;
    const bool done = tx_done;
    if (error) {
        status.last_hal_error = irq_error;
    }
    tx_error = tx_done = false;
    taskEXIT_CRITICAL();
    if (error) {
        ++status.errors;
        recover();
    } else if (status.busy && done) {
        status.busy = false;
        ++status.completed;
    } else if (status.busy && (uint32_t)(HAL_GetTick() - started_ms) >= 100U) {
        ++status.timeouts;
        recover();
    }
}

bool bsp_telemetry_send(const char *data, size_t length) {
    if (!status.ready || status.busy || data == NULL || length == 0U ||
        length > sizeof(tx_buffer)) {
        return false;
    }
    memcpy(tx_buffer, data, length);
    started_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    tx_done = tx_error = false;
    status.busy = true;
    const HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&huart2, tx_buffer, (uint16_t)length);
    taskEXIT_CRITICAL();
    if (result != HAL_OK) {
        ++status.errors;
        status.last_hal_error = huart2.ErrorCode;
        recover();
        return false;
    }
    ++status.started;
    return true;
}

void bsp_telemetry_get_status(bsp_telemetry_status_t *output) {
    if (output != NULL) {
        *output = status;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart == &huart2) {
        tx_done = true; /* UART TC 表示最后一个停止位已经发送完成。 */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart == &huart2) {
        irq_error = uart->ErrorCode;
        tx_error = true;
    }
}
