#include "bsp_mt6826s.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include <string.h>

_Static_assert(pdMS_TO_TICKS(2) > 0, "DMA timeout must span at least one RTOS tick");

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t cs_pin_number;
    uint32_t cs_guard_cycles;
    uint32_t last_latch_ms;
    uint32_t last_latch_us;
    IRQn_Type rx_irq, tx_irq, spi_irq;
    TaskHandle_t owner;
    volatile bool active;
    volatile bool success;
    bool ready;
    /* DMA-accessible SRAM, independent of the caller's stack or flash buffer.
       .data/.bss must remain in SRAM at 0x20000000, not CCM at 0x10000000. */
    _Alignas(4) uint8_t tx[8];
    _Alignas(4) uint8_t rx[8];
    bsp_mt6826s_dma_stats_t stats;
} encoder_bus_context_t;

static encoder_bus_context_t bus_contexts[BSP_MT6826S_COUNT] = {
    { .spi = &hspi1, .cs_port = MT6826S_CS_GPIO_Port,
      .cs_pin = MT6826S_CS_Pin, .cs_pin_number = 4U,
      .rx_irq = DMA2_Stream0_IRQn, .tx_irq = DMA2_Stream3_IRQn, .spi_irq = SPI1_IRQn },
    { .spi = &hspi2, .cs_port = MT6826S_2_CS_GPIO_Port,
      .cs_pin = MT6826S_2_CS_Pin, .cs_pin_number = 12U,
      .rx_irq = DMA1_Stream3_IRQn, .tx_irq = DMA1_Stream4_IRQn, .spi_irq = SPI2_IRQn }
};

static void cs_guard_delay(const encoder_bus_context_t *context) {
    /* Each iteration executes at least one CPU cycle. This is a minimum delay,
       not a precision clock; interrupts and task preemption may extend it. */
    for (uint32_t i = 0; i < context->cs_guard_cycles; ++i) {
        __NOP();
    }
}

static void chip_select(void *context, bool active) {
    encoder_bus_context_t *device = context;
    if (!active) {
        cs_guard_delay(device); /* TH >= half an SCK period after the final clock. */
    }
    if (active) {
        /* Timestamp beside the CS edge, without task preemption between them.
           This marks the latch request, not internal sensor filter latency. */
        taskENTER_CRITICAL();
        device->last_latch_ms = HAL_GetTick();
        device->last_latch_us = bsp_time_micros();
        HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
        taskEXIT_CRITICAL();
    } else {
        HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
    }
    cs_guard_delay(device); /* TL >= 100 ns, and a gap between adjacent transactions. */
}

/* Called only from HAL SPI callbacks, after HAL has finished its IRQ work.
   Successful TxRx completion follows HAL's end-of-wire BSY check. */
static void complete_from_isr(SPI_HandleTypeDef *spi, bool success) {
    const UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    for (unsigned i = 0; i < BSP_MT6826S_COUNT; ++i) {
        encoder_bus_context_t *device = &bus_contexts[i];
        if (device->spi != spi || !device->active) {
            continue;
        }
        cs_guard_delay(device);
        HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
        device->success = success;
        device->active = false;
        const uint32_t elapsed = bsp_time_micros() - device->last_latch_us;
        if (elapsed > device->stats.max_completion_us) {
            device->stats.max_completion_us = elapsed;
        }
        if (success) {
            ++device->stats.completed;
        } else {
            ++device->stats.transfer_errors;
            device->stats.last_error = HAL_SPI_GetError(spi);
        }
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(device->owner, &higher_priority_task_woken);
        taskEXIT_CRITICAL_FROM_ISR(mask);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(mask);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi) {
    complete_from_isr(spi, true);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi) {
    complete_from_isr(spi, false);
}

/* HAL's full-duplex DMA path leaves the TX stream error callback NULL.
   Report a TX-side fault immediately instead of waiting for the task timeout. */
static void tx_dma_error(DMA_HandleTypeDef *dma) {
    SPI_HandleTypeDef *spi = dma->Parent;
    CLEAR_BIT(spi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    SET_BIT(spi->ErrorCode, HAL_SPI_ERROR_DMA);
    complete_from_isr(spi, false);
}

static bool recover(encoder_bus_context_t *device) {
    SPI_HandleTypeDef *spi = device->spi;
    /* Stop further peripheral requests/callbacks before quiescing BOTH streams.
       HAL can clear CR2 DMA bits on error while a stream is still BUSY, so
       HAL_SPI_Abort alone is insufficient. No blocking HAL call under BASEPRI. */
    taskENTER_CRITICAL();
    device->active = false;
    CLEAR_BIT(spi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN |
              SPI_CR2_ERRIE | SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
    DMA_HandleTypeDef *streams[2] = { spi->hdmarx, spi->hdmatx };
    for (unsigned i = 0; i < 2; ++i) {
        __HAL_DMA_DISABLE_IT(streams[i], DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);
        __HAL_DMA_DISABLE_IT(streams[i], DMA_IT_FE);
    }
    __HAL_SPI_DISABLE(spi);
    taskEXIT_CRITICAL();
    bool ok = true;
    for (unsigned i = 0; i < 2; ++i) {
        if (HAL_DMA_GetState(streams[i]) == HAL_DMA_STATE_BUSY && HAL_DMA_Abort(streams[i]) != HAL_OK) {
            ok = false;
        }
        if ((streams[i]->Instance->CR & DMA_SxCR_EN) != 0 ||
            HAL_DMA_GetState(streams[i]) != HAL_DMA_STATE_READY) {
            ok = false;
        }
        __HAL_DMA_CLEAR_FLAG(streams[i], __HAL_DMA_GET_TC_FLAG_INDEX(streams[i]) |
            __HAL_DMA_GET_HT_FLAG_INDEX(streams[i]) | __HAL_DMA_GET_TE_FLAG_INDEX(streams[i]) |
            __HAL_DMA_GET_DME_FLAG_INDEX(streams[i]) | __HAL_DMA_GET_FE_FLAG_INDEX(streams[i]));
    }
    if (!ok) {
        ++device->stats.recovery_errors;
        device->ready = false; /* Do not reuse buffers while DMA shutdown is uncertain. */
    } else if (HAL_SPI_Abort(spi) != HAL_OK) {
        ++device->stats.recovery_errors;
        device->ready = false;
        ok = false;
    }
    chip_select(device, false);
    NVIC_ClearPendingIRQ(device->rx_irq);
    NVIC_ClearPendingIRQ(device->tx_irq);
    NVIC_ClearPendingIRQ(device->spi_irq);
    (void)ulTaskNotifyTake(pdTRUE, 0);
    return ok;
}

static bool transfer(void *context, const uint8_t *tx, uint8_t *rx, size_t length) {
    encoder_bus_context_t *device = context;
    if (tx == NULL || rx == NULL || length == 0 || length > 6U || !device->ready ||
        xTaskGetCurrentTaskHandle() != device->owner) {
        return false;
    }
    (void)ulTaskNotifyTake(pdTRUE, 0);
    memcpy(device->tx, tx, length);
    memset(device->rx, 0, sizeof(device->rx));
    /* Protect HAL's setup, not the wire transfer or the completion wait.
       RX can finish before HAL_SPI_TransmitReceive_DMA returns on a short frame. */
    taskENTER_CRITICAL();
    device->success = false;
    device->active = true;
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(device->spi, device->tx, device->rx, (uint16_t)length);
    if (status == HAL_OK) {
        /* Six-byte frames need no half-transfer interrupt. */
        __HAL_DMA_DISABLE_IT(device->spi->hdmarx, DMA_IT_HT);
        device->spi->hdmatx->XferErrorCallback = tx_dma_error;
        ++device->stats.started;
    } else {
        device->active = false;
        ++device->stats.start_errors;
        device->stats.last_error = HAL_SPI_GetError(device->spi);
    }
    taskEXIT_CRITICAL();
    if (status != HAL_OK) {
        (void)recover(device);
        return false;
    }
    const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
    taskENTER_CRITICAL();
    const bool completed = !device->active;
    const bool success = completed && device->success;
    if (!completed) {
        device->active = false;
        ++device->stats.timeouts;
        device->stats.last_error = HAL_SPI_ERROR_FLAG;
    }
    taskEXIT_CRITICAL();
    (void)notified; /* Completion flag resolves an IRQ arriving at the timeout boundary. */
    if (!success) {
        (void)recover(device);
        return false;
    }
    memcpy(rx, device->rx, length);
    return true;
}

static bool irq_ready(IRQn_Type irq) {
    const uint32_t priority = NVIC_GetPriority(irq);
    return NVIC_GetEnableIRQ(irq) != 0 &&
        priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY &&
        priority <= configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
}

static bool dma_ready(const DMA_HandleTypeDef *dma, DMA_Stream_TypeDef *stream,
                      uint32_t channel, uint32_t direction, const SPI_HandleTypeDef *spi) {
    return dma != NULL && dma->Instance == stream && dma->Parent == spi &&
        dma->Init.Channel == channel && dma->Init.Direction == direction &&
        dma->Init.PeriphInc == DMA_PINC_DISABLE && dma->Init.MemInc == DMA_MINC_ENABLE &&
        dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
        dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE && dma->Init.Mode == DMA_NORMAL &&
        dma->Init.FIFOMode == DMA_FIFOMODE_DISABLE && dma->State == HAL_DMA_STATE_READY;
}

bool bsp_mt6826s_initialise(bsp_mt6826s_id_t id, mt6826s_bus_t *bus) {
    if ((bus == NULL) || ((unsigned)id >= BSP_MT6826S_COUNT)) {
        return false;
    }
    if (!bsp_time_initialise()) {
        return false;
    }
    encoder_bus_context_t *device = &bus_contexts[id];
    SPI_HandleTypeDef *spi = device->spi;
    const bool first = id == BSP_MT6826S_1;
    if ((spi->Instance != (first ? SPI1 : SPI2)) ||
        (device->cs_port != (first ? GPIOA : GPIOB)) ||
        (device->cs_pin != (first ? GPIO_PIN_4 : GPIO_PIN_12))) {
        return false;
    }
    const uint32_t divider = 2U << ((spi->Init.BaudRatePrescaler & SPI_CR1_BR) >> SPI_CR1_BR_Pos);
    const uint32_t pclk_hz = first ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    const uint32_t sck_hz = pclk_hz / divider;
    const uint32_t mode_shift = 2U * device->cs_pin_number;
    if ((spi->Init.Mode != SPI_MODE_MASTER) ||
        (spi->Init.Direction != SPI_DIRECTION_2LINES) ||
        (spi->Init.DataSize != SPI_DATASIZE_8BIT) ||
        (spi->Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (spi->Init.CLKPhase != SPI_PHASE_2EDGE) ||
        (spi->Init.NSS != SPI_NSS_SOFT) || (spi->Init.FirstBit != SPI_FIRSTBIT_MSB) ||
        (spi->Init.TIMode != SPI_TIMODE_DISABLE) ||
        (spi->Init.CRCCalculation != SPI_CRCCALCULATION_DISABLE) ||
        (sck_hz < 1000000U) || (sck_hz > 15625000U) || /* Datasheet TSCK >= 64 ns. */
        (((device->cs_port->MODER >> mode_shift) & 0x03U) != 0x01U) ||
        ((device->cs_port->OTYPER & device->cs_pin) != 0)) {
        return false;
    }
    if (!dma_ready(spi->hdmarx, first ? DMA2_Stream0 : DMA1_Stream3,
                   first ? DMA_CHANNEL_3 : DMA_CHANNEL_0, DMA_PERIPH_TO_MEMORY, spi) ||
        !dma_ready(spi->hdmatx, first ? DMA2_Stream3 : DMA1_Stream4,
                   first ? DMA_CHANNEL_3 : DMA_CHANNEL_0, DMA_MEMORY_TO_PERIPH, spi) ||
        !irq_ready(device->rx_irq) || !irq_ready(device->tx_irq) || !irq_ready(device->spi_irq)) {
        return false;
    }
    device->owner = xTaskGetCurrentTaskHandle();
    device->ready = true;
    /* At least 1 us for TL, and at least half an SCK period for TH. */
    device->cs_guard_cycles = SystemCoreClock / 1000000U + 1U;
    const uint32_t half_sck_cycles = SystemCoreClock / (2U * sck_hz) + 1U;
    if (half_sck_cycles > device->cs_guard_cycles) {
        device->cs_guard_cycles = half_sck_cycles;
    }
    chip_select(device, false);
    /* Establish Mode 3's idle-high clock before the first CS falling edge. */
    __HAL_SPI_ENABLE(spi);
    cs_guard_delay(device);
    *bus = (mt6826s_bus_t) {
        .context = device,
        .select = chip_select,
        .transfer = transfer
    };
    return true;
}

uint32_t bsp_mt6826s_spi_error(bsp_mt6826s_id_t id) {
    if ((unsigned)id >= BSP_MT6826S_COUNT) {
        return HAL_SPI_ERROR_FLAG;
    }
    return HAL_SPI_GetError(bus_contexts[id].spi);
}

uint32_t bsp_mt6826s_last_latch_ms(bsp_mt6826s_id_t id) {
    if ((unsigned)id >= BSP_MT6826S_COUNT) {
        return 0;
    }
    return bus_contexts[id].last_latch_ms;
}

uint32_t bsp_mt6826s_last_latch_us(bsp_mt6826s_id_t id) {
    return (unsigned)id < BSP_MT6826S_COUNT ? bus_contexts[id].last_latch_us : 0;
}

bool bsp_mt6826s_get_dma_stats(bsp_mt6826s_id_t id, bsp_mt6826s_dma_stats_t *stats) {
    if ((unsigned)id >= BSP_MT6826S_COUNT || stats == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    *stats = bus_contexts[id].stats;
    taskEXIT_CRITICAL();
    return true;
}
