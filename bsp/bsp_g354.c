/** @file bsp_g354.c
 * @brief BSP 层 G354 硬件适配。DRDY 中断只记时间并通知任务，SPI 读取在任务中完成。
 */
#include "bsp_g354.h"
#include "bsp_time.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern SPI_HandleTypeDef hspi3;

static TaskHandle_t volatile drdy_task;
static volatile uint32_t drdy_count;
static volatile uint32_t last_drdy_us;

static void chip_select(void *context, bool active) {
    (void)context;
    HAL_GPIO_WritePin(G354_CS_GPIO_Port, G354_CS_Pin, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool transfer(void *context, uint16_t tx, uint16_t *rx) {
    (void)context;
    /* SPI 为 16 位数据宽度，HAL 的 Size=1 表示一个 16 位字，不是一个字节。 */
    return HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx, (uint8_t *)rx, 1, 5) == HAL_OK;
}

static void reset(void *context, bool asserted) {
    (void)context;
    HAL_GPIO_WritePin(G354_RST_GPIO_Port, G354_RST_Pin, asserted ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint32_t bsp_g354_micros(void) {
    return bsp_time_micros();
}

static void delay_us(void *context, uint32_t us) {
    (void)context;
    const uint32_t start = bsp_g354_micros();
    while ((uint32_t)(bsp_g354_micros() - start) < us) {
        /* 协议要求的微秒短延时；被抢占时 TIM2 仍计时，因此实际等待可能更长。 */
    }
}

static void delay_ms(void *context, uint32_t ms) {
    (void)context;
    /* 多等一个 tick，避免 tick 边界让复位低电平或恢复等待短于手册下限。 */
    vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
}

static uint32_t millis(void *context) {
    (void)context;
    return HAL_GetTick();
}

bool bsp_g354_initialise(g354_bus_t *bus) {
    if (bus == NULL) {
        return false;
    }
    const uint32_t spi_divider = 2U << ((hspi3.Init.BaudRatePrescaler & SPI_CR1_BR) >> SPI_CR1_BR_Pos);
    if ((hspi3.Instance != SPI3) || (hspi3.Init.Mode != SPI_MODE_MASTER) ||
        (hspi3.Init.Direction != SPI_DIRECTION_2LINES) ||
        (hspi3.Init.DataSize != SPI_DATASIZE_16BIT) ||
        (hspi3.Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (hspi3.Init.CLKPhase != SPI_PHASE_2EDGE) ||
        (hspi3.Init.FirstBit != SPI_FIRSTBIT_MSB) || (hspi3.Init.NSS != SPI_NSS_SOFT) ||
        (HAL_RCC_GetPCLK1Freq() / spi_divider > 1000000U) ||
        (HAL_RCC_GetPCLK1Freq() / spi_divider < 10000U)) {
        return false;
    }
    if (!bsp_time_initialise()) {
        return false;
    }
    chip_select(NULL, false);
    *bus = (g354_bus_t) {
        .context = NULL,
        .select = chip_select,
        .transfer = transfer,
        .reset = reset,
        .delay_us = delay_us,
        .delay_ms = delay_ms,
        .millis = millis
    };
    return true;
}

void bsp_g354_arm_drdy(void) {
    (void)ulTaskNotifyTake(pdTRUE, 0);
    taskENTER_CRITICAL();
    __HAL_GPIO_EXTI_CLEAR_IT(G354_DRDY_Pin);
    NVIC_ClearPendingIRQ(G354_DRDY_EXTI_IRQn);
    drdy_count = 0;
    last_drdy_us = 0;
    drdy_task = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
}

void bsp_g354_disarm_drdy(void) {
    taskENTER_CRITICAL();
    drdy_task = NULL;
    taskEXIT_CRITICAL();
}

bool bsp_g354_wait_drdy(uint32_t timeout_ms) {
    return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) != 0;
}

uint32_t bsp_g354_drdy_count(void) {
    return drdy_count;
}

uint32_t bsp_g354_last_drdy_us(void) {
    return last_drdy_us;
}

uint32_t bsp_g354_spi_error(void) {
    return HAL_SPI_GetError(&hspi3);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    /* 中断中不做 SPI 访问和解算；唤醒 IMU 任务后由任务确认数据是否连续。 */
    if ((pin == G354_DRDY_Pin) && (drdy_task != NULL)) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        last_drdy_us = bsp_g354_micros();
        ++drdy_count;
        vTaskNotifyGiveFromISR(drdy_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
