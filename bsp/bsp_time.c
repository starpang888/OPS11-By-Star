/** @file bsp_time.c
 * @brief BSP 层 TIM2 时基实现：校验 1 MHz、32 位向上计数配置，并且只启动一次。
 */
#include "bsp_time.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
extern TIM_HandleTypeDef htim2;
static bool started;

bool bsp_time_initialise(void) {
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    /* APB1 分频不为 1 时，定时器输入时钟是 APB1 外设时钟的两倍。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {
        clock *= 2U;
    }
    if ((htim2.Instance != TIM2) || (htim2.Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (clock / (htim2.Init.Prescaler + 1U) != 1000000U) ||
        (htim2.Init.Period != UINT32_MAX)) {
        return false;
    }
    /* 多个采样任务可能同时初始化；临界区保证只执行一次启动。 */
    taskENTER_CRITICAL();
    if (!started) {
        started = HAL_TIM_Base_Start(&htim2) == HAL_OK;
    }
    const bool ready = started;
    taskEXIT_CRITICAL();
    return ready;
}

uint32_t bsp_time_micros(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}
