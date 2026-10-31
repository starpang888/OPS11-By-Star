#include "bsp_time.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
extern TIM_HandleTypeDef htim2;
static bool started;

bool bsp_time_initialise(void) {
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {
        clock *= 2U;
    }
    if ((htim2.Instance != TIM2) || (htim2.Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (clock / (htim2.Init.Prescaler + 1U) != 1000000U) ||
        (htim2.Init.Period != UINT32_MAX)) {
        return false;
    }
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
