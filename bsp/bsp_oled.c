/** @file bsp_oled.c
 * @brief BSP 层硬件 I2C3 轮询传输，分块后主动让出 CPU，不屏蔽采样中断。
 */
#include "bsp_oled.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c3;
static uint8_t device_address;

bool bsp_oled_initialise(void) {
    return hi2c3.Instance == I2C3 &&
        hi2c3.Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT &&
        hi2c3.Init.ClockSpeed > 0U && hi2c3.Init.ClockSpeed <= 400000U;
}

bool bsp_oled_probe(uint8_t address) {
    /* STM32 HAL 接收左移一位的地址；配置和调试变量仍保持标准 7 位形式。 */
    return HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)address << 1U, 1U, 10U) == HAL_OK;
}

void bsp_oled_select_address(uint8_t address) {
    device_address = address;
}

bool bsp_oled_write(void *context, uint8_t control, const uint8_t *data, size_t length) {
    (void)context;
    if (data == NULL || length == 0U || length > 32U) {
        return false;
    }
    uint8_t packet[33];
    packet[0] = control;
    memcpy(packet + 1U, data, length);
    return HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)device_address << 1U,
        packet, (uint16_t)(length + 1U), 20U) == HAL_OK;
}

void bsp_oled_yield(void *context) {
    (void)context;
    /* 每个小块之后阻塞一拍，让同级串口任务和空闲任务也有执行机会。 */
    vTaskDelay(pdMS_TO_TICKS(1));
}

uint32_t bsp_oled_i2c_error(void) {
    return HAL_I2C_GetError(&hi2c3);
}
