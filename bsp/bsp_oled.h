/** @file bsp_oled.h
 * @brief BSP 层 OLED I2C3 适配，提供命令/数据发送及地址应答检查。
 */
#ifndef BSP_OLED_H
#define BSP_OLED_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* I2C3 由显示任务独占；address 为未左移的 7 位地址。 */
bool bsp_oled_initialise(void);
bool bsp_oled_probe(uint8_t address);
/* control=0x00 表示命令，0x40 表示显存数据；每次最多 32 字节。
   轮询传输可被高优先级任务抢占，调用方不可在临界区内刷新屏幕。 */
bool bsp_oled_write(void *context, uint8_t control, const uint8_t *data, size_t length);
void bsp_oled_yield(void *context);
void bsp_oled_select_address(uint8_t address);
uint32_t bsp_oled_i2c_error(void);
#endif
