/** @file bsp_time.h
 * @brief BSP 层公共时基接口：TIM2 以 1 MHz 自由运行，供三路采样与定位统一计时。
 */
#ifndef BSP_TIME_H
#define BSP_TIME_H
#include <stdbool.h>
#include <stdint.h>
/* 在 MX_TIM2_Init 后从任务上下文调用；重复初始化不清零，运行期间禁止重置计数器。 */
bool bsp_time_initialise(void);
/* 返回微秒计数，约 71.6 分钟回绕；短时间差使用 uint32_t 无符号相减。 */
uint32_t bsp_time_micros(void);
#endif
