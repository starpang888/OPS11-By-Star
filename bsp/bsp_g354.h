/** @file bsp_g354.h
 * @brief BSP 层 G354 接口：绑定 SPI3、软件片选、复位和 DRDY 任务通知。
 */
#ifndef BSP_G354_H
#define BSP_G354_H

#include "g354.h"

/* SPI3 由 IMU 独占；公共 TIM2 由 bsp_time 管理。初始化成功后填写协议层总线回调。 */
bool bsp_g354_initialise(g354_bus_t *bus);
/* 由 IMU 任务注册/注销 DRDY 接收者，该任务的直接通知槽不能另作他用。 */
void bsp_g354_arm_drdy(void);
void bsp_g354_disarm_drdy(void);
/* 等待 DRDY 通知并让出 CPU；timeout_ms 为毫秒，返回 false 表示等待超时。 */
bool bsp_g354_wait_drdy(uint32_t timeout_ms);
uint32_t bsp_g354_micros(void);
/* 中断总次数和最近一次上升沿的 TIM2 微秒时间，用于校验数据与采样时刻。 */
uint32_t bsp_g354_drdy_count(void);
uint32_t bsp_g354_last_drdy_us(void);
uint32_t bsp_g354_spi_error(void);

#endif
