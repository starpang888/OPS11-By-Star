#ifndef BSP_G354_H
#define BSP_G354_H

#include "g354.h"

/* SPI3 belongs to the IMU; the free-running TIM2 timebase is shared via bsp_time. */
bool bsp_g354_initialise(g354_bus_t *bus);
void bsp_g354_arm_drdy(void);
void bsp_g354_disarm_drdy(void);
bool bsp_g354_wait_drdy(uint32_t timeout_ms);
uint32_t bsp_g354_micros(void);
uint32_t bsp_g354_drdy_count(void);
uint32_t bsp_g354_last_drdy_us(void);
uint32_t bsp_g354_spi_error(void);

#endif
