#ifndef BSP_TIME_H
#define BSP_TIME_H
#include <stdbool.h>
#include <stdint.h>
/* Shared, free-running TIM2 at 1 MHz. Initialise after MX_TIM2_Init.
   Never reset the counter while acquisition is active. Task context only. */
bool bsp_time_initialise(void);
uint32_t bsp_time_micros(void);
#endif
