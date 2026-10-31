#ifndef BSP_MT6826S_H
#define BSP_MT6826S_H

#include "mt6826s.h"

typedef enum {
    BSP_MT6826S_1 = 0, /* SPI1 + PA4. */
    BSP_MT6826S_2,     /* SPI2 + PB12. */
    BSP_MT6826S_COUNT
} bsp_mt6826s_id_t;

typedef struct {
    uint32_t started;
    uint32_t completed;
    uint32_t start_errors;
    uint32_t transfer_errors;
    uint32_t timeouts;
    uint32_t recovery_errors;
    uint32_t last_error;
    uint32_t max_completion_us; /* CS assertion to completion ISR, including scheduling delay. */
} bsp_mt6826s_dma_stats_t;

/* Each bus and that task's direct notification slot are exclusively owned by
   the corresponding encoder task. DMA wait yields the CPU; task context only. */
bool bsp_mt6826s_initialise(bsp_mt6826s_id_t id, mt6826s_bus_t *bus);
uint32_t bsp_mt6826s_spi_error(bsp_mt6826s_id_t id);
/* HAL tick captured beside the most recent CS assertion, including diagnostics.
   Read immediately after the periodic angle transfer, before another transfer. */
uint32_t bsp_mt6826s_last_latch_ms(bsp_mt6826s_id_t id);
/* Same CS edge in the shared TIM2 domain. Capture before diagnostics. */
uint32_t bsp_mt6826s_last_latch_us(bsp_mt6826s_id_t id);
/* Task context; DMA statistics include periodic AND diagnostic transactions. */
bool bsp_mt6826s_get_dma_stats(bsp_mt6826s_id_t id, bsp_mt6826s_dma_stats_t *stats);

#endif
