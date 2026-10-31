/** @file bsp_mt6826s.h
 * @brief BSP 层双编码器接口：SPI1/PA4 与 SPI2/PB12，DMA 收发并使用软件片选。
 */
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
    uint32_t max_completion_us; /* 片选拉低到完成中断的最大耗时，微秒，包含调度延迟。 */
} bsp_mt6826s_dma_stats_t;

/* 每路总线及对应任务的直接通知槽由本路采样任务独占。
   只允许任务上下文调用；等待 DMA 时阻塞当前任务并让出 CPU。 */
bool bsp_mt6826s_initialise(bsp_mt6826s_id_t id, mt6826s_bus_t *bus);
uint32_t bsp_mt6826s_spi_error(bsp_mt6826s_id_t id);
/* 最近一次片选拉低时的 HAL 毫秒时间，诊断访问也会覆盖它。
   周期角度读取后应立即取时间戳，再执行其他总线访问。 */
uint32_t bsp_mt6826s_last_latch_ms(bsp_mt6826s_id_t id);
/* 同一片选边沿的公共 TIM2 微秒时间；同样必须在诊断访问前取出。 */
uint32_t bsp_mt6826s_last_latch_us(bsp_mt6826s_id_t id);
/* 任务上下文读取一致统计快照；次数包含周期采样及诊断传输。 */
bool bsp_mt6826s_get_dma_stats(bsp_mt6826s_id_t id, bsp_mt6826s_dma_stats_t *stats);

#endif
