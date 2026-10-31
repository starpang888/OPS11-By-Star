/** @file g354.h
 * @brief modules 层 Epson G354 协议接口：配置寄存器、校验 Burst 并换算物理量。
 *        硬件访问通过回调注入，不直接调用 STM32 HAL 或 RTOS。
 */
#ifndef G354_H
#define G354_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define G354_SAMPLE_RATE_HZ 500U
#define G354_COUNTER_STEP 4U /* COUNT 每 500 us 加一，500 Hz 输出相邻计数应相差 4。 */
#define G354_BURST_WORDS 17U

typedef enum {
    G354_OK = 0,
    G354_ERROR_ARGUMENT,
    G354_ERROR_IO,
    G354_ERROR_TIMEOUT,
    G354_ERROR_PRODUCT_ID,
    G354_ERROR_DIAGNOSTIC,
    G354_ERROR_CONFIGURATION,
    G354_ERROR_CHECKSUM,
    G354_ERROR_NOT_READY
} g354_status_t;

/* 总线回调只能由所属任务调用；select(true) 表示拉低片选，复位也为低有效。 */
typedef struct {
    void *context;
    void (*select)(void *context, bool active);
    bool (*transfer)(void *context, uint16_t tx, uint16_t *rx);
    void (*reset)(void *context, bool asserted);
    void (*delay_us)(void *context, uint32_t us);
    void (*delay_ms)(void *context, uint32_t ms);
    uint32_t (*millis)(void *context);
} g354_bus_t;

typedef struct {
    float gyroscope_dps[3]; /* 传感器原始坐标系三轴角速度，度/秒。 */
    float accelerometer_g[3]; /* 传感器原始坐标系三轴加速度，g。 */
    float temperature_c; /* 摄氏度。 */
    uint16_t flags;
    uint16_t counter;
} g354_sample_t;

typedef struct {
    g354_bus_t bus;
    char product_id[9];
    uint16_t firmware_version;
    uint16_t diagnostic; /* DIAG_STAT 读后清除，因此保存最近一次读取值供诊断。 */
    bool configured;
    bool sampling;
} g354_t;

/* 复位、检查产品标识、写入并回读配置；不写传感器非易失存储。 */
g354_status_t g354_initialise(g354_t *device, const g354_bus_t *bus);
g354_status_t g354_start(g354_t *device);
g354_status_t g354_stop(g354_t *device);
/* 完整读取一次 Burst；只有返回 G354_OK 时样本才可进入解算。 */
g354_status_t g354_read_sample(g354_t *device, g354_sample_t *sample);
g354_status_t g354_decode_burst(const uint16_t words[G354_BURST_WORDS], g354_sample_t *sample);

#endif
