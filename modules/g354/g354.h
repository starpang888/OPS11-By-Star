#ifndef G354_H
#define G354_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define G354_SAMPLE_RATE_HZ 500U
#define G354_COUNTER_STEP 4U /* COUNT advances once per 500 us, including decimated samples. */
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

/* Bus callbacks run only in the owning task. select(true) asserts active-low CS. */
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
    float gyroscope_dps[3];
    float accelerometer_g[3];
    float temperature_c;
    uint16_t flags;
    uint16_t counter;
} g354_sample_t;

typedef struct {
    g354_bus_t bus;
    char product_id[9];
    uint16_t firmware_version;
    uint16_t diagnostic; /* DIAG_STAT is cleared on read; preserve its last value here. */
    bool configured;
    bool sampling;
} g354_t;

g354_status_t g354_initialise(g354_t *device, const g354_bus_t *bus);
g354_status_t g354_start(g354_t *device);
g354_status_t g354_stop(g354_t *device);
g354_status_t g354_read_sample(g354_t *device, g354_sample_t *sample);
g354_status_t g354_decode_burst(const uint16_t words[G354_BURST_WORDS], g354_sample_t *sample);

#endif
