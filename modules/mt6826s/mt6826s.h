#ifndef MT6826S_H
#define MT6826S_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MT6826S_COUNTS_PER_REVOLUTION 32768U
#define MT6826S_FRAME_BYTES 4U
#define MT6826S_ALARM_OVERSPEED 0x01U
#define MT6826S_ALARM_WEAK_MAGNET 0x02U
#define MT6826S_ALARM_UNDERVOLTAGE 0x04U

typedef enum {
    MT6826S_OK = 0,
    MT6826S_ERROR_ARGUMENT,
    MT6826S_ERROR_IO,
    MT6826S_ERROR_CRC,
    MT6826S_ERROR_FORMAT,
    MT6826S_ERROR_ALARM,
    MT6826S_ERROR_CONFIGURATION
} mt6826s_status_t;

/* Owned by one task. select(true) asserts CS; the BSP provides CS timing.
   transfer() clocks the entire buffer without releasing CS. */
typedef struct {
    void *context;
    void (*select)(void *context, bool active);
    bool (*transfer)(void *context, const uint8_t *tx, uint8_t *rx, size_t length);
} mt6826s_bus_t;

typedef struct {
    uint8_t raw[MT6826S_FRAME_BYTES]; /* Registers 0x003 through 0x006. */
    uint8_t crc_calculated;
    uint8_t alarms;
    uint16_t angle_raw;
    float angle_deg; /* [0, 360), usable only when the read returns MT6826S_OK. */
} mt6826s_sample_t;

typedef struct {
    uint8_t bandwidth_register; /* 0x011, including reserved bits. */
    uint8_t calibration_status_register; /* 0x113. */
    uint8_t calibration_control; /* 0x155; 0x5E enables calibration by software. */
    uint8_t bandwidth; /* 0x011[2:0], factory default 5. */
    uint8_t calibration_state; /* 0x113[7:6]: 0 idle, 1 running, 2 failed, 3 succeeded. */
} mt6826s_diagnostics_t;

typedef struct {
    uint8_t before_register;
    uint8_t target_register;
    uint8_t after_register;
    bool before_read_ok;
    bool after_read_ok;
    bool write_attempted;
    bool verified; /* Entire register matches, including preserved bits. */
} mt6826s_bandwidth_change_t;

/* CRC-8: poly=0x07, init=0x00, no reflection, xorout=0x00. */
uint8_t mt6826s_crc8(const uint8_t data[3]);
mt6826s_status_t mt6826s_decode(const uint8_t raw[MT6826S_FRAME_BYTES], mt6826s_sample_t *sample);
mt6826s_status_t mt6826s_read(const mt6826s_bus_t *bus, mt6826s_sample_t *sample);
mt6826s_status_t mt6826s_read_register(const mt6826s_bus_t *bus, uint16_t address, uint8_t *value);
/* Datasheet Figure 24: four ordinary reads, one CS interval per register.
   The angle snapshot stays latched until registers 0x003..0x006 are consumed. */
mt6826s_status_t mt6826s_read_via_registers(const mt6826s_bus_t *bus, mt6826s_sample_t *sample);
mt6826s_status_t mt6826s_read_diagnostics(const mt6826s_bus_t *bus, mt6826s_diagnostics_t *diagnostics);
/* Volatile register change only, no EEPROM programming. Refuses an unexpected
   starting BW; an already matching BW succeeds without a write. */
mt6826s_status_t mt6826s_set_bandwidth(const mt6826s_bus_t *bus, uint8_t expected_bandwidth,
                                    uint8_t target_bandwidth, mt6826s_bandwidth_change_t *change);

#endif
