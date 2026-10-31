/** @file mt6826s.h
 * @brief modules 层 MT6826S 协议接口：15 位单圈角度、CRC、报警和配置寄存器访问。
 */
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

/* 总线由单个采样任务独占；select(true) 拉低片选，片选时序由 BSP 保证。
   transfer 在一次片选内收发整个缓冲区，函数内部不主动结束片选。 */
typedef struct {
    void *context;
    void (*select)(void *context, bool active);
    bool (*transfer)(void *context, const uint8_t *tx, uint8_t *rx, size_t length);
} mt6826s_bus_t;

typedef struct {
    uint8_t raw[MT6826S_FRAME_BYTES]; /* 对应寄存器 0x003～0x006，含角度、报警、CRC。 */
    uint8_t crc_calculated;
    uint8_t alarms;
    uint16_t angle_raw;
    float angle_deg; /* 单圈角度 [0,360)，单位度；读取返回 MT6826S_OK 时才可用于控制。 */
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
/* 手册图 24：四次普通寄存器读取，每次独立片选；角度快照在读完 0x003～0x006 前保持锁存。
   此接口用于开发时核对读法，不应穿插周期 Burst 采样。 */
mt6826s_status_t mt6826s_read_via_registers(const mt6826s_bus_t *bus, mt6826s_sample_t *sample);
mt6826s_status_t mt6826s_read_diagnostics(const mt6826s_bus_t *bus, mt6826s_diagnostics_t *diagnostics);
/* 每步只访问一个寄存器：0=带宽、1=校准状态、2=校准控制。
   调用者保存中间结果，三步都成功后再发布整组诊断值。 */
mt6826s_status_t mt6826s_read_diagnostic_step(const mt6826s_bus_t *bus, unsigned step,
                                            mt6826s_diagnostics_t *diagnostics);
/* 只改运行寄存器，不烧写 EEPROM；拒绝非预期的初始带宽，已为目标值时直接成功。 */
mt6826s_status_t mt6826s_set_bandwidth(const mt6826s_bus_t *bus, uint8_t expected_bandwidth,
                                    uint8_t target_bandwidth, mt6826s_bandwidth_change_t *change);

#endif
