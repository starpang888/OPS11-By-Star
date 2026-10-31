/** @file mt6826s.c
 * @brief modules 层 MT6826S 协议实现。只处理命令与数据解释，DMA/片选由总线回调完成。
 */
#include "mt6826s.h"

#include <string.h>

uint8_t mt6826s_crc8(const uint8_t data[3]) {
    uint8_t crc = 0;
    for (size_t i = 0; i < 3; ++i) {
        crc ^= data[i];
        for (size_t bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0x07U) : (crc << 1));
        }
    }
    return crc;
}

mt6826s_status_t mt6826s_decode(const uint8_t raw[MT6826S_FRAME_BYTES], mt6826s_sample_t *sample) {
    if ((raw == NULL) || (sample == NULL)) {
        return MT6826S_ERROR_ARGUMENT;
    }
    *sample = (mt6826s_sample_t) {0};
    memcpy(sample->raw, raw, MT6826S_FRAME_BYTES);
    sample->crc_calculated = mt6826s_crc8(raw);
    if (sample->crc_calculated != raw[3]) {
        return MT6826S_ERROR_CRC;
    }
    /* 格式依次为 ANGLE[14:7]、ANGLE[6:0]+固定零位、五个零位+三位报警状态。 */
    if (((raw[1] & 0x01U) != 0) || ((raw[2] & 0xF8U) != 0)) {
        return MT6826S_ERROR_FORMAT;
    }
    sample->alarms = raw[2] & 0x07U;
    sample->angle_raw = ((uint16_t)raw[0] << 7) | (raw[1] >> 1);
    sample->angle_deg = (float)sample->angle_raw * (360.0f / MT6826S_COUNTS_PER_REVOLUTION);
    return sample->alarms ? MT6826S_ERROR_ALARM : MT6826S_OK;
}

mt6826s_status_t mt6826s_read(const mt6826s_bus_t *bus, mt6826s_sample_t *sample) {
    if ((bus == NULL) || (sample == NULL) || (bus->select == NULL) || (bus->transfer == NULL)) {
        return MT6826S_ERROR_ARGUMENT;
    }
    *sample = (mt6826s_sample_t) {0};
    /* 同一片选内发送 0xA003，再接收角度高/低字节、STATUS 和 CRC。 */
    const uint8_t tx[6] = {0xA0, 0x03, 0, 0, 0, 0};
    uint8_t rx[6] = {0};
    bus->select(bus->context, true);
    const bool transferred = bus->transfer(bus->context, tx, rx, sizeof(tx));
    bus->select(bus->context, false); /* 超时或传输失败时同样释放片选。 */
    if (!transferred) {
        return MT6826S_ERROR_IO;
    }
    return mt6826s_decode(&rx[2], sample);
}

mt6826s_status_t mt6826s_read_register(const mt6826s_bus_t *bus, uint16_t address, uint8_t *value) {
    if ((bus == NULL) || (value == NULL) || (bus->select == NULL) ||
        (bus->transfer == NULL) || (address > 0x0FFFU)) {
        return MT6826S_ERROR_ARGUMENT;
    }
    /* 普通读命令为 0011+12 位地址，返回一个字节；普通寄存器读协议不附加 CRC。 */
    const uint8_t tx[3] = {(uint8_t)(0x30U | (address >> 8)), (uint8_t)address, 0};
    uint8_t rx[3] = {0};
    bus->select(bus->context, true);
    const bool transferred = bus->transfer(bus->context, tx, rx, sizeof(tx));
    bus->select(bus->context, false);
    if (!transferred) {
        return MT6826S_ERROR_IO;
    }
    *value = rx[2];
    return MT6826S_OK;
}

mt6826s_status_t mt6826s_read_via_registers(const mt6826s_bus_t *bus, mt6826s_sample_t *sample) {
    if (sample == NULL) {
        return MT6826S_ERROR_ARGUMENT;
    }
    *sample = (mt6826s_sample_t) {0};
    uint8_t raw[MT6826S_FRAME_BYTES] = {0};
    for (uint16_t i = 0; i < MT6826S_FRAME_BYTES; ++i) {
        const mt6826s_status_t status = mt6826s_read_register(bus, 0x003U + i, &raw[i]);
        if (status != MT6826S_OK) {
            return status;
        }
    }
    return mt6826s_decode(raw, sample);
}

static mt6826s_status_t write_bandwidth_register(const mt6826s_bus_t *bus, uint8_t value) {
    /* 写运行寄存器：0110+地址 0x011+数据；不发送 EEPROM 烧写命令 1100。 */
    const uint8_t tx[3] = {0x60, 0x11, value};
    uint8_t rx[3] = {0};
    bus->select(bus->context, true);
    const bool transferred = bus->transfer(bus->context, tx, rx, sizeof(tx));
    bus->select(bus->context, false);
    return transferred ? MT6826S_OK : MT6826S_ERROR_IO;
}

mt6826s_status_t mt6826s_set_bandwidth(const mt6826s_bus_t *bus, uint8_t expected_bandwidth,
                                    uint8_t target_bandwidth, mt6826s_bandwidth_change_t *change) {
    if (change == NULL) {
        return MT6826S_ERROR_ARGUMENT;
    }
    *change = (mt6826s_bandwidth_change_t) {0};
    if ((expected_bandwidth > 7U) || (target_bandwidth > 7U)) {
        return MT6826S_ERROR_ARGUMENT;
    }
    mt6826s_status_t status = mt6826s_read_register(bus, 0x011U, &change->before_register);
    if (status != MT6826S_OK) {
        return status;
    }
    change->before_read_ok = true;
    change->target_register = (change->before_register & 0xF8U) | target_bandwidth;
    const uint8_t current_bandwidth = change->before_register & 0x07U;
    if (current_bandwidth == target_bandwidth) {
        change->after_register = change->before_register;
        change->after_read_ok = true;
        change->verified = true;
        return MT6826S_OK;
    }
    if (current_bandwidth != expected_bandwidth) {
        return MT6826S_ERROR_CONFIGURATION;
    }
    change->write_attempted = true;
    status = write_bandwidth_register(bus, change->target_register);
    if (status != MT6826S_OK) {
        return status;
    }
    status = mt6826s_read_register(bus, 0x011U, &change->after_register);
    if (status != MT6826S_OK) {
        return status;
    }
    change->after_read_ok = true;
    change->verified = change->after_register == change->target_register;
    return change->verified ? MT6826S_OK : MT6826S_ERROR_CONFIGURATION;
}

mt6826s_status_t mt6826s_read_diagnostic_step(const mt6826s_bus_t *bus, unsigned step,
                                            mt6826s_diagnostics_t *diagnostics) {
    if (diagnostics == NULL || step >= 3U) {
        return MT6826S_ERROR_ARGUMENT;
    }
    static const uint16_t addresses[3] = {0x011U, 0x113U, 0x155U};
    uint8_t value;
    const mt6826s_status_t status = mt6826s_read_register(bus, addresses[step], &value);
    if (status == MT6826S_OK) {
        switch (step) {
            case 0U:
                diagnostics->bandwidth_register = value;
                diagnostics->bandwidth = value & 0x07U;
                break;
            case 1U:
                diagnostics->calibration_status_register = value;
                diagnostics->calibration_state = (value >> 6) & 0x03U;
                break;
            default: diagnostics->calibration_control = value; break;
        }
    }
    return status;
}

mt6826s_status_t mt6826s_read_diagnostics(const mt6826s_bus_t *bus, mt6826s_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return MT6826S_ERROR_ARGUMENT;
    }
    mt6826s_diagnostics_t result = {0};
    for (unsigned step = 0U; step < 3U; ++step) {
        const mt6826s_status_t status = mt6826s_read_diagnostic_step(bus, step, &result);
        if (status != MT6826S_OK) {
            return status;
        }
    }
    *diagnostics = result;
    return MT6826S_OK;
}
