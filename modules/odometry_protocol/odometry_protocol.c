/** @file odometry_protocol.c
 * @brief modules 层报文编码实现。手工输出整数和三位小数，避免依赖 printf 浮点格式化。
 */
#include "odometry_protocol.h"
#include <math.h>

typedef struct {
    char *data;
    size_t length, capacity;
    bool failed;
} writer_t;

static void put(writer_t *writer, char value) {
    /* 始终预留一个结尾 NUL 字节；容量不足时标记失败，不越界写入。 */
    if (writer->length + 1U >= writer->capacity) {
        writer->failed = true;
        return;
    }
    writer->data[writer->length++] = value;
}

static void number(writer_t *writer, uint64_t value) {
    char digits[20];
    unsigned count = 0U;
    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U) {
        put(writer, digits[--count]);
    }
}

static void fixed(writer_t *writer, double value) {
    if (!isfinite(value) || fabs(value) > 999999999.999) {
        writer->failed = true;
        return;
    }
    /* 先按千分之一单位四舍五入，再分离整数与小数部分；抑制输出负零。 */
    const uint64_t scaled = (uint64_t)(fabs(value) * 1000.0 + 0.5);
    if (value < 0.0 && scaled != 0U) {
        put(writer, '-');
    }
    number(writer, scaled / 1000U);
    put(writer, '.');
    put(writer, (char)('0' + scaled / 100U % 10U));
    put(writer, (char)('0' + scaled / 10U % 10U));
    put(writer, (char)('0' + scaled % 10U));
}

size_t odometry_protocol_encode(char *buffer, size_t capacity,
                                const odometry_packet_t *packet) {
    if (buffer == NULL || packet == NULL || capacity == 0U) {
        return 0U;
    }
    writer_t writer = {buffer, 0U, capacity, false};
    const char prefix[] = "$ODOM,";
    for (unsigned i = 0U; i < sizeof(prefix) - 1U; ++i) {
        put(&writer, prefix[i]);
    }
    number(&writer, packet->sequence); put(&writer, ',');
    number(&writer, packet->timestamp_us); put(&writer, ',');
    fixed(&writer, packet->x_mm); put(&writer, ',');
    fixed(&writer, packet->y_mm); put(&writer, ',');
    fixed(&writer, packet->yaw_deg); put(&writer, ',');
    number(&writer, packet->valid ? 1U : 0U); put(&writer, ',');
    number(&writer, packet->tracking_continuous ? 1U : 0U); put(&writer, ',');
    number(&writer, packet->state); put(&writer, ',');
    number(&writer, packet->error);
    uint8_t checksum = 0U;
    for (size_t i = 1U; i < writer.length; ++i) {
        checksum ^= (uint8_t)buffer[i];
    }
    const char hex[] = "0123456789ABCDEF";
    put(&writer, '*'); put(&writer, hex[checksum >> 4U]);
    put(&writer, hex[checksum & 15U]);
    put(&writer, '\r'); put(&writer, '\n');
    buffer[writer.length] = '\0';
    return writer.failed ? 0U : writer.length;
}
