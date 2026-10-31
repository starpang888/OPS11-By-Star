/** @file odometry_protocol.h
 * @brief modules 层位姿文本协议：将位姿快照编码为固定字段的 ASCII 报文，不负责发送。
 */
#ifndef ODOMETRY_PROTOCOL_H
#define ODOMETRY_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ODOMETRY_PROTOCOL_CAPACITY 112U
typedef struct {
    uint32_t sequence, timestamp_us;
    double x_mm, y_mm, yaw_deg;
    bool valid, tracking_continuous;
    uint8_t state, error;
} odometry_packet_t;

/* 格式：$ODOM,seq,t_us,x_mm,y_mm,yaw_deg,valid,continuous,state,error*HH\r\n
   校验为 '$' 后到 '*' 前的字节逐个异或；坐标和角度保留三位小数。
   返回不含结尾 NUL 的字节数；数值无效或容量不足时返回 0。 */
size_t odometry_protocol_encode(char *buffer, size_t capacity,
                                const odometry_packet_t *packet);
#endif
