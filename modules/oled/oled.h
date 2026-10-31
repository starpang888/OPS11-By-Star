/** @file oled.h
 * @brief modules 层 128×64 单色 OLED：控制器初始化、字符绘制与分页刷新，无 HAL/RTOS 依赖。
 */
#ifndef OLED_H
#define OLED_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OLED_WIDTH 128U
#define OLED_PAGES 8U
#define OLED_TEXT_COLUMNS 16U
typedef enum { OLED_SSD1306 = 0, OLED_SH1106 } oled_controller_t;
typedef struct {
    void *context;
    bool (*write)(void *context, uint8_t control, const uint8_t *data, size_t length);
    void (*yield)(void *context);
} oled_bus_t;
typedef struct {
    oled_bus_t bus;
    oled_controller_t controller;
    uint8_t pixels[OLED_PAGES][OLED_WIDTH];
} oled_t;

/* SSD1306 使用 128 列，SH1106 默认可见区从显存第 2 列开始。 */
bool oled_initialise(oled_t *oled, const oled_bus_t *bus, oled_controller_t controller);
void oled_clear(oled_t *oled);
/* 8×16 等宽字体：四行，每行最多 16 个字符；7×12 有效字形直接绘制。 */
void oled_text_line(oled_t *oled, unsigned line, const char *text);
/* 分 32 字节小块刷新整帧；显示失败立即返回，调用者决定何时重连。 */
bool oled_refresh(oled_t *oled);
#endif
