/** @file display_config.h
 * @brief application 层 OLED 参数：控制芯片、地址、刷新周期和显示任务栈。
 */
#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H
/* 用户已确认当前屏幕为 SSD1306、128×64、I2C 接口。 */
#define DISPLAY_CONTROLLER OLED_SSD1306
/* 0 表示仅在 0x3C/0x3D 两个常用地址检查应答；也可填固定 7 位地址。 */
#define DISPLAY_ADDRESS_7BIT 0U
#define DISPLAY_PERIOD_MS 500U
#define DISPLAY_RETRY_MS 1000U
#define DISPLAY_TASK_STACK_WORDS 512U
#endif
