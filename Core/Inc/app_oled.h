#ifndef APP_OLED_H
#define APP_OLED_H

#include <stdint.h>

/* [AI]  SSD1306 OLED 调试显示（软 SPI：PD11=DC / PD12=RES / PD13=DAT / PD14=CLK）。
   惰性初始化：首次调用 oled_show_step 才动 GPIO/时序，不占 boot 路径时间（R3 教训）。
   机械臂板内容（A2）：y0 = 复位原因/运行秒数/build_tag，y10/20 = 六轴实际角
   （J1/J2 = 编码器回读），y30 = 模式机 + 指令流 + 指令异常计数，
   y40 = Home 安全位自检提示，y50 = J1/J2 电机轴状态/归零/故障/按键。 */

void oled_show_step(uint32_t now_ms);

/* [AI] A13：CAN 0x033 UI_CMD 用 —— 页面选择 / 对比度 / 反白 */
void oled_set_page(uint8_t page);      /* 0=自动 1=关节 2=IMU 3=诊断 4=自检条纹 */
void oled_set_contrast(uint8_t c);
void oled_set_invert(uint8_t on);   /* [AI] A9：10 Hz 调用，5 Hz 重绘 */   /* [AI]  10Hz 任务里调；内部 5Hz 重绘 */

#endif /* APP_OLED_H */
