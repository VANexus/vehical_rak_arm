#ifndef APP_CANOPEN_H
#define APP_CANOPEN_H

#include <stdint.h>

/* 50 Hz 调用：把 CANopen RPDO 命令落到 app_arm，再把反馈写回 OD。
 * 由 task_arm_ctrl 调用。 */
void app_canopen_step(uint32_t now_ms);

#endif /* APP_CANOPEN_H */
