/*
 * app_build.c — 固件构建标识（单点定义）
 *
   * A16（2026-10-30 第二改）：CAN 静默自愈（SLAK 清除 + 100ms 滞留邮箱 ABRQ）——
 *   台架实录一次"SLAK=1 + 三邮箱滞留 → 总线静默"（CAN-BRINGUP-2026-10-30.md §3.2），
 *   与底盘 R51 的自愈先例同一思路。
 * A15（2026-10-30）：**v4 整体下线**——删 `app_can.c/h` 与 `APP_USE_CANOPEN` 开关
 *   （本板协议只允许 v5 一套）；接口收敛进 `app_canopen.h`（历史接口名保留，
 *   语义全由 CANopenNode 承担）。行为不变，但"还能编回 v4"这条路不存在了。
 * A14（2026-10-29）：+ 桥接可观测量（0x2150 cmd_en / 0x2151 cmd_age_ms /
 *   0x2154 joint_fault）与设备标识 0x2F00 tag_packed / 0x2F01 reset_cause。
 *   注：CiA 推荐的 0x1008/1009/100A（VISIBLE_STRING）实测触发 HardFault
 *   （CANopenNode v4 + 本移植层组合，见 docs/CANOPEN-V5.md §7 坑 4），已回退；
 *   名称/厂商信息暂由 EDS [DeviceInfo] 承担。
 *
 * A13（2026-10-29）：**CANopen v5 P1**。27 个厂商对象；RPDO1/2=七轴+夹爪命令、
 *   RPDO3=命令序号/远程门；TPDO1/2=关节回读、TPDO3=IMU、TPDO4=状态；50 Hz 桥接。
 *
 * A12（2026-10-29）：**CANopen v5 P0**。CAN 层换成 CANopenNode v4 + 官方 STM32
 *   移植层（node 2，1 Mbps，上电 Pre-operational）；v4 app_can.c 经 APP_USE_CANOPEN
 *   开关保留可回滚。臂帧命令/SRDO 在 P1/P2 接回。
 *
 * A11（2026-10-29）：**OLED 驱动回归软 SPI + 推挽输出**。板级文档（BOARD-C30D §3）与
 *   原理图都是 PD11=DC / PD12=RES / PD13=DATA / PD14=CLK 的 4 线软 SPI；PD11-14 由
 *   开漏改推挽（.ioc + gpio.c 同步）。A8 起没有 OLED 代码、A10 停在 A8 的屏黑状态。
 *
 * A10（2026-10-29）：**修复致命启动复位循环**。HAL I2C 每次失败重试阻塞 20 ms 且不喂狗，
 *   探测期 25×2×3×20ms ≈3 s > IWDG 2 s → 板子到不了 scheduler（OLED 全黑、CAN 全哑）。
 *   修法：读前后喂狗、超时 20→5 ms、重试 3→2、探测轮数 25→6。
 *
 * A9（2026-10-29）：**恢复 OLED 显示**（用户澄清：机械臂板两个预留口装的是与底盘同款
 *   SSD1306）。app_oled 从 A5 删除前的版本恢复（软 SPI：PD11=DC / PD12=RES /
 *   PD13=DAT / PD14=CLK），内容改为 A9 版：构建标签/运行秒数、J1-J6 角度、
 *   模式/指令流/急停/舵机异常、**IMU 行（芯片/实测 Hz/|a|/碰撞计数）**、
 *   J1/J2 电机轴状态（B/H/M/F + 归零 + 故障 + PE0）。10 Hz 任务调、5 Hz 重绘。
 *
 * A8（2026-10-29）：**外设全部改由 CubeMX 生成**。I2C2（PB10=SCL/PB11=SDA，400 kHz）替代
 *   软 I2C（AF 引脚上 BSRR 写无效）；app_imu 改 HAL_I2C_Mem_Read/Write 轮询
 *   （CubeMX 的 -q 脚本设不了 NVIC → 暂不上 DMA）；新增 app_buzz（PA8 蜂鸣器，
 *   非阻塞状态机：上电自检/JOG/IMPACT/FAULT）；PD11-14 保持低电平 GPIO 输出（**用户 2026-10-29 决定不用 OLED**，故不加显示驱动；
 *   这四根是软 SPI 的 GPIO，不占硬件 SPI 外设）。
 *
 * A7（2026-10-29）：**三颗 IMU 全部启用（车体三源）**。前提按用户确认修正：三颗 IMU
 *   全装在车体上（本板也固定在底盘，不在机械臂上）→ 本板这颗是同一刚体上的第二颗：
 *     - 刚体角速度与安装位置无关 → 0x222.ARM_IMU 的 GZ 与底盘板 0x032.DYAW **可互校**；
 *     - 加速度只取模值/冲击（受安装位置影响，不做姿态）。
 *   新增 app_imu（PB10/PB11 软 I2C，双芯片探测，200 Hz，14 字节块读）；新增 0x222 ARM_IMU
 *   （100 Hz）；订阅底盘板 0x202 WHEEL_SPD 判“车在动”；机械臂碰撞确认 = 车静止 + 冲击
 *   （车在动时只报 bump）。bench 控制台新增 i=IMU 状态、j=ISR 占用率。
 *   协议仅加法（0x032/0x222），既有帧一字未动。
 *
 * A6（2026-10-29）：**J1/J2 防撞准备 + 台架串口控制台**。
 *   1) app_joint：软限位改成**每轴可关**（J1 连续旋转、J2 行程未实测 → 默认都关）；
 *      新增“给足占空比却不动”碰撞/堵转判定：点动模式→刹车并上报（不锁故障），
 *      位置环模式→刹车 + 锁 FAULT；新增开环慢速点动 `joint_jog()`（1.2 s 自动刹）。
 *   2) 新增 app_bench：USART1(USB-CH9102F) 单字符控制台 —— 1/2/3/4 点动 J1±/J2±、
 *      0 停、+/- 调占空比（5..60%）、p 打印、m 监视、k 置零、c 清故障；
 *      串口写入口统一为 bench_write()（互斥 + USART1/USART3 双路），CAN 诊断也走它。
 *   3) 协议、帧 ID、字节布局**一字未动**（0x110/111、0x210/211、0x031、0x221）。
 * A5（2026-10-29）：**移除 OLED**（app_oled.c/h 删除，PD11-14 四根 GPIO 释放，
 *   调试信息全部改由 USART1/USART3 输出）；J1/J2 接线映射按实物确认：
 *   **TJ1 = 电机 A（J1/腰）、TJ2 = 电机 B（J2/大臂）**，代码 {A,B} 无需改。
 *   同时实测确认：机械臂板改用**板载 CAN 收发器**（拆掉外挂 TJA1051）后，
 *   三节点 CAN 全通 —— 收帧正常、txfail=0、ESR=0。
 * A4（2026-10-29）：诊断打印同时走 USART1 与 USART3（C30D 有两个 CH9102F
 *   USB 口，台架插的是哪一个不确定，两路都打）。
 * A3（2026-10-29）：新增 CAN 板间互通诊断（app_can_diag.c）—— 1 Hz 经 USART1
 *   打印收帧/发帧/ESR 统计，让"两块 C30D 之间 CAN 通不通"不依赖 Jetson 就能判定。
 *   只加统计与打印，**不改任何帧 ID、字节布局与控制行为**。
 * A2（2026-10-28）：J1(腰)/J2(大臂) 从舵机换成"轮子同款"MG513P30 增量编码器电机，
 *   新增 app_joint（25 kHz 采样 + 1 kHz 定点位置 PI + 软件归零 + 急停刹车）；
 *   J3..J6 仍是舵机。CAN 0x110/111 与 0x210/211 的 ID 与字节布局**一字未动**
 *   （Jetson 侧零改动）；CLEAR_FAULT(0x010 cmd=6) 现在会复位 J1/J2 位置环故障。
 *   改动前（A1）：机械臂板首版——6 路舵机 PWM + CAN v4 臂帧。
 */
#include "app_build.h"

const char build_tag[] = "A16";
