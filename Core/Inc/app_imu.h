#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

/*
 * app_imu.h — [AI] A7 机械臂板载 IMU（PB10=SCL / PB11=SDA 软 I2C）
 *
 * 【前提（2026-10-29 用户确认，必须记住）】
 *   **三颗 IMU 全部装在车体上**：底盘板一颗、机械臂板一颗（这块板也固定在底盘上）、
 *   Jetson 一颗（LSM6DSR，权威姿态源）。所以本板这颗**不是“臂端 IMU”**，
 *   不能用来测关节角、也不能感机械臂末端的重力方向 —— 它和底盘板那颗测的是
 *   同一个刚体的运动，只是安装位置/基向量/受扰环境不同。
 *
 * 【刚体上的物理事实（决定了它能干什么）】
 *   - **角速度与安装位置无关**：刚体上任意点的 ω 相同 →
 *     本板的陀螺 Z 与底盘板陀螺 Z 是**同源冗余**，可逐样本互校（谁坏了、谁被 EMI 打了）；
 *   - **加速度与安装位置有关**（a = a_o + α×r + ω×ω×r）：
 *     所以本板加速度只能用于“模值/冲击”，不参与姿态解算。
 *
 * 【三件产出】
 *   1) impact（冲击/碰撞确认）：|a| 相对慢变基线突变 → 置位 300 ms；
 *      机械臂碰撞的**确认条件**是“车静止 + 冲击尖峰 + 轮速为零”（见 app_bench），
 *      车在动时只算 bump 事件，不参与任何控制判决；
 *   2) 偏航角速率 GZ：与底盘板互校（0x222 上传）；
 *   3) 加速度模值 AMAG：冲击强度，给 Jetson 做事件强度分级。
 *
 * 【事实分级】
 *   - 芯片型号**两块板不同**（R11 实测：一块 MPU6050 0x68，一块 ICM20948 0xEA），
 *     本驱动做双芯片自动探测，与底盘板 `vehical_rak_motion/app_imu.c` 同 套寄存器；
 *   - 采样率 200 Hz、冲击阈值、量程换算系数都是 **[AI] 待实测**初值，见 .c 顶部；
 *   - PB10/PB11 目前是 CubeMX 配的**开漏 GPIO**（软 I2C）。将来若改硬件 I2C2
 *     （AF4 + DMA），只需换掉 .c 里的读写原语，对外接口不变。
 */

/* 上电初始化（含 30 ms 稳定 + 双芯片探测；会喂 IWDG）。返回 0 = 找到芯片 */
uint8_t imu_arm_init(void);

/* 采样任务主体（由 freertos.c 的 task_imu 调用，不返回）。目标 200 Hz。 */
void imu_arm_task(void *argument);

/* ---- 只读快照（任务写、其它任务读）---- */
void    imu_arm_accel_mg(float acc[3]);     /* 低通后的加速度（mg，本板基向量） */
void    imu_arm_gyro_dps(float gyro[3]);    /* 角速度（°/s，未去零偏） */
uint16_t imu_arm_amag_mg(void);             /* 加速度模值（mg，含重力；静止≈1000） */

/* 事件位图（= CAN 0x222 EVT_u8 位定义，见 docs/PROTOCOL.md §3.1） */
#define IMU_EVT_IMPACT     0x01U
#define IMU_EVT_SLIP       0x02U   /* 本板不用（打滑判定在底盘板） */
#define IMU_EVT_STILL      0x04U
#define IMU_EVT_SATURATED  0x08U
#define IMU_EVT_STALE      0x10U

uint8_t  imu_arm_events(void);
uint8_t  imu_arm_age_ms(void);              /* 样本龄（饱和 255） */

/* 冲击确认：读取并清“新冲击”标志（消费方负责判断车是否静止） */
uint8_t  imu_arm_take_impact(void);

/* 诊断：实测采样率 ×10（例如 1983 = 198.3 Hz）、失败计数、芯片类型（0=无 1=ICM 2=MPU） */
uint16_t imu_arm_rate_x10(void);
uint32_t imu_arm_fail_cnt(void);
uint8_t  imu_arm_chip(void);

#endif /* APP_IMU_H */
