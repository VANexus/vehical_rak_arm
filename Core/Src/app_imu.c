/*
 * app_imu.c — [AI] A7 机械臂板载 IMU（PB10=SCL / PB11=SDA 软 I2C，双芯片自动探测）
 *
 * 【分工（三颗 IMU 里本板这颗的位置）】
 *   前提：三颗 IMU **全部装在车体上**（本板也固定在底盘，不在机械臂上）。
 *   权威姿态在 Jetson 的 LSM6DSR（/dev/i2c-7，1 kHz，远离电机噪声）。
 *   本驱动**不做姿态融合**，只出三样东西：
 *     1) impact —— 冲击事件（静止时可作机械臂碰撞的**确认**证据）；
 *     2) GZ     —— 偏航角速率，与底盘板**同源冗余**（角速度与安装位置无关）；
 *     3) AMAG   —— 加速度模值（冲击强度；加速度与安装位置相关，不用于姿态）。
 *   跨板融合放在 Jetson（CAN 无同步时钟，MCU 侧不搞跨总线融合）。
 *
 * 【为什么用块读】
 *   逐寄存器读一次 14 字节要开 4 次 START；块读只开 2 次，软 I2C 上省一半时间。
 *   PB10/PB11 现在是 CubeMX 配的开漏 GPIO（软 I2C）；换硬件 I2C2 只需重写下面
 *   三个原语（imu_scl/imu_sda/imu_sda_r）与 imu_read_n()，对外接口不变。
 *
 * 【事实分级】
 *   - 寄存器/量程/换算与底盘板 `vehical_rak_motion/app_imu.c` 同套（R11 双芯片实测）；
 *   - 采样率 200 Hz、冲击阈值 1200 mg、低通 α=1/8 都是 **[AI] 待实测**初值；
 *   - 实测方法：bench 控制台按 `i` 看 imu_rate（×10）与 impact 计数。
 */
#include "app_imu.h"

#include "i2c.h"

#include "main.h"
#include "iwdg.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"

/* ======================= 配置（单点） ======================= */

#define IMU_SCL_PIN   10U   /* PB10 */
#define IMU_SDA_PIN   11U   /* PB11 */
#define IMU_I2C_DLY   60U   /* 半周期忙等。[AI] 待实测：越小编越块读越快，太小会被 EMI 吃掉 */

#define IMU_PERIOD_MS 5U    /* 目标 200 Hz */
#define IMU_ALPHA     0.125f  /* 加速度低通 α=1/8 @200 Hz → 约 4 Hz 拐点 */

#define IMU_IMPACT_DELTA_MG  1200.0f  /* 相对低通基线的瞬时偏差阈值（g 级冲击） */
#define IMU_IMPACT_HOLD_MS   300U     /* 冲击置位保持时长（供 100 Hz 帧一定能采到） */
#define IMU_SAT_MG           7000.0f  /* ±8g 量程，>7g 认为接近饱和 */
#define IMU_STILL_GYRO_DPS   5.0f
#define IMU_STILL_G_MG       120.0f
#define IMU_STALE_MS         20U

/* ---- 芯片 ---- */
enum { CHIP_NONE = 0, CHIP_ICM = 1, CHIP_MPU = 2 };

#define ICM_WHOAMI_REG   0x00U
#define ICM_WHOAMI_VAL   0xEAU
#define ICM_PWR_MGMT_1   0x06U
#define ICM_PWR_MGMT_2   0x07U
#define ICM_REG_BANK_SEL 0x7FU
#define ICM_ACCEL_XOUT_H 0x2DU    /* bank0: accel6 + temp2 + gyro6 = 14 B */

#define ICM_BANK2_GYRO_CONFIG_1 0x01U
#define ICM_BANK2_ACCEL_CONFIG  0x14U
#define ICM_GYRO_CFG_500DPS     0x33U   /* FS=±500dps, DLPF=3 */
#define ICM_ACCEL_CFG_8G        0x13U   /* FS=±8g,     DLPF=3 */

#define MPU_WHOAMI_REG   0x75U
#define MPU_WHOAMI_VAL   0x68U
#define MPU_PWR_MGMT_1   0x6BU
#define MPU_SMPLRT_DIV   0x19U
#define MPU_CONFIG       0x1AU
#define MPU_GYRO_CONFIG  0x1BU
#define MPU_ACCEL_CONFIG 0x1CU
#define MPU_ACCEL_XOUT_H 0x3BU    /* accel6 + temp2 + gyro6 = 14 B */

/* 换算：accel ±8g → 4096 LSB/g；gyro ±500dps → 65.5 LSB/dps */
#define IMU_LSB_PER_G     4096.0f
#define IMU_LSB_PER_DPS   65.5f

/* ======================= 状态 ======================= */

static uint8_t  s_chip;
static uint8_t  s_addr;
static uint8_t  s_acc_reg;

static volatile float    s_acc[3];        /* 低通后（mg） */
static volatile float    s_gyro[3];       /* dps */
static volatile uint16_t s_amag_mg;       /* 加速度模值（含重力，静止≈1000） */
static volatile uint8_t  s_evt;
static volatile uint8_t  s_age_ms;
static volatile uint8_t  s_impact_new;    /* 一次性，给 app_joint */
static volatile uint32_t s_impact_until_ms;
static volatile uint32_t s_last_ok_ms;
static volatile uint32_t s_fail_cnt;
static volatile uint16_t s_rate_x10;

static uint8_t  s_lp_primed;
static uint16_t s_still_ms;
static uint32_t s_rate_win_ms;
static uint16_t s_rate_cnt;
static uint32_t s_impact_cnt;

/* ======================= 延时 / 软 I2C（与底盘板同套） ======================= */

static void imu_delay_ms(uint32_t ms)
{
    (void)HAL_IWDG_Refresh(&hiwdg);          /* 探测期最坏 1 s 级，必须喂狗 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t span = ms * (SystemCoreClock / 1000U);
    while ((uint32_t)(DWT->CYCCNT - start) < span) { __NOP(); }
}

/* ---- [AI] A8：硬件 I2C2（CubeMX 生成：PB10=SCL / PB11=SDA，400 kHz）----
 * 软 I2C 已废弃：CubeMX 把这两脚配成 AF4 后，BSRR 位操作对它们无效。
 * 为何轮询不用 DMA：CubeMX 的 -q 脚本**设不了 NVIC**（CUBEMX-SOP §4 实测），
 * 没有 I2C2_EV/ER 中断时 HAL 的 DMA 路径收不了尾；轮询 14 字节约 0.36 ms @400 kHz，
 * 200 Hz 下占用 <10%。DMA 等 GUI 勾上两个 I2C2 中断再切。
 */
static int imu_write(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(s_addr << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, &val, 1U, 20U) == HAL_OK) ? 0 : 1;
}

static int imu_read_n(uint8_t reg, uint8_t *buf, int n)
{
    for (int tries = 0; tries < 2; ++tries) {
        /* [AI] A10 致命 bug 修复：每次失败重试都会阻塞 timeout(ms)，探测期最多
           25 轮 × 2 地址 × 3 次 × 20 ms ≈ 3 s >> IWDG 2 s → 复位循环，板子永远
           到不了 scheduler（OLED 全黑、CAN 全哑）。三条防线：
           1) 每次读之前喂狗（HAL 阻塞期间也不会饿死看门狗）；
           2) 超时降到 5 ms（400 kHz 下 14 字节只需 0.4 ms，5 ms 足够）；
           3) 重试 2 次。 */
        (void)HAL_IWDG_Refresh(&hiwdg);
        if (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(s_addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, buf, (uint16_t)n, 5U) == HAL_OK) {
            return 0;
        }
    }
    return 1;
}

static uint8_t imu_read1(uint8_t reg) { uint8_t v = 0U; (void)imu_read_n(reg, &v, 1); return v; }
static void    icm_bank(uint8_t b)   { (void)imu_write(ICM_REG_BANK_SEL, (uint8_t)(b << 4)); }

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ======================= 初始化 ======================= */

uint8_t imu_arm_init(void)
{
    s_chip = CHIP_NONE;
    s_addr = 0U;
    s_evt = 0U;
    s_impact_new = 0U;
    s_lp_primed = 0U;
    s_acc[0] = s_acc[1] = s_acc[2] = 0.0f;
    s_gyro[0] = s_gyro[1] = s_gyro[2] = 0.0f;

    /* [AI] A8：PB10/PB11 现在是 CubeMX 配的 I2C2 AF4 引脚（400 kHz） */
    imu_delay_ms(30);

    /* 双芯片探测：同地址 0x68/0x69，先认 ICM20948(0x00=0xEA) 再认 MPU6050(0x75=0x68) */
    for (int t = 0; t < 6; ++t) {      /* [AI] A10: 25 轮 → 6 轮，总预算 ≤1 s */
        for (unsigned k = 0U; k < 2U; ++k) {
            s_addr = (uint8_t)(0x68U + k);
            if (imu_read1(ICM_WHOAMI_REG) == ICM_WHOAMI_VAL) { s_chip = CHIP_ICM; break; }
            if (imu_read1(MPU_WHOAMI_REG) == MPU_WHOAMI_VAL) { s_chip = CHIP_MPU; break; }
        }
        if (s_chip != CHIP_NONE) break;
        s_addr = 0U;
        imu_delay_ms(20);
    }
    if (s_chip == CHIP_NONE) return 1U;   /* 没有芯片：任务会空转并在控制台报 CHIP=0 */

    if (s_chip == CHIP_ICM) {
        s_acc_reg = ICM_ACCEL_XOUT_H;
        icm_bank(0);
        (void)imu_write(ICM_PWR_MGMT_1, 0x80U);   /* DEVICE_RESET */
        imu_delay_ms(100);
        (void)imu_write(ICM_PWR_MGMT_1, 0x01U);   /* 自动选时钟 */
        (void)imu_write(ICM_PWR_MGMT_2, 0x00U);   /* 6 轴全开 */
        icm_bank(2);
        (void)imu_write(ICM_BANK2_GYRO_CONFIG_1, ICM_GYRO_CFG_500DPS);
        (void)imu_write(ICM_BANK2_ACCEL_CONFIG,  ICM_ACCEL_CFG_8G);
        icm_bank(0);
    } else {
        s_acc_reg = MPU_ACCEL_XOUT_H;
        (void)imu_write(MPU_PWR_MGMT_1, 0x80U);
        imu_delay_ms(100);
        (void)imu_write(MPU_PWR_MGMT_1, 0x01U);
        (void)imu_write(MPU_SMPLRT_DIV, 0x04U);   /* 1 kHz/(1+4) = 200 Hz */
        (void)imu_write(MPU_CONFIG, 0x03U);       /* DLPF 44 Hz */
        (void)imu_write(MPU_GYRO_CONFIG, 0x08U);  /* ±500 dps */
        (void)imu_write(MPU_ACCEL_CONFIG, 0x10U); /* ±8 g */
    }

    s_last_ok_ms = HAL_GetTick();
    return 0U;
}

/* ======================= 对外快照 ======================= */

void imu_arm_accel_mg(float acc[3])
{
    acc[0] = s_acc[0]; acc[1] = s_acc[1]; acc[2] = s_acc[2];
}

void imu_arm_gyro_dps(float gyro[3])
{
    gyro[0] = s_gyro[0]; gyro[1] = s_gyro[1]; gyro[2] = s_gyro[2];
}

uint8_t imu_arm_events(void)     { return s_evt; }
uint8_t imu_arm_age_ms(void)     { return s_age_ms; }
uint16_t imu_arm_amag_mg(void)   { return s_amag_mg; }
uint16_t imu_arm_rate_x10(void)  { return s_rate_x10; }
uint32_t imu_arm_fail_cnt(void)  { return s_fail_cnt; }
uint8_t  imu_arm_chip(void)      { return s_chip; }

uint8_t imu_arm_take_impact(void)
{
    uint8_t v = s_impact_new;
    s_impact_new = 0U;
    return v;
}

/* ======================= 采样任务（200 Hz） ======================= */

void imu_arm_task(void *argument)
{
    (void)argument;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        uint8_t buf[14];
        uint32_t now = HAL_GetTick();

        if (s_chip != CHIP_NONE) {
            if (imu_read_n(s_acc_reg, buf, 14) == 0) {
                /* 小端序大端：芯片是高字节在前 */
                float ax = (float)be16(&buf[0]) * (1000.0f / IMU_LSB_PER_G);
                float ay = (float)be16(&buf[2]) * (1000.0f / IMU_LSB_PER_G);
                float az = (float)be16(&buf[4]) * (1000.0f / IMU_LSB_PER_G);
                float gx = (float)be16(&buf[8])  / IMU_LSB_PER_DPS;
                float gy = (float)be16(&buf[10]) / IMU_LSB_PER_DPS;
                float gz = (float)be16(&buf[12]) / IMU_LSB_PER_DPS;

                if (s_lp_primed == 0U) {          /* 首样本直接进基线，防开机假冲击 */
                    s_acc[0] = ax; s_acc[1] = ay; s_acc[2] = az;
                    s_lp_primed = 1U;
                }

                /* 冲击：当前值相对低通基线的瞬时偏差 */
                float dx = ax - s_acc[0];
                float dy = ay - s_acc[1];
                float dz = az - s_acc[2];
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 > (IMU_IMPACT_DELTA_MG * IMU_IMPACT_DELTA_MG)) {
                    s_impact_until_ms = now + IMU_IMPACT_HOLD_MS;
                    s_impact_new = 1U;
                    s_impact_cnt++;
                }

                /* 低通（α=1/8）：这就是重力方向 */
                s_acc[0] += (ax - s_acc[0]) * IMU_ALPHA;
                s_acc[1] += (ay - s_acc[1]) * IMU_ALPHA;
                s_acc[2] += (az - s_acc[2]) * IMU_ALPHA;
                s_gyro[0] = gx; s_gyro[1] = gy; s_gyro[2] = gz;

                /* 事件位 */
                uint8_t evt = 0U;
                if ((int32_t)(now - s_impact_until_ms) < 0) evt |= IMU_EVT_IMPACT;
                if (d2 > (IMU_SAT_MG * IMU_SAT_MG)) evt |= IMU_EVT_SATURATED;

                /* 静止：|a| 接近 1 g 且三轴陀螺都很小，持续 300 ms */
                float amag = s_acc[0] * s_acc[0] + s_acc[1] * s_acc[1] + s_acc[2] * s_acc[2];
                float am = 0.0f;
                {   /* 快速近似 |a|（避免每次调 sqrtf 链）*/
                    float x = amag;
                    float g = (x > 1.0f) ? x : 1.0f;
                    for (int i = 0; i < 6; ++i) g = 0.5f * (g + x / g);
                    am = g;
                }
                float gmag = gx * gx + gy * gy + gz * gz;
                uint8_t still_now = (uint8_t)(
                    (gmag < (IMU_STILL_GYRO_DPS * IMU_STILL_GYRO_DPS)) &&
                    (am > (1000.0f - IMU_STILL_G_MG)) && (am < (1000.0f + IMU_STILL_G_MG)));
                if (still_now != 0U) {
                    if (s_still_ms < 1000U) s_still_ms++;
                } else {
                    s_still_ms = 0U;
                }
                if (s_still_ms >= (300U / IMU_PERIOD_MS)) evt |= IMU_EVT_STILL;

                s_evt = evt;
                s_amag_mg = (uint16_t)((am > 65535.0f) ? 65535.0f : am);
                s_last_ok_ms = now;

                /* 实测采样率（1 s 窗） */
                s_rate_cnt++;
                if ((uint32_t)(now - s_rate_win_ms) >= 1000U) {
                    s_rate_x10 = (uint16_t)(s_rate_cnt * 10U);
                    s_rate_cnt = 0U;
                    s_rate_win_ms = now;
                }
            } else {
                s_fail_cnt++;
            }
        }

        /* 样本龄 + stale 位 */
        uint32_t age = (uint32_t)(HAL_GetTick() - s_last_ok_ms);
        s_age_ms = (uint8_t)((age > 255U) ? 255U : age);
        if (age > IMU_STALE_MS) {
            s_evt = (uint8_t)(s_evt | IMU_EVT_STALE);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}
