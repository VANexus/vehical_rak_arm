# 快速上手（机械臂板固件仓 vehical_rak_arm）

> ⚠️ **v4 已于 2026-10-30 整体下线。** 本文中所有对 `docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、
> `tools/can_check.py`、`docs/gen/can_protocol.h`、`APP_USE_CANOPEN` 的引用**全部作废**——
> 这些文件已删除，固件里没有 v4 分支。协议唯一规范是 [`docs/CANOPEN-V5.md`](CANOPEN-V5.md)（README 同目录），
> 门禁是 `python3 tools/canopen_gate.py --repo .`。历史说明保留原文只为追溯，**不要照着执行**。



面向新成员（Windows / Linux）。**先读第 0 节再动手**。

---

## 0. 这个仓库是什么

`vehical_rak_arm`（2026-11-14 前叫 `vehical_rak_edge_ai`）= 轮趣 C30D（STM32F407VET6）
上的 **6 路舵机机械臂控制器**固件。与底盘仓 `vehical_rak_motion`（4 轮 25 kHz 闭环 + IMU）
**完全同一块硬件**，只角色不同；上位机是 `VANexus/rak`（Jetson，负责 IK/轨迹规划）。

三节点同挂一条 **CAN 2.0A @1 Mbps** 总线：Jetson + 底盘板 + 机械臂板（本仓）。

**当前进度**：**A1**（首版机械臂板固件）——6 路舵机 PWM（TIM8 4 路 / TIM12 2 路 @50 Hz）
+ CAN v4 臂帧收发 + OLED + IWDG + FreeRTOS；Debug/Release 双预设零警告；**待上板实测**。
`build_tag` 单点定义在 `Core/Src/app_build.c`。

## 1. 本仓硬规矩

| 规则 | 说明 |
| --- | --- |
| **上电先出 Home 安全位** | `servo_init()` 输出 500/2166/2500/500/500/500 µs；测试时臂行程内不得有人/物 |
| **急停 = 冻结** | `0x010 SAFETY_CMD` 的 `EMERGENCY_STOP` 锁存后拒绝落地 ARM 指令（舵机保持当前位置） |
| **1 Mbps CAN 必须 HSE 8 MHz** | HSI 精度不够，1 Mbps 会偶发错误帧，极难定位 |
| **ISR 内禁止浮点/除法/`printf`/阻塞** | 共享数据归 ISR 单一写者，任务只读 |
| **不做 IK/轨迹规划** | 那在 Jetson 的 MoveIt；本板只做「指令角 → 脉宽」+ 本地安全兜底 |

## 2. 工具链与环境（与底盘仓逐字相同）

```bash
. ./env/activate.sh          # Linux/macOS；Windows: . .\env\activate.ps1
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# → build/<预设>/vehical_rak_arm.{elf,hex,bin,map}
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
```

只需 **STM32CubeCLT 1.22.0**（Linux 已在 `/opt/st/stm32cubeclt_1.22.0`）。
跨端比对用 **`.bin`/`.hex` 的 SHA-256**（CubeCLT 14.3.rel1 两端逐字节相同）；**不要比 `.elf`**（内嵌绝对路径）。

## 3. 代码结构约定

- 应用代码在 `Core/Src/app_*.c`：`app_arm`（语义）/`app_servo`（舵机驱动）/`app_can`（通信）/
  `app_oled`（调试显示）/`app_build`（构建标识）。**外设配置一律走 CubeMX**（见 `docs/CUBEMX-SOP.md`）。
- 新文件加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/` 那份会被 CubeMX 覆盖）。
- 只在 `USER CODE BEGIN/END` 标记对之间写。
- 帧宏不要手写：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`；
  提交前 `python3 tools/can_check.py` 必须退出码 0。

## 4. 上板检查清单

1. **先架空 / 臂行程内无人**，第一次跑不要带负载；
2. 烧完**真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 上电），屏上确认 `build_tag`；
3. 确认 6 路舵机停在上电 Home 安全位（未收到指令前不应乱甩）；
4. 每次实测记录：`.bin` SHA-256、供电电压、六轴终态、目视现象。

## 5. 文档索引

| 文件 | 作用 |
| --- | --- |
| [`README.md`](../README.md) | 本仓现状、舵机地图、双板架构、待办 |
| [`docs/PROTOCOL.md`](PROTOCOL.md) | CAN 协议 v4 规范（帧表、超时矩阵、故障码） |
| [`docs/protocol-signals.yaml`](protocol-signals.yaml) | 协议机器可读单一来源 |
| [`docs/BOARD-C30D.md`](BOARD-C30D.md) | 板级资源表（CAN1 = PD0/PD1 板载收发器；无电流采样） |
| [`docs/gen/can_protocol.h`](gen/can_protocol.h) | 生成的帧宏 |
| [`docs/CUBEMX-SOP.md`](CUBEMX-SOP.md) | 改外设前必读 |
| 工作区根 [`AGENTS.md`](../AGENTS.md) | 跨端工具链与工程纪律 |

约定：`docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、
`docs/gen/can_protocol.h`、`tools/can_check.py`、`env/activate.*` 在本仓各存一份
（与姊妹仓内容保持一致，改动时同一批 `cp` 过去）。
