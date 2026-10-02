# 快速上手（机械臂固件仓）

面向新成员（Windows / Linux）。**先读第 0 节再动手**——这个仓有一堆硬件未定项，盲目配 CubeMX 会白干。

---

## 0. 这个仓库是什么，以及现在不能做什么

`vehical_rak_arm` = **4 自由度机械臂**的 STM32F407VET6 固件（与底盘**完全同一块**轮趣 C30D 板，只是角色不同）。
它执行 Jetson 上 MoveIt 下发的关节目标，**自己不做 IK**；四个关节是 **MG513P30_12V 闭环减速电机（25 kHz 硬实时）**，底座旋转用 **1 个步进电机（1 kHz 位置环）**。

**当前进度**：CubeMX 骨架能编译，**应用代码 0 行**，且这个骨架目前与底盘工程**逐字节同构**（两边 `.bin` 的 SHA-256 相同）——也就是说还没有任何机械臂专属配置。

⛔ **在 [`README.md`](../README.md) §4 的"仍需确认"清单落定之前，不要动 `.ioc`**（第 1、2、3 条会直接决定外设怎么选）。

## 1. 三条与底盘不同的安全红线

| 规则 | 原因（已由用户确认） |
| --- | --- |
| **上电绝不自动动臂**；零位标定也必须由显式指令触发（`SAFETY_CMD = HOME_REQUEST`） | 会甩、会撞、可能夹手 |
| **降级 = 主动锁位，不是断电** | 臂**断电/停机会回落**（无抱闸 + 30:1 齿隙）。急停进 `ESTOP_HELD`：继续闭环抱当前角；只有 `RELEASE_HOLD` 才断使能，且必须人工支撑或已 parked |
| **零位靠低速堵转撞死点建立，且复位后作废** | 没有限位开关、没有绝对值编码器、板子上也没有电流采样。`homed` 标志不写 Flash——齿隙与机械变动会让存下来的零点骗人 |

零位标定的完整流程（单向趋近、低速低压、堵转判定、超时即失败）见 [`docs/PROTOCOL.md`](PROTOCOL.md) §7.5。

## 2. 工具链与环境（与底盘逐字相同）

```bash
. ../env/activate.sh          # Linux/macOS；Windows: . ..\env\activate.ps1
cmake --preset Debug && cmake --build --preset Debug
# → build/Debug/vehical_rak_arm.{elf,hex,bin,map}；Debug 基线 FLASH 16 140 B / RAM 43 400 B
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
```

只需 **STM32CubeCLT 1.22.0**（Linux 已在 `/opt/st/stm32cubeclt_1.22.0`）。两端版本必须一致，比对不要比字节，比 `.map` 的 section 与上板行为。

## 3. 代码结构约定

- 业务代码放 `Core/Src/app_*.c`，但 **L1 驱动 / L2 25 kHz 闭环 / L4 安全 / L5 通信来自 `platform/`**（两仓共用，尚未抽出）；本仓只写 `app_traj`（轨迹铺开）、`app_limit`（限位）、`app_step`（底座步进）。
- 新文件加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/` 那份会被 CubeMX 覆盖）。
- 只在 `USER CODE BEGIN/END` 标记对之间写。
- 帧宏不要手写：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`；提交前 `python3 tools/can_check.py` 必须退出码 0。

## 4. 上板检查清单

1. **先空载 / 支起臂**，第一次跑不要带工件；
2. 烧完**真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 上电），屏上确认 `build_tag`；
3. 量动力端子电压再上电（底盘曾被 2.89 V 过放坑成"假软件故障"）；
4. 单轴点动 → 全轴示教回放 → 才允许接 MoveIt 闭环；
5. 每轴记录：角度↔占空比/电流估算曲线、死区、齿隙迟滞带宽（这是"保持位置"能不能真做的前提）。

## 5. 文档索引

| 文件 | 作用 |
| --- | --- |
| [`README.md`](../README.md) | 本仓现状、**已确认/待确认硬件表**、协议角色、待办 |
| [`docs/PROTOCOL.md`](PROTOCOL.md) | CAN 协议 v3 规范（帧表、超时矩阵、homing 流程、故障码） |
| [`docs/protocol-signals.yaml`](protocol-signals.yaml) | 协议机器可读单一来源 |
| [`docs/BOARD-C30D.md`](BOARD-C30D.md) | 板级资源表（步进脉冲的合法落点：PC6=TIM8_CH1） |
| [`docs/gen/can_protocol.h`](gen/can_protocol.h) | 生成的帧宏 |
| 工作区根 [`../AGENTS.md`](AGENTS.md) | 跨端工具链与工程纪律 |

⚠️ `docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、`docs/gen/*`、`tools/can_check.py` 是同步产物，**不要在本仓直接改**。
