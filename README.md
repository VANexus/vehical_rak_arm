# vehical_rak_arm — 机械臂固件（STM32F407VET6）

4 自由度机械臂控制板：**4 个与底盘同型号的闭环减速电机**，跑 **25 kHz 硬实时闭环**；接收 MoveIt 下发的关节轨迹并执行，回报关节角。

工作区总览看 [`../README.md`](../README.md)，行为约束看 [`../AGENTS.md`](../AGENTS.md)，**分层设计看 [`../docs/architecture.md`](../docs/architecture.md)**，**协议看 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)**。本文件只讲**这个工程**。

**新成员先读 [`docs/QUICKSTART.md`](docs/QUICKSTART.md)**（含机械臂特有的三条安全红线）。

### 仓库结构

本目录是**独立 git 仓**（`Drivers/`、`Middlewares/` 已入库，可单独 clone 构建）：`docs/QUICKSTART.md` 上手指南；`docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`、`tools/can_check.py` **均为 `../tools/sync_docs.py` 同步来的产物，不要在本仓改**（只改工作区根的那一份再同步）。

---

**远端**：`https://github.com/VANexus/vehical_rak_arm`（private，默认分支 `main`，团队 `RAK-嵌入式开发组` 有 maintain 权限）。
协作三步：**开工先 `git pull`** → 改动 → **收工 `git add -A && git commit && git push`**；提交前本仓必须 `python3 tools/can_check.py` 与 `cmake --build --preset Debug` 双双零问题。

---

## 1. 当前状态：还是底盘工程的复制品，机械臂专属配置 = 0

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，目标 `STM32F407VETx`，只开了 FreeRTOS + SWD |
| Debug / Release 构建 | ✅ 已在 Linux 实测通过（CubeCLT 14.3.rel1）；Debug = FLASH 16 140 B（3.08%）/ RAM 43 400 B（33.11%） |
| 与底盘工程的差异 | ❌ **没有**。`.ioc` 只差 `ProjectName`/`ProjectFileName` 两行，`Core/Src/main.c` 逐字节相同，两边 `.bin` 的 SHA-256 相同（`b2357e7c…`） |
| 关节外设 / 应用代码 | ❌ 全部未配置、未开始 |

好消息是：既然执行器与底盘同类（4 × 闭环减速电机 + 25 kHz），**本工程与底盘的外设需求完全同构**，它不该是从零造一台新机器，而是 `platform/` 共用层的第二个用户，只多一层关节位置外环。

---

## 2. 架构定位（职责边界，别越层）

| 事情 | 归谁 | 本工程的角色 |
| --- | --- | --- |
| 正解 / 逆解（IK）、轨迹规划 | **Jetson 的 MoveIt** | ❌ 不做。本板只收关节目标并执行 |
| SLAM、Nav2、IMU 融合 | **Jetson**（激光雷达 / 深度相机 / IMU 直连 Jetson） | ❌ 不做 |
| 关节闭环 | **本板** | ✅ 25 kHz ISR：位置外环 + 速度内环（定点 PI，限幅 85%） |
| 轨迹插值 | 本板 L3 | ✅ `app_traj`：把 100 Hz 的 MoveIt 目标点铺成 25 kHz 位置参考 |
| 本地安全兜底 | 本板 L4 | ✅ `app_limit`：限位、急停锁存、断流超时、堵转 |

分层细节与模块清单见 [`../docs/architecture.md`](../docs/architecture.md) §3；L1 驱动 / L2 实时 / L4 安全 / L5 通信 与底盘**共用同一份源码**（`../platform/`，Phase 3 建立），板级差异收在每工程各一份的 `board_c30d.h`。

---

## 3. 构建与烧录（与底盘工程逐字相同）

```bash
. ../env/activate.sh                 # Windows: . ..\env\activate.ps1
cmake --preset Debug && cmake --build --preset Debug
# → build/Debug/vehical_rak_arm.{elf,hex,bin,map}（hex/bin 由顶层 CMakeLists.txt 的 POST_BUILD 生成）

STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
```

烧录后必须**真断电冷启动**并确认 `build_tag`（[`../AGENTS.md`](../AGENTS.md) §5）。

---

## 4. 已确认的，与仍需确认的

**已确认**：

| 项 | 结论 | 来源 |
| --- | --- | --- |
| 关节执行器 | **4 个与底盘同型号的闭环减速电机（MG513P30_12V，30:1）**，**不是舵机** | 用户 2026-10-02 确认（修正上一轮"4 关节电机 + 步进"的含糊说法） |
| 控制频率 | 同样 **25 kHz 硬实时闭环**；**底座旋转额外用 1 个步进电机**（1 kHz 位置环，不进 25 kHz） | 同上 + 设计草案第二份 |
| 底板 | **与底盘完全同一块 C30D**（板载 4 路 AT8236 + 板载 CAN 收发器），只是角色不同 | 用户确认 |
| 步进脉冲引脚 | **PC6 = TIM8_CH1**（脉冲）+ 任一空闲 GPIO（方向）；驱动器外接，不占板载 AT8236 通道 | [`../docs/board-c30d.md`](../docs/board-c30d.md) §4：板上 **PC6-9 + PB14/15 = TIM8/TIM12 航模/舵机接口**，来路是官方资源分配 PDF |
| 夹爪/关节之外无电流采样 | 全板**没有通道电流采样** → 草案的"电流检测"不能直接做，堵转改用软件判据 | 同上 §3 |

**仍需确认（配 CubeMX 前逐条落实，落实一条就写回本表）**：

| # | 问题 | 为什么必须先定 |
| --- | --- | --- |
| 1 | 夹爪用什么驱动？板载只有 **4 路** AT8236，已被 4 个关节占满 → 夹爪接哪里（舵机口 PB14/TIM12？预留 IO + 小驱动芯片？还是根本没有夹爪） | 决定第 5 路输出的物理来源；不定则 Phase B 外设配不完 |
| 2 | 步进**驱动器型号**与细分设定、方向/使能脚选哪几个空闲 IO、有无编码器反馈（草案说"定时器计数脉冲数"——那是**开环**，没有真实位置） | 决定 `app_step` 能不能做位置闭环、`0x102` 的底座角是否只是推算值 |
| 3 | 关节有无限位开关 / 回零基准（限位、霍尔，还是靠电流特征——但本板无电流采样） | 决定上电流程。**禁止上电自动动作**（§7），这条不定就不能写启动序列；也决定 `ARM_JOINT_CMD_*` 的角度零点与 MoveIt 的 URDF 原点 |
| 4 | 臂断电/停机时会不会自落（无抱闸 + 30:1 齿隙） | 直接决定协议里"断流保持位置"这条安全语义能不能真做到（[`../docs/protocol.md`](../docs/protocol.md) §7.2） |
| 5 | 供电拓扑：动力与逻辑是否分开、共地、每关节峰值电流 | 底盘台架期已被 2.89 V 过放坑过一次；臂堵转时电流更大 |
| 6 | 关节角反馈只在电机端（无输出端绝对位置）→ MoveIt 的位置精度靠什么保证 | 30:1 齿隙 + 编码器在输入侧 ⇒ 输出角有迟滞；决定要不要单向趋近策略与零位标定 |

查答案顺序：实物接线与板卡丝印 → [`docs/BOARD-C30D.md`](docs/BOARD-C30D.md)（已从 PDF 提取）→ 原理图 PDF（`../car2/tools/references/.../装配参考资料/手册文档/`）→ 问用户。**不要凭记忆把引脚写进 `.ioc`**。

> 用户 2026-10-02 指示：**夹爪（#1）与步进驱动器细节（#2）“后面再说”**——不阻塞 4 个关节的 Phase B，但底座步进只能先按“能发脉冲、位置为推算值”实现，**不要把推算值当实测反馈上报**。#3（零位）已定：靠低速堵转找死点，流程见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md) §7.5；#4（会回落）已确认，影响见 §7。

---

## 5. 协议角色（本板侧）

规范件是 [`../docs/protocol.md`](../docs/protocol.md) v3，机器可读定义是 [`../docs/protocol-signals.yaml`](../docs/protocol-signals.yaml)。**本表只列本工程收发的帧，不重复字节布局**：

| ID | 方向 / 频率 | 本工程职责 |
| --- | --- | --- |
| `0x101` `ARM_JOINT_CMD_A` | Jetson→臂 @100 Hz | J1-J3 目标角（0.01° int16）+ enable/counter/CRC |
| `0x102` `ARM_JOINT_CMD_B` | Jetson→臂 @100 Hz | J4 目标角 + **底座目标角（0.01° uint16，覆盖 360°）** |
| `0x010` `SAFETY_CMD` | 广播，事件 | **本板必须响应**：急停立即停止并**锁存**，仅 `CLEAR_ESTOP` 可解；不是"只有底盘关心" |
| `0x030` `XFER_CH_TO_ARM` | 底盘→臂 @1 kHz | 接收实测 Vx/Vy/ωz，做前馈补偿与互锁（**10 ms 无帧→本板 FAULT**） |
| `0x031` `XFER_ARM_TO_CH` | 臂→底盘 @1 kHz | 发送故障码 + J4 + 底座角 |
| `0x210` `ARM_JOINT_FB_A` / `0x211` `ARM_JOINT_FB_B` | 臂→Jetson @100 Hz | 回报实测关节角 |
| `0x212` `ARM_DIAG` | 臂→Jetson @10 Hz | 电池电压 + 故障码 |
| `0x021` `EMCY_ARM` | 臂→广播，事件 | 故障上报（首发 ≤10 ms，重发 3 次） |
| `0x231` `NODE_INFO_ARM` | 臂→广播 | 上电 1 次 + 1 Hz：协议版本 + 固件构建标识 |

关键点（与草案不同的三处，见 [`../docs/protocol.md`](../docs/protocol.md) §3/§7）：

1. **字节序是小端（Intel）**，不是草案写的"大端"；每帧带 **4 bit 滚动计数器 + CRC8（初值=本帧 Data ID）**。
2. **断流动作不对称**：`0x101/0x102` 断流 100 ms → **保持当前关节位置**（不回落、不回零），而不是"自动停止"。能否真做保持取决于 §4 #4。
3. 草案把关节全量放 1 kHz 互发 —— v3 里 `0x031` 只送故障 + J4 + 底座（全量 1 kHz 会把总线负载推到 ≈50 %）。

**不要自己手写编解码**：`python3 ../tools/can_check.py --c-header` 生成 `../docs/gen/can_protocol.h`；提代码前 `can_check.py` 必须退出码 0。

---

## 6. 待办（顺序不能换）

- **Phase A（阻塞项）**：落实 §4 的 7 条（尤其 #1 板子形态、#2 夹爪、#3 零位、#7 载荷），把答案写回本文件。
- **Phase B**：CubeMX 配外设（4 路 25 kHz PWM + 4 路编码器 + CAN1 + IWDG + 可选 OLED/ADC），结构照抄底盘 Phase A。
- **Phase C**：复用 `platform/` 的 L1/L2（若底盘尚未抽出，就先在本工程写、按可抽的形状写），只新写 `app_traj`（位置参考铺开）+ `app_limit`（限位/单向趋近）。
- **Phase D**：单轴点动 → 全轴示教-回放（**空载、架空**），记录每轴角度↔占空比/电流实测曲线、死区与齿隙迟滞带宽。
- **Phase E**：与底盘联调 `0x300`/`0x301`，验证急停广播两板同时锁存、断流后臂保持不动而非回零。

---

## 7. 本工程的硬规矩

1. **上电绝不自动动臂**。零位标定也必须由显式指令触发（`SAFETY_CMD = HOME_REQUEST`）；`homed=0` 时任何 `START` 都应被拒（故障码 `0x0053`）。
2. **降级 = 主动锁位，不是断电**（已确认：臂断电会回落）。急停进 `ESTOP_HELD`：继续闭环抱当前关节角、不断使能，只停止接受新目标；锁位**应限时**（建议 ≤30 s，超时转 FAULT 并提示人工介入）。真正释放只能由 `RELEASE_HOLD` 显式触发。
3. **零位靠低速堵转撞死点建立，且单向趋近**（消除齿隙迟滞）；`homed` **不存 Flash**，复位/看门狗后必须重做。详细流程见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md) §7.5。
4. 动作必须**插值分步**执行，不做阶跃目标；每轴速度/加速度上限写成常量并强制。
5. ISR 内禁止浮点、除法链、`printf`、阻塞调用；位置/速度数据**归 ISR 单一写者**，任务只读。
6. 只往 `USER CODE BEGIN xxx` … `USER CODE END xxx` 标记对之间写；新 `.c/.h` 加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/CMakeLists.txt` 会被 CubeMX 覆盖）。
7. 烧完真断电冷启动 + 确认 `build_tag`；一次只改一个变量；实测数据（占空比、角度、堵转判据、齿隙带宽）记进提交说明。

## 8. 参考与可复用代码（只读）

执行器定为闭环减速电机后，**主参照从参考课设的 `arm/` 层转到 `chassis/` 层**（它控的就是同一类电机）：

- 最大头的复用：**`../platform/` 共用层**（L1 驱动 + L2 25 kHz 闭环），从底盘抽出来直接拿。
- [`../car2/tools/references/smart-transport-robot/01_课程设计-智能搬运机器人/代码/chassis/`](../car2/tools/references/smart-transport-robot/01_课程设计-智能搬运机器人/代码/chassis/)：`motor_ctrl.c`（双输入驱动）、`calib.c`（含“架空时 MG513 需 ~75% 挣脱”的实测注释）、编码器 PID 思路。
- [`代码/arm/`](../car2/tools/references/smart-transport-robot/01_课程设计-智能搬运机器人/代码/arm/)（`servo.c` + `arm_task.c`）：**位姿表 / 动作链 / 夹爪状态独立跟踪 / 分步插值防甩的结构可参考**；但它是**舵机角度↔脉宽（500–2500 µs）**模型，与本工程不同，**驱动层不能直接搬**。
- `代码/tools/arm_teach.py`、`armjog.py`：上位机示教与单轴点动的工具思路（本工程的等价物 = SWD 邮箱或 CAN 下发）。
- ⚠️ 参考代码的逻辑做过 host 测试，但**所有角度/增益是占位值且没上实机**；它是 Keil/自建 bsp 结构，移植要重写为 HAL + CubeMX，不要整段拷贝。

*最后更新：2026-10-02。§4 每落实一条就把答案写回该表，不要只留在聊天记录里。*
