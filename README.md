# vehical_rak_arm — 机械臂板固件（STM32F407VET6 / C30D）

> **本仓的 git 仓名是历史遗留的 `vehical_rak_edge_ai`**：2026-10-14 曾把「机械臂板」
> 重命名成「边缘 AI 推理节点」；**2026-10-28 恢复双板架构**，本仓重新担任
> **机械臂板 `vehical_rak_arm`** 的角色（CMake 工程名、目标产物、`.ioc` 都叫
> `vehical_rak_arm`）。GitHub 仓名与目录名沿用旧拼写，**不要再改名**（会打断团队链接）。

本板 = C30D 上的 **6 路舵机机械臂控制器**：接收 Jetson 下发的关节角（CAN v4 `ARM_CMD`），
换算成舵机脉宽驱动机械臂，并把实测关节角、诊断量回报给 Jetson 与上位机 UI。
**不做 IK、不做轨迹规划**（那在 Jetson 的 MoveIt）；本板只做「指令角 → 脉宽」的执行 +
本地安全兜底（急停锁存、指令断流）。

行为约束看本仓 [`AGENTS.md`](AGENTS.md)；协议看 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)
（规范件）+ [`docs/protocol-signals.yaml`](docs/protocol-signals.yaml)（机器可读单一来源）；
板级真相看 [`docs/BOARD-C30D.md`](docs/BOARD-C30D.md)。**新成员先读 [`docs/QUICKSTART.md`](docs/QUICKSTART.md)**。

---

## 0. 双板架构（2026-10-28 恢复）

一条 CAN 总线（CAN 2.0A，1 Mbps）挂三个节点，帧 ID 全局唯一：

```
              CAN 总线 1 Mbps
   Jetson（rak）┬── 0x100 DRIVE_CMD / 0x010 SAFETY_CMD ── 底盘板 vehical_rak_motion
                ├── 0x110/0x111 ARM_CMD_A/B / 0x010 ──── 机械臂板 vehical_rak_arm（本仓）
                ├── 0x200/0x201/0x202 轮遥测 ←────────── 底盘板
                └── 0x210/0x211/0x221/0x031 臂遥测 ←──── 机械臂板
```

- **底盘板** `vehical_rak_motion`：4 轮 25 kHz 闭环 + IMU。
- **机械臂板**（本仓）：J1/J2 = **增量编码器电机位置闭环（25 kHz 采样 / 1 kHz 位置环）**，
  J3..J6 = 6 路舵机里的 4 路 PWM（50 Hz）+ 臂遥测。
- 舵机指令的**处理板**从底盘板改为本板；**帧 ID 与字节布局一字未动**（同总线 ID 唯一
  → Jetson 侧零改动）。本板新增自己的两帧上行 `0x031 ARM_NODE_INFO` / `0x221 ARM_DIAG`，
  避免与底盘板的 `0x030` / `0x220` 同 ID 撞车。协议版本号仍是 **4**。

---

## 1. 当前状态：A2（J1/J2 改造为编码器电机闭环）

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，TargetToolchain=CMake（`.ioc` = `vehical_rak_arm.ioc`） |
| Debug / Release 构建 | ✅ 双预设零警告 |
| 应用代码 | ✅ A2：**J1/J2 增量编码器电机位置闭环**（`app_joint`，25 kHz 采样 + 1 kHz 定点位置 PI + 软件置零 + 急停刹车）+ J3..J6 舵机 PWM + CAN v4 臂帧收发 + OLED + IWDG |
| 上板运行 | ⏳ 待上板实测（真断电冷启动 + 核对 `build_tag=A2`）。**J1/J2 电机轴必须先走 §6 流程** |
| CAN 物理联调 | ⏳ 与底盘板 / Jetson 三节点联调待做（2026-10-29 起） |
| 协议 | ✅ **零改动**：`0x110/0x111` 与 `0x210/0x211` 的 ID / 字节布局一字未动（Jetson 侧零改动） |

`build_tag` 单点定义在 [`Core/Src/app_build.c`](Core/Src/app_build.c)，OLED 右上角显示。

---

## 2. 外设与执行机构地图

| 资源 | 配置 |
| --- | --- |
| CAN1 | **PD0 = RX / PD1 = TX**，板载收发器，**1 Mbps**（Prescaler=2，BS1=16TQ，BS2=4TQ，SJW=1TQ；采样点 81.0 %）；RX 中断 `CAN1_RX0_IRQn` |
| 舵机 PWM（J3..J6） | **TIM8 CH4..CH1 = PC9/PC8/PC7/PC6**，**50 Hz**（1 µs tick、ARR=19999、脉宽 500~2500 µs）。**TIM12（PB14/PB15）已停用**（原 J1/J2，现改电机） |
| 关节电机 PWM（J1/J2） | **TIM10_CH1/TIM11_CH1 = PB8/PB9**（电机口 A）+ **TIM9_CH1/CH2 = PE5/PE6**（电机口 B），**25 kHz**（ARR=6719）← `[AI] 待实测`：默认占位，按实物插头改 `app_joint.c` 的 `kJointMotorCh` |
| 关节编码器（J1/J2） | **TIM2 = PA15/PB3**（编码器口 A，32 位）+ **TIM3 = PB4/PB5**（编码器口 B，16 位），TI12 四倍频 ← 同上，改 `kJointEncIdx` |
| 关节闭环 ISR | **TIM6 25 kHz**（ARR=3359 @84 MHz，`joint_isr`）；DWT 周期计数埋点 `dbg_joint_isr_last/max_cycles` |
| OLED | 128×64 SSD1306 软 SPI（PD11=DC / PD12=RES / PD13=DAT / PD14=CLK） |
| 看门狗 | IWDG（10 Hz 任务喂狗；喂不上 → 2 s 复位，OLED `P` 字段显示 `I`） |
| RTOS | FreeRTOS（CMSIS-RTOS V2），HAL timebase = TIM7 |
| 时钟 | HSE 8 MHz → PLL(PLLM=4, PLLN=168, PLLP=DIV2) → 168 MHz；APB1 = 42 MHz，APB2 = 84 MHz |
| 调试 | SWD（PA13 / PA14） |

**轴 ↔ 通道映射**（单点定义：J1/J2 在 `app_joint.c`，J3..J6 在 `app_arm.c`）：

| 轴 | J1 腰 | J2 大臂 | J3 肘 | J4 小臂 | J5 腕俯仰 | J6 腕旋转 |
| --- | --- | --- | --- | --- | --- | --- |
| 执行机构（A2） | **编码器电机** | **编码器电机** | 舵机 | 舵机 | 舵机 | 舵机 |
| 通道 | 电机口 A / 编码器 A `[AI] 待实测` | 电机口 B / 编码器 B `[AI] 待实测` | ch3 | ch2 | ch1 | ch0 |
| 引脚 | PB8/PB9 + PA15/PB3 | PE5/PE6 + PB4/PB5 | PC9 | PC8 | PC7 | PC6 |

- **归零（J1/J2）**：人工推到机械基准位 → **长按用户键 PE0 ≥1.5 s** → 当前位置定义为 0.0°
  （同时目标置 0、清故障）。未归零时两轴恒刹车、不接受指令。增量编码器**掉电丢零点**，
  每次冷启动都要重做。OLED 第 6 行 `Z1` = 已归零。
- **J1/J2 位置环**：状态显示 `B`=刹车 `H`=在位 `M`=跟踪 `F`=故障（OLED 第 6 行）。

- 位置语义：**0~270° 满量程 ↔ 500~2500 µs**；无效轴回 1500 µs 中位。
- **上电默认位 = Home 安全位**（J1=0°/J2=225°/J3=270°/J4=0°/J5=0°/J6=0° →
  500/2166/2500/500/500/500 µs），与 `rak` 仓 SRDF home 一致。
- 协议第 7 轴（夹爪）**本板无硬件通道**，只回显不落地。
- ⚠️ J1/J6 实测为**连续旋转舵机**，其角度值按「速度偏置」解释（中位=停转）——上位机标定层约定。

> **`.ioc` 现状说明**：`vehical_rak_arm.ioc` 目前与底盘板同构（同一块 C30D 硬件，
> 外设全集；见下节），因为离线重生成需要 CubeMX 指向 FW_F4 V1.28.3 仓库路径而暂缓。
> 本板**只启动舵机所需的 TIM8/TIM12 + CAN1**，其余外设在 `main.c` 里 init 但不启动
> （无输出）。**裁剪 `.ioc` 并重生成 = 待办**（见 §6），属整洁性问题、不影响功能。

## 3. 应用层模块

| 文件 | 职责 |
| --- | --- |
| `Core/Src/app_joint.c` | **L1/L2 实时层（A2 新增）**：J1/J2 AT8236 双输入 + 正交编码器 + 25 kHz 采样 + 1 kHz 定点位置 PI + 软件置零 + 防飞车闸门 + PE0 长按；配置（接线/换算/增益/限位/极性）**单点定义在文件顶部** |
| `Core/Src/app_arm.c` | L3 语义层：角度→执行机构换算（J1/J2→counts，J3..J6→脉宽）、轴↔通道映射、急停锁存、指令断流判定、状态位图/模式机 |
| `Core/Src/app_servo.c` | L1 驱动：6 路舵机 PWM 写 CCR（TIM8/TIM12）+ 读回脉宽；`servo_init` 输出 Home 安全位 |
| `Core/Src/app_can.c` | L5 通信：v4 臂帧编解码（`0x110/0x111` 下行，`0x210/0x211/0x221/0x031` 上行）；`0x010 SAFETY` e2e 校验在 ISR 内；`cmd=6 CLEAR_FAULT` 复位 J1/J2 位置环故障 |
| `Core/Src/app_oled.c` | L1 调试显示：复位原因/运行秒数/build_tag、6 轴角（J1/J2 = 编码器回读）、模式机/指令流/异常计数、急停提示、J1/J2 电机轴状态 |
| `Core/Src/app_build.c` | 构建标识 `build_tag`（单点定义） |

任务（`Core/Src/freertos.c`）：`canrx`（阻塞队列）、`cantx`（1 kHz 底拍）、`stat`（10 Hz 喂狗 + OLED）、
`defaultTask`（启动 25 kHz 关节环后空转）。**舵机没有控制回路**（纯硬件 PWM）；
**J1/J2 的 25 kHz 关节环跑在 TIM6 中断里**（`stm32f4xx_it.c` → `joint_isr`），
由 `defaultTask` 在调度器起来后 `joint_start_isr()` 启动（与底盘板同思路）。

---

## 4. 构建与烧录（与底盘固件逐字相同）

```bash
. ./env/activate.sh                 # Windows: . .\env\activate.ps1
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
# → build/<预设>/vehical_rak_arm.{elf,hex,bin} + .map（hex/bin 由顶层 CMakeLists.txt 的 POST_BUILD 生成）

STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
```

烧录后必须**真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 再上电），
并以屏上 `build_tag` 判别实际运行版本。**每次有效改动升一档标签**（`A1` → `A2` …）。

---

## 5. 通信协议

本仓是 C30D CAN 总线上的一个节点，遵循**共享且已冻结**的协议：规范文字见
[`docs/PROTOCOL.md`](docs/PROTOCOL.md)，机器可读单一来源见
[`docs/protocol-signals.yaml`](docs/protocol-signals.yaml)。`0x010 SAFETY_CMD` 是**广播安全帧**，
本板必须响应：`EMERGENCY_STOP`（cmd=4）→ 立即冻结舵机并锁存；`CLEAR_ESTOP`（cmd=5）→ 解锁存。

**不要自己手写帧表**：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`
（WIP 已接入 `app_can.c`）；提代码前 `can_check.py` 必须退出码 0。

---

## 6. J1/J2 电机轴首次上板流程（**必须照做**）

> 为什么单独一节：J1/J2 是**新增的电机闭环**，接线映射、出力极性、环路增益、软限位
> **都还没实测**（代码里标 `[AI] 待实测`）。极性反了 = 正反馈飞车。所以第一次上电**绝不能**
> 直接发 ARM_CMD。按下面的顺序，一步一个变量。

**准备**：臂固定/悬空支撑（J2 会因自重下落，掉电无抱闸）；电池电压正常；
手放在总电源开关上；先 `candump can0` 只看不发。

| 步 | 做什么 | 判据 / 记录 |
| --- | --- | --- |
| 1 | 真断电冷启动，看 OLED | `build_tag=A2`；第 6 行 `J1B J2B Z0 F0`（B=刹车，Z0=未归零） |
| 2 | 不发任何指令，`candump can0,031:7FF` + `can0,221:7FF` | 1 Hz `0x031`（PROTO=4）+ 10 Hz `0x221` 正常；`0x210/0x211` 50 Hz 自发 |
| 3 | **线序核对**：转动 J1 输出轴（手转即可），看 OLED 的 J1 角度是否变化 | 变了 = 该轴编码器口选对了；两轴都要变。不动 = `kJointEncIdx` 选错，改代码重烧 |
| 4 | **极性确认**：长按 PE0 ≥1.5 s 置零（`Z1`），然后 `candump` 观察或用最小目标 | ⚠️ 见下 |
| 5 | 归零后给 J1 一个**很小的**目标（如 1.0°） | 关节应**朝角度增大方向**微动且误差收敛到 0（OLED `M`→`H`）。若**反向跑** → 翻 `kJointSign` 对应项，重烧重来 |
| 6 | J2 同法 | 同上（J2 注意自重：目标向上/向下各试一次） |
| 7 | 逐步放大目标（5°→20°→90°），记录跟随误差、是否有 FAULT | 反复 FAULT 且误差大 = 增益/目标斜坡要调（`JOINT_KP/KI`、`JOINT_TGT_SLEW`） |
| 8 | 满量程与软限位标定 | 记录实际可达角度 → 填 `JOINT_ANGLE_MAX_DEG10` |
| 9 | **ISR 占用率实测**（AGENTS 要求，仓库无数据） | SWD 读 `dbg_joint_isr_max_cycles`；预算 40 µs = 6720 周期，**≤35% = 2352 周期** |
| 10 | 记录 `.bin` 的 SHA-256、供电电压、结论 | 写进提交说明（AGENTS §5 规则 5） |

**第 4 步的两个注意**：

- 极性确认**不需要**靠"发目标看方向"——更安全的做法是在**臂被手托住/限位挡住**的状态下
  发一个 1° 目标：读数正确则电机只会轻微吃力；若极性反了，2 s 内跟随误差就会涨到 30° 触发
  `FAULT`（另一轴刹车）——**这是设计内的闸门，不是故障**。
- 复位 `FAULT`：长按 PE0 重新置零，或用 CAN `0x010 cmd=6 CLEAR_FAULT`（`joint_fault_clear()`
  只在越界条件已消失时才真正清位）。

**调试便捷量**（SWD 直读，无需加打印）：`dbg_joint_isr_last_cycles` / `dbg_joint_isr_max_cycles`。

---

## 7. 待办

- **`.ioc` 裁剪 + 重生成**：原计划收敛到"舵机板所需外设集"；**A2 起 J1/J2 用上了
  TIM1/9/10/11（电机 PWM）+ TIM2/3/4/5（编码器）+ TIM6（25 kHz 闭环）+ EXT I0（PE0 置零）**
  → 该外设集**正好就是** `vehical_rak_arm.ioc` 现在继承的全集，裁剪时**必须保留**它们，
  只可去 ADC1 / USART3 等真正不用的。重生成后按 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md) 走三步必查。
  （阻塞点：离线重生成需 CubeMX 仓库路径指向 FW_F4 V1.28.3，当前 `~/STM32Cube/Repository` 多了一层。）
- **上板实测**：按 §6 做 J1/J2 电机轴首版；J3..J6 核对 Home 安全位、方向与满量程、急停冻结语义。
- **J1/J2 待实测参数**（全在 `app_joint.c` 顶部，见 `docs/ARM-ENCODER-JOINTS-DESIGN.md` §9）：
  电机口/编码器口映射、出力极性 `kJointSign`、减速比（若有）、角度范围、位置环增益。
- **ISR 占用率实测**：`dbg_joint_isr_max_cycles`（40 µs = 6720 周期，预算 ≤2352 周期）。
- **夹爪**：走第几路、驱动器选型、有无位置反馈（挂起项）。
- **双板 1 kHz 互锁**：v3 的 `XFER_*` 仍未启用（当前两板各自与 Jetson 对话，无板间互锁）。

---

## 8. 本工程的硬规矩

1. **上电绝不自动驱动电机/舵机乱甩**——`servo_init` 先输出 Home 安全位；
   `joint_init` 只刹车不驱动，且 **J1/J2 未归零前不接受任何指令**；测试时臂下无人。
2. 只往 `USER CODE BEGIN xxx` … `USER CODE END xxx` 标记对之间写；新 `.c/.h` 加进
   **顶层** `CMakeLists.txt`（`cmake/stm32cubemx/CMakeLists.txt` 会被 CubeMX 覆盖）。
3. **外设配置一律由 CubeMX 生成**；应用代码不写寄存器级外设初始化（见 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md)）。
4. ISR 内禁止浮点、除法链、`printf`、阻塞调用；共享数据归 ISR 单一写者，任务只读。
5. 烧完**真断电冷启动** + 核对 `build_tag`；一次只改一个变量；实测数据记进提交说明。
6. 注释署名：AI/工具产出注释以 `[AI]` 前缀标记；**用户原有注释永不删除或改写**。

---

## 9. 与姊妹仓的同步约定

`docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`、
`docs/BOARD-C30D.md`、`tools/can_check.py` **在两个 MCU 仓各存一份**，改动时**同一批**
把姊妹仓 `vehical_rak_motion` 的那份一起改（手工 `cp`，内容保持一致），并在提交说明里写明。

**远端**：`https://github.com/VANexus/vehical_rak_edge_ai`（private，默认分支 `main`，
团队 `RAK-嵌入式开发组` 有 maintain）。协作：**开工先 `git pull`** → 改动 →
**收工 `git add -A && git commit && git push`**；提交前必须 `python3 tools/can_check.py`
与 `cmake --build --preset Debug / Release` 双双零问题。

---

*最后更新：2026-10-28（A2：J1/J2 改增量编码器电机闭环 + 软件归零；J3..J6 仍为舵机；协议零改动）。*
