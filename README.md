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
- **机械臂板**（本仓）：6 路舵机 PWM（50 Hz）+ 臂遥测。
- 舵机指令的**处理板**从底盘板改为本板；**帧 ID 与字节布局一字未动**（同总线 ID 唯一
  → Jetson 侧零改动）。本板新增自己的两帧上行 `0x031 ARM_NODE_INFO` / `0x221 ARM_DIAG`，
  避免与底盘板的 `0x030` / `0x220` 同 ID 撞车。协议版本号仍是 **4**。

---

## 1. 当前状态：A1（首版机械臂板固件）

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4 V1.28.3，TargetToolchain=CMake（`.ioc` = `vehical_rak_arm.ioc`） |
| Debug / Release 构建 | ✅ 双预设零警告 |
| 应用代码 | ✅ A1：6 路舵机 PWM + CAN v4 臂帧收发 + OLED + IWDG；**手术级新写**（`app_arm` / `app_servo` / `app_build` / `app_can` / `app_oled`） |
| 上板运行 | ⏳ 首版待上板实测（真断电冷启动 + 核对 `build_tag=A1`） |
| CAN 物理联调 | ⏳ 与底盘板 / Jetson 三节点联调待做（2026-10-29 起） |

`build_tag` 单点定义在 [`Core/Src/app_build.c`](Core/Src/app_build.c)，OLED 右上角显示。

---

## 2. 外设与舵机地图

| 资源 | 配置 |
| --- | --- |
| CAN1 | **PD0 = RX / PD1 = TX**，板载收发器，**1 Mbps**（Prescaler=2，BS1=16TQ，BS2=4TQ，SJW=1TQ；采样点 81.0 %）；RX 中断 `CAN1_RX0_IRQn` |
| 舵机 PWM | **TIM8 CH1-4 = PC6/PC7/PC8/PC9**，**TIM12 CH1-2 = PB14/PB15**，**50 Hz**（1 µs tick、ARR=19999、脉宽 500~2500 µs） |
| OLED | 128×64 SSD1306 软 SPI（PD11=DC / PD12=RES / PD13=DAT / PD14=CLK） |
| 看门狗 | IWDG（10 Hz 任务喂狗；喂不上 → 2 s 复位，OLED `P` 字段显示 `I`） |
| RTOS | FreeRTOS（CMSIS-RTOS V2），HAL timebase = TIM7 |
| 时钟 | HSE 8 MHz → PLL(PLLM=4, PLLN=168, PLLP=DIV2) → 168 MHz；APB1 = 42 MHz，APB2 = 84 MHz |
| 调试 | SWD（PA13 / PA14） |

**轴 ↔ 通道映射（装臂实测接线，单点定义在 `app_arm.c`）**：

| 轴 | J1 腰 | J2 大臂 | J3 肘 | J4 小臂 | J5 腕俯仰 | J6 腕旋转 |
| --- | --- | --- | --- | --- | --- | --- |
| 舵机通道 | ch5 | ch4 | ch3 | ch2 | ch1 | ch0 |
| 引脚 | PB15 | PB14 | PC9 | PC8 | PC7 | PC6 |

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
| `Core/Src/app_arm.c` | L3 语义层：角度→脉宽换算、轴↔通道映射、急停锁存、指令断流判定、状态位图/模式机 |
| `Core/Src/app_servo.c` | L1 驱动：6 路舵机 PWM 写 CCR（TIM8/TIM12）+ 读回脉宽；`servo_init` 输出 Home 安全位 |
| `Core/Src/app_can.c` | L5 通信：v4 臂帧编解码（`0x110/0x111` 下行，`0x210/0x211/0x221/0x031` 上行）；`0x010 SAFETY` e2e 校验在 ISR 内 |
| `Core/Src/app_oled.c` | L1 调试显示：复位原因/运行秒数/build_tag、6 轴角、模式机/指令流/异常计数、急停提示 |
| `Core/Src/app_build.c` | 构建标识 `build_tag`（单点定义） |

任务（`Core/Src/freertos.c`）：`canrx`（阻塞队列）、`cantx`（1 kHz 底拍）、`stat`（10 Hz 喂狗 + OLED）、
`defaultTask`（空转）。**本板无 25 kHz 控制回路**（舵机是纯硬件 PWM）。

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

## 6. 待办

- **`.ioc` 裁剪 + 重生成**：把 `vehical_rak_arm.ioc` 收敛到舵机板所需外设集
  （TIM8/TIM12、CAN1、OLED、IWDG、USART1、FreeRTOS），去掉底盘专用的
  TIM1/3/4/5/6/9/10/11、ADC1、USART3 等；重生成后按 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md) 走三步必查。
  （阻塞点：离线重生成需 CubeMX 仓库路径指向 FW_F4 V1.28.3，当前 `~/STM32Cube/Repository` 多了一层。）
- **上板实测**：架空 / 装臂后核对 Home 安全位、6 轴方向与满量程、急停冻结语义。
- **零位标定**：轴零位如何定（当前靠装臂机械零点）。
- **夹爪**：走第几路、驱动器选型、有无位置反馈（挂起项）。
- **双板 1 kHz 互锁**：v3 的 `XFER_*` 仍未启用（当前两板各自与 Jetson 对话，无板间互锁）。

---

## 7. 本工程的硬规矩

1. **上电绝不自动驱动电机/舵机乱甩**——`servo_init` 先输出 Home 安全位；测试时臂下无人。
2. 只往 `USER CODE BEGIN xxx` … `USER CODE END xxx` 标记对之间写；新 `.c/.h` 加进
   **顶层** `CMakeLists.txt`（`cmake/stm32cubemx/CMakeLists.txt` 会被 CubeMX 覆盖）。
3. **外设配置一律由 CubeMX 生成**；应用代码不写寄存器级外设初始化（见 [`docs/CUBEMX-SOP.md`](docs/CUBEMX-SOP.md)）。
4. ISR 内禁止浮点、除法链、`printf`、阻塞调用；共享数据归 ISR 单一写者，任务只读。
5. 烧完**真断电冷启动** + 核对 `build_tag`；一次只改一个变量；实测数据记进提交说明。
6. 注释署名：AI/工具产出注释以 `[AI]` 前缀标记；**用户原有注释永不删除或改写**。

---

## 8. 与姊妹仓的同步约定

`docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、`docs/gen/can_protocol.h`、
`docs/BOARD-C30D.md`、`tools/can_check.py` **在两个 MCU 仓各存一份**，改动时**同一批**
把姊妹仓 `vehical_rak_motion` 的那份一起改（手工 `cp`，内容保持一致），并在提交说明里写明。

**远端**：`https://github.com/VANexus/vehical_rak_edge_ai`（private，默认分支 `main`，
团队 `RAK-嵌入式开发组` 有 maintain）。协作：**开工先 `git pull`** → 改动 →
**收工 `git add -A && git commit && git push`**；提交前必须 `python3 tools/can_check.py`
与 `cmake --build --preset Debug / Release` 双双零问题。

---

*最后更新：2026-10-28（恢复机械臂板角色 A1：6 路舵机 + CAN v4 臂帧）。*
