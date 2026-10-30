# AGENTS.md — 本仓工作约束（机械臂板固件）

> ⚠️ **v4 已于 2026-10-30 整体下线。** 本文中所有对 `docs/PROTOCOL.md`、`docs/protocol-signals.yaml`、
> `tools/can_check.py`、`docs/gen/can_protocol.h`、`APP_USE_CANOPEN` 的引用**全部作废**——
> 这些文件已删除，固件里没有 v4 分支。协议唯一规范是 [`docs/CANOPEN-V5.md`](CANOPEN-V5.md)（README 同目录），
> 门禁是 `python3 tools/canopen_gate.py --repo .`。历史说明保留原文只为追溯，**不要照着执行**。



> **CAN 协议已于 2026-10-29 切到 v5（CANopen）** —— 规范件 `docs/CANOPEN-V5.md`，
> 门禁 `python3 tools/canopen_gate.py`，台架验收 `python3 tools/canopen_bringup.py`。
> 本文里凡提到 `can_check.py` / `protocol-signals.yaml` / `PROTOCOL.md`(v4) / `can_protocol.h`
> 的条目**只适用于 v4 回滚分支**（`cmake -DAPP_USE_CANOPEN=OFF`）。
> 当前在板 build_tag：底盘 **R73** / 机械臂 **A14**；Jetson `rak` 侧仍是 v4 主站，
> 与 v5 板子不兼容，联调暂用上面的 `canopen_bringup.py`（生产主站 = 待办 P3）。


> **仓名历史遗留**：git 仓 / 目录叫 `vehical_rak_edge_ai`，但 2026-10-28 起本仓
> **恢复为机械臂板 `vehical_rak_arm`** 的角色（CMake 工程名、产物、`.ioc` 都叫 `vehical_rak_arm`）。
> 仓名与目录名**不要再改**（会打断团队链接与 CI）。

本仓 = C30D（STM32F407VET6 + 板载 AT8236）上的 **6 路舵机机械臂控制器**。
与姊妹仓 `vehical_rak_motion`（**底盘板**：4 轮 25 kHz 闭环 + IMU）**完全同一块硬件**，
只是角色不同；上位机是 `VANexus/rak`（Jetson，负责 IK 与轨迹规划）。三节点同挂一条 CAN 总线。

## 先读

- `README.md`：本工程现状、外设与舵机地图、双板架构
- `docs/QUICKSTART.md`：两端装环境→构建→烧录，以及本仓硬规矩
- `docs/BOARD-C30D.md`：引脚与硬件真相（CAN1 = PD0/PD1 板载收发器；无电流采样）
- `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`：CAN 协议 v4（共享、已冻结）
- `docs/CUBEMX-SOP.md`：改任何外设/配置前必读

## 安全红线（优先级高于任何功能需求）

1. **上电绝不乱甩舵机**：`servo_init()` 先输出 **Home 安全位**（500/2166/2500/500/500/500 µs），
   之后只有 `ARM_CMD` 或显式指令才改变角度。测试时机械臂行程内不得有人/物。
2. **急停语义 = 冻结**：收到 `0x010 SAFETY_CMD` 的 `EMERGENCY_STOP`（cmd=4）后，
   臂板**锁存急停、拒绝落地后续 ARM 指令**（舵机保持当前位置，不归零）；仅 `CLEAR_ESTOP`（cmd=5）可解。
3. **1 Mbps CAN 必须 HSE 8 MHz**；上板必须真断电冷启动并核对 `build_tag`（当前 `A1`）。
4. ISR 内禁止浮点/除法/`printf`/阻塞；共享数据归 ISR 单一写者，任务只读。
5. 本板**不做 IK / 轨迹规划**（在 Jetson 的 MoveIt）；只做「指令角 → 脉宽」的执行与本地兜底。

## 代码分层与署名

- 应用层：`app_arm`（L3 语义）、`app_servo`（L1 驱动）、`app_can`（L5 通信）、
  `app_oled`（L1 调试显示）、`app_build`（构建标识）。**外设配置一律由 CubeMX 生成**，
  应用代码不写寄存器级外设初始化。
- 只往 `USER CODE BEGIN/END` 之间写；新 `.c/.h` 加进**顶层** `CMakeLists.txt`。
- AI/工具产出的注释一律以 `[AI]` 前缀标记；**用户原有注释永不删除或改写**。
- 生成后必查（`docs/CUBEMX-SOP.md` §三第 3 步）：`stm32f4xx_it.c` 不得恢复
  SVC/PendSV/SysTick 三个 handler；`SystemClock_Config` 的 HSE 不得被抹回 HSI；
  `main.c` 的 `USER CODE BEGIN 2` 必须在所有 `MX_*_Init()` 之后。

## 构建 / 烧录 / 提交

```bash
. ./env/activate.sh            # Linux（Windows: . .\env\activate.ps1；只需 STM32CubeCLT 1.22.0）
cmake --preset Debug   && cmake --build --preset Debug      # 还要跑 Release
cmake --preset Release && cmake --build --preset Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
python3 tools/can_check.py                                 # 动了协议就必须通过（退出码 0；需 PyYAML）
python3 tools/can_check.py --c-header                      # 改协议后重生成 docs/gen/can_protocol.h（勿手改）
```

- **每条命令都要在同一个 shell 里先 source `env/activate.sh`**（只改当前 shell 的 PATH）。
- 提交前：Debug + Release **零警告** + `can_check.py` 通过；开工先 `git pull`，收工就 push。
- `build_tag` 单点定义在 `Core/Src/app_build.c`；每次有效改动升一档（`A1` → `A2` …）。
- 改 `docs/PROTOCOL.md` / `protocol-signals.yaml` / `docs/gen/can_protocol.h` /
  `docs/BOARD-C30D.md` / `tools/can_check.py` / `env/activate.*` 时，**同一批把姊妹仓
  `vehical_rak_motion` 的那份一起改**（两仓各存一份，手工 `cp`，内容一致），提交说明写明。
