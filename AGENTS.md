# AGENTS.md — 本仓工作约束（机械臂 MCU 固件）

**与底盘完全同一块 C30D 板**（STM32F407VET6 + 板载 4 路 AT8236），只是角色不同：4 个同型号闭环关节电机跑 25 kHz 硬实时，底座 1 个步进电机跑 1 kHz 位置环。姊妹仓 `vehical_rak_chassis`，上位机 `VANexus/rak`（MoveIt 做 IK，**本仓不做 IK**）。

## 先读

- `docs/QUICKSTART.md`：两端装环境→构建→烧录，以及**本仓三条特有安全红线**
- `docs/BOARD-C30D.md`：引脚与硬件真相（步进脉冲合法落点 PC6=TIM8_CH1；无电流采样）
- `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`：CAN 协议 v3

## 安全红线（优先级高于任何功能需求）

1. **上电绝不自动动臂**。零位标定也必须由显式指令触发（`0x010` 的 `HOME_REQUEST`）；`homed=0` 时任何 `START` 都要拒绝并报 `0x0053`。
2. **降级 = 主动锁位，不是断电**——臂断电/停机会回落（无抱闸 + 30:1 齿隙）。急停进 `ESTOP_HELD`：继续闭环抱当前关节角、不断使能；锁位限时（建议 ≤30 s）超时报 FAULT；真正释放只能 `RELEASE_HOLD`。
3. **零位靠低速单向堵转建立**（没有限位开关、没有绝对值编码器、没有电流采样 → 判据只能是"占空比到顶 + 编码器不涨"）。**必须单向趋近**消齿隙；超时即失败不得自动重试；`homed` **不写 Flash**，复位后重做。
4. 动作一律**插值分步**，禁止阶跃目标；每轴速度/加速度上限写成常量并强制。
5. 25 kHz ISR 内禁止浮点/除法/`printf`/阻塞；位置与速度归 ISR 单一写者。
6. **1 Mbps CAN 必须 HSE 8 MHz**；上板必须真断电冷启动并核对 `build_tag`。

## 构建 / 烧录 / 提交

```bash
. ./env/activate.sh            # Linux（Windows: . .\env\activate.ps1；只需 STM32CubeCLT 1.22.0）
cmake --preset Debug && cmake --build --preset Debug      # 还要跑 Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_arm.hex -v -rst
python3 tools/can_check.py                                 # 动了协议就必须通过
```

- 新 `.c/.h` 加进**顶层** `CMakeLists.txt`；只往 `USER CODE BEGIN/END` 之间写；重生成后别让它把 SVC/PendSV/SysTick handler 加回来。
- 与底盘同构的部分（L1 驱动 / L2 闭环 / L4 安全 / L5 通信）将来直接复用 `platform/` 共用层；本仓只新写 `app_traj`（关节轨迹铺开）、`app_limit`（限位与单向趋近）、`app_step`（底座步进）。
- 提交前：Debug + Release 零警告 + `can_check.py` 通过；开工先 pull。
- 改 `docs/PROTOCOL.md` / `protocol-signals.yaml` / `docs/BOARD-C30D.md` 时，**同一批把姊妹仓 `vehical_rak_chassis` 的那份一起改**。
- 待确认硬件项（夹爪接哪一路、步进驱动器型号与有无反馈）见 `README.md` §4，**没确认前不要把这些引脚写进 `.ioc`**。
