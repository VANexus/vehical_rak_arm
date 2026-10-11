# vehical_rak_edge_ai — 边缘 AI 推理节点固件（STM32F407VET6）

**边缘 AI（BNN/SNN）推理节点**：在轮趣 C30D（STM32F407VET6 + 板载 AT8236）上跑板上推理与本地执行。与运动固件仓 `vehical_rak_motion` **完全同一块板**，只是角色不同（本工程的 `.ioc` 已按边缘 AI 节点所需裁剪，不再是底盘工程的复制品）。

行为约束看本仓 [`AGENTS.md`](AGENTS.md)，**协议看 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)**，板级真相看 [`docs/BOARD-C30D.md`](docs/BOARD-C30D.md)。本文件只讲**这个工程**。

**新成员先读 [`docs/QUICKSTART.md`](docs/QUICKSTART.md)**。

### 仓库结构

本目录是**独立 git 仓**（`Drivers/`、`Middlewares/` 已入库，可单独 clone 构建）。文档：`docs/QUICKSTART.md` 上手、`docs/PROTOCOL.md` 协议 v4、`docs/BOARD-C30D.md` 板级真相、`AGENTS.md` 本仓约束。协议与板级文档在两个 MCU 仓各存一份，**改动时同一批同步另一仓**（手工 cp，无工具）。

---

**远端**：`https://github.com/VANexus/vehical_rak_edge_ai`（private，默认分支 `main`，团队 `RAK-嵌入式开发组` 有 maintain 权限）。
协作三步：**开工先 `git pull`** → 改动 → **收工 `git add -A && git commit && git push`**；提交前本仓必须 `python3 tools/can_check.py` 与 `cmake --build --preset Debug` 双双零问题。

---

## 1. 当前状态：CubeMX 骨架，应用代码 0 行

| 项 | 状态 |
| --- | --- |
| CubeMX 生成 | ✅ FW_F4，目标 `STM32F407VETx` |
| 外设 | ✅ 已按边缘 AI 节点所需**裁剪**（集合见 §2），不再与底盘工程逐字节同构 |
| Debug / Release 构建 | ✅ 骨架可编译（CubeCLT 14.3.rel1）；裁剪后**本轮未重测** FLASH/RAM 基线 |
| 应用代码 | ❌ 0 行，尚未开始 |

## 2. 外设集合（`.ioc` 现状）

| 外设 | 配置 |
| --- | --- |
| CAN1 | PD0 = RX / PD1 = TX，**板载收发器**，**1 Mbps**（Prescaler=2，BS1=CAN_BS1_16TQ，BS2=CAN_BS2_4TQ，SJW=CAN_SJW_1TQ） |
| OLED 128×64 | 软 SPI（纯 GPIO 位操作），PD11 = DC / PD12 = RES / PD13 = DATA / PD14 = CLK（与运动板同款） |
| motorA | 底部云台电机，同款 **AT8236 闭环减速电机**；PWM = TIM10_CH1(PB8) + TIM11_CH1(PB9) |
| 编码器 ×1 | TIM2（PA15 / PB3），Encoder Mode TI12 |
| RTOS | FreeRTOS（CMSIS-RTOS V2），HAL timebase = TIM7 |
| 时钟 | HSE 8 MHz → PLL(PLLM=4, PLLN=168, PLLQ=7) → 168 MHz；APB1 = 42 MHz，APB2 = 84 MHz |
| 调试 | SWD（PA13 / PA14） |

其余引脚保持 CubeMX 默认，未启用。

## 3. 架构定位（职责边界）

本板是 C30D 总线上的**边缘 AI 推理节点**：上位机（Jetson `rak`）负责上层规划与调度，本板承担**板上推理 + 本地执行/安全兜底**。两边的具体分工**随应用定，尚未落定**——落定前不要凭印象把职责写进代码注释。

---

## 4. 构建与烧录（与运动固件逐字相同）

```bash
. ./env/activate.sh                 # Windows: . .\env\activate.ps1
cmake --preset Debug && cmake --build --preset Debug
# → build/Debug/vehical_rak_edge_ai.{elf,hex,bin,map}（hex/bin 由顶层 CMakeLists.txt 的 POST_BUILD 生成）

STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_edge_ai.hex -v -rst
```

烧录后必须**真断电冷启动**并确认 `build_tag`（[`AGENTS.md`](AGENTS.md) §构建/烧录/提交）。

---

## 5. 通信协议

本仓是 C30D CAN 总线上的一个节点，遵循**共享且已冻结**的协议规范：规范性文字见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)，机器可读单一来源见 [`docs/protocol-signals.yaml`](docs/protocol-signals.yaml)。其中 `0x010 SAFETY_CMD` 是**广播安全帧**，节点必须响应（急停立即停止并锁存，仅 `CLEAR_ESTOP` 可解）。**不要自己手写编解码**：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`；提代码前 `can_check.py` 必须退出码 0。

---

## 6. 待办

- **Phase A**：确认边缘 AI 节点的应用形态与推理运行时（模型格式、推理库），把结论写回本文件。
- **Phase B**：在现有 CubeMX 骨架上补应用代码；`platform/` 共用层若已从运动仓抽出则直接复用（L1 驱动 / L5 通信等），否则先按可抽的形状写。
- **Phase C**：上板实测（架空、真断电冷启动、核对 `build_tag`），把实测数据记进提交说明。

---

## 7. 本工程的硬规矩

1. **上电绝不自动驱动电机**，必须按键或收到显式指令才动；测试时车轮/负载架空。
2. 只往 `USER CODE BEGIN xxx` … `USER CODE END xxx` 标记对之间写；新 `.c/.h` 加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/CMakeLists.txt` 会被 CubeMX 覆盖）。
3. ISR 内禁止浮点、除法链、`printf`、阻塞调用；共享数据归 ISR 单一写者，任务只读。
4. 烧完**真断电冷启动** + 确认 `build_tag`；一次只改一个变量；实测数据记进提交说明。

---

*最后更新：2026-10-11。§1/§2 随 `.ioc` 裁剪同步更新，应用形态落定后写回 §3/§6。*