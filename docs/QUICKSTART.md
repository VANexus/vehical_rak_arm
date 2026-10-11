# 快速上手（边缘 AI 推理节点固件仓）

面向新成员（Windows / Linux）。**先读第 0 节再动手**。

---

## 0. 这个仓库是什么

`vehical_rak_edge_ai` = **边缘 AI（BNN/SNN）推理节点**的 STM32F407VET6 固件（与运动仓 `vehical_rak_motion` **完全同一块**轮趣 C30D 板，只是角色不同）：在板上跑推理与本地执行。
当前 `.ioc` 已按边缘 AI 节点所需**裁剪**，只保留：CAN1（PD0/PD1，板载收发器，1 Mbps）、OLED 软 SPI（PD11/PD12/PD13/PD14）、motorA 底部云台电机（PWM = TIM10_CH1/PB8 + TIM11_CH1/PB9）、编码器 ×1（TIM2，PA15/PB3）、FreeRTOS（CMSIS-RTOS V2）、HSE 8 MHz → 168 MHz。其余引脚保持 CubeMX 默认。

**当前进度**：CubeMX 骨架能编译，**应用代码 0 行**。

## 1. 本仓硬规矩

| 规则 | 说明 |
| --- | --- |
| **上电绝不自动驱动电机** | 必须按键或收到显式指令才动；测试时架空 |
| **1 Mbps CAN 必须 HSE 8 MHz** | HSI 精度不够，1 Mbps 会偶发错误帧，极难定位 |
| **ISR 内禁止浮点/除法/`printf`/阻塞** | 共享数据归 ISR 单一写者，任务只读 |

## 2. 工具链与环境（与运动仓逐字相同）

```bash
. ./env/activate.sh          # Linux/macOS；Windows: . .\env\activate.ps1
cmake --preset Debug && cmake --build --preset Debug
# → build/Debug/vehical_rak_edge_ai.{elf,hex,bin,map}
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_edge_ai.hex -v -rst
```

只需 **STM32CubeCLT 1.22.0**（Linux 已在 `/opt/st/stm32cubeclt_1.22.0`）。两端版本必须一致，比对不要比字节，比 `.map` 的 section 与上板行为。

## 3. 代码结构约定

- 业务代码放 `Core/Src/app_*.c`；与运动仓同构的部分（L1 驱动 / L5 通信等）来自 `platform/`（两仓共用，尚未抽出）。
- 新文件加进**顶层** `CMakeLists.txt`（`cmake/stm32cubemx/` 那份会被 CubeMX 覆盖）。
- 只在 `USER CODE BEGIN/END` 标记对之间写。
- 帧宏不要手写：`python3 tools/can_check.py --c-header` 生成 `docs/gen/can_protocol.h`；提交前 `python3 tools/can_check.py` 必须退出码 0。

## 4. 上板检查清单

1. **先架空**，第一次跑不要带负载；
2. 烧完**真断电冷启动**（电池关 → 拔 ST-LINK → 等 OLED 全黑 → 上电），屏上确认 `build_tag`；
3. 量动力端子电压再上电（曾被 2.89 V 过放坑成"假软件故障"）；
4. 每次实测记录：`.bin` SHA-256、供电电压、终态、目视现象。

## 5. 文档索引

| 文件 | 作用 |
| --- | --- |
| [`README.md`](../README.md) | 本仓现状、外设集合、通信协议角色、待办 |
| [`docs/PROTOCOL.md`](PROTOCOL.md) | CAN 协议 v4 规范（帧表、超时矩阵、故障码） |
| [`docs/protocol-signals.yaml`](protocol-signals.yaml) | 协议机器可读单一来源 |
| [`docs/BOARD-C30D.md`](BOARD-C30D.md) | 板级资源表（CAN1 = PD0/PD1 板载收发器；无电流采样） |
| [`docs/gen/can_protocol.h`](gen/can_protocol.h) | 生成的帧宏 |
| 工作区根 [`AGENTS.md`](../AGENTS.md) | 跨端工具链与工程纪律 |

约定：`docs/PROTOCOL.md`、`docs/BOARD-C30D.md`、`docs/protocol-signals.yaml`、`tools/can_check.py`、`env/activate.*` 在本仓各存一份（与姊妹仓内容保持一致，改动时同一批 `cp` 过去）；`docs/gen/can_protocol.h` 由 `can_check.py --c-header` 生成，不手改。