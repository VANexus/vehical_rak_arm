# AGENTS.md — 本仓工作约束（边缘 AI 推理节点固件）

**与运动板完全同一块 C30D 板**（STM32F407VET6 + 板载 AT8236），只是角色不同：本仓 = **边缘 AI（BNN/SNN）推理节点**，跑板上推理与本地执行。姊妹仓 `vehical_rak_motion`（运动固件），上位机 `VANexus/rak`。

## 先读

- `docs/QUICKSTART.md`：两端装环境→构建→烧录，以及本仓硬规矩
- `docs/BOARD-C30D.md`：引脚与硬件真相（CAN1 = PD0/PD1 板载收发器；无电流采样）
- `docs/PROTOCOL.md` + `docs/protocol-signals.yaml`：CAN 协议 v4（共享、已冻结）

## 安全红线（优先级高于任何功能需求）

1. **上电绝不自动驱动电机**；必须按键或收到显式指令才动，测试时架空。
2. **1 Mbps CAN 必须 HSE 8 MHz**；上板必须真断电冷启动并核对 `build_tag`。
3. ISR 内禁止浮点/除法/`printf`/阻塞；共享数据归 ISR 单一写者，任务只读。

## 构建 / 烧录 / 提交

```bash
. ./env/activate.sh            # Linux（Windows: . .\env\activate.ps1；只需 STM32CubeCLT 1.22.0）
cmake --preset Debug && cmake --build --preset Debug      # 还要跑 Release
STM32_Programmer_CLI -c port=SWD mode=UR freq=100 -w build/Debug/vehical_rak_edge_ai.hex -v -rst
python3 tools/can_check.py                                 # 动了协议就必须通过
```

- 新 `.c/.h` 加进**顶层** `CMakeLists.txt`；只往 `USER CODE BEGIN/END` 之间写；重生成后别让它把 SVC/PendSV/SysTick handler 加回来。
- 与运动仓同构的部分（L1 驱动 / L5 通信等）将来直接复用 `platform/` 共用层。
- 提交前：Debug + Release 零警告 + `can_check.py` 通过；开工先 pull。
- 改 `docs/PROTOCOL.md` / `protocol-signals.yaml` / `docs/BOARD-C30D.md` 时，**同一批把姊妹仓 `vehical_rak_motion` 的那份一起改**。