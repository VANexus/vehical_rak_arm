# 设计方案：`platform/` 共用层 + OLED 驱动抽取（子项目 A1）

- **日期**：2026-10-14
- **仓库**：`vehical_rak_edge_ai`（首个消费者）· 新建 `vehical_rak_platform`
- **状态**：待评审
- **范围**：只做 **A1**（结构层）。edge_ai 的高级 UI（页面/按键/图形/中文字库）是 **A2**，另立方案。

---

## 1. 为什么现在做

`docs/architecture.md` §3 把 `app_oled` 定为 **L1 驱动**，§4 规定 L1/L2/L4/L5 由两个 MCU 工程**共用同一份源码**，放在工作区根的 `platform/`。
但 `platform/` 至今**未创建**，而 `app_oled` 的驱动部分**已经存在于 `vehical_rak_motion/Core/Src/app_oled.c`（247 行）**。

本方案的目标不是"写一个 OLED 驱动"，而是**把已有的那份抽出来，让两个仓能共用**。

### 事实依据（均实测）

| 事实 | 出处 |
| --- | --- |
| 两块板 OLED 均为 **SSD1306 128×64 软 SPI**，引脚 **PD11=DC / PD12=RES / PD13=DATA / PD14=CLK** | WHEELTEC 资源分配 PDF；两仓 `.ioc` |
| `PD11-14` **不对应任何 SPI 外设** → 软 SPI 是硬件约束，不是选择 | STM32F407 LQFP100 AF 表 |
| 现有驱动的**驱动部分零业务耦合**：只 include `main.h` + `stm32f4xx.h`，HAL 只用了一处 `HAL_GetTick()` | 读 `app_oled.c` 行 1-151 |
| 现有驱动的**内容部分 100% 运动仓专属**：调 `car_battery_mv`/`imu_dbg_*`/`car_wheel_speeds`/… | 读 `app_oled.c` 行 153-247 |
| 字库只覆盖 `' '`～`'Z'`（59 字形），`glyph()` 把 `> 'Z'` 静默吞成空格 | `app_oled.c:129` |

---

## 2. 已定决策（本方案的前提）

| # | 决策 | 定于 |
| --- | --- | --- |
| D1 | **分层 LL**：驱动层走 LL/裸寄存器，中间件与时钟树保留 HAL | 用户 2026-10-14 |
| D2 | **`platform/` = 独立 git 仓 + submodule**（P2） | 用户 2026-10-14 |
| D3 | **`motion` 仓代码不动** —— 本次不改 `vehical_rak_motion` 的任何 `.c/.h` | 用户 2026-10-14（"motion 就不变了"） |
| D4 | 板级事实源改为 **WHEELTEC 官方资料包**（car2 已弃用移除） | 用户 2026-10-14 |

> ⚠️ **D3 的直接后果**：`platform/Src/app_oled.c` 与 `vehical_rak_motion/Core/Src/app_oled.c` 的驱动部分会**并存**，构成架构文档 §4 明确警告过的"两边各一份 → 会立刻漂移"。
> 本方案的处置：**接受这个临时状态**，并在 `vehical_rak_motion/README.md` 记一条 TODO —— 等 motion 侧同意改代码时，再把它的驱动部分删掉、改为引用 `platform/`。**不擅自替 motion 做这个决定。**

---

## 3. `platform/` 仓形态

```
VANexus/vehical_rak_platform        (private, rak-2 maintain)
├── Inc/
│   └── app_oled.h                  公开 API
├── Src/
│   └── app_oled.c                  驱动实现
└── README.md
```

**两个 MCU 仓各自**：

```bash
git submodule add git@github.com:VANexus/vehical_rak_platform.git platform
# → 仓内出现 platform/（不是 ../platform），.gitmodules 与 submodule 指针入库
git clone --recursive …            # 单独 clone 后一条命令补齐
```

**顶层 `CMakeLists.txt`**（CubeMX 只生成一次、不覆盖，是官方指定的挂载点）：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    platform/Src/app_oled.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc          # 放前面 → 本仓 board_c30d.h 优先
    platform/Inc)
```

### 为什么是 submodule（D2 的理由，记录备查）

三条要求同时成立：**单一源码**（架构 §4）· **无漂移**（架构 §4 否决了"各拷一份"）· **单独 clone 可构建**（`AGENTS.md` §8.3 + 本仓 README）。
只有 submodule 能同时满足。伞形仓托管（P1）会让"可单独 clone 构建"变成假话；寄生在 motion 仓（P3）会让 edge_ai 的构建依赖于姊妹仓存在。

---

## 4. 抽取边界：驱动进 `platform/`，内容留各仓

| 归属 | 内容 | 行数 |
| --- | --- | --- |
| **`platform/Src/app_oled.c`** | DWT 毫秒延时 · 软 SPI 位操作 · SSD1306 初始化序列 · 5×7 字库 · 帧缓冲 `frame[8][128]` · `pixel/glyph` · `oled_flush` | ~150 |
| **`platform/Inc/app_oled.h`** | `oled_init()` · `oled_clear()` · `oled_text(x,y,s)` · `oled_flush()` · `oled_set_font()` | — |
| **`motion/Core/Src/app_oled.c`** | 现有 `render()` 原样保留（**本次不动**，见 D3） | ~95 |
| **`edge_ai/Core/Src/app_oled_render.c`** | 新建（内容见 §6） | — |

### 公开 API（A1 只定这些）

```c
void oled_init(void);                              /* 幂等；首次调用才动 GPIO/时序 */
void oled_clear(void);                             /* 清帧缓冲，不推屏 */
void oled_text(unsigned x, unsigned y, const char *s);  /* 写帧缓冲；全大写（A1 字库限制） */
void oled_flush(void);                             /* 位操作推屏 */

/* 字库"接缝"：A2 换中文字库/多字号时不用改本头文件 */
typedef struct { uint8_t w, h; const uint8_t *glyphs; uint16_t first, last; } oled_font_t;
void oled_set_font(const oled_font_t *f);          /* NULL = 默认 5×7 ASCII */
```

**A1 故意不做**（YAGNI —— 用户已明确"未来会专门做 ui/ux"）：`oled_rect` / `oled_bar` / `oled_bitmap` / 双缓冲 / 页面状态机 / 按键处理。
**A1 只保证"接缝位置对"**，不预造 A2 的轮子。唯一例外是 `oled_font_t` 这层间接 —— 现在放进去是几行，等 A2 再改就是**改公开 API**、两个仓都要动。

### 必须写进代码注释的两条约束

1. **`oled_flush()` 是阻塞的**（1024 字节位操作，量级 ~1 ms）。架构 §3 说 L1"调用即返回，不阻塞" —— 软 SPI 做不到，**这是显式例外**。规定：**只能在最低优先级任务调用，绝不可进任何 ISR**（`AGENTS.md` §3 的 ISR 禁令）。
2. **字库只到大写 `'Z'`**，小写字母会被静默吞成空格。屏上内容必须全大写 —— 包括 `build_tag`。

---

## 5. 引脚外置到 `board_c30d.h`

驱动里 `pin_set(11, …)` / `pin_set(14, …)` 是硬编码，按架构 §4「板级差异全部走 `board_c30d.h`」外置：

```c
/* 每工程各一份 Core/Inc/board_c30d.h */
#define OLED_PORT       GPIOD
#define OLED_PIN_DC     11U
#define OLED_PIN_RES    12U
#define OLED_PIN_DATA   13U
#define OLED_PIN_CLK    14U
```

两块板硬件相同 → 两份值一样，但**必须分两份**；`platform/` 内不得 include 工程专属头，由工程侧在 include 路径里优先提供。

---

## 6. edge_ai 侧内容（A1 只做最小可用集）

A1 交付 `edge_ai/Core/Src/app_oled_render.c`，只显示**本仓当前确实有的**数据：

| 行 | 内容 | 数据来源 |
| --- | --- | --- |
| y=0 | `build_tag` + 运行秒数 | **本方案新增**（见下） |
| y=10 | CAN 状态占位（`CAN --`） | 子项目 C 落地前先占位 |
| y=20 | 空 | — |
| y=30 | 空 | — |

### 必须新增：`build_tag`

`AGENTS.md` §5 规则 4 要求**两块板**「冷启动后先在屏上确认标签再判行为」。实测：

```
motion  : Core/Src/app_car.c:141   const char build_tag[] = "R63";   ✅ 有
edge_ai : 全仓搜 build_tag          →  无                            ❌ 缺
```

→ edge_ai 需要 `Core/Src/app_build.c` 定义 `const char build_tag[]`，初值 `"E1"`（`E` = edge_ai，与 motion 的 `R` 序列区分）。
**没有它，edge_ai 上板后无法判别实际运行的固件版本** —— 这正是本子项目要交付的第一件东西。

---

## 7. 验证判据

| # | 判据 | 方法 |
| --- | --- | --- |
| V1 | `platform/` 与 `motion` 的驱动部分**逐字节一致**（抽取无改动） | `diff` 抽出的函数体 |
| V2 | **motion 仓零改动** | `git -C vehical_rak_motion status --short` 为空（D3） |
| V3 | edge_ai Debug + Release **零警告零错误** | `cmake --preset Debug/Release && cmake --build …` |
| V4 | 协议门禁 | `python tools/can_check.py` exit 0 |
| V5 | 上板：冷启动后屏上出现 `E1` | 真断电冷启动（电池关→拔 ST-LINK→等 OLED 全黑→上电），目视 |

---

## 8. 未决 / 风险

| # | 项 | 处置 |
| --- | --- | --- |
| R1 | **`gh` CLI 未安装** → `VANexus/vehical_rak_platform` 建不了 | 需用户在 GitHub 建仓（或用本地路径 submodule 过渡，之后 `git submodule set-url` 切换） |
| R2 | **D3 造成的驱动重复**（§2 的 ⚠️） | 已接受；在 motion README 记 TODO，等 motion 侧同意再收敛 |
| R3 | 驱动位操作**无延时**，SCK 速率可能超出 SSD1306 上限 | 台架已验证可用（R4 起在跑），但**未实测 SCK 频率**。A2 上板时用逻辑分析仪量一次 |
| R4 | `oled_flush()` 阻塞时长是**推断**（~1 ms），未实测 | A2 上板用 DWT 打点实测，记入提交说明 |
| R5 | `motion` 仓的 `app_oled.c` 未来收敛到 `platform/` 时，需要一次 motion 侧改动 | 届时单独走一轮，本方案不含 |

---

## 9. 不做的事（明确划界）

- ❌ 不改 `vehical_rak_motion` 的任何代码（D3）
- ❌ 不做 edge_ai 的高级 UI —— 页面 / 按键 / 波形 / 条形图 / 脉冲栅格 / 中文字库，全部属 **A2**
- ❌ 不动 ST7735S / 蓝牙 / CAN / X-CUBE-AI（各自独立子项目）
- ❌ 不引入任何第三方图形库

---

## 10. 关联文档

- [`docs/architecture.md`](../../../../docs/architecture.md) §3-§4 — 分层与 `platform/` 决策的 SSOT
- [`docs/BOARD-C30D.md`](../BOARD-C30D.md) — 引脚与硬件真相
- [`../vehical_rak_motion/docs/CUBEMX-SOP.md`](../../../vehical_rak_motion/docs/CUBEMX-SOP.md) — 改外设的标准流程
- `AGENTS.md` §3（分层）/ §5（上板规矩）/ §6（提交要求）
