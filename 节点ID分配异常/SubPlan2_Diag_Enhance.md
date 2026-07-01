# 子方案2：从机枚举诊断增强 — 单从机 POLL×4 现象定位

> 父方案：[TEST_PLAN.md](Test_Plan.md)
> 前置子方案：[SubPlan1_Slave_Diag.md](SubPlan1_Slave_Diag.md)（Phase 0 诊断日志打桩，已完成代码）
> 范围：仅从机 APM32F103CB 侧 + 主控 ESP32 侧日志观察（不改 ESP32 固件）
> 版本：v1
> 日期：2026-06-29

---

## 一、背景

### 1.1 前置方案完成情况

SubPlan1（从机侧枚举诊断日志）的代码已落地。`enum_diag.c/h` 以"增添而非修改"策略实现：

- 5 个自含 UART TX 原语（`EnumDiagSendChar` / `SendString` / `SendUint` / `SendHex32` / `SendLineEnd`）
- 4 个枚举帧诊断函数（`EnumDiagLogPoll` / `LogOffer` / `LogAssign` / `LogAckSent`）
- 1 个 CAN 错误告警函数（`EnumDiagLogCanError`）
- `can_service.c` 中 7 处诊断调用点已插入
- `main.c` 中已有运行时调试输出已用 `#ifndef ENUM_DIAG` 抑制


### 1.2 单从机调试发现

在单从机 + 单主控的最简环境下（ESP32 主控固件未修改），使用 SubPlan1 的诊断固件直接串口输出，观察到**系统的帧接收时序异常**：

| 场景 | 观察 |
|------|------|
| 直接串口输出（无调试器） | 连续 4 个 `RX POLL, myId=0, skip (UNSET)` → 第 4 个 POLL 后几乎紧接 `RX OFFER id=1, myId=0, accept` |
| Keil 断点置于 OFFER 条件内 | 第 1 轮仅打印 POLL（断点未命中）→ 第 2 轮 POLL 后断点命中 OFFER |
| 偶发极端情况 | 持续只收到 POLL，OFFER 从未出现 |

**预期行为**（按 SubPlan1 第六节的理想日志）：上电后第 1 个 cycle 内即应同时出现 POLL 和 OFFER，第 1 轮完成 ID 分配。

**实际行为**：在无任何外部干预的情况下，需要 4 个 cycle（约 2.6 秒 @ 650ms/cycle）才能收到 OFFER。

### 1.3 代码流程确认

经代码审查确认，APM32 单次主循环 tick（10ms）内的 CAN 帧处理路径为：

```
CanServicePoll()                          // 读 1 帧
  └─ CanServiceHandleEnum(&rx)            // 若 FIFO[0]=POLL → 处理 POLL → 返回
     （POLL 分支：NodeIdGet → EnumDiagLogPoll → 若 myId≥1 则 schedule ACK → return true）

CanServiceReceiveCommand()                // budget=8，排空 FIFO
  └─ 读下一帧 → CanServiceHandleEnum()    // 若 FIFO[1]=OFFER → 处理 OFFER → 返回
     （OFFER 分支：NodeIdSet → schedule ACK → EnumDiagLogOffer → return true）
```

**关键结论**：POLL 和 OFFER 是两次独立的 FIFO 读取，对应两次独立的 `CanServiceHandleEnum` 调用。若两个帧都在 FIFO 中，它们会在同一 tick 内被先后处理，日志输出应有相同的 tick 时间戳。

ESP32 侧每个 cycle 发送 POLL+OFFER 背靠背（间隔 ~1-2ms，阻塞式 `twai_transmit`），cycle 间隔为 650ms（250ms 采集窗口 + 400ms 活跃间隙）。APM32 CAN RX FIFO0 有 3 级深度。在 650ms 间隔下，每个 cycle 仅 2 帧进入 FIFO，不存在 FIFO 溢出风险。

**因此，连续 4 cycle 只收到 POLL 而无 OFFER，意味着 OFFER 帧在前 3 个 cycle 中未进入 APM32 的 CAN RX FIFO。**

---

## 二、可能根因猜想

### 猜想 A：ESP32 侧 OFFER 发送静默失败（可能性：最高）

**代码依据**：[enum_manager.c:37-41](motor_control_apm32/../ble_esp32/main/app/enum_manager.c#L37-L41)

```c
static void send_offer(uint8_t id)
{
    uint8_t d[1] = { id };
    hal_can_send(CAN_STD_ID_ENUM_OFFER, d, 1, 10);  // 返回值被丢弃！
}
```

`send_poll()` 和 `send_offer()` 均忽略 `hal_can_send` 的返回值。若 ESP32 TWAI 控制器在冷启动后头几个 cycle 的 `twai_transmit(OFFER)` 返回失败（如 TX queue 未完全就绪、总线仲裁异常），`s_tx_fail` 会递增但枚举逻辑完全不知道。

**与观察的匹配度**：

- ✅ 一旦某次 OFFER 发送成功，后续 cycle 稳定正常——符合"4 个 POLL 后突然出现 OFFER，之后不再消失"
- ✅ POLL 持续被收到说明总线基本通路正常——OFFER 选择性缺失指向发送端而非接收端
- ⚠️ 无法直接解释 Keil 断点改变了行为——断点改变的是 APM32 侧时序，不应影响 ESP32 侧发送

**验证手段**：监视 ESP32 串口的 diagnostics_task 输出（每秒一行）：

```
[CAN] state=RUNNING TEC=0 REC=0 | tx_ok=N tx_fail=M ...
```

若上电后 `tx_fail > 0` 则直接确认。

### 猜想 B：APM32 CAN 控制器冷启动同步延迟（可能性：中等）

**机制**：APM32 CAN 控制器使用 HSI→PLL 产生的 36MHz PCLK1，与 ESP32 晶振存在微小频偏。CAN 协议通过每帧 SOF 的硬同步和相位重同步来容忍频偏（最大 ±1.58% for 50kbps）。在总线从空闲恢复后的第一个帧（冷启动场景），控制器需要从隐性电平→SOF 下降沿完成硬同步。如果频偏刚好在临界值，前几个帧可能出现采样点偏差，导致该帧被硬件以 CRC/Form Error 丢弃。

**与观察的匹配度**：

- ✅ 解释 Keil 断点改变行为——断点暂停 CPU 但不暂停 CAN 控制器，改变了 CAN 错误计数器的增长节奏
- ✅ 解释"偶发永远收不到 OFFER"——错误累积导致进入 Error Passive
- ⚠️ CAN 协议设计理论上第 1 帧就能正确同步。需要硬件错误计数器来证实

**验证手段**：在 `CanServicePoll()` 开头读取 CAN ES (Error Status) 寄存器中的 LEC[2:0]（Last Error Code），以及 FIFO 溢出标志 `CAN_FLAG_F0OV`。若在 OFFER 缺失期间看到非零 LEC，则证实。

### 猜想 C：`gateway_broadcast_config()` 帧干扰首个 cycle（可能性：较低）

**代码依据**：[gateway.c:291-309](motor_control_apm32/../ble_esp32/main/app/gateway.c#L291-L309)

ESP32 在进入主循环之前（boot 阶段）通过 `gateway_broadcast_config()` 发送了一帧 CONFIG 广播（MASTER_CAN_ID=0x00, 8 字节数据）。这是 ESP32 CAN 控制器启动后的**第一帧**。

此帧会被 APM32 正常接收（符合滤波器，见 `board_driver.c:85`），处理后丢弃。它占用 FIFO 一个槽位，但不应影响后续 POLL/OFFER 的接收（FIFO 有 3 级深度，且 APM32 在收到此帧后、ESP32 进入主循环前有足够时间排空 FIFO）。

### 猜想 D：CAN 总线物理层问题（可能性：低，但不能排除）

POLL（ID=0x7E3）和 OFFER（ID=0x7E0）仅末尾 2 bit 不同。若总线存在间歇性接触不良、终端电阻未正确就位或 EMI 干扰，理论上不应选择性丢失特定 ID 的帧。但 CAN 总线物理层问题在低频（50kbps）短距离下极为罕见。

### 猜想之间的关系

多个猜想可以叠加。最可能的复合场景：**猜想 A（ESP32 前几个 OFFER 发送失败）为主因，猜想 B（APM32 CAN 同步边界问题）为偶发加剧因素**。这能解释"通常 4 轮恢复，偶尔永不恢复"的双模式现象。

---

## 三、与 Issue #32 主问题的关联

### Issue #32 核心症状

> 6 节点枚举中，偶发某电机单元未能正确捕获/响应枚举帧，导致节点 ID 分配异常（漏分配 / 重号 / 需额外 ASSIGN 重定位才收敛）。不可稳定复现。

### 单从机 POLL×4 与 Issue #32 的关系

| 维度       | 单从机 POLL×4                     | Issue 32（6 从机）                   |
| -------- | ------------------------------ | -------------------------------- |
| 受影响的从机数  | 1（唯一的从机）                       | 偶发 1/6                           |
| 延迟周期     | ~4 cycle                       | 不明（可能更多）                         |
| 是否自愈     | 是（第 4 cycle 后收到 OFFER）         | 有时需 ASSIGN 重定位，有时永远缺 ID          |
| CAN 总线负载 | 极低（每 cycle 仅 POLL+OFFER 共 2 帧） | 高（POLL+OFFER+最多 6 ACK+可能 ASSIGN） |

**关联判断**：

单从机 POLL×4 现象很可能是 Issue #32 根因在**最低负载下的纯净呈现**。两者共享同一失效模式：**枚举早期 cycle 中 OFFER 帧未能被从机接收**。

在 6 从机场景下，该根因会被以下因素放大：

1. **FIFO 压力**：3 级 FIFO 面对 POLL+OFFER+N 个其他从机的 ACK/反馈帧，单 tick 可能到达 4+ 帧，FIFO 溢出风险骤增
2. **ACK 碰撞**：6 节点在 48ms 窗口内竞争，退避碰撞概率随节点数上升
3. **错误传播**：若某从机因猜想 B 进入 Error Passive，其 CAN 外设被静默复位期间（`CanServiceCheckBusHealth`），该从机完全脱离总线

**如果确认了单从机 POLL×4 的根因，Issue #32 的修复方向将明确化**：

| 根因 | 修复方向 |
|------|---------|
| 猜想 A 证实 | ESP32 `send_offer()` 检查返回值 + 失败重试；或在 `enum_manager_tick` 首 cycle 前加 CAN 稳定等待 |
| 猜想 B 证实 | APM32 `BoardCanInit()` 后加 CAN 控制器稳定等待；或在 `CanServiceCheckBusHealth` 中增加诊断可见性 |
| 猜想 A+B 叠加 | 两端同时加固，从协议层面增加首次枚举的超时/重试容错 |

---

## 四、诊断增强方案

SubPlan1 的 `enum_diag.c` 缺少**时间戳**和**FIFO 溢出检测**，无法区分"OFFER 帧未到达 APM32"和"OFFER 帧到达了但被 FIFO 覆盖后丢弃"。本次增强在已有代码基础上追加两项诊断能力。

### 4.1 日志中追加 tick 时间戳

**目的**：确认 POLL 和 OFFER 是否在同一个 tick 内被处理。若同一 tick 内 POLL+OFFER 同时出现，说明两帧均在 FIFO 中；若 POLL 和 OFFER 分属不同 tick，说明 OFFER 在另一个 cycle 才到达。

**修改位置**：`enum_diag.c` — `EnumDiagPrintHeader()` 函数体内，在输出 `[UID=XXXXXXXX]` 之前追加 tick 值前缀。

**输出格式变更**：

```
// 变更前
[UID=15274C76] RX POLL, myId=0, skip (UNSET)

// 变更后
[t=000004D2][UID=15274C76] RX POLL, myId=0, skip (UNSET)
```

其中 `t=` 后的值为 `SystemTickGetMs()` 的 32 位毫秒 tick，以 8 位大写十六进制输出（复用已有的 `EnumDiagSendHex32` 函数，零新增代码）。

**分析价值**：

- 正常情况（OFFER 在 FIFO 中）：同一 tick 值连续出现两行（POLL + OFFER）
- 异常情况（OFFER 缺失）：POLL 出现后，下一个日志的 tick 值跳变 ≥650ms（下一个 cycle），期间无 OFFER

**实现**：

- `enum_diag.c` 新增 `#include "system_tick.h"`（`SystemTickGetMs()` 声明所在头文件）
- `EnumDiagPrintHeader()` 开头插入 `[t=XXXXXXXX]` 前缀
- 声明文件 `enum_diag.h` 无需修改（`EnumDiagPrintHeader` 是 static 函数）

**字符增量**：

- 新增固定 12 字符：`[t=` (3) + 8 hex 位 + `]` (1) = 12
- 使用 `EnumDiagSendHex32` 输出 tick 值（固定 8 hex 字符），比 `EnumDiagSendUint` 的十进制变长方案更快且无字符数波动
- 单行增量：12 字符 × 86.8μs/字符 ≈ **1.0ms**

> **设计说明**：选用 hex 而非十进制是因为 `EnumDiagSendHex32` 循环展开为 8 次定长输出（每次 1 个 hex 字符），无需 `EnumDiagSendUint` 的除 10 取模循环。固定 8 字符宽度也便于上位机解析对齐。

**单 tick 总耗时影响**：

| 场景 | 命中打印点 | 当前耗时 | 加时间戳后 | 增量 |
|------|-----------|---------|-----------|------|
| 单从机，POLL+OFFER 同 tick | ①+② | ~8.1ms | ~10.1ms | **+2.0ms** |
| 单从机，POLL+OFFER+ACK 同 tick | ①+②+④ | ~10.4ms | ~12.8ms | **+2.4ms** |
| 6 从机首轮，POLL+OFFER+ASSIGN | ①+②+③ | ~13.3ms | ~16.0ms | **+2.7ms** |

在枚举阶段（不走电机控制闭环），单 tick 12-16ms 的时长可接受。CAN 硬件收帧独立于 CPU 的 UART 阻塞，不会因此丢失帧。

### 4.2 CanServicePoll 中增加 FIFO 溢出检测

**目的**：确认 APM32 CAN RX FIFO0 是否在枚举期间发生过溢出（overrun）。溢出意味着在 CPU 读完一帧之前，FIFO 已被后续到达的帧填满（≥4 帧在极短时间内到达），导致最旧的帧被丢弃。

**修改位置**：`can_service.c` — `CanServicePoll()` 函数体内，在 `CanServiceServiceAck()` 调用之后、`CAN_ReadStatusFlag(F0MP)` 检查之前。

**插入代码**：

```c
#ifdef ENUM_DIAG
if (CAN_ReadStatusFlag(CAN1, CAN_FLAG_F0OV) != RESET)
{
    /* Minimal log: only emit on actual overflow event */
    EnumDiagLogFifoOverrun();
    CAN_ClearFlag(CAN1, CAN_FLAG_F0OV);
}
#endif
```

**新增函数**：`enum_diag.c` 中追加 `EnumDiagLogFifoOverrun()`（约 6 行），输出：

```
[t=000004D2][UID=15274C76] FIFO OVERRUN
```

**声明**：`enum_diag.h` 中 `#ifdef ENUM_DIAG` 块内追加 `void EnumDiagLogFifoOverrun(void);`

**时序影响**：

- **未溢出时（正常情况）**：仅多一次 `CAN_ReadStatusFlag(CAN1, CAN_FLAG_F0OV)` 寄存器读取（~100ns），**零 UART 输出，零阻塞**
- **溢出时（罕见事件）**：多输出一行约 38 字符的短日志（~3.3ms），同时清空溢出标志

**分析价值**：

- 若在 OFFER 缺失的 cycle 中看到 `FIFO OVERRUN` → 说明有其他 CAN 帧填满了 FIFO，OFFER 被挤出
- 若 OFFER 缺失但没有 `FIFO OVERRUN` → 说明 OFFER 根本没到达 APM32（指向猜想 A 或 B）

### 4.3 ESP32 侧日志监视（不改固件）

ESP32 的 `diagnostics_task`（[main.c:145-179](motor_control_apm32/../ble_esp32/main/main.c#L145-L179)）每秒输出一行 CAN 诊断，包含：

```
[CAN] state=RUNNING TEC=N REC=M | tx_ok=X tx_fail=Y last_tx=Zms_ago | rx=W last_rx=Vms_ago
```

**操作**：在 APM32 串口采集的同时，另开一个终端窗口接 ESP32 的 USB 串口，观察上电后第一秒的诊断输出。

**分析价值**：

- `tx_fail > 0` → **直接锁定猜想 A**（ESP32 侧发送失败）
- `tx_fail = 0` 且 `tx_ok` 正常递增 → 排除猜想 A，问题在 APM32 接收侧（猜想 B）
- `TEC` 或 `REC` 持续增长 → CAN 总线物理层存在错误（猜想 D）

---

## 五、修改清单

### 5.1 文件修改汇总

| 文件 | 改动类型 | 改动量 | 说明 |
|------|---------|--------|------|
| `app/Source/enum_diag.c` | 修改 + 新增 | +15 行 | 新增 `#include "system_tick.h"`；`EnumDiagPrintHeader` 追加 tick 前缀；新增 `EnumDiagLogFifoOverrun` 函数 |
| `app/Include/enum_diag.h` | 修改 | +1 行 | `#ifdef ENUM_DIAG` 块内追加 `EnumDiagLogFifoOverrun` 声明 |
| `app/Source/can_service.c` | 修改 | +6 行 | `CanServicePoll` 函数体内追加 FIFO 溢出检测代码块 |
| ESP32 固件 | **不改** | — | 仅被动观察串口日志 |

### 5.2 不改文件

| 文件 | 原因 |
|------|------|
| `main.c` | 已有 `#ifndef ENUM_DIAG` 噪声抑制已就位，无需额外修改 |
| `app_config.h` | 枚举参数均已定义 |
| `node_id.c/h` | 接口已满足需求 |
| `can_service.h` | 修改均在 `.c` 内部 |
| `board_driver.c` | CAN 滤波器配置已就位（接受 POLL/OFFER/ASSIGN/MASTER/OTA 五组 ID） |
| ESP32 全部文件 | 主控固件本轮不修改 |

### 5.3 编译

与 SubPlan1 相同，使用 Keil 在 `ENUM_DIAG` 宏定义下编译。

---

## 六、预期结果与判据

### 6.1 若猜想 A 被证实（ESP32 tx_fail > 0）

**日志特征**：

```
ESP32:  [CAN] state=RUNNING ... | tx_ok=2 tx_fail=4 ...   ← 头几个 cycle 的 OFFER 发送失败
APM32:  [t=000004D2][UID=15274C76] RX POLL, myId=0, skip (UNSET)
        [t=0000075A][UID=15274C76] RX POLL, myId=0, skip (UNSET)   ← tick 跳变 648ms，无 OFFER
        [t=000009E2][UID=15274C76] RX POLL, myId=0, skip (UNSET)   ← 仍然无 OFFER
        [t=00000C6A][UID=15274C76] RX POLL, myId=0, skip (UNSET)   ← 仍然无 OFFER
        [t=00000EF2][UID=15274C76] RX POLL, myId=0, skip (UNSET)   ← POLL
        [t=00000EF2][UID=15274C76] RX OFFER id=1, myId=0, accept   ← 同 tick OFFER！恢复
```

**后续行动**：修改 ESP32 枚举管理器，在 `send_poll()`/`send_offer()` 中检查返回值并对失败做重试或告警。

### 6.2 若猜想 B 被证实（tx_fail=0，但 APM32 有 CAN 错误或 FIFO 溢出）

**日志特征**：

```
APM32:  [t=000004D2][UID=15274C76] FIFO OVERRUN                    ← 溢出！
        [t=000004D2][UID=15274C76] RX POLL, myId=0, skip (UNSET)
        // OFFER 被 FIFO 溢出丢弃
        [t=0000075A][UID=15274C76] RX POLL, myId=0, skip (UNSET)
        [t=0000075A][UID=15274C76] RX OFFER id=1, myId=0, accept   ← 恢复
```

或：

```
APM32:  [t=000004D2][UID=15274C76] CAN ERR: TEC=0 REC=1, resetting CAN controller
```

**后续行动**：调查 FIFO 溢出来源（是否有非预期的 CAN 帧在总线空闲期间涌入）；或在 `BoardCanInit` 后增加 CAN 控制器稳定等待。

### 6.3 若猜想 C/D 均不成立，且无溢出

`tx_fail = 0`，无 `FIFO OVERRUN`，无 `CAN ERR`，但仍有 POLL×4 现象 → 需要进一步工具（CAN 分析仪抓包）来区分是 ESP32 发出的 OFFER 在总线上丢失还是 APM32 CAN 硬件在无错误标志的情况下丢弃了帧。此种情况可能需要检查 APM32 CAN 控制器的勘误表。

---

## 七、执行顺序

| 优先级 | 步骤 | 改动量 | 耗时 | 产出 |
|--------|------|--------|------|------|
| **P0** | 接 ESP32 串口，观察 `tx_fail` | **零代码** | 5 min | 确认/排除猜想 A |
| **P1** | 实施 4.1（tick 时间戳） | ~15 行 | 15 min | 精确量化每帧到达的 tick |
| **P1** | 实施 4.2（FIFO 溢出检测） | ~7 行 | 10 min | 确认/排除 FIFO 溢出 |
| **P2** | 编译烧录，重新采集单从机日志 | — | 10 min | 获得带时间戳 + 溢出标志的诊断数据 |
| **P3** | 若猜想 A 证实 → 修改 ESP32 `send_offer` 检查返回值 | TBD | 30 min | 修复 ESP32 侧 |
| **P3** | 若猜想 B 证实 → 调查 CAN 错误来源 | TBD | TBD | 修复 APM32 侧 |
| **P4** | 修复后按 [Test_Plan](Test_Plan.md) Phase 2 执行 50 轮上电测试 | — | 13 min | 验证修复效果 |
| **P5** | 扩展到 3 从机 + 6 从机验证 | — | 按需 | 确认 Issue #32 是否解决 |

---

## 八、与 Test_Plan 的关系

本子方案（SubPlan2）是 Test_Plan Phase 1（基础功能确认）执行过程中发现的异常所触发的**诊断增强**。它不替代 Test_Plan 的 Phase 2-5，而是为 Phase 2 的批量测试提供更高精度的诊断工具。

Test_Plan 的 Phase 2（50 轮耐久测试）应在 SubPlan2 的增强代码到位后、且单从机行为恢复正常后再启动。这样每轮测试的日志都带有时间戳和溢出标志，异常判定不再依赖人工经验，而是有精确的毫秒级帧间隔数据。

---

## 九、影响评估

### 9.1 UART 阻塞时序

| 项目 | SubPlan1（当前） | SubPlan2（增强后） | 增量 |
|------|----------------|-------------------|------|
| 单行日志字符数 | 27~60 | 39~72 | +12 |
| 单行耗时 @115200 | 2.3~5.2ms | 3.4~6.2ms | +1.0ms |
| 典型 tick（POLL+OFFER） | ~8.1ms | ~10.1ms | +2.0ms |
| 最坏 tick（POLL+OFFER+ACK+溢出） | ~13.3ms | ~16.6ms | +3.3ms |
| FIFO 溢出检测（正常情况） | — | ~100ns | **忽略不计** |
| FIFO 溢出检测（溢出时） | — | ~3.3ms 单行 | 仅溢出时发生 |

### 9.2 对枚举行为的影响

- 枚举阶段不走电机控制闭环，tick 时长从 10ms 延长到 12-16ms 不影响枚举逻辑
- CAN 硬件收帧独立于 CPU，UART 阻塞不会造成丢帧
- 650ms cycle 间隔远大于最坏 tick 延迟，不存在"上一 tick 的 UART 输出延迟了下一 cycle 帧的读取"问题
- 若 `ENUM_DIAG` 未定义，所有新增代码被预处理器移除，二进制与正式固件完全一致
