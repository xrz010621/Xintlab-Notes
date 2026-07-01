# 子方案1：从机侧枚举诊断日志

> 父方案：[TEST_PLAN.md](Test_Plan.md) — Phase 0 打桩（从机侧内容细化）
> 范围：仅从机 APM32F103CB 侧，不改主控 ESP32
> 版本：v1

---

## 一、目标

在主控上电自动发送 POLL/OFFER/ASSIGN 完成枚举的过程中，通过从机调试 UART 输出诊断日志，使上位机能观察到每块从机的**每一帧收发情况**，从而定位"某块从机在哪个环节漏了帧"。

---

## 二、数据采集方式

- 每个从机通过 **USART1（PA9 TX, 115200 8N1）** 输出日志，协议为纯文本行
- 上位机通过 USB-UART 桥接器（CH340）接收，3 路串口分别接入 3 个终端窗口或由 Python 脚本合并采集
- 每行日志以 `[UID=XXXXXXXX]` 开头（8 位十六进制），天然区分不同从机，无需额外标识

---

## 三、从机枚举帧处理路径（代码依据）

### 3.1 主循环调用链

`main.c:679-801` `RunControlLoop()` 每 10ms 执行一次，枚举相关调用顺序：

```
CanServiceCheckBusHealth()          → 检查 CAN 是否 Error Passive，是则静默复位
CanServicePoll()                    → can_service.c:379-396
  ├─ CanServiceServiceAck()         → 若退避到期，发出 ACK
  └─ 读 FIFO0 1 帧 → CanServiceHandleEnum()
while CanServiceReceiveCommand()    → can_service.c:198-303
  └─ 每帧也走 CanServiceHandleEnum()（budget=8）
```

**关键设计**：`CanServicePoll()` 只读 1 帧，`CanServiceReceiveCommand()` 循环排空剩余。两者都通过 `CanServiceHandleEnum()` 处理枚举帧。因此一次 10ms tick 内最多可处理 9 帧（1+8 budget）。

出处：[can_service.c:379-396](motor_control_apm32/app/Source/can_service.c#L379-L396)（CanServicePoll）、[can_service.c:218-255](motor_control_apm32/app/Source/can_service.c#L218-L255)（CanServiceReceiveCommand 的 FIFO 排空循环）

### 3.2 枚举帧处理入口

`CanServiceHandleEnum()` 是枚举帧的集中处理函数，按 STD ID 分发：

| CAN ID | 宏 | 含义 | 代码位置 |
|--------|----|------|---------|
| `0x7E3` | `CAN_STD_ID_ENUM_POLL` | 主控点名 | [can_service.c:106-113](motor_control_apm32/app/Source/can_service.c#L106-L113) |
| `0x7E0` | `CAN_STD_ID_ENUM_OFFER` | 主控提供空闲 ID | [can_service.c:118-128](motor_control_apm32/app/Source/can_service.c#L118-L128) |
| `0x7E2` | `CAN_STD_ID_ENUM_ASSIGN` | 主控定向重定位 | [can_service.c:132-147](motor_control_apm32/app/Source/can_service.c#L132-L147) |

出处：[can_service.c:98-150](motor_control_apm32/app/Source/can_service.c#L98-L150)（CanServiceHandleEnum 全部逻辑）

### 3.3 ACK 退避与发送机制

从机不是立即回复 ACK，而是通过**单槽异步退避**：

1. `CanServiceScheduleAck(id)` — [can_service.c:71-82](motor_control_apm32/app/Source/can_service.c#L71-L82) — 计算 `hash(UID, seq) % 48ms` 的退避时间，登记到全局单槽 `s_ackPending/Id/DueMs`。**后一次调用会覆盖前一次**。
2. `CanServiceServiceAck()` — [can_service.c:86-94](motor_control_apm32/app/Source/can_service.c#L86-L94) — 由 `CanServicePoll()` 每 tick 调用一次，检查退避是否到期，到期则发出 ACK。

退避参数 `NODE_ID_ENUM_ACK_SPREAD_MS = 48ms`，定义见 [app_config.h:35](motor_control_apm32/app/Include/app_config.h#L35)。

### 3.4 总线健康检查

`CanServiceCheckBusHealth()` — [can_service.c:188-196](motor_control_apm32/app/Source/can_service.c#L188-L196) — 检测到 CAN Error Passive（`CAN_FLAG_ERRP`）时，静默调用 `CAN_Reset()` + `BoardCanInit()` + `CanServiceInit()` 复位整个 CAN 外设。**当前没有任何日志输出**，复位期间的 RX/TX 帧全部丢失。

---

## 四、诊断打印点设计

### 打印点 ① — 收到 POLL

**触发**：`CanServiceHandleEnum()` 匹配到 `CAN_STD_ID_ENUM_POLL`（当前 [can_service.c:106-113](motor_control_apm32/app/Source/can_service.c#L106-L113) 的逻辑）

**打印内容**：

```
[UID=XXXXXXXX] RX POLL, myId=N, will ACK
[UID=XXXXXXXX] RX POLL, myId=0, skip (UNSET)
```

**诊断意义**：

| 日志中 myId 值 | 含义 | 正常/异常 |
|---------------|------|----------|
| 1~6 | 从机已分配 ID，会 schedule ACK | 正常 |
| 0 (UNSET) | 从机尚无 ID，POLL 对它无效 | 正常（首轮枚举中，或 OFFER 漏帧） |
| 连续多周期始终为 0 | 从机一直没收到 OFFER 或 OFFER 被忽略 | **异常**，需对照 OFFER 日志 |

**预期频率**：每个枚举 cycle 1 次（400ms 活跃期 / 1s 稳态期），稳定时每个 cycle 都应出现。

### 打印点 ② — 收到 OFFER

**触发**：`CanServiceHandleEnum()` 匹配到 `CAN_STD_ID_ENUM_OFFER`（当前 [can_service.c:118-128](motor_control_apm32/app/Source/can_service.c#L118-L128) 的逻辑）

**打印内容**：

```
[UID=XXXXXXXX] RX OFFER id=N, myId=M, accept, ack pending
[UID=XXXXXXXX] RX OFFER id=N, myId=M, skip (already assigned)
[UID=XXXXXXXX] RX OFFER id=N, myId=M, skip (invalid id)
```

**诊断意义**：

| 日志中判定 | 含义 | 正常/异常 |
|-----------|------|----------|
| `accept, ack pending` | UNSET 从机接受 OFFER，schedule ACK | 正常 |
| `skip (already assigned)` | 已分配从机忽略 OFFER | 正常 |
| `skip (invalid id)` | OFFER 的 id 不在 1~6 范围内 | **异常**（协议错误） |
| 始终不出现 OFFER 日志 | 从机从未收到 OFFER 帧 | **异常**（CAN RX 丢帧） |

**这是最重要的打印点**。如果某块从机整段日志里只有 POLL(skip UNSET) 而从无 OFFER，说明 OFFER 帧在该从机的 CAN RX 路径上丢失。

**预期频率**：仅在从机仍为 UNSET 的 cycle 内出现。一旦从机获得 ID，后续 cycle 的 OFFER 会被忽略（`skip (already assigned)`）。

### 打印点 ③ — 收到 ASSIGN

**触发**：`CanServiceHandleEnum()` 匹配到 `CAN_STD_ID_ENUM_ASSIGN`（当前 [can_service.c:132-147](motor_control_apm32/app/Source/can_service.c#L132-L147) 的逻辑）

**打印内容**：

```
[UID=XXXXXXXX] RX ASSIGN id=N, target=TTTTTTTT, match, accept
[UID=XXXXXXXX] RX ASSIGN id=N, target=TTTTTTTT, not for me
```

**诊断意义**：

| 日志中判定 | 含义 | 正常/异常 |
|-----------|------|----------|
| `match, accept` | ASSIGN 帧目标 uid 与本从机匹配，接受新 ID | 正常（冲突重定位时） |
| `not for me` | 目标 uid 不匹配，忽略 | 正常（发给其他从机的重定位） |
| 出现 accept 但前序无任何冲突日志 | 主控检测到冲突（另一块板和本板抢同一 ID） | 正常，需对照另一块板的日志 |

**预期频率**：仅在发生 ID 冲突时出现。6 板首轮枚举中概率性出现（取决于 UID hash 是否碰撞到同一 ID），稳态后不应出现。

### 打印点 ④ — 实际发出 ACK

**触发**：`CanServiceServiceAck()` 中退避到期、调用 `CanServiceSendEnumAck()` 之前（当前 [can_service.c:86-94](motor_control_apm32/app/Source/can_service.c#L86-L94) 的逻辑）

**打印内容**：

```
[UID=XXXXXXXX] TX ACK id=N
```

**诊断意义**：

- 与打印点 ①/② 中的 "ack pending" 配对：出现 accept 后，应在 0~48ms 内出现对应的 TX ACK
- 若出现 accept 但长时间无 TX ACK → ACK 槽位被后续帧覆盖（单槽设计的预期行为：POLL 的 ACK 被 OFFER 的 ACK 覆盖，OFFER 的 ACK 被 ASSIGN 的 ACK 覆盖）
- 若出现 TX ACK 但下一个 cycle 主控的 POLL 没有正常回应 → ACK 在 CAN 总线上丢失（碰撞/仲裁/物理层）

**预期频率**：与 "will ACK" / "accept, ack pending" 出现次数接近 1:1。若 TX ACK 明显少于 schedule 次数，检查覆盖逻辑或时序。

---

## 五、额外诊断：CAN 错误状态【暂不进行】

### 5.1 总线健康检查时的日志

**触发**：`CanServiceCheckBusHealth()` 检测到 `CAN_FLAG_ERRP` 时，在执行 `CAN_Reset()` **之前**打印（当前 [can_service.c:188-196](motor_control_apm32/app/Source/can_service.c#L188-L196)，修改为打印后再复位）

**打印内容**：

```
[UID=XXXXXXXX] CAN ERR: TEC=XXX REC=XXX, resetting CAN controller
```

**诊断意义**：如果枚举期间 CAN 错误计数器（TEC/REC）持续增长至 >127 触发 Error Passive，CAN 外设被静默复位，复位期间所有收发帧均丢失。这是"从机突然不响应多周期"的**直接原因**之一。

### 5.2 周期性 CAN 状态上报（可选，ENUM_DIAG 开关控制）

**触发**：在主循环中每 500ms 检查一次 CAN 的 TEC/REC 和 RX FIFO 溢出标志

**打印内容**：

```
[UID=XXXXXXXX] CAN status: TEC=X REC=X, rx_count=N
```

**诊断意义**：即使未达到 Error Passive，TEC/REC 的增长趋势也是判断总线信号完整性的重要证据：
- TEC 增长 → 从机发送的 ACK 帧遇到仲裁丢失或错误
- REC 增长 → 从机接收到的帧有 CRC/格式错误
- 两者同时快速增长 → 总线物理层问题（终端匹配、走线、EMI）

---

## 六、正常枚举的预期日志序列

从机 UNSET 状态下，通电后完整枚举过程的理想日志：

```
[UID=A3F210B1] Node init done id=0 ...                         ← main.c:924-927 现有
[UID=A3F210B1] RX POLL, myId=0, skip (UNSET)                   ← cycle 1: 收到 POLL，无 ID
[UID=A3F210B1] RX OFFER id=1, myId=0, accept, ack pending      ← cycle 1: 收到 OFFER，接受
[UID=A3F210B1] TX ACK id=1                                      ← cycle 1: 退避到期发出
[UID=A3F210B1] RX POLL, myId=1, will ACK                        ← cycle 2: 已分配，回应 roll-call
[UID=A3F210B1] TX ACK id=1
[UID=A3F210B1] RX POLL, myId=1, will ACK                        ← cycle 3~: 稳态
[UID=A3F210B1] TX ACK id=1
```

---

## 七、异常场景的日志判定矩阵

| 观察到的日志特征 | 根因方向 | 后续验证 |
|-----------------|---------|---------|
| 连续多 cycle 仅 `RX POLL skip UNSET`，从不出现 `RX OFFER` | OFFER 帧未到达该从机（RX 丢帧 / FIFO 溢出 / CAN 过滤器误配置） | 检查从机 CAN 过滤器配置（当前为全接受）、RX FIFO 溢出标志 |
| 某从机的日志突然完全中断（无 RX POLL、无 RX OFFER、无 RX ASSIGN），直到多秒后恢复 | 从机 CAN 控制器进入 Error Passive 并被复位（`CanServiceCheckBusHealth` 静默复位） | 检查 CAN ERR 日志、TEC/REC 趋势 |
| `TX ACK` 正常，但下一 cycle 收到 `RX ASSIGN match accept`（被重定位到新 ID） | ACK 在总线上碰撞/丢失，主控未收到，判为冲突 | 对照主控侧 `enum_diag` 日志，CAN 分析仪抓包 |
| 出现 `accept, ack pending` 但短期内无对应 `TX ACK`，随后出现另一个 accept | ACK 槽位被后续帧覆盖（单槽设计的预期行为） | 检查两帧到达时间是否 < 退避时间 |
| 打印点②中 `accept, ack pending` 的同时 `myId` 已为非 0 值 | 接收逻辑的竞争条件或帧的时序异常 | 检查帧序列和 `NodeIdSet()` 调用历史 |
| 整个枚举过程（10 秒内）所有 RX 日志始终缺失某个 ID 帧类型 | RX 过滤问题或 CAN ID 宏定义不一致 | 检查 `CAN_STD_ID_ENUM_*` 宏定义和 CAN 过滤器配置 |

---

## 八、编译开关策略

所有新增枚举诊断日志用 `#ifdef ENUM_DIAG` / `#endif` 包裹，通过 CMake option 控制：

- `-DENUM_DIAG=ON`：编译诊断日志，用于测试
- 不加此 option：零开销，与正式固件二进制完全一致

**例外**：CAN 错误状态日志（打印点 5.1 — CAN Error Passive 复位时的打印）**不使用编译开关**，始终输出。这是一个异常事件的告警，不是常规诊断 trace，不应被屏蔽。

现有项目已有类似模式：`#ifdef TEST_BUILD`（main.c）、`#ifdef MT6701_DIAG`（main.c）、`#ifdef WITH_BOOTLOADER`（main.c）。

---

## 九、影响评估

### 9.1 时序影响

- 每条日志约 40~60 字符，在 115200 波特率下约 3.5~5.2ms
- 一个枚举 cycle 内最多出现 POLL + OFFER + ASSIGN + ACK = 约 4 行 × 5ms = 20ms
- 但**这些事件分散在多个 10ms 主循环 tick 中**，单个 tick 内通常只输出 1~2 行（约 5~10ms）
- 枚举阶段从机不响应电机指令、不走控制闭环，时序余量大

**结论**：对枚举行为的影响可接受。但需在测试时评估实际耗时，若某 tick 中诊断输出导致主循环超出 10ms，考虑减少单个 tick 内的打印量。

### 9.2 UART 阻塞影响

`DebugUartSendChar()` 是阻塞式（polling TX buffer empty），见 [main.c:30-36](motor_control_apm32/app/Source/main.c#L30-L36)。单条日志输出期间 CPU 被占用。在 `ENUM_DIAG` 开启时，仅影响诊断固件；正式固件不受影响。

---

## 十、代码修改思路（经讨论确定）

> 讨论结论：采用**"增添而非修改"**策略。不提取/重构 `main.c` 中已有的 `DebugUartSend*` 函数，改为在新建文件中自含一套最小化的 UART 发送原语。

### 10.1 策略选择：「增添」vs「提取」

项目已有先例：[test_shell.c:74-82](motor_control_apm32/app/Source/test_shell.c#L74-L82) 在当初面临同样的问题（需要在 `main.c` 之外发 UART 输出，但 `DebugUartSendChar` 是 `static` 的），当时的做法是**在 `test_shell.c` 中自己写了一套 `ShellUartSendChar`**，没有去动 `main.c`。本次诊断日志遵循同一工程原则。

**核心理由**：

|          | 提取（refactor）                      | 增添（add）                        |
| -------- | --------------------------------- | ------------------------------ |
| 改动范围     | `main.c` 删除 ~80 行 + 新建 2 个文件      | 仅新建 2 个文件                      |
| 对已有代码的影响 | `static`→`extern` 改变链接属性；需精确删除不遗漏 | **零**——已有文件全部只增不减              |
| 验证成本     | 需重新验证所有已有调试输出的运行时行为               | 仅需验证新增的诊断输出                    |
| 删除成本     | 两处修改（删除提取文件 + 恢复 main.c 删除的代码）    | 删文件 + 删 `can_service.c` 中调用点即可 |

在嵌入式裸机固件中，已有代码的调试输出已经手工验证过，但没有自动化回归测试来保护。因此**不碰已有代码**是风险更低的选择。

### 10.2 文件清单与职责

#### 新建文件

**`motor_control_apm32/app/Source/enum_diag.c`**（~110 行）

自包含两层：

**下层 — UART 发送原语（~50 行）：**

| 函数                                 | 对应 main.c 等价物          | 说明                     |
| ---------------------------------- | ---------------------- | ---------------------- |
| `EnumDiagSendChar(char)`           | `DebugUartSendChar`    | 轮询 USART1 TXBE，发送单字节   |
| `EnumDiagSendString(const char *)` | `DebugUartSendString`  | 遍历字符串逐字节发送             |
| `EnumDiagSendUint(uint32_t)`       | `DebugUartSendUint`    | 十进制无符号整数输出             |
| `EnumDiagSendHex32(uint32_t)`      | (新增，无等价物)              | 8 位大写 hex 无前缀，用于输出 UID |
| `EnumDiagSendLineEnd(void)`        | `DebugUartSendLineEnd` | 输出 `\r\n`              |

不包含的函数及原因：

- `SendInt`、`SendFloatX100`、`SendHex8`、`SendHexNibble` — 枚举诊断日志不需要
- `SendCommandSummary`、`SendRuntimeStatus`、`SendCalibStatus` — 复合日志，依赖 `main.c` 私有状态，与枚举诊断无关
- `DebugUartInit` — USART1 初始化由 `main.c` 在启动时完成，`enum_diag.c` 假设 USART1 已就绪

**上层 — 诊断日志函数（~60 行）：**

| 函数                                                                      | 对应打印点 | 说明                                                            |
| ----------------------------------------------------------------------- | ----- | ------------------------------------------------------------- |
| `EnumDiagPrintHeader(void)`                                             | 辅助    | 输出 `[UID=XXXXXXXX] ` 前缀                                       |
| `EnumDiagLogPoll(uint8_t myId)`                                         | ①     | 收到 POLL 时的判定日志                                                |
| `EnumDiagLogOffer(uint8_t offeredId, uint8_t myId, uint8_t action)`     | ②     | 收到 OFFER 时的判定日志（`accept` / `already assigned` / `invalid id`） |
| `EnumDiagLogAssign(uint8_t newId, uint32_t targetUid, uint8_t matched)` | ③     | 收到 ASSIGN 时的判定日志                                              |
| `EnumDiagLogAckSent(uint8_t id)`                                        | ④     | 实际发出 ACK 时的日志                                                 |
| `EnumDiagLogCanError(uint32_t tec, uint32_t rec)`                       | ⑤     | CAN Error Passive 告警（不用 `#ifdef ENUM_DIAG` 包裹，始终编译）           |
| `EnumDiagLogCanStatus(uint32_t tec, uint32_t rec, uint32_t rxCount)`    | ⑥     | 周期性 CAN 状态上报（可选）                                              |

**`motor_control_apm32/app/Include/enum_diag.h`**（~20 行）

声明上述函数。下层 UART 原语声明为 `static inline` 或在 `.c` 中保持 `static`（仅上层函数对外可见）。诊断日志函数统一用 `#ifdef ENUM_DIAG` 包裹，`EnumDiagLogCanError` 除外。

#### 修改文件

**`motor_control_apm32/app/Source/can_service.c`**：

| 修改点         | 位置                                                                                                                 | 改动内容                                                                                |
| ----------- | ------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------- |
| 新增 include  | 文件头部 `#include` 区域                                                                                                 | `#include "enum_diag.h"`                                                            |
| 打印点①        | `CanServiceHandleEnum()` POLL 分支 [can_service.c:106-113](motor_control_apm32/app/Source/can_service.c#L106-L113)   | `#ifdef ENUM_DIAG` → `EnumDiagLogPoll(myId)` → `#endif`                             |
| 打印点②        | `CanServiceHandleEnum()` OFFER 分支 [can_service.c:118-128](motor_control_apm32/app/Source/can_service.c#L118-L128)  | 三个判定分支各插入 `EnumDiagLogOffer(...)`                                                   |
| 打印点③        | `CanServiceHandleEnum()` ASSIGN 分支 [can_service.c:132-147](motor_control_apm32/app/Source/can_service.c#L132-L147) | 匹配/不匹配分支各插入 `EnumDiagLogAssign(...)`                                                |
| 打印点④        | `CanServiceServiceAck()` [can_service.c:88-93](motor_control_apm32/app/Source/can_service.c#L88-L93)               | `s_ackPending = 0U` 之后、`CanServiceSendEnumAck()` 之前插入 `EnumDiagLogAckSent(s_ackId)` |
| 打印点⑤        | `CanServiceCheckBusHealth()` [can_service.c:188-196](motor_control_apm32/app/Source/can_service.c#L188-L196)       | `CAN_Reset()` **之前**插入 `EnumDiagLogCanError(tec, rec)`（**无 `#ifdef` 保护**）           |
| 可选：CAN 状态读取 | `can_service.c` 或新增辅助函数                                                                                            | 读取 TEC/REC/RX 溢出标志供打印点⑥使用                                                           |

**`motor_control_apm32/app/CMakeLists.txt`**：

参照现有 `BUILD_TEST_FIRMWARE` / `MT6701_DIAG` 的模式（[CMakeLists.txt:46-47](motor_control_apm32/app/CMakeLists.txt#L46-L47) + [CMakeLists.txt:88-90](motor_control_apm32/app/CMakeLists.txt#L88-L90)），新增：

```cmake
option(ENUM_DIAG "Enable slave-side enumeration diagnostic logging on UART1" OFF)
```
和对应的 generator expression：
```cmake
$<$<BOOL:${ENUM_DIAG}>:ENUM_DIAG>
```

`Source/*.c` 的 `file(GLOB ...)` 自动拾取新建的 `enum_diag.c`，无需手动修改源文件列表。

#### 不改文件

| 文件 | 原因 |
|------|------|
| `main.c` | 所有已有调试输出逻辑保持原样，零改动 |
| `app_config.h` | 枚举 CAN ID、时序参数等宏均已有定义 |
| `node_id.c` / `node_id.h` | `NodeIdGetUid()` 已提供 UID 读取接口，无需改动 |
| `can_service.h` | 所有触发点在 `.c` 内部，接口不变 |
| `test_shell.c` | 无交集；其 `ShellUartSendChar` 继续独立存在 |

### 10.3 `enum_diag.c` 与 USART1 的关系

- **初始化**：`enum_diag.c` 不负责 USART1 初始化。它假设调用时 USART1 已由 `main.c:884` 的 `DebugUartInit()` 配置完毕（PA9 TX, 115200 8N1）。
- **发送方式**：阻塞轮询 `USART_FLAG_TXBE`，不使能中断。与 `DebugUartSendChar` 和 `ShellUartSendChar` 的行为一致。
- **并发**：所有调用点均在 `RunControlLoop()` 的主循环上下文中（`CanServicePoll` / `CanServiceCheckBusHealth` / `CanServiceReceiveCommand`），不存在 ISR 上下文并发问题。
- **波特率开销**：115200 下每字符约 87μs。单条诊断日志约 40-60 字符 ≈ 3.5-5.2ms。在枚举阶段的 10ms 主循环 tick 中（此时不跑电机控制闭环），一个 tick 内通常只命中 1-2 个打印点，时序可接受。

### 10.4 与 test_shell.c 的 UART 共存

`test_shell.c`（`#ifdef TEST_BUILD`）和 `enum_diag.c`（`#ifdef ENUM_DIAG`）都向 USART1 输出。两者可以共存：
- `test_shell.c` 的所有输出带 `[TS] ` 前缀，诊断日志带 `[UID=XXXXXXXX]` 前缀，上位机可通过前缀过滤
- 两者都通过相同的 `USART1 TX` 引脚输出，物理上是同一条线，上位机收到的是混合流
- 如果需要同时启用 `TEST_BUILD` 和 `ENUM_DIAG`，需注意 USART1 的 RX 中断（`USART1_IRQHandler`）由 `test_shell.c` 独占管理，不影响 `enum_diag.c` 的纯 TX 行为

### 10.5 构建命令

```bash
# 编译诊断固件（3 块从机各烧一份）
cmake -S motor_control_apm32/app \
      -B motor_control_apm32/app/build-diag \
      -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=$(pwd)/motor_control_apm32/cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DENUM_DIAG=ON
cmake --build motor_control_apm32/app/build-diag

# 正式固件（不加 -DENUM_DIAG=ON，零开销）
cmake -S motor_control_apm32/app \
      -B motor_control_apm32/app/build-gcc \
      -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=$(pwd)/motor_control_apm32/cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build motor_control_apm32/app/build-gcc
```

### 10.6 正常枚举的预期日志序列（与第四节呼应）

从机 UNSET 状态下，通电后完整枚举过程经 `enum_diag.c` 输出的理想日志：

```
[UID=A3F210B1] RX POLL, myId=0, skip (UNSET)                   ← cycle 1: 收到POLL，尚无ID
[UID=A3F210B1] RX OFFER id=1, myId=0, accept, ack pending      ← cycle 1: 收到OFFER，接受
[UID=A3F210B1] TX ACK id=1                                      ← cycle 1: 退避到期发出
[UID=A3F210B1] RX POLL, myId=1, will ACK                        ← cycle 2: 已分配，回应roll-call
[UID=A3F210B1] TX ACK id=1
[UID=A3F210B1] RX POLL, myId=1, will ACK                        ← cycle 3~: 稳态
[UID=A3F210B1] TX ACK id=1
```

对照 SubPlan 第六节的原型日志，最终格式在实现时确定前缀和分隔符的一致性。
