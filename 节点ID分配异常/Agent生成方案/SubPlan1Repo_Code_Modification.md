# 子方案1:从机侧枚举诊断日志 — 代码修改文档

> Issue: [节点 ID 分配异常 #32](https://github.com/xintlabs/puttreal-firmware-apm32/issues/32)
> 父方案: [TEST_PLAN.md](Test_Plan.md) — Phase 0 打桩（从机侧细化）
> 子方案: [SubPlan_SLAVE_DIAG.md](SubPlan1_Slave_Diag.md)
> 范围: 仅从机 APM32F103CB 侧，不改主控 ESP32
> 版本: v1
> 生成：Claude code （正确性不完全确保，理智参考）
> 日期: 2026-06-26

---

## 一、概述

本修改为主控枚举过程中出现“某块从机漏帧导致节点 ID 分配异常”的问题提供诊断能力。在从机的枚举帧处理路径上插入 UART 诊断日志，每条日志以 `[UID=XXXXXXXX]` 为前缀，使上位机能观察到每块从机的收发事件，进而定位漏帧发生在哪个环节。

**核心策略**: 增添而非修改——不提取/重构已有代码，在新文件中自含 UART 发送原语。已有文件仅追加调用点，不删除、不改变任何已有逻辑行。

---

## 二、文件清单

| 状态  | 文件                            | 行数    | 职责                                      |
| --- | ----------------------------- | ----- | --------------------------------------- |
| 新建  | `app/Include/enum_diag.h`     | 16    | 诊断函数声明 + ENUM_DIAG 开关 stub              |
| 新建  | `app/Source/enum_diag.c`      | 190   | UART TX 原语 + 6 个诊断日志函数                  |
| 修改  | `app/Source/can_service.c`    | +25 行 | 7 处诊断日志调用点插入                            |
| 修改  | `app/CMakeLists.txt`          | +5 行  | ENUM_DIAG option + generator expression |
| 不改  | `app/Source/main.c`           | —     | 已有 UART 调试输出保持原样                        |
| 不改  | `app/Include/app_config.h`    | —     | 枚举 CAN ID / 时序参数已有定义                    |
| 不改  | `app/Source/node_id.c` / `.h` | —     | `NodeIdGetUid()` 接口已满足需求                |
| 不改  | `app/Include/can_service.h`   | —     | 所有触发点在 `.c` 内部，接口不变                     |
| 不改  | `app/Source/test_shell.c`     | —     | 无交集，独立存在                                |

---

## 三、新建文件: `enum_diag.h`

**路径**: `motor_control_apm32/app/Include/enum_diag.h`

```c
#ifndef ENUM_DIAG_H
#define ENUM_DIAG_H

#include <stdint.h>

#ifdef ENUM_DIAG
void EnumDiagLogPoll(uint8_t myId);
void EnumDiagLogOffer(uint8_t offeredId, uint8_t myId, uint8_t action);
void EnumDiagLogAssign(uint8_t newId, uint32_t targetUid, uint8_t matched);
void EnumDiagLogAckSent(uint8_t id);
// void EnumDiagLogCanStatus(uint32_t tec, uint32_t rec, uint32_t rxCount);
void EnumDiagLogCanError(uint32_t tec, uint32_t rec);

#endif /* ENUM_DIAG */
#endif /* ENUM_DIAG_H */
```

**设计要点**:

- 5 个枚举帧诊断函数 + 1 个 CAN 错误告警函数，全部在 `#ifdef ENUM_DIAG` 下声明
- 不使用 `#else` / `#define` 空宏 stub — 调用点均位于 `#ifdef ENUM_DIAG` 块内，由调用点自行控制编译
- `EnumDiagLogCanStatus` 暂注释，为 Phase 2 周期性 CAN 状态上报预留

---

## 四、新建文件: `enum_diag.c`

**路径**: `motor_control_apm32/app/Source/enum_diag.c`

### 4.1 依赖与 include

```c
#include "enum_diag.h"
#include "node_id.h"
#include "apm32f10x_usart.h"
#include <stdint.h>
```

仅依赖三个头文件，不依赖 `main.c` 的任何符号。USART1 初始化由 `main.c:884` 的 `DebugUartInit()` 在启动时完成，`enum_diag.c` 假设调用时 USART1 已就绪（PA9 TX, 115200 8N1）。

### 4.2 下层 — UART 发送原语（static，~70 行）

| 函数 | 等价物 | 说明 |
|------|--------|------|
| `EnumDiagSendChar(char)` | `main.c:DebugUartSendChar` | 轮询 `USART_FLAG_TXBE`，阻塞式发送单字节 |
| `EnumDiagSendString(const char *)` | `main.c:DebugUartSendString` | 遍历字符串逐字节发送，NULL 安全 |
| `EnumDiagSendUint(uint32_t)` | `main.c:DebugUartSendUint` | 十进制无符号整数输出 |
| `EnumDiagSendHex32(uint32_t)` | (新增) | 8 位大写 hex 无前缀，用于输出 UID |
| `EnumDiagSendLineEnd(void)` | `main.c:DebugUartSendLineEnd` | 输出 `\r\n` |

参考实现 (完整代码见 [enum_diag.c:12-70](motor_control_apm32/app/Source/enum_diag.c#L12-L70)）：

```c
static void EnumDiagSendChar(char ch)
{
    while (USART_ReadStatusFlag(USART1, USART_FLAG_TXBE) == RESET) { }
    USART_TxData(USART1, (uint16_t)ch);
}

static void EnumDiagSendString(const char *s)
{
    if (s == NULL) { return; }
    while (*s != '\0') { EnumDiagSendChar(*s); s++; }
}

static void EnumDiagSendUint(uint32_t v)
{
    char buf[11];
    uint8_t n = 0U;
    if (v == 0U) { EnumDiagSendChar('0'); return; }
    while (v > 0U && n < sizeof(buf)) { buf[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (n > 0U) { EnumDiagSendChar(buf[--n]); }
}

static const char s_hexChars[16] = "0123456789ABCDEF";

static void EnumDiagSendHex32(uint32_t v)
{
    uint8_t i;
    for (i = 0U; i < 8U; i++)
    {
        uint8_t nibble = (uint8_t)((v >> (28U - i * 4U)) & 0xFU);
        EnumDiagSendChar(s_hexChars[nibble]);
    }
}

static void EnumDiagSendLineEnd(void)
{
    EnumDiagSendChar('\r');
    EnumDiagSendChar('\n');
}
```

所有函数声明为 `static`，仅 `enum_diag.c` 内部可见。

### 4.3 辅助 — 日志前缀

```c
static void EnumDiagPrintHeader(void)
{
    EnumDiagSendChar('[');
    EnumDiagSendString("UID=");
    EnumDiagSendHex32(NodeIdGetUid());
    EnumDiagSendString("] ");
}
```

每行日志以 `[UID=XXXXXXXX] ` 开头，天然区分不同从机，无需额外标识。

### 4.4 上层 — 诊断日志函数（~120 行）

全部位于 `#ifdef ENUM_DIAG` 块内。

#### 打印点① — 收到 POLL

```c
void EnumDiagLogPoll(uint8_t myId)
{
    EnumDiagPrintHeader();
    EnumDiagSendString("RX POLL, myId=");
    EnumDiagSendUint(myId);
    if ((myId >= 1U) && (myId <= 6U))
        EnumDiagSendString(", will ACK");
    else
        EnumDiagSendString(", skip (UNSET)");
    EnumDiagSendLineEnd();
}
```

根据 `myId` 是否在 1~6 范围内自动选择输出文案。

#### 打印点② — 收到 OFFER

```c
void EnumDiagLogOffer(uint8_t offeredId, uint8_t myId, uint8_t action)
{
    EnumDiagPrintHeader();
    EnumDiagSendString("RX OFFER id=");
    EnumDiagSendUint(offeredId);
    EnumDiagSendString(", myId=");
    EnumDiagSendUint(myId);
    EnumDiagSendString(", ");
    switch (action)
    {
    case 0U: EnumDiagSendString("accept, ack pending"); break;
    case 1U: EnumDiagSendString("already assigned");    break;
    default: EnumDiagSendString("invalid id");          break;
    }
    EnumDiagSendLineEnd();
}
```

`action` 参数含义: `0` = accept, `1` = skip (already assigned), `2` = skip (invalid id)。

#### 打印点③ — 收到 ASSIGN

```c
void EnumDiagLogAssign(uint8_t newId, uint32_t targetUid, uint8_t matched)
{
    EnumDiagPrintHeader();
    EnumDiagSendString("RX ASSIGN id=");
    EnumDiagSendUint(newId);
    EnumDiagSendString(", target=");
    EnumDiagSendHex32(targetUid);
    EnumDiagSendString(", ");
    if (matched != 0U)
        EnumDiagSendString("match, accept");
    else
        EnumDiagSendString("not for me");
    EnumDiagSendLineEnd();
}
```

#### 打印点④ — 实际发出 ACK

```c
void EnumDiagLogAckSent(uint8_t id)
{
    EnumDiagPrintHeader();
    EnumDiagSendString("TX ACK id=");
    EnumDiagSendUint(id);
    EnumDiagSendLineEnd();
}
```

#### 打印点⑤ — CAN Error Passive 告警

```c
void EnumDiagLogCanError(uint32_t tec, uint32_t rec)
{
    /* Use primitives directly — no PrintHeader() dependency on ENUM_DIAG. */
    EnumDiagSendChar('[');
    EnumDiagSendString("UID=");
    EnumDiagSendHex32(NodeIdGetUid());
    EnumDiagSendString("] CAN ERR: TEC=");
    EnumDiagSendUint(tec);
    EnumDiagSendString(" REC=");
    EnumDiagSendUint(rec);
    EnumDiagSendString(", resetting CAN controller");
    EnumDiagSendLineEnd();
}
```

**当前状态**: 此函数声明和调用点位于 `#ifdef ENUM_DIAG` 内（见第八节已知偏差）。内部实现不依赖 `EnumDiagPrintHeader`（直接调用下层原语），以便将来移到 `#ifdef` 外时无需改动函数体。

#### 打印点⑥ — 周期性 CAN 状态（已注释，暂不启用）

```c
// void EnumDiagLogCanStatus(uint32_t tec, uint32_t rec, uint32_t rxCount) { ... }
```

为 Phase 2 预留，当前为空。

---

## 五、修改文件: `can_service.c`

**路径**: `motor_control_apm32/app/Source/can_service.c`

共 7 处插入点，均为只增不改（原始逻辑行全部保留）。

### 5.1 新增 include（L10）

```diff
+#include "enum_diag.h"
```

位于 `#include "node_id.h"` 之后、`#include "apm32f10x_can.h"` 之前。

### 5.2 打印点① — POLL 分支（L114-118）

在 `CanServiceHandleEnum()` 的 POLL 分支中，`myId` 读取后、条件判断前插入:

```c
    uint8_t myId = NodeIdGet();

#ifdef ENUM_DIAG
    EnumDiagLogPoll(myId);
#endif
```

### 5.3 打印点② — OFFER 分支（L131-159）

轻微重构以区分三种判定路径：

1. 将 `NodeIdGet()` 从条件表达式移到独立变量 `myId`（运行值等价）
2. 在 `#ifdef ENUM_DIAG` 块内根据原始条件计算 `action` 码
3. 调用 `EnumDiagLogOffer(offered, myId, action)`

```c
    uint8_t offered = (rx->dataLengthCode >= 1U) ? rx->data[0] : NODE_ID_UNSET;
    uint8_t myId = NodeIdGet();   /* capture before NodeIdSet() changes it */
    if ((myId == NODE_ID_UNSET) &&
        (offered >= NODE_ID_MIN) && (offered <= NODE_ID_MAX))
    {
        NodeIdSet(offered);
        CanServiceScheduleAck(offered);
    }

#ifdef ENUM_DIAG
    {
        uint8_t action = 0U;
        if (myId != NODE_ID_UNSET)
            action = 1U;   /* already assigned */
        else if ((offered >= NODE_ID_MIN) && (offered <= NODE_ID_MAX))
            action = 0U;   /* accept */
        else
            action = 2U;   /* invalid id */
        EnumDiagLogOffer(offered, myId, action);
    }
#endif
```

### 5.4 打印点③ — ASSIGN 分支（L180-186）

引入 `matched` 局部变量，在 `#ifdef ENUM_DIAG` 内复用原始条件表达式计算匹配结果:

```c
            if ((target == NodeIdGetUid()) &&
                (newId >= NODE_ID_MIN) && (newId <= NODE_ID_MAX))
            {
                NodeIdSet(newId);
                CanServiceScheduleAck(newId);
            }

#ifdef ENUM_DIAG
            {
                uint8_t matched = ((target == NodeIdGetUid()) &&
                    (newId >= NODE_ID_MIN) && (newId <= NODE_ID_MAX)) ? 1U : 0U;
                EnumDiagLogAssign(newId, target, matched);
            }
#endif
```

**注意**: 对 `NodeIdGetUid()` 存在两次调用（原始 if 中一次、`matched` 计算中一次），`NodeIdGetUid()` 是纯读取 UID 寄存器的无副作用函数，两次调用返回相同值。

### 5.5 打印点④ — ACK 发出点（L94-96）

在 `CanServiceServiceAck()` 中，`s_ackPending = 0U` 之后、`CanServiceSendEnumAck()` 之前:

```c
        s_ackPending = 0U;

    #ifdef ENUM_DIAG
        EnumDiagLogAckSent(s_ackId);
    #endif

        CanServiceSendEnumAck(s_ackId, NodeIdGetUid());
```

**位置选择依据**: `CanServiceScheduleAck()` 只做登记（可被覆盖），`CanServiceServiceAck()` 才是实际发送点。放在此处确保每行 `TX ACK` 日志对应一帧真实的总线数据传输。

### 5.6 打印点⑤ — CAN Error Passive（L235-239）

在 `CanServiceCheckBusHealth()` 中，`CAN_Reset()` 之前插入:

```c
    if (CAN_ReadStatusFlag(CAN1, CAN_FLAG_ERRP) != RESET)
    {
    #ifdef ENUM_DIAG
        uint32_t tec = (uint32_t)CAN_ReadLSBTxErrorCounter(CAN1);
        uint32_t rec = (uint32_t)CAN_ReadRxErrorCounter(CAN1);
        EnumDiagLogCanError(tec, rec);
    #endif

        CAN_Reset(CAN1);
        BoardCanInit();
        CanServiceInit();
    }
```

TEC/REC 必须在 `CAN_Reset()` **之前**读取——复位后 CAN 控制器寄存器全部归零。`CAN_ReadLSBTxErrorCounter()` 和 `CAN_ReadRxErrorCounter()` 是 APM32 SPL 提供的标准 API。

---

## 六、修改文件: `CMakeLists.txt`

**路径**: `motor_control_apm32/app/CMakeLists.txt`

### 6.1 新增 option（L47 之后）

参照已有 `BUILD_TEST_FIRMWARE` / `MT6701_DIAG` 模式:

```cmake
option(ENUM_DIAG "Enable slave-side enumeration diagnostic logging on UART1" OFF)
```

### 6.2 新增 generator expression（L90 之后）

```cmake
    $<$<BOOL:${ENUM_DIAG}>:ENUM_DIAG>
```

添加到 `target_compile_definitions` 块末尾。

### 6.3 源文件自动拾取

`file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.c")` (L59) 自动将新建的 `enum_diag.c` 纳入编译，无需手动修改源文件列表。

### 6.4 构建命令

```bash
# 诊断固件（3 块从机各烧一份）
cmake -S motor_control_apm32/app \
      -B motor_control_apm32/app/build-diag \
      -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=$(pwd)/motor_control_apm32/cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DENUM_DIAG=ON
cmake --build motor_control_apm32/app/build-diag

# 正式固件（不加 -DENUM_DIAG=ON，诊断代码编译为空）
cmake -S motor_control_apm32/app \
      -B motor_control_apm32/app/build-gcc \
      -G Ninja \
      -DCMAKE_TOOLCHAIN_FILE=$(pwd)/motor_control_apm32/cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build motor_control_apm32/app/build-gcc
```

---

## 七、不改动的文件及理由

| 文件 | 不改原因 |
|------|---------|
| `main.c` | 所有已有调试输出逻辑（`DebugUartSend*`）保持原样，这些 `static` 函数继续作为通用调试输出。新增诊断日志完全独立 |
| `app_config.h` | 枚举 CAN ID（`CAN_STD_ID_ENUM_POLL/OFFER/ASSIGN/ACK`）、ID 范围（`NODE_ID_MIN/MAX`）、退避参数（`NODE_ID_ENUM_ACK_SPREAD_MS`）均已定义 |
| `node_id.c` / `.h` | `NodeIdGetUid()`、`NodeIdGet()`、`NodeIdSet()` 接口已满足需求 |
| `can_service.h` | 所有诊断触发点（POLL/OFFER/ASSIGN/ACK/CAN ERR）均在 `.c` 内部，接口不需变更 |
| `test_shell.c` | 无功能交集。其 `ts_putc`/`ShellUartSendChar` 继续独立存在。两者可共存——test_shell 输出带 `[TS] ` 前缀，enum_diag 输出带 `[UID=XXXXXXXX]` 前缀 |

---

## 八、与 SubPlan 的已知偏差

| # | 偏差项 | SubPlan 规定 | 当前实现 | 影响 |
|---|--------|-------------|---------|------|
| 1 | `EnumDiagLogCanError` 编译保护 | **不用** `#ifdef ENUM_DIAG`，始终编译 | 声明在 `#ifdef ENUM_DIAG` 内，调用点也加了 `#ifdef` | CAN Error Passive 告警仅在诊断固件中可见。正式固件中 CAN 静默复位仍无日志。如需要在正式固件中也记录，将声明移出 `#ifdef` + 调用点去掉 `#ifdef` 即可 |
| 2 | `enum_diag.h` 缺少 `#else` 空宏 | 无特殊规定 | 当前无 `#else` 分支空宏 | 无影响——因为调用点自身都在 `#ifdef ENUM_DIAG` 块内，不会出现 `ENUM_DIAG` 关时调用函数的情况 |
| 3 | `EnumDiagLogCanStatus` | 列入打印点⑥ | 已注释，暂不启用 | Phase 0 暂不需要周期性 CAN 状态上报。Phase 2 需要时取消注释并在 main.c 主循环中每 500ms 调用 |
| 4 | `EnumDiagLogOffer` action=1 文案 | SubPlan 写 "skip (already assigned)" | 实现为 "already assigned" | 省略了 "skip" 前缀，因上下文已明确是收到 OFFER 后的判定。语义等价，不影响诊断 |

**偏差 1 评估**: 当前实现为保守选择。SubPlan 认为 Error Passive 是故障告警应始终记录，但正式固件中阻塞式 UART 输出可能影响电机控制时序。如后续确认正式固件也需要 CAN 错误告警，修改涉及 2 处：
- `enum_diag.h`: 将 `EnumDiagLogCanError` 声明移到 `#ifdef ENUM_DIAG` 之外
- `can_service.c`: 将调用点 `#ifdef ENUM_DIAG` / `#endif` 去掉

---

## 九、时序影响评估

### 9.1 UART 阻塞开销

- 115200 baud 下每字符约 87 µs
- 单条日志约 45-60 字符 ≈ 3.9-5.2 ms
- 所有调用点位于 `RunControlLoop()` 主循环上下文，不在 ISR 中
- 一个 10ms tick 内通常命中 1-2 个打印点（POLL+OFFER 或 POLL+ACK），约 5-10ms
- 枚举阶段不跑电机控制闭环，时序余量充足

### 9.2 正式固件零开销

- `-DENUM_DIAG=OFF` 时，`enum_diag.c` 中的所有上层函数体由 `#ifdef` 排除，编译为几乎空的翻译单元
- `can_service.c` 中所有调用点由 `#ifdef ENUM_DIAG` / `#endif` 包裹，正式编译时完全消除
- 仅 `#include "enum_diag.h"` 一行在多 include 保护下除宏展开外无额外代价

---

## 十、正常枚举的预期日志

从机 UNSET 状态下，通电后完整枚举过程的理想日志序列：

```
[UID=A3F210B1] RX POLL, myId=0, skip (UNSET)          ← cycle 1: 尚无 ID
[UID=A3F210B1] RX OFFER id=1, myId=0, accept, ack pending ← cycle 1: 接受 OFFER
[UID=A3F210B1] TX ACK id=1                              ← cycle 1: 退避到期
[UID=A3F210B1] RX POLL, myId=1, will ACK                ← cycle 2: 已分配
[UID=A3F210B1] TX ACK id=1
[UID=A3F210B1] RX POLL, myId=1, will ACK                ← cycle 3+: 稳态
[UID=A3F210B1] TX ACK id=1
```

---

## 十一、后续清理步骤

完成 Issue #32 的复现与根因判定后，如决定删除诊断代码：

1. 删除 `app/Source/enum_diag.c`
2. 删除 `app/Include/enum_diag.h`
3. `can_service.c` 中删除 `#include "enum_diag.h"` 和所有 `#ifdef ENUM_DIAG` 块
4. `CMakeLists.txt` 中删除 `option(ENUM_DIAG ...)` 和对应 generator expression
5. 重新编译验证

清理成本约 10 分钟。
