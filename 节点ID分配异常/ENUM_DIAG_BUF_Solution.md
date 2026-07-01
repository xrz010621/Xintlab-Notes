# ENUM_DIAG 非阻塞方案：Ring Buffer 诊断日志

## 1. 问题诊断

### 1.1 实验矩阵

| # | main.c | ENUM_DIAG | TEST_BUILD | CAN 枚举 |
|---|--------|-----------|------------|----------|
| 1 | main1.c（修改版） | ✅ ON | ✅ ON | ❌ FAIL |
| 2 | main.c（原始） | ✅ ON | ❌ OFF | ❌ FAIL |
| 3 | main.c（原始） | ❌ OFF | ✅ ON | ✅ PASS |
| 4 | main.c（原始） | ✅ ON | ✅ ON | ✅ PASS |

### 1.2 根因分析

**直接原因：** `enum_diag.c` 中的 `EnumDiagSendChar()` 使用阻塞式 spin-wait（等待 USART TXBE 标志），且该函数在 CAN 接收路径内部被调用。

调用链：
```
CanServicePoll()
  → CanServiceHandleEnum()
    → EnumDiagLogOffer/Poll/Assign()       ← 在 ENUM_DIAG 下激活
      → EnumDiagPrintHeader()
        → EnumDiagSendChar()               ← 阻塞 spin-wait ~87µs/字符
          → 每行日志约 50-70 字符 → ~5-6ms 阻塞
```

**为什么不同配置表现不同：**

- **配置 4（✅）：** `main.c` 中 `ProcessCanCommand()` / `RunControlLoop()` 有活跃的 `DebugUartSend*()` 调用，频繁执行 UART TX 操作。这维持了 USART1 外设的"热"状态（寄存器被频繁访问），使得 ENUM_DIAG 的间歇性 UART 输出不会触发潜在的 USART 状态机异常。

- **配置 1（❌）：** `main1.c` 中所有 `#ifndef ENUM_DIAG` 保护的 debug 输出被注释为空。USART1 在 `DebugUartInit()` 后长期静默直到 CAN 枚举帧到达，突然大量 UART TX 操作可能触发 USART 状态异常（可能与 APM32 的 USART/GPIO 硬件行为有关）。

- **配置 2（❌）：** `TEST_BUILD` 未定义时，`test_shell.c` 不编译，`USART1_IRQHandler` 回退到启动文件的弱默认（`B .` 死循环），且 USART1 RX 中断未使能。ESP32 发送到 APM32 USART1 RX 的数据无人消费，导致 RX 端产生 ORE（溢出错误），USART 外设进入异常状态后影响 TX 行为。

**根本矛盾：** `EnumDiagSendChar()` 的阻塞特征使其不适合在 CAN 帧接收路径中调用。无论哪个触发条件，阻塞 UART TX 在中断/实时路径中都是设计缺陷。

### 1.3 现有 `enum_diag_buf.c` 为何无效

```
enum_diag.c                         enum_diag_buf.c
┌──────────────────────────┐       ┌──────────────────────────┐
│ static EnumDiagSendChar  │       │ static EnumDiagSendChar  │
│   → blocking (TXBE wait) │       │   → ring buffer (no wait)│
│                          │       │                          │
│ EnumDiagLogOffer()       │       │ EnumDiagFlush()          │
│ EnumDiagLogPoll()        │       │   → drain buffer to UART │
│   → call EnumDiagSendChar│       └──────────────────────────┘
│     (file-local blocking)│               ↑
└──────────────────────────┘        这两个 static 函数在
        ↑                         不同编译单元，互不可见！
   日志函数永远调用阻塞版本
```

`enum_diag.c` 的日志函数调用的是**本文件内**的 `static EnumDiagSendChar()`（阻塞版）。
`enum_diag_buf.c` 的 ring-buffer 版 `EnumDiagSendChar()` 是**另一个编译单元**的 static 函数，日志函数永远用不到它。
`EnumDiagFlush()` drain 的是一个**日志函数永远不会写入**的缓冲区。

**结论：即使定义 `ENUM_DIAG_BUF`，诊断日志仍然走阻塞路径。整个 ring buffer 机制处于"断连"状态。**

---

## 2. 解决方案

### 2.1 设计思路

将 ring buffer 逻辑**合并到 `enum_diag.c` 内部**，通过 `#ifdef ENUM_DIAG_BUF` 控制 `EnumDiagSendChar()` 的行为：

```
ENUM_DIAG_BUF 未定义                     ENUM_DIAG_BUF 已定义
─────────────────────                   ─────────────────────
EnumDiagSendChar()                      EnumDiagSendChar()
  → spin-wait TXBE                        → 写入 ring buffer (无等待)
  → USART_TxData()                        → 仅 buffer 满时静默丢弃
                                       EnumDiagFlush()
  (保持原始行为，向后兼容)                  → 在 main loop 空闲时 drain
                                         (非阻塞，CAN 路径无延迟)
```

**优点：**
- 日志函数 (`EnumDiagLogOffer` 等) 无需修改——它们调用 `EnumDiagSendChar()`，后者的行为由宏开关决定
- 向后兼容：不定义 `ENUM_DIAG_BUF` 时行为与原始代码完全一致
- 删除冗余的 `enum_diag_buf.c` / `enum_diag_buf.h`，架构更清晰

### 2.2 Ring Buffer 尺寸估算

| 场景 | 每周期字符数 | Ring Buffer 需求 |
|------|------------|-----------------|
| 单次 POLL 日志 | ~54 chars | — |
| 单次 OFFER 日志 | ~63 chars | — |
| 单次 ASSIGN 日志 | ~58 chars | — |
| 单次 ACK 日志 | ~40 chars | — |
| 单次 FIFO Overrun 日志 | ~48 chars | — |
| 典型单次枚举 (POLL+OFFER+ACK) | ~157 chars | — |
| 3 从机同时枚举 | ~500 chars/周期 | — |
| 4096 bytes | — | **可容纳 ~8 个完整枚举周期** |

`DIAG_BUF_SIZE = 4096` 在 10ms 控制循环下绰绰有余。

### 2.3 Flush 时机

```
while (1) {
    RunControlLoop();                          // CAN 帧处理，无 UART 阻塞
    SystemTickDelayMs(CONTROL_LOOP_MS);        // 10ms 空闲窗口
    #ifdef ENUM_DIAG_BUF
    EnumDiagFlush();                           // 安全 drain，不干扰 CAN
    #endif
}
```

`EnumDiagFlush()` 在 `RunControlLoop()` 完成后调用，此时 CAN 帧已处理完毕。即使 flush 阻塞 UART（它内部也 spin-wait TXBE），也不会延迟 CAN 帧接收。ESP32 枚举周期为 650ms，而主循环为 10ms，有充足的时间在下一个 CAN 帧到达前 flush 完成。

---

## 3. 实施步骤

### 3.1 修改 `enum_diag.c`

**文件路径：** `motor_control_apm32/app/Source/enum_diag.c`

将 12-19 行的 `EnumDiagSendChar()` 替换为 `ENUM_DIAG_BUF` 双实现，并在文件末尾（`#endif /* ENUM_DIAG */` 之前）添加 `EnumDiagFlush()`：

<details>
<summary>点击展开完整修改内容</summary>

**修改点 1：** 替换第 12-19 行的 `EnumDiagSendChar()`

```c
/* ===================================================================
 *  Lower layer — UART TX primitives (static, internal to this file)
 *
 *  When ENUM_DIAG_BUF is defined, characters are written into a RAM
 *  ring buffer instead of spin-waiting on USART TXE.  The buffer is
 *  drained by EnumDiagFlush() during the main-loop idle window, so
 *  CAN frame processing is never delayed by UART.
 * =================================================================== */

#ifdef ENUM_DIAG_BUF

#define DIAG_BUF_SIZE   4096U

static char     s_diagBuf[DIAG_BUF_SIZE];
static uint16_t s_diagWr = 0U;
static uint16_t s_diagRd = 0U;

static void EnumDiagSendChar(char ch)
{
    uint16_t next = (uint16_t)((s_diagWr + 1U) % DIAG_BUF_SIZE);
    if (next != s_diagRd)
    {
        s_diagBuf[s_diagWr] = ch;
        s_diagWr = next;
    }
    /* silently drop when full (enumeration traffic is low enough that
     * 4 KB easily holds several cycles' worth of diagnostic lines) */
}

#else  /* !ENUM_DIAG_BUF — original blocking behaviour */

static void EnumDiagSendChar(char ch)
{
    while (USART_ReadStatusFlag(USART1, USART_FLAG_TXBE) == RESET)
    {
    }
    USART_TxData(USART1, (uint16_t)ch);
}

#endif /* ENUM_DIAG_BUF */
```

**修改点 2：** 在 `#endif /* ENUM_DIAG */` 之前，添加 `EnumDiagFlush()`（仅 `ENUM_DIAG_BUF` 时）

```c
#ifdef ENUM_DIAG_BUF

void EnumDiagFlush(void)
{
    while (s_diagRd != s_diagWr)
    {
        while (USART_ReadStatusFlag(USART1, USART_FLAG_TXBE) == RESET)
        {
        }
        USART_TxData(USART1, (uint16_t)s_diagBuf[s_diagRd]);
        s_diagRd = (uint16_t)((s_diagRd + 1U) % DIAG_BUF_SIZE);
    }
}

#endif /* ENUM_DIAG_BUF */
```

**修改点 3（可选）：** `EnumDiagLogCanError()` 函数（第 178-191 行）在 `ENUM_DIAG_BUF` 模式下也需要 flush 缓冲区后才能输出。CAN 错误通常意味着 CAN 控制器即将 reset，需要在 reset 前确保日志已输出：

```c
void EnumDiagLogCanError(uint32_t tec, uint32_t rec)
{
    /* Use primitives directly */
    EnumDiagSendChar('[');
    EnumDiagSendString("UID=");
    EnumDiagSendHex32(NodeIdGetUid());
    EnumDiagSendString("] CAN ERR: TEC=");
    EnumDiagSendUint(tec);
    EnumDiagSendString(" REC=");
    EnumDiagSendUint(rec);
    EnumDiagSendString(", resetting CAN controller");
    EnumDiagSendLineEnd();

#ifdef ENUM_DIAG_BUF
    /* CAN controller is about to be reset — flush the buffer now so
     * this critical error message is not lost. */
    EnumDiagFlush();
#endif
}
```

</details>

### 3.2 修改 `enum_diag.h`

**文件路径：** `motor_control_apm32/app/Include/enum_diag.h`

在 `ENUM_DIAG` 块内，添加 `EnumDiagFlush()` 声明：

```c
#ifndef ENUM_DIAG_H
#define ENUM_DIAG_H

#include <stdint.h>

#ifdef ENUM_DIAG
void EnumDiagLogPoll(uint8_t myId);
void EnumDiagLogOffer(uint8_t offeredId, uint8_t myId, uint8_t action);
void EnumDiagLogAssign(uint8_t newId, uint32_t targetUid, uint8_t matched);
void EnumDiagLogAckSent(uint8_t id);
void EnumDiagLogCanError(uint32_t tec, uint32_t rec);
void EnumDiagLogFifoOverrun(void);

#ifdef ENUM_DIAG_BUF
void EnumDiagFlush(void);
#endif

#endif /* ENUM_DIAG */

#endif /* ENUM_DIAG_H */
```

### 3.3 修改 `main.c`

**文件路径：** `motor_control_apm32/app/Source/main.c`

**位置 1：** 文件头部 include 区域，添加：

```c
#ifdef ENUM_DIAG_BUF
#include "enum_diag.h"
#endif
```

（建议将此 include 放在 `#include "can_service.h"` 附近，如第 5 行之后。）

**位置 2：** 主循环（约第 943-947 行），在 `SystemTickDelayMs()` 之后添加：

```c
    while (1)
    {
        RunControlLoop();
        SystemTickDelayMs(CONTROL_LOOP_MS);
#ifdef ENUM_DIAG_BUF
        /* Drain the diagnostic ring-buffer in the idle gap between ticks.
         * CAN is quiet here (ESP32 cycle = 650 ms, tick = 10 ms), so
         * blocking on UART TXE cannot delay frame processing. */
        EnumDiagFlush();
#endif
    }
```

### 3.4 清理冗余文件

以下文件的功能已完全合并到 `enum_diag.c` 和 `enum_diag.h` 中，可以删除：

- `motor_control_apm32/app/Source/enum_diag_buf.c` — 删除
- `motor_control_apm32/app/Include/enum_diag_buf.h` — 删除

同时从 Keil 工程 `DC_Motor_APM32.uvprojx` 中移除 `enum_diag_buf.c` 的文件引用。

### 3.5 Keil 编译宏配置

在 Keil 工程 `Options → C/C++ → Define` 中：

| 宏 | 作用 | 推荐值 |
|----|------|-------|
| `APM32F103CB` | 芯片型号 | ✅ 始终定义 |
| `APM32F103xB` | 芯片系列 | ✅ 始终定义 |
| `TEST_BUILD` | 使能 test_shell (USART1 中断驱动 RX) | ✅ 保持定义 |
| `ENUM_DIAG` | 使能 CAN 枚举诊断日志 | 调试时定义 |
| `ENUM_DIAG_BUF` | 使用 ring buffer 非阻塞输出（依赖 `ENUM_DIAG`） | 调试时与 ENUM_DIAG 同时定义 |

**推荐调试组合：**
```
APM32F103CB,APM32F103xB,TEST_BUILD,ENUM_DIAG,ENUM_DIAG_BUF
```

---

## 4. 验证计划

### 4.1 编译验证

- [ ] `ENUM_DIAG + ENUM_DIAG_BUF` → 编译通过，无 warning
- [ ] `ENUM_DIAG` without `ENUM_DIAG_BUF` → 编译通过（阻塞模式向后兼容）
- [ ] without `ENUM_DIAG` → 编译通过（完全无诊断代码，与原始固件一致）

### 4.2 功能验证

- [ ] 单从机枚举：ESP32 侧日志确认首个枚举周期内收到 ACK
- [ ] `enum_diag` 日志完整输出（通过 UART 观察，无丢行、无截断）
- [ ] 与原始固件（无 `ENUM_DIAG`）枚举时序对比无明显差异
- [ ] 50 轮单从机枚举 100% 成功率

### 4.3 回归验证

- [ ] 多从机（3+6）枚举测试 → Issue #32 诊断

---

## 5. 变更总结

| 项目 | 现状 | 修改后 |
|------|------|-------|
| `EnumDiagSendChar()` 位置 | `enum_diag.c` 阻塞版 + `enum_diag_buf.c` ring-buffer 版（互不连通） | 统一在 `enum_diag.c`，`#ifdef ENUM_DIAG_BUF` 控制实现 |
| 日志函数调用路径 | 固定走阻塞版（无论是否定义 `ENUM_DIAG_BUF`） | 跟随宏自动选择阻塞/非阻塞 |
| `EnumDiagFlush()` 是否生效 | ❌ flush 空缓冲区（日志函数写入的是 `enum_diag.c` 的阻塞版） | ✅ flush 日志函数写入的 ring buffer |
| 对 CAN 接收路径影响 | 每行日志阻塞 ~5-6ms | 0ms（字符仅写入 RAM buffer） |
| `enum_diag_buf.c/h` | 存在但功能断连 | 删除（功能已合并到 `enum_diag.c/h`） |
| 文件数量 | 4 个文件（2×.c + 2×.h） | 2 个文件（1×.c + 1×.h） |
| 向后兼容 | 阻塞版始终生效 | 不定义 `ENUM_DIAG_BUF` 时行为与原始完全一致 |
