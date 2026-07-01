# 偶发枚举(POLL/OFFER)帧未被电机单元正确捕获 → 节点 ID 分配异常:复现、定位与可靠性评估(结合耐久测试) #32
*相关链接：*[偶发枚举(POLL/OFFER)帧未被电机单元正确捕获 → 节点 ID 分配异常:复现、定位与可靠性评估(结合耐久测试) · Issue #32 · xintlabs/puttreal-firmware-apm32](https://github.com/xintlabs/puttreal-firmware-apm32/issues/32)

## 背景与现象

在多电机(6 节点)上电枚举测试中,**偶发**出现某一电机单元未能正确捕获/响应 ESP32 主控广播的枚举帧,导致 **节点 ID 分配异常**(漏分配 / 重号 / 需要额外 ASSIGN 重定位才收敛)。该现象**不可稳定复现**,初步怀疑与 CAN 帧丢失、ACK 退避时序竞争或总线信号完整性相关。

> 报告人最初以"tick 计数未及时截止"描述。需说明:**当前固件已是主控驱动、基于 UID 的枚举**(非按 tick 计数自分配 ID,旧方案已废弃,见 `app_config.h` 注释与 `git log 1d25a70`)。本 issue 按现行机制复核,但保留所观察到的真实失效:**枚举帧偶发未被从机捕获**。

## 现行枚举机制(对齐源码)

* **主控(ESP32,**`ble_esp32/main/app/enum_manager.c` **+** `enum_logic.c`**)**:周期性 roll-call。每个 cycle 广播 `ENUM_POLL`(0x7E3)+ 对最低空闲 ID 的 `ENUM_OFFER`(0x7E0),采集窗口 `ENUM_COLLECT_WINDOW_MS=250ms`;cycle 周期填充期 400ms、6 个全部就位后 1Hz。对收集到的 `ENUM_ACK` 做仲裁:同一 ID 被多 UID 认领→保留其一、其余经 `ENUM_ASSIGN`(0x7E2)重定位;某已占用 ID 连续 `ENUM_MISS_THRESHOLD=3` 个 cycle 未被听到→释放。
* **从机(APM32,**`motor_control_apm32/app/Source/can_service.c` **+** `node_id.c`**)**:被动。读取 XOR 折叠的 32-bit 芯片 UID,上电为 `NODE_ID_UNSET`。收到 POLL/OFFER/ASSIGN 后按 `hash(UID,seq) % NODE_ID_ENUM_ACK_SPREAD_MS(48ms)` 退避再回 `ENUM_ACK(id,uid)`(0x7E1),由 `CanServicePoll()` 主循环轮询发出。
* ID 取值 1..6(`NODE_ID_MIN..MAX`);CAN 实测 50 kbps。

## 目标

1. **复现**:设计并执行可量化复现该偶发失效的测试,统计发生率(次/枚举轮次或次/小时)。
2. **定位**:确定根因落在何处——CAN RX FIFO 溢出/漏帧、ACK 退避时序竞争与仲裁丢失、总线信号完整性/终端匹配、上电时序错位、或枚举逻辑在丢帧下的边界处理。
3. **风险评估**:判断该 bug 是否**暗示潜在硬件或固件稳定性/可靠性风险**(而非仅枚举阶段的良性瞬态),并给出结论与整改建议。

## 复现与定位的可考察方向(供工程师参考,非限定)

* 从机 `CanServicePoll()` 轮询节奏 vs. 单个 cycle 内 POLL+OFFER(+ASSIGN)突发帧密度,是否存在 RX FIFO0 溢出/漏帧。
* 6 节点在 250ms 窗口内集中 ACK 时,48ms 退避是否足以避免帧碰撞/仲裁丢失,致主控漏听 ACK。
* 50kbps 下的 TEC/REC 变化、错误帧;总线终端匹配(主控 R19 固定 120Ω,从机端终端是否就位)、走线/EMI。
* 上电时序差异(6 板非同时就绪)对首轮枚举收敛的影响。
* `enum_logic_process` 在连续丢帧/丢 roll-call 下的 miss/碰撞处理边界(现有主机单测 `tests/host/ble/test_enum_logic.c` 已覆盖逻辑层"丢 roll-call 顺延 / 掉线释放重填",故纯逻辑层大概率已正确,问题更可能在 CAN/时序/硬件层)。

## 与耐久性测试结合

将复现纳入**耐久/老化测试**:长时间反复枚举 + 反复上电/复位循环压力运行,以暴露低概率偶发故障并量化发生率;同步采集主控 `[CAN] state=.. TEC=.. REC=..` 诊断、`ENUM assigned=.. offer=..` 状态(`enum_manager_status`)与从机侧日志作为证据。

> **具体测试方案(复现条件、循环次数/时长、采样与判据、夹具)由嵌入式软件/硬件工程师设计。** 本 issue 仅界定目标、范围与验收。

## 验收标准

* 给出可重复的复现步骤与实测发生率;
* 给出根因结论(硬件 / 固件 / 时序 / 信号完整性),附 CAN 诊断与枚举日志证据;
* 明确该现象是否构成稳定性/可靠性风险,并给出修复或缓解方案(必要时拆分独立修复 issue);
* 若判定为良性瞬态,亦需以耐久数据支撑该结论。

## 相关代码

* ESP32:`ble_esp32/main/app/enum_manager.c`、`enum_logic.c`、`enum_logic.h`
* APM32:`motor_control_apm32/app/Source/can_service.c`、`node_id.c`、`app/Include/app_config.h`
* 现有主机单测:`tests/host/ble/test_enum_logic.c`