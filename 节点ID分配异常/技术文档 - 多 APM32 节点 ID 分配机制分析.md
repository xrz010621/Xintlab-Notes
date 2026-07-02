>**生成**：Claude code （正确性不完全确保，理智参考）
>**日期**：2026-06-30

### 1. 拓扑与角色

```
  ESP32-C3 (唯一主控)
       │ CAN 50 kbps (菊花链)
   ┌───┼───┬───┬───┬───┐
  APM32 APM32 APM32 ... APM32  (1~6 个从机)
   #1    #2    #3        #6
```

- 多个节点（1–6 号）挂在同一条 CAN 总线上，组成**菊花链**
- **ESP32 主控**：唯一主控，拥有节点 ID 分配权，所有从机被动接受。
- **APM32 从机**：上电即处于 `NODE_ID_UNSET(0)` 状态，**不响应任何电机指令**，只响应枚举帧。

| 项目      | 取值                                | 来源                               |
| ------- | --------------------------------- | -------------------------------- |
| MCU     | APM32F103CBT6（Cortex-M3 @ 72 MHz） | `README.md`、`system_apm32f10x.c` |
| CAN 波特率 | 50 kbps                           | `board_driver.c`、`app_config.h`  |
| PWM     | TIM3_CH4，5 kHz                    | `board_config.h:18`              |
| 节点 ID 范围 | 1–6（主控分配） | `app_config.h:29-30` |



---

### 2. 核心设计原则

| 原则            | 实现方式                                     |
| ------------- | ---------------------------------------- |
| **主控驱动**      | ESP32 周期性广播 POLL/OFFER，从机被动应答            |
| **芯片 UID 区分** | APM32 读取 96 位芯片 UID → XOR 折叠为 32 位 `uid` |
| **ID 不持久化**   | 每次上电/软复位都重新枚举，ID 只存 RAM                  |
| **退避防碰撞**     | 从机 ACK 经 `hash(uid, seq) % 48ms` 延迟后回复   |
| **冲突自动解决**    | 同一 ID 被多个 UID 认领时，随机保留一个，其余 ASSIGN 重定位   |
| **掉线自动回收**    | 连续 3 个 cycle 未收到某 ID 的 ACK → 释放该 ID      |

---

### 3. 枚举协议帧 (CAN 50 kbps)

|CAN ID|名称|方向|负载|用途|
|---|---|---|---|---|
|`0x7E3`|ENUM_POLL|主→总线|(空)|点名：已分配节点回复 ACK|
|`0x7E0`|ENUM_OFFER|主→总线|`[0]=id`|向 UNSET 节点提供 ID|
|`0x7E1`|ENUM_ACK|从→主|`[0]=id [1..4]=uid32(LE)`|确认/认领 ID|
|`0x7E2`|ENUM_ASSIGN|主→总线|`[0]=new_id [1..4]=target_uid32(LE)`|冲突重定位|

全部落在 `0x7E0–0x7E3`，避开了：

- `0x000` 电机指令帧
- `0x001–0x006` 从机反馈帧
- `0x100+id` 事件帧
- `0x7DD/0x7DE` OTA 帧

---

### 4. 完整枚举流程

#### 4.1 从机侧 ([node_id.c](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/motor_control_apm32/app/Source/node_id.c) + [can_service.c](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/motor_control_apm32/app/Source/can_service.c))

```
上电 / 软复位
  │
  ├─ NodeIdInit()
  │    ├─ s_uid = UID_WORD0 ^ UID_WORD1 ^ UID_WORD2  (来自 0x1FFFF7E8)
  │    └─ s_nodeId = NODE_ID_UNSET (0)
  │
  └─ 主循环 (10ms tick)
       └─ CanServicePoll()
            ├─ 发射到期 ACK (延迟退避结束)
            └─ 读 FIFO0 → CanServiceHandleEnum()
                 ├─ ENUM_POLL (0x7E3):
                 │     if 已分配 ID (1..6):
                 │       CanServiceScheduleAck(myId)  ← 退避延迟后回复
                 │
                 ├─ ENUM_OFFER (0x7E0, data[0]=offered_id):
                 │     if 当前 UNSET && offered_id 有效(1..6):
                 │       NodeIdSet(offered_id)        ← 暂取该 ID
                 │       CanServiceScheduleAck(offered_id)
                 │
                 └─ ENUM_ASSIGN (0x7E2, [0]=new_id, [1..4]=target_uid):
                       if target_uid == 我的 s_uid:
                         NodeIdSet(new_id)            ← 主控指定新 ID
                         CanServiceScheduleAck(new_id)
```

**ACK 退避算法** ([can_service.c:72-83](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/motor_control_apm32/app/Source/can_service.c#L72-L83)):

```c
// hash = UID XOR (seq * 0x9E3779B9)  ← Knuth 乘数
// 再加双重 XOR-shift 混合
h = NodeIdGetUid() ^ (s_ackSeq * 2654435761U);
h ^= h >> 15;
h *= 2246822519U;
h ^= h >> 13;
s_ackSeq++;                                // 每次调度递增，打散碰撞

delay = h % NODE_ID_ENUM_ACK_SPREAD_MS;    // 0~47 ms
// ACK 在 delay 后由 CanServiceServiceAck() 发出
```

关键设计：使用**完整 UID** 而非 `uid % spread`，且混入 `s_ackSeq`，使得：

- 两个 UID 低位相同的板子不会落在同一时隙
- 即使某对板子在本轮碰撞，`s_ackSeq` 递增后下一轮必然不同槽位
- 6个从机在0~47ms的退避时间内均匀分布

#### 4.2 主控侧 ([enum_manager.c](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/ble_esp32/main/app/enum_manager.c) + [enum_logic.c](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/ble_esp32/main/app/enum_logic.c))

```
enum_manager_init()
  └─ state: 所有 ID 空闲, offer_id=1

主循环 (每 10ms tick, 但 OTA/标定时暂停)
  └─ enum_manager_tick(now_ms): - 通过main循环内RX_Handle调用
  
  ┌─ ENUM_IDLE ──────────────────────────────────────────┐                         
  │   ├─ 发送 ENUM_POLL(0x7E3)    ← 点名                   │
  │   ├─ 发送 ENUM_OFFER(offer_id) ← 若还有空闲 ID          │
  │   └─ 进入 ENUM_COLLECTING, 启动 250ms 收集窗口          │
  │ return后：将ACK响应的id&uid存入s_obs[]缓冲区           │
  ├─ ENUM_COLLECTING(250ms) ─────────────────────────────┤
  │ 等待 250ms 窗口结束 (覆盖所有从机的 0~47ms ACK 延迟)     │
  │   ├─ enum_logic_process() 处理收集到的 ACK              │
  │   |    ├─ 按 ID 分组认领的 UID                          │
  │   |    ├─ 冲突检测: 同 ID 多 UID → 随机保留1个          │
  │   |    │              其余 → 加入 relocate 列表          │
  │   |    ├─ 缺失检测: 某 ID 连续 3 cycle 未回复 → 释放     │
  │   |    └─ 对 relocate 中每个 UID 分配最低空闲 ID         │
  │   ├─→ 广播 ENUM_ASSIGN(new_id, target_uid)         │
  │   |     └─ 计算下次 offer_id = 最低空闲 ID             |
  │   └─→ 计算下一周期period time：1000ms(已分配满>=6)/400ms,   |
  |          状态机进入IDLE                                |
  |-  ENUM_IDLE ——————————————————————————————————————————|
  |  新的周期,直到所经过时间>period time,重复广播过程       |
  |                                                       |
  └──────────────────────────────────────────────────────┘
```

##### 4.2.1 发送 POLL/OFFER 帧的时间
**时刻**：在 `ENUM_IDLE` 阶段，一旦当前时间 >= `s_nextCycleMs`（[enum_manager.c:104-114](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/ble_esp32/main/app/enum_manager.c#L104-L114)）：

```c
if (s_phase == ENUM_IDLE) {
    if ((int32_t)(now_ms - s_nextCycleMs) >= 0) {
        s_nObs = 0;
        send_poll();                          // CAN ID 0x7E3, 1 字节, 10ms 超时
        if (s_offerId != 0) {
            send_offer(s_offerId);             // CAN ID 0x7E0, 1 字节, 10ms 超时
        }
        s_collectUntilMs = now_ms + ENUM_COLLECT_WINDOW_MS;  // = now + 250ms
        s_phase = ENUM_COLLECTING;
    }
}
```

**关键细节**：

- POLL 和 OFFER **在同一瞬间背靠背发送**（两个连续的 CAN 帧，无插入延迟）。
- `send_poll()` 发送 CAN ID `0x7E3`，1 字节有效载荷（零），10 ms 发送超时。
- `send_offer(s_offerId)` 发送 CAN ID `0x7E0`，1 字节有效载荷（最低空闲 ID），仅当 `s_offerId != 0`（即有空闲槽位）时才发送。
- 首次启动时 `s_nextCycleMs = 0`，`s_offerId = 1`，因此**第一个周期在启动后立即开始**，在第一个 10 ms 控制循环周期内。

**CAN 帧时间开销**：CAN 波特率为 50 kbit/s。一个标准 11 位 CAN 帧包含约 47–55 位（含填充位）+ 1 字节有效载荷 8 位。约 55–63 位 / 50,000 = **约 1.1–1.3 ms 每帧**。因此 POLL + OFFER 传输占用总线约 **2.2–2.6 ms**。

**冲突仲裁** ([enum_logic.c:76-99](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/ble_esp32/main/app/enum_logic.c#L76-L99)):

```c
if (n_claim[id] >= 1) {
    // 多个 UID 认领同一 ID → 随机保留一个
    int keep = (n_claim[id] == 1) ? 0 : (int)(rnd % n_claim[id]);
    new_holder[id] = claimers[id][keep];
    // 其余加入 relocate 列表
    for (int k = 0; k < n_claim[id]; k++)
        if (k != keep) relocate[n_reloc++] = claimers[id][k];
} else {
    // 本周期无 ACK → 继承旧 holder (容忍丢 1~2 次), 超阈值释放
}
```

---

### 5. 时序全景

```
ESP32 周期 N 开始
│
├─ ~0ms:  发送 POLL(0x7E3) + OFFER(0x7E0, id=N)
│         两根帧间隔约 1ms (CAN 50kbps 下每帧约 1.4ms)
│
├─ 0~48ms: 各 APM32 从机在不同退避时刻回复 ACK(0x7E1)
│           (6 节点均匀分布在 48ms 窗口内，几乎不会碰撞)
│
├─ 250ms:  收集窗口结束
│          enum_logic_process() 处理
│          → 如有冲突则发 ASSIGN(0x7E2)
│
├─ 400ms:  下一周期 (填充期) 或
│  1000ms: 下一周期 (全部就绪后，仅心跳巡检)
│
└─ 周期 N+1 开始...
```
