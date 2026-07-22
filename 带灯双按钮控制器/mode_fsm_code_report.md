# 双按钮模式状态机 / 灯效 / 电机控制 代码报告

> 作者自研模块，覆盖范围：`mode_fsm.c`、`motor_cmd.c`、`lightshow.c`（仅 effect 层）、`buttons.c`（按钮去抖）、以及 `main.c` 中的集成调度逻辑。  
> 文件位置：`ble_esp32/main/app/`

---

## 一、总体架构

### 1.1 模块分层与数据流向

```
┌─────────────────────────────────────────────────────────────────┐
│                        main.c 主循环                            │
│  CONTROL_LOOP_MS = 10ms 周期调度                                │
│                                                                 │
│  buttons_poll() ──→ mode_fsm_on_button() ──→ motor_cmd_start_*│
│                            │                      │             │
│                            ▼                      ▼             │
│                     mode_fsm_tick()        motor_cmd_tick()     │
│                            │                      │             │
│                            ▼                      ▼             │
│                     mode_fsm_get_state()  submit_angles()       │
│                            │                      │             │
│                            ▼                      ▼             │
│   lightshow_set_effect()        execute_gateway_commands()     │
│            │                              │                     │
│            ▼                              ▼                     │
│   lightshow_tick()              gateway (CAN总线 → 6台电机)     │
│            │                              │                     │
│            ▼                              ▼                     │
│   hal_strip (WS2812灯带)       mode_fsm_report_error()          │
│                                       │                        │
│                                       ▼                        │
│                                  fsm → ERROR                   │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 核心设计理念

1. **状态机与执行器分离**：`mode_fsm` 只管理状态逻辑和按钮仲裁，不知道具体电机动作；`motor_cmd` 只负责电机时序，不知道 FSM 状态。两者通过 `motor_cmd_is_done()` 单向耦合——FSM 轮询"做完了吗"来自动推进状态。
2. **灯效与状态机分离**：main.c 每周期将 FSM 状态映射为灯效，通过 `lightshow_set_effect()` 推给灯效模块。灯效模块（effect 层）不知道 FSM 存在，只管渲染。
3. **互斥仲裁**：两个按钮（回球 / 随机地形）共享 6 台电机资源，同一时刻只允许一个模式运行。按 A 时如果 B 正在运行，B 被立即停止，A 启动。
4. **错误漏斗**：底层网关/电机检测到硬件故障（堵转、超时、传感器故障、CAN总线关闭）后，通过 `motor_cmd_active_fsm_mode()` 反查当前是哪个模式在运行，再调用 `mode_fsm_report_error()` 注入到 FSM。

### 1.3 涉及的核心文件清单

| 文件 | 职责 |
|---|---|
| `main.c` | 主循环，串联按钮→FSM→电机→灯效的调度链路 |
| `buttons.c` / `.h` | 按钮硬件去抖 + 短按/长按事件检测 |
| `mode_fsm.c` / `.h` | 双模式有限状态机 (IDLE/RUNNING/FINISHED/ERROR) |
| `motor_cmd.c` / `.h` | 回球多步序列 & 随机地形 preset 驱动 |
| `lightshow.c` / `.h` | 灯带渲染（effect 状态灯效 + legacy pattern 灯效） |
| `gateway.c` / `.h` | CAN总线电机调度、队列管理、故障检测与上报告警 |
| `app_config.h` | 全局常量（CONTROL_LOOP_MS 等） |

---

## 二、按钮模块 (buttons.c)

### 2.1 功能概述

在 `hal_button` 硬件抽象层之上提供**软件去抖**、**短按检测**、**长按检测**，每次稳定按下只产生一次性事件，避免重复触发。

### 2.2 核心数据结构

```c
static bool     s_stable[HAL_BUTTON_COUNT];   // 去抖后的稳定状态 (true=按下)
static uint8_t  s_counter[HAL_BUTTON_COUNT];  // 候选状态连续采样计数
static uint16_t s_hold_ticks[HAL_BUTTON_COUNT]; // 长按累计 tick 数
static bool     s_long_fired[HAL_BUTTON_COUNT]; // 长按事件是否已触发(防重复)
```

### 2.3 关键参数

| 参数 | 值 | 含义 |
|---|---|---|
| `BUTTON_DEBOUNCE_TICKS` | 3 | 连续 3 次采样一致才认为状态稳定 (= 30ms) |
| `BUTTON_LONGPRESS_TICKS` | 150 | 持续按下 150 tick 触发长按 (= 1.5s) |

### 2.4 主函数逻辑：`buttons_poll()`

```
每 CONTROL_LOOP_MS (10ms) 由 main 循环调用一次：
  1. 读取 hal_button_is_pressed(i) 当前电平
  2. 与 s_stable[i] 比较：
     - 不同 → s_counter[i]++；达到 3 次阈值后：
         - 提交新状态
         - 若是按下沿(0→1)，置位 s_event_bit[i]（短按事件）
         同时重置长按计数器
     - 相同 → s_counter[i] 重置为 0
  3. 长按检测：
     - 若 s_stable[i]==true，s_hold_ticks[i]++ 
     - 累积达到 150 且未触发过 → 置位 s_long_event_bit[i]
  4. 返回所有事件的位掩码
```

**关键设计**：长按事件**必定在短按事件之后**触发（因为按下时先产生短按事件，持续按住 1.5s 后才补发长按），且每个长按只触发一次直到松开重新计时。

### 2.5 事件位定义

| 位 | 含义 | 消费方 |
|---|---|---|
| `BUTTON1_EVENT_BIT` | Button1 短按 | main.c → `mode_fsm_on_button(FSM_MODE_RETURN)` |
| `BUTTON2_EVENT_BIT` | Button2 短按 | main.c → `mode_fsm_on_button(FSM_MODE_RANDOM)` |
| `BUTTON_BOOT_LONG_EVENT_BIT` | BOOT 长按 | main.c → `gateway_start_calibration()` |

---

## 三、模式状态机 (mode_fsm.c)

### 3.1 状态定义

```
                  ┌──────────┐
     按钮/重试    │   IDLE   │◄──────────────┐
   ┌─────────────►│  空闲    │                │
   │              └────┬─────┘                │
   │       start_mode()│                      │
   │                   ▼                      │
   │              ┌──────────┐   完成         │
   │   按钮(STOP) │ RUNNING  │────────────┐   │
   │   ◄──────────│  运行中  │ motor_cmd  │   │
   │              └────┬─────┘ _is_done() │   │
   │                   │                  ▼   │
   │    report_error() │           ┌──────────┐│
   │   (gateway注入)   │           │ FINISHED ││
   │                   ▼           │ 完成展示 ││
   │              ┌──────────┐     └────┬─────┘│
   └─── 按钮 ────│  ERROR   │   600ms后 │
       (清错重试) │  故障    │           │
                  └──────────┘           │
                         ▲               │
                         └───────────────┘
```

四个状态：
- **IDLE**（0）：等待按钮触发
- **RUNNING**（1）：电机正在执行
- **FINISHED**（2）：执行完成，过渡展示 600ms 后自动回 IDLE
- **ERROR**（3）：硬件故障，锁定直到用户再次按按钮清错重试

### 3.2 核心数据结构

```c
typedef struct {
    fsm_state_t state;      // 当前状态
    uint32_t    timer_ms;   // 在当前状态已停留的毫秒数
} mode_ctx_t;

static mode_ctx_t s_mode_ctx[FSM_MODE_COUNT];  // [0]=RETURN, [1]=RANDOM
```

每个模式维护一个独立的状态上下文。两个模式之间没有共享变量，互斥通过 `mode_fsm_on_button()` 中的仲裁逻辑实现。

### 3.3 关键函数

#### `mode_fsm_on_button(fsm_mode_t mode)` —— 按钮事件处理

```
输入：按下的按钮对应的模式（RETURN 或 RANDOM）

逻辑：
  1. 参数校验：mode >= FSM_MODE_COUNT 则忽略
  2. 获取对方模式 other = (mode == RETURN) ? RANDOM : RETURN
  3. 获取当前状态 st = s_mode_ctx[mode].state

  4. 若 st == ERROR：
       → mode_fsm_clear_error(mode) 回到 IDLE
       → 继续向下执行 (fall-through 到启动)

  5. 若 st == RUNNING：
       → stop_mode(mode) 回到 IDLE
       → return (不做进一步处理)

  6. 仲裁：若 对方模式正在 RUNNING：
       → stop_mode(other) 停掉对方

  7. start_mode(mode)：
       → 调用 motor_cmd_start_return() 或 motor_cmd_start_random()
       → enter_state(mode, FSM_STATE_RUNNING)
```

**设计要点**：
- ERROR 状态下按按钮 = 清错 + 立即重试
- RUNNING 状态下按按钮 = 停止，不做下一个动作
- IDLE/FINISHED/刚清错的 ERROR 状态 = 启动（如对方在跑则先停对方）

#### `mode_fsm_tick(uint32_t dt_ms)` —— 自动状态推进

```
每 10ms 调用一次，遍历两个模式：

  for each mode：
    timer_ms += dt_ms

    若 state == RUNNING：
      if (motor_cmd_is_done())：
        → enter_state(FINISHED)     // 电机执行完毕

    若 state == FINISHED：
      if (timer_ms >= 600ms)：
        → enter_state(IDLE)         // 完成展示超时，回到空闲
```

**关键设计**：RUNNING→FINISHED 的转换**完全依赖** `motor_cmd_is_done()` 的返回值。FSM 本身没有超时——超时保护在 gateway 层，gateway 检测到电机超时/堵转会通过 `mode_fsm_report_error()` 主动注入 ERROR 状态。

#### `mode_fsm_report_error(fsm_mode_t mode)` —— 错误注入（外部调用）

被调用场景（均在 `gateway.c` 中）：
1. **CAN 总线关闭** (bus-off)：`gateway_can_health_check()` 检测到后调用
2. **电机传感器故障**：`gateway_can_rx_handler()` 解析 CAN 反馈帧时检测到 encoder error
3. **电机堵转**（stall）：队列调度器重试耗尽后
4. **电机超时**（queue timeout）：单电机等待超时 `MOTOR_WAIT_TIMEOUT_MS`

这些外部调用者都通过 `motor_cmd_active_fsm_mode()` 反查当前是 RETURN 还是 RANDOM 在运行，然后精准注入到对应的 FSM 实例。

---

## 四、电机命令模块 (motor_cmd.c)

### 4.1 功能概述

负责把 FSM 的模式请求转化为**完整的电机角度/高度命令序列**，通过 gateway 的 CAN 队列逐台发送给 6 个直流无刷电机节点。

### 4.2 内部命令模式

```c
typedef enum {
    CMD_IDLE = 0,   // 空闲
    CMD_RETURN,     // 回球模式多步序列执行中
    CMD_RANDOM,     // 随机地形模式执行中
} cmd_mode_t;
```

### 4.3 数据表设计

#### 回球序列 (Return Steps)

```c
// 共 3 步 × 1 个循环
static const return_step_t RETURN_STEPS[3] = {
    // Step 1: 电机3,4 顶起到 71.5°，电机5,6 降到 1.5° → 球从洞中脱出
    [0] = { .angles = {-1, -1, 71.5, 71.5, 1.5, 1.5}, .hold_ms = 500 },

    // Step 2: 电机5 顶起到 71.5°，电机1-4 降到中位 → 球向左滚动
    [1] = { .angles = {39, 39, 58, 58, 71.5, -1}, .hold_ms = 500 },

    // Step 3: 电机6 顶起到 71.5° → 球完全返回
    [2] = { .angles = {-1, -1, -1, -1, -1, 71.5}, .hold_ms = 800 },
};
```

- `-1` 表示该电机在当前步骤不参与（保持原位）
- `hold_ms` 表示所有参与电机到达目标后在当前位姿保持的时间

#### 随机地形 (Terrain Presets)

```c
static const terrain_preset_t TERRAIN_PRESETS[3] = {
    [0] = { .angles = {39, 39, 33, 31.5, 28.5, 26.4} },  // 预设1：缓坡
    [1] = { .angles = {60, 57, 58, 55, 61.5, 53.4} },     // 预设2：陡坡
    [2] = { .angles = {29, 25.4, 17.5, 13, 5.5, 4.96} },  // 预设3：深谷
};
```

每次随机选择一个 preset，所有 6 台电机一次性下达目标角度。

### 4.4 关键函数

#### `motor_cmd_start_return()` —— 启动回球模式

```
1. s_mode = CMD_RETURN
2. step=0, cycle=0, hold_timer=0, done=false
3. submit_angles(RETURN_STEPS[0].angles)   // 立即提交第 1 步
```

#### `motor_cmd_start_random()` —— 启动随机模式

```
1. s_mode = CMD_RANDOM
2. hold_timer=0, done=false
3. preset_index = esp_random() % 3   // 随机选一个 preset
4. submit_angles(TERRAIN_PRESETS[preset_index].angles)  // 立即提交全部角度
```

#### `motor_cmd_tick(uint32_t dt_ms)` —— 步骤推进器

```
1. 若 s_mode == CMD_IDLE → 直接返回
2. 若 gateway_is_queue_idle() == false → 等待队列空闲 (电机还在移动)
3. s_hold_timer += dt_ms    (累加保持时间)

4. 若 s_mode == CMD_RETURN：
   a. 若 hold_timer < RETURN_STEPS[step].hold_ms → 还在保持，返回等待
   b. 保持时间到：
      - step++
      - 若 step >= 3（走完所有步）：
        - cycle++
        - 若 cycle >= 1（完成所需循环）：
          → s_done = true, s_mode = CMD_IDLE, return
        - 否则重置 step=0 重新开始
      - submit_angles(RETURN_STEPS[step].angles)  // 提交下一步

5. 若 s_mode == CMD_RANDOM：
   → 一次性的，所有电机到齐即完成：
      → s_done = true, s_mode = CMD_IDLE
```

**关键设计**：
- 回球是**异步多步序列**：每一步提交一组角度 → 等待所有参与的电机到达 → 保持 → 提交下一步
- 随机地形是**一次性并行**：6 台电机同时发往各自目标，全部到达即完成
- `gateway_is_queue_idle()` 返回 true 时，说明当前批次所有电机均已到达目标

#### `motor_cmd_is_done()` —— 一次性完成标志

```c
bool motor_cmd_is_done(void) {
    bool was_done = s_done;
    s_done = false;   // 读取后立即清除，防止旧标志重复触发
    return was_done;
}
```

典型的一次性信号模式（oneshot flag），读后即清零，确保 `mode_fsm_tick()` 不会因旧标志重复触发 FINISHED 转换。

#### `submit_angles()` —— 角度→高度→提交到 gateway

```c
static void submit_angles(const float angles[6]) {
    float heights[6];
    for (int i = 0; i < 6; i++) {
        heights[i] = (angles[i] < 0) ? -1.0f : height_from_angle(angles[i]);
    }
    execute_gateway_commands(
        GATEWAY_TS_INTERNAL,   // 内部提交，跳过时间戳重放保护
        heights[0], ..., heights[5],
        999.0f                 // 安全限值跳过（预定义序列无需保护）
    );
}
```

内部命令使用 `GATEWAY_TS_INTERNAL` 标记和时间戳保护；相邻高度差检查用 `999.0f` 跳过——因为回球/随机地形的角度表是人工验证过的，不需要安全检查。

#### `motor_cmd_stop()` —— 紧急停止

```
调用 gateway_estop() （广播 ESTOP 到 CAN 总线）
所有内部状态归零 → CMD_IDLE
```

---

## 五、灯效模块 (lightshow.c) — 仅 effect 层

### 5.1 双系统架构

lightshow.c 内部维护了两套独立的灯效系统：

| 层 | 枚举 | 触发方式 | 优先级 | 本报告范围 |
|---|---|---|---|---|
| **Effect 层** | `light_effect_t` | main.c 根据 FSM 状态自动设置 | **高**（effect≠NONE 时覆盖 pattern） | ✅ **本次新做** |
| Pattern 层 | `light_pattern_t` | 按钮循环切换 (OFF→SOLID→BLINK→…) | 低（仅在 effect=NONE 时生效） | ❌ 非本次所做 |

### 5.2 Effect 枚举与 FSM 状态映射

```
FSM_STATE_IDLE     ──→ LIGHT_EFFECT_IDLE      ──→ 白色常亮
FSM_STATE_RUNNING  ──→ LIGHT_EFFECT_RUNNING   ──→ 黄绿色呼吸 (2s周期, 30%~100%)
FSM_STATE_FINISHED ──→ LIGHT_EFFECT_FINISHED  ──→ 黄绿色快闪 3次 (100ms on/off)
FSM_STATE_ERROR    ──→ LIGHT_EFFECT_ERROR     ──→ 橙红色快闪持续 (250ms on/off)
```

映射函数在 `main.c` 中：

```c
static light_effect_t effect_for_state(fsm_state_t s) {
    switch (s) {
        case FSM_STATE_RUNNING:  return LIGHT_EFFECT_RUNNING;
        case FSM_STATE_FINISHED: return LIGHT_EFFECT_FINISHED;
        case FSM_STATE_ERROR:    return LIGHT_EFFECT_ERROR;
        case FSM_STATE_IDLE:
        default:                 return LIGHT_EFFECT_IDLE;
    }
}
```

### 5.3 Effect 渲染实现

#### IDLE — 白色常亮
```c
hal_strip_fill(strip, 255, 255, 255);  // 全白，亮度可调
```

#### RUNNING — 黄绿色呼吸
```c
// 三角波亮度：30% → 100% → 30%，一个完整周期 = 2s
uint32_t pct = 30 + ramp * 70 / 100;   // 30~100%
hal_strip_fill(strip, 220*pct/100, 255*pct/100, 3*pct/100);
```

色值：`R=220, G=255, B=3`，定义为 `YELLOWGREEN_*`。

#### FINISHED — 黄绿色快闪
```c
// ON 100ms / OFF 100ms 交替闪烁
bool on = ((phase / 10) % 2) == 0;
hal_strip_fill(strip, on ? 220:0, on ? 255:0, on ? 3:0);
```

由于 FINISHED 状态在 600ms 后自动回到 IDLE，实际可见约 3 次闪烁。

#### ERROR — 橙红色快闪
```c
// ON 250ms / OFF 250ms 交替闪烁
bool on = ((phase / 25) % 2) == 0;
hal_strip_fill(strip, on ? 255:0, on ? 60:0, on ? 0:0);
```

色值：`R=255, G=60, B=0` (橙红)。ERROR 状态不会自动退出，持续闪烁直到用户按按钮清错。

### 5.4 关键函数

#### `lightshow_set_effect(uint8_t strip, light_effect_t effect)`

```c
void lightshow_set_effect(uint8_t strip, light_effect_t effect) {
    if (s_state[strip].effect == effect) return;   // 相同 → 不重置 phase，动画不中断
    s_state[strip].effect = effect;
    s_state[strip].phase  = 0;    // 切换 effect 时重置动画相位
    s_state[strip].dirty  = true; // 标记需要刷新
}
```

**关键设计**：相同 effect 被重复设置时直接 return，**不重置 phase**。这很重要——main 循环每 10ms 都会调用一次，如果每次都重置 phase，呼吸/闪烁动画的计时会被不断打断，永远无法完成完整周期。

#### `lightshow_tick()` —— 每帧渲染

```c
void lightshow_tick(void) {
    for (uint8_t i = 0; i < 2; i++) {
        // 判断是否需要动画刷新
        bool animated = (effect != NONE)
            ? (effect != IDLE)         // effect模式：IDLE是静态，其余动画
            : pattern_is_animated();   // pattern回退模式

        if (dirty || animated) {
            render_strip(i);    // → 先检查 effect≠NONE → render_effect()
            dirty = false;
        }
        phase++;  // 帧计数器累加
    }
}
```

**优化**：静态效果（IDLE 白色）只在 dirty 标记时渲染一次，不每帧刷新，减少不必要的 SPI 传输。

---

## 六、主循环集成 (main.c)

### 6.1 初始化顺序

```
app_main():
  1. buttons_init()
  2. lightshow_init()
  3. gateway_init()
  4. hal_ble_init()            // BLE 必须先初始化
  5. mode_fsm_init()           // 依赖 BLE（发送 JSON 消息）
  6. motor_cmd_init()
  7. gateway_broadcast_config()
```

### 6.2 主循环调度（10ms 周期）

```
while(1):
  ┌─ gateway_can_health_check()     // CAN 健康检测 + 自动恢复
  ├─ gateway_can_rx_handler()       // 处理 CAN 反馈帧（故障上报等）
  ├─ gateway_update_motor_queue()   // 队列调度：检查到达/超时/堵转
  │
  ├─ uint32_t btn = buttons_poll()  // 按钮去抖 + 事件检测
  │   ├─ btn & BUTTON1 → mode_fsm_on_button(FSM_MODE_RETURN)
  │   └─ btn & BUTTON2 → mode_fsm_on_button(FSM_MODE_RANDOM)
  │
  ├─ mode_fsm_tick(10)              // FSM 时间推进（RUNNING→FINISHED→IDLE）
  ├─ motor_cmd_tick(10)             // 电机步骤推进（hold→next step→done）
  │
  ├─ lightshow_set_effect(0, effect_for_state(fsm_ret))   // 灯带0 ← RETURN状态
  ├─ lightshow_set_effect(1, effect_for_state(fsm_rnd))   // 灯带1 ← RANDOM状态
  └─ lightshow_tick()               // 渲染灯带帧
```

### 6.3 完整事件流示例

**场景：用户按 Button1（回球模式）**

```
1. buttons_poll() 检测到 BUTTON1 按下沿 → 返回 BUTTON1_EVENT_BIT
2. main → mode_fsm_on_button(FSM_MODE_RETURN)
3.   mode_fsm: IDLE → start_mode():
4.     motor_cmd_start_return() → submit_angles(step1) → CAN发送6台电机命令
5.     enter_state(RUNNING) → BLE发送 {"fsm":"RETURN","state":"RUNNING"}
6.   (若 Button2 正在 RUNNING → stop_mode(other) → gateway_estop() → enter_state(IDLE))
7. 下一帧 main: lightshow_set_effect(0, LIGHT_EFFECT_RUNNING) → 灯带0绿呼吸
8. motor_cmd_tick(): 等 gateway 队列空闲 → 保持 hold_ms → 下一步 → ... → done=true
9. mode_fsm_tick(): motor_cmd_is_done()==true → enter_state(FINISHED)
10. main: lightshow_set_effect(0, LIGHT_EFFECT_FINISHED) → 灯带0快闪
11. mode_fsm_tick(): timer >= 600ms → enter_state(IDLE)
12. main: lightshow_set_effect(0, LIGHT_EFFECT_IDLE) → 灯带0白色常亮
```

**场景：电机堵转 (stall) → ERROR 流程**

```
1. gateway_update_motor_queue(): 检测某电机堵转，重试耗尽
2.   active = motor_cmd_active_fsm_mode()  → 返回当前模式（如 FSM_MODE_RETURN）
3.   mode_fsm_report_error(FSM_MODE_RETURN)
4.     → s_mode_ctx[RETURN].state = ERROR, timer=0
5.     → BLE发送 {"fsm":"RETURN","state":"ERROR"}
6. 下一帧 main: lightshow_set_effect(0, LIGHT_EFFECT_ERROR) → 灯带0橙红快闪(持续)
7. 用户按 Button1：
8.   mode_fsm_on_button(RETURN): st==ERROR → mode_fsm_clear_error() → IDLE
9.   → 继续 fall-through → start_mode(RETURN) → 重试回球
```

---

## 七、关键设计决策与要点总结

| 决策点 | 方案 | 原因 |
|---|---|---|
| FSM 与 motor_cmd 分离 | 通过 `motor_cmd_is_done()` 单向轮询 | 解耦，FSM 不关心电机内部步骤数、hold 时间等细节 |
| 灯效通过 main 映射而不是 FSM 直接驱动 | main.c 每帧调用 `effect_for_state(fsm)` | FSM 不依赖 lightshow，可独立测试 |
| effect 重复设置不重置 phase | `if (effect == st->effect) return;` | 主循环每 10ms 推一次，若重置 phase 会导致动画永远卡在第 0 帧 |
| 错误注入通过 gateway → motor_cmd_active_fsm_mode() | 反向查 active mode | gateway 不知道 FSM 的存在，只通过 motor_cmd 反查当前模式 |
| 回球多步序列用 `-1` 表示"不参与" | `if (angles[i] < 0) height = -1` | 允许每一步只动部分电机，其余保持原位 |
| 内部命令跳过 ts 重放保护 | `GATEWAY_TS_INTERNAL = -1LL` | 内部无 epoch 时钟，用 sentinel 值绕过 |
| 预定义序列跳过相邻高度差检查 | `current_limit = 999.0f` | 数据表由人工验证，不需要运行时安全保护 |
| FINISHED 600ms 过渡态 | 短暂展示完成效果后回 IDLE | 给用户明确的"操作完成"视觉反馈 |
| oneshot `motor_cmd_is_done()` | 读后清零 | 防止因控制循环多次调用而重复触发 FINISHED 转换 |

---

## 八、时序参数汇总

| 参数 | 值 | 位置 |
|---|---|---|
| 主循环周期 | 10ms | `CONTROL_LOOP_MS` |
| 按钮去抖 | 3 ticks (30ms) | `BUTTON_DEBOUNCE_TICKS` |
| 长按判定 | 150 ticks (1.5s) | `BUTTON_LONGPRESS_TICKS` |
| FINISHED 展示 | 600ms | `FINISHED_MS` |
| 回球 Step1 hold | 500ms | `RETURN_STEPS[0].hold_ms` |
| 回球 Step2 hold | 500ms | `RETURN_STEPS[1].hold_ms` |
| 回球 Step3 hold | 800ms | `RETURN_STEPS[2].hold_ms` |
| RUNNING 呼吸周期 | 200 ticks (2s) | `BREATHE_PERIOD_TICKS` |
| FINISHED 快闪 | ON 100ms / OFF 100ms | `FINISH_ON_TICKS = 10` |
| ERROR 快闪 | ON 250ms / OFF 250ms | `ERROR_ON_TICKS = 25` |

---

*报告结束。如需修改参数或调整数据表，只需修改对应的 `#define` 或 `RETURN_STEPS`/`TERRAIN_PRESETS` 数组即可，核心逻辑无需变更。*
