### 缺陷1 电流堵转 —— 失效
#### **代码位置**：[control_protection.c:88-125]
**函数：void ControlProtectionUpdate(uint16_t currentMa, uint16_t angleDeg)**
这个函数整体是在**每个控制周期（`CONTROL_LOOP_MS`）更新保护状态**。它维护了三类保护：
1. 命令超时（Command timeout）
2. 过流保护（Overcurrent）
3. 堵转保护（Current-based stall detection）

##### 1 调用链
```
main() @ main.c:970
  └─ while(1)
       └─ RunControlLoop() @ main.c:693           ← 每 10ms 执行一次
            │
            ├─ ① CanServicePoll / ProcessCanCommand  ← 处理 CAN 帧
            │    └─ ControlProtectionSetCommandTimestamp()   ← 任何 CAN 帧都刷新
            │    └─ ControlProtectionOnMotionCommand()       ← 仅 CONTROL 帧触发
            │
            ├─ ② nowAngle = motor_ctrl_get_angle()   //得到当前角度
            │
            ├─ ③ if (校准中) → CalibrationUpdate(); return;  ← 校准期间跳过保护
            │
            ├─ ④ if (启动阶段 is_startup_phase) → 仅刷新时间戳，不调用 Update  ← L759-762
            │
            └─ ⑤ else → 正式执行保护 ───────────────────────┐
                 currentMa = AdcServiceReadMotorCurrentMilliAmp();//读进电机当前电流
                 angleCenti = (uint16_t)(nowAngle * 100.0);
                 ControlProtectionUpdate(currentMa, angleCenti);  ← ★ 分析目标
                 │
                 └─ if (stall || overcurrent)
                      motor_ctrl_force_fault() → 停机
                      CanServiceSendFeedback() → 上报 ERROR
```
**时序：**
```c
main() while(1):
    RunControlLoop()
    SystemTickDelayMs(CONTROL_LOOP_MS)   // = 10ms
```
所以 `ControlProtectionUpdate()` 的调用周期精确等于 `CONTROL_LOOP_MS` = **10ms**。
##### **2 Stall Detection**
>只有"电流**一直很大**"并且"角度一直没有明显前进"，才认为堵转
>`Current High  AND  No Progress`

**1）起步宽限期 (Motion Grace Period)**
```c
/* s_motionGraceMs: remaining post-command stall-detection grace
 * s_motionGraceMs > 0,说明系统刚刚下发了新动作指令,给予一段宽限期，期间内不进行堵转判定 */
if (s_motionGraceMs > 0U)
    {
        /* Post-command grace (see ControlProtectionOnMotionCommand): keep the
         * window anchored to wherever the shaft is so the 500 ms judgement
         * starts fresh when the grace runs out. */
        /* 将窗口固定在轴的任何位置，以便在宽限期结束时重新开始500毫秒的判断。*/
        s_motionGraceMs = (s_motionGraceMs > CONTROL_LOOP_MS)
                              ? (uint16_t)(s_motionGraceMs - CONTROL_LOOP_MS)
                              : 0U;
        // 安全的无符号数防溢出递减操作：定时器负责递减宽限时间，每轮减少10ms            
        s_stallAnchor = angleDeg; // 更新锚点
        s_stallMs = 0U; // 清零累计时间
        s_stall = 0U;   // 清零堵转标记
    }
```
- `s_motionGraceMs`：每次运动命令后的宽限期（ms），在此期间，基于电流的失速检测器仅重新锚定，从不累积。
- **关键动作**：不断将当前的轴角度 (`angleDeg`) 刷新为“锚点”(`s_stallAnchor`)。这确保了当宽限期结束时，堵转判定的起点是最新的实际位置。

**2）高电流状态下的净位移判定 (Stall Condition Check)**
```c
/* 如果电流超过了堵转阈值STALL_CURRENT_LIMIT_MA=800mA*/
else if (currentMa >= STALL_CURRENT_LIMIT_MA)
    {
        uint16_t moved = (angleDeg >= s_stallAnchor)
                             ? (uint16_t)(angleDeg - s_stallAnchor)
                             : (uint16_t)(s_stallAnchor - angleDeg);
        // 计算净位移(绝对值)
        if (moved >= STALL_CURRENT_PROGRESS_CENTIDEG) // 判断为正在移动
        {
            s_stallAnchor = angleDeg; // 重锚
            s_stallMs = 0U;
            s_stall = 0U;
        }
        else
        {
            s_stallMs += CONTROL_LOOP_MS; // 累计stall时间
            if (s_stallMs >= STALL_CURRENT_TRIP_MS)
            {
                s_stall = 1U;
            }
        }
    }
```
- 当前电流>=`STALL_CURRENT_LIMIT_MA`(800mA)时，才开始判定堵转：
- 计算`moved`净位移量（当前角度与锚点角度）—— 如果超过最小步进角度阈值（0.2deg -> 0.2×100=20），说明虽然电流大但仍在转动，进行重锚。否则累计stall时间（每轮增加调用周期10ms），直到累计时间大于`STALL_CURRENT_TRIP_MS`，判定为堵转。

**3）正常电流状态**
```c
else
    {
        s_stallAnchor = angleDeg; //持续重锚
        s_stallMs = 0U;
        s_stall = 0U;
    }
```

##### 3 Issue42对应场景
> "阻尼器退化后电机空转、**电流仅约 0.3–0.4 A**"  ->  即该电流 < 800mA的阈值`STALL_CURRENT_LIMIT_MA`，从判定的角度属于**正常电流状态**，无法触发堵转累计。

##### 4 问题总结
`ControlProtectionUpdate()` 的 stall 检测是一个**合取门**（AND gate）：

```
stall = (电流 ≥ 800mA) ∧ (净进展 < 0.2°) ∧ (累积 ≥ 500ms)
```

退化阻尼器场景下，第一个条件 `电流 ≥ 800mA` 不成立，整个合取表达式短路为 false。**角度信息虽然在参数中传入（`angleDeg`），但在 L120-125 的 else 分支中仅用于重置锚点，不做任何堵转判定。**

### 缺陷2 角度堵转 ——又慢又易被绕过
#### **代码位置**：[motor_controller.c:316-350]
**函数：bool motor_ctrl_update(motor_ctrl_t \*ctrl, float now_angle)**

##### 总体逻辑拓扑
```
motor_ctrl_update()
|
├─ fault_latched 故障锁存判定 (是) -> 速度归零，return false
|
├─ MOTOR_STATUS_RUNNING 电机运行状态判定 (否) -> return false
|
├─ SUB_MODE_SPEED 子模式速度模式判定 (是) -> return false
|
├─ motor_ctrl_angle_is_valid(now_angle) 角度传感器数据有效 (否) -> return false
|
|  更新轨迹
|
|  timeout_counter计时增加
|
├─ 启动: is_startup_phase 是否已经克服静摩擦(移动>0.5度或超时) -> 若是则退出“启动阶段”
|
|
├─ 堵转Detection: fabsf(pos_err) > POSITION_TOLERANCE(1.5°) 距离目标较远
|    ├─ |now_angle - ctrl->stall_anchor_angle| >= STALL_PROGRESS_DEG(0.5°)
           [净进度大于步进窗口]
             -> 重锚
|    └─ 累积Stall计数
|
|
└─ 核心状态判定
    |
    ├─ |pos_err| <= POSITION_ARRIVE_DEADBAND [目标距离死区范围内] && 
        |derivative| < ARRIVE_SPEED_THRESH [速度减小至阈值范围] &&
        |ctrl->profile_velocity|  < ARRIVE_SPEED_THRESH * 100.0f
        误差极小&速度极低 -> 停机，切换为“已到达”状态
    | 
    ├─ ctrl->stall_counter > STALL_DETECT_CYCLES 
        净进度堵转时间超过周期窗口(3s) -> “堵转”状态
    
    |
    ├─ timeout_counter > timeout_ms(6000ms) 
        超过6s没有判定为“到达”和“堵转” -> "超时"状态
    |
    └─ PID运算与动力输出 (正常运行控制) - 计算基础输出：PD控制 + 速度前馈(Feedforward)
        |
        ├─ “启动阶段” 
             阶段 1 (Kick)：输出瞬时高PWM脉冲，打破静摩擦。
             阶段 2 (Ramp)：PWM线性爬升，直到追上PD计算值或目标限速。
        |
        └─ 常规运行阶段
             
    
            

```