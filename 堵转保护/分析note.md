#### **代码位置**：[control_protection.c:88-125]
**函数：void ControlProtectionUpdate(uint16_t currentMa, uint16_t angleDeg)**
这个函数整体是在**每个控制周期（`CONTROL_LOOP_MS`）更新保护状态**。它维护了三类保护：
1. 命令超时（Command timeout）
2. 过流保护（Overcurrent）
3. 堵转保护（Current-based stall detection）

**Stall Detection**
>只有"电流一直很大"并且"角度一直没有明显前进"，才认为堵转
>`Current High  AND  No Progress`

1.起步宽限期 (Motion Grace Period)
```c
/* s_motionGraceMs: remaining post-command stall-detection grace
 * 命令刚发出去以后的一段宽限期 */
if (s_motionGraceMs > 0U)
    {
        /* Post-command grace (see ControlProtectionOnMotionCommand): keep the
         * window anchored to wherever the shaft is so the 500 ms judgement
         * starts fresh when the grace runs out. */
        /* 将窗口固定在轴的任何位置，以便在宽限期结束时重新开始500毫秒的判断。*/
        s_motionGraceMs = (s_motionGraceMs > CONTROL_LOOP_MS)
                              ? (uint16_t)(s_motionGraceMs - CONTROL_LOOP_MS)
                              : 0U;
        s_stallAnchor = angleDeg;
        s_stallMs = 0U;
        s_stall = 0U;
    }
```
- 如果 `s_motionGraceMs > 0`，说明系统刚刚下发了新动作指令，给予一段“宽限期”。
- 在这个期间内，不进行堵转判定，定时器只负责递减宽限时间。
- **关键动作**：不断将当前的轴角度 (`angleDeg`) 刷新为“锚点”(`s_stallAnchor`)。这确保了当宽限期结束时，堵转判定的起点是最新的实际位置。
2.
```c
else if (currentMa >= STALL_CURRENT_LIMIT_MA)
    {
        uint16_t moved = (angleDeg >= s_stallAnchor)
                             ? (uint16_t)(angleDeg - s_stallAnchor)
                             : (uint16_t)(s_stallAnchor - angleDeg);
                             
        if (moved >= STALL_CURRENT_PROGRESS_CENTIDEG)
        {
            s_stallAnchor = angleDeg;
            s_stallMs = 0U;
            s_stall = 0U;
        }
        else
        {
            s_stallMs += CONTROL_LOOP_MS;
            if (s_stallMs >= STALL_CURRENT_TRIP_MS)
            {
                s_stall = 1U;
            }
        }
    }
    else
    {
        s_stallAnchor = angleDeg;
        s_stallMs = 0U;
        s_stall = 0U;
    }
```