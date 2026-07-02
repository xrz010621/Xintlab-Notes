# 堵转保护中霍尔传感器固件可能缺失+多次堵转后阻尼器失效 #42
相关链接：[堵转保护中霍尔传感器固件可能缺失+多次堵转后阻尼器失效·Issue #42·xintlabs/puttreal-firmware-apm32](https://github.com/xintlabs/puttreal-firmware-apm32/issues/42#issue-4742349703)
## 现象

摇臂多次堵转（约 10–20 次）使阻尼器（滑动离合）退化后，堵住摇臂时电流保护不再触发，电机长时间空转。

## 硬件前提

MT6701 磁编码器磁铁装在**摇臂（输出）轴**上，摇臂被堵时编码器随之冻结（即使阻尼器打滑、电机空转）。主控用**位置模式**驱动（gateway `data[0]=0x05` → `sub_mode=1`），故 `motor_ctrl_update()` 的角度堵转检测会运行。因此“角度/霍尔保护”并非缺失，也不会被打滑本身击穿。

## 根因（已复现验证）

阻尼器退化后电机空转、电流仅约 0.3–0.4 A，暴露出三处固件缺陷：

1. **电流堵转\*\*\*\*失效** — `control_protection.c` `ControlProtectionUpdate()` 要求 `电流≥800 mA` **且**编码器冻结（合取）。低电流下条件不成立，永不触发（对应报告中“没触发电流保护”）。
2. **角度堵转\*\*\*\*又慢又易被绕过** — `motor_controller.c`（约 L306–349）要求 `STALL_DETECT_CYCLES(300=3 s)` 内净进展 < `STALL_PROGRESS_DEG(0.5°)`。缺快速路径（3 s vs 0.5 s）；且摇臂只要每 3 s 蠕动 ≥0.5°（0.17°/s 缓慢爬行）就重锚清零，永不触发。
3. **速度模式无检测** — `motor_ctrl_update()` 起始 `if (ctrl->sub_mode == SUB_MODE_SPEED) return false;`，速度模式下角度堵转根本不运行。

> `STALL_DETECT_CYCLES_STARTUP(450)`、`STALL_SPEED_THRESH*` 为死宏，判定恒用 300 周期。

## 复现（`tests/host/motor/repro_embd41.c`，真实 `control_protection.c`，7/7 通过）

```
cc -std=c11 -Wall -Wextra -O1 -Itests/host/motor/stubs -Imotor_control_apm32/app/Include \
   repro_embd41.c motor_control_apm32/app/Source/control_protection.c -o repro_embd41 -lm && ./repro_embd41
```

| 场景                          | 电流堵转       | 净进展角度堵转     |
| --------------------------- | ---------- | ----------- |
| 1 阻尼器正常（冻结+0.85A）           | 0.5 s 触发 ✅ | 3.0 s 触发 ✅  |
| 2 退化+摇臂完全堵住（冻结+0.35A）       | 永不触发 ❌     | 3.0 s 触发（慢） |
| 3 退化+摇臂缓慢蠕动（≥0.5°/3s+0.35A） | 永不触发 ❌     | 永不触发 ❌      |
| 4 速度模式                      | —          | 根本不运行 ❌     |

**台架**：堵转摇臂 10–20 次退化阻尼器 → 堵住摇臂下发朝远端 MOVE → `STATUS` 查 `over=0`；摇臂完全静止约 3 s 后 `fault=STALL`，缓慢爬行则长时间无 fault。

## 修复方向

1. **新增快速角度冻结堵转（主修，纯固件）**：运动指令期间，编码器在短窗口（300–500 ms）内净进展低于小阈值（~0.1°，略高于 MT6701 抖动 0.04°）即latch `MOTOR_FAULT_STALL`，独立于电流。
2. 让该检测覆盖 `SUB_MODE_SPEED` / `SUB_MODE_TIMED`。
3. 改用“输出速率下限”判据以收敛缓慢蠕动；电流阈值维持现状作过流后备。

## 验收标准

- [ ] 台架：阻尼器退化后堵住/半堵摇臂（含缓慢蠕动）、及速度/定时模式，固件 ≤1 s 内latch stall 并停机，`FAULT_CLR` 后可恢复。
- [ ] 正常/慢速移动、校准打端点不误触发；`tests/host/motor`、`hil_motor/test_stall.py`、`test_current.py` 保持通过。
- [ ] 新增主机单测覆盖“完全堵住/缓慢蠕动/速度模式”三场景（见 `repro_embd41.c`）。
- [ ] 更新 `docs/esp32c3-apm32f103_firmware_specs.md` §9。

关联：`tests/integration/hil_motor/test_stall.py`、`tests/host/motor/test_stall_recovery.c`、GitHub xintlabs/puttreal-firmware-apm32#42。