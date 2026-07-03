
> 范围：APM32F103CB 单通道闭环电机控制节点固件 (`motor_control_apm32/`)
> 
> 主机：ESP32-C3 BLE/CAN 网关 (`ble_esp32/`)
> 
> **文档版本：v1.3** ｜ 最后更新：2026-06-17 ｜ 代码基线：`d0dccff` (PR #28)
> 
> 配套硬件规格：APM32 电机控制器 `APM32_Motor_Controller_HW_Spec_V1.2` (2026-05-27)；ESP32-C3 主机 `BT_Controller_HW_Spec_V1.0`
> 
> **环境搭建、构建、烧录、OTA 操作和测试详见 [`../README.md`](https://gemini.google.com/README.md)。** 本文档仅作为架构/协议参考。修订历史：见附录 A。

这是整个固件的综合技术参考，涵盖了两个 MCU：

- **第一部分 — APM32F103CB 从机固件** (`motor_control_apm32/`)：电机节点 — 闭环控制、校准、保护、CAN 协议、OTA。见 §1–§12。
    
- **第二部分 — ESP32-C3 主机/网关固件** (`ble_esp32/`)：BLE/UART ↔ CAN 网关 — 节点 ID 分配、命令分发、遥测数据聚合、OTA 桥接、本地 UI。见 §13–§21。
    

两者通过一条 **50 kbps 的 CAN 总线**协同工作：ESP32 是唯一的主机；APM32 从机被动执行命令。

### 系统拓扑

```
  iOS / Web 主机
       │ BLE (NUS, "VeryFun BLE") + UART0 115200 8N1
       ▼
  ┌──────────────────────┐
  │  ESP32-C3 主机/网关   │  第二部分
  │  (ble_esp32)         │
  └──────────┬───────────┘
             │ CAN 50 kbps (菊花链)
   ┌─────────┼─────────┬─────────┐
   ▼         ▼         ▼         ▼
 APM32#1  APM32#2  APM32#3 … APM32#6   第一部分
 (电机)    (电机)    (电机)     (电机)
```

# 第一部分 · APM32F103CB 从机固件

## 1. 系统概述

每个电机节点都是一块搭载 **APM32F103CBT6** 的控制板，驱动一个**蜗轮蜗杆有刷直流电机**，通过 **MT6701 绝对值磁编码器**运行闭环位置控制，并通过 **CAN 总线 (50 kbps)** 与主机/对等节点进行通信。

- 多个节点 (1–6) 以菊花链形式挂载在同一条 CAN 总线上。
    
- **ESP32-C3** 是唯一的主机：充当 BLE/UART ↔ CAN 网关，负责节点 ID 分配、命令广播、遥测数据聚合和 OTA 固件推送。
    
- 固件分为两段：一个 **16 KB 的 CAN 引导程序 (Bootloader)** + 一个 **111 KB 的应用程序 (Application)**，并且整个固件支持通过 CAN 总线进行现场升级。
    

### 1.1 关键规格

|**项目**|**值**|**来源**|
|---|---|---|
|MCU|APM32F103CBT6 (Cortex-M3 @ 72 MHz)|`README.md`, `system_apm32f10x.c`|
|Flash / RAM|128 KB / 20 KB|`ota_layout.h`|
|CAN 波特率|50 kbps|`board_driver.c`, `app_config.h`|
|控制周期|10 ms (`CONTROL_LOOP_MS`)|`app_config.h:44`|
|编码器|MT6701，14位绝对角度，软件 I²C|`board_config.h`, `sensor_mt6701.c`|
|PWM|TIM3_CH4, 5 kHz|`board_config.h:18`|
|节点 ID 范围|1–6 (由主机分配)|`app_config.h:29-30`|
|控制算法|PD + 梯形速度曲线规划 + 速度前馈|`motor_controller.c`|

## 2. 硬件与引脚映射

参见硬件规格 `APM32_Motor_Controller_HW_Spec_V1.2`（内部标签 "version V1.2", 2026-05-27; PCBA `APM32_DRV8876_V1.2`）。固件引脚定义集中在 `app/Include/board_config.h` 中。

|**功能**|**器件**|**MCU 引脚**|
|---|---|---|
|电机驱动 (PH/EN)|DRV8876|`EN`=PB1 (TIM3_CH4 PWM, 5 kHz), `PH`=PB0, `nSLEEP`=PA7, `nFAULT`=PA6|
|电流采样|DRV8876 IPROPI → R8 (2.2 kΩ)|PA0 / ADC1_IN0|
|角度反馈|MT6701 (软件位带 I²C)|`SDA`=PB10, `SCL`=PB11|
|CAN|SIT65HVD230 收发器|`RX`=PA11, `TX`=PA12|
|状态 LED|—|PB2|
|调试 UART1|WCH USB-UART|`TX`=PA9, `RX`=PA10 @ 115200 8N1|

电流转换 (`board_config.h:23-26`)：IPROPI 增益 1000 µA/A，采样电阻 2.2 kΩ，ADC 满量程 4095 = 3300 mV。

> **注意：** 在 `board_config.h` 中，`BOARD_PWM_EN_PIN` / `BOARD_PWM_PH_PIN` / `BOARD_I2C_*` 宏使用的是 `GPIO_PIN_x` 常量；端口 (GPIOA/GPIOB) 则在 `board_driver.c` 的初始化代码中绑定。修改引脚时，请在两处同时更改。

## 3. 内存布局 (128 KB Flash / 20 KB RAM)

Flash 分区是由引导程序、应用程序和上位机工具共享的唯一真相来源，定义在 `common/ota_layout.h` 中：

```
0x08000000 ┌──────────────────────────┐
           │  引导程序 (Bootloader) 16 KB│  apm32f103xb_boot.ld
0x08004000 ├──────────────────────────┤
           │  应用程序 (Application) 111 KB│ apm32f103xb_app.ld (OTA 目标区域)
           │  (0x1BC00 = 113664 B)     │
0x0801FC00 ├──────────────────────────┤
           │  校准数据 (Calibration) 1 KB │  CRC32 保护，升级时永不擦除
0x08020000 └──────────────────────────┘
```


关键的 `ota_layout.h` 宏定义：

```
#define OTA_FLASH_BASE    0x08000000UL
#define OTA_BOOT_SIZE     0x00004000UL   /* 16 KB */
#define OTA_APP_BASE      0x08004000UL
#define OTA_CALIB_BASE    0x0801FC00UL   /* 最后 1 KB */
#define OTA_APP_END       OTA_CALIB_BASE
#define OTA_APP_CAPACITY  (OTA_APP_END - OTA_APP_BASE)  /* 0x1BC00 = 111 KB */
#define OTA_FLASH_PAGE    0x00000400UL   /* 1 KB 页大小 */
#define OTA_BKP_MAGIC     0xB007u        /* app→bootloader 软复位标志，存储在BKP_DR1 */
```

### 3.1 链接器脚本

| **场景**                         | **链接器脚本**              | **Flash 范围**            |
| ------------------------------ | ---------------------- | ----------------------- |
| App (`WITH_BOOTLOADER=ON`, 默认) | `apm32f103xb_app.ld`   | 从 0x08004000 开始, 111 KB |
| App (独立运行，无 bootloader)        | `apm32f103xb_flash.ld` | 从 0x08000000 开始, 128 KB |
| 引导程序                           | `apm32f103xb_boot.ld`  | 从 0x08000000 开始, 16 KB  |

当 `WITH_BOOTLOADER=ON` 时，`system_apm32f10x.c` 会将中断向量表 (VTOR) 重定位到 `0x08004000`。

### 3.2 芯片唯一 ID 与备份寄存器

- **96位芯片 UID**：`0x1FFFF7E8 / 0x1FFFF7EC / 0x1FFFF7F0`；`node_id.c` 会将这三个字异或折叠成 32 位，在枚举期间用于区分两块不同的板子。
    
- **备份寄存器 BKP_DR**：能够在 `SYSRESETREQ` 软复位（非掉电）后存活。`BKP_DR1` 保存 `0xB007` 表示“应用程序请求停留在引导程序中”；应用程序在重启进入引导程序前，还会将其枚举的节点 ID 写入 `BKP_DR2`（见 §11.3）。

## 4. 源码结构

应用程序源码位于 `motor_control_apm32/app/Source/`，头文件在 `app/Include/`。

|**文件**|**职责**|
|---|---|
|`main.c`|启动初始化、主控制循环、校准状态机、CAN 命令分发、UART 调试输出|
|`motor_controller.c`|PD 闭环控制、梯形轨迹规划、净进度堵转检测、到达判定、故障处理|
|`can_service.c`|CAN 协议解析、主机驱动的节点枚举 (POLL/OFFER/ACK/ASSIGN)、帧分发和反馈|
|`sensor_mt6701.c`|MT6701 软件位带 I²C 驱动、原始角度 → 校准角度映射、读取错误计数|
|`board_driver.c`|GPIO/PWM(TIM3_CH4)/CAN(50 kbps) 外设初始化，电机相位和占空比控制|
|`adc_service.c`|ADC1 IPROPI 电流采样、原始值 → mA 转换|
|`calib_storage.c`|校准数据 Flash 读写 (0x0801FC00)、CRC32 校验|
|`control_protection.c`|过流 (≥1.0 A / 200 ms) 和电流堵转 (≥0.8 A + 轴无净进度 / 500 ms) 检测；每次运动命令后有 300 ms 重新触发宽限期|
|`bootloader_update.c`|运行中的 app 通过 CAN 接收并烧录 16 KB 引导区域 (CONFIG ext_cmd=0xB2)|
|`node_id.c`|读取芯片 UID，保存主机分配的 RAM 节点 ID（每次上电重新枚举）|
|`system_state.c`|全局状态机 (STOPPED/RUNNING/FAULT)、正常运行时间 (uptime)|
|`system_tick.c`|用于超时/防抖/报告定时的 1 ms SysTick|
|`sensor_error.c`|MT6701 读取错误计数及健康阈值报告|
|`system_apm32f10x.c`|时钟配置 (HSE 8 MHz → PLL 72 MHz) + VTOR 重定位|
|`test_shell.c`|可选的 UART 命令 shell (`-DBUILD_TEST_FIRMWARE=ON`，用于 HIL 测试)|
|`crc32.c`|CRC32 算法 (多项式 0xEDB88320)，校准与 OTA 共享|
|`apm32f10x_int.c` / `syscalls.c`|中断存根 / libc 存根|

共享的 OTA 协议（与 HAL 无关，由引导程序 / app / 上位机测试共享）存放在 `common/` 目录下：`ota_protocol.{c,h}`, `ota_layout.h`。

## 5. CAN 总线协议 (50 kbps)

### 5.1 波特率配置

50 kbps 由 PCLK1 = 36 MHz 派生而来 (`board_driver.c`)：

```C
/* 50 kbps: 36e6 / (45 * (1 + 13 + 2)) = 50000 */
canCfg.prescaler     = 45U;
canCfg.timeSegment1  = CAN_TIME_SEGMENT1_13;
canCfg.timeSegment2  = CAN_TIME_SEGMENT2_2;
canCfg.syncJumpWidth = CAN_SJW_1;
```

> 50 kbps 与 ESP32 主机/网络配置相匹配 (`TWAI_TIMING_CONFIG_50KBITS`)。硬件规格中提到的“默认 500 kbps”是性能说明，并非固件实际设置。

### 5.2 CAN 标识符分配

| **CAN ID**                  | **方向**    | **用途**                           |
| --------------------------- | --------- | -------------------------------- |
| `0x000` (`MASTER_CAN_ID`)   | 主机 → 总线   | 电机命令帧（所有从机解析）                    |
| `0x001`–`0x006`             | 从机 → 主机   | 状态/位置/电流反馈（按节点 ID）               |
| `0x100 + node_id`           | 从机 → 主机   | 事件帧（校准结果/限位/故障）                  |
| `0x7E0` (`ENUM_OFFER`)      | 主机 → 总线   | 向 UNSET 节点提供 ID：`data[0]`=提供的 ID |
| `0x7E1` (`ENUM_ACK`)        | 从机 → 主机   | 回复：`[0]`=ID `[1..4]`=uid32       |
| `0x7E2` (`ENUM_ASSIGN`)     | 主机 → 总线   | 冲突重分配：`[0]`=ID `[1..4]`=目标 uid32 |
| `0x7E3` (`ENUM_POLL`)       | 主机 → 总线   | 点名轮询（无负载）                        |
| `0x7DD` (`OTA_CAN_ID_REQ`)  | 主机 → 引导程序 | OTA 固件下行                         |
| `0x7DE` (`OTA_CAN_ID_RESP`) | 引导程序 → 主机 | OTA 回复上行                         |

### 5.3 命令帧结构

`can_service.h` 中的 `CanCommandFrame_t`：

```C
typedef struct {
    uint8_t  base_mode;        /* 0x00=QUERY 0x01=CONTROL 0x02=CONFIG 0x03=ESTOP */
    uint8_t  sub_mode;         /* 0=SPEED 1=POSITION 2=TIMED */
    uint8_t  target_id;        /* 0x00=广播, 0x01–0x06=单节点 */
    float    target_angle;     /* 目标角度，度 */
    int16_t  value;            /* 速度 (PWM 0–10000) 或时间 (ms) */
    uint16_t config_pwm;       /* 最小 PWM 阈值 */
    uint8_t  config_ext_cmd;   /* 扩展命令，见下文 */
    uint8_t  config_timeout_s;
    uint8_t  config_report_s;
} CanCommandFrame_t;
```

**基础模式** (`app_config.h:90-93`)：`CAN_CMD_QUERY=0x00`，`CAN_CMD_CONTROL=0x01`，`CAN_CMD_CONFIG=0x02`，`CAN_CMD_ESTOP=0x03`。

**子模式** (`app_config.h:153-155`)：`SUB_MODE_SPEED=0` (开环 PWM)，`SUB_MODE_POSITION=1` (闭环位置)，`SUB_MODE_TIMED=2` (时间约束到达)。

**CONFIG 扩展命令** (`config_ext_cmd` 字段)：

|**宏定义**|**值**|**含义**|
|---|---|---|
|`CAN_CONFIG_CMD_CALIBRATE`|`0xA5`|触发端到端校准扫描|
|`CAN_EVENT_CMD_CALIBRATION_RESULT`|`0xA6`|(事件) 校准结果报告|
|`CAN_EVENT_CMD_CALIBRATION_LIMIT`|`0xA7`|(事件) 限位报告|
|`CAN_CONFIG_CMD_REBOOT_BOOTLOADER`|`0xB0`|设置 BKP 标志并软复位进入引导程序（更新 app 区域）|
|`CAN_CONFIG_CMD_UPDATE_BOOTLOADER`|`0xB2`|让运行中的 app 进入“接收新引导程序”模式（更新 boot 区域）|

> `0xB1` (旧的 `RESET_NODE_ID`) 现已弃用/保留：节点 ID 不再存储在 Flash 中，每次上电都会重新枚举，因此重排菊花链顺序只需重新上电即可。

## 6. 节点 ID 枚举 (主机驱动)

> ⚠️ 以 `app_config.h` 为准。README 中“通过上电顺序自我分配并持久化到 Flash”的描述是**旧方案**，现已被主机驱动方案取代（见 git 日志 `1d25a70 feat: use master-driven ID assignment scheme`）。

机制 (`app_config.h:11-35`, `can_service.c`)：

- 从机上电后处于 **UNSET (0)** 状态，**不响应电机命令**，并且 ID 仅保存在 RAM 中。
    
- 主机广播 **`ENUM_POLL` (0x7E3)** 进行点名 → 已分配的从机回复 **`ENUM_ACK`(id, uid)**。
    
- 主机广播 **`ENUM_OFFER` (0x7E0, data[0]=id)** → 处于 UNSET 状态的从机会临时接受该 ID 并回复 ACK。
    
- 当发生 ID 冲突时，主机广播 **`ENUM_ASSIGN` (0x7E2, [0]=新ID, [1..4]=目标uid32)** 进行定向重分配。
    
- **退避机制**：在收到 POLL/OFFER 后，从机会等待 `(uid XOR s_ackSeq) % NODE_ID_ENUM_ACK_SPREAD_MS`（一个 48 ms 的窗口）后再回复，以错开总线冲突；残余冲突由主机通过 ASSIGN 解决。
    

此方案不依赖上电顺序，因此对绕过 PMOS 上电级联的软复位（如 OTA）具有鲁棒性。`ENUM_ACK` 携带 32 位芯片 UID，因此主机可以区分两块不同的板子。已通过硬件验证：干净的 1–6 枚举（修复 ACK 退避哈希后）。

引导程序后备方案：如果 app 从未运行过（如恢复场景），引导程序默认使用 `MY_MOTOR_ID`(1)；广播 OTA（节点=0xFF）仍然可以触达它。

## 7. MT6701 磁编码器

一款绝对值磁编码器，14位角度（0–16383 count = 0–360°），通过**软件位带 I²C**读取 (`sensor_mt6701.c`)。

**寄存器/地址** (`board_config.h:20-22`)：7位地址 `0x06`，角度高字节 `0x03` (位13:6)，低字节 `0x04` (位5:0)。引脚 `SDA`=PB10, `SCL`=PB11 (开漏，板载上拉)。

**读取序列**：START → 写地址(0x06,W) → 写角度寄存器 → STOP → START → 读 2 字节 → STOP；原始角度 = `(ANGLE_H<<8)|ANGLE_L` 移位至 14 位值。

**角度映射**：校准操作会存储 `raw_at_min_deg` / `raw_at_max_deg`；线性插值将原始计数映射到工作窗口中。发生读取失败时错误计数累加；超过 `SENSOR_ERROR_THRESHOLD=3` (`app_config.h:87`) 后将报告 `MOTOR_STATUS_SENSOR_ERROR`。

### 7.1 角度窗口 (`app_config.h:50-57`)

```
ANGLE_CALIB_MIN_DEG = 0.0°      校准原始下界
ANGLE_CALIB_MAX_DEG = 75.0°     校准原始上界
ANGLE_LIMIT_MARGIN_REV_DEG = 1.5°   反向安全裕度
ANGLE_LIMIT_MARGIN_FWD_DEG = 3.5°   正向安全裕度
→ 工作窗口 ANGLE_OP_MIN ≈ 1.5°, ANGLE_OP_MAX ≈ 71.5°
```

## 8. 电机控制

### 8.1 控制模式

- **SUB_MODE_SPEED**：开环 PWM。
    
- **SUB_MODE_POSITION**：PD 闭环控制至目标角度。
    
- **SUB_MODE_TIMED**：在给定毫秒数内到达目标角度（带有时间约束的轨迹规划）。
    

### 8.2 PD 控制器 + 速度前馈

`motor_controller.c` 输出：

```c
output = kp * error + kd * derivative + TRAJ_VEL_FF_GAIN * profile_velocity;
```

参数 (`app_config.h:65-85`)：

|**参数**|**默认值**|**备注**|
|---|---|---|
|`PD_KP_DEFAULT`|500.0|比例增益|
|`PD_KD_DEFAULT`|50.0|微分增益（误差的一阶差分，无滤波）|
|`TRAJ_VEL_FF_GAIN`|8.0|速度前馈系数|
|`TRAJ_MAX_VEL_DPS`|120.0|轨迹最大速度 (deg/s)|
|`TRAJ_MAX_ACCEL_DPS2`|360.0|轨迹最大加速度 (deg/s²)|
|`DEFAULT_MIN_PWM`|3500|启动最小 PWM（克服静摩擦，10000的35%）|
|`POSITION_TOLERANCE`|1.5°|到达公差|
|`POSITION_ARRIVE_DEADBAND`|0.3°|到达死区|

**启动阶段 (`app_config.h:72-77`)** — 分两步克服静摩擦 (`motor_controller.c`)：

1. **踢击脉冲 (Kick pulse)**：在新目标的最初 `STARTUP_KICK_MS=30 ms`（3个周期）内，施加高 PWM `STARTUP_KICK_PWM=6000`（方向跟随 PD 输出符号）强制突破静摩擦。
    
2. **爬升 (Ramp-up)**：踢击结束后，从 `STARTUP_INITIAL_PWM=2500` 开始，每 10 ms 增加 `STARTUP_PWM_STEP=300`，取该值与 PD 输出中的较大者，直到经过 `STARTUP_PHASE_MS=200` 并且退出启动阶段。
    

> 踢击脉冲会使电流瞬间出现尖峰，因此在启动阶段（踢击 + 爬升）会跳过电流保护，以避免误触发过流/电流堵转（见 §9.2）。控制器结构体中增加了一个 `kick_counter` 来计算踢击周期。

### 8.3 PWM 生成

`board_driver.c`：TIM3_CH4 (PB1)，5 kHz (`BOARD_PWM_FREQUENCY_HZ`)。`PH`(PB0) 是方向位；占空比通过 `pwm / PWM_SPEED_LIMIT(10000)` 映射到定时器计数值。

## 9. 保护机制

### 9.1 净进度堵转检测（主要保护）

`motor_controller.c`。**已从“每周期速度门限”更改为“净进度累加”**：旧逻辑对比每周期角度变化 `|now - last|` 与速度阈值，这会在合法慢速运动（例如从校准端点突破静摩擦时）产生误触发。新逻辑追踪相对于锚点的**净进度** — 只要转轴在 `STALL_DETECT_CYCLES` 内累积了 `STALL_PROGRESS_DEG` 的位移，锚点就会重置并清空计数；只有持续缺乏进展才会被判定为堵转。

| **参数**                        | **值**              | **备注**        |
| ----------------------------- | ------------------ | ------------- |
| `STALL_DETECT_CYCLES`         | 300 (×10 ms = 3 s) | 净进度检测窗口 (周期)  |
| `STALL_DETECT_CYCLES_STARTUP` | 450 (4.5 s)        | 启动阶段窗口 (周期)   |
| `STALL_PROGRESS_DEG`          | 0.5°               | 窗口内必须累积的最小净进度 |

判定条件（仅在启动阶段之外以及 `|位置误差| > POSITION_TOLERANCE` 时累加）：距离锚点的净进度 `|now - anchor| ≥ STALL_PROGRESS_DEG` → 重置锚点，清除计数；否则计数递增，超过 `STALL_DETECT_CYCLES` 后锁存 `MOTOR_FAULT_STALL`。控制器结构体增加了 `stall_anchor_angle` 来保存锚点角度；新目标、故障清除和到达都会重置锚点。

> 旧宏 `STALL_SPEED_THRESH` / `STALL_SPEED_THRESH_STARTUP` 依然在 `app_config.h` 中定义，但不再被新逻辑使用。

><font color="#2DC26B"> 注：2026.7.3记录：</font>当前代码中没有`STALL_DETECT_CYCLES`和`STALL_DETECT_CYCLES_STARTUP`，起步宽限期以`s_motionGraceMs = STALL_RECOVERY_GRACE_MS(300ms)`定义。净进度检测中以堵转计数`> STALL_CURRENT_TRIP_MS(500ms)`后锁存。净进度步长为`STALL_CURRENT_PROGRESS_CENTIDEG (20 -> 0.2°)`。锚点角度为静态c

### 9.2 基于电流的保护（
粗略后备）

`control_protection.c` (`app_config.h:135-151`)：

- 过流：≥ `OVERCURRENT_LIMIT_MA`(1000 mA) 持续 `OVERCURRENT_TRIP_MS`(200 ms)。
    
- 电流堵转：≥ `STALL_CURRENT_LIMIT_MA`(800 mA) **并且轴无净进度**，持续 `STALL_CURRENT_TRIP_MS`(500 ms)。
    
- 仅在**校准之外且启动阶段之外**处于激活状态（启动踢击会产生电流尖峰；`main.c` 在启动阶段只会重新锚定，绝不累加）。
    

电流堵转同样从“每周期整度数比较”转移到了**百分之一度 (centi-degree) 净进度**追踪：角度以 centi-degrees (×100) 传入，转轴必须累积 `STALL_CURRENT_PROGRESS_CENTIDEG=20` (0.20°, 约 0.4°/s 的下限，高于 MT6701 的 ~0.04° 抖动) 才能重置锚点；否则会累加到 `STALL_CURRENT_TRIP_MS` 并被判定为堵转。

**每次命令的重触发宽限期** (`ControlProtectionOnMotionCommand`)：每收到一次运动命令，都会清除上一次命令积累的电流堵转窗口，并授予一个 `STALL_RECOVERY_GRACE_MS=300 ms` 的带锚定宽限期 — 从静止状态重新启动的电机（通常是在收到新命令清除堵转故障后）在突破静摩擦时会汲取接近堵转的电流，宽限期可防止在此陈旧窗口上立即再次触发（即“堵转节点永远无法接受新命令”的故障）。该重置**仅针对运动帧**触发；QUERY/ESTOP/CONFIG 帧不会触发它，并且过流（硬性电气限制）不受影响。

> **硬件测试结果 (2026-06)**：在这款蜗轮电机上，运行/加速电流峰值 (<0.8 A) 与堵转电流 (~0.7–0.8 A) 有重叠，因此**不存在**既能触发真堵转又能避开正常运动的完美电流阈值。因此，**净进度堵转是主要保护手段**，而电流限制仅保持 HW Spec V1.2 的值 (过流 1.0 A，堵转 0.8 A / 500 ms) 作为粗略的过流后备。

### 9.3 状态与故障码

|**状态码 (app_config.h:113-117)**|**值**|**故障码 (:119-123)**|**值**|
|---|---|---|---|
|`MOTOR_STATUS_IDLE`|0|`MOTOR_FAULT_NONE`|0|
|`MOTOR_STATUS_RUNNING`|1|`MOTOR_FAULT_STALL`|1|
|`MOTOR_STATUS_ARRIVED`|2|`MOTOR_FAULT_TIMEOUT`|2|
|`MOTOR_STATUS_ERROR`|3|`MOTOR_FAULT_SENSOR`|3|
|`MOTOR_STATUS_SENSOR_ERROR`|4|`MOTOR_FAULT_OVERCURRENT`|4|

故障会被锁存并通过 CAN 报告；恢复需要急停 / 故障清除命令。