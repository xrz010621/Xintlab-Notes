
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

| **项目**      | **值**                              | **来源**                              |
| ----------- | ---------------------------------- | ----------------------------------- |
| MCU         | APM32F103CBT6 (Cortex-M3 @ 72 MHz) | `README.md`, `system_apm32f10x.c`   |
| Flash / RAM | 128 KB / 20 KB                     | `ota_layout.h`                      |
| CAN 波特率     | 50 kbps                            | `board_driver.c`, `app_config.h`    |
| 控制周期        | 10 ms (`CONTROL_LOOP_MS`)          | `app_config.h:44`                   |
| 编码器         | MT6701，14位绝对角度，软件 I²C              | `board_config.h`, `sensor_mt6701.c` |
| PWM         | TIM3_CH4, 5 kHz                    | `board_config.h:18`                 |
| 节点 ID 范围    | 1–6 (由主机分配)                        | `app_config.h:29-30`                |
| 控制算法        | PD + 梯形速度曲线规划 + 速度前馈               | `motor_controller.c`                |

## 2. 硬件与引脚映射

参见硬件规格 `APM32_Motor_Controller_HW_Spec_V1.2`（内部标签 "version V1.2", 2026-05-27; PCBA `APM32_DRV8876_V1.2`）。固件引脚定义集中在 `app/Include/board_config.h` 中。

| **功能**       | **器件**                       | **MCU 引脚**                                                           |
| ------------ | ---------------------------- | -------------------------------------------------------------------- |
| 电机驱动 (PH/EN) | DRV8876                      | `EN`=PB1 (TIM3_CH4 PWM, 5 kHz), `PH`=PB0, `nSLEEP`=PA7, `nFAULT`=PA6 |
| 电流采样         | DRV8876 IPROPI → R8 (2.2 kΩ) | PA0 / ADC1_IN0                                                       |
| 角度反馈         | MT6701 (软件位带 I²C)            | `SDA`=PB10, `SCL`=PB11                                               |
| CAN          | SIT65HVD230 收发器              | `RX`=PA11, `TX`=PA12                                                 |
| 状态 LED       | —                            | PB2                                                                  |
| 调试 UART1     | WCH USB-UART                 | `TX`=PA9, `RX`=PA10 @ 115200 8N1                                     |

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


### 9.2 基于电流的保护（粗略后备）

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

## 10. 自动校准

端到端的机械限位扫描能够建立从原始编码器角度 → 工作窗口的映射，并持久化到 Flash 的最后 1 KB (经过 CRC 校验)。触发条件：启动时如果未存储有效校准；或者随时通过 CAN 总线下发 `CONFIG ext_cmd=0xA5`（可通过 `target_id=0` 广播以同时重新校准所有节点）。ESP32 主机在收到上位机 `CALIB` / `{"action":"calibrate"}` 命令时下发此指令，并将各节点的限位/结果事件帧以 JSON 格式转发。

### 10.1 校准状态机 (`main.c`，参数见 `app_config.h:60-63`)

```
IDLE
 └─(0xA5 / UART CALIB)→ 急停 + 清除角度映射 → FORWARD_SETTLE (pwm=3000)
      FORWARD_SETTLE   正向驱动，稳定 CALIBRATION_SETTLE_CYCLES=30 周期 (300 ms)
      FORWARD_SEARCH   保持正向，编码器稳定 CALIBRATION_STABLE_CYCLES=80 周期 (800 ms)
                       → 记录 raw_at_max_deg
      REVERSE_SETTLE   反向驱动，稳定 30 周期
      REVERSE_SEARCH   保持反向，稳定 → 记录 raw_at_min_deg
      DONE             计算跨度 → 存储 Flash @0x0801FC00 → 发送 0xA6 结果事件 → IDLE
      (超时/错误)       CalibrationFail → IDLE → 发送错误事件
```

稳定标准：原始计数变化 < `CALIBRATION_RAW_STABLE_THRESHOLD=2` counts；超时上限 `CALIBRATION_MAX_CYCLES=1200` 周期。

### 10.2 校准存储

`calib_storage.h` 结构体存储在 `0x0801FC00`，包含 `raw_at_min_deg`/`raw_at_max_deg`/`mapped_min_deg`/`mapped_max_deg` + CRC32 (多项式 0xEDB88320)。该页在应用程序/引导程序升级期间永远不会被擦除；CRC 校验失败会强制在下次启动时重新校准。

> **方向锚定（非原始值锚定）**：反向端点（通过 −PWM 到达）始终映射为角度 0（高度 0），而正向端点（通过 +PWM 到达）始终映射为最大值（高度 254）。`Mt6701SetAngleMap` 携带**带符号的**原始跨度，因此无论编码器/磁铁在组装时是 `forward_raw > reverse_raw` 还是相反，"增加高度" 始终对应相同的物理方向（转子向外）。两种原始大小顺序均被上位机单元测试覆盖。

## 11. 固件更新 (通过 CAN 进行 OTA)

APM32 从机应用程序 — 乃至引导程序自身 — 均可通过从 ESP32-C3 主机的 UART 驱动下**通过 CAN 总线**进行现场升级；一旦安装了引导程序，就不再需要 SWD 烧录器。本节是架构 / 线路协议参考。有关烧录单个主板或整个总线的**操作手册**详见 README。

> **状态 / 验证**：协议状态机已被上位机单元测试覆盖 (`tests/host/motor`，包括一个 16 KB 的 boot 区域后端)。引导程序 (5.7 KB / 16 KB) 和重新链接的 app (18 KB / 111 KB) 可以成功构建并容纳。**app 区域** CAN 升级 (阶段 A) 已经在所有 6 个节点上经过硬件验证。**通过 CAN 的引导程序升级** (阶段 B / `OTABL`) 是**全新且尚未经过硬件验证**的 — 在依赖它之前，请务必在一块连接了稳定电源和 SWD 探针的备用板上验证阶段 B。

### 11.1 Flash 布局与组件

| **区域** | **地址**       | **大小** | **内容**                          |
| ------ | ------------ | ------ | ------------------------------- |
| 引导程序   | `0x08000000` | 16 KB  | CAN 接收器；可通过 app 经 CAN 更新 (阶段 B) |
| 应用程序   | `0x08004000` | 111 KB | 通过 CAN 传输的镜像文件                  |
| 校准     | `0x0801FC00` | 1 KB   | MT6701 映射，在更新期间**保留**           |

两个代码区域都可以通过 CAN 现场升级；只有第一次安装（或变砖恢复）需要 SWD — 完全空白的芯片没有可运行以接收 CAN 帧的代码。唯一真实来源：`common/ota_layout.h`。

|**部分**|**路径**|**角色**|
|---|---|---|
|协议状态机|`common/ota_protocol.{c,h}`|纯C语言，与 HAL 无关的接收逻辑 + CRC32；由引导程序、app 端的引导程序更新器和上位机测试共享|
|引导程序|`motor_control_apm32/bootloader/`|将协议连接到 FMC flash + CAN；在启动时决定是进 app 还是升级；写入 **app** 区域|
|App 钩子|`motor_control_apm32/app` (`WITH_BOOTLOADER`)|链接在 `0x08004000`，设置 `VTOR`，处理重启到引导程序的 CAN 命令（并将枚举的节点 ID 交给 `BKP_DR2` 中的引导程序）|
|App端引导程序更新器|`app/Source/bootloader_update.c`|指向 **boot** 区域后端的相同协议状态机；由 CONFIG `0xB2` 进入，写入 `0x08000000`|
|主机桥接|`ble_esp32/main/app/ota.c`|将 UART 上的 XMODEM-1K 桥接到 CAN OTA 帧|
|上位机工具|`motor_control_apm32/tools/ota_flash.py`|向主机发送头信息 + XMODEM-1K 镜像|

### 11.2 启动决策 (`bl_main.c`)

复位时引导程序执行：

1. 读取 **BKP_DR1**。如果包含 `0xB007` (app 请求更新)，它会清除该标志并停留在引导程序中等待传输。BKP 能在软复位后存活，因此触发是可靠的。
    
2. 否则检查 app 的复位向量 (`0x08004000`：初始 SP 在 RAM 中，复位 PC 在 app 区域内且设置了 Thumb 位)。如果**无效**（例如未完成的更新），它将停留在引导程序中 — 损坏的镜像总是回到这里以便恢复。
    
3. 否则监听约 300 ms 以获取任何 OTA 帧（允许主机强制处于健康状态的节点进入恢复模式），如果没有到达，则**跳转到 app**。
    

快速闪烁的 LED（PB2，~5 Hz）表示引导程序正在运行/等待。

### 11.3 CAN 线路协议 (`common/ota_protocol.h`)

请求 (主机→从机) 位于标准 ID **`0x7DD`** 上；响应在 **`0x7DE`** 上 — 远离电机协议。CRC-32 = 多项式 `0xEDB88320` (与校准存储相同)。上位机→主机的链路是通过 UART 传输 XMODEM-1K；主机将其桥接到 CAN 帧。

```
#define OTA_OP_START 0xA0   /* [1]=node [2..5]=size LE32 */
#define OTA_OP_DATA  0xA1   /* [1]=seq8 [2..7]=≤6 数据字节 */
#define OTA_OP_END   0xA2   /* [1..4]=crc32 LE32 */
#define OTA_OP_GO    0xA3   /* 提交并跳转 */
#define OTA_OP_PING  0xA4   /* [1]=node */
#define OTA_ST_ACK   0x50   /* [1]=op [2]=next_seq */
#define OTA_ST_NAK   0x51   /* [1]=op [2]=err [3]=expected_seq */
#define OTA_ST_PONG  0x52   /* [1]=node [2]=ver [3]=state */
#define OTA_ST_DONE  0x53   /* 传输成功 */
```

- **停等协议 (Stop-and-wait)**：主机发送一帧 DATA 并等待其 ACK，然后再发下一帧。
    
- **8位滚动序列**：丢帧 → 超时 → 重发；重复帧（丢失 ACK）会被重新回复 ACK，但不会重新写入 flash。
    
- **CRC32**：在 END 时全镜像校验 (多项式 0xEDB88320)；不匹配将强制重新开始 START (重新擦除) 然后才能重试。
    
- Flash 后端按 1 KB 页擦除并按 32 位字对齐编程 (`FMC_ErasePage` / `FMC_ProgramWord`，被 `FMC_Unlock`/`Lock` 包裹)；部分最后的不满字使用 `0xFF` 填充。
    
- 每次只有一个节点进行更新：START 寻址特定节点 ID，因此其他通电的节点会忽略该会话，从而避免 ACK 冲突。
    

引导程序知道自己的节点 ID，因为应用程序在重启进入引导程序前会将其运行时枚举的 ID 写入 `BKP_DR2`（该 ID 不再保存在 Flash 中）。从未运行过应用程序的控制板 `BKP_DR2` 中没有 ID，它将回退使用 `MY_MOTOR_ID`；广播 OTA (`node = 0xFF`) 依然能够触达它。

### 11.4 更新完整固件 (Bootloader + App)

引导程序无法覆盖自己 — 芯片不能擦除其正在执行的页面。因此，通过 CAN 更新**整个**固件需要分两个阶段进行，每个阶段的写入程序绝不会接触自己的区域：

|**阶段**|**写入程序**|**写入区域**|**进入方式**|**主机头信息**|
|---|---|---|---|---|
|**A (app)**|引导程序|app 区域 `0x08004000` (111 KB)|重启进入引导程序 (CONFIG `0xB0`)|`OTA,<node>,<size>,<crc>`|
|**B (bootloader)**|运行中的 app|boot 区域 `0x08000000` (16 KB)|app 引导程序接收模式 (CONFIG `0xB2`) → `bootloader_update.c`|`OTABL,<node>,<size>,<crc>`|

阶段 B 复用了完全相同的 `ota_protocol.c` 状态机以及相同的 `0x7DD`/`0x7DE` ID — 唯一不同的是 flash 后端（它指向 16 KB 的 boot 区域，并在 START 时进行了容量检查）。运行中的 app 已经知道了其枚举的节点 ID，因此不需要像引导程序那样通过 `BKP_DR2` 移交，就能实现按节点定向。校准页始终被保留。操作员工具始终**先刷入 app**（处于安全状态），然后再刷入引导程序。

> **⚠️ 变砖风险 — 仅限阶段 B。** 从阶段 B 的 START 擦除 boot 区域那一刻起，直到新镜像被验证和提交，节点上没有有效的引导程序；如果在此以秒计的窗口内断电，复位向量将被擦除 → 下次复位时产生 HardFault → **只能通过 SWD 恢复**。内置缓解措施：写入程序是 app（_失败的传输_仍会让节点运行 app，主机可以重试 — 只有物理断电才会变砖），复位前会对新 boot 向量进行有效性检查，并且在 START 时拒绝超大镜像。操作建议：在阶段 B 期间保持电源稳定，只发送已知完好的引导程序 `.bin`，并且在节点被重新刷写完成之前，千万不要对其正在中断的阶段 B 进电循环。

### 11.5 恢复

- app 区域无效/写了一半的节点会**停留在引导程序中**并不断响应 PING，因此重新运行 app 更新即可恢复它 — 不需要 SWD。
    
- **未掉电的阶段 B 传输失败**会让节点保持在运行其 app 的状态，因此通过 CAN 重新运行引导程序步骤即可恢复。只有在写入 boot 区域_期间_掉电才需要 SWD。
    
- 如果丢失了引导程序（损坏或在阶段 B 中途掉电），请通过 SWD 运行 `tools/flash_bootloader_and_app.command` 进行重装。
    
- 1 KB 的校准页在更新期间绝不被擦除，因此校准数据得以保留。
    

> 已知台架测试问题：台架测试装置上出现过可重现的 "block 8" OTA 阻断（上位机 ACK 超时已提高至 3 s，UART 环形缓冲区扩大至 4 KB；待重新测试）。在台架上，请通过 SWD 而不是 OTA 烧录从机。

## 12. 主程序流程

### 12.1 启动序列 (`main.c`)


```
SystemClockConfig()   HSE 8 MHz → PLL 72 MHz
NodeIdInit()          读取芯片 UID, 状态=UNSET
BoardGpioInit()       电机/CAN/I²C/LED/UART 引脚
BoardCanInit()        CAN 50 kbps, 过滤主机命令 + 枚举帧
BoardPwmInit()        TIM3_CH4 @ 5 kHz
Mt6701Init()          软件 I²C 初始化
CalibStorageLoad()    从 Flash 加载角度映射 (或标记无效)
motor_ctrl_init()     设置 PD 增益, 清除故障
SystemStateSet(STOPPED)
```

### 12.2 主控制循环 (10 ms 周期)

```
1. CanServicePoll()                        处理枚举回复
2. while CanServiceReceiveCommand(&cmd)    排空 RX FIFO
       ProcessCanCommand(&cmd)             分发 QUERY/CONTROL/CONFIG/ESTOP
3. nowAngle = motor_ctrl_get_angle()       读取 MT6701, 更新读取错误计数
4. if calibrating: CalibrationUpdate()     推进校准状态机
5. ControlProtectionUpdate(curMa, angleCenti)  过流/电流堵转 (校准及启动阶段外；角度单位为厘度 ×100)
6. if fault: motor_ctrl_force_fault()
7. motor_ctrl_update(&s_motor, nowAngle)   PD 闭环 + 净进度堵转 + 到达判定
8. if 状态改变: CanServiceSendFeedback()  反馈给主机
9. 报告定时器触发 report_ms → CanServiceSendFeedback()
```

定时常量：`CONTROL_LOOP_MS=10`，`DEFAULT_REPORT_MS=1000`，`DEFAULT_TIMEOUT_MS=6000` (`app_config.h:44-46`)。

> 本固件的构建、烧录和 HIL 测试指令详见 [README](https://gemini.google.com/README.md)。

# 第二部分 · ESP32-C3 主机/网关固件

> 范围：ESP32-C3 BLE/CAN 网关固件 (`ble_esp32/`)
> 
> 构建框架：ESP-IDF 6.0.1

## 13. 主机概述

`ble_esp32/` 是位于 **ESP32-C3-MINI-1-H4** (RISC-V, 4 MB Flash, 原生 USB) 上的 BLE/CAN 网关固件，具有**无本地电机、无 MT6701**的特点 — 承担纯粹的监督/转发角色。主要职责：

- 接收来自 iOS/Web 主机的命令，通过 **BLE (NUS 协议) + UART0 (115200 8N1)** 进行通信；
    
- 验证后，将其通过 **CAN (50 kbps)** 转发给 APM32 从机；
    
- 聚合从机的反馈，并作为 **JSON** 通过 BLE 和 UART 返回；
    
- 桥接固件 OTA（上位机 UART XMODEM-1K ↔ 从机引导程序 CAN 帧）；
    
- 驱动本地 UI：1 颗板载 WS2816C 状态 LED + 2 条外部 WS2812 灯带 + 3 个按钮。
    

> **C3-MINI-1 提示**：GPIO11–17 已与内部 SPI Flash 绑定，**不能作为 I/O 使用**。

### 13.1 关键规格

|**项目**|**值**|**来源**|
|---|---|---|
|芯片|ESP32-C3-MINI-1-H4|`board_config.h:8`|
|ESP-IDF|6.0.1|`sdkconfig`|
|CAN|TX=IO7, RX=IO6, 50 kbps|`board_config.h:24,27`, `hal_can_esp32.c`|
|UART|UART0, 115200 8N1, 4 KB RX 环形缓冲|`board_config.h:17`, `hal_uart_esp32.c`|
|BLE 设备名|`"VeryFun BLE"`|`sdkconfig`|
|BLE 服务/特征值|0xFFE0 / TX 0xFFE1 (Notify) / RX 0xFFE2 (Write)|`hal_ble_esp32.c:37-39`|
|控制循环|10 ms|`app_config.h:80`|
|状态 LED|WS2816C @ IO0 (SPI2_HOST 后端)|`board_config.h:45,48`|
|灯带|2×WS2812, 每条6颗LED, IO4(RMT1)/IO5(RMT2)|`board_config.h:75,78,81`|
|按钮|BUT1=IO3, BUT2=IO10, BOOT=IO9 (低有效上拉)|`board_config.h:55,58,68`|

## 14. 三层架构

代码被组织为三层 — **config / hal / app** — 以便移植到其他芯片（只需交换 HAL 实现，保留 app 和 `hal_*.h` 接口即可）：

```
config/   board_config.h (引脚/外设参数), app_config.h (控制常量/CAN IDs)
   │
app/      与芯片无关的应用逻辑
   │  gateway · ble_message · buttons · indicator · lightshow
   │  height_map · config_mgr · enum_logic · enum_manager · ota
   │  (调用 hal_* 接口)
hal/      硬件抽象接口 hal_*.h  +  ESP32 实现 hal_*_esp32.c
          NimBLE / TWAI / UART / led_strip(RMT,SPI) / GPIO / NVS / timer
   │
main.c    启动初始化、任务、10 ms 主循环
```

### 14.1 源文件清单

**app/ (应用逻辑):**

|**文件**|**职责**|
|---|---|
|`gateway.c/.h`|命令解析、顺序电机队列执行、CAN 收发、校准/枚举处理（核心）|
|`ble_message.c/.h`|JSON 双通道发送 (同时至 UART0 + BLE)，256 B 内部缓冲|
|`buttons.c/.h`|3 按钮防抖及短按/长按检测 (BOOT 长按 1.5 s 触发全总线校准)|
|`indicator.c/.h`|板载状态 LED 颜色/闪烁状态机 (红=错误，蓝=收到命令，绿=CAN活动，闪烁=校准中)|
|`lightshow.c/.h`|每条灯带 5 种模式 (关闭→常亮→闪烁→呼吸→彩虹)|
|`height_map.c/.h`|角度 ↔ 高度 (0–254) 正弦映射 (工作窗口 1.5°–71.5°)|
|`config_mgr.c/.h`|运行时配置持久化 (NVS)，启动加载，收到 CFG 命令后向从机重播|
|`enum_logic.c/.h`|纯枚举逻辑 (上位机端可单元测试)：折叠 ACK 观察、检测冲突/丢失、输出 ASSIGN|
|`enum_manager.c/.h`|枚举驱动器：定期 POLL，接收 ACK，调用逻辑，广播 ASSIGN|
|`ota.c/.h`|XMODEM-1K (主机 UART) → CAN 引导程序 (0x7DD/0x7DE) 桥接|

**hal/ (硬件抽象 + ESP32 实现):**

|**文件**|**职责**|
|---|---|
|`hal_ble_esp32.c`|NimBLE GATT 服务 (NUS UUID)，MTU 512，单连接|
|`hal_can_esp32.c`|TWAI 驱动 (50 kbps)，bus-off 恢复，收发计数器|
|`hal_uart_esp32.c`|UART0 115200，4 KB RX 环形缓冲 (可容纳 4× XMODEM-1K 数据包用于 OTA 重传)|
|`hal_led_esp32.c`|板载 WS2816C (SPI2 后端，避免占用灯带的 RMT 通道)，GRB|
|`hal_strip_esp32.c`|2× WS2812 (RMT @ 10 MHz, 48 mem_block_symbols)|
|`hal_button_esp32.c`|GPIO 输入轮询 (低有效，软件防抖)|
|`hal_storage_esp32.c`|NVS 键值存储|
|`hal_system_esp32.c`|延时，正常运行时间，系统滴答|

`main.c` 任务：`uart_rx_task` (按行累加上位机命令 → `gateway_process_line()`)，`ble_rx_callback` (BLE 写入累加至换行)，`diagnostics_task` (每 1 s 打印 SYS/BLE/CAN 状态，在 OTA 期间静默)，以及主循环 (10 ms: CAN 健康检查 → 排空 RX → 推进电机队列 → 指示灯/按钮/灯光秀更新 → 周期报告)。

## 15. BLE 接口 (NimBLE GATT Server)

主机作为一个 BLE 外设 (GATT Server)，使用 **Nordic UART Service (NUS)** 协议以保证 iOS 兼容性 (`hal_ble_esp32.c`)：

|**项目**|**值**|
|---|---|
|设备名|`"VeryFun BLE"`|
|主服务 UUID|`0xFFE0`|
|TX 特征值 `0xFFE1`|NOTIFY (主机 → 上位机)|
|RX 特征值 `0xFFE2`|WRITE / WRITE_NO_RSP (上位机 → 主机)|
|MTU|512 字节|
|最大连接数|1 (断开连接后自动重新广播)|

数据流：上位机向 RX 特征值写入 → `ble_rx_callback` 累加至换行 → `gateway_process_line()`；主机通过 `ble_message_send()` → TX 特征值 Notify 进行回复。

> 初始化鲁棒性：在外设初始化阶段如果 BLE 失败被设定为非致命错误，以确保 BLE 始终能启动（避免出现“找不到 VeryFun BLE”）。

## 16. CAN/TWAI 与帧映射

**引脚/波特率**：TX=IO7, RX=IO6, `TWAI_TIMING_CONFIG_50KBITS()`, TX 队列 20, RX 队列 50 (`board_config.h`, `hal_can_esp32.c`)。健康管理：自动 bus-off 恢复 (`twai_initiate_recovery()`)，维护 TEC/REC 及 tx_ok/tx_fail/rx_count 计数器用于诊断。

主机通过 `gateway_can_send_checked(id, data, len, retries, tag)` 发送所有帧，失败时会作为 JSON 报告。主机**主动发送**至 `MASTER_CAN_ID(0x000)` 的帧（`data[0]` 是命令字节）：

|**命令**|**data[0]**|**有效载荷**|**来源**|
|---|---|---|---|
|运动 (每个电机)|`0x05`|`[1]`=motor_id, `[2:3]`=pos_raw (大端，角度 0–75° 归一化为 0–65535), `[4:5]`=speed (大端)|`gateway.c` `start_next_motor()`|
|配置广播|`0x02`|`[2:3]`=min_pwm, `[6]`=timeout_s, `[7]`=report_s|`gateway.c` `handle_cfg()`|
|校准广播|`0x02`|`[4]`=`CAN_CONFIG_CMD_CALIBRATE(0xA5)`|`gateway.c:189-208`|
|紧急停止|`0x03`|—|`gateway.c` `handle_estop()`|

> 注意：从机端的基础模式在 §5.3 中定义 (QUERY/CONTROL/CONFIG/ESTOP = 0x00–0x03)。运动帧的 `data[0]=0x05` 是主机用于“移动到目标位置”的命令字节，由从机的位置控制路径执行。

> **校准广播前的枚举静默**：在发送 CALIBRATE 之前，BLE/UART 任务会设置 `enum_manager` 的挂起标志 (`enum_manager` 在主循环任务上运行 POLL/OFFER/ASSIGN)，因此枚举流量立即停止；接着延迟排空正在传输的枚举帧，最后发送带重试的 CALIBRATE。否则，高优先级的 ASSIGN 帧可能会挤掉单次未重试的 CALIBRATE，导致某些板子未校准。

主机**接收并解析**的帧 (`gateway_can_rx_handler`)：

|**ID**|**含义**|**处理**|
|---|---|---|
|`0x001`–`0x006`|从机反馈: `data[0]>>5`=状态位, 位置|→ JSON `{"motor":"DC","id":N,"status":...,"height":...}`|
|`0x7E1`|`ENUM_ACK`[id,uid32]|`enum_manager_on_ack()`|
|`0x100+id` 事件|校准限位 `0xA7` / 结果 `0xA6`|→ JSON 校准事件|
|`0x7DE`|OTA 响应 (ACK/NAK/PONG/DONE)|OTA 状态机|

## 17. 上位机命令集

上位机通过 UART/BLE 发送文本行，由 `gateway_process_line()` 进行分发 (`gateway.c:1003`)：

|**命令**|**格式**|**效果 / CAN 映射**|
|---|---|---|
|运动|`timestamp,h1,h2,h3,h4,h5,h6[,limit]` (CSV 高度 0–254)|验证高度范围和相邻高度差，按顺序入队，逐一发送运动帧 (`data[0]=0x05`)|
|急停|`{"action":"estop"}`|清空队列 + 广播 ESTOP (`0x03`)|
|查询|`{"action":"query"}`|报告网关状态并触发从机查询|
|校准|`CALIB` / `{"action":"calibration"}`|静默枚举，然后广播 `CONFIG+0xA5`，收集限位/结果事件|
|配置|`CFG,max_diff,delay,min_pwm,timeout,report[,motor_start_delay]`|存储在 NVS 中，向从机广播配置帧|
|单次调试|`ONESHOT,<id>,<height>`|**调试专用**：直接向单个节点发送单个 CONTROL 位置帧 (无队列，无重试)，回复 `oneshot_sent`；越界 `id`/`height` 会报错|
|OTA (app)|`OTA,node,size,crc32hex` + XMODEM-1K|将从机重启至引导程序，桥接二进制文件烧录 app 区域|
|OTA (bootloader)|`OTABL,node,size,crc32hex` + XMODEM-1K|运行中的 app 自我烧录 boot 区域 (有变砖风险)|

**电机队列** (`gateway.c` `start_next_motor()`)：顺序执行，每个电机之间等待 `motor_start_delay` (默认 800 ms)；发生堵转/超时跳过该电机并发出 `queue_skip`/`queue_timeout` 事件。

**运行时配置** (`config_mgr`，存于 NVS)：`max_diff` (相邻高度差上限，默认 20.0)、`async_delay`、`min_pwm` (默认 3500)、`timeout_s` (默认 6)、`report_s` (默认 1)、`motor_start_delay` (默认 800)。

## 18. 上位机报告协议 (JSON)

所有报告均通过 `ble_message_send()` **同时**发送至 UART0 和 BLE。`id=0` 代表网关自身；`id=1–6` 为从机。消息为按行分隔的 JSON 格式。

> **接口说明 (已针对 `d0dccff` 代码的 `gateway.c` 进行验证)**：电机反馈携带的位置是 **`height` (0–254)**，而不是角度 — 网关通过其 `height_map` 正弦映射将角度 ↔ 高度进行转换 (工作窗口 1.5°–71.5°)。旧版上位机协议文档 (v1.0) 中使用度数表示的 `pos` 及其角度范围已被取代；下方目录反映了当前固件的行为。

### 18.1 周期性与电机状态

每隔 `report_s` 秒（默认 1 s）自动报告一次，并在每次状态改变时报告：


```JSON
{"motor":"DC", "id":0, "status":"gateway_alive"}                   // 网关心跳
{"motor":"DC", "id":1, "status":"running", "height":120}          // 电机反馈
{"motor":"DC", "id":1, "status":"arrived", "height":220}
{"motor":"DC", "id":1, "status":"standby", "height":0}
{"motor":"DC", "id":1, "status":"sensor_error", "msg":"Motor 1 encoder read failed", "height":0}
{"motor":"DC", "id":1, "status":"sensor_recovered", "msg":"Motor 1 encoder recovered", "height":120}
```

电机 `status` 取值：`running` / `arrived` / `standby` / `sensor_error` / `sensor_recovered`。

### 18.2 队列调度

网关顺序运行电机（一次一个）。生命周期事件：

```JSON
{"motor":"DC", "status":"queue_created", "count":4, "msg":"Motors will run sequentially"}
{"motor":"DC", "status":"queue_start", "id":2, "target":230.00}
{"motor":"DC", "status":"queue_retry", "id":2, "attempt":1, "msg":"Stalled, re-issuing command"}
{"motor":"DC", "status":"queue_done", "id":2}
{"motor":"DC", "status":"queue_skip", "id":3, "msg":"Motor stalled"}
{"motor":"DC", "status":"queue_timeout", "id":4, "msg":"Motor timeout, skipping"}
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}
```

`queue_retry` 会在放弃前向堵转的电机重新下发命令；重试耗尽后发出 `queue_skip`。当等待电机到达的时间超过队列等待窗口时，会触发 `queue_timeout`。

### 18.3 命令响应

```JSON
{"motor":"DC", "status":"estop_triggered"}                    //{"action":"estop"}
{"motor":"DC", "id":1, "status":"queried", "height":225}                    // {"action":"query"}
{"motor":"DC", "status":"oneshot_sent", "id":3, "height":120, "ok":true}    // ONESHOT,3,120
{"motor":"DC", "status":"config_updated", "max_diff":20.0, "delay":50, "min_pwm":4500, "timeout":6, "report_rate":1, "motor_start_delay":800}   // CFG,...
```

### 18.4 校准事件

响应 `CALIB` / `{"action":"calibrate"}` 时发出的序列（网关首先执行急停并清空队列，然后广播 `CONFIG+0xA5`）：

JSON

```
{"motor":"DC", "status":"calibration_started", "msg":"Calibration started for all nodes"}
{"motor":"DC", "id":2, "status":"calibration_limit", "direction":"forward", "raw_angle":75.00, "raw_count":13011}
{"motor":"DC", "id":2, "status":"calibration_limit", "direction":"reverse", "raw_angle":0.50, "raw_count":9335}
{"motor":"DC", "id":2, "status":"calibrated", "msg":"Calibration succeeded", "reverse_raw":0.50, "forward_raw":74.80, "raw_span":74.30}
{"motor":"DC", "status":"calibration_complete", "msg":"All calibration reports received"}
```

失败 / 超时：

JSON

```
{"motor":"DC", "id":2, "status":"calibration_failed", "msg":"Calibration failed"}
{"motor":"DC", "status":"calibration_timeout", "msg":"Calibration timed out"}
```

只有在总线上所有的 `MOTOR_TOTAL_COUNT` (6) 个节点都报告了结果后，才会发出 `calibration_complete`；如果总线上的节点少于这个数字，尽管存在的节点均成功校准，它还是会在 30 s 后以 `calibration_timeout` 结束。

### 18.5 OTA 进度

在进行 `OTA`/`OTABL` 传输时，主机 UART 承载的是 XMODEM 二进制数据流，因此 `progress` 帧**仅**通过 BLE 发送；`start` / `done` / `error` 会发送给两个通道：

JSON

```
{"motor":"DC", "ota":"start", "id":2, "size":21760}
{"ota":"progress", "sent":8192}
{"motor":"DC", "ota":"done", "id":2}
{"motor":"DC", "ota":"error", "msg":"target did not enter bootloader"}
```

### 18.6 错误与警告

JSON

```
{"motor":"DC", "status":"error", "msg":"Timestamp expired"}                                  // 时间戳过期
{"motor":"DC", "status":"error", "msg":"Height out of range"}                                // 高度指令超出 0–254 范围
{"motor":"DC", "status":"error", "msg":"Adjacent height limit exceeded! Max height diff is 20.0"}
{"motor":"DC", "status":"error", "msg":"CAN transmit failed: broadcast_or_command, id=0x000"}
{"motor":"DC", "status":"error", "msg":"ESTOP send failed after 3 retries"}
{"motor":"DC", "status":"error", "msg":"CAN bus-off, recovering"}
{"motor":"DC", "status":"warning", "msg":"CAN error: TEC=100, REC=98"}                       // TEC 或 REC > 96
```

### 18.7 通道与快速参考

**双通道输出**：每条消息都会通过 UART0 (115200 8N1，有线上位机) 和 BLE (NUS，iOS/无线) 同时发送，调用函数为 `ble_message_send()` / `ble_message_send_fmt()`。

|**类别**|**status 取值**|**触发条件**|
|---|---|---|
|网关心跳|`gateway_alive`|定时 (每 `report_s`)|
|电机状态|`running` / `arrived` / `standby` / `sensor_error` / `sensor_recovered`|定时 + 事件|
|队列|`queue_created` / `queue_start` / `queue_retry` / `queue_done` / `queue_skip` / `queue_timeout` / `queue_complete`|事件|
|命令响应|`estop_triggered` / `queried` / `oneshot_sent` / `config_updated`|命令|
|校准|`calibration_started` / `calibration_limit` / `calibrated` / `calibration_complete` / `calibration_failed` / `calibration_timeout`|命令 + CAN 事件|
|OTA|`ota:start` / `ota:progress` / `ota:done` / `ota:error`|命令|
|错误 / 警告|`error` / `warning`|异常|

**上位机解析提示**：`motor` 始终为 `"DC"` 且必定包含 `status`；`id`、`height`、`target` 和 `msg` 视条件出现。请基于 `status` 进行分支处理：队列状态以 `queue_` 开头，异常状态为 `error`/`warning`，而 OTA 消息携带的是 `ota` 键而非 `status`。

## 19. 节点 ID 枚举 (主机端)

与 §6 (从机端) 相对应。主机由 `enum_manager` 驱动，其核心纯逻辑在 `enum_logic` 中 (可在上位机端单元测试，`test_enum_logic` 41/41 测试通过)：

1. 广播 **`ENUM_POLL` (0x7E3)** 进行点名 (空帧)。
    
2. 从机回复 **`ENUM_ACK` (0x7E1)** `[id, uid32]`。
    
3. `enum_logic` 折叠接收到的反馈：检测**冲突**（同一个 ID 出现多个 UID → 随机保留一个，将其他重定位）和**丢失**（多个周期未收到 ACK → 释放该 ID）。
    
4. 对于每一次冲突，广播 **`ENUM_ASSIGN` (0x7E2)** `[目标 ID, 目标 uid32]`。
    
5. 所有节点就绪后，以 ~1 Hz 的频率进行巡检；在 OTA/校准期间暂停。
    

> **校准前的枚举静默**：触发校准时，BLE/UART 任务首先设置 `enum_manager` 的挂起标志，让主循环任务停止发送 POLL/OFFER/ASSIGN，然后延迟排空传输中的枚举帧，再广播 CALIBRATE — 这样避免了高优先级的 ASSIGN 帧挤占掉仅发送一次的校准帧（见 §16）。

此方案与上电顺序无关，并对 OTA 软复位具有鲁棒性。已验证：干净的 1–6 枚举。

## 20. OTA 桥接 (主机端)

上位机通过 UART 运行 **XMODEM-1K**；主机将其桥接到从机引导程序的 CAN 帧中 (`0x7DD` 下行 / `0x7DE` 上行，`ota.c`)：

1. 上位机发送 `OTA,node,size,crc32hex`。
    
2. 主机将目标重启到引导程序 (`CONFIG 0xB0`) 并等待 **PING(0xA4)→PONG(0x52)**。
    
3. **START(0xA0)**：宣告大小，擦除区域 → 等待 ACK。
    
4. XMODEM 接收循环：对接收到的每个 1K 包，拆分为 6 字节的 **DATA(0xA1)** 帧并通过 CAN 转发，带有重传机制的停等协议。
    
5. **END(0xA2)**：CRC32 校验。**GO(0xA3)**：跳转运行。
    
6. 在 XMODEM 期间，进度**仅**通过 BLE 报告 (UART 打印会破坏二进制流)；`ota_is_active()` 会暂停主循环和诊断任务以保护 CAN RX 队列。
    

对于 `OTABL` (`CONFIG 0xB2`) 变体：应用程序自己接收并写入 boot 区域（**断电有变砖风险，只能通过 SWD 恢复，尚未经过硬件验证**）。架构见 §11，操作指南见 README。

## 21. 本地 UI

**状态 LED** (IO0 上的单颗板载 WS2816C，SPI 后端)：红色=错误 (10 个滴答)，绿色=CAN 活动 (3 个滴答)，蓝色=收到命令 (5 个滴答)，闪烁=校准中 (~2 Hz)。

**灯带** (2× WS2812, 每条 6 颗 LED, RMT 后端)：Strip0(IO4) 由 BUT1 切换，Strip1(IO5) 由 BUT2 切换；模式循环：关闭→常亮→闪烁→呼吸→彩虹。

**按钮** (低有效上拉)：BUT1(IO3) 切换 Strip0，BUT2(IO10) 切换 Strip1，BOOT(IO9) **长按 1.5 s → 触发整条总线的校准** (IO9 是 C3 的 strapping/BOOT 引脚 — 复位时必须为高电平，之后可作普通输入使用)。

## 22. 系统交接与上电自检

> 本节内容由前 `交接规格书.md` (交接规范) 合并而来，并逐行依据当前代码进行了验证；删除了原文档中过时的描述 (固定的 `MY_MOTOR_ID=1`、手动 ID 更改、ESP32-S3 从机参考、旧的主机引脚 GPIO4/5+MT6701)。

### 22.1 上电自检流程

1. **硬件检查**：CANH/CANL 接线、两端 120 Ω 终端电阻、各板共地、电机电源、MT6701 磁铁位置与气隙。
    
2. **节点枚举**：上电后，每个 APM32 会被 ESP32 主机枚举并自动获取 1–6 的 ID（**无需手动修改 ID**）；UNSET 节点不响应电机命令。机制：§6 / §19。
    
3. **连接主机**：在浏览器中打开 `tests/web/web_ble_test.html`，连接 BLE 设备 `VeryFun BLE`。
    
4. **查询**：发送 `{"action":"query"}`，确认网关 (`id:0`) 和各从机 (`id:1–6`) 均返回 JSON 反馈。
    
5. **校准**：首次使用或未存储有效校准时，请先进行校准 (发送上位机命令 `CALIB` / `{"action":"calibrate"}`，或长按主机 BOOT 键 1.5 s 触发整条总线校准)。观察各节点是否报告正反限位事件及 `calibrated` 结果。**校准是一个开环驱动直至机械硬限位的过程 — 首次运行时必须有人值守，随时准备急停。**
    
6. **协同运动**：将所有电机发送到 `50%` 高度，确认队列显示 `queue_start → arrived → queue_complete`。
    
7. **端点裕度**：测试 `0%` / `100%`，确认软件裕度未撞击机械硬限位 (工作窗口 1.5°–71.5°)。
    
8. **急停**：运动中途按下急停，确认电机停止并报告 `estop_triggered`。
    
9. **持久性**：通过 `CFG` 更改配置后重启网关，确认主机的 NVS 配置得以保留；校准后将从机断电再上电，确认其从 Flash 重新加载了校准映射 (`calib_ok` 仍为 1)。
    

### 22.2 交接说明 (已据代码验证)

- **外部接口使用的是 `0–254` 的高度值 (`-1`=忽略该电机)，而不是角度。** 相邻高度差限制也同样比较的是高度值。角度 ↔ 高度转换是由主机的 `height_map` 正弦映射处理的 (工作窗口 1.5°–71.5°)。旧的“角度命令”描述已不再适用。
    
- **目前为顺序队列**：一次只有一个电机启动（电机间间隔为 `motor_start_delay`，默认 800 ms），而不是 6 个并行。如果需要并行运动，则必须重新评估功率峰值和机械干涉。
    
- **APM32 Flash 仅持久化校准映射** (`raw_at_min_deg`/`raw_at_max_deg`/`mapped_min_deg`/`mapped_max_deg` + CRC32，见 §10.2)。`min_pwm`/`timeout_s`/`report_s` **未**存储在 APM32 的 Flash 中 — 主机在每次上电时会通过 CONFIG 帧将其推送给节点；主机负责将这些参数保存在 NVS 中 (`config_mgr`)。
    
- **节点 ID 不再固定也不再保存在 Flash 中**：每次上电都会由主机进行枚举 (§6)。`MY_MOTOR_ID=1` 仅作为 app 从未运行时的引导程序回退使用。
    
- **混合 CAN 字节序** (`can_service.c`)：控制/配置帧以及常规反馈帧中的位置、速度和 PWM 为**大端序 (big-endian)**；校准限位 (`0xA7`) / 结果 (`0xA6`) 事件中的原始角度 (int32 ×100) 和跨度 (uint16 ×100) 为**小端序 (little-endian)**。更改协议时，需同步修改主机、从机及测试代码。
    
- **首次上电若无有效校准则会在启动时自动触发校准** (`main.c` 中的 `needBootCalib` 分支)。
    

## 23. 宏定义快速参考 (涵盖两部分)

```C
/* ---- CAN IDs (两端一致) ---- */
MASTER_CAN_ID           0x000   /* 主机→总线：电机命令 */
/* 0x001-0x006  从机反馈   0x100+id  从机事件 */
CAN_STD_ID_ENUM_OFFER   0x7E0   CAN_STD_ID_ENUM_ACK    0x7E1
CAN_STD_ID_ENUM_ASSIGN  0x7E2   CAN_STD_ID_ENUM_POLL   0x7E3
OTA_CAN_ID_REQ          0x7DD   OTA_CAN_ID_RESP        0x7DE

/* ---- 基础模式 / 命令字节 ---- */
CAN_CMD_QUERY 0x00  CAN_CMD_CONTROL 0x01  CAN_CMD_CONFIG 0x02  CAN_CMD_ESTOP 0x03
/* 主机运动帧命令字节 data[0]=0x05 */
CAN_CONFIG_CMD_CALIBRATE 0xA5  CAN_EVENT_CMD_CALIBRATION_RESULT 0xA6
CAN_EVENT_CMD_CALIBRATION_LIMIT 0xA7
CAN_CONFIG_CMD_REBOOT_BOOTLOADER 0xB0  CAN_CONFIG_CMD_UPDATE_BOOTLOADER 0xB2

/* ---- OTA 操作码 ---- */
OP:  START 0xA0  DATA 0xA1  END 0xA2  GO 0xA3  PING 0xA4
ST:  ACK 0x50  NAK 0x51  PONG 0x52  DONE 0x53

/* ---- APM32 地址 ---- */
UID  0x1FFFF7E8 / 0x1FFFF7EC / 0x1FFFF7F0
OTA_APP_BASE 0x08004000   OTA_CALIB_BASE 0x0801FC00   OTA_BKP_MAGIC 0xB007

/* ---- ESP32 BLE / 引脚 ---- */
设备名 "VeryFun BLE"   服务 0xFFE0   TX 0xFFE1(Notify)   RX 0xFFE2(Write)
CAN TX=IO7 RX=IO6   UART0 115200   LED=IO0   STRIP IO4/IO5   BUT IO3/IO10/BOOT IO9
```

## 附录 A · 文档版本历史

> **版本约定**：本文档版本与固件/硬件版本各自独立。每次修订均记录三个基线 — **文档版本**、**代码基线** (git 简短哈希) 及 **配套硬件规格版本** — 以便追溯。修订版本会更新头部区块和下方表格。如果固件源码与本文档不一致，以源码为准（见页脚）。

| **文档版本** | **日期**     | **代码基线**  | **配套硬件规格**                                                  | **主要变更**                                                                                                                                                                                                                                                                                                                                 |
| -------- | ---------- | --------- | ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **v1.3** | 2026-06-17 | `d0dccff` | APM32 `HW_Spec_V1.2`, ESP32-C3 `BT_Controller_HW_Spec_V1.0` | 将所有独立文档合并为这本单一参考指南，并将全文翻译为**英文**(此为中文版)。将通过 CAN 的 OTA 架构 (前 `FIRMWARE_OVER_CAN.md`) 合并入 §11，将完整的上位机报告协议 (前 `HOST_REPORTING_PROTOCOL.md`，针对当前基于高度的接口进行协调) 合并入 §18。将所有环境搭建、构建、烧录、OTA 操作员指南及测试说明移至 `README.md`；从本规格书中删除构建/烧录/测试相关章节。删除了五份独立文档 (DEV_ENV, TESTING, FIRMWARE_OVER_CAN, FLASHING_ALL_BOARDS_OVER_CAN, HOST_REPORTING_PROTOCOL)。 |
| **v1.2** | 2026-06-17 | `d0dccff` | APM32 `HW_Spec_V1.2`, ESP32-C3 `BT_Controller_HW_Spec_V1.0` | 修正了整个代码库中硬件规格版本的引用 **V1.4 → V1.2** (该仓库归档的是 V1.2，不存在 V1.4 文件；规格书的内部标签为 V1.2 / 2026-05-27，并且其过流 1.0 A / 堵转 0.8 A·500 ms 与固件匹配)。同步了 `README.md`, `app_config.h`, `main.c` 注释。新增本版本历史附录。将仓库中的双生文档同步至 v1.2 并重命名 `docs/技术文档.md` → `docs/esp32c3-apm32f103_firmware_specs.md`。                                                                 |
| **v1.1** | 2026-06-16 | `d0dccff` | V1.2 / V1.0                                                 | 启动**踢击脉冲** (6000 PWM / 30 ms)；堵转检测从每周期速度门限更改为**净进度累加** (300/450 个周期, `STALL_PROGRESS_DEG`)；电流保护更改为**百分之一度净进度** + 一次基于每个命令的**重触发宽限期** (`STALL_RECOVERY_GRACE_MS`)，启动期间跳过；校准**方向锚定**；新增了上位机 `ONESHOT` 调试命令；校准广播前引入**枚举静默**；测试目录树移至 `tests/integration/`，添加了 `hil_system` (L3/L4) 和 `test_stall_recovery`。                                  |
| **v1.0** | 2026-06-08 | `c9309fe` | (注释误标为 V1.4，实际为 V1.2)                                       | 首次发布合并的双 MCU 固件技术参考，涵盖了 APM32 从机和 ESP32-C3 主机、CAN 协议、校准、OTA、本地 UI 以及交接自检。                                                                                                                                                                                                                                                                |

**配套硬件基线 (位于 v1.2+)**：APM32 电机控制板 PCBA `APM32_DRV8876_V1.2` (规格书 `APM32_Motor_Controller_HW_Spec_V1.2`, 2026-05-27，归档于 `hardware/motor-control-apm32/`)；ESP32-C3 主机板规格书 `BT_Controller_HW_Spec_V1.0` (归档于 `hardware/esp32-c3-mini/`)。硬件规格书变更日志见各目录下的 `hardware/*/changes/CHANGELOG.md`。

_本文档基于代码核对生成 (`d0dccff`)。当代码与本文档不一致时，以源文件为准：APM32 端 `motor_control_apm32/app/Include/app_config.h`, `board_config.h`, `common/ota_layout.h`, `app/Source/{motor_controller.c,control_protection.c,main.c}`；ESP32 端 `ble_esp32/main/config/{app_config.h,board_config.h}`, `main/app/gateway.c`, `main/hal/hal_*_esp32.c`。搭建、构建、烧录、OTA 操作和测试详见 [`../README.md`](../README.md)。_