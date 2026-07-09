>类型：**测试报告** - 错误现象查因，主联调  
>日期：2026-7-9

# APM32无法操控电机

## 问题背景

ESP32给APM32发送电机转动命令`ONESHOT,1,0`，APM32可以接受到指令（状态灯有显示），但电机无反应

## 调试环境

24V电源，esp32，apm32，CAN总线连接，UART串口，SSCOM5.13.1，万用表

## 排查方向

### 1. 电机驱动芯片损坏：怀疑APM32下达pwm控制，但电机驱动接收未工作，电机驱动芯片损坏

**方法**：用万用表电压档，检测电机驱动和mcu的pwm使能引脚是否有高电平，MC_EN引脚

**结果**：电机驱动芯片nsleep使能引脚高电平，但MC_EN（即mcu的pwm）引脚没有电压，无pwm输出。

### 2. MCU芯片损坏：怀疑PWM引脚损坏
    
**方法**：电脑连接apm32串口，发现串口接收指令后输出：

```
[16:02:13.680]收←◆MT6701 read_failed last_raw=0 err=255
RUN state=1 status=3 sub=1 angle=0.00 target=0.00 pwm=0 raw=0.00 map=1 min=199.34 max=281.16 span=81.83 fault=0 enc_err=255
```

其中关于status的取值：

|值|宏|含义|
|---|---|---|
|**0**|`MOTOR_STATUS_IDLE`|空闲，等待指令|
|**1**|`MOTOR_STATUS_RUNNING`|运行中，`motor_ctrl_update()` 每10ms执行PD控制|
|**2**|`MOTOR_STATUS_ARRIVED`|已到达目标位置|
|**3**|`MOTOR_STATUS_ERROR`|故障状态，`fault_latched=true`|
|**4**|`MOTOR_STATUS_SENSOR_ERROR`|传感器错误（仅用于CAN反馈上报，内部实际进入ERROR）|



结果：apm32存在错误，超时或电机堵转，导致mcu未使能pwm操控电机

### 3. 磁编码器I2C通信问题：怀疑I2C通信错误
    

静态断电调试：

- 万用表测试I2C上拉电阻阻值是否正确：阻值无穷大——虚焊；阻值为0——短路
    
- 检查I2C引脚是否对地短路
    
- 检查mcu引脚和磁编码器引脚是否对应连接导通
    

动态通电调试：

- 测量SCL和SDA空闲时是否都是高电平（3.3V）
    
- 测量SCL和SDA发送命令时万用表是否高低电平交替（ps：可能存在万用表刷新频率不够，无法显示交替）
    
- 排查磁编码器芯片本身是否损坏，强行把电平拖死：去除编码器芯片，让mcu I2C引脚悬空，测I2C引脚电压是否有变化
    

## 调试结论

软硬件版本未对齐，I2C引脚互换

```
[17:01:18.863]收←◆Motor controller entered ERROR reason=1 pwm=0
```