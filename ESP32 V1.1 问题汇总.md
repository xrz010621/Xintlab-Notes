- 长时间放置后，首次上电，STATUS指示灯不亮，按键和灯带不工作，接串口后无日志打印
```
[10:40:35.832]收←◆ESP-ROM:esp32c3-api1-20210207
Build:Feb  7 2021
rst:0x1 (POWERON),boot:0x5 (DOWNLOAD(USB/UART0/1))
waiting for download
```
进入了烧录模式

ESP32-C3数据手册：

| 参数    | 名称         | 含义                                               | ESP32-C3值  |
| ----- | ---------- | ------------------------------------------------ | ---------- |
| `tSU` | Setup time | 电源稳定后，到 `CHIP_EN` 拉高前，给 strapping pin 准备状态的时间    | ≥ 0 ms     |
| `tH`  | Hold time  | `CHIP_EN` 拉高后，芯片读取 GPIO2/GPIO8/GPIO9 状态，并保持采样的时间 | **≥ 3 ms** |
Strapping Pins：GPIO2/GPIO8/GPIO9

- 上电后随机出现灯带若干LED灯闪烁，数量不等，颜色大部分是白色和绿色
- 每次上电时STATUS灯的亮度和颜色不同，和后面灯带同步
- 连接蓝牙后或者按按键时，STATUS灯会随机变颜色
