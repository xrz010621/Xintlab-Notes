### 调用情况1（仅在apm32侧加串口输出日志）
**操作**：参考[[SubPlan1Repo_Code_Modification|子方案代码修改repo]]完成了代码增改（不完全一致）
① 情况 1 - 只上电观察串口输出
![[{E565757C-B8E1-4023-8301-79D940A90479}.png]]
<u>第四次和offer之间的间隔很短，几乎是前后一起出现</u><sup>（①）</sup>，相较于前面的规律性间隔
目前固定是在rx 4 poll之后，rx offer

② 情况2 - 只上电观察串口输出
![[{C96601B5-9CDF-4819-A5E9-5CB2AE81A3E4}.png]]
一直没法调用offer
这种情况符合猜想：<u>由于串口阻塞式打印占用了时间</u><sup>（③）</sup>，以至于相同时间间隔下接收到的数据是相同的（都是POLL帧）

③ 情况3 - 调试模式下+断点+手动run
![[{A4F8C490-97C6-4211-BB90-07E0AB7FD849}.png]]
这种情况也符合猜想：由于串口阻塞式打印占用了时间，以至于相同时间间隔下接收到的数据是相同的，然后由于手动run会出现细微时间偏差，所以可能从POLL帧的接收进入到OFFER帧接收。（这个情况的图应该是前面部分只有POLL的响应，然后后面突然变成只有OFFER响应，即同时只触发某一个帧的响应）
但这里的POLL和OFFER是同时触发的，说明时间是足够两个帧都触发？还是由于手动断点增加了CPU响应时间以至于能够打印？

④ 情况4 - 调试模式+断点
![[{84EE7D55-49B3-4B41-AF5C-CC1B061BCBA5}.png]]
只加断点的情况下，如果因为串口打印问题应该响应如情况2. 但会自行进入OFFER断点（类似情况1，但比情况1少了2轮）

**Note1**
① 三个POLL间隔较远是来自三个不同的cycle（≈650ms->400ms+250ms），而第四轮间隔很近是由于，POLL和OFFER帧本身是在同一个cycle前后进FIFO（到达间隔仅 ~1ms）
② **硬件收帧**：CAN 控制器完全自主，不需要 CPU 参与。帧从总线上收下来，自动推入 FIFO 队尾。
	**CPU 读帧**：FIFO0为3级深度，通过`CAN_RxMessage()` 读队头，`CAN_ReleaseFIFO()` 让队头出队，后面的帧前移。
	二者为并行处理，因此理论上来说，<u>排进FIFO0的帧会顺序进行处理</u>。
	==时序==：
```
     ESP32 发:  [POLL]──1ms──[OFFER]                    [POLL]──1ms──[OFFER]
     APM32 HW:  收POLL→FIFO[0]  收OFFER→FIFO[1]        ...
     APM32 CPU:                                        
     同一tick内:  CanServicePoll 读 FIFO[0]=POLL
                 └─ UART 4.5ms（硬件继续收帧，但总线此时空闲）
                 CanServiceReceiveCommand 读 FIFO[0]=OFFER（已前移）
                 └─ UART 4.5ms
                 FIFO 空，tick 结束
                                                        
     下一个cycle是650ms（400ms+250ms收集窗口）后 → FIFO绝不会溢出                       
```

③每条日志的传输时间
- UART 配置：**115200 bps, 8N1**（[main.c:203](vscode-webview://1di0rj3m68rtilivvsof2ar2qa51473mfp0b4lnv0lh12a90doum/motor_control_apm32/app/Source/main.c:203)）
- 每字符 = 1 start + 8 data + 1 stop = **10 bit** → **~87µs/字符**
- 每条日志约 40~80 字符（含 `[UID=XXXXXXXX]` 前缀 + 消息体 + `\r\n`）
- **每条日志耗时：约 3.5~7.0ms**（全阻塞，`EnumDiagSendChar()` spin-wait TX buffer empty）

	影响路径：
```
RunControlLoop() 10ms 一次
  ├─ CanServicePoll()
  │    ├─ CanServiceServiceAck()     ← 不涉及日志，快速通过
  │    └─ if (FIFO 有帧):
  │         读取 1 帧
  │         CanServiceHandleEnum()   ← 【如果匹配 POLL/OFFER/ASSIGN，则 inline 日志输出 3.5~7ms】
  │
  └─ while (CanServiceReceiveCommand())
       └─ drain FIFO（最多 8 帧）
           每帧也可能触发 CanServiceHandleEnum()
           CanServiceHandleEnum()    ← 【同上的日志阻塞】
```

**结论**：在调试时开启会导致每收到一个枚举帧产生约 3.5~7ms 的 CPU 阻塞（等待 UART TX buffer 空），该阻塞发生在 CAN 帧处理的 inline 路径上。在极端情况下（总线高负载 + 多枚举帧同时到达）可能导致 3 帧的 bxCAN FIFO 溢出丢帧，但正常的枚举流程（<u>每 400ms 周期仅 2 个枚举帧</u>）下风险可控。

下一步可能分析：
1. **ESP32 侧 `twai_transmit(OFFER)` 在头几个 cycle 静默失败**（`send_offer` 忽略了返回值）——用 ESP32 串口看 `tx_fail` 计数就能确认
2. 或 APM32 CAN 控制器在冷启动头几个帧存在同步问题导致 OFFER 被硬件丢弃——用 `CAN_FLAG_F0OV` 和错误计数器可确认


### 调用情况2：
**操作**：从机侧添加时间戳、CAN FIFO over标记检测；

①接主机侧串口查看日志：
```
第1轮： 
I (766) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0 
W (776) HAL_CAN: [CAN] transmitting but no frames received-- check slave power, wiring, 50k baud match, and 120R termination               [← POLL帧响应]
I (786) GATEWAY_CTRL: ENUM assigned=0 offer=1:             [← OFFER帧发送但未被处理]
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}

【第2轮 ~ 第6轮日志同第1轮】

直到第7轮日志收到回复： 
I (7056) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=21 tx_fail=0 last_tx=381ms_ago | rx=3 last_rx=331ms_ago | msgs_to_rx=0 
I (7066) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76  [← id分配成功]
{"motor":"DC", "id":1, "status":"standby", "height":0} 
{"motor":"DC", "id":0, "status":"gateway_alive"}
```
从以上日志可发现：
	①tx_fail = 0，主机侧发送是成功的。
	②第一次发送POLL帧后，出现的`no frames received`，对应为从机端的第一个【POLL -> skip】情况
	
②如果正常情况下， 应该是
```
--主机端正常工作日志（单从机）--
I (746) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0 
W (756) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination 
I (766) GATEWAY_CTRL: ENUM assigned=0 offer=1: 
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"} {"motor":"DC", "id":1, "status":"standby", "height":0}
```
可以发现：
在第一对I&W（POLL帧对应）的diag之后 ，还有关于OFFER帧对应的diag输出，且有从机已成功分配id后的上报`{"motor":"DC", "id":1, "status":"standby", "height":0}`。即此时是从机有对OFFER帧进行处理和响应输出的。
->
**初步怀疑，在从机侧进行日志输出会影响到帧处理。**

### 调用情况3（新发现）：
①仅考虑原项目代码情况，但不导入`test_shell.c`&不烧写`TEST_BUILD`时（无日志输出），发现也出现了从机侧无法在上电后第一个cycle响应OFFER的情况：
```
主机侧日志：
1.第一个cycle
I (776) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (786) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (796) GATEWAY_CTRL: ENUM assigned=0 offer=1:          
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}

2.第二个cycle - 依旧在offer=1，即第一个cycle未分配成功
I (1836) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (1846) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (1856) GATEWAY_CTRL: ENUM assigned=0 offer=1: [可以发现没有从机分配成功，assigned=0]
{"motor":"DC", "id":0, "status":"gateway_alive"}

3.第三个cycle - 显示已经assigned了，但并未在主机侧前两个cycle日志发现
I (2886) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=9 tx_fail=0 last_tx=131ms_ago | rx=2 last_rx=91ms_ago | msgs_to_rx=0 
				[这里rx=2，猜测为从机漏发]
I (2896) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76    [从机侧已经被分配“1”]
{"motor":"DC", "id":0, "status":"gateway_alive"}     
[一直没有触发从机端分配id后周期性上报(*)的日志]
```
(\*)从机端分配id后周期性上报：`{"motor":"DC", "id":1, "status":"standby", "height":0}`，这一条在从机得到id后，会以1000ms为周期由从机向主机端上报。

②以在原项目代码下导入`test_shell.c`&烧写`TEST_BUILD`的日志作为对比（**正常工作情况**）：
``` 
1.第一个cycle
I (766) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (776) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (786) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}
{"motor":"DC", "id":1, "status":"standby", "height":0}   [第一个cycle即成功分配id]

2.第二个cycle - id=1已成功分配，POLL帧有rx，OFFER帧分配offer=2
I (1826) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=3 last_rx=341ms_ago | msgs_to_rx=0
I (1836) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76
{"motor":"DC", "id":0, "status":"gateway_alive"}
```

③尝试如果在`输出枚举日志（即同调用情况1的代码）`的同时也烧写`TEST_BUILD&test_shell.c`，主机侧日志和从机侧日志**显示正常**。
```
1.第一个cycle
--主机侧--
I (776) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (786) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (796) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}
{"motor":"DC", "id":1, "status":"standby", "height":0}
--从机侧--
[t=0000031B][UID=15274C76] RX POLL, myId=0, skip (UNSET)
[t=00000320][UID=15274C76] RX OFFER id=1, myId=0, accept, ack pending
[t=0000034E][UID=15274C76] TX ACK id=1

2.第二个cycle
--主机侧--
I (1836) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=3 last_rx=341ms_ago | msgs_to_rx=0
I (1846) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76
{"motor":"DC", "id":0, "status":"gateway_alive"}
--从机侧--
[t=000005A4][UID=15274C76] RX POLL, myId=1, will ACK
[t=000005A8][UID=15274C76] RX OFFER id=2, myId=1, already assigned
[t=000005C2][UID=15274C76] TX ACK id=1
```
- 即可以发现**调用情况1&2**关于`在从机侧使用串口输出日志会对节点分配造成影响`的猜测不正确，转而考虑`TEST_BUILD`会对该通信过程产生影响。
	- 但由于`TEST_BUILD`本身仅仅是作为测试固件来使用，不影响CAN通信 -> *暂考虑可能原因在于时序问题影响下的FIFO溢出（从机侧无法接收同一cycle内的offer）*

### 调用情况4
根据1-3的调用情况，对多种因素分别进行多次测试，统计如下：

| #   | main.c                          | ENUM_DIAG | TEST_BUILD | 结果  |
| --- | ------------------------------- | --------- | ---------- | --- |
| 1   | 当前<br>（即将`DebugUartSend*` 函数删除） | ON        | ON         | ❌   |
| 2   | 原始<br>（原始项目代码，不管UART的输出调用）      | ON        | OFF        | ❌   |
| 3   | 原始                              | OFF       | ON         | ✅   |
| 4   | 原始                              | ON        | ON         | ✅   |
| 5   | 原始                              | OFF       | OFF        | ❌   |

### 最终发现
**现象**：在对**单个**从机(APM32) 进行日志打印测试验证时，发现在不烧录`TEST_BUILD`固件的情况下，连续出现上电后从机仅`打印POLL接收&无OFFER接收`的情况，在主机端表现为日志中`assigned=0`状态反复循环（即`节点ID分配失败`）
**修改方案**：apm32项目中`Source/can_service.c`内：
1. 轮询函数`CanServicePoll`中删除`CAN_ReleaseFIFO(CAN1, CAN_RX_FIFO_0);`
2. 遍历函数`CanServiceReceiveCommand`中删除`CAN_ReleaseFIFO(CAN1, CAN_RX_FIFO_0);`
**原因**：apm32_can库中`CAN_RxMessage`和`CAN_ReleaseFIFO`函数有相同的`FIFO释放`流程（详见`apm32f10x_can.c : 588-595 & 614-621`），原`can_service.c`中连续调用`CAN_RxMessage`和`CAN_ReleaseFIFO`，由于主机发送的`POLL`和`OFFER`连续到达从机，会导致未读的信息帧被直接释放，表现为`从机仅收到POLL，未收到OFFER` -> `从机无id，不响应ACK` -> `主机收不到ACK，assigned不更新`。
**与`TEST_BUILD`的关系**：可能是由于TEST固件中的UART阻塞式调用和函数不同方式定义导致的系统时序变动，恰好错开了以上情况。
**修改后结果**：重新尝试`烧写/不烧写TEST固件`，节点均分配正常：①从机端日志正常输出`RX POLL | RX OFFER | TX ACK`的周期循环；②主机端日志在上电后第一周期内即可成功对该唯一从机分配id，正确更新`assigned`值。