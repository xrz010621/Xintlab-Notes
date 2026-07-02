**MT6701**
对apm32电机模块上电后运行测试，最开始目的为测试不同型号磁铁对MT6701的影响做准备，后该测试取消。因此主要是对apm32的驱动功能做了一定了解，明确了apm32烧写方式，完整地对电机的存活检测、标定、以及运行指令进行尝试。
在运行过程中有发现电机仅对标定CALIB指令有响应，而对于MOVE指令则一直返回ERROR状态，日志具体输出为stall=1，电流过高（>850mA,进入堵转保护）. 后续检查后发现为代码中引脚定义错误：
```
	[原代码，定义为PA0，但PA0悬空]
	#define BOARD_IPROPI_GPIO_PIN   GPIO_PIN_0
	#define BOARD_IPROPI_ADC_CHANNEL ADC_CHANNEL_0

	[实际引脚为PA3，修改后工作正常]
	#define BOARD_IPROPI_GPIO_PIN   GPIO_PIN_3
	#define BOARD_IPROPI_ADC_CHANNEL ADC_CHANNEL_3	
```

针对**issue#32** **偶发枚举(POLL/OFFER)帧未被电机单元正确捕获，节点 ID 分配异常**（[[Issue32|详见文档]]）开展工作。
第一步想法是在从机侧打印CAN帧接收情况，通过查看CAN接收日志判断是否出现帧丢失等问题。
但在仅连接一台从机进行测试时，发现了以下问题：

1. 在从机端`can handle`后通过UART输出日志 （未烧写`TEST_BUILD`)：
```
	1.仅处理POLL，显示为没有接收到OFFER -> 
	主机侧日志反复显示如下：
	   transmitting but no frames received                 [POLL帧响应]
	   GATEWAY_CTRL: ENUM assigned=0 offer=1:
		{"motor":"DC", "id":0, "status":"gateway_alive"}   [OFFER帧发送但未被处理]
	从机侧日志：[仅显示接收到POLL，未有OFFER]
	[t=00000316][UID=15274C76] RX POLL, myId=0, skip (UNSET)
	[t=000005A0][UID=15274C76] RX POLL, myId=0, skip (UNSET) 
	[t=0000082E][UID=15274C76] RX POLL, myId=0, skip (UNSET)
	
	2.处理多轮POLL后，处理OFFER【次数随机】 -> 即随机出现assigned情况
	主机侧日志多次“no frames received"之后：
		GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76 
		{"motor":"DC", "id":1, "status":"standby", "height":0} 
		{"motor":"DC", "id":0, "status":"gateway_alive"}
	从机侧日志：
	[t=00000334][UID=15274C76] RX POLL, myId=0, skip (UNSET) 
	[t=000005BE][UID=15274C76] RX POLL, myId=0, skip (UNSET)
	[t=00000842][UID=15274C76] RX POLL, myId=0, skip (UNSET)
	[t=00000847][UID=15274C76] RX OFFER id=1, myId=0, accept, ack pending
	[t=00000875][UID=15274C76] TX ACK id=1
	[t=00000AD4][UID=15274C76] RX POLL, myId=1, will ACK
	[t=00000AF6][UID=15274C76] TX ACK id=1              [并且这里也没有OFFER的RX]
```
- 在从机侧打印时间戳后会发现`连续多次POLL之间时间差在650左右`，即分别`来自不同的cycle`，而<u>同一cycle中的OFFER帧未被处理</u>（FIFO为空，因为下一次处理的是新的POLL；且主机侧日志排除了`未成功发送OFFER`的情况 - `tx_fail=0`）
	- *目前猜测为在handle函数中调用` USART_TxData`，串口阻塞式输出约4~7ms，对FIFO中帧处理造成影响。*

2. 仅考虑原项目代码情况，但不导入`test_shell.c`&不烧写`TEST_BUILD`时（无日志输出），发现也出现了从机侧无法在上电后第一帧响应OFFER的情况：
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
				[这里rx=2，有丢帧还是未发送？]
I (2896) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76    [从机侧已经被分配“1”]
{"motor":"DC", "id":0, "status":"gateway_alive"}     
[一直没有触发从机端分配id后周期性上报的日志]
```
  以导入`test_shell.c`&烧写`TEST_BUILD`的日志作为对比：
``` 
1.第一个cycle
I (766) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (776) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (786) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}
{"motor":"DC", "id":1, "status":"standby", "height":0}   [第一个cycle即成功分配的从机]
[↑这一条后续会以1000ms为周期由从机端向主机端上报]

2.第二个cycle - id=1已成功分配，POLL帧有rx，OFFER帧分配offer=2
I (1826) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=3 last_rx=341ms_ago | msgs_to_rx=0
I (1836) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=15274c76
{"motor":"DC", "id":0, "status":"gateway_alive"}
```
- 因此可以发现测试固件`TEST_BUILD`会产生影响
	- *具体影响的原因还待进一步检测。*

3. 尝试如果在枚举输出日志的同时也烧写`TEST_BUILD&test_shell.c`，主机侧日志和从机侧日志显示正常。
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
- 即可以发现`TEST_BUILD`会对该通信过程产生影响。
	- *暂考虑可能原因在于时序问题影响下的FIFO溢出（从机侧无法接收同一cycle内的offer）*

4. 考虑main.c循环中反复出现的DebugUartSend*()函数，通过UART阻塞式tx造成的延时，注释掉后再次测试无test固件情况下，主机侧日志情况同2.


