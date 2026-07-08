>Issue文档：[[Issue32]]
>前置已完成单个从机连接测试：[[Self - Test - 单从机id分配异常测试过程和结果记录]]
>类型：测试记录 - 测试过程记录（搁置中）
>作者：本人记录
>日期：2026-7-1

### 1 前置背景


### 2 测试过程
#### 2.1 初步上电
**场景1 原apm32项目代码+无TEST固件**
主机端日志表现出**无从机响应**的情况，或者是**随机cycle出现响应**。
```
##主机端日志##
--第1个cycle--
I (806) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (816) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (826) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}

--第2个cycle--
I (1866) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (1876) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (1886) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "id":0, "status":"gateway_alive"}

--第3个cycle--
I (2916) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=9 tx_fail=0 last_tx=131ms_ago | rx=1 last_rx=81ms_ago | msgs_to_rx=0
I (2926) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "id":0, "status":"gateway_alive"}

--第4个cycle--
I (3956) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=11 tx_fail=0 last_tx=521ms_ago | rx=2 last_rx=481ms_ago | msgs_to_rx=0
I (3966) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=6e00521d
{"motor":"DC", "id":0, "status":"gateway_alive"}
```
**现象**：
①有出现一直循环POLL帧无响应的情况，即主机日志中一直`rx=0`和`no frames received`，OFFER帧相关日志一直是`assigned=0 offer=1`。（5+个cycle）
②如上日志显示，某一个cycle（随机）出现`rx=1`，但状态`assign=0 offer=1`依旧没变，即显示依旧无从机分配；在下一个cycle出现`assigned=1 offer=2: 1=6e00521d`，即显示已分配从机uid；而后一直呈现相同状态，不会出现`assigned=2`的情况。

**场景2 从机侧日志输出+消除双重FIFO释放+无TEST固件**
主机端日志
```
--第1个cycle--
I (766) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=1 tx_fail=0 last_tx=44ms_ago | rx=0 last_rx=never | msgs_to_rx=0
W (776) HAL_CAN: [CAN] transmitting but no frames received -- check slave power, wiring, 50k baud match, and 120R termination
I (786) GATEWAY_CTRL: ENUM assigned=0 offer=1:
{"motor":"DC", "status":"queue_complete", "msg":"All motors completed"}

--第2个cycle--
I (1826) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=5 tx_fail=0 last_tx=381ms_ago | rx=2 last_rx=341ms_ago | msgs_to_rx=0
I (1836) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=6e00521d
{"motor":"DC", "id":0, "status":"gateway_alive"}

--第3个cycle--
I (2866) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=9 tx_fail=0 last_tx=121ms_ago | rx=4 last_rx=51ms_ago | msgs_to_rx=0
I (2876) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=6e00521d
{"motor":"DC", "id":0, "status":"gateway_alive"}

--第4个cycle
I (3906) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=11 tx_fail=0 last_tx=511ms_ago | rx=5 last_rx=471ms_ago | msgs_to_rx=0
I (3916) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=6e00521d
{"motor":"DC", "id":0, "status":"gateway_alive"}

--第5个cycle--
I (4946) HAL_CAN: [CAN] state=RUNNING TEC=0 REC=0 | tx_ok=15 tx_fail=0 last_tx=251ms_ago | rx=7 last_rx=211ms_ago | msgs_to_rx=0
I (4956) GATEWAY_CTRL: ENUM assigned=1 offer=2: 1=6e00521d
{"motor":"DC", "id":0, "status":"gateway_alive"}
```
**现象**：
①从第2个cycle开始，就已经有一个从机`assigned=1` ，主机开始`offer=2`，但并未在第1个cycle里有应答。
	这里可能是主机在第1个cycle发送POLL+OFFER帧后，在`COLLECTING窗口`对从机进行`assign`？那么主机的主动assign是否会使从机产生应答？  —— 需要进一步确认
②第2个cycle处的`rx=2`，第3个cycle的`rx=4`，即此时一个周期主机收到来自从机的应答应当为2个，但第4个cycle处只增加了1个。tx同理，`tx_ok`每一个cycle增加4，但第4个cycle仅增加2。但第5个cycle又恢复。
③既然第2个cycle出现了`assigned=1`，那么说明是有某一种方式可以使从机又自行成功分配id的。但一直到第9个cycle一直未出现`assigned=2`。 —— 进一步检测第二个从机状态