
```c
//插入位置：main.c main()中各种初始化后，TEST_BUILD前
    /* === B1 power-on startup status report === */
    DebugUartSendString("=== B1 Startup Report ===");
    DebugUartSendLineEnd();

    /* HSE crystal status */
    {
        uint8_t hse_ready = (RCM->CTRL_B.HSERDYFLG == BIT_SET) ? 1U : 0U;
        DebugUartSendString("HSE ready=");
        DebugUartSendUint(hse_ready);
        DebugUartSendString(" (0=fail 1=ok)");
        DebugUartSendLineEnd();
    }

    /* DRV8876 status */
    {
        uint8_t fault = BoardReadFaultPin();  // nFAULT=PA6, LOW=fault
        DebugUartSendString("DRV8876 nFAULT=");
        DebugUartSendUint(fault);
        DebugUartSendString(" (0=ok 1=fault)");
        DebugUartSendLineEnd();
    }

    /* CAN init status */
    {
        /* CAN init status: INITFLG == 0 means CAN has exited init mode and is in normal operation */
        uint8_t can_ok = (CAN1->MSTS_B.INITFLG == RESET) ? 1U : 0U;
        DebugUartSendString("CAN init=");
        DebugUartSendUint(can_ok);
        DebugUartSendString(" (0=fail 1=ok)");
        DebugUartSendLineEnd();
    }

    DebugUartSendString("Node init done id=");
    DebugUartSendUint(NodeIdGet());
    DebugUartSendString(" nsleep=PA7 high can=50k uart=115200");
    DebugUartSendLineEnd();
    DebugUartSendString("=== B1 Report End ===");
    DebugUartSendLineEnd();
```
