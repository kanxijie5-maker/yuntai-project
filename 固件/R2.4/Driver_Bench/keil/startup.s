        PRESERVE8
        THUMB
        AREA    RESET, DATA, READONLY
        EXPORT  __Vectors
        IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Limit|
        IMPORT  ADC0_1_IRQHandler
__Vectors
        DCD     |Image$$ARM_LIB_STACK$$ZI$$Limit|, Reset_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     ADC0_1_IRQHandler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler, Default_Handler, Default_Handler
        DCD     Default_Handler, Default_Handler
        AREA    |.text|, CODE, READONLY
Reset_Handler PROC
        EXPORT  Reset_Handler
        IMPORT  SystemInit
        IMPORT  __main
        ; 指定 Flash 向量表，设置系统时钟，再由 C 运行库初始化数据并调用 main。
        LDR     r0, =0xE000ED08
        LDR     r1, =__Vectors
        STR     r1, [r0]
        DSB
        ISB
        BL      SystemInit
        B       __main
        ENDP
Default_Handler PROC
        EXPORT  Default_Handler
        CPSID   i
        ; 异常时先关闭三相桥，再锁定故障现场。
        IMPORT  bridge_off
        BL      bridge_off
halt    B       halt
        ENDP
        ALIGN   4
        LTORG
        END
