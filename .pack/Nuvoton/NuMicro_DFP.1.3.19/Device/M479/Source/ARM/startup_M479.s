;/******************************************************************************
; * @file     startup_M479.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M4 Core Device Startup File for M479
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000800
	ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000100
	ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     BOD_IRQHandler            ; 0: Brown Out detection
                DCD     IRC_IRQHandler            ; 1: Internal RC
                DCD     PWRWU_IRQHandler          ; 2: Power down wake up
                DCD     RAMPE_IRQHandler          ; 3: RAM parity error
                DCD     CKFAIL_IRQHandler         ; 4: Clock detection fail
                DCD     Default_Handler           ; 5: Reserved
                DCD     RTC_IRQHandler            ; 6: Real Time Clock
                DCD     TAMPER_IRQHandler         ; 7: Tamper detection
                DCD     WDT_IRQHandler            ; 8: Watchdog timer
                DCD     WWDT_IRQHandler           ; 9: Window watchdog timer
                DCD     EINT0_IRQHandler          ; 10: External Input 0
                DCD     EINT1_IRQHandler          ; 11: External Input 1
                DCD     EINT2_IRQHandler          ; 12: External Input 2
                DCD     EINT3_IRQHandler          ; 13: External Input 3
                DCD     EINT4_IRQHandler          ; 14: External Input 4
                DCD     EINT5_IRQHandler          ; 15: External Input 5
                DCD     GPA_IRQHandler            ; 16: GPIO Port A
                DCD     GPB_IRQHandler            ; 17: GPIO Port B
                DCD     GPC_IRQHandler            ; 18: GPIO Port C
                DCD     GPD_IRQHandler            ; 19: GPIO Port D
                DCD     Default_Handler           ; 20: GPIO Port E
                DCD     GPF_IRQHandler            ; 21: GPIO Port F
                DCD     QSPI0_IRQHandler          ; 22: QSPI0
                DCD     SPI0_IRQHandler           ; 23: SPI0
                DCD     BRAKE0_IRQHandler         ; 24:
                DCD     EPWM0P0_IRQHandler        ; 25:
                DCD     EPWM0P1_IRQHandler        ; 26:
                DCD     EPWM0P2_IRQHandler        ; 27:
                DCD     BRAKE1_IRQHandler         ; 28:
                DCD     EPWM1P0_IRQHandler        ; 29:
                DCD     EPWM1P1_IRQHandler        ; 30:
                DCD     EPWM1P2_IRQHandler        ; 31:
                DCD     TMR0_IRQHandler           ; 32: Timer 0
                DCD     TMR1_IRQHandler           ; 33: Timer 1
                DCD     TMR2_IRQHandler           ; 34: Timer 2
                DCD     TMR3_IRQHandler           ; 35: Timer 3
                DCD     UART0_IRQHandler          ; 36: UART0
                DCD     UART1_IRQHandler          ; 37: UART1
                DCD     I2C0_IRQHandler           ; 38: I2C0
                DCD     I2C1_IRQHandler           ; 39: I2C1
                DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
                DCD     DAC_IRQHandler            ; 41: DAC
                DCD     EADC00_IRQHandler         ; 42: EADC0 interrupt source 0
                DCD     EADC01_IRQHandler         ; 43: EADC0 interrupt source 1
                DCD     ACMP01_IRQHandler         ; 44: ACMP0 and ACMP1
                DCD     Default_Handler           ; 45: Reserved
                DCD     EADC02_IRQHandler         ; 46: EADC0 interrupt source 2
                DCD     EADC03_IRQHandler         ; 47: EADC0 interrupt source 3
                DCD     UART2_IRQHandler          ; 48: UART2
                DCD     UART3_IRQHandler          ; 49: UART3
                DCD     Default_Handler           ; 50:
                DCD     SPI1_IRQHandler           ; 51: SPI1
                DCD     Default_Handler           ; 52:
                DCD     Default_Handler           ; 53:
                DCD     Default_Handler           ; 54:
                DCD     Default_Handler           ; 55:
                DCD     Default_Handler           ; 56:
                DCD     Default_Handler           ; 57:
                DCD     Default_Handler           ; 58:
                DCD     Default_Handler           ; 59:
                DCD     Default_Handler           ; 60:
                DCD     Default_Handler           ; 61:
                DCD     Default_Handler           ; 62:
                DCD     Default_Handler           ; 63:
                DCD     Default_Handler           ; 64:
                DCD     Default_Handler           ; 65:
                DCD     Default_Handler           ; 66:
                DCD     Default_Handler           ; 67:
                DCD     I2S0_IRQHandler           ; 68: I2S0
                DCD     Default_Handler           ; 69:
                DCD     Default_Handler           ; 70:
                DCD     CRYPTO_IRQHandler         ; 71: CRYPTO
                DCD     Default_Handler           ; 72:
                DCD     EINT6_IRQHandler          ; 73:
                DCD     UART4_IRQHandler          ; 74: UART4
                DCD     UART5_IRQHandler          ; 75: UART5
                DCD     Default_Handler           ; 76:
                DCD     Default_Handler           ; 77:
                DCD     Default_Handler           ; 78:
                DCD     Default_Handler           ; 79:
                DCD     Default_Handler           ; 80:
                DCD     Default_Handler           ; 81:
                DCD     I2C2_IRQHandler           ; 82: I2C2
                DCD     Default_Handler           ; 83:
                DCD     QEI0_IRQHandler           ; 84: QEI0
                DCD     QEI1_IRQHandler           ; 85: QEI1
                DCD     ECAP0_IRQHandler          ; 86: ECAP0
                DCD     Default_Handler           ; 87:
                DCD     Default_Handler           ; 88:
                DCD     EINT7_IRQHandler          ; 89:
                DCD     Default_Handler           ; 90:
                DCD     Default_Handler           ; 91:
                DCD     Default_Handler           ; 92:
                DCD     Default_Handler           ; 93:
                DCD     Default_Handler           ; 94:
                DCD     Default_Handler           ; 95:
                DCD     Default_Handler           ; 96:
                DCD     Default_Handler           ; 97:
                DCD     Default_Handler           ; 98:
                DCD     Default_Handler           ; 99:
                DCD     Default_Handler           ; 100:
                DCD     Default_Handler           ; 101:
                DCD     Default_Handler           ; 102:
                DCD     Default_Handler           ; 103:
                DCD     EADC10_IRQHandler         ; 104: EADC1 interrupt source 0
                DCD     EADC11_IRQHandler         ; 105: EADC1 interrupt source 1
                DCD     EADC12_IRQHandler         ; 106: EADC1 interrupt source 2
                DCD     EADC13_IRQHandler         ; 107: EADC1 interrupt source 3
                DCD     Default_Handler           ; 108:


__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                ; Unlock Register
                LDR     R0, =0x40000100
                LDR     R1, =0x59
                STR     R1, [R0]
                LDR     R1, =0x16
                STR     R1, [R0]
                LDR     R1, =0x88
                STR     R1, [R0]

                LDR     R0, =SystemInit
                BLX     R0

                ; Init POR
                ; LDR     R2, =0x40000024
                ; LDR     R1, =0x00005AA5
                ; STR     R1, [R2]

                ; Lock
                LDR     R0, =0x40000100
                LDR     R1, =0
                STR     R1, [R0]

                LDR     R0, =__main
                BX      R0

                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  IRC_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  RAMPE_IRQHandler          [WEAK]
                EXPORT  CKFAIL_IRQHandler         [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  TAMPER_IRQHandler         [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT4_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPA_IRQHandler            [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPD_IRQHandler            [WEAK]
                EXPORT  GPF_IRQHandler            [WEAK]
                EXPORT  QSPI0_IRQHandler          [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  BRAKE0_IRQHandler         [WEAK]
                EXPORT  EPWM0P0_IRQHandler        [WEAK]
                EXPORT  EPWM0P1_IRQHandler        [WEAK]
                EXPORT  EPWM0P2_IRQHandler        [WEAK]
                EXPORT  BRAKE1_IRQHandler         [WEAK]
                EXPORT  EPWM1P0_IRQHandler        [WEAK]
                EXPORT  EPWM1P1_IRQHandler        [WEAK]
                EXPORT  EPWM1P2_IRQHandler        [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  DAC_IRQHandler            [WEAK]
                EXPORT  EADC00_IRQHandler         [WEAK]
                EXPORT  EADC01_IRQHandler         [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  EADC02_IRQHandler         [WEAK]
                EXPORT  EADC03_IRQHandler         [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  I2S0_IRQHandler           [WEAK]
                EXPORT  CRYPTO_IRQHandler         [WEAK]
                EXPORT  EINT6_IRQHandler          [WEAK]
                EXPORT  UART4_IRQHandler          [WEAK]
                EXPORT  UART5_IRQHandler          [WEAK]
                EXPORT  I2C2_IRQHandler           [WEAK]
                EXPORT  QEI0_IRQHandler           [WEAK]
                EXPORT  QEI1_IRQHandler           [WEAK]
                EXPORT  ECAP0_IRQHandler          [WEAK]
                EXPORT  EINT7_IRQHandler          [WEAK]
                EXPORT  EADC10_IRQHandler         [WEAK]
                EXPORT  EADC11_IRQHandler         [WEAK]
                EXPORT  EADC12_IRQHandler         [WEAK]
                EXPORT  EADC13_IRQHandler         [WEAK]

Default__IRQHandler
BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CKFAIL_IRQHandler
RTC_IRQHandler
TAMPER_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
GPA_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPD_IRQHandler
GPF_IRQHandler
QSPI0_IRQHandler
SPI0_IRQHandler
BRAKE0_IRQHandler
EPWM0P0_IRQHandler
EPWM0P1_IRQHandler
EPWM0P2_IRQHandler
BRAKE1_IRQHandler
EPWM1P0_IRQHandler
EPWM1P1_IRQHandler
EPWM1P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA_IRQHandler
DAC_IRQHandler
EADC00_IRQHandler
EADC01_IRQHandler
ACMP01_IRQHandler
EADC02_IRQHandler
EADC03_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
SPI1_IRQHandler
I2S0_IRQHandler
CRYPTO_IRQHandler
EINT6_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
I2C2_IRQHandler
QEI0_IRQHandler
QEI1_IRQHandler
ECAP0_IRQHandler
EINT7_IRQHandler
EADC10_IRQHandler
EADC11_IRQHandler
EADC12_IRQHandler
EADC13_IRQHandler



                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
;/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
