/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/**
   @addtogroup REGISTER Control Register
   @{
*/

/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct
{


    /**
     * @var CLK_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTEN     |HXT Enable Bit (Write Protect)
     * |        |          |0 = 4~24 MHz external high speed crystal (HXT) Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal (HXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: When HXT is enabled, GPF.2 and GPF.3 must be set as input mode.
     * |[1]     |LXTEN     |LXT Enable Bit (Write Protect)
     * |        |          |0 = 32.768 kHz external low speed crystal (LXT) Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal (LXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: When LXT is enabled, GPF.4 and GPF.5 must be set as input mode.
     * |[2]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 38.4 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 38.4 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |PDWKDLY   |Enable the Wake-up Delay Counter (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip works at 4~24 MHz external high speed crystal oscillator (HXT), and 256 clock cycles when chip works at 48 MHz internal high speed RC oscillator (HIRC).
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note 1: The interrupt will occur when both PDWKIF and PDWKIEN are high.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
     * |        |          |The flag is set if any wake-up source is occurred. Refer Power Modes and Wake-up Sources chapter.
     * |        |          |Note 1: Write 1 to clear the bit to 0.
     * |        |          |Note 2: This bit works only if PDWKIEN (CLK_PWRCTL[5]) is set to 1.
     * |[7]     |PDEN      |System Power-down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |In Power-down mode, HXT and the HIRC will be disabled in this mode, but LXT and LIRC are not controlled by Power-down mode.
     * |        |          |In Power-down mode, the PLL and system clock are disabled, and ignored the clock source selection
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from LXT or LIRC.
     * |        |          |0 = Chip will not enter Power-down mode after CPU sleep command WFI.
     * |        |          |1 = Chip enters Power-down mode after CPU sleep command WFI.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17:16] |HIRCSTBS  |HIRC Stable Count Select (Write Protect)
     * |        |          |00 = HIRC stable count = 512 clocks.
     * |        |          |01 = HIRC stable count = 1024 clocks.
     * |        |          |10 = HIRC stable count = 2048 clocks.
     * |        |          |11 = HIRC stable count = 256 clocks.
     * |        |          |Others: Reserved
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[22:20] |HXTGAIN   |HXT Gain Control Bit (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally
     * |        |          |If gain control is enabled, crystal will consume more power than gain control off.
     * |        |          |000 = HXT frequency is from 4 MHz to 8 MHz.
     * |        |          |001 = HXT frequency is from 8 MHz to 12 MHz.
     * |        |          |010 = HXT frequency is from 12 MHz to 16 MHz.
     * |        |          |011 = HXT frequency is from 16 MHz to 24 MHz.
     * |        |          |000 = HXT frequency is from 4 MHz to 8 MHz. (Crystal)
     * |        |          |001 = HXT frequency is from 8 MHz to 12 MHz. (Crystal)
     * |        |          |010 = HXT frequency is from 12 MHz to 16 MHz. (Crystal)
     * |        |          |011 = HXT frequency is from 16 MHz to 24 MHz. (Crystal)
     * |        |          |100 = HXT frequency is from 4 MHz to 8 MHz. (Resonator)
     * |        |          |101 = HXT frequency is from 8 MHz to 12 MHz. (Resonator)
     * |        |          |110 = HXT frequency is from 12 MHz to 16 MHz. (Resonator)
     * |        |          |111 = HXT frequency is from 16 MHz to 24 MHz. (Resonator)
     * |        |          |Others: Reserved
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[31]    |HXTMD     |HXT Bypass Mode (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |0 = HXT work as crystal mode. PF.2 and PF.3 are configured as external high speed crystal (HXT) pins.
     * |        |          |1 = HXT works as external clock mode. PF.3 is configured as external clock input pin.
     * |        |          |Note 1: When HXTMD = 1, PF.3 MFP should be setting as GPIO mode
     * |        |          |The DC characteristic of XT1_IN is the same as GPIO.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGCTL register.
     * @var CLK_T::AHBCLK
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |PDMACKEN  |PDMA Controller Clock Enable Bit
     * |        |          |0 = PDMA peripheral clock Disabled.
     * |        |          |1 = PDMA peripheral clock Enabled.
     * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     * |[4]     |STCLKEN   |Cortex-M4 SysTick Clock Enable Bit
     * |        |          |0 = Cortex-M4 sys tick clock Disabled.
     * |        |          |1 = Cortex-M4 sys tick clock Enabled.
     * |[7]     |CRCCKEN   |CRC Generator Controller Clock Enable Bit
     * |        |          |0 = CRC peripheral clock Disabled.
     * |        |          |1 = CRC peripheral clock Enabled.
     * |[15]    |FMCIDLE   |Flash Memory Controller Clock Enable Bit in IDLE Mode
     * |        |          |0 = FMC clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMC clock Enabled when chip is under IDLE mode.
     * |[19]    |TRACECKEN |TRACE Clock Enable Bit
     * |        |          |0 = TRACE clock Disabled.
     * |        |          |1 = TRACE clock Enabled.
     * |[23]    |GPICKEN   |GPIOI Clock Enable Bit
     * |        |          |0 = GPIOI port clock Disabled.
     * |        |          |1 = GPIOI port clock Enabled.
     * |[24]    |GPACKEN   |GPIOA Clock Enable Bit
     * |        |          |0 = GPIOA port clock Disabled.
     * |        |          |1 = GPIOA port clock Enabled.
     * |[25]    |GPBCKEN   |GPIOB Clock Enable Bit
     * |        |          |0 = GPIOB port clock Disabled.
     * |        |          |1 = GPIOB port clock Enabled.
     * |[26]    |GPCCKEN   |GPIOC Clock Enable Bit
     * |        |          |0 = GPIOC port clock Disabled.
     * |        |          |1 = GPIOC port clock Enabled.
     * |[27]    |GPDCKEN   |GPIOD Clock Enable Bit
     * |        |          |0 = GPIOD port clock Disabled.
     * |        |          |1 = GPIOD port clock Enabled.
     * |[28]    |GPECKEN   |GPIOE Clock Enable Bit
     * |        |          |0 = GPIOE port clock Disabled.
     * |        |          |1 = GPIOE port clock Enabled.
     * |[29]    |GPFCKEN   |GPIOF Clock Enable Bit
     * |        |          |0 = GPIOF port clock Disabled.
     * |        |          |1 = GPIOF port clock Enabled.
     * |[30]    |GPGCKEN   |GPIOG Clock Enable Bit
     * |        |          |0 = GPIOG port clock Disabled.
     * |        |          |1 = GPIOG port clock Enabled.
     * |[31]    |GPHCKEN   |GPIOH Clock Enable Bit
     * |        |          |0 = GPIOH port clock Disabled.
     * |        |          |1 = GPIOH port clock Enabled.
     * @var CLK_T::APBCLK0
     * Offset: 0x08  APB Devices Clock Enable Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Watchdog timer clock Disabled.
     * |        |          |1 = Watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |RTCCKEN   |Real-time-clock APB Interface Clock Enable Bit
     * |        |          |This bit is used to control the RTC APB clock only.
     * |        |          |0 = RTC clock Disabled.
     * |        |          |1 = RTC clock Enabled.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2CKEN  |Timer2 Clock Enable Bit
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3CKEN  |Timer3 Clock Enable Bit
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |CLKOCKEN  |CLKO Clock Enable Bit
     * |        |          |0 = CLKO clock Disabled.
     * |        |          |1 = CLKO clock Enabled.
     * |[7]     |ACMP01CKEN|Analog Comparator 0/1 Clock Enable Bit
     * |        |          |0 = Analog comparator 0/1 clock Disabled.
     * |        |          |1 = Analog comparator 0/1 clock Enabled.
     * |[8]     |I2C0CKEN  |I2C0 Clock Enable Bit
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[9]     |I2C1CKEN  |I2C1 Clock Enable Bit
     * |        |          |0 = I2C1 clock Disabled.
     * |        |          |1 = I2C1 clock Enabled.
     * |[13]    |SPI0CKEN  |SPI0 Clock Enable Bit
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[14]    |SPI1CKEN  |SPI1 Clock Enable Bit
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[16]    |UART0CKEN |UART0 Clock Enable Bit
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Bit
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[18]    |UART2CKEN |UART2 Clock Enable Bit
     * |        |          |0 = UART2 clock Disabled.
     * |        |          |1 = UART2 clock Enabled.
     * |[19]    |UART3CKEN |UART3 Clock Enable Bit
     * |        |          |0 = UART3 clock Disabled.
     * |        |          |1 = UART3 clock Enabled.
     * |[20]    |UART4CKEN |UART4 Clock Enable Bit
     * |        |          |0 = UART4 clock Disabled.
     * |        |          |1 = UART4 clock Enabled.
     * |[21]    |UART5CKEN |UART5 Clock Enable Bit
     * |        |          |0 = UART5 clock Disabled.
     * |        |          |1 = UART5 clock Enabled.
     * |[28]    |EADCCKEN  |Enhanced Analog-digital-converter (EADC) Clock Enable Bit
     * |        |          |0 = EADC clock Disabled.
     * |        |          |1 = EADC clock Enabled.
     * @var CLK_T::APBCLK1
     * Offset: 0x0C  APB Devices Clock Enable Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[12]    |DACCKEN   |DAC Clock Enable Bit
     * |        |          |0 = DAC clock Disabled.
     * |        |          |1 = DAC clock Enabled.
     * |[15]    |CIR0CKEN  |CIR0 Clock Enable Bit
     * |        |          |0 = CIR0 clock Disabled.
     * |        |          |1 = CIR0 clock Enabled.
     * |[16]    |EPWM0CKEN |EPWM0 Clock Enable Bit
     * |        |          |0 = EPWM0 clock Disabled.
     * |        |          |1 = EPWM0 clock Enabled.
     * |[17]    |EPWM1CKEN |EPWM1 Clock Enable Bit
     * |        |          |0 = EPWM1 clock Disabled.
     * |        |          |1 = EPWM1 clock Enabled.
     * |[18]    |BPWM0CKEN |BPWM0 Clock Enable Bit
     * |        |          |0 = BPWM0 clock Disabled.
     * |        |          |1 = BPWM0 clock Enabled.
     * |[19]    |BPWM1CKEN |BPWM1 Clock Enable Bit
     * |        |          |0 = BPWM1 clock Disabled.
     * |        |          |1 = BPWM1 clock Enabled.
     * |[24]    |PRNGCKEN  |PRNG Clock Enable Bit
     * |        |          |0 = PRNG clock Disabled.
     * |        |          |1 = PRNG clock Enabled.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |Before clock switching, the related clock sources (both pre-select and new-select) must be turned on.
     * |        |          |The default value is reloaded from the value of CFOSC (CONFIG0[26:24]) in user configuration register of Flash controller by any reset
     * |        |          |Therefore the default value is either 000b or 111b.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PLL.
     * |        |          |011 = Clock source from LIRC.
     * |        |          |111 = Clock source from HIRC.
     * |        |          |Other = Reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5:3]   |STCLKSEL  |Cortex-M4 SysTick Clock Source Selection (Write Protect)
     * |        |          |If SYST_CTRL[2]=0, SysTick uses listed clock source below.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from HXT/2.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |111 = Clock source from HIRC/2.
     * |        |          |Note 1: if SysTick clock source is not from HCLK (i.e. SYST_CTRL[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |00 = Reserved.
     * |        |          |01 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM0 pin.
     * |        |          |101 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from external clock TM1 pin.
     * |        |          |101 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[18:16] |TMR2SEL   |TIMER2 Clock Source Selection
     * |        |          |000 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM2 pin.
     * |        |          |101 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[22:20] |TMR3SEL   |TIMER3 Clock Source Selection
     * |        |          |000 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM3 pin.
     * |        |          |101 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |111 = Clock source from internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[25:24] |UART0SEL  |UART0 Clock Source Selection
    * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[27:26] |UART1SEL  |UART1 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[29:28] |CLKOSEL   |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[31:30] |WWDTSEL   |Window Watchdog Timer Clock Source Selection
     * |        |          |10 = Clock source from HCLK/2048.
     * |        |          |11 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |Others = Reserved.
     * @var CLK_T::CLKSEL2
     * Offset: 0x18  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EPWM0SEL  |EPWM0 Clock Source Selection
     * |        |          |The peripheral clock source of EPWM0 is defined by EPWM0SEL.
     * |        |          |0 = Clock source from HCLK.
     * |        |          |1 = Clock source from PCLK0.
     * |[1]     |EPWM1SEL  |EPWM1 Clock Source Selection
     * |        |          |The peripheral clock source of EPWM1 is defined by EPWM1SEL.
     * |        |          |0 = Clock source from HCLK.
     * |        |          |1 = Clock source from PCLK1.
     * |[5:4]   |SPI0SEL   |SPI0 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK1.
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[7:6]   |SPI1SEL   |SPI1 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from PCLK0.
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[8]     |BPWM0SEL  |BPWM0 Clock Source Selection
     * |        |          |The peripheral clock source of BPWM0 is defined by BPWM0SEL.
     * |        |          |0 = Clock source from HCLK.
     * |        |          |1 = Clock source from PCLK0.
     * |[9]     |BPWM1SEL  |BPWM1 Clock Source Selection
     * |        |          |The peripheral clock source of BPWM1 is defined by BPWM1SEL.
     * |        |          |0 = Clock source from HCLK.
     * |        |          |1 = Clock source from PCLK1.
     * |[26:24] |CIR0SEL   |CIR0 Clock Source Selection
     * |        |          |000 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |001 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |010 = Clock source from Timer0 clock output (TM0).
     * |        |          |011 = Clock source from internal low speed RC oscillator (LIRC).
     * |        |          |100 = Clock source from internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * @var CLK_T::CLKSEL3
     * Offset: 0x1C  Clock Source Select Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[25:24] |UART2SEL  |UART2 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[27:26] |UART3SEL  |UART3 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[29:28] |UART4SEL  |UART4 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * |[31:30] |UART5SEL  |UART5 Clock Source Selection
     * |        |          |00 = Clock source from external high speed crystal oscillator (HXT).
     * |        |          |01 = Clock source from PLL.
     * |        |          |10 = Clock source from external low speed crystal oscillator (LXT).
     * |        |          |11 = Clock source from internal high speed RC oscillator (HIRC).
     * @var CLK_T::CLKDIV0
     * Offset: 0x20  Clock Divider Number Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[11:8]  |UART0DIV  |UART0 Clock Divide Number From UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |[15:12] |UART1DIV  |UART1 Clock Divide Number From UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |[23:16] |EADCDIV   |EADC Clock Divide Number From EADC Clock Source
     * |        |          |EADC clock frequency = (EADC clock source frequency) / (EADCDIV + 1).
     * @var CLK_T::CLKDIV4
     * Offset: 0x30  Clock Divider Number Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART2DIV  |UART2 Clock Divide Number From UART2 Clock Source
     * |        |          |UART2 clock frequency = (UART2 clock source frequency) / (UART2DIV + 1).
     * |[7:4]   |UART3DIV  |UART3 Clock Divide Number From UART3 Clock Source
     * |        |          |UART3 clock frequency = (UART3 clock source frequency) / (UART3DIV + 1).
     * |[11:8]  |UART4DIV  |UART4 Clock Divide Number From UART4 Clock Source
     * |        |          |UART4 clock frequency = (UART4 clock source frequency) / (UART4DIV + 1).
     * |[15:12] |UART5DIV  |UART5 Clock Divide Number From UART5 Clock Source
     * |        |          |UART5 clock frequency = (UART5 clock source frequency) / (UART5DIV + 1).
     * |[31:24] |TRACEDIV  |Cortex M4 ETM Trace Clock Divide Number From ETM Trace Clock Source
     * |        |          |Cortex M4 ETM TRACE clock frequency = (HCLK clock source frequency) / (TRACEDIV + 1).
     * @var CLK_T::PCLKDIV
     * Offset: 0x34  APB Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |APB0DIV   |APB0 Clock Divider
     * |        |          |APB0 clock can be divided from HCLK
     * |        |          |000: PCLK0 = HCLK.
     * |        |          |001: PCLK0 = 1/2 HCLK.
     * |        |          |010: PCLK0 = 1/4 HCLK.
     * |        |          |011: PCLK0 = 1/8 HCLK.
     * |        |          |100: PCLK0 = 1/16 HCLK.
     * |        |          |Others: Reserved.
     * |[6:4]   |APB1DIV   |APB1 Clock Divider
     * |        |          |APB1 clock can be divided from HCLK
     * |        |          |000: PCLK1 = HCLK.
     * |        |          |001: PCLK1 = 1/2 HCLK.
     * |        |          |010: PCLK1 = 1/4 HCLK.
     * |        |          |011: PCLK1 = 1/8 HCLK.
     * |        |          |100: PCLK1 = 1/16 HCLK.
     * |        |          |Others: Reserved.
     * @var CLK_T::PLLCTL
     * Offset: 0x40  PLL Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FBDIV     |PLL Feedback Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13:9]  |INDIV     |PLL Input Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[15:14] |OUTDIV    |PLL Output Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |PD        |Power-down Mode (Write Protect)
     * |        |          |If set the PDEN bit to 1 in CLK_PWRCTL register, the PLL will enter Power-down mode, too.
     * |        |          |0 = PLL is in normal mode.
     * |        |          |1 = PLL is in Power-down mode (default).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17]    |BP        |PLL Bypass Control (Write Protect)
     * |        |          |0 = PLL is in normal mode (default).
     * |        |          |1 = PLL clock output is same as PLL input clock FIN.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[18]    |OE        |PLL OE (FOUT Enable) Pin Control (Write Protect)
     * |        |          |0 = PLL FOUT Enabled.
     * |        |          |1 = PLL FOUT is fixed low.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[19]    |PLLSRC    |PLL Source Clock Selection (Write Protect)
     * |        |          |0 = PLL source clock from 4~24 MHz external high-speed crystal oscillator (HXT).
     * |        |          |1 = PLL source clock from 48 MHz internal high-speed oscillator divided by 4 (HIRC/4).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[23]    |STBSEL    |PLL Stable Counter Selection (Write Protect)
     * |        |          |0 = PLL stable time is 6144 PLL source clock (suitable for source clock is equal to or less than 12 MHz).
     * |        |          |1 = PLL stable time is 16128 PLL source clock (suitable for source clock is larger than 12 MHz).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::STATUS
     * Offset: 0x50  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTSTB    |HXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock is not stable or disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock is stable and enabled.
     * |[1]     |LXTSTB    |LXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is not stable or disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock is stabled and enabled.
     * |[2]     |PLLSTB    |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable and enabled.
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 38.4 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 38.4 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |[7]     |CLKSFAIL  |Clock Switching Fail Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 0
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * @var CLK_T::CLKOCTL
     * Offset: 0x60  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is:
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     * |[6]     |CLK1HZEN  |Clock Output 1Hz Enable Bit
     * |        |          |0 = 1 Hz clock output for 32.768 kHz or 38 kHz frequency compensation Disabled.
     * |        |          |1 = 1 Hz clock output for 32.768 kHz or 38 kHz frequency compensation Enabled.
     * |        |          |Note: Output for 32.768 kHz(LXT) or 38 kHz(LIRC) based on RTCCKSEL(RTC_LXTCTL[7]).
     * @var CLK_T::CLKDCTL
     * Offset: 0x70  Clock Fail Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDEN   |HXT Clock Fail Detector Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail detector Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail detector Enabled.
     * |[5]     |HXTFIEN   |HXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail interrupt Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock fail interrupt Enabled.
     * |[12]    |LXTFDEN   |LXT Clock Fail Detector Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Enabled.
     * |[13]    |LXTFIEN   |LXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Enabled.
     * |[16]    |HXTFQDEN  |HXT Clock Frequency Range Detector Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector Enabled.
     * |[17]    |HXTFQIEN  |HXT Clock Frequency Range Detector Interrupt Enable Bit
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Disabled.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Enabled.
     * @var CLK_T::CLKDSTS
     * Offset: 0x74  Clock Fail Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HXTFIF    |HXT Clock Fail Interrupt Flag
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock is normal.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock stops.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * |[1]     |LXTFIF    |LXT Clock Fail Interrupt Flag
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is normal.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) stops.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * |[8]     |HXTFQIF   |HXT Clock Frequency Range Detector Interrupt Flag
     * |        |          |0 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency is normal.
     * |        |          |1 = 4~24 MHz external high speed crystal oscillator (HXT) clock frequency is abnormal.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * @var CLK_T::CDUPB
     * Offset: 0x78  Clock Frequency Range Detector Upper Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10:0]  |UPERBD    |HXT Clock Frequency Range Detector Upper Boundary Value
     * |        |          |The bits define the maximum value of frequency range detector window.
     * |        |          |When HXT frequency higher than this maximum frequency value, the HXT Clock Frequency Range Detector Interrupt Flag will be set to 1.
     * @var CLK_T::CDLOWB
     * Offset: 0x7C  Clock Frequency Range Detector Lower Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10:0]  |LOWERBD   |HXT Clock Frequency Range Detector Lower Boundary Value
     * |        |          |The bits define the minimum value of frequency range detector window.
     * |        |          |When HXT frequency lower than this minimum frequency value, the HXT Clock Frequency Range Detector Interrupt Flag will be set to 1.
     * @var CLK_T::PMUCTL
     * Offset: 0x90  Power Manager Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PDMSEL    |Power-down Mode Selection (Write Protect)
     * |        |          |This is a protected bit. Please refer to open lock sequence to program it.
     * |        |          |These bits control chip Power-down mode grade selection when CPU executes WFI/WFE instruction.
     * |        |          |000 = Power-down mode is selected. (NPD)
     * |        |          |001 = Reserved.
     * |        |          |010 = Reserved.
     * |        |          |011 = Reserved.
     * |        |          |100 = Reserved.
     * |        |          |101 = Reserved..
     * |        |          |110 = Reserved.
     * |        |          |111 = Reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     */
    __IO uint32_t PWRCTL;               /*!< [0x0000] System Power-down Control Register                                */
    __IO uint32_t AHBCLK;               /*!< [0x0004] AHB Devices Clock Enable Control Register                         */
    __IO uint32_t APBCLK0;              /*!< [0x0008] APB Devices Clock Enable Control Register 0                       */
    __IO uint32_t APBCLK1;              /*!< [0x000c] APB Devices Clock Enable Control Register 1                       */
    __IO uint32_t CLKSEL0;              /*!< [0x0010] Clock Source Select Control Register 0                            */
    __IO uint32_t CLKSEL1;              /*!< [0x0014] Clock Source Select Control Register 1                            */
    __IO uint32_t CLKSEL2;              /*!< [0x0018] Clock Source Select Control Register 2                            */
    __IO uint32_t CLKSEL3;              /*!< [0x001c] Clock Source Select Control Register 3                            */
    __IO uint32_t CLKDIV0;              /*!< [0x0020] Clock Divider Number Register 0                                   */
    __I  uint32_t RESERVE0[3];
    __IO uint32_t CLKDIV4;              /*!< [0x0030] Clock Divider Number Register 4                                   */
    __IO uint32_t PCLKDIV;              /*!< [0x0034] APB Clock Divider Register                                        */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t PLLCTL;                /*!< [0x0040] PLL Control Register                                             */
    __I  uint32_t RESERVE2[3];
    __I  uint32_t STATUS;                /*!< [0x0050] Clock Status Monitor Register                                    */
    __I  uint32_t RESERVE3[3];
    __IO uint32_t CLKOCTL;               /*!< [0x0060] Clock Output Control Register                                    */
    __I  uint32_t RESERVE4[3];
    __IO uint32_t CLKDCTL;               /*!< [0x0070] Clock Fail Detector Control Register                             */
    __IO uint32_t CLKDSTS;               /*!< [0x0074] Clock Fail Detector Status Register                              */
    __IO uint32_t CDUPB;                 /*!< [0x0078] Clock Frequency Range Detector Upper Boundary Register           */
    __IO uint32_t CDLOWB;                /*!< [0x007c] Clock Frequency Range Detector Lower Boundary Register           */
    __I  uint32_t RESERVE5[4];
    __IO uint32_t PMUCTL;                /*!< [0x0090] Power Manager Control Register                                   */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_HXTEN_Pos            (0)                                             /*!< CLK_T::PWRCTL: HXTEN Position          */
#define CLK_PWRCTL_HXTEN_Msk            (0x1ul << CLK_PWRCTL_HXTEN_Pos)                 /*!< CLK_T::PWRCTL: HXTEN Mask              */

#define CLK_PWRCTL_LXTEN_Pos            (1)                                             /*!< CLK_T::PWRCTL: LXTEN Position          */
#define CLK_PWRCTL_LXTEN_Msk            (0x1ul << CLK_PWRCTL_LXTEN_Pos)                 /*!< CLK_T::PWRCTL: LXTEN Mask              */

#define CLK_PWRCTL_HIRCEN_Pos           (2)                                             /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk           (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_LIRCEN_Pos           (3)                                             /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk           (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_PDWKDLY_Pos          (4)                                             /*!< CLK_T::PWRCTL: PDWKDLY Position        */
#define CLK_PWRCTL_PDWKDLY_Msk          (0x1ul << CLK_PWRCTL_PDWKDLY_Pos)               /*!< CLK_T::PWRCTL: PDWKDLY Mask            */

#define CLK_PWRCTL_PDWKIEN_Pos          (5)                                             /*!< CLK_T::PWRCTL: PDWKIEN Position        */
#define CLK_PWRCTL_PDWKIEN_Msk          (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)               /*!< CLK_T::PWRCTL: PDWKIEN Mask            */

#define CLK_PWRCTL_PDWKIF_Pos           (6)                                             /*!< CLK_T::PWRCTL: PDWKIF Position         */
#define CLK_PWRCTL_PDWKIF_Msk           (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                /*!< CLK_T::PWRCTL: PDWKIF Mask             */

#define CLK_PWRCTL_PDEN_Pos             (7)                                             /*!< CLK_T::PWRCTL: PDEN Position           */
#define CLK_PWRCTL_PDEN_Msk             (0x1ul << CLK_PWRCTL_PDEN_Pos)                  /*!< CLK_T::PWRCTL: PDEN Mask               */

#define CLK_PWRCTL_HIRCSTBS_Pos         (16)                                            /*!< CLK_T::PWRCTL: HIRCSTBS Position       */
#define CLK_PWRCTL_HIRCSTBS_Msk         (0x3ul << CLK_PWRCTL_HIRCSTBS_Pos)              /*!< CLK_T::PWRCTL: HIRCSTBS Mask           */

#define CLK_PWRCTL_HXTGAIN_Pos          (20)                                            /*!< CLK_T::PWRCTL: HXTGAIN Position        */
#define CLK_PWRCTL_HXTGAIN_Msk          (0x7ul << CLK_PWRCTL_HXTGAIN_Pos)               /*!< CLK_T::PWRCTL: HXTGAIN Mask            */

#define CLK_PWRCTL_HXTMD_Pos            (31)                                            /*!< CLK_T::PWRCTL: HXTMD Position          */
#define CLK_PWRCTL_HXTMD_Msk            (0x1ul << CLK_PWRCTL_HXTMD_Pos)                 /*!< CLK_T::PWRCTL: HXTMD Mask              */

#define CLK_AHBCLK_PDMACKEN_Pos         (1)                                             /*!< CLK_T::AHBCLK: PDMACKEN Position       */
#define CLK_AHBCLK_PDMACKEN_Msk         (0x1ul << CLK_AHBCLK_PDMACKEN_Pos)              /*!< CLK_T::AHBCLK: PDMACKEN Mask           */

#define CLK_AHBCLK_ISPCKEN_Pos          (2)                                             /*!< CLK_T::AHBCLK: ISPCKEN Position        */
#define CLK_AHBCLK_ISPCKEN_Msk          (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)               /*!< CLK_T::AHBCLK: ISPCKEN Mask            */

#define CLK_AHBCLK_STCLKEN_Pos          (4)                                             /*!< CLK_T::AHBCLK: STCLKEN Position        */
#define CLK_AHBCLK_STCLKEN_Msk          (0x1ul << CLK_AHBCLK_STCLKEN_Pos)               /*!< CLK_T::AHBCLK: STCLKEN Mask            */

#define CLK_AHBCLK_CRCCKEN_Pos          (7)                                             /*!< CLK_T::AHBCLK: CRCCKEN Position        */
#define CLK_AHBCLK_CRCCKEN_Msk          (0x1ul << CLK_AHBCLK_CRCCKEN_Pos)               /*!< CLK_T::AHBCLK: CRCCKEN Mask            */

#define CLK_AHBCLK_FMCIDLE_Pos          (15)                                            /*!< CLK_T::AHBCLK: FMCIDLE Position        */
#define CLK_AHBCLK_FMCIDLE_Msk          (0x1ul << CLK_AHBCLK_FMCIDLE_Pos)               /*!< CLK_T::AHBCLK: FMCIDLE Mask            */

#define CLK_AHBCLK_TRACECKEN_Pos        (19)                                            /*!< CLK_T::AHBCLK: TRACECKEN Position      */
#define CLK_AHBCLK_TRACECKEN_Msk        (0x1ul << CLK_AHBCLK_TRACECKEN_Pos)             /*!< CLK_T::AHBCLK: TRACECKEN Mask          */

#define CLK_AHBCLK_GPICKEN_Pos          (23)                                            /*!< CLK_T::AHBCLK: GPICKEN Position        */
#define CLK_AHBCLK_GPICKEN_Msk          (0x1ul << CLK_AHBCLK_GPICKEN_Pos)               /*!< CLK_T::AHBCLK: GPICKEN Mask            */

#define CLK_AHBCLK_GPACKEN_Pos          (24)                                            /*!< CLK_T::AHBCLK: GPACKEN Position        */
#define CLK_AHBCLK_GPACKEN_Msk          (0x1ul << CLK_AHBCLK_GPACKEN_Pos)               /*!< CLK_T::AHBCLK: GPACKEN Mask            */

#define CLK_AHBCLK_GPBCKEN_Pos          (25)                                            /*!< CLK_T::AHBCLK: GPBCKEN Position        */
#define CLK_AHBCLK_GPBCKEN_Msk          (0x1ul << CLK_AHBCLK_GPBCKEN_Pos)               /*!< CLK_T::AHBCLK: GPBCKEN Mask            */

#define CLK_AHBCLK_GPCCKEN_Pos          (26)                                            /*!< CLK_T::AHBCLK: GPCCKEN Position        */
#define CLK_AHBCLK_GPCCKEN_Msk          (0x1ul << CLK_AHBCLK_GPCCKEN_Pos)               /*!< CLK_T::AHBCLK: GPCCKEN Mask            */

#define CLK_AHBCLK_GPDCKEN_Pos          (27)                                            /*!< CLK_T::AHBCLK: GPDCKEN Position        */
#define CLK_AHBCLK_GPDCKEN_Msk          (0x1ul << CLK_AHBCLK_GPDCKEN_Pos)               /*!< CLK_T::AHBCLK: GPDCKEN Mask            */

#define CLK_AHBCLK_GPECKEN_Pos          (28)                                            /*!< CLK_T::AHBCLK: GPECKEN Position        */
#define CLK_AHBCLK_GPECKEN_Msk          (0x1ul << CLK_AHBCLK_GPECKEN_Pos)               /*!< CLK_T::AHBCLK: GPECKEN Mask            */

#define CLK_AHBCLK_GPFCKEN_Pos          (29)                                            /*!< CLK_T::AHBCLK: GPFCKEN Position        */
#define CLK_AHBCLK_GPFCKEN_Msk          (0x1ul << CLK_AHBCLK_GPFCKEN_Pos)               /*!< CLK_T::AHBCLK: GPFCKEN Mask            */

#define CLK_AHBCLK_GPGCKEN_Pos          (30)                                            /*!< CLK_T::AHBCLK: GPGCKEN Position        */
#define CLK_AHBCLK_GPGCKEN_Msk          (0x1ul << CLK_AHBCLK_GPGCKEN_Pos)               /*!< CLK_T::AHBCLK: GPGCKEN Mask            */

#define CLK_AHBCLK_GPHCKEN_Pos          (31)                                            /*!< CLK_T::AHBCLK: GPHCKEN Position        */
#define CLK_AHBCLK_GPHCKEN_Msk          (0x1ul << CLK_AHBCLK_GPHCKEN_Pos)               /*!< CLK_T::AHBCLK: GPHCKEN Mask            */

#define CLK_APBCLK0_WDTCKEN_Pos         (0)                                             /*!< CLK_T::APBCLK0: WDTCKEN Position       */
#define CLK_APBCLK0_WDTCKEN_Msk         (0x1ul << CLK_APBCLK0_WDTCKEN_Pos)              /*!< CLK_T::APBCLK0: WDTCKEN Mask           */

#define CLK_APBCLK0_RTCCKEN_Pos         (1)                                             /*!< CLK_T::APBCLK0: RTCCKEN Position       */
#define CLK_APBCLK0_RTCCKEN_Msk         (0x1ul << CLK_APBCLK0_RTCCKEN_Pos)              /*!< CLK_T::APBCLK0: RTCCKEN Mask           */

#define CLK_APBCLK0_TMR0CKEN_Pos        (2)                                             /*!< CLK_T::APBCLK0: TMR0CKEN Position      */
#define CLK_APBCLK0_TMR0CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)             /*!< CLK_T::APBCLK0: TMR0CKEN Mask          */

#define CLK_APBCLK0_TMR1CKEN_Pos        (3)                                             /*!< CLK_T::APBCLK0: TMR1CKEN Position      */
#define CLK_APBCLK0_TMR1CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)             /*!< CLK_T::APBCLK0: TMR1CKEN Mask          */

#define CLK_APBCLK0_TMR2CKEN_Pos        (4)                                             /*!< CLK_T::APBCLK0: TMR2CKEN Position      */
#define CLK_APBCLK0_TMR2CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR2CKEN_Pos)             /*!< CLK_T::APBCLK0: TMR2CKEN Mask          */

#define CLK_APBCLK0_TMR3CKEN_Pos        (5)                                             /*!< CLK_T::APBCLK0: TMR3CKEN Position      */
#define CLK_APBCLK0_TMR3CKEN_Msk        (0x1ul << CLK_APBCLK0_TMR3CKEN_Pos)             /*!< CLK_T::APBCLK0: TMR3CKEN Mask          */

#define CLK_APBCLK0_CLKOCKEN_Pos        (6)                                             /*!< CLK_T::APBCLK0: CLKOCKEN Position      */
#define CLK_APBCLK0_CLKOCKEN_Msk        (0x1ul << CLK_APBCLK0_CLKOCKEN_Pos)             /*!< CLK_T::APBCLK0: CLKOCKEN Mask          */

#define CLK_APBCLK0_ACMP01CKEN_Pos      (7)                                             /*!< CLK_T::APBCLK0: ACMP01CKEN Position    */
#define CLK_APBCLK0_ACMP01CKEN_Msk      (0x1ul << CLK_APBCLK0_ACMP01CKEN_Pos)           /*!< CLK_T::APBCLK0: ACMP01CKEN Mask        */

#define CLK_APBCLK0_I2C0CKEN_Pos        (8)                                             /*!< CLK_T::APBCLK0: I2C0CKEN Position      */
#define CLK_APBCLK0_I2C0CKEN_Msk        (0x1ul << CLK_APBCLK0_I2C0CKEN_Pos)             /*!< CLK_T::APBCLK0: I2C0CKEN Mask          */

#define CLK_APBCLK0_I2C1CKEN_Pos        (9)                                             /*!< CLK_T::APBCLK0: I2C1CKEN Position      */
#define CLK_APBCLK0_I2C1CKEN_Msk        (0x1ul << CLK_APBCLK0_I2C1CKEN_Pos)             /*!< CLK_T::APBCLK0: I2C1CKEN Mask          */

#define CLK_APBCLK0_SPI0CKEN_Pos        (13)                                            /*!< CLK_T::APBCLK0: SPI0CKEN Position      */
#define CLK_APBCLK0_SPI0CKEN_Msk        (0x1ul << CLK_APBCLK0_SPI0CKEN_Pos)             /*!< CLK_T::APBCLK0: SPI0CKEN Mask          */

#define CLK_APBCLK0_SPI1CKEN_Pos        (14)                                            /*!< CLK_T::APBCLK0: SPI1CKEN Position      */
#define CLK_APBCLK0_SPI1CKEN_Msk        (0x1ul << CLK_APBCLK0_SPI1CKEN_Pos)             /*!< CLK_T::APBCLK0: SPI1CKEN Mask          */

#define CLK_APBCLK0_UART0CKEN_Pos       (16)                                            /*!< CLK_T::APBCLK0: UART0CKEN Position     */
#define CLK_APBCLK0_UART0CKEN_Msk       (0x1ul << CLK_APBCLK0_UART0CKEN_Pos)            /*!< CLK_T::APBCLK0: UART0CKEN Mask         */

#define CLK_APBCLK0_UART1CKEN_Pos       (17)                                            /*!< CLK_T::APBCLK0: UART1CKEN Position     */
#define CLK_APBCLK0_UART1CKEN_Msk       (0x1ul << CLK_APBCLK0_UART1CKEN_Pos)            /*!< CLK_T::APBCLK0: UART1CKEN Mask         */

#define CLK_APBCLK0_UART2CKEN_Pos       (18)                                            /*!< CLK_T::APBCLK0: UART2CKEN Position     */
#define CLK_APBCLK0_UART2CKEN_Msk       (0x1ul << CLK_APBCLK0_UART2CKEN_Pos)            /*!< CLK_T::APBCLK0: UART2CKEN Mask         */

#define CLK_APBCLK0_UART3CKEN_Pos       (19)                                            /*!< CLK_T::APBCLK0: UART3CKEN Position     */
#define CLK_APBCLK0_UART3CKEN_Msk       (0x1ul << CLK_APBCLK0_UART3CKEN_Pos)            /*!< CLK_T::APBCLK0: UART3CKEN Mask         */

#define CLK_APBCLK0_UART4CKEN_Pos       (20)                                            /*!< CLK_T::APBCLK0: UART4CKEN Position     */
#define CLK_APBCLK0_UART4CKEN_Msk       (0x1ul << CLK_APBCLK0_UART4CKEN_Pos)            /*!< CLK_T::APBCLK0: UART4CKEN Mask         */

#define CLK_APBCLK0_UART5CKEN_Pos       (21)                                            /*!< CLK_T::APBCLK0: UART5CKEN Position     */
#define CLK_APBCLK0_UART5CKEN_Msk       (0x1ul << CLK_APBCLK0_UART5CKEN_Pos)            /*!< CLK_T::APBCLK0: UART5CKEN Mask         */

#define CLK_APBCLK0_EADCCKEN_Pos        (28)                                            /*!< CLK_T::APBCLK0: EADCCKEN Position      */
#define CLK_APBCLK0_EADCCKEN_Msk        (0x1ul << CLK_APBCLK0_EADCCKEN_Pos)             /*!< CLK_T::APBCLK0: EADCCKEN Mask          */

#define CLK_APBCLK1_DACCKEN_Pos         (12)                                            /*!< CLK_T::APBCLK1: DACCKEN Position       */
#define CLK_APBCLK1_DACCKEN_Msk         (0x1ul << CLK_APBCLK1_DACCKEN_Pos)              /*!< CLK_T::APBCLK1: DACCKEN Mask           */

#define CLK_APBCLK1_CIR0CKEN_Pos        (15)                                            /*!< CLK_T::APBCLK1: CIR0CKEN Position      */
#define CLK_APBCLK1_CIR0CKEN_Msk        (0x1ul << CLK_APBCLK1_CIR0CKEN_Pos)             /*!< CLK_T::APBCLK1: CIR0CKEN Mask          */

#define CLK_APBCLK1_EPWM0CKEN_Pos       (16)                                            /*!< CLK_T::APBCLK1: EPWM0CKEN Position     */
#define CLK_APBCLK1_EPWM0CKEN_Msk       (0x1ul << CLK_APBCLK1_EPWM0CKEN_Pos)            /*!< CLK_T::APBCLK1: EPWM0CKEN Mask         */

#define CLK_APBCLK1_EPWM1CKEN_Pos       (17)                                            /*!< CLK_T::APBCLK1: EPWM1CKEN Position     */
#define CLK_APBCLK1_EPWM1CKEN_Msk       (0x1ul << CLK_APBCLK1_EPWM1CKEN_Pos)            /*!< CLK_T::APBCLK1: EPWM1CKEN Mask         */

#define CLK_APBCLK1_BPWM0CKEN_Pos       (18)                                            /*!< CLK_T::APBCLK1: BPWM0CKEN Position     */
#define CLK_APBCLK1_BPWM0CKEN_Msk       (0x1ul << CLK_APBCLK1_BPWM0CKEN_Pos)            /*!< CLK_T::APBCLK1: BPWM0CKEN Mask         */

#define CLK_APBCLK1_BPWM1CKEN_Pos       (19)                                            /*!< CLK_T::APBCLK1: BPWM1CKEN Position     */
#define CLK_APBCLK1_BPWM1CKEN_Msk       (0x1ul << CLK_APBCLK1_BPWM1CKEN_Pos)            /*!< CLK_T::APBCLK1: BPWM1CKEN Mask         */

#define CLK_APBCLK1_PRNGCKEN_Pos        (24)                                            /*!< CLK_T::APBCLK1: PRNGCKEN Position      */
#define CLK_APBCLK1_PRNGCKEN_Msk        (0x1ul << CLK_APBCLK1_PRNGCKEN_Pos)             /*!< CLK_T::APBCLK1: PRNGCKEN Mask          */

#define CLK_CLKSEL0_HCLKSEL_Pos         (0)                                             /*!< CLK_T::CLKSEL0: HCLKSEL Position       */
#define CLK_CLKSEL0_HCLKSEL_Msk         (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)              /*!< CLK_T::CLKSEL0: HCLKSEL Mask           */

#define CLK_CLKSEL0_STCLKSEL_Pos        (3)                                             /*!< CLK_T::CLKSEL0: STCLKSEL Position      */
#define CLK_CLKSEL0_STCLKSEL_Msk        (0x7ul << CLK_CLKSEL0_STCLKSEL_Pos)             /*!< CLK_T::CLKSEL0: STCLKSEL Mask          */

#define CLK_CLKSEL1_WDTSEL_Pos          (0)                                             /*!< CLK_T::CLKSEL1: WDTSEL Position        */
#define CLK_CLKSEL1_WDTSEL_Msk          (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)               /*!< CLK_T::CLKSEL1: WDTSEL Mask            */

#define CLK_CLKSEL1_TMR0SEL_Pos         (8)                                             /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)              /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos         (12)                                            /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)              /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_TMR2SEL_Pos         (16)                                            /*!< CLK_T::CLKSEL1: TMR2SEL Position       */
#define CLK_CLKSEL1_TMR2SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)              /*!< CLK_T::CLKSEL1: TMR2SEL Mask           */

#define CLK_CLKSEL1_TMR3SEL_Pos         (20)                                            /*!< CLK_T::CLKSEL1: TMR3SEL Position       */
#define CLK_CLKSEL1_TMR3SEL_Msk         (0x7ul << CLK_CLKSEL1_TMR3SEL_Pos)              /*!< CLK_T::CLKSEL1: TMR3SEL Mask           */

#define CLK_CLKSEL1_UART0SEL_Pos        (24)                                            /*!< CLK_T::CLKSEL1: UART0SEL Position      */
#define CLK_CLKSEL1_UART0SEL_Msk        (0x3ul << CLK_CLKSEL1_UART0SEL_Pos)             /*!< CLK_T::CLKSEL1: UART0SEL Mask          */

#define CLK_CLKSEL1_UART1SEL_Pos        (26)                                            /*!< CLK_T::CLKSEL1: UART1SEL Position      */
#define CLK_CLKSEL1_UART1SEL_Msk        (0x3ul << CLK_CLKSEL1_UART1SEL_Pos)             /*!< CLK_T::CLKSEL1: UART1SEL Mask          */

#define CLK_CLKSEL1_CLKOSEL_Pos         (28)                                            /*!< CLK_T::CLKSEL1: CLKOSEL Position       */
#define CLK_CLKSEL1_CLKOSEL_Msk         (0x3ul << CLK_CLKSEL1_CLKOSEL_Pos)              /*!< CLK_T::CLKSEL1: CLKOSEL Mask           */

#define CLK_CLKSEL1_WWDTSEL_Pos         (30)                                            /*!< CLK_T::CLKSEL1: WWDTSEL Position       */
#define CLK_CLKSEL1_WWDTSEL_Msk         (0x3ul << CLK_CLKSEL1_WWDTSEL_Pos)              /*!< CLK_T::CLKSEL1: WWDTSEL Mask           */

#define CLK_CLKSEL2_EPWM0SEL_Pos        (0)                                             /*!< CLK_T::CLKSEL2: EPWM0SEL Position      */
#define CLK_CLKSEL2_EPWM0SEL_Msk        (0x1ul << CLK_CLKSEL2_EPWM0SEL_Pos)             /*!< CLK_T::CLKSEL2: EPWM0SEL Mask          */

#define CLK_CLKSEL2_EPWM1SEL_Pos        (1)                                             /*!< CLK_T::CLKSEL2: EPWM1SEL Position      */
#define CLK_CLKSEL2_EPWM1SEL_Msk        (0x1ul << CLK_CLKSEL2_EPWM1SEL_Pos)             /*!< CLK_T::CLKSEL2: EPWM1SEL Mask          */

#define CLK_CLKSEL2_SPI0SEL_Pos         (4)                                             /*!< CLK_T::CLKSEL2: SPI0SEL Position       */
#define CLK_CLKSEL2_SPI0SEL_Msk         (0x3ul << CLK_CLKSEL2_SPI0SEL_Pos)              /*!< CLK_T::CLKSEL2: SPI0SEL Mask           */

#define CLK_CLKSEL2_SPI1SEL_Pos         (6)                                             /*!< CLK_T::CLKSEL2: SPI1SEL Position       */
#define CLK_CLKSEL2_SPI1SEL_Msk         (0x3ul << CLK_CLKSEL2_SPI1SEL_Pos)              /*!< CLK_T::CLKSEL2: SPI1SEL Mask           */

#define CLK_CLKSEL2_BPWM0SEL_Pos        (8)                                             /*!< CLK_T::CLKSEL2: BPWM0SEL Position      */
#define CLK_CLKSEL2_BPWM0SEL_Msk        (0x1ul << CLK_CLKSEL2_BPWM0SEL_Pos)             /*!< CLK_T::CLKSEL2: BPWM0SEL Mask          */

#define CLK_CLKSEL2_BPWM1SEL_Pos        (9)                                             /*!< CLK_T::CLKSEL2: BPWM1SEL Position      */
#define CLK_CLKSEL2_BPWM1SEL_Msk        (0x1ul << CLK_CLKSEL2_BPWM1SEL_Pos)             /*!< CLK_T::CLKSEL2: BPWM1SEL Mask          */

#define CLK_CLKSEL2_CIR0SEL_Pos         (24)                                            /*!< CLK_T::CLKSEL2: CIR0SEL Position       */
#define CLK_CLKSEL2_CIR0SEL_Msk         (0x7ul << CLK_CLKSEL2_CIR0SEL_Pos)              /*!< CLK_T::CLKSEL2: CIR0SEL Mask           */

#define CLK_CLKSEL3_UART2SEL_Pos        (24)                                            /*!< CLK_T::CLKSEL3: UART2SEL Position      */
#define CLK_CLKSEL3_UART2SEL_Msk        (0x3ul << CLK_CLKSEL3_UART2SEL_Pos)             /*!< CLK_T::CLKSEL3: UART2SEL Mask          */

#define CLK_CLKSEL3_UART3SEL_Pos        (26)                                            /*!< CLK_T::CLKSEL3: UART3SEL Position      */
#define CLK_CLKSEL3_UART3SEL_Msk        (0x3ul << CLK_CLKSEL3_UART3SEL_Pos)             /*!< CLK_T::CLKSEL3: UART3SEL Mask          */

#define CLK_CLKSEL3_UART4SEL_Pos        (28)                                            /*!< CLK_T::CLKSEL3: UART4SEL Position      */
#define CLK_CLKSEL3_UART4SEL_Msk        (0x3ul << CLK_CLKSEL3_UART4SEL_Pos)             /*!< CLK_T::CLKSEL3: UART4SEL Mask          */

#define CLK_CLKSEL3_UART5SEL_Pos        (30)                                            /*!< CLK_T::CLKSEL3: UART5SEL Position      */
#define CLK_CLKSEL3_UART5SEL_Msk        (0x3ul << CLK_CLKSEL3_UART5SEL_Pos)             /*!< CLK_T::CLKSEL3: UART5SEL Mask          */

#define CLK_CLKDIV0_HCLKDIV_Pos         (0)                                             /*!< CLK_T::CLKDIV0: HCLKDIV Position       */
#define CLK_CLKDIV0_HCLKDIV_Msk         (0xful << CLK_CLKDIV0_HCLKDIV_Pos)              /*!< CLK_T::CLKDIV0: HCLKDIV Mask           */

#define CLK_CLKDIV0_UART0DIV_Pos        (8)                                             /*!< CLK_T::CLKDIV0: UART0DIV Position      */
#define CLK_CLKDIV0_UART0DIV_Msk        (0xful << CLK_CLKDIV0_UART0DIV_Pos)             /*!< CLK_T::CLKDIV0: UART0DIV Mask          */

#define CLK_CLKDIV0_UART1DIV_Pos        (12)                                            /*!< CLK_T::CLKDIV0: UART1DIV Position      */
#define CLK_CLKDIV0_UART1DIV_Msk        (0xful << CLK_CLKDIV0_UART1DIV_Pos)             /*!< CLK_T::CLKDIV0: UART1DIV Mask          */

#define CLK_CLKDIV0_EADCDIV_Pos         (16)                                            /*!< CLK_T::CLKDIV0: EADCDIV Position       */
#define CLK_CLKDIV0_EADCDIV_Msk         (0xfful << CLK_CLKDIV0_EADCDIV_Pos)             /*!< CLK_T::CLKDIV0: EADCDIV Mask           */

#define CLK_CLKDIV4_UART2DIV_Pos        (0)                                             /*!< CLK_T::CLKDIV4: UART2DIV Position      */
#define CLK_CLKDIV4_UART2DIV_Msk        (0xful << CLK_CLKDIV4_UART2DIV_Pos)             /*!< CLK_T::CLKDIV4: UART2DIV Mask          */

#define CLK_CLKDIV4_UART3DIV_Pos        (4)                                             /*!< CLK_T::CLKDIV4: UART3DIV Position      */
#define CLK_CLKDIV4_UART3DIV_Msk        (0xful << CLK_CLKDIV4_UART3DIV_Pos)             /*!< CLK_T::CLKDIV4: UART3DIV Mask          */

#define CLK_CLKDIV4_UART4DIV_Pos        (8)                                             /*!< CLK_T::CLKDIV4: UART4DIV Position      */
#define CLK_CLKDIV4_UART4DIV_Msk        (0xful << CLK_CLKDIV4_UART4DIV_Pos)             /*!< CLK_T::CLKDIV4: UART4DIV Mask          */

#define CLK_CLKDIV4_UART5DIV_Pos        (12)                                            /*!< CLK_T::CLKDIV4: UART5DIV Position      */
#define CLK_CLKDIV4_UART5DIV_Msk        (0xful << CLK_CLKDIV4_UART5DIV_Pos)             /*!< CLK_T::CLKDIV4: UART5DIV Mask          */

#define CLK_CLKDIV4_TRACEDIV_Pos        (24)                                            /*!< CLK_T::CLKDIV4: TRACEDIV Position      */
#define CLK_CLKDIV4_TRACEDIV_Msk        (0xfful << CLK_CLKDIV4_TRACEDIV_Pos)            /*!< CLK_T::CLKDIV4: TRACEDIV Mask          */

#define CLK_PCLKDIV_APB0DIV_Pos         (0)                                             /*!< CLK_T::PCLKDIV: APB0DIV Position       */
#define CLK_PCLKDIV_APB0DIV_Msk         (0x7ul << CLK_PCLKDIV_APB0DIV_Pos)              /*!< CLK_T::PCLKDIV: APB0DIV Mask           */

#define CLK_PCLKDIV_APB1DIV_Pos         (4)                                             /*!< CLK_T::PCLKDIV: APB1DIV Position       */
#define CLK_PCLKDIV_APB1DIV_Msk         (0x7ul << CLK_PCLKDIV_APB1DIV_Pos)              /*!< CLK_T::PCLKDIV: APB1DIV Mask           */

#define CLK_PLLCTL_FBDIV_Pos            (0)                                             /*!< CLK_T::PLLCTL: FBDIV Position          */
#define CLK_PLLCTL_FBDIV_Msk            (0x1fful << CLK_PLLCTL_FBDIV_Pos)               /*!< CLK_T::PLLCTL: FBDIV Mask              */

#define CLK_PLLCTL_INDIV_Pos            (9)                                             /*!< CLK_T::PLLCTL: INDIV Position          */
#define CLK_PLLCTL_INDIV_Msk            (0x1ful << CLK_PLLCTL_INDIV_Pos)                /*!< CLK_T::PLLCTL: INDIV Mask              */

#define CLK_PLLCTL_OUTDIV_Pos           (14)                                            /*!< CLK_T::PLLCTL: OUTDIV Position         */
#define CLK_PLLCTL_OUTDIV_Msk           (0x3ul << CLK_PLLCTL_OUTDIV_Pos)                /*!< CLK_T::PLLCTL: OUTDIV Mask             */

#define CLK_PLLCTL_PD_Pos               (16)                                            /*!< CLK_T::PLLCTL: PD Position             */
#define CLK_PLLCTL_PD_Msk               (0x1ul << CLK_PLLCTL_PD_Pos)                    /*!< CLK_T::PLLCTL: PD Mask                 */

#define CLK_PLLCTL_BP_Pos               (17)                                            /*!< CLK_T::PLLCTL: BP Position             */
#define CLK_PLLCTL_BP_Msk               (0x1ul << CLK_PLLCTL_BP_Pos)                    /*!< CLK_T::PLLCTL: BP Mask                 */

#define CLK_PLLCTL_OE_Pos               (18)                                            /*!< CLK_T::PLLCTL: OE Position             */
#define CLK_PLLCTL_OE_Msk               (0x1ul << CLK_PLLCTL_OE_Pos)                    /*!< CLK_T::PLLCTL: OE Mask                 */

#define CLK_PLLCTL_PLLSRC_Pos           (19)                                            /*!< CLK_T::PLLCTL: PLLSRC Position         */
#define CLK_PLLCTL_PLLSRC_Msk           (0x1ul << CLK_PLLCTL_PLLSRC_Pos)                /*!< CLK_T::PLLCTL: PLLSRC Mask             */

#define CLK_PLLCTL_STBSEL_Pos           (23)                                            /*!< CLK_T::PLLCTL: STBSEL Position         */
#define CLK_PLLCTL_STBSEL_Msk           (0x1ul << CLK_PLLCTL_STBSEL_Pos)                /*!< CLK_T::PLLCTL: STBSEL Mask             */

#define CLK_STATUS_HXTSTB_Pos           (0)                                             /*!< CLK_T::STATUS: HXTSTB Position         */
#define CLK_STATUS_HXTSTB_Msk           (0x1ul << CLK_STATUS_HXTSTB_Pos)                /*!< CLK_T::STATUS: HXTSTB Mask             */

#define CLK_STATUS_LXTSTB_Pos           (1)                                             /*!< CLK_T::STATUS: LXTSTB Position         */
#define CLK_STATUS_LXTSTB_Msk           (0x1ul << CLK_STATUS_LXTSTB_Pos)                /*!< CLK_T::STATUS: LXTSTB Mask             */

#define CLK_STATUS_PLLSTB_Pos           (2)                                             /*!< CLK_T::STATUS: PLLSTB Position         */
#define CLK_STATUS_PLLSTB_Msk           (0x1ul << CLK_STATUS_PLLSTB_Pos)                /*!< CLK_T::STATUS: PLLSTB Mask             */

#define CLK_STATUS_LIRCSTB_Pos          (3)                                             /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk          (0x1ul << CLK_STATUS_LIRCSTB_Pos)               /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos          (4)                                             /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk          (0x1ul << CLK_STATUS_HIRCSTB_Pos)               /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_CLKSFAIL_Pos         (7)                                             /*!< CLK_T::STATUS: CLKSFAIL Position       */
#define CLK_STATUS_CLKSFAIL_Msk         (0x1ul << CLK_STATUS_CLKSFAIL_Pos)              /*!< CLK_T::STATUS: CLKSFAIL Mask           */

#define CLK_CLKOCTL_FREQSEL_Pos         (0)                                             /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk         (0xful << CLK_CLKOCTL_FREQSEL_Pos)              /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos          (4)                                             /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk          (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)               /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos          (5)                                             /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk          (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)               /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

#define CLK_CLKOCTL_CLK1HZEN_Pos        (6)                                             /*!< CLK_T::CLKOCTL: CLK1HZEN Position      */
#define CLK_CLKOCTL_CLK1HZEN_Msk        (0x1ul << CLK_CLKOCTL_CLK1HZEN_Pos)             /*!< CLK_T::CLKOCTL: CLK1HZEN Mask          */

#define CLK_CLKDCTL_HXTFDEN_Pos         (4)                                             /*!< CLK_T::CLKDCTL: HXTFDEN Position       */
#define CLK_CLKDCTL_HXTFDEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFDEN_Pos)              /*!< CLK_T::CLKDCTL: HXTFDEN Mask           */

#define CLK_CLKDCTL_HXTFIEN_Pos         (5)                                             /*!< CLK_T::CLKDCTL: HXTFIEN Position       */
#define CLK_CLKDCTL_HXTFIEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFIEN_Pos)              /*!< CLK_T::CLKDCTL: HXTFIEN Mask           */

#define CLK_CLKDCTL_LXTFDEN_Pos         (12)                                            /*!< CLK_T::CLKDCTL: LXTFDEN Position       */
#define CLK_CLKDCTL_LXTFDEN_Msk         (0x1ul << CLK_CLKDCTL_LXTFDEN_Pos)              /*!< CLK_T::CLKDCTL: LXTFDEN Mask           */

#define CLK_CLKDCTL_LXTFIEN_Pos         (13)                                            /*!< CLK_T::CLKDCTL: LXTFIEN Position       */
#define CLK_CLKDCTL_LXTFIEN_Msk         (0x1ul << CLK_CLKDCTL_LXTFIEN_Pos)              /*!< CLK_T::CLKDCTL: LXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFQDEN_Pos        (16)                                            /*!< CLK_T::CLKDCTL: HXTFQDEN Position      */
#define CLK_CLKDCTL_HXTFQDEN_Msk        (0x1ul << CLK_CLKDCTL_HXTFQDEN_Pos)             /*!< CLK_T::CLKDCTL: HXTFQDEN Mask          */

#define CLK_CLKDCTL_HXTFQIEN_Pos        (17)                                            /*!< CLK_T::CLKDCTL: HXTFQIEN Position      */
#define CLK_CLKDCTL_HXTFQIEN_Msk        (0x1ul << CLK_CLKDCTL_HXTFQIEN_Pos)             /*!< CLK_T::CLKDCTL: HXTFQIEN Mask          */

#define CLK_CLKDSTS_HXTFIF_Pos          (0)                                             /*!< CLK_T::CLKDSTS: HXTFIF Position        */
#define CLK_CLKDSTS_HXTFIF_Msk          (0x1ul << CLK_CLKDSTS_HXTFIF_Pos)               /*!< CLK_T::CLKDSTS: HXTFIF Mask            */

#define CLK_CLKDSTS_LXTFIF_Pos          (1)                                             /*!< CLK_T::CLKDSTS: LXTFIF Position        */
#define CLK_CLKDSTS_LXTFIF_Msk          (0x1ul << CLK_CLKDSTS_LXTFIF_Pos)               /*!< CLK_T::CLKDSTS: LXTFIF Mask            */

#define CLK_CLKDSTS_HXTFQIF_Pos         (8)                                             /*!< CLK_T::CLKDSTS: HXTFQIF Position       */
#define CLK_CLKDSTS_HXTFQIF_Msk         (0x1ul << CLK_CLKDSTS_HXTFQIF_Pos)              /*!< CLK_T::CLKDSTS: HXTFQIF Mask           */

#define CLK_CDUPB_UPERBD_Pos            (0)                                             /*!< CLK_T::CDUPB: UPERBD Position          */
#define CLK_CDUPB_UPERBD_Msk            (0x7fful << CLK_CDUPB_UPERBD_Pos)               /*!< CLK_T::CDUPB: UPERBD Mask              */

#define CLK_CDLOWB_LOWERBD_Pos          (0)                                             /*!< CLK_T::CDLOWB: LOWERBD Position        */
#define CLK_CDLOWB_LOWERBD_Msk          (0x7fful << CLK_CDLOWB_LOWERBD_Pos)             /*!< CLK_T::CDLOWB: LOWERBD Mask            */

#define CLK_PMUCTL_PDMSEL_Pos           (0)                                             /*!< CLK_T::PMUCTL: PDMSEL Position         */
#define CLK_PMUCTL_PDMSEL_Msk           (0x7ul << CLK_PMUCTL_PDMSEL_Pos)                /*!< CLK_T::PMUCTL: PDMSEL Mask             */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */
/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __CLK_REG_H__ */
