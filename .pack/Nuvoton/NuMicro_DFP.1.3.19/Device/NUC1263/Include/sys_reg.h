/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V3.00
 * @brief    SYS register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_REG_H__
#define __SYS_REG_H__

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


 /******************************************************************************/
 /*                Device Specific Peripheral registers structures             */
 /******************************************************************************/

 /** @addtogroup REGISTER Control Register

   @{

 */


/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
    @{
*/

typedef struct
{


    /**
     * @var SYS_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number (Read Only)
     * |        |          |This register reflects device part number code.
     * |        |          |Software can read this register to identify which device is used.
     * @var SYS_T::RSTSTS
     * Offset: 0x04  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the Reset Signal from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_IPRST0[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[1]     |PINRF     |nRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the Reset Signal from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the Reset Signal from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note1: This bit can be cleared by software writing 1.
     * |        |          |Note2: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset.
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |[3]     |LVRF      |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the Reset Signal from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the Reset Signal from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[5]     |MCURF     |MCU Reset Flag
     * |        |          |The MCU reset flag is set by the Reset Signal from the Cortex-M23 Core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M23.
     * |        |          |1 = The Cortex-M23 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M23 core.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M23 Core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M23 Core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[8]     |CPULKRF   |CPU Lockup Reset Flag
     * |        |          |The CPU lockup reset flag is set by hardware if Cortex-M23 lockup happened.
     * |        |          |0 = No reset from CPU lockup happened.
     * |        |          |1 = The Cortex-M23 lockup happened and chip is reset.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * @var SYS_T::IPRST0
     * Offset: 0x08  Peripheral  Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |Chip One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is same as the POR reset, all the chip controllers is reset and the chip setting from flash are also reload.
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = Chip one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |CPURST    |Processor Core One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC), and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PDMARST   |PDMA Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMA.
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMA controller normal operation.
     * |        |          |1 = PDMA controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |CRCRST    |CRC Calculation Controller Reset (Write Protect)
     * |        |          |Set this bit to 1 will generate a reset signal to the CRC calculation controller.
     * |        |          |User needs to set this bit to 0 to release from the reset state.
     * |        |          |0 = CRC calculation controller normal operation.
     * |        |          |1 = CRC calculation controller reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::IPRST1
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[4]     |TMR2RST   |Timer2 Controller Reset
     * |        |          |0 = Timer2 controller normal operation.
     * |        |          |1 = Timer2 controller reset.
     * |[5]     |TMR3RST   |Timer3 Controller Reset
     * |        |          |0 = Timer3 controller normal operation.
     * |        |          |1 = Timer3 controller reset.
     * |[8]     |I2C0RST   |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[9]     |I2C1RST   |I2C1 Controller Reset
     * |        |          |0 = I2C1 controller normal operation.
     * |        |          |1 = I2C1 controller reset.
     * |[10]    |I2C2RST   |I2C2 Controller Reset
     * |        |          |0 = I2C2 controller normal operation.
     * |        |          |1 = I2C2 controller reset.
     * |[12]    |SPI0RST   |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1RST   |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[14]    |SPI2RST   |SPI2 Controller Reset
     * |        |          |0 = SPI2 controller normal operation.
     * |        |          |1 = SPI2 controller reset.
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1RST  |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[18]    |UART2RST  |UART2 Controller Reset
     * |        |          |0 = UART2 controller normal operation.
     * |        |          |1 = UART2 controller reset.
     * |[20]    |BPWM0RST  |BPWM0 Controller Reset
     * |        |          |0 = BPWM0 controller normal operation.
     * |        |          |1 = BPWM0 controller reset.
     * |[21]    |BPWM1RST  |BPWM1 Controller Reset
     * |        |          |0 = BPWM1 controller normal operation.
     * |        |          |1 = BPWM1 controller reset.
     * |[22]    |BPWM2RST  |BPWM2 Controller Reset
     * |        |          |0 = BPWM2 controller normal operation.
     * |        |          |1 = BPWM2 controller reset.
     * |[23]    |BPWM3RST  |BPWM3 Controller Reset
     * |        |          |0 = BPWM3 controller normal operation.
     * |        |          |1 = BPWM3 controller reset.
     * |[24]    |I3C0RST   |I3C0 Controller Reset
     * |        |          |0 = I3C0 controller normal operation.
     * |        |          |1 = I3C0 controller reset.
     * |[25]    |I3C1RST   |I3C1 Controller Reset
     * |        |          |0 = I3C1 controller normal operation.
     * |        |          |1 = I3C1 controller reset.
     * |[26]    |SPDHRST   |SPD5 Hub Controller Reset
     * |        |          |0 = SPD5 Hub controller normal operation.
     * |        |          |1 = SPD5 Hub controller reset.
     * |[27]    |USBDRST   |USB Device Controller Reset
     * |        |          |0 = USB device controller normal operation.
     * |        |          |1 = USB device controller reset.
     * |[28]    |ADCRST    |ADC Controller Reset
     * |        |          |0 = ADC controller normal operation.
     * |        |          |1 = ADC controller reset.
     * |[29]    |DACRST    |DAC Controller Reset
     * |        |          |0 = DAC controller normal operation.
     * |        |          |1 = DAC controller reset.
     * |[30]    |ACMP01RST |ACMP01 Controller Reset
     * |        |          |0 = ACMP01 controller normal operation.
     * |        |          |1 = ACMP01 controller reset.
     * |[31]    |ACMP23RST |ACMP23 Controller Reset
     * |        |          |0 = ACMP23 controller normal operation.
     * |        |          |1 = ACMP23 controller reset.
     * @var SYS_T::IPRST2
     * Offset: 0x10  Peripheral Reset Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |LLSI0RST  |LLSI0 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 0 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 0 controller reset.
     * |[17]    |LLSI1RST  |LLSI1 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 1 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 1 controller reset.
     * |[18]    |LLSI2RST  |LLSI2 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 2 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 2 controller reset.
     * |[19]    |LLSI3RST  |LLSI3 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 3 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 3 controller reset.
     * |[20]    |LLSI4RST  |LLSI4 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 4 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 4 controller reset.
     * |[21]    |LLSI5RST  |LLSI5 Controller Reset
     * |        |          |0 = LED Lighting Strip Interface 5 controller normal operation.
     * |        |          |1 = LED Lighting Strip Interface 5 controller reset.
     * |[28]    |TSRST     |Temperature Sensor Controller Reset
     * |        |          |0 = Temperature Sensor controller normal operation.
     * |        |          |1 = Temperature Sensor controller reset.
     * @var SYS_T::BODCTL
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBODEN (CONFIG0 [23]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2:1]   |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBOV (CONFIG0 [22:21]).
     * |        |          |00 = Brown-Out Detector threshold voltage is 2.2V.
     * |        |          |01 = Brown-Out Detector threshold voltage is 2.7V.
     * |        |          |10 = Brown-Out Detector threshold voltage is 3.7V.
     * |        |          |11 = Brown-Out Detector threshold voltage is 4.5V.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |0 = Brown-out interrupt function Enabled.
     * |        |          |1 = Brown-out reset function Enabled.
     * |        |          |Note1:
     * |        |          |While the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |While the BOD function is enabled (BODEN high) and BOD interrupt function is enabled (BODRSTEN low), BOD will assert an interrupt if BODOUT is high
     * |        |          |BOD interrupt will keep till to the BODEN set to 0.
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BODEN low).
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at AVDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the AVDD is dropped down through the voltage of BODVL setting or the AVDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[5]     |BODLPM    |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note1: The BOD consumes about 100uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |BODOUT    |Brown-out Detector Output Status
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL setting.
     * |        |          |If the BODEN is 0, BOD function disabled, this bit always responds 0.
     * |[7]     |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting.
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note1: After enabling the bit, the LVR function will be active with 200us delay for LVR output stable (default).
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10:8]  |BODDGSEL  |Brown-out Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = BOD output is sampled by RC10K clock.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[14:12] |LVRDGSEL  |LVR Output De-glitch Time Select (Write Protect)
     * |        |          |000 = Without de-glitch function.
     * |        |          |001 = 4 system clock (HCLK).
     * |        |          |010 = 8 system clock (HCLK).
     * |        |          |011 = 16 system clock (HCLK).
     * |        |          |100 = 32 system clock (HCLK).
     * |        |          |101 = 64 system clock (HCLK).
     * |        |          |110 = 128 system clock (HCLK).
     * |        |          |111 = 256 system clock (HCLK).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |VDETEN    |Voltage Detector Enable Bit
     * |        |          |0 = VDET detect external input voltage function Disabled.
     * |        |          |1 = VDET detect external input voltage function Enabled.
     * |        |          |Note1: This function is still active in whole chip power-down mode.
     * |        |          |Note2: This function need use LIRC or LXT as VDET clock source, which is selected in VDETCKSEL (CLK_BODCLK[0]).
     * |        |          |Note3: The input pin for VDET detect voltage is selectabe by VDETPINSEL (SYS_BODCTL[17]).
     * |[17]    |VDETPINSEL|Voltage Detector External Input Voltage Pin Selection
     * |        |          |0 = The input voltage is from VDET_P0 (PB.0).
     * |        |          |1 = The input voltage is from VDET_P1 (PB.1).
     * |        |          |Note1: If VDET_P0 is selected, multi-function pin must be selected correctly in PB0MFP (SYS_GPB_MFPL[3:0]).
     * |        |          |Note2: If VDET_P1 is selected, multi-function pin must be selected correctly in PB1MFP (SYS_GPB_MFPL[7:4]).
     * |[18]    |VDETIEN   |Voltage Detector Interrupt Enable Bit
     * |        |          |0 = VDET interrupt Disabled.
     * |        |          |1 = VDET interrupt Enabled.
     * |[19]    |VDETIF    |Voltage Detector Interrupt Flag
     * |        |          |0 = VDET does not detect any voltage draft at external pin down through or up through the voltage of Bandgap.
     * |        |          |1 = When VDET detects the external pin is dropped down through the voltage of Bandgap or the external pin is raised up through the voltage of Bandgap, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[24]    |VDETOUT   |Voltage Detector Output Status
     * |        |          |0 = VDET output status is 0.
     * |        |          |It means the detected voltage is higher than Bandgap or VDETEN is 0.
     * |        |          |1 = VDET output status is 1.
     * |        |          |It means the detected voltage is lower than Bandgap.
     * |        |          |If the VDETEN is 0, VDET function disabled, this bit always responds 0. 
     * |[27:25] |VDETDGSEL |Voltage Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = VDET output is sampled by VDET clock.
     * |        |          |001 = 16 system clock (HCLK).
     * |        |          |010 = 32 system clock (HCLK).
     * |        |          |011 = 64 system clock (HCLK).
     * |        |          |100 = 128 system clock (HCLK).
     * |        |          |101 = 256 system clock (HCLK).
     * |        |          |110 = 512 system clock (HCLK).
     * |        |          |111 = 1024 system clock (HCLK).
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.     
     * @var SYS_T::IVSCTL
     * Offset: 0x1C  Internal Voltage Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |VBGUGEN   |Band-gap VBG Unity Gain Buffer Enable Bit
     * |        |          |This bit is used to enable/disable Band-gap VBG unity gain buffer function.
     * |        |          |0 = VBG unity gain buffer function Disabled (default).
     * |        |          |1 = VBG unity gain buffer function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of VBG unity gain buffer output voltage can be obtained from ADC conversion result. Please refer to ADC function chapter for details.
     * @var SYS_T::PORCTL
     * Offset: 0x24  Power-on Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POROFF    |Power-on Reset Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::VREFCTL
     * Offset: 0x28  VREF Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |VREFCTL   |VREF Control Bits (Write Protect)
     * |        |          |00000 = VREF is from external pin.
     * |        |          |00010 = VREF is from internal reference voltage 2.048V.
     * |        |          |00110 = VREF is from internal reference voltage 2.56V.
     * |        |          |01010 = VREF is from internal reference voltage 3.072V.
     * |        |          |01110 = VREF is from internal reference voltage 4.096V.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PRELOADEN |Pre-load Function Enable Bit (Write Protect)
     * |        |          |This bit should be enabled and keep during Tstable when VREFCTL(SYS_VREFCTL[4:0]) change setting(except set to 00000). 
     * |        |          |Tstable depends on different situations has different requirement, please refer to datasheet.
     * |        |          |0 = VREF Pre-load function Disabled. (Default).
     * |        |          |1 = VREF Pre-load function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |VBGFEN    |Chip Internal Voltage Bandgap Force Enable Bit (Write Protect)
     * |        |          |0 = Chip internal voltage bandgap controlled by ADC/ACMP if source selected.
     * |        |          |1 = Chip internal voltage bandgap force enable.
     * @var SYS_T::GPA_MFPL
     * Offset: 0x30  GPIOA Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
     * |        |          |02 = SPI_MOSI_MUX
     * |        |          |04 = SPI0_MOSI
     * |        |          |05 = SPI2_MOSI
     * |        |          |06 = I3CS0_SDA
     * |        |          |07 = UART0_RXD
     * |        |          |08 = UART1_nRTS
     * |        |          |09 = I2C2_SDA
     * |        |          |12 = BPWM0_CH0
     * |        |          |13 = BPWM2_CH5
     * |        |          |14 = ACMP3_O
     * |        |          |15 = DAC0_ST
     * |[7:4]   |PA1MFP    |PA.1 Multi-function Pin Selection
     * |        |          |02 = SPI_MISO_MUX
     * |        |          |04 = SPI0_MISO
     * |        |          |05 = SPI2_MISO
     * |        |          |06 = I3CS0_SCL
     * |        |          |07 = UART0_TXD
     * |        |          |08 = UART1_nCTS
     * |        |          |09 = I2C2_SCL
     * |        |          |12 = BPWM0_CH1
     * |        |          |13 = BPWM2_CH4
     * |        |          |15 = ACMP2_O
     * |[11:8]  |PA2MFP    |PA.2 Multi-function Pin Selection
     * |        |          |02 = SPI_CLK_MUX
     * |        |          |04 = SPI0_CLK
     * |        |          |05 = SPI2_CLK
     * |        |          |06 = I3CS1_SDA
     * |        |          |07 = I2C0_SMBSUS
     * |        |          |08 = UART1_RXD
     * |        |          |09 = I2C1_SDA
     * |        |          |10 = LLSI5_OUT
     * |        |          |12 = BPWM0_CH2
     * |        |          |13 = BPWM2_CH3
     * |        |          |15 = ACMP3_WLAT
     * |[15:12] |PA3MFP    |PA.3 Multi-function Pin Selection
     * |        |          |02 = SPI_SS_MUX
     * |        |          |04 = SPI0_SS
     * |        |          |05 = SPI2_SS
     * |        |          |06 = I3CS1_SCL
     * |        |          |07 = I2C0_SMBAL
     * |        |          |08 = UART1_TXD
     * |        |          |09 = I2C1_SCL
     * |        |          |10 = LLSI4_OUT
     * |        |          |12 = BPWM0_CH3
     * |        |          |13 = BPWM2_CH2
     * |        |          |14 = CLKO
     * |        |          |15 = ACMP2_WLAT
     * |[19:16] |PA4MFP    |PA.4 Multi-function Pin Selection
     * |        |          |04 = SPI0_I2SMCLK
     * |        |          |06 = I3CS0_SDA
     * |        |          |07 = UART0_nRTS
     * |        |          |08 = UART0_RXD
     * |        |          |09 = I2C0_SDA
     * |        |          |12 = BPWM0_CH4
     * |        |          |13 = BPWM2_CH1
     * |        |          |15 = DAC3_ST
     * |[23:20] |PA5MFP    |PA.5 Multi-function Pin Selection
     * |        |          |02 = SPI1_I2SMCLK
     * |        |          |04 = SPI0_I2SMCLK
     * |        |          |05 = SPI2_I2SMCLK
     * |        |          |06 = I3CS0_SCL
     * |        |          |07 = UART0_nCTS
     * |        |          |08 = UART0_TXD
     * |        |          |09 = I2C0_SCL
     * |        |          |12 = BPWM0_CH5
     * |        |          |13 = BPWM2_CH0
     * |        |          |15 = DAC2_ST
     * |[27:24] |PA6MFP    |PA.6 Multi-function Pin Selection
     * |        |          |02 = SPI1_SS
     * |        |          |07 = UART0_RXD
     * |        |          |08 = I2C1_SDA
     * |        |          |11 = BPWM3_CH5
     * |        |          |12 = BPWM1_CH3
     * |        |          |13 = ACMP1_WLAT
     * |        |          |14 = TM3
     * |        |          |15 = INT0
     * |[31:28] |PA7MFP    |PA.7 Multi-function Pin Selection
     * |        |          |02 = SPI1_CLK
     * |        |          |07 = UART0_TXD
     * |        |          |08 = I2C1_SCL
     * |        |          |11 = BPWM3_CH4
     * |        |          |12 = BPWM1_CH2
     * |        |          |13 = ACMP0_WLAT
     * |        |          |14 = TM2
     * |        |          |15 = INT1
     * @var SYS_T::GPA_MFPH
     * Offset: 0x34  GPIOA High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA8MFP    |PA.8 Multi-function Pin Selection
     * |        |          |03 = SPI1_SS
     * |        |          |04 = SPI2_MOSI
     * |        |          |07 = UART1_RXD
     * |        |          |09 = BPWM0_CH3
     * |        |          |10 = I2C0_SCL
     * |        |          |13 = TM3_EXT
     * |        |          |14 = DAC3_ST
     * |        |          |15 = INT4
     * |[7:4]   |PA9MFP    |PA.9 Multi-function Pin Selection
     * |        |          |03 = SPI1_CLK
     * |        |          |04 = SPI2_MISO
     * |        |          |07 = UART1_TXD
     * |        |          |09 = BPWM0_CH2
     * |        |          |10 = I2C1_SDA
     * |        |          |13 = TM2_EXT
     * |        |          |14 = DAC2_ST
     * |[11:8]  |PA10MFP   |PA.10 Multi-function Pin Selection
     * |        |          |03 = SPI1_MISO
     * |        |          |04 = SPI2_CLK
     * |        |          |07 = I2C2_SDA
     * |        |          |09 = BPWM0_CH1
     * |        |          |10 = I2C1_SCL
     * |        |          |13 = TM1_EXT
     * |        |          |14 = DAC0_ST
     * |[15:12] |PA11MFP   |PA.11 Multi-function Pin Selection
     * |        |          |03 = SPI1_MOSI
     * |        |          |04 = SPI2_SS
     * |        |          |07 = I2C2_SCL
     * |        |          |09 = BPWM0_CH0
     * |        |          |13 = TM0_EXT
     * |        |          |14 = DAC1_ST
     * @var SYS_T::GPB_MFPL
     * Offset: 0x38  GPIOB Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH0, VDET_P0
     * |        |          |03 = SPI0_SS
     * |        |          |04 = SPI2_I2SMCLK
     * |        |          |07 = UART2_RXD
     * |        |          |08 = SPI0_I2SMCLK
     * |        |          |09 = I2C1_SDA
     * |        |          |11 = BPWM2_CH5
     * |        |          |12 = BPWM3_CH5
     * |[7:4]   |PB1MFP    |PB.1 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH1, VDET_P1, SPDH_HSA
     * |        |          |02 = SPI1_I2SMCLK
     * |        |          |07 = UART2_TXD
     * |        |          |09 = I2C1_SCL
     * |        |          |11 = BPWM2_CH4
     * |        |          |12 = BPWM3_CH4
     * |        |          |13 = TM3_EXT
     * |[11:8]  |PB2MFP    |PB.2 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH2, ACMP0_P0
     * |        |          |02 = SPI1_SS
     * |        |          |04 = I2C1_SDA
     * |        |          |06 = UART1_RXD
     * |        |          |07 = I2C2_SDA
     * |        |          |08 = SPI0_I2SMCLK
     * |        |          |11 = BPWM2_CH3
     * |        |          |14 = TM3
     * |        |          |15 = INT3
     * |[15:12] |PB3MFP    |PB.3 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH3, ACMP0_P1, ACMP1_P0
     * |        |          |02 = SPI1_CLK
     * |        |          |04 = I2C1_SCL
     * |        |          |06 = UART1_TXD
     * |        |          |07 = I2C2_SCL
     * |        |          |11 = BPWM2_CH2
     * |        |          |14 = TM2
     * |        |          |15 = INT2
     * |[19:16] |PB4MFP    |PB.4 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH4, ACMP0_N, ACMP1_P1, ACMP2_P0
     * |        |          |02 = SPI1_MOSI
     * |        |          |06 = I2C0_SDA
     * |        |          |11 = BPWM2_CH1
     * |        |          |12 = LLSI5_OUT
     * |        |          |13 = UART2_RXD
     * |        |          |14 = TM1
     * |        |          |15 = INT1
     * |[23:20] |PB5MFP    |PB.5 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH5, ACMP1_N, ACMP2_P1, ACMP3_P0
     * |        |          |02 = SPI1_MISO
     * |        |          |06 = I2C0_SCL
     * |        |          |11 = BPWM2_CH0
     * |        |          |12 = LLSI4_OUT
     * |        |          |13 = UART2_TXD
     * |        |          |14 = TM0
     * |        |          |15 = INT0
     * |[27:24] |PB6MFP    |PB.6 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH6, ACMP2_N, ACMP3_P1
     * |        |          |06 = UART1_RXD
     * |        |          |10 = BPWM1_CH5
     * |        |          |12 = BPWM3_CH5
     * |        |          |13 = INT4
     * |        |          |15 = ACMP1_O
     * |[31:28] |PB7MFP    |PB.7 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH7, ACMP3_N
     * |        |          |06 = UART1_TXD
     * |        |          |10 = BPWM1_CH4
     * |        |          |12 = BPWM3_CH4
     * |        |          |13 = INT5
     * |        |          |15 = ACMP0_O
     * @var SYS_T::GPB_MFPH
     * Offset: 0x3C  GPIOB High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB8MFP    |PB.8 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH8
     * |        |          |05 = UART0_RXD
     * |        |          |06 = UART1_nRTS
     * |        |          |07 = I2C1_SMBSUS
     * |        |          |09 = I2C0_SDA
     * |        |          |10 = BPWM1_CH3
     * |[7:4]   |PB9MFP    |PB.9 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH9
     * |        |          |05 = UART0_TXD
     * |        |          |06 = UART1_nCTS
     * |        |          |07 = I2C1_SMBAL
     * |        |          |09 = I2C0_SCL
     * |        |          |10 = BPWM1_CH2
     * |[11:8]  |PB10MFP   |PB.10 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH10
     * |        |          |05 = UART0_nRTS
     * |        |          |07 = I2C1_SDA
     * |        |          |10 = BPWM1_CH1
     * |[15:12] |PB11MFP   |PB.11 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH11
     * |        |          |05 = UART0_nCTS
     * |        |          |07 = I2C1_SCL
     * |        |          |09 = SPI0_I2SMCLK
     * |        |          |10 = BPWM1_CH0
     * |[19:16] |PB12MFP   |PB.12 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH12, DAC0_OUT, ACMP0_P2, ACMP1_P2
     * |        |          |04 = SPI0_MOSI
     * |        |          |06 = UART0_RXD
     * |        |          |08 = I2C2_SDA
     * |        |          |10 = LLSI3_OUT
     * |        |          |11 = BPWM3_CH3
     * |        |          |13 = TM3_EXT
     * |[23:20] |PB13MFP   |PB.13 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH13, DAC1_OUT, ACMP0_P3, ACMP1_P3
     * |        |          |04 = SPI0_MISO
     * |        |          |06 = UART0_TXD
     * |        |          |08 = I2C2_SCL
     * |        |          |10 = LLSI2_OUT
     * |        |          |11 = BPWM3_CH2
     * |        |          |13 = TM2_EXT
     * |[27:24] |PB14MFP   |PB.14 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH14, DAC2_OUT, ACMP2_P2, ACMP3_P2
     * |        |          |04 = SPI0_CLK
     * |        |          |06 = UART0_nRTS
     * |        |          |08 = I2C2_SMBSUS
     * |        |          |10 = LLSI1_OUT
     * |        |          |11 = BPWM3_CH1
     * |        |          |13 = TM1_EXT
     * |        |          |14 = CLKO
     * |[31:28] |PB15MFP   |PB.15 Multi-function Pin Selection
     * |        |          |01 = ADC0_CH15, DAC3_OUT, ACMP2_P3, ACMP3_P3
     * |        |          |04 = SPI0_SS
     * |        |          |06 = UART0_nCTS
     * |        |          |08 = I2C2_SMBAL
     * |        |          |10 = LLSI0_OUT
     * |        |          |11 = BPWM3_CH0
     * |        |          |13 = TM0_EXT
     * @var SYS_T::GPC_MFPL
     * Offset: 0x40  GPIOC Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
     * |        |          |02 = SPI1_SS
     * |        |          |07 = I2C1_SDA
     * |        |          |08 = UART2_RXD
     * |        |          |09 = I2C0_SDA
     * |        |          |12 = BPWM3_CH5
     * |        |          |13 = BPWM3_CH3
     * |        |          |14 = ACMP1_O
     * |[7:4]   |PC1MFP    |PC.1 Multi-function Pin Selection
     * |        |          |02 = SPI1_CLK
     * |        |          |07 = I2C1_SCL
     * |        |          |08 = UART2_TXD
     * |        |          |09 = I2C0_SCL
     * |        |          |11 = ADC0_ST
     * |        |          |12 = BPWM3_CH4
     * |        |          |13 = BPWM2_CH1
     * |        |          |14 = ACMP0_O
     * |[11:8]  |PC2MFP    |PC.2 Multi-function Pin Selection
     * |        |          |02 = SPI1_MOSI
     * |        |          |07 = I2C2_SMBSUS
     * |        |          |08 = UART2_nCTS
     * |        |          |09 = I2C0_SMBSUS
     * |        |          |10 = I2C0_SDA
     * |        |          |12 = BPWM3_CH3
     * |        |          |15 = LLSI3_OUT
     * |[15:12] |PC3MFP    |PC.3 Multi-function Pin Selection
     * |        |          |02 = SPI1_MISO
     * |        |          |07 = I2C2_SMBAL
     * |        |          |08 = UART2_nRTS
     * |        |          |09 = I2C0_SMBAL
     * |        |          |10 = I2C0_SCL
     * |        |          |12 = BPWM3_CH2
     * |        |          |15 = LLSI2_OUT
     * |[19:16] |PC4MFP    |PC.4 Multi-function Pin Selection
     * |        |          |02 = SPI1_I2SMCLK
     * |        |          |08 = UART2_RXD
     * |        |          |09 = I2C1_SDA
     * |        |          |12 = BPWM3_CH1
     * |        |          |15 = LLSI1_OUT
     * |[23:20] |PC5MFP    |PC.5 Multi-function Pin Selection
     * |        |          |08 = UART2_TXD
     * |        |          |09 = I2C1_SCL
     * |        |          |12 = BPWM3_CH0
     * |        |          |15 = LLSI0_OUT
     * |[27:24] |PC6MFP    |PC.6 Multi-function Pin Selection
     * |        |          |04 = SPI1_MOSI
     * |        |          |07 = UART0_nRTS
     * |        |          |08 = I2C1_SMBSUS
     * |        |          |11 = BPWM3_CH3
     * |        |          |12 = BPWM1_CH1
     * |        |          |14 = TM1
     * |        |          |15 = INT2
     * |[31:28] |PC7MFP    |PC.7 Multi-function Pin Selection
     * |        |          |04 = SPI1_MISO
     * |        |          |07 = UART0_nCTS
     * |        |          |08 = I2C1_SMBAL
     * |        |          |11 = BPWM3_CH2
     * |        |          |12 = BPWM1_CH0
     * |        |          |14 = TM0
     * |        |          |15 = INT3
     * @var SYS_T::GPC_MFPH
     * Offset: 0x44  GPIOC High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:24] |PC14MFP   |PC.14 Multi-function Pin Selection
     * |        |          |02 = SPI1_MOSI
     * |        |          |04 = SPI0_I2SMCLK
     * |        |          |13 = TM1
     * |        |          |15 = DAC1_ST
     * @var SYS_T::GPD_MFPL
     * Offset: 0x48  GPIOD Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PD0MFP    |PD.0 Multi-function Pin Selection
     * |        |          |04 = SPI0_MOSI
     * |        |          |14 = TM2
     * |[7:4]   |PD1MFP    |PD.1 Multi-function Pin Selection
     * |        |          |04 = SPI0_MISO
     * |[11:8]  |PD2MFP    |PD.2 Multi-function Pin Selection
     * |        |          |04 = SPI0_CLK
     * |        |          |09 = UART0_RXD
     * |[15:12] |PD3MFP    |PD.3 Multi-function Pin Selection
     * |        |          |04 = SPI0_SS
     * |        |          |09 = UART0_TXD
     * @var SYS_T::GPD_MFPH
     * Offset: 0x4C  GPIOD High Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:28] |PD15MFP   |PD.15 Multi-function Pin Selection
     * |        |          |12 = BPWM2_CH5
     * |        |          |14 = TM3
     * |        |          |15 = INT1
     * @var SYS_T::GPF_MFPL
     * Offset: 0x58  GPIOF Low Byte Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PF0MFP    |PF.0 Multi-function Pin Selection
     * |        |          |02 = UART1_TXD
     * |        |          |03 = I2C1_SCL
     * |        |          |04 = UART0_TXD
     * |        |          |07 = I2C2_SCL
     * |        |          |08 = UART2_TXD
     * |        |          |09 = I2C1_SMBAL
     * |        |          |12 = BPWM1_CH0
     * |        |          |13 = ACMP0_O
     * |        |          |14 = ICE_DAT
     * |[7:4]   |PF1MFP    |PF.1 Multi-function Pin Selection
     * |        |          |02 = UART1_RXD
     * |        |          |03 = I2C1_SDA
     * |        |          |04 = UART0_RXD
     * |        |          |07 = I2C2_SDA
     * |        |          |08 = UART2_RXD
     * |        |          |09 = I2C1_SMBSUS
     * |        |          |12 = BPWM1_CH1
     * |        |          |13 = ACMP1_O
     * |        |          |14 = ICE_CLK
     * |[11:8]  |PF2MFP    |PF.2 Multi-function Pin Selection
     * |        |          |03 = UART0_RXD
     * |        |          |04 = I2C0_SDA
     * |        |          |07 = UART1_RXD
     * |        |          |10 = XT1_OUT
     * |        |          |11 = BPWM1_CH1
     * |        |          |13 = ACMP3_O
     * |        |          |15 = INT4
     * |[15:12] |PF3MFP    |PF.3 Multi-function Pin Selection
     * |        |          |03 = UART0_TXD
     * |        |          |04 = I2C0_SCL
     * |        |          |07 = UART2_RXD
     * |        |          |09 = BPWM0_CH3
     * |        |          |10 = XT1_IN
     * |        |          |11 = BPWM1_CH0
     * |        |          |13 = ACMP2_O
     * |[19:16] |PF4MFP    |PF.4 Multi-function Pin Selection
     * |        |          |02 = UART2_TXD
     * |        |          |04 = UART2_nRTS
     * |        |          |05 = SPI0_MISO
     * |        |          |07 = BPWM2_CH1
     * |        |          |08 = BPWM0_CH5
     * |        |          |09 = I2C0_SDA
     * |        |          |10 = X32_OUT
     * |        |          |11 = BPWM2_CH5
     * |        |          |12 = BPWM3_CH5
     * |        |          |13 = ACMP3_WLAT
     * |[23:20] |PF5MFP    |PF.5 Multi-function Pin Selection
     * |        |          |02 = UART2_RXD
     * |        |          |03 = SPI1_SS
     * |        |          |04 = UART2_nCTS
     * |        |          |05 = SPI0_CLK
     * |        |          |07 = BPWM2_CH0
     * |        |          |08 = BPWM0_CH4
     * |        |          |09 = I2C0_SCL
     * |        |          |10 = X32_IN
     * |        |          |11 = ADC0_ST
     * |        |          |13 = ACMP2_WLAT
     * |[27:24] |PF6MFP    |PF.6 Multi-function Pin Selection
     * |        |          |05 = SPI0_MOSI
     * |        |          |10 = I2C0_SDA
     * |        |          |11 = LLSI3_OUT
     * |        |          |12 = BPWM2_CH4
     * |        |          |13 = CLKO
     * |        |          |14 = TM3
     * |        |          |15 = INT5
     * @var SYS_T::IRCTCTL
     * Offset: 0x80  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of internal high speed RC oscillator (HIRC) auto trim.
     * |        |          |During auto trim operation, if clock error detected with CESTOPEN(SYS_IRCTCTL[8]) is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
     * |        |          |00 = Disable HIRC auto trim function.
     * |        |          |01 = Enable HIRC auto trim function and trim HIRC to 48 MHz.
     * |        |          |10 = Reserved.
     * |        |          |11 = Reserved.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many clocks of reference clock (32.768 kHz, LXT).
     * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
     * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC still does not lock, the auto trim operation will be disabled and FREQSEL(SYS_IRCTCTL[1:0]) will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = The trim operation is keep going if clock is inaccurate.
     * |        |          |1 = The trim operation is stopped if clock is inaccurate.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = HIRC trim reference clock is from LXT (32.768 kHz).
     * |        |          |1 = HIRC trim reference clock is from USB synchronous mode packet.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |Fill the boundary range from 0x1 to 0x1F, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_IRCTCTL[9]) is enabled.  
     * @var SYS_T::IRCTIEN
     * Offset: 0x84  HIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |HIRC Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL(SYS_IRCTCTL[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_IRCTSTS[1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count reached.
     * |        |          |0 = Disable TFAILIF(SYS_IRCTSTS[1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_IRCTSTS[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |HIRC Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while HIRC clock is inaccurate during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_IRCTSTS[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccurate.
     * |        |          |0 = Disable CLKERRIF(SYS_IRCTSTS[2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_IRCTSTS[2]) status to trigger an interrupt to CPU.
     * @var SYS_T::IRCTISTS
     * Offset: 0x88  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency is locked.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |0 = The internal high-speed RC oscillator frequency does not lock at 48 MHz yet.
     * |        |          |1 = The internal high-speed RC oscillator frequency locked at 48 MHz.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency still does not be locked.
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_IRCTCTL[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_IRCTIEN[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count reached.
     * |        |          |Write 1 to clear this to 0.
     * |        |          |0 = Trim value update limitation count not reached.
     * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or 48 MHz internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy.
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_IRCTCTL[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_IRCTCTL[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_IRCTIEN[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccurate.
     * |        |          |Write 1 to clear this to 0.
     * |        |          |0 = Clock frequency is accurate.
     * |        |          |1 = Clock frequency is inaccurate.
     * @var SYS_T::MODCTL
     * Offset: 0xC0  Modulation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MODEN     |Modulation Function Enable Bit
     * |        |          |This bit enables modulation funcion by modulating with BPWM channel output and UART1_TXD.
     * |        |          |0 = Modulation Function Disabled.
     * |        |          |1 = Modulation Function Enabled.
     * |[1]     |MODH      |Modulation at Data High
     * |        |          |Select modulation pulse(BPWM) at UART1_TXD high or low.
     * |        |          |0 = Modulation pulse at UART1_TXD low.
     * |        |          |1 = Modulation pulse at UART1_TXD high.
     * |[6:4]   |MODPWMSEL |PWM0 Channel Select for Modulation
     * |        |          |Select the PWM0 channel to modulate with the UART1_TXD.
     * |        |          |000 = BPWM0 channel 0 modulate with UART1_TXD.
     * |        |          |001 = BPWM0 channel 1 modulate with UART1_TXD.
     * |        |          |010 = BPWM0 channel 2 modulate with UART1_TXD.
     * |        |          |011 = BPWM0 channel 3 modulete with UART1_TXD.
     * |        |          |Others = Reserved.
     * |        |          |Note: This bis is valid while MODEN (SYS_MODCTL[0]) is set to 1.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Lock Control Code
     * |        |          |Write operation:
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * |        |          |Read operation:
     * |        |          |0 = Write-protection Enabled for writing protected registers. Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
    * @var SYS_T::TSCTL
     * Offset: 0x140  Temperature Sensor Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TSEN      |Temperature Sensor Enable Bit
     * |        |          |0 = Temperature Sensor function Disabled. (Default)
     * |        |          |1 = Temperature Sensor function Enabled.
     * |        |          |Note: TSEN should be enabled only when TSBGEN is enabled.
     * |[1]     |TSBGEN    |Temperature Sensor Bandgap Enable Bit
     * |        |          |If this bit is set to 1, Temperature sensor bandgap will be enabled and Temperature sensor will supply current source to DAC and Internal Voltage Reference.
     * |        |          |0 = Temperature Sensor Bandgap function Disabled. (Default)
     * |        |          |1 = Temperature Sensor Bandgap function Enabled.
     * |        |          |Note: After TSBGEN is set, users should wait 200us stable time to enable DACEN(DACx_CTL[0]) or VREFEN (SYS_VREFCTL[0]).
     * |[2]     |TSST      |Temperature Sensor Conversion Start
     * |        |          |0 = Conversion stops or finished. (Default)
     * |        |          |1 = Conversion starts.
     * |        |          |Note 1: User needs to set TSEN first, and wait for at least 200us to start temperature sensor conversion.
     * |        |          |Note 2: This bit will be cleared to 0 by hardware automatically when temperature sensor conversion is finished.
     * |[3]     |TSIEN     |Temperature Sensor Interrupt Enable Bit
     * |        |          |If this bit is set to 1, temperature sensor interrupt is requested when TSIF (SYS_TSCTL[16]) is set.
     * |        |          |0 = Temperature Sensor finish interrupt Disabled. (Default)
     * |        |          |1 = Temperature Sensor finish interrupt Enabled.
     * |[16]    |TSIF      |Temperature Sensor Interrupt Flag
     * |        |          |This bit indicates the end of temperature sensor conversion.
     * |        |          |This bit will be set automatically when temperature sensor conversion is finished.
     * |        |          |0 = Tempertaure Sensor conversion not finished.
     * |        |          |1 = Tempertaure Sensor conversion finished.
     * |        |          |Note: Write 1 to clear this to 0.
     * @var SYS_T::TSDATA
     * Offset: 0x144  Temperature Sensor Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TSEOC     |Temperature Sensor Conversion Finish Flag
     * |        |          |This bit indicates the end of temperature sensor conversion.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[27:16] |TSDATA    |Temperature Sensor Conversion Data Bits (Read Only)
     * |        |          |This field present the conversion result of Temperature Sensor, ranges from -40 to 105 degrees Celsius.
     * |        |          |Note: Negative temperature is represented by 2's complement format, and per LSB difference is equivalent to 0.0625 degrees Celsius.
     * @var SYS_T::SPDHCTL
     * Offset: 0x150  SPD5 Hub Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31]    |SPDHEN    |SPD5 Hub Controller Enable Bit (Write Protect)
     * |        |          |This bit is used to enable the whole SPD5 Hub controller function.
     * |        |          |When SPD5 Hub function is enabled, I3C0 and I3C1 are for DDR5 platform purpose only.
     * |        |          |When SPD5 Hub function is disabled, I3C0 and I3C1 are general purpose I3C slave.
     * |        |          |0 = SPD5 Hub function Disabled.
     * |        |          |1 = SPD5 Hub function Enable.
     * |[30]    |HSADIS    |HSA Resistor Disable Bit (Write Protect)
     * |        |          |This bit is used to disable SPD5 Hub HSA resistor.
     * |        |          |0 = SPD5 Hub HSA Resistor enabled.
     * |        |          |1 = SPD5 Hub HSA Resistor disabled (default).
     * @var SYS_T::SPIMUX
     * Offset: 0x160  SPI Mux Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MUXSWEN   |SPI Mux Software Selection Mode Enable Bit
     * |        |          |This bit is used to enable/disable SPI Mux software selection mode function.
     * |        |          |0 = SPI Mux software selection mode Disabled (default).
     * |        |          |1 = SPI Mux software selection mode Enabled.
     * |        |          |Note: By default, SPI mux is controlled by SPI_SS_MUX.
     * |[1]     |MUXSWSEL  |SPI Mux Software Selection Mode Selection Bit
     * |        |          |This bit is used to select which SPI master's signals are going to pass through the SPI mux in software selection mode.
     * |        |          |0 = Choose external SPI master (default).
     * |        |          |1 = Choose internal SPI master
     */

    __I  uint32_t PDID;                  /*!< [0x0000] Part Device Identification Number Register                       */
    __IO uint32_t RSTSTS;                /*!< [0x0004] System Reset Status Register                                     */
    __IO uint32_t IPRST0;                /*!< [0x0008] Peripheral  Reset Control Register 0                             */
    __IO uint32_t IPRST1;                /*!< [0x000c] Peripheral Reset Control Register 1                              */
    __IO uint32_t IPRST2;                /*!< [0x0010] Peripheral Reset Control Register 2                              */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t BODCTL;                /*!< [0x0018] Brown-out Detector Control Register                              */
    __IO uint32_t IVSCTL;                /*!< [0x001c] Internal Voltage Source Control Register                         */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t PORCTL;                /*!< [0x0024] Power-on Reset Controller Register                               */
    __IO uint32_t VREFCTL;               /*!< [0x0028] VREF Control Register                                            */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t GPA_MFPL;              /*!< [0x0030] GPIOA Low Byte Multiple Function Control Register                */
    __IO uint32_t GPA_MFPH;              /*!< [0x0034] GPIOA High Byte Multiple Function Control Register               */
    __IO uint32_t GPB_MFPL;              /*!< [0x0038] GPIOB Low Byte Multiple Function Control Register                */
    __IO uint32_t GPB_MFPH;              /*!< [0x003c] GPIOB High Byte Multiple Function Control Register               */
    __IO uint32_t GPC_MFPL;              /*!< [0x0040] GPIOC Low Byte Multiple Function Control Register                */
    __IO uint32_t GPC_MFPH;              /*!< [0x0044] GPIOC High Byte Multiple Function Control Register               */
    __IO uint32_t GPD_MFPL;              /*!< [0x0048] GPIOD Low Byte Multiple Function Control Register                */
    __IO uint32_t GPD_MFPH;              /*!< [0x004c] GPIOD High Byte Multiple Function Control Register               */
    __I  uint32_t RESERVE3[2];
    __IO uint32_t GPF_MFPL;              /*!< [0x0058] GPIOF Low Byte Multiple Function Control Register                */
    __I  uint32_t RESERVE4[9];
    __IO uint32_t IRCTCTL;               /*!< [0x0080] HIRC Trim Control Register                                       */
    __IO uint32_t IRCTIEN;               /*!< [0x0084] HIRC Trim Interrupt Enable Register                              */
    __IO uint32_t IRCTISTS;              /*!< [0x0088] HIRC Trim Interrupt Status Register                              */
    __I  uint32_t RESERVE5[13];
    __IO uint32_t MODCTL;                /*!< [0x00c0] Modulation Control Register                                      */
    __I  uint32_t RESERVE6[15];
    __IO uint32_t REGLCTL;               /*!< [0x0100] Register Lock Control Register                                   */
    __I  uint32_t RESERVE7[15];
    __IO uint32_t TSCTL;                 /*!< [0x0140] Temperature Sensor Control Register                              */
    __IO uint32_t TSDATA;                /*!< [0x0144] Temperature Sensor Data Register                                 */
    __I  uint32_t RESERVE8[2];
    __IO uint32_t SPDHCTL;               /*!< [0x0150] SPD5 Hub Control Register                                        */
    __I  uint32_t RESERVE9[3];
    __IO uint32_t SPIMUX;                /*!< [0x0160] SPI Mux Function Control Register                                */


} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
    @{
*/

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position             */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                 */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position          */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS_T::RSTSTS: LVRF Position           */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS_T::RSTSTS: LVRF Mask               */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position          */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask              */

#define SYS_RSTSTS_MCURF_Pos             (5)                                               /*!< SYS_T::RSTSTS: MCURF Position          */
#define SYS_RSTSTS_MCURF_Msk             (0x1ul << SYS_RSTSTS_MCURF_Pos)                   /*!< SYS_T::RSTSTS: MCURF Mask              */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position          */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask              */

#define SYS_RSTSTS_CPULKRF_Pos           (8)                                               /*!< SYS_T::RSTSTS: CPULKRF Position        */
#define SYS_RSTSTS_CPULKRF_Msk           (0x1ul << SYS_RSTSTS_CPULKRF_Pos)                 /*!< SYS_T::RSTSTS: CPULKRF Mask            */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position        */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask            */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position         */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask             */

#define SYS_IPRST0_PDMARST_Pos           (2)                                               /*!< SYS_T::IPRST0: PDMARST Position        */
#define SYS_IPRST0_PDMARST_Msk           (0x1ul << SYS_IPRST0_PDMARST_Pos)                 /*!< SYS_T::IPRST0: PDMARST Mask            */

#define SYS_IPRST0_CRCRST_Pos            (7)                                               /*!< SYS_T::IPRST0: CRCRST Position         */
#define SYS_IPRST0_CRCRST_Msk            (0x1ul << SYS_IPRST0_CRCRST_Pos)                  /*!< SYS_T::IPRST0: CRCRST Mask             */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position        */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask            */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position        */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask            */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position        */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask            */

#define SYS_IPRST1_TMR2RST_Pos           (4)                                               /*!< SYS_T::IPRST1: TMR2RST Position        */
#define SYS_IPRST1_TMR2RST_Msk           (0x1ul << SYS_IPRST1_TMR2RST_Pos)                 /*!< SYS_T::IPRST1: TMR2RST Mask            */

#define SYS_IPRST1_TMR3RST_Pos           (5)                                               /*!< SYS_T::IPRST1: TMR3RST Position        */
#define SYS_IPRST1_TMR3RST_Msk           (0x1ul << SYS_IPRST1_TMR3RST_Pos)                 /*!< SYS_T::IPRST1: TMR3RST Mask            */

#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C0RST Position        */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS_T::IPRST1: I2C0RST Mask            */

#define SYS_IPRST1_I2C1RST_Pos           (9)                                               /*!< SYS_T::IPRST1: I2C1RST Position        */
#define SYS_IPRST1_I2C1RST_Msk           (0x1ul << SYS_IPRST1_I2C1RST_Pos)                 /*!< SYS_T::IPRST1: I2C1RST Mask            */

#define SYS_IPRST1_I2C2RST_Pos           (10)                                              /*!< SYS_T::IPRST1: I2C2RST Position        */
#define SYS_IPRST1_I2C2RST_Msk           (0x1ul << SYS_IPRST1_I2C2RST_Pos)                 /*!< SYS_T::IPRST1: I2C2RST Mask            */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS_T::IPRST1: SPI0RST Position        */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS_T::IPRST1: SPI0RST Mask            */

#define SYS_IPRST1_SPI1RST_Pos           (13)                                              /*!< SYS_T::IPRST1: SPI1RST Position        */
#define SYS_IPRST1_SPI1RST_Msk           (0x1ul << SYS_IPRST1_SPI1RST_Pos)                 /*!< SYS_T::IPRST1: SPI1RST Mask            */

#define SYS_IPRST1_SPI2RST_Pos           (14)                                              /*!< SYS_T::IPRST1: SPI2RST Position        */
#define SYS_IPRST1_SPI2RST_Msk           (0x1ul << SYS_IPRST1_SPI2RST_Pos)                 /*!< SYS_T::IPRST1: SPI2RST Mask            */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position       */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask           */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position       */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask           */

#define SYS_IPRST1_UART2RST_Pos          (18)                                              /*!< SYS_T::IPRST1: UART2RST Position       */
#define SYS_IPRST1_UART2RST_Msk          (0x1ul << SYS_IPRST1_UART2RST_Pos)                /*!< SYS_T::IPRST1: UART2RST Mask           */

#define SYS_IPRST1_BPWM0RST_Pos          (20)                                              /*!< SYS_T::IPRST1: BPWM0RST Position       */
#define SYS_IPRST1_BPWM0RST_Msk          (0x1ul << SYS_IPRST1_BPWM0RST_Pos)                /*!< SYS_T::IPRST1: BPWM0RST Mask           */

#define SYS_IPRST1_BPWM1RST_Pos          (21)                                              /*!< SYS_T::IPRST1: BPWM1RST Position       */
#define SYS_IPRST1_BPWM1RST_Msk          (0x1ul << SYS_IPRST1_BPWM1RST_Pos)                /*!< SYS_T::IPRST1: BPWM1RST Mask           */

#define SYS_IPRST1_BPWM2RST_Pos          (22)                                              /*!< SYS_T::IPRST1: BPWM2RST Position       */
#define SYS_IPRST1_BPWM2RST_Msk          (0x1ul << SYS_IPRST1_BPWM2RST_Pos)                /*!< SYS_T::IPRST1: BPWM2RST Mask           */

#define SYS_IPRST1_BPWM3RST_Pos          (23)                                              /*!< SYS_T::IPRST1: BPWM3RST Position       */
#define SYS_IPRST1_BPWM3RST_Msk          (0x1ul << SYS_IPRST1_BPWM3RST_Pos)                /*!< SYS_T::IPRST1: BPWM3RST Mask           */

#define SYS_IPRST1_I3CS0RST_Pos          (24)                                              /*!< SYS_T::IPRST1: I3CS0RST Position       */
#define SYS_IPRST1_I3CS0RST_Msk          (0x1ul << SYS_IPRST1_I3CS0RST_Pos)                /*!< SYS_T::IPRST1: I3CS0RST Mask           */

#define SYS_IPRST1_I3CS1RST_Pos          (25)                                              /*!< SYS_T::IPRST1: I3CS1RST Position       */
#define SYS_IPRST1_I3CS1RST_Msk          (0x1ul << SYS_IPRST1_I3CS1RST_Pos)                /*!< SYS_T::IPRST1: I3CS1RST Mask           */

#define SYS_IPRST1_SPDHRST_Pos           (26)                                              /*!< SYS_T::IPRST1: SPDhRST Position        */
#define SYS_IPRST1_SPDHRST_Msk           (0x1ul << SYS_IPRST1_SPDHRST_Pos)                 /*!< SYS_T::IPRST1: SPDHRST Mask            */

#define SYS_IPRST1_USBDRST_Pos           (27)                                              /*!< SYS_T::IPRST1: USBDRST Position        */
#define SYS_IPRST1_USBDRST_Msk           (0x1ul << SYS_IPRST1_USBDRST_Pos)                 /*!< SYS_T::IPRST1: USBDRST Mask            */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS_T::IPRST1: ADCRST Position         */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS_T::IPRST1: ADCRST Mask             */

#define SYS_IPRST1_DACRST_Pos            (29)                                              /*!< SYS_T::IPRST1: DACRST Position         */
#define SYS_IPRST1_DACRST_Msk            (0x1ul << SYS_IPRST1_DACRST_Pos)                  /*!< SYS_T::IPRST1: DACRST Mask             */

#define SYS_IPRST1_ACMP01RST_Pos         (30)                                              /*!< SYS_T::IPRST1: ACMP01RST Position      */
#define SYS_IPRST1_ACMP01RST_Msk         (0x1ul << SYS_IPRST1_ACMP01RST_Pos)               /*!< SYS_T::IPRST1: ACMP01RST Mask          */

#define SYS_IPRST1_ACMP23RST_Pos         (31)                                              /*!< SYS_T::IPRST1: ACMP23RST Position      */
#define SYS_IPRST1_ACMP23RST_Msk         (0x1ul << SYS_IPRST1_ACMP23RST_Pos)               /*!< SYS_T::IPRST1: ACMP23RST Mask          */

#define SYS_IPRST2_LLSI0RST_Pos          (16)                                              /*!< SYS_T::IPRST2: LLSI0RST Position       */
#define SYS_IPRST2_LLSI0RST_Msk          (0x1ul << SYS_IPRST2_LLSI0RST_Pos)                /*!< SYS_T::IPRST2: LLSI0RST Mask           */

#define SYS_IPRST2_LLSI1RST_Pos          (17)                                              /*!< SYS_T::IPRST2: LLSI1RST Position       */
#define SYS_IPRST2_LLSI1RST_Msk          (0x1ul << SYS_IPRST2_LLSI1RST_Pos)                /*!< SYS_T::IPRST2: LLSI1RST Mask           */

#define SYS_IPRST2_LLSI2RST_Pos          (18)                                              /*!< SYS_T::IPRST2: LLSI2RST Position       */
#define SYS_IPRST2_LLSI2RST_Msk          (0x1ul << SYS_IPRST2_LLSI2RST_Pos)                /*!< SYS_T::IPRST2: LLSI2RST Mask           */

#define SYS_IPRST2_LLSI3RST_Pos          (19)                                              /*!< SYS_T::IPRST2: LLSI3RST Position       */
#define SYS_IPRST2_LLSI3RST_Msk          (0x1ul << SYS_IPRST2_LLSI3RST_Pos)                /*!< SYS_T::IPRST2: LLSI3RST Mask           */

#define SYS_IPRST2_LLSI4RST_Pos          (20)                                              /*!< SYS_T::IPRST2: LLSI4RST Position       */
#define SYS_IPRST2_LLSI4RST_Msk          (0x1ul << SYS_IPRST2_LLSI4RST_Pos)                /*!< SYS_T::IPRST2: LLSI4RST Mask           */

#define SYS_IPRST2_LLSI5RST_Pos          (21)                                              /*!< SYS_T::IPRST2: LLSI5RST Position       */
#define SYS_IPRST2_LLSI5RST_Msk          (0x1ul << SYS_IPRST2_LLSI5RST_Pos)                /*!< SYS_T::IPRST2: LLSI5RST Mask           */

#define SYS_IPRST2_TSRST_Pos             (28)                                              /*!< SYS_T::IPRST2: TSRST Position          */
#define SYS_IPRST2_TSRST_Msk             (0x1ul << SYS_IPRST2_TSRST_Pos)                   /*!< SYS_T::IPRST2: TSRST Mask              */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position          */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask              */

#define SYS_BODCTL_BODVL_Pos             (1)                                               /*!< SYS_T::BODCTL: BODVL Position          */
#define SYS_BODCTL_BODVL_Msk             (0x3ul << SYS_BODCTL_BODVL_Pos)                   /*!< SYS_T::BODCTL: BODVL Mask              */

#define SYS_BODCTL_BODRSTEN_Pos          (3)                                               /*!< SYS_T::BODCTL: BODRSTEN Position       */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask           */

#define SYS_BODCTL_BODIF_Pos             (4)                                               /*!< SYS_T::BODCTL: BODIF Position          */
#define SYS_BODCTL_BODIF_Msk             (0x1ul << SYS_BODCTL_BODIF_Pos)                   /*!< SYS_T::BODCTL: BODIF Mask              */

#define SYS_BODCTL_BODLPM_Pos            (5)                                               /*!< SYS_T::BODCTL: BODLPM Position         */
#define SYS_BODCTL_BODLPM_Msk            (0x1ul << SYS_BODCTL_BODLPM_Pos)                  /*!< SYS_T::BODCTL: BODLPM Mask             */

#define SYS_BODCTL_BODOUT_Pos            (6)                                               /*!< SYS_T::BODCTL: BODOUT Position         */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask             */

#define SYS_BODCTL_LVREN_Pos             (7)                                               /*!< SYS_T::BODCTL: LVREN Position          */
#define SYS_BODCTL_LVREN_Msk             (0x1ul << SYS_BODCTL_LVREN_Pos)                   /*!< SYS_T::BODCTL: LVREN Mask              */

#define SYS_BODCTL_BODDGSEL_Pos          (8)                                               /*!< SYS_T::BODCTL: BODDGSEL Position       */
#define SYS_BODCTL_BODDGSEL_Msk          (0x7ul << SYS_BODCTL_BODDGSEL_Pos)                /*!< SYS_T::BODCTL: BODDGSEL Mask           */

#define SYS_BODCTL_LVRDGSEL_Pos          (12)                                              /*!< SYS_T::BODCTL: LVRDGSEL Position       */
#define SYS_BODCTL_LVRDGSEL_Msk          (0x7ul << SYS_BODCTL_LVRDGSEL_Pos)                /*!< SYS_T::BODCTL: LVRDGSEL Mask           */

#define SYS_BODCTL_VDETEN_Pos            (16)                                              /*!< SYS_T::BODCTL: VDETEN Position         */
#define SYS_BODCTL_VDETEN_Msk            (0x1ul << SYS_BODCTL_VDETEN_Pos)                  /*!< SYS_T::BODCTL: VDETEN Mask             */

#define SYS_BODCTL_VDETPINSEL_Pos        (17)                                              /*!< SYS_T::BODCTL: VDETPINSEL Position     */
#define SYS_BODCTL_VDETPINSEL_Msk        (0x1ul << SYS_BODCTL_VDETPINSEL_Pos)              /*!< SYS_T::BODCTL: VDETPINSEL Mask         */

#define SYS_BODCTL_VDETIEN_Pos           (18)                                              /*!< SYS_T::BODCTL: VDETIEN Position        */
#define SYS_BODCTL_VDETIEN_Msk           (0x1ul << SYS_BODCTL_VDETIEN_Pos)                 /*!< SYS_T::BODCTL: VDETIEN Mask            */

#define SYS_BODCTL_VDETIF_Pos            (19)                                              /*!< SYS_T::BODCTL: VDETIF Position         */
#define SYS_BODCTL_VDETIF_Msk            (0x1ul << SYS_BODCTL_VDETIF_Pos)                  /*!< SYS_T::BODCTL: VDETIF Mask             */

#define SYS_BODCTL_VDETOUT_Pos           (24)                                              /*!< SYS_T::BODCTL: VDETOUT Position        */
#define SYS_BODCTL_VDETOUT_Msk           (0x1ul << SYS_BODCTL_VDETOUT_Pos)                 /*!< SYS_T::BODCTL: VDETOUT Mask            */

#define SYS_BODCTL_VDETDGSEL_Pos         (25)                                              /*!< SYS_T::BODCTL: VDETDGSEL Position      */
#define SYS_BODCTL_VDETDGSEL_Msk         (0x7ul << SYS_BODCTL_VDETDGSEL_Pos)               /*!< SYS_T::BODCTL: VDETDGSEL Mask          */

#define SYS_IVSCTL_VBGUGEN_Pos           (1)                                               /*!< SYS_T::IVSCTL: VBGUGEN Position        */
#define SYS_IVSCTL_VBGUGEN_Msk           (0x1ul << SYS_IVSCTL_VBGUGEN_Pos)                 /*!< SYS_T::IVSCTL: VBGUGEN Mask            */

#define SYS_PORCTL_POROFF_Pos            (0)                                               /*!< SYS_T::PORCTL: POROFF Position         */
#define SYS_PORCTL_POROFF_Msk            (0xfffful << SYS_PORCTL_POROFF_Pos)               /*!< SYS_T::PORCTL: POROFF Mask             */

#define SYS_VREFCTL_VREFCTL_Pos          (0)                                               /*!< SYS_T::VREFCTL: VREFCTL Position       */
#define SYS_VREFCTL_VREFCTL_Msk          (0x1ful << SYS_VREFCTL_VREFCTL_Pos)               /*!< SYS_T::VREFCTL: VREFCTL Mask           */

#define SYS_VREFCTL_PRELOADEN_Pos        (6)                                               /*!< SYS_T::VREFCTL: PRELOADEN Position     */
#define SYS_VREFCTL_PRELOADEN_Msk        (0x1ul << SYS_VREFCTL_PRELOADEN_Pos)              /*!< SYS_T::VREFCTL: PRELOADEN Mask         */

#define SYS_VREFCTL_VBGFEN_Pos           (24)                                              /*!< SYS_T::VREFCTL: VBGFEN Position        */
#define SYS_VREFCTL_VBGFEN_Msk           (0x1ul << SYS_VREFCTL_VBGFEN_Pos)                 /*!< SYS_T::VREFCTL: VBGFEN Mask            */

#define SYS_GPA_MFPL_PA0MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFPL: PA0MFP Position       */
#define SYS_GPA_MFPL_PA0MFP_Msk          (0xful << SYS_GPA_MFPL_PA0MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA0MFP Mask           */

#define SYS_GPA_MFPL_PA1MFP_Pos          (4)                                               /*!< SYS_T::GPA_MFPL: PA1MFP Position       */
#define SYS_GPA_MFPL_PA1MFP_Msk          (0xful << SYS_GPA_MFPL_PA1MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA1MFP Mask           */

#define SYS_GPA_MFPL_PA2MFP_Pos          (8)                                               /*!< SYS_T::GPA_MFPL: PA2MFP Position       */
#define SYS_GPA_MFPL_PA2MFP_Msk          (0xful << SYS_GPA_MFPL_PA2MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA2MFP Mask           */

#define SYS_GPA_MFPL_PA3MFP_Pos          (12)                                              /*!< SYS_T::GPA_MFPL: PA3MFP Position       */
#define SYS_GPA_MFPL_PA3MFP_Msk          (0xful << SYS_GPA_MFPL_PA3MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA3MFP Mask           */

#define SYS_GPA_MFPL_PA4MFP_Pos          (16)                                              /*!< SYS_T::GPA_MFPL: PA4MFP Position       */
#define SYS_GPA_MFPL_PA4MFP_Msk          (0xful << SYS_GPA_MFPL_PA4MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA4MFP Mask           */

#define SYS_GPA_MFPL_PA5MFP_Pos          (20)                                              /*!< SYS_T::GPA_MFPL: PA5MFP Position       */
#define SYS_GPA_MFPL_PA5MFP_Msk          (0xful << SYS_GPA_MFPL_PA5MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA5MFP Mask           */

#define SYS_GPA_MFPL_PA6MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFPL: PA6MFP Position       */
#define SYS_GPA_MFPL_PA6MFP_Msk          (0xful << SYS_GPA_MFPL_PA6MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA6MFP Mask           */

#define SYS_GPA_MFPL_PA7MFP_Pos          (28)                                              /*!< SYS_T::GPA_MFPL: PA7MFP Position       */
#define SYS_GPA_MFPL_PA7MFP_Msk          (0xful << SYS_GPA_MFPL_PA7MFP_Pos)                /*!< SYS_T::GPA_MFPL: PA7MFP Mask           */

#define SYS_GPA_MFPH_PA8MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFPH: PA8MFP Position       */
#define SYS_GPA_MFPH_PA8MFP_Msk          (0xful << SYS_GPA_MFPH_PA8MFP_Pos)                /*!< SYS_T::GPA_MFPH: PA8MFP Mask           */

#define SYS_GPA_MFPH_PA9MFP_Pos          (4)                                               /*!< SYS_T::GPA_MFPH: PA9MFP Position       */
#define SYS_GPA_MFPH_PA9MFP_Msk          (0xful << SYS_GPA_MFPH_PA9MFP_Pos)                /*!< SYS_T::GPA_MFPH: PA9MFP Mask           */

#define SYS_GPA_MFPH_PA10MFP_Pos         (8)                                               /*!< SYS_T::GPA_MFPH: PA10MFP Position      */
#define SYS_GPA_MFPH_PA10MFP_Msk         (0xful << SYS_GPA_MFPH_PA10MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA10MFP Mask          */

#define SYS_GPA_MFPH_PA11MFP_Pos         (12)                                              /*!< SYS_T::GPA_MFPH: PA11MFP Position      */
#define SYS_GPA_MFPH_PA11MFP_Msk         (0xful << SYS_GPA_MFPH_PA11MFP_Pos)               /*!< SYS_T::GPA_MFPH: PA11MFP Mask          */

#define SYS_GPB_MFPL_PB0MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFPL: PB0MFP Position       */
#define SYS_GPB_MFPL_PB0MFP_Msk          (0xful << SYS_GPB_MFPL_PB0MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB0MFP Mask           */

#define SYS_GPB_MFPL_PB1MFP_Pos          (4)                                               /*!< SYS_T::GPB_MFPL: PB1MFP Position       */
#define SYS_GPB_MFPL_PB1MFP_Msk          (0xful << SYS_GPB_MFPL_PB1MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB1MFP Mask           */

#define SYS_GPB_MFPL_PB2MFP_Pos          (8)                                               /*!< SYS_T::GPB_MFPL: PB2MFP Position       */
#define SYS_GPB_MFPL_PB2MFP_Msk          (0xful << SYS_GPB_MFPL_PB2MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB2MFP Mask           */

#define SYS_GPB_MFPL_PB3MFP_Pos          (12)                                              /*!< SYS_T::GPB_MFPL: PB3MFP Position       */
#define SYS_GPB_MFPL_PB3MFP_Msk          (0xful << SYS_GPB_MFPL_PB3MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB3MFP Mask           */

#define SYS_GPB_MFPL_PB4MFP_Pos          (16)                                              /*!< SYS_T::GPB_MFPL: PB4MFP Position       */
#define SYS_GPB_MFPL_PB4MFP_Msk          (0xful << SYS_GPB_MFPL_PB4MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB4MFP Mask           */

#define SYS_GPB_MFPL_PB5MFP_Pos          (20)                                              /*!< SYS_T::GPB_MFPL: PB5MFP Position       */
#define SYS_GPB_MFPL_PB5MFP_Msk          (0xful << SYS_GPB_MFPL_PB5MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB5MFP Mask           */

#define SYS_GPB_MFPL_PB6MFP_Pos          (24)                                              /*!< SYS_T::GPB_MFPL: PB6MFP Position       */
#define SYS_GPB_MFPL_PB6MFP_Msk          (0xful << SYS_GPB_MFPL_PB6MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB6MFP Mask           */

#define SYS_GPB_MFPL_PB7MFP_Pos          (28)                                              /*!< SYS_T::GPB_MFPL: PB7MFP Position       */
#define SYS_GPB_MFPL_PB7MFP_Msk          (0xful << SYS_GPB_MFPL_PB7MFP_Pos)                /*!< SYS_T::GPB_MFPL: PB7MFP Mask           */

#define SYS_GPB_MFPH_PB8MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFPH: PB8MFP Position       */
#define SYS_GPB_MFPH_PB8MFP_Msk          (0xful << SYS_GPB_MFPH_PB8MFP_Pos)                /*!< SYS_T::GPB_MFPH: PB8MFP Mask           */

#define SYS_GPB_MFPH_PB9MFP_Pos          (4)                                               /*!< SYS_T::GPB_MFPH: PB9MFP Position       */
#define SYS_GPB_MFPH_PB9MFP_Msk          (0xful << SYS_GPB_MFPH_PB9MFP_Pos)                /*!< SYS_T::GPB_MFPH: PB9MFP Mask           */

#define SYS_GPB_MFPH_PB10MFP_Pos         (8)                                               /*!< SYS_T::GPB_MFPH: PB10MFP Position      */
#define SYS_GPB_MFPH_PB10MFP_Msk         (0xful << SYS_GPB_MFPH_PB10MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB10MFP Mask          */

#define SYS_GPB_MFPH_PB11MFP_Pos         (12)                                              /*!< SYS_T::GPB_MFPH: PB11MFP Position      */
#define SYS_GPB_MFPH_PB11MFP_Msk         (0xful << SYS_GPB_MFPH_PB11MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB11MFP Mask          */

#define SYS_GPB_MFPH_PB12MFP_Pos         (16)                                              /*!< SYS_T::GPB_MFPH: PB12MFP Position      */
#define SYS_GPB_MFPH_PB12MFP_Msk         (0xful << SYS_GPB_MFPH_PB12MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB12MFP Mask          */

#define SYS_GPB_MFPH_PB13MFP_Pos         (20)                                              /*!< SYS_T::GPB_MFPH: PB13MFP Position      */
#define SYS_GPB_MFPH_PB13MFP_Msk         (0xful << SYS_GPB_MFPH_PB13MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB13MFP Mask          */

#define SYS_GPB_MFPH_PB14MFP_Pos         (24)                                              /*!< SYS_T::GPB_MFPH: PB14MFP Position      */
#define SYS_GPB_MFPH_PB14MFP_Msk         (0xful << SYS_GPB_MFPH_PB14MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB14MFP Mask          */

#define SYS_GPB_MFPH_PB15MFP_Pos         (28)                                              /*!< SYS_T::GPB_MFPH: PB15MFP Position      */
#define SYS_GPB_MFPH_PB15MFP_Msk         (0xful << SYS_GPB_MFPH_PB15MFP_Pos)               /*!< SYS_T::GPB_MFPH: PB15MFP Mask          */

#define SYS_GPC_MFPL_PC0MFP_Pos          (0)                                               /*!< SYS_T::GPC_MFPL: PC0MFP Position       */
#define SYS_GPC_MFPL_PC0MFP_Msk          (0xful << SYS_GPC_MFPL_PC0MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC0MFP Mask           */

#define SYS_GPC_MFPL_PC1MFP_Pos          (4)                                               /*!< SYS_T::GPC_MFPL: PC1MFP Position       */
#define SYS_GPC_MFPL_PC1MFP_Msk          (0xful << SYS_GPC_MFPL_PC1MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC1MFP Mask           */

#define SYS_GPC_MFPL_PC2MFP_Pos          (8)                                               /*!< SYS_T::GPC_MFPL: PC2MFP Position       */
#define SYS_GPC_MFPL_PC2MFP_Msk          (0xful << SYS_GPC_MFPL_PC2MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC2MFP Mask           */

#define SYS_GPC_MFPL_PC3MFP_Pos          (12)                                              /*!< SYS_T::GPC_MFPL: PC3MFP Position       */
#define SYS_GPC_MFPL_PC3MFP_Msk          (0xful << SYS_GPC_MFPL_PC3MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC3MFP Mask           */

#define SYS_GPC_MFPL_PC4MFP_Pos          (16)                                              /*!< SYS_T::GPC_MFPL: PC4MFP Position       */
#define SYS_GPC_MFPL_PC4MFP_Msk          (0xful << SYS_GPC_MFPL_PC4MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC4MFP Mask           */

#define SYS_GPC_MFPL_PC5MFP_Pos          (20)                                              /*!< SYS_T::GPC_MFPL: PC5MFP Position       */
#define SYS_GPC_MFPL_PC5MFP_Msk          (0xful << SYS_GPC_MFPL_PC5MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC5MFP Mask           */

#define SYS_GPC_MFPL_PC6MFP_Pos          (24)                                              /*!< SYS_T::GPC_MFPL: PC6MFP Position       */
#define SYS_GPC_MFPL_PC6MFP_Msk          (0xful << SYS_GPC_MFPL_PC6MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC6MFP Mask           */

#define SYS_GPC_MFPL_PC7MFP_Pos          (28)                                              /*!< SYS_T::GPC_MFPL: PC7MFP Position       */
#define SYS_GPC_MFPL_PC7MFP_Msk          (0xful << SYS_GPC_MFPL_PC7MFP_Pos)                /*!< SYS_T::GPC_MFPL: PC7MFP Mask           */

#define SYS_GPC_MFPH_PC14MFP_Pos         (24)                                              /*!< SYS_T::GPC_MFPH: PC14MFP Position      */
#define SYS_GPC_MFPH_PC14MFP_Msk         (0xful << SYS_GPC_MFPH_PC14MFP_Pos)               /*!< SYS_T::GPC_MFPH: PC14MFP Mask          */

#define SYS_GPD_MFPL_PD0MFP_Pos          (0)                                               /*!< SYS_T::GPD_MFPL: PD0MFP Position       */
#define SYS_GPD_MFPL_PD0MFP_Msk          (0xful << SYS_GPD_MFPL_PD0MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD0MFP Mask           */

#define SYS_GPD_MFPL_PD1MFP_Pos          (4)                                               /*!< SYS_T::GPD_MFPL: PD1MFP Position       */
#define SYS_GPD_MFPL_PD1MFP_Msk          (0xful << SYS_GPD_MFPL_PD1MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD1MFP Mask           */

#define SYS_GPD_MFPL_PD2MFP_Pos          (8)                                               /*!< SYS_T::GPD_MFPL: PD2MFP Position       */
#define SYS_GPD_MFPL_PD2MFP_Msk          (0xful << SYS_GPD_MFPL_PD2MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD2MFP Mask           */

#define SYS_GPD_MFPL_PD3MFP_Pos          (12)                                              /*!< SYS_T::GPD_MFPL: PD3MFP Position       */
#define SYS_GPD_MFPL_PD3MFP_Msk          (0xful << SYS_GPD_MFPL_PD3MFP_Pos)                /*!< SYS_T::GPD_MFPL: PD3MFP Mask           */

#define SYS_GPD_MFPH_PD15MFP_Pos         (28)                                              /*!< SYS_T::GPD_MFPH: PD15MFP Position      */
#define SYS_GPD_MFPH_PD15MFP_Msk         (0xful << SYS_GPD_MFPH_PD15MFP_Pos)               /*!< SYS_T::GPD_MFPH: PD15MFP Mask          */

#define SYS_GPF_MFPL_PF0MFP_Pos          (0)                                               /*!< SYS_T::GPF_MFPL: PF0MFP Position       */
#define SYS_GPF_MFPL_PF0MFP_Msk          (0xful << SYS_GPF_MFPL_PF0MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF0MFP Mask           */

#define SYS_GPF_MFPL_PF1MFP_Pos          (4)                                               /*!< SYS_T::GPF_MFPL: PF1MFP Position       */
#define SYS_GPF_MFPL_PF1MFP_Msk          (0xful << SYS_GPF_MFPL_PF1MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF1MFP Mask           */

#define SYS_GPF_MFPL_PF2MFP_Pos          (8)                                               /*!< SYS_T::GPF_MFPL: PF2MFP Position       */
#define SYS_GPF_MFPL_PF2MFP_Msk          (0xful << SYS_GPF_MFPL_PF2MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF2MFP Mask           */

#define SYS_GPF_MFPL_PF3MFP_Pos          (12)                                              /*!< SYS_T::GPF_MFPL: PF3MFP Position       */
#define SYS_GPF_MFPL_PF3MFP_Msk          (0xful << SYS_GPF_MFPL_PF3MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF3MFP Mask           */

#define SYS_GPF_MFPL_PF4MFP_Pos          (16)                                              /*!< SYS_T::GPF_MFPL: PF4MFP Position       */
#define SYS_GPF_MFPL_PF4MFP_Msk          (0xful << SYS_GPF_MFPL_PF4MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF4MFP Mask           */

#define SYS_GPF_MFPL_PF5MFP_Pos          (20)                                              /*!< SYS_T::GPF_MFPL: PF5MFP Position       */
#define SYS_GPF_MFPL_PF5MFP_Msk          (0xful << SYS_GPF_MFPL_PF5MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF5MFP Mask           */

#define SYS_GPF_MFPL_PF6MFP_Pos          (24)                                              /*!< SYS_T::GPF_MFPL: PF6MFP Position       */
#define SYS_GPF_MFPL_PF6MFP_Msk          (0xful << SYS_GPF_MFPL_PF6MFP_Pos)                /*!< SYS_T::GPF_MFPL: PF6MFP Mask           */

#define SYS_IRCTCTL_FREQSEL_Pos          (0)                                               /*!< SYS_T::IRCTCTL: FREQSEL Position       */
#define SYS_IRCTCTL_FREQSEL_Msk          (0x3ul << SYS_IRCTCTL_FREQSEL_Pos)                /*!< SYS_T::IRCTCTL: FREQSEL Mask           */

#define SYS_IRCTCTL_LOOPSEL_Pos          (4)                                               /*!< SYS_T::IRCTCT0: LOOPSEL Position       */
#define SYS_IRCTCTL_LOOPSEL_Msk          (0x3ul << SYS_IRCTCTL_LOOPSEL_Pos)                /*!< SYS_T::IRCTCTL: LOOPSEL Mask           */

#define SYS_IRCTCTL_RETRYCNT_Pos         (6)                                               /*!< SYS_T::IRCTCTL: RETRYCNT Position      */
#define SYS_IRCTCTL_RETRYCNT_Msk         (0x3ul << SYS_IRCTCTL_RETRYCNT_Pos)               /*!< SYS_T::IRCTCTL: RETRYCNT Mask          */

#define SYS_IRCTCTL_CESTOPEN_Pos         (8)                                               /*!< SYS_T::IRCTCTL: CESTOPEN Position      */
#define SYS_IRCTCTL_CESTOPEN_Msk         (0x1ul << SYS_IRCTCTL_CESTOPEN_Pos)               /*!< SYS_T::IRCTCTL: CESTOPEN Mask          */

#define SYS_IRCTCTL_BOUNDEN_Pos          (9)                                               /*!< SYS_T::IRCTCTL: BOUNDEN Position       */
#define SYS_IRCTCTL_BOUNDEN_Msk          (0x1ul << SYS_IRCTCTL_BOUNDEN_Pos)                /*!< SYS_T::IRCTCTL: BOUNDEN Mask           */

#define SYS_IRCTCTL_REFCKSEL_Pos         (10)                                              /*!< SYS_T::IRCTCTL: REFCKSEL Position      */
#define SYS_IRCTCTL_REFCKSEL_Msk         (0x1ul << SYS_IRCTCTL_REFCKSEL_Pos)               /*!< SYS_T::IRCTCTL: REFCKSEL Mask          */

#define SYS_IRCTCTL_BOUNDARY_Pos         (16)                                              /*!< SYS_T::IRCTCTL: BOUNDARY Position      */
#define SYS_IRCTCTL_BOUNDARY_Msk         (0x1ful << SYS_IRCTCTL_BOUNDARY_Pos)              /*!< SYS_T::IRCTCTL: BOUNDARY Mask          */

#define SYS_IRCTIEN_TFAILIEN_Pos         (1)                                               /*!< SYS_T::IRCTIEN: TFAILIEN Position      */
#define SYS_IRCTIEN_TFAILIEN_Msk         (0x1ul << SYS_IRCTIEN_TFAILIEN_Pos)               /*!< SYS_T::IRCTIEN: TFAILIEN Mask          */

#define SYS_IRCTIEN_CLKEIEN_Pos          (2)                                               /*!< SYS_T::IRCTIEN: CLKEIEN Position       */
#define SYS_IRCTIEN_CLKEIEN_Msk          (0x1ul << SYS_IRCTIEN_CLKEIEN_Pos)                /*!< SYS_T::IRCTIEN: CLKEIEN Mask           */

#define SYS_IRCTISTS_FREQLOCK_Pos        (0)                                               /*!< SYS_T::IRCTISTS: FREQLOCK Position     */
#define SYS_IRCTISTS_FREQLOCK_Msk        (0x1ul << SYS_IRCTISTS_FREQLOCK_Pos)              /*!< SYS_T::IRCTISTS: FREQLOCK Mask         */

#define SYS_IRCTISTS_TFAILIF_Pos         (1)                                               /*!< SYS_T::IRCTISTS: TFAILIF Position      */
#define SYS_IRCTISTS_TFAILIF_Msk         (0x1ul << SYS_IRCTISTS_TFAILIF_Pos)               /*!< SYS_T::IRCTISTS: TFAILIF Mask          */

#define SYS_IRCTISTS_CLKERRIF_Pos        (2)                                               /*!< SYS_T::IRCTISTS: CLKERRIF Position     */
#define SYS_IRCTISTS_CLKERRIF_Msk        (0x1ul << SYS_IRCTISTS_CLKERRIF_Pos)              /*!< SYS_T::IRCTISTS: CLKERRIF Mask         */

#define SYS_MODCTL_MODEN_Pos             (0)                                               /*!< SYS_T::MODCTL: MODEN Position          */
#define SYS_MODCTL_MODEN_Msk             (0x1ul << SYS_MODCTL_MODEN_Pos)                   /*!< SYS_T::MODCTL: MODEN Mask              */

#define SYS_MODCTL_MODH_Pos              (1)                                               /*!< SYS_T::MODCTL: MODH Position           */
#define SYS_MODCTL_MODH_Msk              (0x1ul << SYS_MODCTL_MODH_Pos)                    /*!< SYS_T::MODCTL: MODH Mask               */

#define SYS_MODCTL_MODPWMSEL_Pos         (4)                                               /*!< SYS_T::MODCTL: MODPWMSEL Position      */
#define SYS_MODCTL_MODPWMSEL_Msk         (0x7ul << SYS_MODCTL_MODPWMSEL_Pos)               /*!< SYS_T::MODCTL: MODPWMSEL Mask          */

#define SYS_TSCTL_TSEN_Pos               (0)                                               /*!< SYS_T::TSCTL: TSEN Position            */
#define SYS_TSCTL_TSEN_Msk               (0x1ul << SYS_TSCTL_TSEN_Pos)                     /*!< SYS_T::TSCTL: TSEN Mask                */

#define SYS_TSCTL_TSBGEN_Pos             (1)                                               /*!< SYS_T::TSCTL: TSBGEN Position          */
#define SYS_TSCTL_TSBGEN_Msk             (0x1ul << SYS_TSCTL_TSBGEN_Pos)                   /*!< SYS_T::TSCTL: TSBGEN Mask              */

#define SYS_TSCTL_TSST_Pos               (2)                                               /*!< SYS_T::TSCTL: TSST Position            */
#define SYS_TSCTL_TSST_Msk               (0x1ul << SYS_TSCTL_TSST_Pos)                     /*!< SYS_T::TSCTL: TSST Mask                */

#define SYS_TSCTL_TSIEN_Pos              (3)                                               /*!< SYS_T::TSCTL: TSIEN Position           */
#define SYS_TSCTL_TSIEN_Msk              (0x1ul << SYS_TSCTL_TSIEN_Pos)                    /*!< SYS_T::TSCTL: TSIEN Mask               */

#define SYS_TSCTL_TSIF_Pos               (16)                                              /*!< SYS_T::TSCTL: TSIF Position            */
#define SYS_TSCTL_TSIF_Msk               (0x1ul << SYS_TSCTL_TSIF_Pos)                     /*!< SYS_T::TSCTL: TSIF Mask                */

#define SYS_TSDATA_TSEOC_Pos             (0)                                               /*!< SYS_T::TSDATA: TSEOC Position          */
#define SYS_TSDATA_TSEOC_Msk             (0x1ul << SYS_TSDATA_TSEOC_Pos)                   /*!< SYS_T::TSDATA: TSEOC Mask              */

#define SYS_TSDATA_TSDATA_Pos            (16)                                              /*!< SYS_T::TSDATA: TSDATA Position         */
#define SYS_TSDATA_TSDATA_Msk            (0xffful << SYS_TSDATA_TSDATA_Pos)                /*!< SYS_T::TSDATA: TSDATA Mask             */

#define SYS_SPDHCTL_HSADIS_Pos           (30)                                              /*!< SYS_T::SPDHCTL: HSADIS Position        */
#define SYS_SPDHCTL_HSADIS_Msk           (0x1ul << SYS_SPDHCTL_HSADIS_Pos)                 /*!< SYS_T::SPDHCTL: HSADIS Mask            */

#define SYS_SPDHCTL_SPDHEN_Pos           (31)                                              /*!< SYS_T::SPDHCTL: SPDHEN Position        */
#define SYS_SPDHCTL_SPDHEN_Msk           (0x1ul << SYS_SPDHCTL_SPDHEN_Pos)                 /*!< SYS_T::SPDHCTL: SPDHEN Mask            */

#define SYS_SPIMUX_MUXSWEN_Pos           (0)                                               /*!< SYS_T::SPIMUX: MUXSWEN Position        */
#define SYS_SPIMUX_MUXSWEN_Msk           (0x1ul << SYS_SPIMUX_MUXSWEN_Pos)                 /*!< SYS_T::SPIMUX: MUXSWEN Mask            */

#define SYS_SPIMUX_MUXSWSEL_Pos          (1)                                               /*!< SYS_T::SPIMUX: MUXSWSEL Position       */
#define SYS_SPIMUX_MUXSWSEL_Msk          (0x1ul << SYS_SPIMUX_MUXSWSEL_Pos)                /*!< SYS_T::SPIMUX: MUXSWSEL Mask           */

/**@}*/ /* SYS_CONST */

typedef struct
{

    /**
     * @var SYS_INT_T::NMIEN
     * Offset: 0x00  NMI Source Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD NMI Source Enable (Write Protect)
     * |        |          |0 = BOD NMI source Disabled.
     * |        |          |1 = BOD NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |IRCINT    |IRC TRIM NMI Source Enable (Write Protect)
     * |        |          |0 = IRC TRIM NMI source Disabled.
     * |        |          |1 = IRC TRIM NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up NMI Source Enable (Write Protect)
     * |        |          |0 = Power-down mode wake-up NMI source Disabled.
     * |        |          |1 = Power-down mode wake-up NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |SRAMPERR  |SRAM Parity Check Error NMI Source Enable (Write Protect)
     * |        |          |0 = SRAM parity check error NMI source Disabled.
     * |        |          |1 = SRAM parity check error NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |CLKFAIL   |Clock Fail Detected NMI Source Enable (Write Protect)
     * |        |          |0 = Clock fail detected interrupt NMI source Disabled.
     * |        |          |1 = Clock fail detected interrupt NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |EINT0     |External Interrupt From INT0 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT0 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT0 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[9]     |EINT1     |External Interrupt From INT1 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT1 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT1 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10]    |EINT2     |External Interrupt From INT2 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT2 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT2 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[11]    |EINT3     |External Interrupt From INT3 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT3 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT3 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |EINT4     |External Interrupt From INT4 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT4 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT4 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |EINT5     |External Interrupt From INT5 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from INT5 pin NMI source Disabled.
     * |        |          |1 = External interrupt from INT5 pin NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[14]    |UART0INT  |UART0 NMI Source Enable (Write Protect)
     * |        |          |0 = UART0 NMI source Disabled.
     * |        |          |1 = UART0 NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[15]    |UART1INT  |UART1 NMI Source Enable (Write Protect)
     * |        |          |0 = UART1 NMI source Disabled.
     * |        |          |1 = UART1 NMI source Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_INT_T::NMISTS
     * Offset: 0x04  NMI source interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD Interrupt Flag (Read Only)
     * |        |          |0 = BOD interrupt is de-asserted.
     * |        |          |1 = BOD interrupt is asserted.
     * |[1]     |IRCINT    |IRC TRIM Interrupt Flag (Read Only)
     * |        |          |0 = HIRC TRIM interrupt is de-asserted.
     * |        |          |1 = HIRC TRIM interrupt is asserted.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up Interrupt Flag (Read Only)
     * |        |          |0 = Power-down mode wake-up interrupt is de-asserted.
     * |        |          |1 = Power-down mode wake-up interrupt is asserted.
     * |[4]     |CLKFAIL   |Clock Fail Detected Interrupt Flag (Read Only)
     * |        |          |0 = Clock fail detected interrupt is de-asserted.
     * |        |          |1 = Clock fail detected interrupt is asserted.
     * |[8]     |EINT0     |External Interrupt From INT0 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT0 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT0 pin interrupt is asserted.
     * |[9]     |EINT1     |External Interrupt From INT1 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT1 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT1 pin interrupt is asserted.
     * |[10]    |EINT2     |External Interrupt From INT2 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT2 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT2 pin interrupt is asserted.
     * |[11]    |EINT3     |External Interrupt From INT3 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT3 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT3 pin interrupt is asserted.
     * |[12]    |EINT4     |External Interrupt From INT4 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT4 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT4 pin interrupt is asserted.
     * |[13]    |EINT5     |External Interrupt From INT5 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from INT5 pin interrupt is deasserted.
     * |        |          |1 = External Interrupt from INT5 pin interrupt is asserted.
     * |[14]    |UART0INT  |UART0 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is de-asserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * |[15]    |UART1INT  |UART1 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is de-asserted.
     * |        |          |1 = UART1 interrupt is asserted.
     */

    __IO  uint32_t NMIEN;          /* Offset: 0x00  NMI Source Interrupt Enable Register                               */
    __I   uint32_t NMISTS;         /* Offset: 0x04  NMI source interrupt Status Register                               */

} SYS_INT_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
    @{ 
*/

#define SYS_NMIEN_BODOUT_Pos             (0)                                               /*!< SYS_INT_T::NMIEN: BODOUT Position         */
#define SYS_NMIEN_BODOUT_Msk             (0x1ul << SYS_NMIEN_BODOUT_Pos )                  /*!< SYS_INT_T::NMIEN: BODOUT Mask             */

#define SYS_NMIEN_IRCINT_Pos             (1)                                               /*!< SYS_INT_T::NMIEN: IRCINT Position         */
#define SYS_NMIEN_IRCINT_Msk             (0x1ul << SYS_NMIEN_IRCINT_Pos )                  /*!< SYS_INT_T::NMIEN: IRCINT Mask             */

#define SYS_NMIEN_PWRWUINT_Pos           (2)                                               /*!< SYS_INT_T::NMIEN: PWRWUINT Position       */
#define SYS_NMIEN_PWRWUINT_Msk           (0x1ul << SYS_NMIEN_PWRWUINT_Pos )                /*!< SYS_INT_T::NMIEN: PWRWUINT Mask           */

#define SYS_NMIEN_CLKFAIL_Pos            (4)                                               /*!< SYS_INT_T::NMIEN: CLKFAIL Position        */
#define SYS_NMIEN_CLKFAIL_Msk            (0x1ul << SYS_NMIEN_CLKFAIL_Pos )                 /*!< SYS_INT_T::NMIEN: CLKFAIL Mask            */

#define SYS_NMIEN_EINT0_Pos              (8)                                               /*!< SYS_INT_T::NMIEN: EINT0 Position          */
#define SYS_NMIEN_EINT0_Msk              (0x1ul << SYS_NMIEN_EINT0_Pos )                   /*!< SYS_INT_T::NMIEN: EINT0 Mask              */

#define SYS_NMIEN_EINT1_Pos              (9)                                               /*!< SYS_INT_T::NMIEN: EINT1 Position          */
#define SYS_NMIEN_EINT1_Msk              (0x1ul << SYS_NMIEN_EINT1_Pos )                   /*!< SYS_INT_T::NMIEN: EINT1 Mask              */

#define SYS_NMIEN_EINT2_Pos              (10)                                              /*!< SYS_INT_T::NMIEN: EINT2 Position          */
#define SYS_NMIEN_EINT2_Msk              (0x1ul << SYS_NMIEN_EINT2_Pos )                   /*!< SYS_INT_T::NMIEN: EINT2 Mask              */

#define SYS_NMIEN_EINT3_Pos              (11)                                              /*!< SYS_INT_T::NMIEN: EINT3 Position          */
#define SYS_NMIEN_EINT3_Msk              (0x1ul << SYS_NMIEN_EINT3_Pos )                   /*!< SYS_INT_T::NMIEN: EINT3 Mask              */

#define SYS_NMIEN_EINT4_Pos              (12)                                              /*!< SYS_INT_T::NMIEN: EINT4 Position          */
#define SYS_NMIEN_EINT4_Msk              (0x1ul << SYS_NMIEN_EINT4_Pos )                   /*!< SYS_INT_T::NMIEN: EINT4 Mask              */

#define SYS_NMIEN_EINT5_Pos              (13)                                              /*!< SYS_INT_T::NMIEN: EINT5 Position          */
#define SYS_NMIEN_EINT5_Msk              (0x1ul << SYS_NMIEN_EINT5_Pos )                   /*!< SYS_INT_T::NMIEN: EINT5 Mask              */

#define SYS_NMIEN_UART0INT_Pos           (14)                                              /*!< SYS_INT_T::NMIEN: UART0INT Position       */
#define SYS_NMIEN_UART0INT_Msk           (0x1ul << SYS_NMIEN_UART0INT_Pos )                /*!< SYS_INT_T::NMIEN: UART0INT Mask           */

#define SYS_NMIEN_UART1INT_Pos           (15)                                              /*!< SYS_INT_T::NMIEN: UART1INT Position       */
#define SYS_NMIEN_UART1INT_Msk           (0x1ul << SYS_NMIEN_UART1INT_Pos )                /*!< SYS_INT_T::NMIEN: UART1INT Mask           */

#define SYS_NMISTS_BODOUT_Pos            (0)                                               /*!< SYS_INT_T::NMISTS: BODOUT Position        */
#define SYS_NMISTS_BODOUT_Msk            (0x1ul << SYS_NMISTS_BODOUT_Pos )                 /*!< SYS_INT_T::NMISTS: BODOUT Mask            */

#define SYS_NMISTS_IRCINT_Pos            (1)                                               /*!< SYS_INT_T::NMISTS: IRCINT Position        */
#define SYS_NMISTS_IRCINT_Msk            (0x1ul << SYS_NMISTS_IRCINT_Pos )                 /*!< SYS_INT_T::NMISTS: IRCINT Mask            */

#define SYS_NMISTS_PWRWUINT_Pos          (2)                                               /*!< SYS_INT_T::NMISTS: PWRWUINT Position      */
#define SYS_NMISTS_PWRWUINT_Msk          (0x1ul << SYS_NMISTS_PWRWUINT_Pos )               /*!< SYS_INT_T::NMISTS: PWRWUINT Mask          */

#define SYS_NMISTS_CLKFAIL_Pos           (4)                                               /*!< SYS_INT_T::NMISTS: CLKFAIL Position       */
#define SYS_NMISTS_CLKFAIL_Msk           (0x1ul << SYS_NMISTS_CLKFAIL_Pos )                /*!< SYS_INT_T::NMISTS: CLKFAIL Mask           */

#define SYS_NMISTS_EINT0_Pos             (8)                                               /*!< SYS_INT_T::NMISTS: EINT0 Position         */
#define SYS_NMISTS_EINT0_Msk             (0x1ul << SYS_NMISTS_EINT0_Pos )                  /*!< SYS_INT_T::NMISTS: EINT0 Mask             */

#define SYS_NMISTS_EINT1_Pos             (9)                                               /*!< SYS_INT_T::NMISTS: EINT1 Position         */
#define SYS_NMISTS_EINT1_Msk             (0x1ul << SYS_NMISTS_EINT1_Pos )                  /*!< SYS_INT_T::NMISTS: EINT1 Mask             */

#define SYS_NMISTS_EINT2_Pos             (10)                                              /*!< SYS_INT_T::NMISTS: EINT2 Position         */
#define SYS_NMISTS_EINT2_Msk             (0x1ul << SYS_NMISTS_EINT2_Pos )                  /*!< SYS_INT_T::NMISTS: EINT2 Mask             */

#define SYS_NMISTS_EINT3_Pos             (11)                                              /*!< SYS_INT_T::NMISTS: EINT3 Position         */
#define SYS_NMISTS_EINT3_Msk             (0x1ul << SYS_NMISTS_EINT3_Pos )                  /*!< SYS_INT_T::NMISTS: EINT3 Mask             */

#define SYS_NMISTS_EINT4_Pos             (12)                                              /*!< SYS_INT_T::NMISTS: EINT4 Position         */
#define SYS_NMISTS_EINT4_Msk             (0x1ul << SYS_NMISTS_EINT4_Pos )                  /*!< SYS_INT_T::NMISTS: EINT4 Mask             */

#define SYS_NMISTS_EINT5_Pos             (13)                                              /*!< SYS_INT_T::NMISTS: EINT5 Position         */
#define SYS_NMISTS_EINT5_Msk             (0x1ul << SYS_NMISTS_EINT5_Pos )                  /*!< SYS_INT_T::NMISTS: EINT5 Mask             */

#define SYS_NMISTS_UART0INT_Pos          (14)                                              /*!< SYS_INT_T::NMISTS: UART0INT Position      */
#define SYS_NMISTS_UART0INT_Msk          (0x1ul << SYS_NMISTS_UART0INT_Pos )               /*!< SYS_INT_T::NMISTS: UART0INT Mask          */

#define SYS_NMISTS_UART1INT_Pos          (15)                                              /*!< SYS_INT_T::NMISTS: UART1INT Position      */
#define SYS_NMISTS_UART1INT_Msk          (0x1ul << SYS_NMISTS_UART1INT_Pos )               /*!< SYS_INT_T::NMISTS: UART1INT Mask          */

/**@}*/ /* INT_CONST */
/**@}*/ /* end of SYS register group */

/**@}*/ /* end of REGISTER group */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

#endif /* __SYS_REG_H__ */
