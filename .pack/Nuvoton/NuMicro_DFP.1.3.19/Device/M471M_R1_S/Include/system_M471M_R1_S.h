/******************************************************************************
 * @file     system_M471M_R1_S.h
 * @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Header File for M471M/R1/S
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SYSTEM_M471M_R1_S_H__
#define __SYSTEM_M471M_R1_S_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef DEBUG_PORT
# define DEBUG_PORT      UART0       /*!< Select Debug Port which is used for retarget.c to output debug message to UART */
#endif


/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#define __HSI       (12000000UL)    /*!< PLL default output is 72MHz */
#define __HXT       (12000000UL)    /*!< External Crystal Clock Frequency     */
#define __LXT       (32768UL)       /*!< External Crystal Clock Frequency 32.768KHz */
#define __HIRC      (22118400UL)    /*!< Internal 22M RC Oscillator Frequency */
#define __LIRC      (10000UL)       /*!< Internal 10K RC Oscillator Frequency */
#define __SYS_OSC_CLK     (    ___HSI)    /* Main oscillator frequency        */


#define __SYSTEM_CLOCK    (1*__HXT)

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;         /*!< Cycles per micro second              */
extern uint32_t PllClock;            /*!< PLL Output Clock Frequency           */


/**
 * Initialize the system
 *
 * @param  None
 * @return None
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit(void);

/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_M471M_R1_S_H__ */

