/**************************************************************************//**
 * @file     system_Nano1X2Series.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 13/12/16 4:27p $
 * @brief    Nano1X2 series system clock definition file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __SYSTEM_NANO1X2SERIES_H__
#define __SYSTEM_NANO1X2SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif


/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/

#define __HXT           (12000000UL)
#define __LXT         (32768UL)
#define __HIRC12M       (12000000UL)
#define __HIRC16M       (16000000UL)
#define __LIRC        (10000UL)
#define __HIRC          __HIRC12M
#define __HSI           (__HIRC12M)      /* Factory Default is internal 12MHz */


extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;            /*!< Cycles per micro second */

/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */

extern void SystemCoreClockUpdate (void);
extern uint32_t SysGet_PLLClockFreq(void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_NANO1X2SERIES_H__


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
