/**************************************************************************//**
 * @file     wwdt.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/12/07 5:41p $
 * @brief    NANO103 series WWDT driver header file
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __WWDT_H__
#define __WWDT_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_WWDT_Driver WWDT Driver
  @{
*/

/** @addtogroup NANO103_WWDT_EXPORTED_CONSTANTS WWDT Exported Constants
  @{
*/
#define WWDT_PRESCALER_1          (0UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 1    */
#define WWDT_PRESCALER_2          (1UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 2    */
#define WWDT_PRESCALER_4          (2UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 4    */
#define WWDT_PRESCALER_8          (3UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 8    */
#define WWDT_PRESCALER_16         (4UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 16   */
#define WWDT_PRESCALER_32         (5UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 32   */
#define WWDT_PRESCALER_64         (6UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 64   */
#define WWDT_PRESCALER_128        (7UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 128  */
#define WWDT_PRESCALER_192        (8UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 192  */
#define WWDT_PRESCALER_256        (9UL << WWDT_CTL_PERIODSEL_Pos)   /*!< WWDT setting prescaler to 256  */
#define WWDT_PRESCALER_384        (0xAUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 384  */
#define WWDT_PRESCALER_512        (0xBUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 512  */
#define WWDT_PRESCALER_768        (0xCUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 768  */
#define WWDT_PRESCALER_1024       (0xDUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 1024 */
#define WWDT_PRESCALER_1536       (0xEUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 1536 */
#define WWDT_PRESCALER_2048       (0xFUL << WWDT_CTL_PERIODSEL_Pos) /*!< WWDT setting prescaler to 2048 */

#define WWDT_RELOAD_WORD          (0x00005AA5)                     /*!< Fill this value to RLD register to reload WWDT counter */
/*@}*/ /* end of group NANO103_WWDT_EXPORTED_CONSTANTS */


/** @addtogroup NANO103_WWDT_EXPORTED_FUNCTIONS WWDT Exported Functions
  @{
*/

/**
  * @brief This macro clear WWDT time-out reset system flag.
  * @param None
  * @return None
  * \hideinitializer
  */
#define WWDT_CLEAR_RESET_FLAG()  (WWDT->STATUS = WWDT_STATUS_WWDTRF_Msk)

/**
  * @brief This macro clears WWDT compare match interrupt flag.
  * @param None
  * @return None
  * \hideinitializer
  */
#define WWDT_CLEAR_INT_FLAG() (WWDT->STATUS = WWDT_STATUS_WWDTIF_Msk)

/**
  * @brief This macro is use to get WWDT time-out reset system flag.
  * @return WWDT reset system or not
  * @retval 0 WWDT did not cause system reset
  * @retval 1 WWDT caused system reset
  * \hideinitializer
  */
#define WWDT_GET_RESET_FLAG() (WWDT->STATUS & WWDT_STATUS_WWDTRF_Msk ? 1 : 0)

/**
  * @brief This macro is used to indicate WWDT compare match interrupt flag.
  * @return WWDT compare match interrupt occurred or not
  * @retval 0 WWDT compare match interrupt did not occur
  * @retval 1 WWDT compare match interrupt occurred
  * \hideinitializer
  */
#define WWDT_GET_INT_FLAG() (WWDT->STATUS & WWDT_STATUS_WWDTIF_Msk ? 1 : 0)

/**
  * @brief This macro to reflects current WWDT counter value
  * @param None
  * @return Return current WWDT counter value
  * \hideinitializer
  */
#define WWDT_GET_COUNTER() (WWDT->CNT)

/**
  * @brief This macro is used to reload the WWDT counter value to 0x3F.
  * @param None
  * @return None
  * @details After WWDT enabled, application must reload WWDT counter while
  *          current counter is less than compare value and larger than 0,
  *          otherwise WWDT will cause system reset.
  * \hideinitializer
  */
#define WWDT_RELOAD_COUNTER() (WWDT->RLDCNT  = WWDT_RELOAD_WORD)


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);


/*@}*/ /* end of group NANO103_WWDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_WWDT_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__WWDT_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
