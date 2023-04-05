/**************************************************************************//**
 * @file     dfmc.h
 * @version  V1.00
 * @brief    M471 Series Data Flash Memory Controller Driver Header File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DFMC_H__
#define __DFMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DFMC_Driver DFMC Driver
  @{
*/


/** @addtogroup DFMC_EXPORTED_CONSTANTS DFMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_DFLASH_BASE         0x00400000UL    /*!< Data flash base address      \hideinitializer */
#define DFMC_DFLASH_END          0x00408000UL    /*!< Data flash  end address      \hideinitializer */
#define DFMC_DFLASH_SIZE         0x00008000UL    /*!< Data flash  end address      \hideinitializer */
#define DFMC_DFLASH_PAGE_SIZE    0x100UL         /*!< Flash Page Size (256 bytes)  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_ISPCMD_READ         0x00UL          /*!< ISP Command: Read flash word          \hideinitializer */
#define DFMC_ISPCMD_READ_ALL1    0x08UL          /*!< ISP Command: Read all-one result      \hideinitializer */
#define DFMC_ISPCMD_READ_CID     0x0BUL          /*!< ISP Command: Read Company ID          \hideinitializer */
#define DFMC_ISPCMD_READ_DID     0x0CUL          /*!< ISP Command: Read Device ID           \hideinitializer */
#define DFMC_ISPCMD_READ_CKS     0x0DUL          /*!< ISP Command: Read checksum            \hideinitializer */
#define DFMC_ISPCMD_PROGRAM      0x21UL          /*!< ISP Command: Write flash word         \hideinitializer */
#define DFMC_ISPCMD_PAGE_ERASE   0x22UL          /*!< ISP Command: Page Erase Flash         \hideinitializer */
#define DFMC_ISPCMD_RUN_ALL1     0x28UL          /*!< ISP Command: Run all-one verification \hideinitializer */
#define DFMC_ISPCMD_RUN_CKS      0x2DUL          /*!< ISP Command: Run checksum calculation \hideinitializer */

#define DREAD_ALLONE_YES         0xA11FFFFFUL    /*!< Check-all-one result is all one.      \hideinitializer */
#define DREAD_ALLONE_NOT         0xA1100000UL    /*!< Check-all-one result is not all one.  \hideinitializer */
#define DREAD_ALLONE_CMD_FAIL    0xFFFFFFFFUL    /*!< Check-all-one command failed.         \hideinitializer */


/*@}*/ /* end of group DFMC_EXPORTED_CONSTANTS */


/** @addtogroup DFMC_EXPORTED_MACROS DFMC Exported Macros
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_DISABLE_ISP()           (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk)               /*!< Disable ISP function           \hideinitializer */
#define DFMC_ENABLE_ISP()            (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk)               /*!< Enable ISP function            \hideinitializer */
#define DFMC_GET_FAIL_FLAG()         ((DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk) ? 1UL : 0UL)   /*!< Get ISP fail flag              \hideinitializer */
#define DFMC_CLR_FAIL_FLAG()         (DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk)                /*!< Clear ISP fail flag            \hideinitializer */
#define DFMC_ENABLE_UPDATE()         (DFMC->ISPCTL |=  DFMC_ISPCTL_DATAEN_Msk)             /*!< Enable Update                  \hideinitializer */
#define DFMC_DISABLE_UPDATE()        (DFMC->ISPCTL &= ~DFMC_ISPCTL_DATAEN_Msk)             /*!< Disable Update                 \hideinitializer */
#define DFMC_ENABLE_ISP_INT()        (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPIFEN_Msk)             /*!< Enable ISP interrupt           \hideinitializer */
#define DFMC_DISABLE_ISP_INT()       (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPIFEN_Msk)             /*!< Disable ISP interrupt          \hideinitializer */
#define DFMC_GET_ISP_INT_FLAG()      ((DFMC->ISPSTS & DFMC_ISPSTS_ISPIF_Msk) ? 1UL : 0UL)   /*!< Get ISP interrupt flag Status  \hideinitializer */
#define DFMC_CLEAR_ISP_INT_FLAG()    (DFMC->ISPSTS = DFMC_ISPSTS_ISPIF_Msk)               /*!< Clear ISP interrupt flag       \hideinitializer */

/*@}*/ /* end of group DFMC_EXPORTED_MACROS */


/** @addtogroup DFMC_EXPORTED_FUNCTIONS DFMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
__STATIC_INLINE uint32_t DFMC_ReadCID(void);
__STATIC_INLINE uint32_t DFMC_ReadPID(void);

/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit)
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  */
__STATIC_INLINE uint32_t DFMC_ReadCID(void)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    DFMC->ISPADDR = 0x0u;                          /* Must keep 0x0 when read CID */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                             /* To make sure ISP/CPU be Synchronized */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    return DFMC->ISPDAT;
}

/**
  * @brief    Read product ID
  * @param    None
  * @return   The product ID (32-bit)
  * @details  This function is used to read product ID.
  */
__STATIC_INLINE uint32_t DFMC_ReadPID(void)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    DFMC->ISPADDR = 0x04u;                        /* Must keep 0x4 when read PID */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                          /* To make sure ISP/CPU be Synchronized */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    return DFMC->ISPDAT;
}

/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

extern void     DFMC_Close(void);
extern int32_t  DFMC_Erase(uint32_t u32PageAddr);
extern void     DFMC_Open(void);
extern uint32_t DFMC_Read(uint32_t u32Addr);
extern void     DFMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern int32_t DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);

/*@}*/ /* end of group DFMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DFMC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif   /* __DFMC_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
