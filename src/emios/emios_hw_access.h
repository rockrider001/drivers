/*
 * Copyright 2017-2019 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file emios_hw_access.h
 */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro.
 * These macro will used in next release.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 4.9, Function-like macro.
 * The macro defines a bitmask used to access status flags.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are static data and it's used in local source files only.
 */

#ifndef EMIOS_HW_ACCESS_H
#define EMIOS_HW_ACCESS_H

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include "device_registers.h"


/* Max value of global prescaler is 0xFF */
#define EMIOS_GPRE_MAX_VAL           0xFFU

/* Max value of data register(24 bits used) */
#define EMIOS_DATA_REG_MAX_VAL       0xFFFFFFUL
/*Min value of data register*/
#define EMIOS_DATA_REG_MIN_VAL       0x00UL

#if defined(FEATURE_EMIOS_COUNTER_24_BITS)
/* Max counter value (eMIOS CNT register in MC mode - 24 bit) */
#define EMIOS_MC_MAX_CNT_VAL         0xFFFFFFUL
/* Max counter value (eMIOS CNT register in MCB mode - 24 bit) */
#define EMIOS_MCB_MAX_CNT_VAL        0xFFFFFFUL
/* Max counter value (eMIOS CNT register in OPWFMB mode - 24 bit) */
#define EMIOS_OPWFMB_MAX_CNT_VAL     0xFFFFFFUL
/* Max counter value (eMIOS CNT register in OPWMCB mode - 24 bit) */
#define EMIOS_OPWMCB_MAX_CNT_VAL     0xFFFFFFUL

#else
/* Max counter value (eMIOS CNT register in MC mode - 16 bit) */
#define EMIOS_MC_MAX_CNT_VAL         0xFFFFUL
/* Max counter value (eMIOS CNT register in MCB mode - 16 bit) */
#define EMIOS_MCB_MAX_CNT_VAL        0xFFFFUL
/* Max counter value (eMIOS CNT register in OPWFMB mode - 16 bit) */
#define EMIOS_OPWFMB_MAX_CNT_VAL     0xFFFFUL
/* Max counter value (eMIOS CNT register in OPWMCB mode - 16 bit) */
#define EMIOS_OPWMCB_MAX_CNT_VAL     0xFFFFUL
#endif /* FEATURE_EMIOS_COUNTER_24_BITS */
/* Min counter value (eMIOS CNT register in MC mode) */
#define EMIOS_MC_MIN_CNT_VAL         0x00UL
/* Min counter value (eMIOS CNT register in MCB mode) */
#define EMIOS_MCB_MIN_CNT_VAL        0x01UL
/* Min counter value (eMIOS CNT register in OPWFMB mode) */
#define EMIOS_OPWFMB_MIN_CNT_VAL     0x01UL
/* Min counter value (eMIOS CNT register in OPWMCB mode) */
#define EMIOS_OPWMCB_MIN_CNT_VAL     0x00UL

/*-------------define MASK---------------------*/

/*!< Modulus Counter Buffered MASK */
#define EMIOS_FILTER_MCB             0xFCU
/*!< Modulus Counter Buffered (Up counter) MASK */
#define EMIOS_MASK_MCB_UP            0x50U
/*!< Modulus Counter Buffered (Up/down counter) MASK */
#define EMIOS_MASK_MCB_UPDOWN        0x54U
/*!< Modulus Counter MASK */
#define EMIOS_FILTER_MC              0xFCU
/*!< Modulus Counter (Up counter) MASK */
#define EMIOS_MASK_MC_UP             0x10U
/*!< Modulus Counter (Up/down counter) MASK */
#define EMIOS_MASK_MC_UPDOWN         0x14U
/*!< Center Aligned Output Pulse Width Modulation Buffered */
#define EMIOS_MASK_OPWMCB            0x5CU
/*!< Center Aligned Output Pulse Width Modulation Buffered Filter mask */
#define EMIOS_FILTER_OPWMCB          0xFCU
/* define boundaries */
#define EMIOS_NUMBER_GROUP_MAX       eMIOS_INSTANCE_COUNT
#if ((FEATURE_EMIOS_BUS_B_SELECT == 0U)&&(FEATURE_EMIOS_BUS_C_SELECT == 1U)&&(FEATURE_EMIOS_BUS_D_SELECT == 1U)&&(FEATURE_EMIOS_BUS_E_SELECT == 0U))
#define EMIOS_NUMBER_CHANNEL_MAX     (eMIOS_UC_COUNT + 8U)
#else
#define EMIOS_NUMBER_CHANNEL_MAX     eMIOS_UC_COUNT
#endif
#define EMIOS_REGISTER_F31_F0_MASK   0xFFFFFFFFu
#define EMIOS_REGISTER_F31_F0_SHIFT  0u

#define EMIOS_GET_REG(reg, mask, shift) ((reg & (uint32_t)(mask)) >> shift)

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern eMIOS_Type* const eMIOS[EMIOS_NUMBER_GROUP_MAX];

/*---------------------MCR register access---------------------------*/
/* Server time slot bits set/get */
#define eMIOS_MCR_SET_SRV(emiosGroup, val)   {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_SRV_MASK, eMIOS_MCR_SRV(val));}
#define eMIOS_MCR_GET_SRV(emiosGroup)         EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_SRV_MASK, eMIOS_MCR_SRV_SHIFT)
/* Global prescaler value set/get */
#define eMIOS_MCR_SET_GPRE(emiosGroup, val)    {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_GPRE_MASK, eMIOS_MCR_GPRE(val));}
#define eMIOS_MCR_GET_GPRE(emiosGroup)          EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_GPRE_MASK, eMIOS_MCR_GPRE_SHIFT)
/* Global prescaler enable set/get */
#define eMIOS_MCR_SET_GPREN(emiosGroup, val)   {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_GPREN_MASK, eMIOS_MCR_GPREN(val));}
#define eMIOS_MCR_GET_GPREN(emiosGroup)         EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_GPREN_MASK, eMIOS_MCR_GPREN_SHIFT)
/* External Time Base set/get */
#define eMIOS_MCR_SET_ETB(emiosGroup, val)     {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_ETB_MASK, eMIOS_MCR_ETB(val));}
#define eMIOS_MCR_GET_ETB(emiosGroup)           EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_ETB_MASK, eMIOS_MCR_ETB_SHIFT)
/* Global Time Base Enable set/get */
#define eMIOS_MCR_SET_GTBE(emiosGroup, val)    {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_GTBE_MASK, eMIOS_MCR_GTBE(val));}
#define eMIOS_MCR_GET_GTBE(emiosGroup)          EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_GTBE_MASK, eMIOS_MCR_GTBE_SHIFT)
/* Freeze bit set/get */
#define eMIOS_MCR_SET_FRZ(emiosGroup, val)     {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_FRZ_MASK, eMIOS_MCR_FRZ(val));}
#define eMIOS_MCR_GET_FRZ(emiosGroup)           EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_FRZ_MASK, eMIOS_MCR_FRZ_SHIFT)
/* Module Disable bit set/get */
#define eMIOS_MCR_SET_MDIS(emiosGroup, val)    {REG_RMW32(&eMIOS[emiosGroup]->MCR, eMIOS_MCR_MDIS_MASK, eMIOS_MCR_MDIS(val));}
#define eMIOS_MCR_GET_MDIS(emiosGroup)          EMIOS_GET_REG(eMIOS[emiosGroup]->MCR, eMIOS_MCR_MDIS_MASK, eMIOS_MCR_MDIS_SHIFT)

#if defined(FEATURE_EMIOS_UC_DISABLE)
/*UCDIS register*/
#if defined(eMIOS_UCDIS_CHDIS23_CHDIS0)
#define eMIOS_MCR_SET_UCDIS(emiosGroup, val)   {REG_RMW32(&eMIOS[emiosGroup]->UCDIS, eMIOS_UCDIS_CHDIS23_CHDIS0_MASK, eMIOS_UCDIS_CHDIS23_CHDIS0(val));}
#define eMIOS_MCR_GET_UCDIS(emiosGroup)         EMIOS_GET_REG(eMIOS[emiosGroup]->UCDIS, eMIOS_UCDIS_CHDIS23_CHDIS0_MASK, eMIOS_UCDIS_CHDIS23_CHDIS0_SHIFT)
#else
#define eMIOS_MCR_SET_UCDIS(emiosGroup, val)   {REG_RMW32(&eMIOS[emiosGroup]->UCDIS, eMIOS_UCDIS_CHDIS31_CHDIS0_MASK, eMIOS_UCDIS_CHDIS31_CHDIS0(val));}
#define eMIOS_MCR_GET_UCDIS(emiosGroup)         EMIOS_GET_REG(eMIOS[emiosGroup]->UCDIS, eMIOS_UCDIS_CHDIS31_CHDIS0_MASK, eMIOS_UCDIS_CHDIS31_CHDIS0_SHIFT)
#endif
#endif

/*!
 * brief Get eMIOS Validation channel support
 * param[in] channel The input channel in group
 * param[out] outChVal The output channel in group
 * return rue/false is channel status
 */
bool EMIOS_ValidateChannel(uint8_t inChVal, uint8_t * outChVal);

/*!
 * brief Get eMIOS global FLAG register
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return rue/false is channel status
 */
static inline bool EMIOS_GetFLAGReg(uint8_t emiosGroup, uint8_t channel)
{
    uint32_t tmp = 0UL;
    tmp = ((EMIOS_GET_REG(eMIOS[emiosGroup]->GFLAG, EMIOS_REGISTER_F31_F0_MASK, EMIOS_REGISTER_F31_F0_SHIFT) >> channel) & 0x01UL);

    return ((tmp == 1UL) ? true : false);
}

/*!
 * brief Set eMIOS output update disable register
 * param[in] emiosGroup The eMIOS group id
 * param[in] val value set channel in group
 */
static inline void EMIOS_SetOudisReg(uint8_t emiosGroup, uint32_t val)
{
#if defined(eMIOS_OUDIS_OU31_OU0)
    REG_RMW32(&eMIOS[emiosGroup]->OUDIS, eMIOS_OUDIS_OU31_OU0_MASK, eMIOS_OUDIS_OU31_OU0(val));
#elif defined(eMIOS_OUDIS_OU23_OU0)
    REG_RMW32(&eMIOS[emiosGroup]->OUDIS, eMIOS_OUDIS_OU23_OU0_MASK, eMIOS_OUDIS_OU23_OU0(val));
#else
    REG_RMW32(&eMIOS[emiosGroup]->OUDIS, eMIOS_OUDIS_OU7_OU0_MASK, eMIOS_OUDIS_OU7_OU0((uint8_t)val));
    REG_RMW32(&eMIOS[emiosGroup]->OUDIS, eMIOS_OUDIS_OU23_OU16_MASK, eMIOS_OUDIS_OU23_OU16((uint8_t)val));
#endif
}

/*!
 * brief Get eMIOS output update disable register
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return true/false is channel status
 */
static inline bool EMIOS_GetOudisReg(uint8_t emiosGroup, uint8_t channel)
{
    uint32_t tmp = 0UL;
    tmp = (EMIOS_GET_REG(eMIOS[emiosGroup]->OUDIS, EMIOS_REGISTER_F31_F0_MASK, EMIOS_REGISTER_F31_F0_SHIFT) >> channel);

    return (((tmp & 0x01UL) == 1UL) ? true : false);
}

/*-----------------------UC register A-----------------------------*/

/*!
 * brief Set A register's value, maximum value is 0xFFFFFF
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegA(uint8_t emiosGroup,
                                   uint8_t channel,
                                   uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].A, eMIOS_A_A_MASK, eMIOS_A_A(val));
}

/*!
 * brief Get A register's value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Unified Channels A register
 */
static inline uint32_t EMIOS_GetUCRegA(uint8_t emiosGroup,
                                       uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].A, eMIOS_A_A_MASK, eMIOS_A_A_SHIFT);
}

/*-----------------------UC register B-----------------------------*/

/*!
 * brief Set B register's value, maximum value is 0xFFFFFF
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegB(uint8_t emiosGroup,
                                   uint8_t channel,
                                   uint32_t val)
{
	REG_RMW32(&eMIOS[emiosGroup]->UC[channel].B, eMIOS_B_B_MASK, eMIOS_B_B(val));
}

/*!
 * brief Get B register's value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Unified Channels B register
 */
static inline uint32_t EMIOS_GetUCRegB(uint8_t emiosGroup,
                                       uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].B, eMIOS_B_B_MASK, eMIOS_B_B_SHIFT);
}

/*!
 * brief Get CNT register's value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Unified Channels CNT register
 */
static inline uint32_t EMIOS_GetUCRegCNT(uint8_t emiosGroup,
                                         uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].CNT, eMIOS_CNT_C_MASK, eMIOS_CNT_C_SHIFT);
}

/*!
 * brief Set CNT register's value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCNT(uint8_t emiosGroup,
                                     uint8_t channel,
                                     uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].CNT, eMIOS_CNT_C_MASK, eMIOS_CNT_C(val));
}

/*----------------------------UC register C bit fields--------------------------*/

/*!
 * brief Set mode of operation of the Unified Channel
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCMode(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_MODE_MASK, eMIOS_C_MODE(val));
}

/*!
 * brief Get mode of operation of the Unified Channel
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Mode selection in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegCMode(uint8_t emiosGroup,
                                           uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_MODE_MASK, eMIOS_C_MODE_SHIFT);
}

/*!
 * brief Set Edge Polarity bit.
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCEdpol(uint8_t emiosGroup,
                                        uint8_t channel,
                                        uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_EDPOL_MASK, eMIOS_C_EDPOL(val));
}

/*!
 * brief Get Edge Polarity bit.
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Edge Polarity bit in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegCEdpol(uint8_t emiosGroup,
                                            uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_EDPOL_MASK, eMIOS_C_EDPOL_SHIFT);
}

/*!
 * brief Set Edge Selection bit.
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCEdsel(uint8_t emiosGroup,
                                        uint8_t channel,
                                        uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_EDSEL_MASK, eMIOS_C_EDSEL(val));
}

/*!
 * brief Set Bus Select bits.
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCBsl(uint8_t emiosGroup,
                                      uint8_t channel,
                                      uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_BSL_MASK, eMIOS_C_BSL(val));
}

/*!
 * brief Get Bus Select value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Bus Select in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegCBsl(uint8_t emiosGroup,
                                          uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_BSL_MASK, eMIOS_C_BSL_SHIFT);
}

/*!
 * brief Force Match B bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0 Has no effect.
 *            - 1 Force a match at comparator B
 */
static inline void EMIOS_SetUCRegCForcmb(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_FORCMB_MASK, eMIOS_C_FORCMB(val));
}

/*!
 * brief Force Match A bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0 Has no effect.
 *            - 1 Force a match at comparator A
 */
static inline void EMIOS_SetUCRegCForcma(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_FORCMA_MASK, eMIOS_C_FORCMA(val));
}

/*!
 * brief Set FLAG Enable bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0 Disable (FLAG does not generate an interrupt request)
 *            - 1 Enable (FLAG generates an interrupt request)
 */
static inline void EMIOS_SetUCRegCFen(uint8_t emiosGroup,
                                      uint8_t channel,
                                      uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_FEN_MASK, eMIOS_C_FEN(val));
}

/*!
 * brief Set Filter Clock select bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCFck(uint8_t emiosGroup,
                                      uint8_t channel,
                                      uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_FCK_MASK, eMIOS_C_FCK(val));
}

/*!
 * brief Set Input Filter value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCIf(uint8_t emiosGroup,
                                     uint8_t channel,
                                     uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_IF_MASK, eMIOS_C_IF(val));
}

/*!
 * brief Set Direct Memory Access bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0      Flag/overrun assigned to Interrupt request.
 *            - 1      Flag/overrun assigned to DMA request.
 */
static inline void EMIOS_SetUCRegCDma(uint8_t emiosGroup,
                                      uint8_t channel,
                                      uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_DMA_MASK, eMIOS_C_DMA(val));
}

/*!
 * brief Get Direct Memory Access bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Direct Memory Access state in Unified Channels Control register
 *          - 0      Flag/overrun assigned to Interrupt request.
 *          - 1      Flag/overrun assigned to DMA request.
 */
static inline uint32_t EMIOS_GetUCRegCDma(uint8_t emiosGroup,
                                          uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_DMA_MASK, eMIOS_C_DMA_SHIFT);
}

/*!
 * brief Set Prescaler Enable bit.
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0     Prescaler disabled (no clock)
 *            - 1     Prescaler enabled
 */
static inline void EMIOS_SetUCRegCUcpren(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_UCPREN_MASK, eMIOS_C_UCPREN(val));
}

/*!
 * brief Set Output Disable select value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCOdissl(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_ODISSL_MASK, eMIOS_C_ODISSL(val));
}

/*!
 * brief Set Output Disable bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCOdis(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_ODIS_MASK, eMIOS_C_ODIS(val));
}

/*!
 * brief Set Freeze Enable bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *            - 0      Normal operation
 *            - 1      Freeze UC registers values
 */
static inline void EMIOS_SetUCRegCFren(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_FREN_MASK, eMIOS_C_FREN(val));
}

/*----------------------UC register S bit field-----------------------------*/
/*!
 * brief Write 1 to clear the FLAG bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 */
static inline void EMIOS_SetUCRegSFlag(uint8_t emiosGroup,
                                       uint8_t channel)
{
    /* OVR bit, OVFL bit & FLAG bit can be cleared when write 1 to them */
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].S, (eMIOS_S_FLAG_MASK & eMIOS_S_OVFL_MASK) & eMIOS_S_OVR_MASK, eMIOS_S_FLAG(1UL));
}

/*!
 * brief Get Unified Channel Output pin bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Unified Channel Output pin bit in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegSUcout(uint8_t emiosGroup,
                                            uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].S, eMIOS_S_UCOUT_MASK, eMIOS_S_UCOUT_SHIFT);
}

/*!
 * brief Get Unified Channel Input pin bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of Unified Channel Input pin bit in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegSUcin(uint8_t emiosGroup,
                                           uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].S, eMIOS_S_UCIN_MASK, eMIOS_S_UCIN_SHIFT);
}

#if defined(FEATURE_EMIOS_QD_MODE_SUPPORT)
/*!
 * brief Get Unified Channel Overrun bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return True if Overrun has occurred else overrun has not occurred
 */
static inline bool EMIOS_GetUCRegSOverrun(uint8_t emiosGroup,
                                              uint8_t channel)
{
    return ((eMIOS[emiosGroup]->UC[channel].S & eMIOS_S_OVR_MASK) >> eMIOS_S_OVR_SHIFT) != 0U;
}

/*!
 * brief Get Unified Channel Overflow bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return True if Overflow has occurred else overflow has not occurred
 */
static inline bool EMIOS_GetUCRegSOverflow(uint8_t emiosGroup,
                                           uint8_t channel)
{
    return ((eMIOS[emiosGroup]->UC[channel].S & eMIOS_S_OVFL_MASK) >> eMIOS_S_OVFL_SHIFT) != 0U;
}

/*!
 * brief Get Unified Channel FLAG bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return True if a match event in the comparators occurred else the has not a match event in the comparators occurred
 */
static inline bool EMIOS_GetUCRegSFlagState(uint8_t emiosGroup,
                                            uint8_t channel)
{
    return (eMIOS[emiosGroup]->UC[channel].S & eMIOS_S_FLAG_MASK) != 0U;
}
#endif /* FEATURE_EMIOS_QD_MODE_SUPPORT */

/*-----------------------UC register ALTA-----------------------------*/

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*!
 * brief Set A2 channel registers value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegALTA(uint8_t emiosGroup,
                                      uint8_t channel,
                                      uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].ALTA, eMIOS_ALTA_ALTA_MASK, eMIOS_ALTA_ALTA(val));
}

/*!
 * brief Get A2 channel registers value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * return Value of A2 channel registers in Unified Channels Control register
 */
static inline uint32_t EMIOS_GetUCRegALTA(uint8_t emiosGroup,
                                          uint8_t channel)
{
    return EMIOS_GET_REG(eMIOS[emiosGroup]->UC[channel].ALTA, eMIOS_ALTA_ALTA_MASK, eMIOS_ALTA_ALTA_SHIFT);
}
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
/*------------------UC register C2 --------------------------*/
/*!
 * brief Set Prescaler Clock select bit
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 *         - 0 Prescaled Clock
 *         - 1 eMIOS module clock
 */
static inline void EMIOS_SetUCRegC2UCPRECLK(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C2, eMIOS_C2_UCPRECLK_MASK, eMIOS_C2_UCPRECLK(val));
}

/*!
 * brief Set Extended Prescaler bits
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegC2UCEXTPRE(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C2, eMIOS_C2_UCEXTPRE_MASK, eMIOS_C2_UCEXTPRE(val));
}
#else /* Not defined FEATURE_EMIOS_PRESCALER_SELECT_BITS */
/*!
 * brief Set Prescaler value
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in group
 * param[in] val The value to set
 */
static inline void EMIOS_SetUCRegCUcpre(uint8_t emiosGroup,
                                        uint8_t channel,
                                        uint32_t val)
{
    REG_RMW32(&eMIOS[emiosGroup]->UC[channel].C, eMIOS_C_UCPRE_MASK, eMIOS_C_UCPRE(val));
}
#endif /* FEATURE_EMIOS_PRESCALER_SELECT_BITS */

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
/*!
 * brief EMIOS_ValidateMode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel channel
 * param[in] mode eMIOS mode of channel
 * return operation bool
 *        - true:        Valid mode.
 *        - false:       Invalid mode.
 */
bool EMIOS_ValidateMode(uint8_t emiosGroup,
                        uint8_t channel,
                        uint8_t mode);

/*!
 * brief EMIOS_ValidateInternalCnt
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel channel
 * return operation bool
 *        - true:        This channel has internal counter.
 *        - false:       This channel has external counter only.
 */
bool EMIOS_ValidateInternalCnt(uint8_t emiosGroup,
                               uint8_t channel);
#endif /* FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */

/*!
 * brief Allowing all channels in eMIOS group enter debug mode
 *
 * param[in] emiosGroup The eMIOS group id
 * return void
 */
void EMIOS_AllowEnterDebugMode(uint8_t emiosGroup);

#if defined(__cplusplus)
}
#endif

#endif /* EMIOS_HW_ACCESS_H */
/*******************************************************************************
* EOF
******************************************************************************/
