/*
 * Copyright 2017-2019 NXP.
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
 * @file fccu_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 */

#include <stddef.h>
#include "fccu_hw_access.h"

/*******************************************************************************
* Code
*******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetNCFConfig
 * Description   : Set the Non Critical Fault corresponding bits field to Hardware
 * or Software recoverable fault
 *
 * Implements    : FCCU_SetNCFConfig_Activity
 * END**************************************************************************/
void FCCU_SetNCFConfig(FCCU_Type * const base,
                       uint8_t faultIndex,
                       fccu_ncf_config_t config)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    base->NCF_CFG[rIdx] = (base->NCF_CFG[rIdx] & ~(1UL << fIdx))
                        | ((uint32_t)config << fIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetNCFStatusConfig
 * Description   : Set the Non Critical Fault corresponding bits field to Reset behavior
 *
 * Implements    : FCCU_SetNCFStatusConfig_Activity
 * END**************************************************************************/
void FCCU_SetNCFStatusConfig(FCCU_Type * const base,
                             uint8_t faultIndex,
                             fccu_ncfs_config_t config)
{
    uint8_t rIdx = faultIndex >> 4U;
    uint8_t fIdx = (faultIndex & 0xFU) << 1U;

    base->NCFS_CFG[rIdx] = (base->NCFS_CFG[rIdx] & ~(3UL << fIdx))
                         | ((uint32_t)config << fIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_GetNCFStatus
 * Description   : Return if a NonCritical Fault is active or not
 *
 * Implements    : FCCU_GetNCFStatus_Activity
 * END**************************************************************************/
bool FCCU_GetNCFStatus(const FCCU_Type * const base,
                       uint8_t faultIndex)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    return (((base->NCF_E[rIdx] & (1UL << fIdx)) != 0U) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_ClearNCFStatus
 * Description   : Set as HW recoverable fault the corresponding Non Critical Fault source
 *
 * Implements    : FCCU_ClearNCFStatus_Activity
 * END**************************************************************************/
void FCCU_ClearNCFStatus(FCCU_Type * const base,
                         uint8_t faultIndex)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    base->NCF_CFG[rIdx] &= ~(1UL << fIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_ClearNCFsStatus
 * Description   : Clear the Fault latched
 *
 * Implements    : FCCU_ClearNCFsStatus_Activity
 * END**************************************************************************/
void FCCU_ClearNCFsStatus(FCCU_Type * const base,
                          uint8_t faultIndex)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    base->NCF_S[rIdx] = 1UL << fIdx;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_GetNCFsStatus
 * Description   : Return the status if a NonCritical Fault source was triggered or not
 *
 * Implements    : FCCU_GetNCFsStatus_Activity
 * END**************************************************************************/
bool FCCU_GetNCFsStatus(const FCCU_Type * const base,
                        uint8_t faultIndex)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    return (((base->NCF_S[rIdx] & (1UL << fIdx)) != 0U) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetNCFEnable
 * Description   : Enable the Fault source to transition from Normal to Fault or Alarm state
 *
 * Implements    : FCCU_SetNCFEnable_Activity
 * END**************************************************************************/
void FCCU_SetNCFEnable(FCCU_Type * const base,
                       uint8_t faultIndex,
                       bool enable)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    base->NCF_E[rIdx] = (base->NCF_E[rIdx] & ~(1UL << fIdx))
                        | ((enable ? 1UL : 0UL) << fIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetNCFTimeoutEnable
 * Description   : Enable the Fault source to transition from Alarm to Fault state
 * when the Time Out (NCF_TO) timer expires
 *
 * Implements    : FCCU_SetNCFTimeoutEnable_Activity
 * END**************************************************************************/
void FCCU_SetNCFTimeoutEnable(FCCU_Type * const base,
                             uint8_t faultIndex,
                             bool enable)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    base->NCF_TOE[rIdx] = (base->NCF_TOE[rIdx] & ~(1UL << fIdx))
                        | ((enable ? 1UL : 0UL) << fIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_GetFreezeStatus
 * Description   : Returns the noncritical source of a Triggered event based on the type requested
 *
 * Implements    : FCCU_GetFreezeStatus_Activity
 * END**************************************************************************/
uint8_t FCCU_GetFreezeStatus(const FCCU_Type * const base,
                             fccu_freeze_type_t type)
{
    uint8_t status;

    switch (type)
    {
        case FCCU_FRZ_NORMAL_ALARM:
            status = (uint8_t)((base->N2AF_STATUS & FCCU_N2AF_STATUS_NAFS_MASK) >> FCCU_N2AF_STATUS_NAFS_SHIFT);
            break;
        case FCCU_FRZ_ALARM_FAULT:
            status = (uint8_t)((base->A2FF_STATUS & FCCU_A2FF_STATUS_AFFS_MASK) >> FCCU_A2FF_STATUS_AFFS_SHIFT);
            break;
        case FCCU_FRZ_NORMAL_FAULT:
            status = (uint8_t)((base->N2FF_STATUS & FCCU_N2FF_STATUS_NFFS_MASK) >> FCCU_N2FF_STATUS_NFFS_SHIFT);
            break;
#if FEATURE_FCCU_RCC_EN
        case FCCU_FRZ_SELF_CHECK_RCC:
            status = (uint8_t)(base->SCFS & (FCCU_SCFS_RCCS0_MASK | FCCU_SCFS_RCCS1_MASK));
            break;
#endif
        default:
            status = (uint8_t)((base->F2A_STATUS & FCCU_F2A_STATUS_FAFS_MASK) >> FCCU_F2A_STATUS_FAFS_SHIFT);
            break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_GetIntStatus
 * Description   : Returns the if an interrupt source was triggered or not
 *
 * Implements    : FCCU_GetIntStatus_Activity
 * END**************************************************************************/
bool FCCU_GetIntStatus(const FCCU_Type * const base,
                       fccu_int_status_t intStatus)
{
    register uint32_t regTempVal;
    regTempVal = base->IRQ_STAT;
    regTempVal &= ((uint32_t)FCCU_IRQ_STAT_CFG_TO_STAT_MASK << intStatus);

    return (regTempVal != 0U) ? true : false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_ClearIntFlag
 * Description   : Clears an interrupt flag based on type selected
 *
 * Implements    : FCCU_ClearIntFlag_Activity
 * END**************************************************************************/
void FCCU_ClearIntFlag(FCCU_Type * const base,
                       fccu_int_status_t intStatus)
{
    if ((FEATURE_FCCU_IRQ_EN_MASK & (FCCU_IRQ_EN_CFG_TO_IEN_MASK << intStatus)) != 0U)
    {
        register uint32_t regTempVar;
        base->IRQ_STAT = (base->IRQ_STAT & ~FEATURE_FCCU_IRQ_EN_MASK);
        regTempVar = ((uint32_t)FCCU_IRQ_EN_CFG_TO_IEN_MASK << intStatus);
        base->IRQ_STAT |= regTempVar;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetIntEnable
 * Description   : Configure an global interrupt  based on type selected
 *
 * Implements    : FCCU_SetIntEnable_Activity
 * END**************************************************************************/
void FCCU_SetIntEnable(FCCU_Type * const base,
                       fccu_int_status_t intStatus,
                       bool enable)
{
    if ((FEATURE_FCCU_IRQ_EN_MASK & (FCCU_IRQ_EN_CFG_TO_IEN_MASK << intStatus)) != 0U)
    {
        register uint32_t regTempVar;
        regTempVar = ~((uint32_t)FCCU_IRQ_EN_CFG_TO_IEN_MASK << intStatus);
        base->IRQ_EN &= regTempVar;
        regTempVar = (uint32_t)(enable ? (FCCU_IRQ_EN_CFG_TO_IEN_MASK << intStatus) : 0UL);
        base->IRQ_EN |= regTempVar;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_SetIntSource
 * Description   : Configure an interrupt type for NonCritical Fault based on type selected
 *
 * Implements    : FCCU_SetIntSource_Activity
 * END**************************************************************************/
void FCCU_SetIntSource(FCCU_Type * const base,
                       fccu_int_status_t intStatus,
                       uint8_t faultIndex,
                       bool enable)
{
    uint8_t rIdx = faultIndex >> 5U;
    uint8_t fIdx = faultIndex & 0x1FU;

    switch (intStatus)
    {
        case FCCU_INT_ALARM:
            base->IRQ_ALARM_EN[rIdx] = (base->IRQ_ALARM_EN[rIdx] & ~(1UL << fIdx))
                                     | ((enable ? 1UL : 0UL) << fIdx);
            break;
        case FCCU_INT_NMI:
            base->NMI_EN[rIdx] = (base->NMI_EN[rIdx] & ~(1UL << fIdx))
                               | ((enable ? 1UL : 0UL) << fIdx);
            break;
        case FCCU_INT_EOUT:
            base->EOUT_SIG_EN[rIdx] = (base->EOUT_SIG_EN[rIdx] & ~(1UL << fIdx))
                                    | ((enable ? 1UL : 0UL) << fIdx);
            break;
        default:
            /* Do Nothing */
        break;
    }
}

#if FEATURE_FCCU_CONTROL_MODE_EN
/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_GetModeControllerStatus
 * Description   : Indicates the last 4 chip modes as defined by MC_ME module when FCCU is
 * in Fault State and a NMIOUT was asserted. And returns the validity, status and the chip mode
 * for the last recent event selected.
 *
 * Implements    : FCCU_GetModeControllerStatus_Activity
 * END**************************************************************************/
void FCCU_GetModeControllerStatus(const FCCU_Type * const base,
                                  fccu_chip_mode_t recentMode,
                                  fccu_mode_info_t * infoPtr)
{
    register uint32_t regTempVal;
    register uint32_t recentModeTemp=(uint32_t)recentMode<<3U;

    infoPtr->valid = ((base->MCS & ((uint32_t)FCCU_MCS_VL0_MASK << recentModeTemp)) != 0U) ? true : false;
    infoPtr->faultStatus = ((base->MCS & ((uint32_t)FCCU_MCS_FS0_MASK << recentModeTemp)) != 0U) ? true : false;
    regTempVal = ((uint32_t)FCCU_MCS_MCS0_MASK << recentModeTemp);
    infoPtr->mode = (uint8_t)((base->MCS & regTempVal) >> recentModeTemp);
}
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

/*******************************************************************************
* EOF
*******************************************************************************/
