/*
 * Copyright 2017-2018 NXP
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
 * @file smpu_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or
 * different essential type.
 * This is required by the conversion of a bit-field of a register into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' type to 'essentially enum<i>'.
 * This is required by the conversion of a bit-field of a register into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (different essential type categories).
 * This is required by the conversion of a bit-field of a register into enum type.
 */

#include "smpu_hw_access.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/* Default value of access right */
#define DEFAULT_ACCESS_RIGHT    (SMPU_RW_OR_SET_3)

/*******************************************************************************
 * Code
 *******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_UnlockRegion
 * Description   : Unlocks the region descriptor.
 * If the region is locked by the master, only that master can modify the region descriptor
 * else need system reset to unlock.
 *
 *END**************************************************************************/
status_t SMPU_UnlockRegion(SMPU_Type * const base,
                           uint8_t regionNum)
{
    status_t ret = STATUS_SUCCESS;

    /* Unlock region */
    SMPU_SetRegionLock(base, regionNum, SMPU_UNLOCK);

    /* Check whether region is unlocked */
    if (SMPU_GetRegionLock(base, regionNum) != SMPU_UNLOCK)
    {
        ret = STATUS_ERROR;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_SetRegionAddr
 * Description   : Sets region start and end address.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 *END**************************************************************************/
void SMPU_SetRegionAddr(SMPU_Type * const base,
                        uint8_t regionNum,
                        uint32_t startAddr,
                        uint32_t endAddr)
{
    /* Write start address to RGD_WORD0 */
    base->RGD[regionNum].WORD0 = startAddr;

    /* Write end address to RGD_WORD1 */
    base->RGD[regionNum].WORD1 = endAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_SetMasterAccessRight
 * Description   : Sets access permission for master in region descriptor.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 *END**************************************************************************/
void SMPU_SetMasterAccessRight(SMPU_Type * const base,
                               uint8_t regionNum,
                               const smpu_master_access_right_t * const masterAccRight)
{
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;
#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
    accShift = (uint32_t)SMPU_WORD2_F0_M0P_WIDTH * masterAccRight->masterNum;
    accMask  = (uint32_t)SMPU_WORD2_F0_M0P_MASK >> accShift;
    accRight = SMPU_WORD2_F0_M0P(masterAccRight->accessRight) >> accShift;

    /* Set access right */
    temp = base->RGD[regionNum].WORD2.F0;
    temp = (temp & ~accMask) | accRight;
    base->RGD[regionNum].WORD2.F0 = temp;
#else
    accShift = (uint32_t)SMPU_WORD2_M0P_WIDTH * masterAccRight->masterNum;
    accMask  = (uint32_t)SMPU_WORD2_M0P_MASK >> accShift;
    accRight = SMPU_WORD2_M0P(masterAccRight->accessRight) >> accShift;

    /* Set access right */
    temp = base->RGD[regionNum].WORD2;
    temp = (temp & ~accMask) | accRight;
    base->RGD[regionNum].WORD2 = temp;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_InitRegion
 * Description   : Resets the region descriptor to default.
 *
 *END**************************************************************************/
void SMPU_InitRegion(SMPU_Type * const base,
                     uint8_t regionNum)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    /* Resets the WORD5 region descriptor */
    base->RGD[regionNum].WORD5 = 0U;
    /* Resets the WORD4 region descriptor */
    base->RGD[regionNum].WORD4 = 0U;
#endif
    /* Resets the WORD3 region descriptor */
    base->RGD[regionNum].WORD3 = 0U;
    /* Resets the WORD2 region descriptor */
#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
    base->RGD[regionNum].WORD2.F0 = 0U;
#else
    base->RGD[regionNum].WORD2 = 0U;
#endif
    /* Resets the WORD1 region descriptor */
    base->RGD[regionNum].WORD1 = FEATURE_SMPU_END_ADDRESS_RESET_VALUE;
    /* Resets the WORD0 region descriptor */
    base->RGD[regionNum].WORD0 = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_GetErrorInfo
 * Description   : Reports the SMPU access error detail information of error master
 * and clear this error flag.
 *
 *END**************************************************************************/
void SMPU_GetErrorInfo(const SMPU_Type * const base,
                       uint8_t masterNum,
                       smpu_access_err_info_t * const errInfoPtr)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
#if FEATURE_SMPU_SUPPORT_GETTING_ERROR_DETAIL
    /* Reports error master number */
    errInfoPtr->master = (uint8_t)((base->ERROR[masterNum].ADTL & SMPU_ADTL_EMN_MASK)
                       >> SMPU_ADTL_EMN_SHIFT);

    /* Reports error attribute */
    errInfoPtr->attributes = (smpu_err_attributes_t)((base->ERROR[masterNum].ADTL & SMPU_ADTL_EATTR_MASK)
                           >> SMPU_ADTL_EATTR_SHIFT);

    /* Reports error access type */
    errInfoPtr->accessType = (smpu_err_access_type_t)((base->ERROR[masterNum].ADTL & SMPU_ADTL_ERW_MASK)
                           >> SMPU_ADTL_ERW_SHIFT);

    /* Reports Error Access Control Detail:
     * If EDRn contains a captured error and EACD is all zeroes, an access did not hit
     * in any region descriptor.
     * If only a single EACD bit is set, the access error was caused by a single
     * non-overlapping region descriptor.
     * If two or more EACD bits are set, the access error was caused by an overlapping
     * set of region descriptors.
     */
    errInfoPtr->accessCtr = (base->ERROR[masterNum].ACDL & SMPU_ACDL_EACD_MASK) >> SMPU_ACDL_EACD_SHIFT;
#endif

    /* Reports error address */
    errInfoPtr->addr = base->ERROR[masterNum].ADR;

    /* Reports master error overrun status */
    errInfoPtr->overrun = (((base->CES1 & SMPU_CES1_MEOVR_MASK)
                            & SMPU_CES1_MEOVR(1UL << ((SMPU_CES1_MEOVR_WIDTH - 1U) - masterNum)))
                            != 0U);
#else
#if FEATURE_SMPU_SUPPORT_GETTING_ERROR_DETAIL
    /* Reports error master number */
    errInfoPtr->master = (uint8_t)((base->ERROR[masterNum].EDR & SMPU_EDR_EMN_MASK)
                       >> SMPU_EDR_EMN_SHIFT);

    /* Reports error attribute */
    errInfoPtr->attributes = (smpu_err_attributes_t)((base->ERROR[masterNum].EDR & SMPU_EDR_EATTR_MASK)
                           >> SMPU_EDR_EATTR_SHIFT);

    /* Reports error access type */
    errInfoPtr->accessType = (smpu_err_access_type_t)((base->ERROR[masterNum].EDR & SMPU_EDR_ERW_MASK)
                           >> SMPU_EDR_ERW_SHIFT);

    /* Reports Error Access Control Detail:
     * If EDRn contains a captured error and EACD is all zeroes, an access did not hit
     * in any region descriptor.
     * If only a single EACD bit is set, the access error was caused by a single
     * non-overlapping region descriptor.
     * If two or more EACD bits are set, the access error was caused by an overlapping
     * set of region descriptors.
     */
    errInfoPtr->accessCtr = (base->ERROR[masterNum].EDR & SMPU_EDR_EACD_MASK) >> SMPU_EDR_EACD_SHIFT;
#endif

    /* Reports error address */
    errInfoPtr->addr = base->ERROR[masterNum].EAR;

    /* Reports master error overrun status */
    errInfoPtr->overrun = (((base->CESR1 & SMPU_CESR1_MEOVR_MASK)
                            & SMPU_CESR1_MEOVR(1UL << ((SMPU_CESR1_MEOVR_WIDTH - 1U) - masterNum)))
                            != 0U);
#endif

#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
    /* Report Error Process Identification */
    errInfoPtr->processorIdentification = (uint8_t)((base->ERROR[masterNum].ADTL & SMPU_ADTL_PID_MASK)
                                        >> SMPU_ADTL_PID_SHIFT);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_GetDefaultMasterAccRight
 * Description   : Reports the default master access rights.
 *
 *END**************************************************************************/
void SMPU_GetDefaultMasterAccRight(uint8_t masterNum,
                                   smpu_master_access_right_t * const masterAccRight)
{
    masterAccRight->masterNum = masterNum;
    masterAccRight->accessRight = DEFAULT_ACCESS_RIGHT;
}

#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_SetRegionAccessSet
 * Description   : Sets access configuration for region.
 *
 *END**************************************************************************/
void SMPU_SetRegionAccessSet(SMPU_Type * const base,
                             uint8_t regionNum,
                             uint8_t setIdx,
                             smpu_specific_access_rights_t accSet)
{
    uint32_t accRight;
    uint32_t accMask;
    uint32_t accShift;
    uint32_t temp;

    accShift = (uint32_t)SMPU_WORD3_ACCSET1_WIDTH * setIdx;
    accMask  = (uint32_t)SMPU_WORD3_ACCSET1_MASK >> accShift;
    accRight = SMPU_WORD3_ACCSET1(accSet) >> accShift;

    /* Set access set for region */
    temp = base->RGD[regionNum].WORD3;
    temp = (temp & ~accMask) | accRight;
    base->RGD[regionNum].WORD3 = temp;
}
#endif

/*******************************************************************************
 * EOF
 *******************************************************************************/
