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
 * @file smpu_hw_access.h
 */

#ifndef SMPU_HW_ACCESS_H
#define SMPU_HW_ACCESS_H

#include "smpu_driver.h"

/*!
 * @brief System Memory Protection Unit Hardware Access layer.
 * @{
 */

/*******************************************************************************
 * API
 *******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables/Disables the SMPU module.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] enable Valid state
 *            - true  : Enable SMPU module.
 *            - false : Disable SMPU module.
 */
static inline void SMPU_Enable(SMPU_Type * const base,
                               bool enable)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    base->CES0 = (base->CES0 & ~SMPU_CES0_GVLD_MASK) | SMPU_CES0_GVLD(enable ? 1UL : 0UL);
#else
    base->CESR0 = (base->CESR0 & ~SMPU_CESR0_GVLD_MASK) | SMPU_CESR0_GVLD(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Enables/Disables the region descriptor.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] enable Valid state
 *            - true  : Region descriptor is valid.
 *            - false : Region descriptor is invalid.
 */
static inline void SMPU_SetRegionValidCmd(SMPU_Type * const base,
                                          uint8_t regionNum,
                                          bool enable)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    base->RGD[regionNum].WORD5 = ((base->RGD[regionNum].WORD5 & ~SMPU_WORD5_VLD_MASK)
                               | SMPU_WORD5_VLD(enable ? 1UL : 0UL));
#else
    base->RGD[regionNum].WORD3 = ((base->RGD[regionNum].WORD3 & ~SMPU_WORD3_VLD_MASK)
                               | SMPU_WORD3_VLD(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets region cache inhibit.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] enable Cache inhibit state
 *            - true  : Region cannot be cached.
 *            - false : Region can be cached.
 */
static inline void SMPU_SetRegionCacheInhibit(SMPU_Type * const base,
                                              uint8_t regionNum,
                                              bool enable)
{
    base->RGD[regionNum].WORD3 = ((base->RGD[regionNum].WORD3 & ~SMPU_WORD3_CI_MASK)
                               | SMPU_WORD3_CI(enable ? 1UL : 0UL));
}

/*!
 * @brief Gets region lock configuration.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @return operation status:
 *         - SMPU_UNLOCK     : Region unlocked.
 *         - SMPU_OWNER_LOCK : Region locked, only the master owner can modify the region descriptor.
 *         - SMPU_ALL_LOCK   : Region locked, all master cannot modify the region descriptor.
 */
static inline smpu_lock_t SMPU_GetRegionLock(const SMPU_Type * const base,
                                             uint8_t regionNum)
{
    smpu_lock_t ret;
    uint32_t temp;

#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    temp = (base->RGD[regionNum].WORD5 & SMPU_WORD5_LCK_MASK) >> SMPU_WORD5_LCK_SHIFT;
#else
    temp = (base->RGD[regionNum].WORD3 & SMPU_WORD3_RO_MASK) >> SMPU_WORD3_RO_SHIFT;
#endif
    switch (temp)
    {
        case 0U:
            ret = SMPU_UNLOCK;
            break;
    #if FEATURE_SMPU_HAS_OWNER_LOCK
        case 1U:
            ret = SMPU_OWNER_LOCK;
            break;
    #endif
        default:
            ret = SMPU_ALL_LOCK;
            break;
    }

    return ret;
}

/*!
 * @brief Sets region lock configuration.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] lockConfig Lock configuration
 *            - SMPU_UNLOCK     : Region unlocked.
 *            - SMPU_OWNER_LOCK : Region locked, only the master owner can modify the region descriptor.
 *            - SMPU_ALL_LOCK   : Region locked, all master cannot modify the region descriptor.
 */
static inline void SMPU_SetRegionLock(SMPU_Type * const base,
                                      uint8_t regionNum,
                                      smpu_lock_t lockConfig)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    base->RGD[regionNum].WORD5 = (base->RGD[regionNum].WORD5 & ~SMPU_WORD5_LCK_MASK)
                               | SMPU_WORD5_LCK(lockConfig);
#else
    base->RGD[regionNum].WORD3 = (base->RGD[regionNum].WORD3 & ~SMPU_WORD3_RO_MASK)
                               | SMPU_WORD3_RO(lockConfig);
#endif
}

/*!
 * @brief Unlocks the region descriptor.
 * If the region is locked, only region owner can unlock the region descriptor
 * locked by itself else need reset to unlock.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @return operation status:
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                            or all masters are locked.
 */
status_t SMPU_UnlockRegion(SMPU_Type * const base,
                           uint8_t regionNum);

#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
/*!
 * @brief Gets the master ID which is region owner.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @return The master ID
 */
static inline uint8_t SMPU_GetRegionOwner(const SMPU_Type * const base,
                                          uint8_t regionNum)
{
    return (uint8_t)((base->RGD[regionNum].WORD5 & SMPU_WORD5_MID_MASK) >> SMPU_WORD5_MID_SHIFT);
}

/*!
 * @brief Enables/Disables the region process identifier.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] enable State
 *            - true  : Include PID in the region hit evaluation.
 *            - false : Do not include PID in the region hit evaluation.
 */
static inline void SMPU_EnableProcessIdentifier(SMPU_Type * const base,
                                                uint8_t regionNum,
                                                bool enable)
{
    base->RGD[regionNum].WORD4 = (base->RGD[regionNum].WORD4 & ~SMPU_WORD4_PIDEN_MASK)
                               | SMPU_WORD4_PIDEN(enable ? 1UL : 0UL);
}

/*!
 * @brief Sets the region process identifier.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] pid The process identifier.
 */
static inline void SMPU_SetProcessIdentifier(SMPU_Type * const base,
                                             uint8_t regionNum,
                                             uint8_t pid)
{
    base->RGD[regionNum].WORD4 = (base->RGD[regionNum].WORD4 & ~SMPU_WORD4_PID_MASK)
                               | SMPU_WORD4_PID(pid);
}

/*!
 * @brief Sets the region process identifier mask.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] pidMask The process identifier mask.
 */
static inline void SMPU_SetProcessIdentifierMask(SMPU_Type * const base,
                                                 uint8_t regionNum,
                                                 uint8_t pidMask)
{
    base->RGD[regionNum].WORD4 = (base->RGD[regionNum].WORD4 & ~SMPU_WORD4_PID_MSK_MASK)
                               | SMPU_WORD4_PID_MSK(pidMask);
}
#endif

/*!
 * @brief Sets region start and end address.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] startAddr Region start address.
 * @param[in] endAddr Region end address.
 */
void SMPU_SetRegionAddr(SMPU_Type * const base,
                        uint8_t regionNum,
                        uint32_t startAddr,
                        uint32_t endAddr);

/*!
 * @brief Sets access permission for master in region descriptor.
 * Please note that using this function will clear the valid bit of the region,
 * and a further validation might be needed.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] masterNum The master number.
 * @param[in] masterAccRight The pointer to master access right structure, see #smpu_master_access_right_t.
 */
void SMPU_SetMasterAccessRight(SMPU_Type * const base,
                               uint8_t regionNum,
                               const smpu_master_access_right_t * const masterAccRight);

/*!
 * @brief Resets the region descriptor to default.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 */
void SMPU_InitRegion(SMPU_Type * const base,
                     uint8_t regionNum);

/*!
 * @brief Gets the error status of a specified bus master.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] masterNum The master number.
 * @return operation status:
 *         - true  : An error has occurred for bus master.
 *         - false : No error has occurred for bus master.
 */
static inline bool SMPU_GetErrorStatus(const SMPU_Type * const base,
                                       uint8_t masterNum)
{
    bool status;
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    status = ((base->CES0 & SMPU_CES0_MERR(1UL << ((SMPU_CES0_MERR_WIDTH - 1U) - masterNum))) != 0U);
#else
    status = ((base->CESR0 & SMPU_CESR0_MERR(1UL << ((SMPU_CESR0_MERR_WIDTH - 1U) - masterNum))) != 0U);
#endif
    return status;
}

/*!
 * @brief Clears the error flag of a specified bus master.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] masterNum The error master number.
 */
static inline void SMPU_ClearErrorFlag(SMPU_Type * const base,
                                       uint8_t masterNum)
{
#if (FEATURE_SMPU_HARDWARE_REVISION_LEVEL == 4U)
    base->CES0 = (base->CES0 & ~SMPU_CES0_MERR_MASK)
               | SMPU_CES0_MERR(1UL << ((SMPU_CES0_MERR_WIDTH - 1U) - masterNum));
#else
    base->CESR0 = (base->CESR0 & ~SMPU_CESR0_MERR_MASK)
               | SMPU_CESR0_MERR(1UL << ((SMPU_CESR0_MERR_WIDTH - 1U) - masterNum));
#endif
}

/*!
 * @brief Reports the SMPU access error detail information of error master.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] masterNum The master number.
 * @param[out] errInfoPtr The pointer to the SMPU access error information.
 */
void SMPU_GetErrorInfo(const SMPU_Type * const base,
                       uint8_t masterNum,
                       smpu_access_err_info_t * const errInfoPtr);

/*!
 * @brief Reports the default master access rights.
 *
 * @param[in] masterNum The master number.
 * @param[out] masterAccRight The pointer to the master access rights.
 */
void SMPU_GetDefaultMasterAccRight(uint8_t masterNum,
                                   smpu_master_access_right_t * const masterAccRight);

#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
/*!
 * @brief Uses specific access permission
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] enable State
 *            - true  : Use specific access rights
 *            - false : Use normal access rights
 */
static inline void SMPU_EnableRegionSpecificAccess(SMPU_Type * const base,
                                                   uint8_t regionNum,
                                                   bool enable)
{
    base->RGD[regionNum].WORD3 = (base->RGD[regionNum].WORD3 & ~SMPU_WORD3_FMT_MASK)
                               | SMPU_WORD3_FMT(enable ? 1UL : 0UL);
}

/*!
 * @brief Sets access configurations for region.
 *
 * @param[in] base The SMPU peripheral base address.
 * @param[in] regionNum The region number.
 * @param[in] setIdx The set index, Set n = setIdx + 1.
 * @param[in] accSet The access configuration.
 */
void SMPU_SetRegionAccessSet(SMPU_Type * const base,
                             uint8_t regionNum,
                             uint8_t setIdx,
                             smpu_specific_access_rights_t accSet);
#endif

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* SMPU_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
