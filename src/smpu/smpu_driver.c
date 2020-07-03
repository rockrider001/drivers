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
 * @file smpu_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include <stddef.h>
#include "smpu_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for SMPU instances. */
static SMPU_Type * const s_smpuBase[SMPU_INSTANCE_COUNT] = SMPU_BASE_PTRS;

/*******************************************************************************
 * Code
 *******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_Init
 * Description   : Initializes system memory protection unit
 * by setting the access configurations of all available masters,
 * process identifier and the memory location for the given regions
 * and activate module finally.
 *
 * Implements    : SMPU_DRV_Init_Activity
 *END**************************************************************************/
status_t SMPU_DRV_Init(uint32_t instance,
                       uint8_t regionCnt,
                       const smpu_user_config_t * userConfigPtr)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT((regionCnt > 0U) && (regionCnt <= FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT));
    }
    else
#endif
    {
        DEV_ASSERT((regionCnt > 0U) && (regionCnt <= SMPU_RGD_COUNT));
    }

    SMPU_Type * base = s_smpuBase[instance];
    uint8_t regionNum = 0U;

    /* De-initializes all region descriptor */
    status_t ret = SMPU_DRV_Deinit(instance);

    if (ret == STATUS_SUCCESS)
    {
        for (regionNum = 0U; regionNum < regionCnt; regionNum++)
        {
            /* Sets region configuration */
            ret = SMPU_DRV_SetRegionConfig(instance, regionNum, &userConfigPtr[regionNum]);

            if (ret != STATUS_SUCCESS)
            {
                break;
            }
        }

        if (ret == STATUS_SUCCESS)
        {
            /* Enables the SMPU module operation */
            SMPU_Enable(base, true);
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_Deinit
 * Description   : De-initializes system memory protection unit
 * by reseting all regions to default and disable module.
 *
 * Implements    : SMPU_DRV_Deinit_Activity
 *END**************************************************************************/
status_t SMPU_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);

    SMPU_Type * base = s_smpuBase[instance];
    status_t ret = STATUS_SUCCESS;
    uint8_t regionNum = 0U;
    uint8_t errorNum = 0U;
    uint8_t maxRegionNum = SMPU_RGD_COUNT;

    /* Disables the entire SMPU module */
    SMPU_Enable(base, false);

    for (errorNum = 0U; errorNum < SMPU_ERROR_COUNT; errorNum++)
    {
        /* Clears master error flag */
        SMPU_ClearErrorFlag(base, errorNum);
    }

    /* Set maximum region descriptor number */
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        maxRegionNum = FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT;
    }
#endif
    /* De-initializes all region descriptors */
    for (regionNum = 0U; regionNum < maxRegionNum; regionNum++)
    {
        /* Unlock region descriptors */
        ret = SMPU_UnlockRegion(base, regionNum);

        /* Unlock successful */
        if (ret == STATUS_SUCCESS)
        {
            /* Resets the region configuration to default */
            SMPU_InitRegion(base, regionNum);
        }
        else
        {
            /* Break out of loop */
            break;
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_GetDefaultRegionConfig
 * Description   : Gets default region configuration.
 * Grants all access rights for masters; disable PID and cache; unlock region descriptor.
 *
 * Implements    : SMPU_DRV_GetDefaultRegionConfig_Activity
 *END**************************************************************************/
smpu_user_config_t SMPU_DRV_GetDefaultRegionConfig(smpu_master_access_right_t * masterAccRight)
{
    DEV_ASSERT(masterAccRight != NULL);

    uint8_t masterIdx = 0U;
    uint8_t masterNum[FEATURE_SMPU_MASTER_COUNT] = FEATURE_SMPU_MASTER;
    smpu_user_config_t regionConfig;

    /* Allocates entire memory */
    regionConfig.startAddr = 0x0U;
    regionConfig.endAddr = 0xFFFFFFFFU;

#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
    regionConfig.specAccessEnable = false;
    regionConfig.specAccessSet = NULL;
#endif

    /* All access rights are allowed */
    for (masterIdx = 0U; masterIdx < FEATURE_SMPU_MASTER_COUNT; masterIdx++)
    {
        SMPU_GetDefaultMasterAccRight(masterNum[masterIdx], &masterAccRight[masterIdx]);
    }
    regionConfig.masterAccRight = masterAccRight;

    /* Disables cache */
    regionConfig.cacheInhibitEnable = true;

#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
    /* Disables process identifier */
    regionConfig.processIdEnable = false;
    regionConfig.processIdentifier = 0U;
    regionConfig.processIdMask = 0U;
#endif

    /* Does not lock the region descriptor */
    regionConfig.lockConfig = SMPU_UNLOCK;

    return regionConfig;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_SetRegionConfig
 * Description   : Configures the region descriptor.
 * Updates the access configuration of all available masters,
 * process identifier and memory location in a given region.
 *
 * Implements    : SMPU_DRV_SetRegionConfig_Activity
 *END**************************************************************************/
status_t SMPU_DRV_SetRegionConfig(uint32_t instance,
                                  uint8_t regionNum,
                                  const smpu_user_config_t * regionConfigPtr)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
    DEV_ASSERT(regionConfigPtr != NULL);
    DEV_ASSERT(regionConfigPtr->startAddr <= regionConfigPtr->endAddr);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];
    uint8_t masterIdx = 0U;
    uint8_t setIdx = 0U;
    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    /* Unlocks region descriptor */
    if (ret == STATUS_SUCCESS)
    {
    #if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
        /* Sets specific access set for region */
        SMPU_EnableRegionSpecificAccess(base, regionNum, regionConfigPtr->specAccessEnable);
        if (regionConfigPtr->specAccessEnable)
        {
            DEV_ASSERT(regionConfigPtr->specAccessSet != NULL);
            for (setIdx = 0U; setIdx < FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT; setIdx++)
            {
                SMPU_SetRegionAccessSet(base,
                                        regionNum,
                                        setIdx,
                                        regionConfigPtr->specAccessSet[setIdx]);
            }
        }
    #endif
        /* Sets access right for masters */
        for (masterIdx = 0U; masterIdx < FEATURE_SMPU_MASTER_COUNT; masterIdx++)
        {
            DEV_ASSERT(regionConfigPtr->masterAccRight != NULL);
            if (regionConfigPtr->masterAccRight[masterIdx].masterNum <= FEATURE_SMPU_MAX_MASTER_NUMBER)
            {
                SMPU_SetMasterAccessRight(base,
                                          regionNum,
                                          &regionConfigPtr->masterAccRight[masterIdx]);
            }
            else
            {
                ret = STATUS_ERROR;
                break;
            }
        }

        if (ret == STATUS_SUCCESS)
        {
            /* Sets a region's start and end addresses */
            SMPU_SetRegionAddr(base, regionNum, regionConfigPtr->startAddr, regionConfigPtr->endAddr);

        #if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
            /* Sets process identifier */
            SMPU_SetProcessIdentifier(base, regionNum, regionConfigPtr->processIdentifier);

            /* Sets process identifier mask */
            SMPU_SetProcessIdentifierMask(base, regionNum, regionConfigPtr->processIdMask);

            /* Enables/Disables process identifier */
            SMPU_EnableProcessIdentifier(base, regionNum, regionConfigPtr->processIdEnable);
        #endif

            /* Sets cache inhibit */
            SMPU_SetRegionCacheInhibit(base, regionNum, regionConfigPtr->cacheInhibitEnable);

            /* Enables the region descriptor valid bit */
            SMPU_SetRegionValidCmd(base, regionNum, true);

            /* Sets region lock configuration */
            SMPU_SetRegionLock(base, regionNum, regionConfigPtr->lockConfig);
        }
    }

    (void)setIdx;
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_SetRegionAddr
 * Description   : Configures the region start and end address.
 * Please note that using this function will unlock the region descriptor.
 *
 * Implements    : SMPU_DRV_SetRegionAddr_Activity
 *END**************************************************************************/
status_t SMPU_DRV_SetRegionAddr(uint32_t instance,
                                uint8_t regionNum,
                                uint32_t startAddr,
                                uint32_t endAddr)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
    DEV_ASSERT(startAddr <= endAddr);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];
    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    /* Checks region unlock status */
    if (ret == STATUS_SUCCESS)
    {
        /* Sets a region's start and end addresses */
        SMPU_SetRegionAddr(base, regionNum, startAddr, endAddr);

        /* Re-enables the region descriptor valid bit */
        SMPU_SetRegionValidCmd(base, regionNum, true);
    }

    return ret;
}

#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_SetRegionProcessId
 * Description   : Configures the region process identifier.
 *
 * Implements    : SMPU_DRV_SetRegionProcessId_Activity
 *END**************************************************************************/
status_t SMPU_DRV_SetRegionProcessId(uint32_t instance,
                                     uint8_t regionNum,
                                     bool enable,
                                     uint8_t pid,
                                     uint8_t pidMask)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];
    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    /* Unlocks region descriptor */
    if (ret == STATUS_SUCCESS)
    {
        /* Sets process identifier */
        SMPU_SetProcessIdentifier(base, regionNum, pid);

        /* Sets process identifier mask */
        SMPU_SetProcessIdentifierMask(base, regionNum, pidMask);

        /* Enables/Disables process identifier */
        SMPU_EnableProcessIdentifier(base, regionNum, enable);

        /* Enables the region descriptor valid bit */
        SMPU_SetRegionValidCmd(base, regionNum, true);
    }

    return ret;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_EnableRegion
 * Description   : Enables/Disables region descriptor.
 * Please note that using this function will unlock the region descriptor.
 *
 * Implements    : SMPU_DRV_EnableRegion_Activity
 *END**************************************************************************/
status_t SMPU_DRV_EnableRegion(uint32_t instance,
                               uint8_t regionNum,
                               bool enable)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];
    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    /* Checks region unlock status */
    if (ret == STATUS_SUCCESS)
    {
        /* Enables/Disables region descriptor */
        SMPU_SetRegionValidCmd(base, regionNum, enable);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_SetMasterAccessRights
 * Description   : Configures access permission of master in the region.
 * Please note that using this function will unlock the region descriptor.
 *
 * Implements    : SMPU_DRV_SetMasterAccessRights_Activity
 *END**************************************************************************/
status_t SMPU_DRV_SetMasterAccessRights(uint32_t instance,
                                        uint8_t regionNum,
                                        const smpu_master_access_right_t * masterAccRight)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
    DEV_ASSERT(masterAccRight != NULL);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];
    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    if (ret == STATUS_SUCCESS)
    {
        if (masterAccRight->masterNum <= FEATURE_SMPU_MAX_MASTER_NUMBER)
        {
            /* Sets master access right */
            SMPU_SetMasterAccessRight(base, regionNum, masterAccRight);

            /* Re-enables region descriptor valid */
            SMPU_SetRegionValidCmd(base, regionNum, true);
        }
        else
        {
            ret = STATUS_ERROR;
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_SetRegionLockConfig
 * Description   : Sets the region lock configuration.
 *
 * Implements    : SMPU_DRV_SetRegionLockConfig_Activity
 *END**************************************************************************/
status_t SMPU_DRV_SetRegionLockConfig(uint32_t instance,
                                      uint8_t regionNum,
                                      smpu_lock_t lockConfig)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    SMPU_Type * base = s_smpuBase[instance];

    /* Unlocks region descriptor */
    status_t ret = SMPU_UnlockRegion(base, regionNum);

    if (ret == STATUS_SUCCESS)
    {
        /* Sets region lock configuration */
        SMPU_SetRegionLock(base, regionNum, lockConfig);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_GetRegionLockInfo
 * Description   : Reports the region lock status.
 *
 * Implements    : SMPU_DRV_GetRegionLockInfo_Activity
 *END**************************************************************************/
smpu_region_lock_t SMPU_DRV_GetRegionLockInfo(uint32_t instance,
                                              uint8_t regionNum)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
#ifdef FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT
    if (FEATURE_SMPU_HAS_SPECIFIC_INSTANCE == instance)
    {
        DEV_ASSERT(regionNum < FEATURE_SMPU_HAS_SPECIFIC_RGD_COUNT);
    }
    else
#endif
    {
        DEV_ASSERT(regionNum < SMPU_RGD_COUNT);
    }

    const SMPU_Type * base = s_smpuBase[instance];
    smpu_region_lock_t regionLockStatus;

    regionLockStatus.regionNum = regionNum;
#if FEATURE_SMPU_HAS_OWNER_LOCK
    regionLockStatus.masterOwner = SMPU_GetRegionOwner(base, regionNum);
#endif
    regionLockStatus.lockConfig = SMPU_GetRegionLock(base, regionNum);

    return regionLockStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SMPU_DRV_GetDetailErrorInfo
 * Description   : Checks and gets the access error detail information
 * if the error caused by master.
 *
 * Implements    : SMPU_DRV_GetDetailErrorInfo_Activity
 *END**************************************************************************/
bool SMPU_DRV_GetDetailErrorInfo(uint32_t instance,
                                 uint8_t errorNum,
                                 smpu_access_err_info_t * errInfoPtr)
{
    DEV_ASSERT(instance < SMPU_INSTANCE_COUNT);
    DEV_ASSERT(errorNum < SMPU_ERROR_COUNT);
    DEV_ASSERT(errInfoPtr != NULL);

    SMPU_Type * base = s_smpuBase[instance];
    /* Gets error status */
    bool ret = SMPU_GetErrorStatus(base, errorNum);

    /* Checks error status */
    if (ret)
    {
        /* Gets the detail error */
        SMPU_GetErrorInfo(base, errorNum, errInfoPtr);

        /* Clears error flag */
        SMPU_ClearErrorFlag(base, errorNum);
    }

    return ret;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
