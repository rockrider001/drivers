/*
 * Copyright 2019 NXP
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
 * @file mpu_e200_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 2.2, Highest operation function lacks
 * side-effects.
 * Function contain assembly code that is compiler specific.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower
 * or different essential type.
 * Enumeration values are declared to hold the bitfield value.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast.
 * This is required by the conversion of a bool into a bit and from a bit to a
 * boolean value. Enumeration values are declared to hold the bitfield value.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression.
 * This is required by the conversion of a bool into a bit and from a bit to a
 * boolean value. Enumeration values are declared to hold the bitfield value.
 */

#include "mpu_e200_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Macro for all access value */
#define MPU_E200_BYPASS_ALL (uint8_t)(MPU_E200_BYPASS_EXEC | MPU_E200_BYPASS_READ | MPU_E200_BYPASS_WRITE)
#define MPU_E200_DEBUG_ALL (uint8_t)(MPU_E200_DEBUG_EXEC | MPU_E200_DEBUG_READ | MPU_E200_DEBUG_WRITE)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* @brief Variable storing special register MAS0 value */
static uint32_t mas0 = 0UL;
/* @brief Variable storing special register MAS1 value */
static uint32_t mas1 = 0UL;
/* @brief Variable storing special register MAS2 value */
static uint32_t mas2 = 0UL;
/* @brief Variable storing special register MAS3 value */
static uint32_t mas3 = 0UL;
/* @brief Variable storing special register CSR0 value */
static uint32_t csr0 = 0UL;

/*******************************************************************************
 * Code
 *******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_WriteEntry
 * Description   : Writes all MPU Assist Registers with the user configuration
 * and then writes the configuration in the MPU entry.
 *
 *END**************************************************************************/
static void MPU_E200_WriteEntry(uint32_t mas0Value, uint32_t mas1Value,
                        uint32_t mas2Value, uint32_t mas3Value)
{
    /* Write MPU entry configuration to MPU Assist Registers */
    MTSPR(MPU_E200_MAS0_SPRN, mas0Value);
    MTSPR(MPU_E200_MAS1_SPRN, mas1Value);
    MTSPR(MPU_E200_MAS2_SPRN, mas2Value);
    MTSPR(MPU_E200_MAS3_SPRN, mas3Value);
    /* Configure MPU entry using the MAS registers*/
    MPU_E200_MPUWE();
    MPU_E200_MPUSYNC();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_ReadEntry
 * Description   : Writes the entry ID to the MPU Assist Register 0 and reads
 * from the MPU the configuration into the MPU Assist Registers.
 *
 *END**************************************************************************/
static void MPU_E200_ReadEntry(uint32_t entryId, uint32_t *mas0Value,
                            uint32_t *mas1Value, uint32_t *mas2Value,
                            uint32_t *mas3Value)
{
    /* Read entry configuration into MPU Assist registers */
    MTSPR(MPU_E200_MAS0_SPRN, entryId);
    MPU_E200_MPURE();
    MPU_E200_MPUSYNC();
    /* Read configuration for selected entry */
    *(mas0Value) = (uint32_t)MFSPR(MPU_E200_MAS0_SPRN);
    *(mas1Value) = (uint32_t)MFSPR(MPU_E200_MAS1_SPRN);
    *(mas2Value) = (uint32_t)MFSPR(MPU_E200_MAS2_SPRN);
    *(mas3Value) = (uint32_t)MFSPR(MPU_E200_MAS3_SPRN);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_ConfigRegion
 * Description   : Checks to see if the region ID is valid for selected region
 * type and configures the bit fields accordingly.
 *
 *END**************************************************************************/
static status_t MPU_E200_ConfigRegion(uint8_t regionId,
                                    mpu_e200_region_t regionType,
                                    uint32_t *reg)
{
    status_t result = STATUS_SUCCESS;
    switch(regionType)
    {
        case MPU_E200_DATA_REGION:
            /* Data Region configuration */
            if(regionId < MPU_E200_MAX_DATA_REGIONS)
            {
                (*reg) |= MPU_E200_MAS0_INST(0U)
                        | MPU_E200_MAS0_SHD(0U)
                        | MPU_E200_MAS0_ESEL(regionId);
            }
            else
            {
                result = STATUS_ERROR;
            }
            break;
        case MPU_E200_SHARED_DATA_REGION:
            /* Shared Data Region configuration */
            if(regionId < MPU_E200_MAX_SHARED_REGIONS)
            {
                (*reg) |= MPU_E200_MAS0_INST(0U)
                        | MPU_E200_MAS0_SHD(1U)
                        | MPU_E200_MAS0_ESEL(regionId);
            }
            else
            {
                result = STATUS_ERROR;
            }
            break;
        case MPU_E200_EXEC_REGION:
            /* Instruction Region configuration */
            if(regionId < MPU_E200_MAX_EXEC_REGIONS)
            {
                (*reg) |= MPU_E200_MAS0_INST(1U)
                        | MPU_E200_MAS0_SHD(0U)
                        | MPU_E200_MAS0_ESEL(regionId);
            }
            else
            {
                result = STATUS_ERROR;
            }
            break;
        case MPU_E200_SHARED_EXEC_REGION:
            /* Shared Instruction Region configuration */
            if(regionId < MPU_E200_MAX_SHARED_REGIONS)
            {
                (*reg) |= MPU_E200_MAS0_INST(1U)
                        | MPU_E200_MAS0_SHD(1U)
                        | MPU_E200_MAS0_ESEL(regionId);
            }
            else
            {
                result = STATUS_ERROR;
            }
            break;
        default:
            /* Unsupported value */
            result = STATUS_ERROR;
            break;
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_Invalidate
 * Description   : Initiate a hardware MPU invalidation and waits until it is
 * finished. This function clears the access rights and debug configuration
 * of the MPU.
 *
 *END**************************************************************************/
void MPU_E200_Invalidate(void)
{
    /* Write the MPU Flash Invalidate bit to start the hardware invalidation process */
    MTSPR(MPU_E200_MPU0CSR0_SPRN, MPU_E200_CSR_MPUFI(1U));
    /* Wait for the invalidation process to finish and CSR register to be 0x0UL */
    csr0 = (uint32_t)MFSPR(MPU_E200_MPU0CSR0_SPRN);
    while(csr0 != 0UL)
    {
        csr0 = (uint32_t)MFSPR(MPU_E200_MPU0CSR0_SPRN);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_ConfigEntry
 * Description   : Configures the entry ID to the MPU Assist Register 0 and
 * transfers the configuration into the MPU Assist Registers.
 *
 *END**************************************************************************/
status_t MPU_E200_ConfigEntry(const mpu_e200_entry_cfg_t *entryCfg)
{
    status_t result = STATUS_SUCCESS;

    /* Initialize MPU MAS register values */
    mas0 = 0x00000030UL;
    mas1 = 0x00000000UL;
    mas2 = 0x00000000UL;
    mas3 = 0x00000000UL;

    /*
        * Configure Invalidation and debug behaviour as well as the
        * read-only property.
        */
    mas0 |= MPU_E200_MAS0_VALID(1U)
            | MPU_E200_MAS0_IPROT(entryCfg->invalidProtection)
            | MPU_E200_MAS0_SEL(2U)
            | MPU_E200_MAS0_RO(entryCfg->readOnly)
            | MPU_E200_MAS0_DEBUG(entryCfg->debug);
    /* Configure the entry as a data or instruction region */
    result = MPU_E200_ConfigRegion(entryCfg->regionId,
                                    entryCfg->regionType, &mas0);

    if(result == STATUS_SUCCESS)
    {
        /* Check Address Mask Control for out of range values */
        if(entryCfg->addressMask < MPU_E200_MAX_ADDRESS_MASK)
        {
            mas0 |= MPU_E200_MAS0_UAMSK(entryCfg->addressMask);
        }
        else
        {
            result = STATUS_ERROR;
        }
    }

    if(result == STATUS_SUCCESS)
    {
        /*
         * Configure access rights for user & supervisor
         * cache inhibit behaviour and guard behaviour
         */
        mas0 |= MPU_E200_MAS0_UW((uint8_t)entryCfg->userRights >> 2U)
              | MPU_E200_MAS0_UXR(entryCfg->userRights)
              | MPU_E200_MAS0_SW((uint8_t)entryCfg->supervisorRights >> 2U)
              | MPU_E200_MAS0_SXR(entryCfg->supervisorRights)
              | MPU_E200_MAS0_IOVR(entryCfg->cacheInhibitOverride)
              | MPU_E200_MAS0_GOVR(entryCfg->guardOverride)
              | MPU_E200_MAS0_I(entryCfg->cacheInhibit)
              | MPU_E200_MAS0_G(entryCfg->guard);
        /*
         * Configure region ID bits used to compare against process ID
         * and comparison behaviour
         */
        mas1 |= MPU_E200_MAS1_TID(entryCfg->tid)
              | MPU_E200_MAS1_TIDMSK(entryCfg->tidMask);
        /*
         * Write the configuration to the MPU entry
         */
        MPU_E200_WriteEntry(mas0, mas1, entryCfg->endAddress,
                            entryCfg->startAddress);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_ConfigModule
 * Description   : Configures the Core MPU module and enables the MPU
 *
 *END**************************************************************************/
status_t MPU_E200_ConfigModule(const mpu_e200_module_cfg_t *mpuCfg)
{
    status_t result = STATUS_SUCCESS;

    /* Initialize MPU CSR0 register value */
    csr0 = 0x00000000UL;
    /* Check Bypass and Debug parameters for unsupported value */
    if ((mpuCfg->userBypass > MPU_E200_BYPASS_ALL)
        || (mpuCfg->supervisorBypass > MPU_E200_BYPASS_ALL)
        || (mpuCfg->debugOptions > MPU_E200_DEBUG_ALL))
    {
        result = STATUS_ERROR;
    }
    else
    {
        /* Configure Control and Status Register value */
        csr0 |= MPU_E200_CSR_BYPS(mpuCfg->supervisorBypass)
              | MPU_E200_CSR_BYPU(mpuCfg->userBypass)
              | MPU_E200_CSR_DEN(mpuCfg->debugOptions)
              | MPU_E200_CSR_TIDCTL(mpuCfg->tidControl)
              | MPU_E200_CSR_MPUEN(1U);
        /* Write the config to the CSR register */
        MPU_E200_SetCSR0(csr0);
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_GetEntryCfg
 * Description   : Gets the configuration of an entry from the MPU
 *
 *END**************************************************************************/
status_t MPU_E200_GetEntryCfg(const uint8_t regionId,
                            const mpu_e200_region_t regionType,
                            mpu_e200_entry_cfg_t *entryCfg)
{
    status_t result = STATUS_SUCCESS;

    /* Initialize MPU MAS register values */
    mas0 = 0x00000030UL | MPU_E200_MAS0_SEL(0x2U);
    mas1 = 0x00000000UL;
    mas2 = 0x00000000UL;
    mas3 = 0x00000000UL;

    /* Check if region ID is valid and configure MAS0 for entry retrieval */
    result = MPU_E200_ConfigRegion(regionId, regionType, &mas0);

    if (result == STATUS_SUCCESS)
    {
        /* Read the entry config from the MPU */
        MPU_E200_ReadEntry(mas0, &mas0, &mas1, &mas2, &mas3);
        /*
         * Check to see if this entry has a valid configuration
         * or if it has been configured already.
         */
        if(MPU_E200_MAS0_VALID_GET(mas0) == 0UL)
        {
            result = STATUS_ERROR;
        }
    }
    if (result == STATUS_SUCCESS)
    {
        /* Extract configuration from the MPU Assist Registers */
        entryCfg->invalidProtection    = (bool)MPU_E200_MAS0_IPROT_GET(mas0);
        entryCfg->readOnly             = (bool)MPU_E200_MAS0_RO_GET(mas0);
        entryCfg->debug                = (bool)MPU_E200_MAS0_DEBUG_GET(mas0);
        entryCfg->regionId          = (uint8_t)MPU_E200_MAS0_ESEL_GET(mas0);
        entryCfg->addressMask       = (uint8_t)MPU_E200_MAS0_UAMSK_GET(mas0);
        entryCfg->cacheInhibitOverride = (bool)MPU_E200_MAS0_IOVR_GET(mas0);
        entryCfg->guardOverride        = (bool)MPU_E200_MAS0_GOVR_GET(mas0);
        entryCfg->cacheInhibit         = (bool)MPU_E200_MAS0_I_GET(mas0);
        entryCfg->guard                = (bool)MPU_E200_MAS0_G_GET(mas0);
        entryCfg->tid               = (uint8_t)MPU_E200_MAS1_TID_GET(mas1);
        entryCfg->tidMask           = (uint8_t)MPU_E200_MAS1_TIDMSK_GET(mas1);
        /* Get access rights from the MPU Assist Registers */
        entryCfg->userRights           = (mpu_e200_access_rights_t)
            ((MPU_E200_MAS0_UW_GET(mas0) << 2U) | MPU_E200_MAS0_UXR_GET(mas0));
        entryCfg->supervisorRights     = (mpu_e200_access_rights_t)
            ((MPU_E200_MAS0_SW_GET(mas0) << 2U) | MPU_E200_MAS0_SXR_GET(mas0));
        /* Get the region type */
        if(MPU_E200_MAS0_SHD_GET(mas0) == 1UL)
        {
            if(MPU_E200_MAS0_INST_GET(mas0) == 1UL)
            {
                entryCfg->regionType = MPU_E200_SHARED_EXEC_REGION;
            }
            else
            {
                entryCfg->regionType = MPU_E200_SHARED_DATA_REGION;
            }
        }
        else
        {
            if(MPU_E200_MAS0_INST_GET(mas0) == 1UL)
            {
                entryCfg->regionType = MPU_E200_EXEC_REGION;
            }
            else
            {
                entryCfg->regionType = MPU_E200_DATA_REGION;
            }
        }
        /* Get configured region boundaries */
        entryCfg->endAddress = mas2;
        entryCfg->startAddress = mas3;
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_GetModuleCfg
 * Description   : Gets the current configuration of the MPU module
 *
 *END**************************************************************************/
void MPU_E200_GetModuleCfg(mpu_e200_module_cfg_t *mpuCfg)
{
    /* Initialize MPU CSR0 register value */
    csr0 = 0x00000000UL;
    /* Read the CSR0 from the special purpose register */
    csr0 = MPU_E200_GetCSR0();
    /* Extract configuration from the CSR0 */
    mpuCfg->supervisorBypass = (uint8_t)MPU_E200_CSR_BYPS_GET(csr0);
    mpuCfg->userBypass       = (uint8_t)MPU_E200_CSR_BYPU_GET(csr0);
    mpuCfg->debugOptions     = (uint8_t)MPU_E200_CSR_DEN_GET(csr0);
    mpuCfg->tidControl       = (bool)MPU_E200_CSR_TIDCTL_GET(csr0);
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
