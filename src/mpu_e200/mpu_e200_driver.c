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
 * @file mpu_e200_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined to be used by the application code.
 */

#include <stddef.h>
#include "mpu_e200_hw_access.h"

/*******************************************************************************
 * Code
 *******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_DRV_Init
 * Description   : This function initializes the Core MPU and configures the
 * region entries. It enables the MPU module.
 *
 * Implements    : MPU_E200_DRV_Init_Activity
 * END**************************************************************************/
status_t MPU_E200_DRV_Init(const mpu_e200_module_cfg_t *mpuConfig,
                        const mpu_e200_entry_cfg_t *entriesConfig,
                        const uint8_t entriesCount)
{
    /* Check received parameters */
    DEV_ASSERT(mpuConfig != NULL);
    DEV_ASSERT(entriesConfig != NULL);
    DEV_ASSERT((entriesCount > 0U) && (entriesCount < 25U));

    status_t result = STATUS_SUCCESS;
    uint32_t regionSize = 0U;
    uint8_t index = 0U;

    /* Check that MPU module is not already enabled */
    if((MPU_E200_GetCSR0() & MPU_E200_CSR_MPUEN_MASK) == 1U)
    {
        result = STATUS_ERROR;
    }

    /* Configure all regions from the configuration structure */
    for(index = 0U; (index < entriesCount) && (result == STATUS_SUCCESS); index++)
    {
        /* Check if regions' boundaries are valid */
        if(entriesConfig[index].startAddress > entriesConfig[index].endAddress)
        {
            result = STATUS_ERROR;
        }

        if (result == STATUS_SUCCESS)
        {
            /* Check size of the region to be protected */
            regionSize = entriesConfig[index].endAddress - entriesConfig[index].startAddress;
            if (regionSize == 0U)
            {
                /* Zero region config have to be skipped */
                continue;
            }
            else if (regionSize < MPU_E200_MIN_REGION_SIZE)
            {
                result = STATUS_ERROR;
            }
            else
            {
                result = MPU_E200_ConfigEntry(&(entriesConfig[index]));
            }
        }
    }

    /* Configure and start Core MPU module */
    if(result == STATUS_SUCCESS)
    {
        result = MPU_E200_ConfigModule(mpuConfig);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_DRV_DeInit
 * Description   : This function de-initializes the core MPU and invalidates all
 * entries that are not protected.
 *
 * Implements    : MPU_E200_DRV_DeInit_Activity
 * END**************************************************************************/
void MPU_E200_DRV_DeInit(void)
{
    /* Invalidate and disable MPU */
    MPU_E200_Invalidate();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_DRV_GetEntryConfig
 * Description   : This function reads the requested entry from the MPU and
 * returns the configuration as a structure.
 *
 * Implements    : MPU_E200_DRV_GetEntryConfig_Activity
 * END**************************************************************************/
status_t MPU_E200_DRV_GetEntryConfig(const uint8_t regionId,
                                    const mpu_e200_region_t regionType,
                                    mpu_e200_entry_cfg_t *entryConfig)
{
    /* Check the pointer to the entry config structure */
    DEV_ASSERT(entryConfig != NULL);
    /* Request the selected entry */
    return MPU_E200_GetEntryCfg(regionId, regionType, entryConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_DRV_GetModuleConfig
 * Description   : This function reads the current configuration of the MPU module
 *
 * Implements    : MPU_E200_DRV_GetModuleConfig_Activity
 * END**************************************************************************/
void MPU_E200_DRV_GetModuleConfig(mpu_e200_module_cfg_t *mpuCfg)
{
    /* Check the pointer to the module config structure */
    DEV_ASSERT(mpuCfg != NULL);
    /* Get the configuration */
    MPU_E200_GetModuleCfg(mpuCfg);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MPU_E200_DRV_AddEntry
 * Description   : This function adds a new entry into MPU.
 *
 * Implements    : MPU_E200_DRV_AddEntry_Activity
 * END**************************************************************************/
status_t MPU_E200_DRV_AddEntry(const mpu_e200_entry_cfg_t *entryConfig)
{
    /* Check the pointer to the entry config structure */
    DEV_ASSERT(entryConfig != NULL);

    status_t result = STATUS_SUCCESS;
    /* Check address range */
    if ((entryConfig->endAddress - entryConfig->startAddress) < MPU_E200_MIN_REGION_SIZE)
    {
        result = STATUS_ERROR;
    }
    else
    {
        /* Try and add a new entry */
        result = MPU_E200_ConfigEntry(entryConfig);
    }
    return result;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
