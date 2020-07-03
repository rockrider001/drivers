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
 * @file mpu_e200_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined
 * Macro is used to allow the use of assembly module specific instructions
 */
#ifndef MPU_E200_HW_ACCESS_H
#define MPU_E200_HW_ACCESS_H

#include "mpu_e200_driver.h"

/*!
 * @brief Core Memory Protection Unit for E200 Hardware Access layer.
 * @{
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Core MPU Read entry instruction */
#define MPU_E200_MPURE()    PPCASM("mpure")
/* Core MPU Write entry instruction */
#define MPU_E200_MPUWE()    PPCASM("mpuwe")
/* Core MPU Synchronize instruction */
#define MPU_E200_MPUSYNC()  PPCASM("mpusync")

/*******************************************************************************
 * API
 *******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Writes the Core MPU configuration
 *
 * @param[in] mpuConfig The Core MPU CSR register value.
 */
static inline void MPU_E200_SetCSR0(uint32_t mpuConfig)
{
    MTSPR(MPU_E200_MPU0CSR0_SPRN, mpuConfig);
}

/*!
 * @brief Reads the Core MPU configuration
 *
 * @return The Core MPU CSR register value.
 */
static inline uint32_t MPU_E200_GetCSR0(void)
{
    return (uint32_t)MFSPR(MPU_E200_MPU0CSR0_SPRN);
}

/*!
 * @brief Initiate a hardware MPU invalidation and waits until it is finished.
 * This function clears the access rights and debug configuration of the MPU.
 */
void MPU_E200_Invalidate(void);

/*!
 * @brief Configures the entry ID to the MPU Assist Register 0 and reads
 * from the MPU the configuration into the MPU Assist Registers.
 *
 * @param[in] entryCfg Pointer to the configuration structure of an entry
 * @return Status of operation:
 *          -STATUS_SUCCES: entry has been successfully configured
 *          -STATUS_ERROR: region boundaries are not correct
 *          -STATUS_ERROR: region size is too big/small
 *          -STATUS_ERROR: region ID is not supported
 *          -STATUS_ERROR: address mask control is too big
 */
status_t MPU_E200_ConfigEntry(const mpu_e200_entry_cfg_t *entryCfg);

/*!
 * @brief Configures the Core MPU module and enables the MPU
 *
 * @param[in] mpuCfg Pointer to the configuration structure of the MPU module
 * @return Status of operation:
 *          -STATUS_SUCCES: module has been successfully configured
 *          -STATUS_ERROR: user bypass config is incorrect
 *          -STATUS_ERROR: supervisor bypass config is incorrect
 *          -STATUS_ERROR: debug behaviour config is incorrect
 */
status_t MPU_E200_ConfigModule(const mpu_e200_module_cfg_t *mpuCfg);

/*!
 * @brief Gets the configuration of an entry from the MPU
 *
 * @param[in] regionID Entry number of the region
 * @param[in] regionType Region type of the entry
 * @param[out] entryConfig Pointer to where the selected entry config will be stored
 * @return Status of operation:
 *          -STATUS_SUCCES: entry configuration retrieved successfully
 *          -STATUS_ERROR: region ID is not supported
 *          -STATUS_ERROR: the entry selected does not contain a valid configuration
 */
status_t MPU_E200_GetEntryCfg(const uint8_t regionId,
                            const mpu_e200_region_t regionType,
                            mpu_e200_entry_cfg_t *entryCfg);

/*!
 * @brief Gets the current configuration of the MPU module
 *
 * @param[in] mpuCfg Pointer to where the module config will be stored
 */
void MPU_E200_GetModuleCfg(mpu_e200_module_cfg_t *mpuCfg);

/*! @} */
#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* MPU_E200_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
