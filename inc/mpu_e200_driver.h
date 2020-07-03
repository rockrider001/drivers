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
 * @file mpu_e200_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * This macro is used by user.
 */
#ifndef MPU_E200_DRIVER_H
#define MPU_E200_DRIVER_H

#include "status.h"
#include "device_registers.h"
#include <stdbool.h>

/*!
 * @addtogroup mpu_e200_drv MPU for E200
 * @ingroup mpu_e200
 * @brief Core Memory Protection Unit for E200
 * @{
 *******************************************************************************
 * Definitions
 ******************************************************************************/
#define MPU_E200_BYPASS_NONE  (0x0U)  /*!< No bypass acceptable */
#define MPU_E200_BYPASS_EXEC  (0x1U)  /*!< Instruction protection can be bypassed */
#define MPU_E200_BYPASS_WRITE (0x2U)  /*!< Write data protection can be bypassed */
#define MPU_E200_BYPASS_READ  (0x4U)  /*!< Read data protection can be bypassed */
#define MPU_E200_DEBUG_NONE   (0x0U)  /*!< No debug event can be generated */
#define MPU_E200_DEBUG_EXEC   (0x1U)  /*!< Instruction access can generate debug events */
#define MPU_E200_DEBUG_WRITE  (0x2U)  /*!< Write data access can generate debug events */
#define MPU_E200_DEBUG_READ   (0x4U)  /*!< Read data access can generate debug events */

/*******************************************************************************
 * Enumerations
 ******************************************************************************/
/*!
 * @brief Core MPU region types options
 * Implements : mpu_e200_region_t_Class
 */
typedef enum
{
    MPU_E200_DATA_REGION        = 0x1U, /*!< The region is used for data */
    MPU_E200_EXEC_REGION        = 0x2U, /*!< The region is used for instruction */
    MPU_E200_SHARED_DATA_REGION = 0x3U, /*!< The shared region is used for data */
    MPU_E200_SHARED_EXEC_REGION = 0x4U  /*!< The shared region is used for instruction */
} mpu_e200_region_t;

/*!
 * @brief Core MPU access rights options
 * Implements : mpu_e200_access_rights_t_Class
 */
typedef enum
{
    MPU_E200_ACCESS_NONE        = 0x0U, /*!< No access is allowed */
    MPU_E200_ACCESS_READ        = 0x1U, /*!< Instruction or data read is allowed */
    MPU_E200_ACCESS_WRITE       = 0x4U, /*!< Write data is allowed */
    MPU_E200_ACCESS_READWRITE   = 0x5U  /*!< Instruction or data read and Write data is allowed */
} mpu_e200_access_rights_t;

/*!
 * @brief Core MPU entry configuration structure
 * Implements : mpu_e200_entry_cfg_t_Class
 */
typedef struct
{
    bool                        invalidProtection;    /*!< Invalidation protection */
    bool                        readOnly;             /*!< Read-Only attribute for entry */
    bool                        debug;                /*!< Debug event generation behaviour */
    mpu_e200_region_t           regionType;           /*!< Entry region type */
    uint8_t                     regionId;             /*!< Entry region ID */
    uint8_t                     addressMask;          /*!< Masking of upper address bits */
    mpu_e200_access_rights_t    userRights;           /*!< User region access rights */
    mpu_e200_access_rights_t    supervisorRights;     /*!< Supervisor region access rights */
    bool                        cacheInhibitOverride; /*!< Override for cache-inhibit attribute */
    bool                        guardOverride;        /*!< Override for guard attribute */
    bool                        cacheInhibit;         /*!< Cache Inhibit attribute for entry */
    bool                        guard;                /*!< Guard attribute for data regions */
    uint8_t                     tid;                  /*!< Region ID bits */
    uint8_t                     tidMask;              /*!< Masking support of TID bits */
    uint32_t                    startAddress;         /*!< Lower bound of memory region */
    uint32_t                    endAddress;           /*!< Upper bound of memory region */
} mpu_e200_entry_cfg_t;

/*!
 * @brief Core MPU configuration structure
 * Implements : mpu_e200_module_cfg_t_Class
 */
typedef struct
{
    uint8_t     supervisorBypass;   /*!< Bypass options for supervisor mode */
    uint8_t     userBypass;         /*!< Bypass options for user mode */
    uint8_t     debugOptions;       /*!< Debug event generation behaviour */
    bool        tidControl;         /*!< TID usage control */
} mpu_e200_module_cfg_t;

/*******************************************************************************
 * API
 ******************************************************************************/
 /*!
 * @name Core MPU DRV for E200.
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief This function initializes the Core MPU and configures the
 * region entries. It enables the MPU module.
 *
 * @param[in] mpuConfig Pointer to the MPU module configuration structure
 * @param[in] entriesConfig Array containing all entries configuration structures
 * @param[in] entriesCount Number of entries in the array
 * @return Status of operation:
 *          -STATUS_SUCCES: MPU successfully configured and started
 *          -STATUS_ERROR: One of the entries could not be configured
 *          -STATUS_ERROR: MPU module could not be configured
 */
status_t MPU_E200_DRV_Init(const mpu_e200_module_cfg_t *mpuConfig,
                        const mpu_e200_entry_cfg_t *entriesConfig,
                        const uint8_t entriesCount);

/*!
 * @brief This function de-initializes the core MPU and invalidates all
 * entries that are not protected.
 */
void MPU_E200_DRV_DeInit(void);

/*!
 * @brief This function reads the requested entry from the MPU and
 * returns the configuration as a structure.
 *
 * @param[in] regionID Entry number of the region
 * @param[in] regionType Region type of the entry
 * @param[out] entryConfig Pointer to where the selected entry config will be stored
 * @return Status of operation:
 *          -STATUS_SUCCES: entry configuration retrieved successfully
 *          -STATUS_ERROR: entry could not be retrieved
 */
status_t MPU_E200_DRV_GetEntryConfig(const uint8_t regionId,
                                    const mpu_e200_region_t regionType,
                                    mpu_e200_entry_cfg_t *entryConfig);

/*!
 * @brief This function reads the current configuration of the MPU module
 *
 * @param[out] mpuCfg Pointer to the MPU module structure to store config
 */
void MPU_E200_DRV_GetModuleConfig(mpu_e200_module_cfg_t *mpuCfg);

/*!
 * @brief This function reads the current configuration of the MPU module
 *
 * @param[in] entriesConfig Pointer to the entry configuration structure
 * @return Status of operation:
 *          -STATUS_SUCCES: new entry configured successfully
 *          -STATUS_ERROR: new entry could not be configured
 */
status_t MPU_E200_DRV_AddEntry(const mpu_e200_entry_cfg_t *entryConfig);

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* MPU_E200_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

