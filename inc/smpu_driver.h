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
 * @file smpu_driver.h
 */

#ifndef SMPU_DRIVER_H
#define SMPU_DRIVER_H

#include "status.h"
#include "device_registers.h"

/*!
 * @defgroup smpu_drv SMPU Driver
 * @ingroup smpu
 * @brief System Memory Protection Unit Peripheral Driver.
 * @addtogroup smpu_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/*!
 * @brief Lock configuration.
 * Implements : smpu_lock_t_Class
 */
typedef enum
{
    SMPU_UNLOCK     = 0U,    /*!< Unlocked */
#if FEATURE_SMPU_HAS_OWNER_LOCK
    SMPU_OWNER_LOCK = 1U,    /*!< Locked by the master wrote this register and LCK bit
                                  Attempted writes by other masters are ignored */
#endif
    SMPU_ALL_LOCK   = 3U     /*!< Attempted writes to any location in the region descriptor are ignored */
} smpu_lock_t;

/*!
 * @brief Region lock configuration structure.
 * Implements : smpu_region_lock_t_Class
 */
typedef struct
{
    uint8_t     regionNum;      /*!< Region number */
#if FEATURE_SMPU_HAS_OWNER_LOCK
    uint8_t     masterOwner;    /*!< Master number */
#endif
    smpu_lock_t lockConfig;     /*!< Lock configuration */
} smpu_region_lock_t;

#if FEATURE_SMPU_SUPPORT_GETTING_ERROR_DETAIL
/*!
 * @brief Error access type.
 * Implements : smpu_err_access_type_t_Class
 */
typedef enum
{
    SMPU_ERR_TYPE_READ  = 0U,    /*!< Error type: read */
    SMPU_ERR_TYPE_WRITE = 1U     /*!< Error type: write */
} smpu_err_access_type_t;

/*!
 * @brief Error access attribute.
 * Implements : smpu_err_attributes_t_Class
 */
typedef enum
{
    SMPU_INSTRUCTION_ACCESS_IN_USER_MODE       = 0U,    /*!< Access instruction error in user mode */
    SMPU_DATA_ACCESS_IN_USER_MODE              = 1U,    /*!< Access data error in user mode */
    SMPU_INSTRUCTION_ACCESS_IN_SUPERVISOR_MODE = 2U,    /*!< Access instruction error in supervisor mode */
    SMPU_DATA_ACCESS_IN_SUPERVISOR_MODE        = 3U     /*!< Access data error in supervisor mode */
} smpu_err_attributes_t;
#endif

/*!
 * @brief Detail error access info structure.
 * Implements : smpu_access_err_info_t_Class
 */
typedef struct
{
    bool                   overrun;                    /*!< Access error master overrun */
#if FEATURE_SMPU_SUPPORT_GETTING_ERROR_DETAIL
    uint8_t                master;                     /*!< Access error master */
    smpu_err_attributes_t  attributes;                 /*!< Access error attributes */
    smpu_err_access_type_t accessType;                 /*!< Access error type */
    uint32_t               accessCtr;                  /*!< Access error control */
#endif
    uint32_t               addr;                       /*!< Access error address */
#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
    uint8_t                processorIdentification;    /*!< Access error processor identification */
#endif
} smpu_access_err_info_t;

#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
/*!
 * @brief Specific access rights.
 * Specifies separate access rights for supervisor mode (privilege) and user mode (unprivileged).
 * - Read (r) permission refers to the ability to access the referenced memory address
 * using an operand (data) fetch.
 * - Write (w) permission refers to the ability to update the referenced memory address
 * using a store (data) operation.
 * - Execute (x) permission refers to the ability to read the referenced memory address
 * using an instruction fetch
 * Implements : smpu_specific_access_rights_t_Class
 */
typedef enum
{
    SMPU_SUPERVISOR_NONE_USER_NONE = 0x00U,    /*!< All accesses are not allowed */
    SMPU_SUPERVISOR_NONE_USER_X    = 0x01U,    /*!< No access in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_NONE_USER_W    = 0x02U,    /*!< No access in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_NONE_USER_WX   = 0x03U,    /*!< No access in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_NONE_USER_R    = 0x04U,    /*!< No access in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_NONE_USER_RX   = 0x05U,    /*!< No access in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_NONE_USER_RW   = 0x06U,    /*!< No access in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_NONE_USER_RWX  = 0x07U,    /*!< No access in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_X_USER_NONE    = 0x08U,    /*!< Allow Execute in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_X_USER_X       = 0x09U,    /*!< Allow Execute in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_X_USER_W       = 0x0AU,    /*!< Allow Execute in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_X_USER_WX      = 0x0BU,    /*!< Allow Execute in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_X_USER_R       = 0x0CU,    /*!< Allow Execute in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_X_USER_RX      = 0x0DU,    /*!< Allow Execute in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_X_USER_RW      = 0x0EU,    /*!< Allow Execute in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_X_USER_RWX     = 0x0FU,    /*!< Allow Execute in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_W_USER_NONE    = 0x10U,    /*!< Allow Write in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_W_USER_X       = 0x11U,    /*!< Allow Write in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_W_USER_W       = 0x12U,    /*!< Allow Write in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_W_USER_WX      = 0x13U,    /*!< Allow Write in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_W_USER_R       = 0x14U,    /*!< Allow Write in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_W_USER_RX      = 0x15U,    /*!< Allow Write in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_W_USER_RW      = 0x16U,    /*!< Allow Write in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_W_USER_RWX     = 0x17U,    /*!< Allow Write in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_WX_USER_NONE   = 0x18U,    /*!< Allow Write, Execute in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_WX_USER_X      = 0x19U,    /*!< Allow Write, Execute in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_WX_USER_W      = 0x1AU,    /*!< Allow Write, Execute in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_WX_USER_WX     = 0x1BU,    /*!< Allow Write, Execute in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_WX_USER_R      = 0x1CU,    /*!< Allow Write, Execute in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_WX_USER_RX     = 0x1DU,    /*!< Allow Write, Execute in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_WX_USER_RW     = 0x1EU,    /*!< Allow Write, Execute in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_WX_USER_RWX    = 0x1FU,    /*!< Allow Write, Execute in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_R_USER_NONE    = 0x20U,    /*!< Allow Read in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_R_USER_X       = 0x21U,    /*!< Allow Read in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_R_USER_W       = 0x22U,    /*!< Allow Read in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_R_USER_WX      = 0x23U,    /*!< Allow Read in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_R_USER_R       = 0x24U,    /*!< Allow Read in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_R_USER_RX      = 0x25U,    /*!< Allow Read in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_R_USER_RW      = 0x26U,    /*!< Allow Read in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_R_USER_RWX     = 0x27U,    /*!< Allow Read in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_RX_USER_NONE   = 0x28U,    /*!< Allow Read, Execute in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_RX_USER_X      = 0x29U,    /*!< Allow Read, Execute in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_RX_USER_W      = 0x2AU,    /*!< Allow Read, Execute in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_RX_USER_WX     = 0x2BU,    /*!< Allow Read, Execute in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_RX_USER_R      = 0x2CU,    /*!< Allow Read, Execute in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_RX_USER_RX     = 0x2DU,    /*!< Allow Read, Execute in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_RX_USER_RW     = 0x2EU,    /*!< Allow Read, Execute in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_RX_USER_RWX    = 0x2FU,    /*!< Allow Read, Execute in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_RW_USER_NONE   = 0x30U,    /*!< Allow Read, Write in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_RW_USER_X      = 0x31U,    /*!< Allow Read, Write in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_RW_USER_W      = 0x32U,    /*!< Allow Read, Write in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_RW_USER_WX     = 0x33U,    /*!< Allow Read, Write in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_RW_USER_R      = 0x34U,    /*!< Allow Read, Write in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_RW_USER_RX     = 0x35U,    /*!< Allow Read, Write in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_RW_USER_RW     = 0x36U,    /*!< Allow Read, Write in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_RW_USER_RWX    = 0x37U,    /*!< Allow Read, Write in supervisor mode; Allow Read, Write, Execute in user mode */
    SMPU_SUPERVISOR_RWX_USER_NONE  = 0x38U,    /*!< Allow Read, Write, Execute in supervisor mode; No access in user mode */
    SMPU_SUPERVISOR_RWX_USER_X     = 0x39U,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Execute in user mode */
    SMPU_SUPERVISOR_RWX_USER_W     = 0x3AU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Write in user mode */
    SMPU_SUPERVISOR_RWX_USER_WX    = 0x3BU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Write, Execute in user mode */
    SMPU_SUPERVISOR_RWX_USER_R     = 0x3CU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Read in user mode */
    SMPU_SUPERVISOR_RWX_USER_RX    = 0x3DU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Read, Execute in user mode */
    SMPU_SUPERVISOR_RWX_USER_RW    = 0x3EU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Read, Write in user mode */
    SMPU_SUPERVISOR_RWX_USER_RWX   = 0x3FU,    /*!< Allow Read, Write, Execute in supervisor mode; Allow Read, Write, Execute in user mode */
} smpu_specific_access_rights_t;
#endif

/*!
 * @brief Access rights.
 * For normal mode:
 * - Read (r) permission refers to the ability to access the referenced memory address
 * using an operand (data) fetch or an instruction fetch.
 * - Write (w) permission refers to the ability to update the referenced memory address
 * using a store (data) operation.
 * For specific mode:
 * - NONE All accesses are not allowed.
 * - SET_1 Use set 1 in region configuration.
 * - SET_2 Use set 2 in region configuration.
 * - SET_3 Use set 3 in region configuration.
 * Implements : smpu_access_rights_t_Class
 */
typedef enum
{
    SMPU_NONE           = 0U,    /*!< All accesses are not allowed */
    SMPU_W_OR_SET_1     = 1U,    /*!< Write allowed, no Read; Specific access set 1 */
    SMPU_R_OR_SET_2     = 2U,    /*!< Read allowed, no Write; Specific access set 2 */
    SMPU_RW_OR_SET_3    = 3U,    /*!< Both Read and Write allowed; Specific access set 3 */
} smpu_access_rights_t;

/*!
 * @brief Master access rights structure.
 * Implements : smpu_master_access_right_t_Class
 */
typedef struct
{
    uint8_t              masterNum;      /*!< Master number */
    smpu_access_rights_t accessRight;    /*!< Access right */
} smpu_master_access_right_t;

/*!
 * @brief User region configuration structure.
 * Implements : smpu_user_config_t_Class
 */
typedef struct
{
    uint32_t                            startAddr;            /*!< Memory region start address */
    uint32_t                            endAddr;              /*!< Memory region end address */
#if (FEATURE_SMPU_HAS_SPECIFIC_ACCESS_RIGHT_COUNT != 0U)
    bool                                specAccessEnable;     /*!< Specific access enable */
    const smpu_specific_access_rights_t *specAccessSet;       /*!< Specific access configurations */
#endif
    const smpu_master_access_right_t    *masterAccRight;      /*!< Access permission for masters */
    bool                                cacheInhibitEnable;   /*!< Cache Inhibit */
#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
    bool                                processIdEnable;      /*!< Process identifier enable */
    uint8_t                             processIdentifier;    /*!< Process identifier */
    uint8_t                             processIdMask;        /*!< Process identifier mask */
#endif
    smpu_lock_t                         lockConfig;           /*!< Lock configuration */
} smpu_user_config_t;

/*******************************************************************************
 * API
 *******************************************************************************/
/*!
 * @name SMPU Driver API
 * @{
 */
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes system memory protection unit
 * by setting the access configurations of all available masters,
 * process identifier and the memory location for the given regions
 * and activate module finally.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionCnt The number of regions configured.
 * @param[in] userConfigPtr The pointer to user configuration structure, see #smpu_user_config_t.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to master number is out of range supported by hardware
 *                           or the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_Init(uint32_t instance,
                       uint8_t regionCnt,
                       const smpu_user_config_t * userConfigPtr);

/*!
 * @brief De-initializes system memory protection unit
 * by reseting all regions to default and disable module.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_Deinit(uint32_t instance);

/*!
 * @brief Gets default region configuration. Grants all access rights for masters;
 * disable PID and cache; unlock region descriptor.
 *
 * @param[out] masterAccRight The pointer to master configuration structure, see #smpu_master_access_right_t.
 *                            The length of array should be defined by number of masters supported by hardware.
 * @return The default region configuration, see #smpu_user_config_t.
 */
smpu_user_config_t SMPU_DRV_GetDefaultRegionConfig(smpu_master_access_right_t * masterAccRight);

/*!
 * @brief Configures the region descriptor.
 * Updates the access configuration of all available masters,
 * process identifier and memory location in a given region.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] regionConfigPtr The pointer to region configuration structure, see #smpu_user_config_t.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to master number is out of range supported by hardware
 *                           or the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_SetRegionConfig(uint32_t instance,
                                  uint8_t regionNum,
                                  const smpu_user_config_t * regionConfigPtr);

#if FEATURE_SMPU_HAS_PROCESS_IDENTIFIER
/*!
 * @brief Configures the region process identifier.
 * Please note that using this function will unlock the region descriptor.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] enable State
 *            - true  : Include PID in the region hit evaluation.
 *            - false : Do not include PID in the region hit evaluation.
 * @param[in] pid The process identifier.
 * @param[in] pidMask The process identifier mask.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_SetRegionProcessId(uint32_t instance,
                                     uint8_t regionNum,
                                     bool enable,
                                     uint8_t pid,
                                     uint8_t pidMask);
#endif

/*!
 * @brief Configures the region start and end address.
 * Please note that using this function will unlock the region descriptor.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] startAddr The region start address.
 * @param[in] endAddr The region end address.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_SetRegionAddr(uint32_t instance,
                                uint8_t regionNum,
                                uint32_t startAddr,
                                uint32_t endAddr);

/*!
 * @brief Enables/Disables region descriptor.
 * Please note that using this function will unlock the region descriptor.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] enable Valid state
 *            - true  : Enable region.
 *            - false : Disable region.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_EnableRegion(uint32_t instance,
                               uint8_t regionNum,
                               bool enable);

/*!
 * @brief Configures access permission of master in the region.
 * Please note that using this function will unlock the region descriptor.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] masterAccRight Pointer to master access right structure, see #smpu_master_access_right_t.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to master number is out of range supported by hardware
 *                           or the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_SetMasterAccessRights(uint32_t instance,
                                        uint8_t regionNum,
                                        const smpu_master_access_right_t * masterAccRight);

/*!
 * @brief Sets the region lock configuration.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] regionNum The region number.
 * @param[in] lockConfig The lock configuration
 *            - SMPU_UNLOCK     : Region unlocked.
 *            - SMPU_OWNER_LOCK : Region locked, only the master owner can modify the region descriptor.
 *            - SMPU_ALL_LOCK   : Region locked, all master cannot modify the region descriptor.
 * @return operation status
 *        - STATUS_SUCCESS : Operation was successful.
 *        - STATUS_ERROR   : Operation failed due to the region was locked by another master
 *                           or all masters are locked.
 */
status_t SMPU_DRV_SetRegionLockConfig(uint32_t instance,
                                      uint8_t regionNum,
                                      smpu_lock_t lockConfig);

/*!
 * @brief Reports the region lock status.
 *
 * @param[in] instance The SMPU peripheral instance number
 * @param[in] regionNum The region number.
 * @return The region lock status, see #smpu_region_lock_t.
 */
smpu_region_lock_t SMPU_DRV_GetRegionLockInfo(uint32_t instance,
                                              uint8_t regionNum);

/*!
 * @brief Checks and gets the access error detail information
 * if the error caused by master.
 *
 * @param[in] instance The SMPU peripheral instance number.
 * @param[in] errorNum The error channel number.
 * @param[out] errInfoPtr The pointer to access error info structure, see #smpu_access_err_info_t.
 * @return operation status
 *        - true  : An error has occurred.
 *        - false : No error has occurred.
 */
bool SMPU_DRV_GetDetailErrorInfo(uint32_t instance,
                                 uint8_t errorNum,
                                 smpu_access_err_info_t * errInfoPtr);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* SMPU_DRIVER_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
