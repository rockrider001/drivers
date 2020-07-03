/*
 * Copyright 2016-2019 NXP
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

#ifndef POWER_MANAGER_MPC57XX_H
#define POWER_MANAGER_MPC57XX_H

/*!
 * @file power_manager_MPC57xx.h
 */

#include "device_registers.h"
#include "status.h"

/*!
 * @ingroup power_manager
 * @defgroup power_mpc57xx
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Power modes enumeration.
 *
 * Defines power mode. Used in the power mode configuration structure
 * (power_manager_user_config_t).
 * List of power modes supported by specific chip along with requirements for entering
 * and exiting of these modes can be found in chip documentation.
 * List of all supported power modes:\n
 *  \li POWER_MANAGER_RESET - RESET (system mode)
 *  \li POWER_MANAGER_TEST - TEST (system mode)
 *  \li POWER_MANAGER_SAFE - SAFE (system mode)
 *  \li POWER_MANAGER_DRUN - DRUN (system mode)
 *  \li POWER_MANAGER_RUN0 - RUN0 (user mode)
 *  \li POWER_MANAGER_RUN1 - RUN1 (user mode)
 *  \li POWER_MANAGER_RUN2 - RUN2 (user mode)
 *  \li POWER_MANAGER_RUN3 - RUN3 (user mode)
 *  \li POWER_MANAGER_HALT0 - HALT0 (user mode)
 *  \li POWER_MANAGER_STOP0 - STOP0 (user mode)
 *  \li POWER_MANAGER_STANDBY0 - STANDBY0 (user mode)
 *  Implements power_manager_modes_t_Class
 */
typedef enum
{
    POWER_MANAGER_RESET                = 0U,             /*!< FUNCTIONAL RESET */
#if FEATURE_MC_ME_HAS_TEST_MODE
    POWER_MANAGER_TEST                 = 1U,             /*!< TEST (system mode) */
#endif
    POWER_MANAGER_SAFE                 = 2U,             /*!< SAFE (system mode) */
    POWER_MANAGER_DRUN                 = 3U,             /*!< DRUN (system mode) */
    POWER_MANAGER_RUN0                 = 4U,             /*!< RUN0 (user mode) */
    POWER_MANAGER_RUN1                 = 5U,             /*!< RUN1 (user mode) */
    POWER_MANAGER_RUN2                 = 6U,             /*!< RUN2 (user mode) */
    POWER_MANAGER_RUN3                 = 7U,             /*!< RUN3 (user mode) */
#if FEATURE_MC_ME_HAS_HALT_MODE
    POWER_MANAGER_HALT0                = 8U,             /*!< HALT0 (user mode) */
#endif
    POWER_MANAGER_STOP0                = 10U,            /*!< STOP0 (user mode) */
#if FEATURE_MC_ME_HAS_STANDBY_MODE
    POWER_MANAGER_STANDBY0             = 13U,            /*!< STANDBY0 (user mode) */
#endif
} power_manager_modes_t;

/*!
 * @brief Power level enumeration.
 *
 * Indicates the relative power consumption level of a mode with respect to that of other modes.
 * When switching between two modes with differing power levels, system clock progressive switching is enabled.
 * Used in the power mode configuration structure (power_manager_user_config_t).
 * Implements mc_me_power_level_t_Class
 */
typedef enum
{
    MC_ME_PWRLVL_0,                                  /*!< Power Level 0 */
    MC_ME_PWRLVL_1,                                  /*!< Power Level 1 */
    MC_ME_PWRLVL_2,                                  /*!< Power Level 2 */
    MC_ME_PWRLVL_3,                                  /*!< Power Level 3 */
    MC_ME_PWRLVL_4,                                  /*!< Power Level 4 */
    MC_ME_PWRLVL_5,                                  /*!< Power Level 5 */
    MC_ME_PWRLVL_6,                                  /*!< Power Level 6 */
    MC_ME_PWRLVL_7                                   /*!< Power Level 7 */
} mc_me_power_level_t;

#if FEATURE_MC_ME_HAS_FLAON_CONFIG
/*!
 * @brief Flash power-down control enum.
 * Implements mc_me_flash_mode_t_Class
 */
typedef enum
{
#if FEATURE_MC_ME_HAS_FLAON_PD_MODE
    MC_ME_FLASH_POWER_DOWN_MODE = 0x1U,             /*!< Flash is in power-down mode */
#endif
#if FEATURE_MC_ME_HAS_FLAON_LP_MODE
    MC_ME_FLASH_LOW_POWER_MODE = 0x2U,              /*!< Flash is in low power mode */
#endif
    MC_ME_FLASH_RUN_MODE        = 0x3U              /*!< Flash is in RUN mode */
} mc_me_flash_mode_t;
#endif /* FEATURE_MC_ME_HAS_FLAON_CONFIG */
/*!
 * @brief Run mode configuration structure
 * Implements power_manager_user_config_t_Class
 */
typedef struct
{
    power_manager_modes_t   powerMode;      /*!< Power mode (enum), see power_manager_modes_t */
    mc_me_power_level_t     powerLevel;     /*!< Relative power level indicator (enum), see mc_me_power_level_t */
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
    mc_me_flash_mode_t      flashMode;      /*!< Flash power-down control, see mc_me_flash_mode_t */
#endif /* FEATURE_MC_ME_HAS_FLAON_CONFIG */
    bool                    outputPowerdown;/*!< Output power-down control */
    bool                    mainVoltage;    /*!< Main voltage regulator control */
} power_manager_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief This function implementation-specific configuration of power modes.
 *
 * This function performs the actual implementation-specific initialization based on the provided power mode configurations.
 *
 * @return Operation status
 *        - STATUS_SUCCESS: Operation was successful.
 */
status_t POWER_SYS_DoInit(void);

/*!
 * @brief This function implementation-specific de-initialization of power manager.
 *
 * This function performs the actual implementation-specific de-initialization.
 *
 * @return Operation status
 *        - STATUS_SUCCESS: Operation was successful.
 */
status_t POWER_SYS_DoDeinit(void);

/*!
 * @brief This function configures the power mode.
 *
 * This function performs the actual implementation-specific logic to switch to one of the defined power modes.
 *
 * @return Operation status
 *        - STATUS_SUCCESS: Operation was successful.
 *        - STATUS_MCU_TRANSITION_FAILED: Operation failed.
 *        - STATUS_UNSUPPORTED: The function do not support the mode.
 */
status_t POWER_SYS_DoSetMode(const power_manager_user_config_t * const configPtr);

/*!
 * @brief This function get previous mode.
 *
 * This function will return previous mode where the chip jump 
 * from lastest mode to current mode. The function can not return STANDBY mode
 * because register control is disabled.
 *
 */
power_manager_modes_t POWER_SYS_GetPreviousMode(void);

/*!
 * @brief Gets the default power_manager configuration structure.
 *
 * This function gets the power_manager configuration structure of the default power mode.
 *
 * @param[out] defaultConfig Pointer to power mode configuration structure of the default power mode.
 */
static inline void POWER_SYS_DoGetDefaultConfig(power_manager_user_config_t * const defaultConfig)
{
    defaultConfig->powerMode = POWER_MANAGER_DRUN;   /*!< Power manager mode  */
    defaultConfig->powerLevel = MC_ME_PWRLVL_0;      /*!< Relative power level indicator */
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
    defaultConfig->flashMode = MC_ME_FLASH_RUN_MODE; /*!< Flash power-down control */
#endif
    defaultConfig->outputPowerdown = false;          /*!< Output power-down control */
    defaultConfig->mainVoltage = true;               /*!< Main voltage regulator control */
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* POWER_MANAGER_MPC57XX_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
