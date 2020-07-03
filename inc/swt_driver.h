/*
 * Copyright 2017 NXP.
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
 * @file swt_driver.h
 */

#ifndef SWT_DRIVER_H
#define SWT_DRIVER_H

#include "status.h"
#include "device_registers.h"

/* */
/* */
/* */

/*!
 * @defgroup swt_drv SWT Driver
 * @ingroup swt
 * @brief Software Watchdog Timer Peripheral Driver.
 * @addtogroup swt_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/*!
 * @brief Lock configuration.
 * Implements : swt_lock_t_Class
 */
typedef enum
{
    SWT_UNLOCK     = 0x00U,    /*!< Unlock */
    SWT_SOFTLOCK   = 0x01U,    /*!< Soft lock */
    SWT_HARDLOCK   = 0x02U     /*!< Hard lock */
} swt_lock_t;

/*!
 * @brief Servicing modes.
 * Implements : swt_service_mode_t_Class
 */
typedef enum
{
#if FEATURE_SWT_SUPPORT_WATCHPOINT
    SWT_IA_EXE_MODE    = 0x03U,    /*!< Incremental Address Execution */
    SWT_FA_EXE_MODE    = 0x02U,    /*!< Fixed Address Execution */
#endif
    SWT_KS_SEQ_MODE    = 0x01U,    /*!< Keyed Service Sequence */
    SWT_FS_SEQ_MODE    = 0x00U     /*!< Fixed Service Sequence */
} swt_service_mode_t;

/*!
 * @brief SWT configuration structure.
 * Implements : swt_user_config_t_Class
 */
typedef struct
{
    uint8_t              mapConfig;       /*!< Master Access Protection for Master 0-7, LSB is Master 7 */
    bool                 invalidReset;    /*!< If true, reset request on invalid access when SWT is active */
#if FEATURE_SWT_HAS_CLOCK_SELECT
    bool                 clockSelect;     /*!< If true, the SWT is driven by the internal oscillator */
#endif
#if FEATURE_SWT_HAS_STOP_MODE
    bool                 stop;            /*!< If true, the SWT counter is stopped in stop mode */
#endif
    bool                 debug;           /*!< If true, the SWT counter is stopped in debug mode */
    bool                 winEnable;       /*!< If true, window mode is enabled */
    bool                 intEnable;       /*!< If true, an interrupt request is generated before reset */
    swt_service_mode_t   serviceMode;     /*!< Service Mode, see #swt_service_mode_t */
    uint32_t             timeoutValue;    /*!< The timeout value */
    uint32_t             windowValue;     /*!< The window value */
    uint16_t             initKey;         /*!< The initial service key */
    swt_lock_t           lockConfig;      /*!< Lock configuration, see #swt_lock_t */
} swt_user_config_t;

/*******************************************************************************
 * API
 *******************************************************************************/
/*!
 * @name SWT Driver API
 * @{
 */
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Locks SWT registers.
 * This function locks the SWT registers.
 * When locked, the SWT_CR, SWT_TO, SWT_WN and SWT_SK registers are read-only.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] lockConfig The configuration lock bits
 *            - SWT_UNLOCK   : Unlock
 *            - SWT_SOFTLOCK : Soft lock
 *            - SWT_HARDLOCK : Hard lock
 * @return operation status:
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_LockConfig(uint32_t instance,
                            swt_lock_t lockConfig);

/*!
 * @brief Initializes the SWT instance and start timer.
 * This function initializes the SWT instance by user configuration and start timer.
 * Ensure that the SIRC clock gate is enabled.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] userConfigPtr Pointer to the SWT user configuration structure
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to the SWT has been initialized
 *                            or timeout value less than minimum timeout
 *                            or the SWT was locked by hard lock.
 */
status_t SWT_DRV_Init(uint32_t instance,
                      const swt_user_config_t * const userConfigPtr);

/*!
 * @brief Gets the default configuration of the SWT.
 * This function gets the default configuration of the SWT.
 *
 * @param[out] config Pointer to the default configuration
 */
void SWT_DRV_GetDefaultConfig(swt_user_config_t * const config);

/*!
 * @brief De-initializes the SWT instance
 * This function resets all configuration to default and disable the SWT instance.
 *
 * @param[in] instance The SWT peripheral instance number
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_Deinit(uint32_t instance);

/*!
 * @brief Sets service mode.
 * This function sets service mode and sets initial service key if in Keyed Service Mode.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] mode Service mode
 *            - SWT_FS_SEQ_MODE : Fixed Service Sequence mode
 *            - SWT_KS_SEQ_MODE : Keyed Service Sequence mode
 *            - SWT_FA_EXE_MODE : Fixed Address Execution mode
 *            - SWT_IA_EXE_MODE : Incremental Address Execution mode
 * @param[in] serviceKey The initial service key
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_SetServiceConfig(uint32_t instance,
                                  swt_service_mode_t mode,
                                  uint16_t serviceKey);

/*!
 * @brief Services the SWT.
 * This function resets the SWT counter in fixed service sequence and keyed service
 * sequence modes.
 * - Fixed Service Sequence Mode: write fixed keys.
 * - Keyed Service Sequence Mode: write pseudo random keys (17*SK+3) mod 2^16.
 *
 * @param[in] instance The SWT peripheral instance number
 */
void SWT_DRV_Service(uint32_t instance);

/*!
 * @brief Sets timeout interrupt.
 * This function enables/disables the SWT timeout interrupt.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] enable
 *            - true  : Enable interrupt
 *            - false : Disable interrupt
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_SetIntConfig(uint32_t instance,
                              bool enable);

/*!
 * @brief Clears the Timeout Interrupt Flag.
 * This function clears the Timeout Interrupt Flag.
 *
 * @param[in] instance The SWT peripheral instance number
 */
void SWT_DRV_ClearIntFlag(uint32_t instance);

/*!
 * @brief Gets the Timeout Interrupt Status.
 * This function gets the Timeout Interrupt Status.
 *
 * @param[in] instance The SWT peripheral instance number
 * @return operation status
 *         - true  : Interrupt request due to an initial time-out.
 *         - false : No interrupt request.
 */
bool SWT_DRV_GetIntStatus(uint32_t instance);

/*!
 * @brief Sets timeout value.
 * This function sets timeout value.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] value The timeout value
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to timeout value less than minimum timeout
 *                            or the SWT was locked by hard lock.
 */
status_t SWT_DRV_SetTimeoutValue(uint32_t instance,
                                 uint32_t value);

/*!
 * @brief Sets window mode.
 * This function enables/disables window mode and sets window value if enabled.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[in] enable State
 *            - true  : Enable window mode
 *            - false : Disable window mode
 * @param[in] value The window value
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_SetWindowConfig(uint32_t instance,
                                 bool enable,
                                 uint32_t value);

/*!
 * @brief Gets the value of the SWT counter.
 * This function gets the value of the SWT counter.
 * When the watchdog is disabled (SWT_CR[WEN] is 0), this field shows the value of the internal down
 * counter. When the watchdog is enabled (SWT_CR[WEN] is 1), this field is cleared (the value is
 * 0x0000_0000). Values in this field can lag behind the internal counter value for up to 6 system clock
 * cycles plus 8 counter clock cycles. Therefore, the value read from this field immediately after
 * disabling the watchdog may be higher than the actual value of the internal counter.
 * The SWT timer should be stopped before calling this function.
 *
 * @param[in] instance The SWT peripheral instance number
 * @param[out] value Pointer to the counter value
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to the SWT timer is enabled.
 */
status_t SWT_DRV_GetCounterValue(uint32_t instance,
                                 uint32_t * value);

/*!
 * @brief Enables the SWT timer.
 * This function enables the SWT timer.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_StartTimer(uint32_t instance);

/*!
 * @brief Disables the SWT timer.
 * This function disables the SWT timer.
 * This function will unlock the SWT configuration.
 *
 * @param[in] instance The SWT peripheral instance number
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to SWT was locked by hard lock.
 */
status_t SWT_DRV_StopTimer(uint32_t instance);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* SWT_DRIVER_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
