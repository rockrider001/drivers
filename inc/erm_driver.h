/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef ERM_DRIVER_H
#define ERM_DRIVER_H

/*! @file erm_driver.h */

#include "device_registers.h"
#include "status.h"

/*!
 * @defgroup erm_driver ERM Driver
 * @ingroup erm
 * @brief Error Reporting Module Peripheral Driver.
 * @details This section describes the programming interface of the ERM driver.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief ERM types of ECC events
 * Implements : erm_ecc_event_t_Class
 */
typedef enum
{
    ERM_EVENT_NONE              = 0U,   /*!< None events */
    ERM_EVENT_SINGLE_BIT        = 1U,   /*!< Single-Bit correction ECC events */
    ERM_EVENT_NON_CORRECTABLE   = 2U    /*!< Non-Correctable ECC events */
} erm_ecc_event_t;

/*!
 * @brief ERM interrupt notification configuration structure
 * Implements : erm_interrupt_config_t_Class
 */
typedef struct
{
    bool enableSingleCorrection;    /*!< Enable Single-Bit Correction Interrupt Notification */
    bool enableNonCorrectable;      /*!< Enable Non-Correctable Interrupt Notification */
} erm_interrupt_config_t;

/*!
 * @brief ERM user configuration structure
 * Implements : erm_user_config_t_Class
 */
typedef struct
{
    uint8_t channel;                                /*!< Channel configured */
    const erm_interrupt_config_t * interruptCfg;    /*!< Interrupt configuration assigned */
} erm_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name ERM DRIVER API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the module.
 *
 * This function initializes the module based on provided configuration.
 * Parameter 'channelCnt' takes values between 1 and the maximum channel
 * count supported by the hardware.
 *
 * @param[in] instance      ERM instance number.
 * @param[in] channelCnt    Number of channels.
 * @param[in] userConfigArr Pointer to user configuration structure array.
 */
void ERM_DRV_Init(uint32_t instance,
                  uint8_t channelCnt,
                  const erm_user_config_t * userConfigArr);

/*!
 * @brief De-initializes the module.
 *
 * This function de-initializes the module and resets it to the default configuration.
 *
 * @param[in] instance ERM instance number.
 */
void ERM_DRV_Deinit(uint32_t instance);

/*!
 * @brief Configures the interrupt notifications.
 *
 * This function configures the interrupt notifications based on
 * the provided configuration.
 *
 * @param[in] instance      ERM instance number.
 * @param[in] channel       Configured memory channel.
 * @param[in] interruptCfg  ERM interrupt notifications configuration structure.
 */
void ERM_DRV_SetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t interruptCfg);

/*!
 * @brief Returns the configured interrupt notifications.
 *
 * This function returns the currently configured interrupt notifications
 * for the available events (which type of event interrupts are enabled/disabled).
 *
 * @param[in]  instance      ERM instance number.
 * @param[in]  channel       Examined memory channel.
 * @param[out] interruptPtr  Pointer to the ERM interrupt notifications configuration structure.
 */
void ERM_DRV_GetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t * const interruptPtr);

/*!
 * @brief Clears the record of an error event and the corresponding interrupt notification.
 *
 * This function clears the record of an error event.
 * If the corresponding interrupt notification is enabled, the interrupt notification will be cleared.
 *
 * @param[in] instance  ERM instance number.
 * @param[in] channel   Configured memory channel.
 * @param[in] eccEvent  Type of ECC event record to be cleared.
 */
void ERM_DRV_ClearEvent(uint32_t instance,
                        uint8_t channel,
                        erm_ecc_event_t eccEvent);

/*!
 * @brief Returns the type of the last occurred ECC event and its related information.
 *
 * This function returns the type of the last occurred ECC event and other information
 * such as the address of the error or the bit position (if supported by the product).
 *
 * @param[in]  instance     ERM instance number.
 * @param[in]  channel      Examined memory channel.
 * @param[out] addressPtr   Pointer to the address of the last occurred ECC event.
 * @param[out] bitPosPtr    Pointer to the bit position of the last occurred ECC event (present only on products which support this feature).
 * @return Type of the last occurred ECC event.
 */
erm_ecc_event_t ERM_DRV_GetErrorDetail(uint32_t instance,
                                       uint8_t channel,
                                       uint32_t * addressPtr
#ifdef FEATURE_ERM_ECC_BIT_POS
                                     , uint32_t * bitPosPtr
#endif
                                       );

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* ERM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
