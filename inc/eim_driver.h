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

/*!
 * @file eim_driver.h
 */

#ifndef EIM_DRIVER_H
#define EIM_DRIVER_H

#include <stddef.h>
#include "device_registers.h"

/*!
 * @defgroup eim_drv EIM Driver
 * @ingroup eim
 * @brief Error Injection Module Peripheral Driver.@n
 * EIM PD provides a set of high-level APIs/services to configure the
 * Error Injection Module (EIM) module.
 * @addtogroup eim_drv
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The value default of EIM check-bit mask  */
#define EIM_CHECKBITMASK_DEFAULT    (0x01U)
/*! @brief The value default of EIM data mask  */
#define EIM_DATAMASK_DEFAULT        (0x00U)

/*!
 * @brief EIM channel configuration structure
 *
 * This structure holds the configuration settings for the EIM channel
 * Implements : eim_user_channel_config_t_Class
 */
typedef struct
{
    uint8_t channel;       /*!< EIM channel number                                          */
#if defined(FEATURE_EIM_CHECKBITMASK_32BIT)
    uint32_t checkBitMask; /*!< Specifies whether the corresponding bit of the check-bit bus
                               from the target RAM should be inverted or remain unmodified */
#else
    uint8_t checkBitMask; /*!< Specifies whether the corresponding bit of the check-bit bus
                               from the target RAM should be inverted or remain unmodified */
#endif
    uint32_t dataMask;    /*!< Specifies whether the corresponding bit of the read data bus
                               from the target RAM should be inverted or remain unmodified */
#if defined(FEATURE_EIM_DATAMASK1)
    uint32_t dataMask1;   /*!< Specifies whether the corresponding bit of the read data bus
                               from the target RAM should be inverted or remain unmodified */
#endif
    bool enable;          /*!< true : EIM channel operation is enabled
                               false : EIM channel operation is disabled                   */
} eim_user_channel_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name EIM Driver API
 * @{
 */

#if defined(__cplusplus)
extern "C"
{
#endif

/*!
 * @brief Initializes the module and configures the channels.
 *
 * This function initializes the module and configures the channels
 * using the provided configuration.
 *
 * @param[in] instance          EIM module instance number.
 * @param[in] channelCnt        Number of configured channels.
 * @param[in] channelConfigArr  EIM channel configuration structure array.
 */
void EIM_DRV_Init(uint32_t instance,
                  uint8_t channelCnt,
                  const eim_user_channel_config_t *channelConfigArr);

/*!
 * @brief De-initializes the module.
 *
 * This function sets the module parameters to reset values and disables the module.
 * In order to use the the module again, the initialization function must be called.
 *
 * @param[in] instance EIM module instance number
 */
void EIM_DRV_Deinit(uint32_t instance);

/*!
 * @brief Configures a channel with provided settings.
 *
 * This function configures the check-bit mask, data mask(s) and
 * operation status (enabled/disabled) for the provided channel.
 * The module MUST be disabled before calling this function.
 *
 * @param[in] instance          EIM module instance number.
 * @param[in] userChannelConfig Pointer to EIM channel configuration structure.
 */
void EIM_DRV_ConfigChannel(uint32_t instance,
                           const eim_user_channel_config_t *userChannelConfig);

/*!
 * @brief Returns the configuration of the provided channel.
 *
 * This function returns the check bit mask, data mask(s) and
 * operation status of the provided channel.
 *
 * @param[in]   instance        EIM module instance number.
 * @param[in]   channel         EIM channel number.
 * @param[out]  channelConfig   Pointer to EIM channel configuration structure.
 */
void EIM_DRV_GetChannelConfig(uint32_t instance,
                              uint8_t channel,
                              eim_user_channel_config_t *channelConfig);

/*!
 * @brief Returns a default configuration for the provided channel.
 *
 * This function returns default values for check bit mask, data mask and operation status
 * for the provided channel.
 *
 * @param[in]   channel         EIM channel number.
 * @param[out]  channelConfig   Pointer to EIM channel configuration structure with default values.
 */
void EIM_DRV_GetDefaultConfig(uint8_t channel,
                              eim_user_channel_config_t *channelConfig);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* EIM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
