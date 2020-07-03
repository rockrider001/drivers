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

#ifndef IGF_DRIVER_H
#define IGF_DRIVER_H

/*!
 * @file igf_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro defines a bit mask or shifting value used to access register bit-fields.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * This increases ease of use: allows users to access the corresponding field in the register
 * using an already defined type.
 *
 */

/*!
 * @addtogroup igf_driver
 * @{
 */

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @brief igf filter mode enumeration.
 * This one selects the filter type for the input signal.
 * Implements : filter_type_t_Class
 */
typedef enum
{
    FILTER_BY_PASS            = 0U,    /*!< Filter type by pass            */
    FILTER_WINDOWING          = 1U,    /*!< Filter type windowing          */
    FILTER_INTERGRATING       = 2U,    /*!< Filter type intergrating       */
    FILTER_INTERGRATING_HOLD  = 3U,    /*!< Filter type intergrating-holde */
} filter_type_t;

/*!
 * @brief igf immediate edge propagation filter.
 * This one selects the edge propagation depends on prescaler or within three system clock cycles.
 * Implements : edge_propagation_t_Class
 */
typedef enum
{
    EDGE_PROPAGATION_PRESCALER         = 0U,    /*!< Edge propagation depends on prescaler selected  */
    EDGE_PROPAGATION_SYSTEM_CLK        = 1U     /*!< Edge propagation within three system clock cycles selected  */
} edge_propagation_t;

/*!
 * @brief igf channel configuration structure
 *
 * Implements : igf_ch_param_t_Class
 */
typedef struct
{
    uint8_t                 channel;                     /*!< The Id of channel */
    bool                    allowDebugMode;              /*!< Allow channel in group can enter debug mode */
    edge_propagation_t      edgePropagation;             /*!< Select the propagation of an edge through the filter */
    uint32_t                prescalerValue;              /*!< Value filter prescaler defines the rate of the filter internal counter */
    filter_type_t           filterModeRisingEdge;        /*!< Filter Mode of channel with rising edge */
    filter_type_t           filterModeFallingEdge;       /*!< Filter Mode of channel with falling edge */
    uint32_t                risingEdgeThreshold;         /*!< The rising edge threshold value when a rising edge occurs is being filtered */
    uint32_t                fallingEdgeThreshold;        /*!< The falling edge threshold value when a rising edge occurs is being filtered */
    bool                    invertOutput;                /*!< Output polarity bit allow inverted or not inverted output */
} igf_ch_param_t;

/*!
 * @brief Defines the configuration structures are used in the input filter mode.
 *
 * Implements : igf_config_t_Class
 */
typedef struct
{
    uint8_t configsNumber;                         /*!< Number of igf configurations */
    const igf_ch_param_t * igfChConfig;            /*!< Pointer to igf configure table.*/
} igf_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes input glitch filter according to user input.
 *        This function performs the actual implementation-specific initialization
 *        based on the provided input filter type configurations.
 *        If a channel uses the internal prescaler counter, user need to configure 
 *        a filter cluster channel.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] configPtr The pointer to configuration structure. If user configures
 *                      a channel using internal counter prescaler, user need to
 *                      enable filter cluster channel.
 * @return Operation status
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t IGF_DRV_Init(uint32_t instance, const igf_config_t * configPtr);

/*!
 * @brief This function deinitializesDe-initialize a input glitch filter instance.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 *
 */
void IGF_DRV_DeInit(uint32_t instance);

/*!
 * @brief This function enable the filter operation.
 * The filter is enabled the filtering selected types are applied
 * to the rising and falling edges of the input signal.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to enable or disable filter.
 *                    When user disable the cluster filter channel of group, all of channel
 *                    in the group will be disabled because that all are turned off internal
 *                    counter prescaler.
 *
 */
void IGF_DRV_EnableChannel(uint32_t instance,const uint8_t channelId);

/*!
 * @brief This function disable the filter operation.
 * The filter is disabled the filtering selected types are applied
 * to the rising and falling edges of the input signal.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to enable or disable filter.
 *                    When user disable the cluster filter channel of group, all of channel
 *                    in the group will be disabled because that all are turned off internal
 *                    counter prescaler.
 *
 */
void IGF_DRV_DisableChannel(uint32_t instance,const uint8_t channelId);

/*!
 * @brief This function will check the filtering is enabled or disabled.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to check.
 * @return The filtering is enabled or disabled
 *            true:  Enable
 *            false: Disable
 *
 */
bool IGF_DRV_GetEnableChannel(uint32_t instance, const uint8_t channelId);

/*!
 * @brief This function will set the rising filter mode of channel.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to enable or disable filter.
 * @param[in] mode The rising filter mode is selected.
 *
 */
void IGF_DRV_SetRisingFilterModeChannel(uint32_t instance, const uint8_t channelId, filter_type_t mode);

/*!
 * @brief This function will set the rising filter mode of channel.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] mode The rising filter mode is selected.
 *
 */
void IGF_DRV_SetFallingFilterModeChannel(uint32_t instance, filter_type_t mode);

/*!
 * @brief This function will check the current rising filter type mode of channel.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to check.
 * @return filter_type_t The current rising filter type mode of channel.
 *
 */
filter_type_t IGF_DRV_GetRisingFilterModeChannel(uint32_t instance, const uint8_t channelId);

/*!
 * @brief This function will check the current falling filter type mode of channel.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to check.
 * @return filter_type_t The current falling filter type mode of channel.
 *
 */
filter_type_t IGF_DRV_GetFallingFilterModeChannel(uint32_t instance, const uint8_t channelId);

/*!
 * @brief This function will set value the filter counter threshold when a rising
 *                 edge is being filtered.
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] channel The channel number id want to enable or disable filter.
 * @param[in] value The value defines the filter counter threshold
 * when a rising edge is being filtered
 *
 */
void IGF_DRV_SetRisingThreshold(uint32_t instance,const uint8_t channelId, const uint32_t value);

/*!
 * @brief This function will set value the filter counter threshold when a falling
 *                 edge is being filtered. This one only sets falling threshold value
 *                 for channel 0.
 *
 * @param[in] instance The input glitch filter peripheral instance number.
 * @param[in] value The value defines the filter counter threshold
 * when a falling edge is being filtered
 *
 */
void IGF_DRV_SetFallingThreshold(uint32_t instance, const uint32_t value);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup igf */

#endif /* IGF_DRIVER_H */
/**************************************************************************************************
 * EOF
 ******************************************************************************/
