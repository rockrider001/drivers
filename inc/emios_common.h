/*
 * Copyright 2017-2019 NXP
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
* @file emios_common.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an struct type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter 'commonParam' could be declared as pointing to const.
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * This macro is used by user.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, global typedef not referenced
 * This structure is used by user.
 */

#ifndef EMIOS_COMMON_H
#define EMIOS_COMMON_H

#if defined(__cplusplus)
extern "C" {
#endif
#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @defgroup emios_common eMIOS common
 * @ingroup mc_emios_driver
 * @brief The C55 SDK eMIOS provides functionality to generate or measure time events like PWM,
 * counter generation, period measurement, output compare.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EMIOS_GROUP0                 0U
#define EMIOS_GROUP1                 1U
#define EMIOS_GROUP2                 2U
/*******************************************************************************
* Variables
******************************************************************************/

/*!
 * @brief Counter bus driven
 * Implements : emios_counter_bus_driven_t_Class
 */
typedef enum
{
#if FEATURE_EMIOS_BUS_B_SELECT
    EMIOS_CNT_BUSB_DRIVEN             = 0U,
#endif
#if FEATURE_EMIOS_BUS_C_SELECT
    EMIOS_CNT_BUSC_DRIVEN             = 8U,
#endif
#if FEATURE_EMIOS_BUS_D_SELECT
    EMIOS_CNT_BUSD_DRIVEN             = 16U,
#endif
#if FEATURE_EMIOS_BUS_F_SELECT
    EMIOS_CNT_BUSF_DRIVEN             = 22U,
#endif
#if FEATURE_EMIOS_BUS_A_SELECT
    EMIOS_CNT_BUSA_DRIVEN             = 23U,
#endif
#if FEATURE_EMIOS_BUS_E_SELECT
    EMIOS_CNT_BUSE_DRIVEN             = 24U
#endif
} emios_counter_bus_driven_t;

/*!
 * @brief Input capture edge mode : rising edge, falling edge or both of them
 * Implements : emios_edge_trigger_mode_t_Class
 */
typedef enum
{
    EMIOS_TRIGGER_EDGE_FALLING        = 0x00U,               /*!< Falling edge trigger */
    EMIOS_TRIGGER_EDGE_RISING         = 0x01U,               /*!< Rising edge trigger */
    EMIOS_TRIGGER_EDGE_ANY            = 0x02U                /*!< Rising and falling edge trigger */
} emios_edge_trigger_mode_t;

/*!
 * @brief Pulse polarity capturing
 * Implements : emios_pulse_polarity_mode_t_Class
 */
typedef enum
{
    EMIOS_NEGATIVE_PULSE              = 0x00U,               /*!< Negative pulse capturing / active low */
    EMIOS_POSITIVE_PULSE              = 0x01U                /*!< Positive pulse capturing / active high */
} emios_pulse_polarity_mode_t;

/*!
* @brief Sub-mode
* Implements : emios_submode_config_t_Class
*/
typedef enum
{
    EMIOS_MODE_GPIO_INPUT             = 0x00U,               /*!< General-Purpose Input */
    EMIOS_MODE_GPIO_OUTPUT            = 0x01U                /*!< General-Purpose Output */
} emios_submode_config_t;

/*!
 * @brief Internal pre-scaler factor selection for the clock source.
 * Implements : emios_clock_internal_ps_t_Class
 */
typedef enum
{
    EMIOS_CLOCK_DIVID_BY_1            = 0x00U,                /*!< Divide by 1 */
    EMIOS_CLOCK_DIVID_BY_2            = 0x01U,                /*!< Divide by 2 */
    EMIOS_CLOCK_DIVID_BY_3            = 0x02U,                /*!< Divide by 3 */
    EMIOS_CLOCK_DIVID_BY_4            = 0x03U,                /*!< Divide by 4 */
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
    EMIOS_CLOCK_DIVID_BY_5            = 0x04U,                /*!< Divide by 5 */
    EMIOS_CLOCK_DIVID_BY_6            = 0x05U,                /*!< Divide by 6 */
    EMIOS_CLOCK_DIVID_BY_7            = 0x06U,                /*!< Divide by 7 */
    EMIOS_CLOCK_DIVID_BY_8            = 0x07U,                /*!< Divide by 8 */
    EMIOS_CLOCK_DIVID_BY_9            = 0x08U,                /*!< Divide by 9 */
    EMIOS_CLOCK_DIVID_BY_10           = 0x09U,                /*!< Divide by 10 */
    EMIOS_CLOCK_DIVID_BY_11           = 0x0AU,                /*!< Divide by 11 */
    EMIOS_CLOCK_DIVID_BY_12           = 0x0BU,                /*!< Divide by 12 */
    EMIOS_CLOCK_DIVID_BY_13           = 0x0CU,                /*!< Divide by 13 */
    EMIOS_CLOCK_DIVID_BY_14           = 0x0DU,                /*!< Divide by 14 */
    EMIOS_CLOCK_DIVID_BY_15           = 0x0EU,                /*!< Divide by 15 */
    EMIOS_CLOCK_DIVID_BY_16           = 0x0FU                 /*!< Divide by 16 */
#endif
} emios_clock_internal_ps_t;

/*!
 * @brief Input filter select.
 * The Input filter control the programmable input filter, selecting the minimum input pulse width that can
 * pass through the filter. For output modes, these bits have no meaning.
 * Implements : emios_input_filter_t_Class
 */
typedef enum
{
    EMIOS_INPUT_FILTER_BYPASS         = 0x00U,
    EMIOS_INPUT_FILTER_02             = 0x01U,
    EMIOS_INPUT_FILTER_04             = 0x02U,
    EMIOS_INPUT_FILTER_08             = 0x04U,
    EMIOS_INPUT_FILTER_16             = 0x08U
} emios_input_filter_t;

/*!
 * @brief Output Disable select, select one of the four output disable input signals
 * Implements : emios_output_dis_sel_t_Class
 */
typedef enum
{
    EMIOS_OUTPUT_DISABLE_0            = 0x00U,
    EMIOS_OUTPUT_DISABLE_1            = 0x01U,
    EMIOS_OUTPUT_DISABLE_2            = 0x02U,
    EMIOS_OUTPUT_DISABLE_3            = 0x03U
} emios_output_dis_sel_t;

/*!
 * @brief Counter bus select. Select either one of the counter buses or the internal counter to be used by
 * the Unified Channel.
 * Implements : emios_bus_select_t_Class
 */
typedef enum
{
    EMIOS_BUS_SEL_A                   = 0x00U,                /*!< Global counter bus A */
    EMIOS_BUS_SEL_BCDE                = 0x01U,                /*!< Local counter bus */
#if FEATURE_EMIOS_BUS_F_SELECT
    EMIOS_BUS_SEL_F                   = 0x02U,                /*!< Global counter bus F */
#endif
    EMIOS_BUS_SEL_INTERNAL            = 0x03U                 /*!< Internal counter bus */
} emios_bus_select_t;

/*!
* @brief Select a basic mode for each of channels
* Implements : emios_mode_config_t_Class
*/
typedef enum
{
    EMIOS_GMODE_GPIO                  = 0x00U,                /*!< General-Purpose Input/ Output */
    EMIOS_GMODE_SAIC                  = 0x01U,                /*!< Single-Action Input Capture */
    EMIOS_GMODE_SAOC                  = 0x02U,                /*!< Single-Action Output Compare */
    EMIOS_GMODE_IPWM                  = 0x03U,                /*!< Input Pulse-Width Measurement */
    EMIOS_GMODE_IPM                   = 0x04U,                /*!< Input Period Measurement */
    EMIOS_GMODE_DAOC                  = 0x05U,                /*!< Double-Action Output Compare */
    EMIOS_GMODE_MC                    = 0x06U,                /*!< Modulus Counter */
    EMIOS_GMODE_MCB                   = 0x07U,                /*!< Modulus Counter Buffered */
    EMIOS_GMODE_OPWFMB                = 0x08U,                /*!< Output Pulse-Width and Frequency Modulation Buffered */
    EMIOS_GMODE_OPWMCB                = 0x09U,                /*!< Center Aligned Output Pulse Width Modulation with
                                                                   Dead Time Insertion Buffered */
    EMIOS_GMODE_OPWMB                 = 0x0AU,                /*!< Output Pulse-Width Modulation Buffered */
    EMIOS_GMODE_OPWMT                 = 0x0BU                 /*!< Output Pulse-Width Modulation with Trigger */
} emios_mode_config_t;

/*!
 * @brief eMIOS common configuration parameters structure
 * Implements : emios_common_param_t_Class
 */
typedef struct
{
    bool                              allowDebugMode;         /*!< Allow all channel in eMIOS group can enter debug mode */
    bool                              lowPowerMode;           /*!< Low power mode or normal mode */
    uint16_t                          clkDivVal;              /*!< Select the clock divider value for the global prescaler in range (1-256) */
    bool                              enableGlobalPrescaler;  /*!< Enable global prescaler or disable  */
    bool                              enableGlobalTimeBase;   /*!< Enable global timebase or disable   */
#if defined(FEATURE_EMIOS_STAC_CLIENT)
    bool                              enableExternalTimeBase; /*!< Enable external timebase or disable */
    uint8_t                           serverTimeSlot;         /*!< Select the address of a specific STAC server to which the STAC is assigned */
#endif
} emios_common_param_t;

/*!
 * @brief GPIO configuration parameters structure
 * Implements : emios_gpio_mode_param_t_Class
 */
typedef struct
{
    emios_submode_config_t          mode;                     /*!< Sub-mode selected */
    emios_input_filter_t            filterInput;              /*!< Filter Value, ignore if not gpio input mode */
    bool                            filterEn;                 /*!< Input capture filter state, ignore if not gpio input mode */
    emios_edge_trigger_mode_t       triggerMode;              /*!< Input signal trigger mode */
} emios_gpio_mode_param_t;

/******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS Driver API
 * @{
 */

/*!
 * @brief Setup global prescaler
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] commonParam A pointer to emios_common_param_t structure.
 * @return void
 */
void EMIOS_DRV_InitGlobal(uint8_t emiosGroup,
                          const emios_common_param_t *commonParam);

/*!
 * @brief Start global eMIOS group
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_EnableGlobalEmios(uint8_t emiosGroup);

/*!
 * @brief Disable global eMIOS group
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_DisableGlobalEmios(uint8_t emiosGroup);

/*!
 * @brief Enter low power mode.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_EnterLowPowerMode(uint8_t emiosGroup);

/*!
 * @brief Escape low power mode to normal mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_EscLowPowerMode(uint8_t emiosGroup);

/*!
 * @brief Get Low power mode state. Get TRUE if eMIOS group is running in Low power mode.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return bool
 *        - true    :    Low power mode is actived.
 *        - false   :    Normal mode is actived.
 */
bool EMIOS_DRV_IsLowPowerMode(uint8_t emiosGroup);

/*!
 * @brief Get Allowing debug mode state. Get TRUE if all channels in the eMIOS group can enter debug mode.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return bool
 *        - false :   If Don't allow channel enter debug mode.
 *        - true  :   allow channel enter debug mode.
 */
bool EMIOS_DRV_IsAllowDebugMode(uint8_t emiosGroup);

/*!
 * @brief Get global prescaler enable bit state. Get TRUE if eMIOS group is enabling global prescaler
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return bool
 *        - true    :   Global prescaler is enabled
 *        - false   :   Global prescaler is disabled
 */
bool EMIOS_DRV_IsGlobalPresEnabled(uint8_t emiosGroup);

/*!
 * @brief Get global timebase enable bit. Get TRUE if the global timebase is enabling
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return bool
 *        - true    :    Global timebase is enabled.
 *        - false   :    Global timebase is disabled.
 */
bool EMIOS_DRV_IsGlobalTimebaseEn(uint8_t emiosGroup);

/*!
 * @brief Read FLAG bit state. The FLAG bit is set when an input capture or
 * a match event in the comparators occurred.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - true    :    Flag bit is set.
 *        - false   :    Flag bit is not set.
 */
bool EMIOS_DRV_ReadFlagState(uint8_t emiosGroup,
                             uint8_t channel);

/*!
 * @brief Get state of output update disable
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - true:   If output update disable bit is asserted.
 *        - false:  If output update disable bit is not asserted.
 */
bool EMIOS_DRV_IsOutputUpdateDisabled(uint8_t emiosGroup,
                                      uint8_t channel);

/*!
 * @brief Disable channel output update
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_DisableOutputUpdate(uint8_t emiosGroup,
                                   uint8_t channel);

/*!
 * @brief Enable channel output update
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_EnableAChOutputUpdate(uint8_t emiosGroup,
                                     uint8_t channel);

/*!
 * @brief Enable all channels output update
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_EnableAllChOutputUpdate(uint8_t emiosGroup);

/*!
 * @brief Initialize eMIOS General-Purpose Input/Output (GPIO) mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] gpioParam A pointer to the GPIO configuration structure
 * @return void
 */
void EMIOS_DRV_InitGpioMode(uint8_t emiosGroup,
                            uint8_t channel,
                            const emios_gpio_mode_param_t *gpioParam);

/*!
 * @brief Set a channel enters freeze state, should be setting
 * EMIOS_AllowEnterDebugMode first.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return operation status
 *        - STATUS_SUCCESS                 :  Operation was successful.
 *        - STATUS_ERROR                   :  Operation failed, invalid input value.
 *        - STATUS_EMIOS_ENABLE_GLOBAL_FRZ :  Need call EMIOS_AllowEnterDebugMode first.
 */
status_t EMIOS_DRV_ChannelEnterDebugMode(uint8_t emiosGroup,
                                         uint8_t channel);

/*!
 * @brief Release a channel from freeze state
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelEscDebugMode(uint8_t emiosGroup,
                                   uint8_t channel);

/*!
 * @brief Setup Output disable:
 *        eMIOS can disable it's output using it's disable inputs.
 *        Disable inputs[3:0] are driven from the flag out bits[11:8].
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] isOutputDisable If output is disabled, this param set to TRUE
 * @param[in] odissl Output disable select mode
 * @return void
 */
void EMIOS_DRV_ChannelSetupOutputDisable(uint8_t emiosGroup,
                                         uint8_t channel,
                                         bool isOutputDisable,
                                         emios_output_dis_sel_t odissl);

/*!
 * @brief Enable DMA request
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelEnableDMARequest(uint8_t emiosGroup,
                                       uint8_t channel);

/*!
 * @brief Disable DMA request (assigned to interrupt request)
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelEnableInterruptRequest(uint8_t emiosGroup,
                                             uint8_t channel);

/*!
 * @brief Get state of DMA mode enable bit. Get TRUE if eMIOS channel is DMA request mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - true    :   If DMA mode is enabled.
 *        - false   :   If DMA mode is disabled.
 */
bool EMIOS_DRV_IsDMAMode(uint8_t emiosGroup,
                         uint8_t channel);

/*!
 * @brief Disallow the Unified Channel FLAG bit to generate an interrupt signal or
 * a DMA request signal.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelDisableIsrRequest(uint8_t emiosGroup,
                                        uint8_t channel);

/*!
 * @brief Allow the Unified Channel FLAG bit to generate an interrupt signal or
 * a DMA request signal
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelEnableIsrRequest(uint8_t emiosGroup,
                                       uint8_t channel);

/*!
 * @brief Reflects the input pin state after being filtered and synchronized.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - true    :  If input pin state is set.
 *        - false   :  If input pin state is not set.
 */
bool EMIOS_DRV_ReadInputPinState(uint8_t emiosGroup,
                                 uint8_t channel);

/*!
 * @brief Reflects the output pin state
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - true    :  If output pin state is set.
 *        - false   :  If output pin state is not set.
 */
bool EMIOS_DRV_ReadOutputPinState(uint8_t emiosGroup,
                                  uint8_t channel);

/*!
 * @brief Reset eMIOS channel to GPIO mode (reset default)
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_DeInitChannel(uint8_t emiosGroup,
                             uint8_t channel);

/*!
* brief Clear FLAG bit. FLAG bit is set when an input capture or a match event in the
* comparators occurred.
*
* param[in] emiosGroup The eMIOS group id
* param[in] channel The channel in this eMIOS group
* return void
*/
void EMIOS_DRV_ClrFlagState(uint8_t emiosGroup,
                            uint8_t channel);

/*!
* brief This function is used to set the action executed on a compare
 * match value to set output pin, clear output pin, toggle output pin.
*
* param[in] emiosGroup The eMIOS group id
* param[in] channel The channel in this eMIOS group
* param[in] edgeSel The edge selection
* param[in] edgePol The edge polarity
* return void
*/
void EMIOS_DRV_SetOutputLevel(uint8_t emiosGroup,
                              uint8_t channel,
                              bool edgeSel,
                              bool edgePol);

/*!
 * @brief Get state of Channel clock disable bit
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return bool
 *        - false :   If Channel clock is enabled.
 *        - true  :   If Channel clock is disabled.
 */
bool EMIOS_DRV_IsChannelClkDisabled(uint8_t emiosGroup,
                                    uint8_t channel);

/*!
 * @brief Disable a channel clock
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelDisableClk(uint8_t emiosGroup,
                                 uint8_t channel);

/*!
 * @brief Enable a channel clock
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_ChannelEnableClk(uint8_t emiosGroup,
                                uint8_t channel);

/*!
 * @brief Enable all channels clock
 *
 * @param[in] emiosGroup The eMIOS group id
 * @return void
 */
void EMIOS_DRV_EnableAllChannelClk(uint8_t emiosGroup);

/*!
* brief This function is used to set output disable. The output after call this function is depended on its last logic level.
*
* param[in] emiosGroup The eMIOS group id
* param[in] channel The channel in this eMIOS group
* return void
*/
void EMIOS_DRV_OutputDisable(uint8_t emiosGroup,
                             uint8_t channel);

/*!
* brief This function is used to set output enable.
*
* param[in] emiosGroup The eMIOS group id
* param[in] channel The channel in this eMIOS group
* return void
*/
void EMIOS_DRV_OutputEnable(uint8_t emiosGroup,
                            uint8_t channel);

/*!
 * brief Get current output pin
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * return bool:
 *            true:  output high
 *            false: output low
 */
bool EMIOS_DRV_GetCurrentOutputPin(uint8_t emiosGroup,
                                   uint8_t channel);
/*! @} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif /* EMIOS_COMMON_H */
/*******************************************************************************
* EOF
******************************************************************************/
