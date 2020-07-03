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
* @file pwm_emios_driver.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an struct type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, global typedef not referenced
 * This structure is used by user.
 */

#ifndef PWM_EMIOS_DRIVER_H
#define PWM_EMIOS_DRIVER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "emios_common.h"

/*!
 * @defgroup pwm_emios_driver Pulse Width Modulation (eMIOS)
 * @ingroup emios
 * @brief PWM mode supported
 *           - Output Pulse Width and Frequency Modulation Buffered (OPWFMB) Mode
 *           - Output Pulse Width Modulation Buffered (OPWMB) Mode.
 *           - Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode.
 *           - Output Pulse Width Modulation with Trigger (OPWMT) Mode.
 * @{
 */

/*******************************************************************************
* Variables
******************************************************************************/
/*!
* @brief PWM mode
* Implements : emios_pwm_mode_config_t_Class
*/
typedef enum
{
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    EMIOS_MODE_OPWMT                             = 0x26U,     /*!< Output Pulse-Width Modulation with Trigger */
#endif
    EMIOS_MODE_OPWFMB_FLAGX1                     = 0x58U,     /*!< Output Pulse Width and Frequency Modulation Buffered,
                                                                   Flags are generated only on B1 matches */
    EMIOS_MODE_OPWFMB_FLAGX2                     = 0x5AU,     /*!< Output Pulse Width and Frequency Modulation Buffered,
                                                                   Flags are generated on both A1 & B1 matches */

    EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1 = 0x5CU,     /*!< Center Aligned Output Pulse Width Modulation Buffered
                                                                   (with trail edge dead-time), FLAG be generated in the trailing edge */
    EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2 = 0x5EU,     /*!< Center Aligned Output Pulse Width Modulation Buffered
                                                                   (with trail edge dead-time), FLAG be generated in the both edges */

    EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1  = 0x5DU,     /*!< Center Aligned Output Pulse Width Modulation Buffered
                                                                   (with lead edge dead-time), FLAG be generated in the trailing edge */
    EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2  = 0x5FU,     /*!< Center Aligned Output Pulse Width Modulation Buffered
                                                                   (with lead edge dead-time), FLAG be generated in the both edges */

    EMIOS_MODE_OPWMB_FLAGX1                      = 0x60U,     /*!< Output Pulse Width Modulation Buffered,
                                                                   Flags are generated only on trailing matches */
    EMIOS_MODE_OPWMB_FLAGX2                      = 0x62U      /*!< Output Pulse Width Modulation Buffered,
                                                                   Flags are generated on both leading and trailing matches */
} emios_pwm_mode_config_t;

/*!
* @brief PWM configuration parameters structure
* Implements : emios_pwm_param_t_Class
*/
typedef struct
{
    emios_pwm_mode_config_t         mode;                     /*!< Sub-mode selected */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler, pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_pulse_polarity_mode_t     outputActiveMode;         /*!< Output active value, Choose active low or high level */
    uint32_t                        periodCount;              /*!< Period count for OPWFM mode only */
    uint32_t                        dutyCycleCount;           /*!< Duty cycle count */
    emios_bus_select_t              timebase;                 /*!< Counter bus selected, ignore with OPWFM mode */
    uint32_t                        idealDutyCycle;           /*!< Ideal duty cycle of the PWM signal using to
                                                                   compare with the selected time base, for OPWMCB only */
    uint32_t                        deadTime;                 /*!< The dead time value and is compared against
                                                                   the internal counter, for OPWMCB only */
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    uint32_t                        triggerEventPlacement;    /*!< Trigger Event placement, for OPWMT mode only */
#endif
} emios_pwm_param_t;

/*******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS DRIVER API
 * @{
 */

/*!
* @brief Initialize PWM Mode
* For all mode operation: Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
   in the eMIOS UC Control register.
*
* @param[in] emiosGroup The eMIOS group id
* @param[in] channel The channel in this eMIOS group
* @param[in] pwmParam A pointer to the PWM configuration structure
* @return operation status
*        - STATUS_SUCCESS        :  Operation was successful.
*        - STATUS_ERROR          :  Operation failed, invalid input value.
*/
status_t EMIOS_DRV_PWM_InitMode(uint8_t emiosGroup,
                                uint8_t channel,
                                const emios_pwm_param_t *pwmParam);

 /*!
 * @brief Allow the software to force the output flip-flop to the level corresponding
 * to a match on leading edge. The FLAG bit is not set.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_PWM_ForcePWMMatchLeadingEdge(uint8_t emiosGroup,
                                            uint8_t channel);

/*!
 * @brief Allow the software to force the output flip-flop to the level corresponding
 * to a match on trailing edge. The FLAG bit is not set.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_PWM_ForcePWMMatchTrailingEdge(uint8_t emiosGroup,
                                             uint8_t channel);

/*!
 * @brief Get Period value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[out] retPeriod A pointer to return period value
 * @return uint32_t Value of period
 */
uint32_t EMIOS_DRV_PWM_GetPeriod(uint8_t emiosGroup,
                                 uint8_t channel);

/*!
 * @brief Set new Period value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newPeriod New Period value
 * @return void
 */
void EMIOS_DRV_PWM_SetPeriod(uint8_t emiosGroup,
                             uint8_t channel,
                             uint32_t newPeriod);

/*!
 * @brief Get Duty Cycle value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of duty cycle
 */
uint32_t EMIOS_DRV_PWM_GetDutyCycle(uint8_t emiosGroup,
                                    uint8_t channel);

/*!
 * @brief Set new Duty Cycle value in PWM mode
 * For this mode operation: Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
   in the eMIOS UC Control register.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newDutyCycle New duty cycle value
 * @return operation status
 *        - STATUS_SUCCESS      :  Operation was successful.
 *        - STATUS_ERROR        :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_PWM_SetDutyCycle(uint8_t emiosGroup,
                                    uint8_t channel,
                                    uint32_t newDutyCycle);

/*!
 * @brief Get Leading Edge Placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of leading edge placement in counter bus time base
 */
uint32_t EMIOS_DRV_PWM_GetLeadingEdgePlacement(uint8_t emiosGroup,
                                               uint8_t channel);

/*!
 * @brief Set new Leading edge placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newLeadingEdgePlacement new leading edge placement value
 * @return operation status
 *        - STATUS_SUCCESS      :  Operation was successful.
 *        - STATUS_ERROR        :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_PWM_SetLeadingEdgePlacement(uint8_t emiosGroup,
                                               uint8_t channel,
                                               uint32_t newLeadingEdgePlacement);

/*!
 * @brief Get Trailing Edge Placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of Trailing edge placement in counter bus time base
 */
uint32_t EMIOS_DRV_PWM_GetTrailingEdgePlacement(uint8_t emiosGroup,
                                                uint8_t channel);

/*!
 * @brief Set new Trailing edge placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newTrailingEdgePlacement New trailing edge placement value
 * @return void
 */
void EMIOS_DRV_PWM_SetTrailingEdgePlacement(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t newTrailingEdgePlacement);

/*!
 * @brief Set new dead time value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newDeadTime New Dead Time value
 * @return void
 */
void EMIOS_DRV_PWM_SetCenterAlignDeadTime(uint8_t emiosGroup,
                                          uint8_t channel,
                                          uint32_t newDeadTime);

/*!
 * @brief Get dead time value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of Dead Time
 */
uint32_t EMIOS_DRV_PWM_GetCenterAlignDeadTime(uint8_t emiosGroup,
                                              uint8_t channel);

/*!
 * @brief Get Ideal duty cycle value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of Ideal Duty Cycle
 */
uint32_t EMIOS_DRV_PWM_GetCenterAlignIdealDutyCycle(uint8_t emiosGroup,
                                                    uint8_t channel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*!
 * @brief Set new Trigger Placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newTriggerPlacement New Trigger Placement value
 * @return void
 */
void EMIOS_DRV_PWM_SetTriggerPlacement(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t newTriggerPlacement);

/*!
 * @brief Get Trigger Placement value in PWM mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of Trigger Placement
 */
uint32_t EMIOS_DRV_PWM_GetTriggerPlacement(uint8_t emiosGroup,
                                           uint8_t channel);
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* PWM_EMIOS_DRIVER_H */
/*******************************************************************************
* EOF
******************************************************************************/

