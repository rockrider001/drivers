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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. Also, the called functions
 * do not store the address into variables with lifetime longer then its own call.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an struct type with many values.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, cast performed between a pointer to
 * object type and a pointer to a different object type.
 * The cast is used for casting a pointer to a struct in order to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 2.2, Last value assigned to variable
 * These parameters are assigned after declared to avoid return a unknown value after they are initialized.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, conversion from pointer to void to pointer to other type
 * This is required for working with the common initialize function in initialized counter bus function.
 * Counter bus can run with many different mode.
 *
 */

#include "emios_pwm_driver.h"
#include "emios_mc_driver.h"
#include "emios_hw_access.h"

/*******************************************************************************
* Static variables define
******************************************************************************/
/*!
* @brief OPWFM configuration parameters structure
*/
typedef struct
{
    emios_pwm_mode_config_t         mode;                     /*!< Sub-mode selected */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler, pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_pulse_polarity_mode_t     outputActiveMode;         /*!< Output active value, Choose active low or high level */
    uint32_t                        periodCount;              /*!< Period count */
    uint32_t                        dutyCycleCount;           /*!< Duty cycle count */
} emios_opwfm_param_t;

/*!
* @brief OPWMB configuration parameters structure
*/
typedef struct
{
    emios_pwm_mode_config_t         mode;                     /*!< Sub-mode selected */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler, pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_pulse_polarity_mode_t     outputActiveMode;         /*!< Output active value, Choose active low or high level */
    uint32_t                        leadingEdgePlacement;     /*!< Leading edge placement */
    uint32_t                        trailingEdgePlacement;    /*!< Trailing edge placement */
    emios_bus_select_t              timebase;                 /*!< Counter bus selected */
} emios_opwmb_param_t;

/*!
* @brief OPWMCB configuration parameters structure
*/
typedef struct
{
    emios_pwm_mode_config_t         mode;                     /*!< Sub-mode selected */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler, pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_pulse_polarity_mode_t     outputActiveMode;         /*!< Output active value, Choose active low or high level */
    uint32_t                        idealDutyCycle;           /*!< Ideal duty cycle of the PWM signal using to
                                                              compare with the selected time base */
    uint32_t                        deadTime;                 /*!< The dead time value and is compared against the internal counter */
    emios_bus_select_t              timebase;                 /*!< Counter bus selected */
} emios_opwmcb_param_t;

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*!
* @brief OPWMT configuration parameters structure
*/
typedef struct
{
    emios_pwm_mode_config_t         mode;                     /*!< Sub-mode selected */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler, pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_pulse_polarity_mode_t     outputActiveMode;         /*!< Output active value, Choose active low or high level */
    uint32_t                        leadingEdgePlacement;     /*!< Leading edge placement */
    uint32_t                        trailingEdgePlacement;    /*!< Trailing edge placement */
    emios_bus_select_t              timebase;                 /*!< Counter bus selected */
    uint32_t                        triggerEventPlacement;    /*!< Trigger Event placement */
} emios_opwmt_param_t;
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

/*******************************************************************************
 * Private API declaration
 ******************************************************************************/
/*!
 * brief Set Ideal Duty Cycle value of waveforms in OPWMCB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] newIdealDutyCycle Ideal duty cycle value
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 */
static status_t EMIOS_DRV_PWM_SetCenterAlignIdealDutyCycle(uint8_t emiosGroup,
                                                           uint8_t channel,
                                                           uint32_t newIdealDutyCycle);

/*!
 * brief Get duty value for OPWFMB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] ARegVal A register's value
 * param[in] BRegVal B register's value
 * return duty value for OPWFMB mode
 */
static uint32_t EMIOS_DRV_PWM_GetDutyOpwfmb(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t ARegVal,
                                            uint32_t BRegVal);

/*!
 * brief Get Leading ideal duty value for OPWMCB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] busSelect The bus selected in this eMIOS group
 * param[in] ARegVal A register's value
 * param[in] BRegVal B register's value
 * param[in] edpolActive Output active value, Choose active low or high level
 * return leading ideal duty value for OPWMCB mode
 */
static uint32_t EMIOS_DRV_PWM_GetDutyLead(uint8_t emiosGroup,
                                          uint8_t busSelect,
                                          uint32_t ARegVal,
                                          uint32_t BRegVal,
                                          uint32_t edpolActive);

/*!
 * brief Get Trailing ideal duty value for OPWMCB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] busSelect The bus selected in this eMIOS group
 * param[in] ARegVal A register's value
 * param[in] BRegVal B register's value
 * param[in] edpolActive Output active value, Choose active low or high level
 * return Trailing ideal duty value for OPWMCB mode
 */
static uint32_t EMIOS_DRV_PWM_GetDutyTrail(uint8_t emiosGroup,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal,
                                           uint32_t edpolActive);

/*!
 * brief Get duty value for OPWMB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] busSelect The bus selected in this eMIOS group
 * param[in] ARegVal A register's value
 * param[in] BRegVal B register's value
 * return duty value for OPWMB mode
 */
static uint32_t EMIOS_DRV_PWM_GetDutyOpwmb(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal);

/*!
 * brief contains specific checks for SetLeadingEdgePlacement.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] newLeadingEdgePlacement leading edge placement value
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 */
static status_t EMIOS_DRV_PWM_SetLeadingEdgeCheck(uint8_t emiosGroup,
                                                  uint8_t channel,
                                                  uint32_t newLeadingEdgePlacement);

/*!
 * brief Initialization for OPWMB mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] pwmParam A pointer input configures structure for OPWMB mode
 * param[out] opwmb_param A pointer output configures structure for OPWMB mode
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 */
static status_t EMIOS_DRV_PWM_InitOpwmb(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_pwm_param_t *pwmParam,
                                        emios_opwmb_param_t * opwmb_param);

 /*!
 * brief Calibration ideal duty value for Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] activeMode Output active value, Choose active low or high level
 * param[in] idealDutyCycle Ideal duty cycle value
 * return new ideal duty value
 */
static uint32_t EMIOS_DRV_PWM_CalibrationDutyCycle(uint8_t emiosGroup,
                                                   uint8_t channel,
                                                   uint32_t activeMode,
                                                   uint32_t idealDutyCycle);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*!
 * brief Get duty value for OPWMT mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] busSelect The bus selected in this eMIOS group
 * param[in] ARegVal A register's value
 * param[in] BRegVal B register's value
 * return duty value for OPWMT mode
 */
static uint32_t EMIOS_DRV_PWM_GetDutyOpwmt(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal);

/*!
 * brief Initialization for OPWMT mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] pwmParam A pointer input configures structure for OPWMT mode
 * param[out] opwmt_param A pointer output configures structure for OPWMT mode
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 */
static status_t EMIOS_DRV_PWM_InitOpwmt(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_pwm_param_t *pwmParam,
                                        emios_opwmt_param_t * opwmt_param);

 /*!
 * brief Initialize Output Pulse Width Modulation with Trigger (OPWMT) Mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] opwmtParam A pointer to the OPWMT configuration structure
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
static status_t EMIOS_DRV_PWM_InitTriggerMode(uint8_t emiosGroup,
                                              uint8_t channel,
                                              const emios_opwmt_param_t *opwmtParam);
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

/*!
 * brief Initialize Output Pulse Width and Frequency Modulation Buffered (OPWFMB) Mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] opwfmParam A pointer to the OPWFM configuration structure
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value. This status only has for MPC5748G and MPC5746C.
 */
static status_t EMIOS_DRV_PWM_InitPeriodDutyCycleMode(uint8_t emiosGroup,
                                                      uint8_t channel,
                                                      const emios_opwfm_param_t *opwfmParam);

/*!
 * brief Initialize Output Pulse Width Modulation Buffered (OPWMB) Mode.
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] opwmbParam A pointer to the OPWMB configuration structure
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
static status_t EMIOS_DRV_PWM_InitEdgePlacementMode(uint8_t emiosGroup,
                                                    uint8_t channel,
                                                    const emios_opwmb_param_t *opwmbParam);

/*!
 * brief Initialize Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] opwmcbParam A pointer to the OPWMCB configuration structure.
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
static status_t EMIOS_DRV_PWM_InitCenterAlignDeadTimeMode(uint8_t emiosGroup,
                                                          uint8_t channel,
                                                          emios_opwmcb_param_t *opwmcbParam);

/*!
 * brief Check setup period value  in OPWFMB, OPWMB and OPWMCB mode.
 *
 * param[in] mode OPWFMB, OPWMB and OPWMCB mode
 * param[in] newPeriod period value to check
 * return true/false
 */
static bool EMIOS_DRV_PWM_SetPeriodCheck(uint32_t mode,
                                         uint32_t newPeriod);

/*******************************************************************************
  * Static functions
  ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetCenterAlignIdealDutyCycle
 * Description   : Set Ideal Duty Cycle value of waveforms in OPWMCB mode. New ideal duty cycle
 * should be greater than 0 and less than or equal EMIOS_OPWMCB_MAX_CNT_VAL.
 * For this mode operation: Ideal Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
   in the eMIOS UC Control register.
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_SetCenterAlignIdealDutyCycle(uint8_t emiosGroup,
                                                           uint8_t channel,
                                                           uint32_t newIdealDutyCycle)
{
    status_t status            = STATUS_SUCCESS;
    uint8_t busSelect          = 0U;
    uint32_t tmpIdealDutyCycle = 0U;
    uint8_t restChannel        = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel);

    /* Validate timebase: must be external counter, MCB Up mode */
    if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_BCDE)
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }
    else
    {
        /* Wrong counter bus */
        status = STATUS_ERROR;
    }

    if (status == STATUS_SUCCESS)
    {
        /* Calibration ideal duty value */
        tmpIdealDutyCycle = EMIOS_DRV_PWM_CalibrationDutyCycle(emiosGroup, busSelect, EMIOS_GetUCRegCEdpol(emiosGroup, restChannel), newIdealDutyCycle);
        /* Set ideal duty value */
        EMIOS_SetUCRegA(emiosGroup, restChannel, tmpIdealDutyCycle);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyOpwfmb
 * Description   : getter, Duty cycle : OPWFMB
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_GetDutyOpwfmb(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t ARegVal,
                                            uint32_t BRegVal)
{
    uint32_t ret;
    uint8_t restChannel = 0U;
    /* Validate channel support */
    (void)EMIOS_ValidateChannel(channel, &restChannel);

    if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
    {
        ret = 0UL; /* 0% duty cycle */
    }
    else if (ARegVal < BRegVal)
    {
        ret = ARegVal;
    }
    else
    {
        /* Duty cycle = period */
        ret = BRegVal; /* 100% duty cycle */
    }

    if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        ret = BRegVal - ret;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyLead
 * Description   : getter, Duty cycle : Leading edge
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_GetDutyLead(uint8_t emiosGroup,
                                          uint8_t busSelect,
                                          uint32_t ARegVal,
                                          uint32_t BRegVal,
                                          uint32_t edpolActive)
{
    uint32_t ret          = 0UL;
    uint8_t restBusSelect = 0U;
    (void)EMIOS_ValidateChannel(busSelect, &restBusSelect); /* Validate bus select support */
    uint32_t tmpARegVal   = EMIOS_GetUCRegA(emiosGroup, restBusSelect);

    if (edpolActive == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        /* Leading edge dead-time insertion */
        if (ARegVal < tmpARegVal)
        {
            if (ARegVal == 1UL)
            {
                ret = (tmpARegVal << 1UL) - 2UL; /* 100% duty cycle */
            }
            else
            {
                if ((tmpARegVal - ARegVal) <= BRegVal)
                {
                    ret = 0UL; /* 0% duty cycle */
                }
                else
                {
                    ret = ((tmpARegVal - ARegVal) << 1UL) - BRegVal;
                }
            }
        }
        else if (ARegVal == tmpARegVal)
        {
            /* Special case Note RM Page 1035/4083 MPC5748G_RM_Rev5_RC */
            ret = (tmpARegVal << 1UL) - 2UL; /* 100% duty cycle */
        }
        else
        {
            ret = 0UL; /* 0% duty cycle */
        }
    }
    else
    {
        /* Leading edge dead-time insertion */
        if (ARegVal < tmpARegVal)
        {
            if (ARegVal <= 1UL)
            {
                ret = 0UL; /* 0% duty cycle */
            }
            else
            {
                if ((ARegVal + BRegVal) > tmpARegVal)
                {
                    ret = ((tmpARegVal - 1UL) << 1UL); /* 100 % duty cycle */
                }
                else
                {
                    ret = ((ARegVal - 1UL) << 1UL) + BRegVal;
                }
            }
        }
        else
        {
            /* Special case Note RM Page 1035/4083 MPC5748G_RM_Rev5_RC */
            ret = ((tmpARegVal - 1UL) << 1UL); /* 100 % duty cycle */
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyTrail
 * Description   : getter, Duty cycle : Trailing edge
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_GetDutyTrail(uint8_t emiosGroup,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal,
                                           uint32_t edpolActive)
{
    uint32_t ret          = 0UL;
    uint8_t restBusSelect = 0U;
    (void)EMIOS_ValidateChannel(busSelect, &restBusSelect); /* Validate bus select support */
    uint32_t tmpARegVal   = EMIOS_GetUCRegA(emiosGroup, restBusSelect);

    if (edpolActive == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        /* Leading edge dead-time insertion */
        if (ARegVal < tmpARegVal)
        {
            if (ARegVal <= 1UL)
            {
                /* 100% duty cycle */
                ret = (tmpARegVal << 1UL) - 2UL;
            }
            else
            {
                if (BRegVal >= ARegVal)
                {
                    /* 100% duty cycle */
                    ret = (tmpARegVal << 1UL) - 2UL;
                }
                else
                {
                    ret = ((tmpARegVal - ARegVal) << 1UL) + BRegVal;
                }
            }
        }
        else if (ARegVal == tmpARegVal)
        {
            /* Special case Note RM Page 1035/4083 MPC5748G_RM_Rev5_RC */
            ret = (tmpARegVal << 1UL) - 2UL; /* 100% duty cycle */
        }
        else
        {
            /* 0% duty cycle */
            ret = 0UL;
        }
    }
    else
    {
        /* Leading edge dead-time insertion */
        if (ARegVal < tmpARegVal)
        {
            if (ARegVal <= 1UL)
            {
                ret = 0UL; /* 0% duty cycle */
            }
            else
            {
                if (BRegVal >= ARegVal)
                {
                    ret = 0UL; /* 0% duty cycle */
                }
                else
                {
                    ret = ((ARegVal - 1UL) << 1UL) - BRegVal;
                }
            }
        }
        else
        {
            /* Special case Note RM Page 1035/4083 MPC5748G_RM_Rev5_RC */
            ret = ((tmpARegVal - 1UL) << 1UL); /* 100& duty cycle */
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyOpwmb
 * Description   : getter, Duty cycle : OPWMB
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_GetDutyOpwmb(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal)
{
    uint32_t ret;
    uint32_t cntPeriod = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect);
    uint8_t restChannel = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel); /* Validate channel support */

    if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
    {
        ret = 0UL; /* 0% duty cycle */
    }
    else if ((ARegVal > cntPeriod) || ((ARegVal == cntPeriod) && (BRegVal == cntPeriod)))
    {
        /* Does not match any edge */
        ret = cntPeriod; /* 100% duty cycle */
    }
    else if ((ARegVal <= cntPeriod) && (BRegVal > cntPeriod))
    {
        /* Leading edge matches only */
        ret = 0UL;
    }
    else if (BRegVal == ARegVal)
    {
        /* Trailing edge matches have precedence over leading edge matches */
        ret = cntPeriod; /* 100% duty cycle */
    }
    else
    {
        ret = cntPeriod - (BRegVal - ARegVal);
    }

    if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        ret = cntPeriod - ret;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetLeadingEdgeCheck
 * Description   : contains specific checks for SetLeadingEdgePlacement
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus.
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_SetLeadingEdgeCheck(uint8_t emiosGroup,
                                                  uint8_t channel,
                                                  uint32_t newLeadingEdgePlacement)
{
    status_t status       = STATUS_SUCCESS;
    uint8_t busSelect     = 0U;
    uint8_t restBusSelect = 0U;
    uint32_t cmode        = 0UL;
    uint8_t restChannel   = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel); /* Validate channel support */
    uint32_t selTimebase = EMIOS_GetUCRegCBsl(emiosGroup, restChannel);

    if (selTimebase == (uint32_t)EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (selTimebase == (uint32_t)EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else if (selTimebase == (uint32_t)EMIOS_BUS_SEL_BCDE)
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }
    else
    {
        /* unknown timebase value */
        status = STATUS_ERROR;
    }

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    if (status == STATUS_SUCCESS)
    {
        (void)EMIOS_ValidateChannel(busSelect, &restBusSelect);

        cmode = EMIOS_GetUCRegCMode(emiosGroup, restChannel);
        if ((((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restBusSelect) & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) && \
                (cmode == (uint32_t)EMIOS_MODE_OPWMT))
        {
            if (newLeadingEdgePlacement < EMIOS_MCB_MIN_CNT_VAL)
            {
                status = STATUS_ERROR;
            }
        }
    }
#endif
    (void)cmode;
    (void)busSelect;
    (void)restBusSelect;
    (void)newLeadingEdgePlacement;
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitOpwmb
 * Description   : initializer, Setup structure : Opwmb
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitOpwmb(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_pwm_param_t *pwmParam,
                                        emios_opwmb_param_t * opwmb_param)
{
    status_t status                    = STATUS_SUCCESS;
    uint8_t busSelect                  = 0U;
    uint8_t restBusSelect              = 0U;
    opwmb_param->mode                  = pwmParam->mode;
    opwmb_param->internalPrescaler     = pwmParam->internalPrescaler;
    opwmb_param->internalPrescalerEn   = pwmParam->internalPrescalerEn;
    opwmb_param->leadingEdgePlacement  = 0UL;
    opwmb_param->trailingEdgePlacement = 0UL;

    if (pwmParam->timebase == EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (pwmParam->timebase == EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else if (pwmParam->timebase == EMIOS_BUS_SEL_BCDE)
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }
    else
    {
        /* unknown timebase value */
        status = STATUS_ERROR;
    }

    if (status == STATUS_SUCCESS)
    {
        if (pwmParam->outputActiveMode == EMIOS_POSITIVE_PULSE)
        {
            opwmb_param->trailingEdgePlacement = pwmParam->dutyCycleCount;
        }
        else
        {
            if (EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) >= pwmParam->dutyCycleCount)
            {
                opwmb_param->trailingEdgePlacement = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) - pwmParam->dutyCycleCount;
            }
            else
            {
                /* duty cycle count should not be greater than period of timebase  */
                status = STATUS_ERROR;
            }
        }

        if (status == STATUS_SUCCESS)
        {
            (void)EMIOS_ValidateChannel(busSelect, &restBusSelect);
            /* If MCB counter mode, this timebase count from 1 */
            if (((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restBusSelect) & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP)
            {
                opwmb_param->leadingEdgePlacement   = 1UL;
                opwmb_param->trailingEdgePlacement += 1UL;
            }
            opwmb_param->timebase              = pwmParam->timebase;
            opwmb_param->outputActiveMode      = pwmParam->outputActiveMode;
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_CalibrationDutyCycle
 * Description   : Calibration duty cycle
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_CalibrationDutyCycle(uint8_t emiosGroup,
                                                   uint8_t channel,
                                                   uint32_t activeMode,
                                                   uint32_t idealDutyCycle)
{
    uint32_t newIdealDutyCycle = 0UL;
    if (activeMode == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        if (idealDutyCycle < EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, channel))
        {
            if (idealDutyCycle > 0U)
            {
                newIdealDutyCycle = ((EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, channel) - idealDutyCycle)/2U) + 1UL;
            }
            else
            {
                newIdealDutyCycle = (EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, channel)/2U) + 2UL; /* 0% duty cycle */
            }
        }
        else
        {
            newIdealDutyCycle = 1UL; /* 100% duty cycle */
        }
    }
    else
    {
        if (idealDutyCycle >= EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, channel))
        {
            newIdealDutyCycle = (EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, channel)/2U) + 2UL; /* 100% duty cycle */
        }
        else
        {
            newIdealDutyCycle = (idealDutyCycle/2U) + 1UL;
        }
    }
    /* New Ideal duty value */
    return newIdealDutyCycle;
}

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyOpwmt
 * Description   : getter, Duty cycle : OPWMT
 *END**************************************************************************/
static uint32_t EMIOS_DRV_PWM_GetDutyOpwmt(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint8_t busSelect,
                                           uint32_t ARegVal,
                                           uint32_t BRegVal)
{
    uint32_t ret;
    uint8_t restChannel = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel); /* Validate channel support */
    /* If the selected Output Disable input signal is asserted for the channel,
       the output pin will go to the inverse of the EDPOL */
    if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
    {
        ret = 0UL; /* 0% duty cycle */
    }
    else if (BRegVal > EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect))
    {
        ret = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect); /* 100% duty cycle */
    }
    else if (BRegVal == ARegVal)
    {
        ret = 0UL;
    }
    else
    {
        ret = BRegVal - ARegVal;
    }

    if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_NEGATIVE_PULSE)
    {
        ret = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) - ret;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitOpwmt
 * Description   : initializer, Setup structure : Opwmt
 * return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, wrong counter bus
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitOpwmt(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_pwm_param_t *pwmParam,
                                        emios_opwmt_param_t * opwmt_param)
{
    status_t status                    = STATUS_SUCCESS;
    uint8_t busSelect                  = 0U;
    uint8_t restBusSelect              = 0U;
    opwmt_param->mode                  = pwmParam->mode;
    opwmt_param->internalPrescaler     = pwmParam->internalPrescaler;
    opwmt_param->internalPrescalerEn   = pwmParam->internalPrescalerEn;
    opwmt_param->leadingEdgePlacement  = 0UL;
    opwmt_param->trailingEdgePlacement = 0UL;

    if (pwmParam->timebase == EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (pwmParam->timebase == EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else if (pwmParam->timebase == EMIOS_BUS_SEL_BCDE)
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }
    else /* Internal bus */
    {
        /* unknown timebase value */
        status = STATUS_ERROR;
    }

    if (status == STATUS_SUCCESS)
    {
        if (pwmParam->outputActiveMode == EMIOS_POSITIVE_PULSE)
        {
            opwmt_param->trailingEdgePlacement = pwmParam->dutyCycleCount;
        }
        else
        {
            if (EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) >= pwmParam->dutyCycleCount)
            {
                opwmt_param->trailingEdgePlacement = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) - pwmParam->dutyCycleCount;
            }
            else
            {
                /* duty cycle count should not be greater than period of timebase  */
                status = STATUS_ERROR;
            }
        }

        if (status == STATUS_SUCCESS)
        {
            (void)EMIOS_ValidateChannel(busSelect, &restBusSelect);
            /* If MCB counter mode, this timebase count from 1 */
            if (((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restBusSelect) & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP)
            {
                opwmt_param->leadingEdgePlacement = 1UL;
                opwmt_param->trailingEdgePlacement += 1UL;
            }

            opwmt_param->outputActiveMode      = pwmParam->outputActiveMode;
            opwmt_param->timebase              = pwmParam->timebase;
            opwmt_param->triggerEventPlacement = pwmParam->triggerEventPlacement;
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitTriggerMode
 * Description   : Initial Output Pulse Width Modulation with Trigger (OPWMT) Mode
 * OPWMT mode is intended to support the generation of Pulse Width Modulation signals where
 * the period is not modified while the signal is being output, but where the duty cycle will
 * be varied and must not create glitches. The mode is intended to be used in conjunction with
 * other channels executing in the same mode and sharing a common timebase. It will support each
 * channel with a fixed PWM leading edge position with respect to the other channels and the
 * ability to generate a trigger signal at any point in the period that can be output from
 * the module to initiate activity in other parts of the SoC such as starting ADC conversions.
 *
 * An external counter driven in either MC Up or MCB Up mode must be selected from one of the counter buses.
 *
 * The leading edge can be configured with any value within the range of the selected time base. Note that registers
 * loaded with 0x0 will not produce matches if the timebase is driven by a channel in MCB mode.
 *
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitTriggerMode(uint8_t emiosGroup,
                                              uint8_t channel,
                                              const emios_opwmt_param_t *opwmtParam)
{
    status_t status           = STATUS_SUCCESS;
    uint8_t selCounterBus     = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    uint8_t temp              = 0U;
    uint8_t restChannel       = 0U;
    uint8_t restSelCounterBus = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel);
    /* Validate timebase: must be external counter, MC or MCB Up mode */
    if (opwmtParam->timebase == EMIOS_BUS_SEL_A)
    {
        selCounterBus = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (opwmtParam->timebase == EMIOS_BUS_SEL_F)
    {
        selCounterBus = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else /* Local bus */
    {
        selCounterBus = (uint8_t)(channel & 0xF8U);
    }

    (void)EMIOS_ValidateChannel(selCounterBus, &restSelCounterBus);
    temp = (uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restSelCounterBus);

    /* Validate opwmt parameter */
    if ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP)
    {
        DEV_ASSERT((opwmtParam->leadingEdgePlacement >= EMIOS_MCB_MIN_CNT_VAL) && \
                  (opwmtParam->trailingEdgePlacement >= EMIOS_MCB_MIN_CNT_VAL));
    }
    else if ((temp & EMIOS_FILTER_MC) != EMIOS_MASK_MC_UP)
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Do nothing */
    }

    if (status == STATUS_SUCCESS)
    {
        DEV_ASSERT(opwmtParam->triggerEventPlacement <= EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, selCounterBus));
        /* Cleared UC configure registers */
        eMIOS[emiosGroup]->UC[restChannel].C = 0UL;                                         /* Disable channel pre-scaler (reset default) */
        EMIOS_SetUCRegA(emiosGroup, restChannel, opwmtParam->leadingEdgePlacement);
        EMIOS_SetUCRegB(emiosGroup, restChannel, opwmtParam->trailingEdgePlacement);
        EMIOS_SetUCRegALTA(emiosGroup, restChannel, opwmtParam->triggerEventPlacement);
        EMIOS_SetUCRegCBsl(emiosGroup, restChannel, (uint32_t)opwmtParam->timebase);
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)opwmtParam->outputActiveMode);
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)opwmtParam->mode);
        EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, opwmtParam->internalPrescalerEn ? 1UL: 0UL);
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, restChannel, (uint32_t)opwmtParam->internalPrescaler);   /* Pre-scale channel clock by internalPrescaler +1 */
        EMIOS_SetUCRegC2UCPRECLK(emiosGroup, restChannel, 0UL);                                       /* Prescaler clock selected */
#else
        EMIOS_SetUCRegCUcpre(emiosGroup, restChannel, (uint32_t)opwmtParam->internalPrescaler);          /* Pre-scale channel clock by internalPrescaler +1 */
#endif
    }

    return status;
}
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitPeriodDutyCycleMode
 * Description   : Initial Output Pulse Width and Frequency Modulation Buffered (OPWFMB) Mode.
 * This mode provides waveforms with variable duty cycle and frequency. The internal channel counter
 * is automatically selected as the time base when this mode is selected.
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitPeriodDutyCycleMode(uint8_t emiosGroup,
                                                      uint8_t channel,
                                                      const emios_opwfm_param_t *opwfmParam)
{
    status_t status     = STATUS_SUCCESS;
    uint8_t restChannel = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel);

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid Opwfmb with channels supported */
    if (EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_OPWFMB) == false)
    {
         status = STATUS_ERROR;
    }
    else
#endif /* FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Cleared UC configure registers */
        eMIOS[emiosGroup]->UC[restChannel].C = 0UL; /* Disable channel pre-scaler (reset default) */
        if (opwfmParam->outputActiveMode == EMIOS_NEGATIVE_PULSE)
        {
            EMIOS_SetUCRegA(emiosGroup, restChannel, opwfmParam->dutyCycleCount);
        }
        else
        {
            EMIOS_SetUCRegA(emiosGroup, restChannel, opwfmParam->periodCount - opwfmParam->dutyCycleCount);
        }

        EMIOS_SetUCRegB(emiosGroup, restChannel, opwfmParam->periodCount);
        EMIOS_SetUCRegCNT(emiosGroup, restChannel, 1UL);
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)opwfmParam->outputActiveMode);
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)opwfmParam->mode);
        EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, opwfmParam->internalPrescalerEn ? 1UL: 0UL);
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, restChannel, (uint32_t)opwfmParam->internalPrescaler);  /* Pre-scale channel clock by internalPrescaler +1 */
        EMIOS_SetUCRegC2UCPRECLK(emiosGroup, restChannel, 0UL);                                      /* Prescaler clock selected*/
#else
        EMIOS_SetUCRegCUcpre(emiosGroup, restChannel, (uint32_t)opwfmParam->internalPrescaler);         /* Pre-scale channel clock by internalPrescaler +1 */
#endif
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitEdgePlacementMode
 * Description   : Initial Output Pulse Width Modulation Buffered (OPWMB) Mode.
 * OPWMB mode is used to generate pulses with programmable leading and trailing edge placement.
 * An external counter driven in MCB Up mode must be selected from one of the counter buses.
 * opwmbParam defines the first edge and the second edge. The output signal polarity is defined
 * by outputActiveMode in opwmbParam.
 *
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitEdgePlacementMode(uint8_t emiosGroup,
                                                    uint8_t channel,
                                                    const emios_opwmb_param_t *opwmbParam)
{
    uint8_t restChannel   = 0U;
    status_t ret          = STATUS_SUCCESS;
    uint8_t  busSelect    = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    uint8_t restBusSelect = 0U;
    (void)EMIOS_ValidateChannel(channel, &restChannel);
    /* Validate timebase: must be external counter, MCB Up mode */
    if (opwmbParam->timebase == EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (opwmbParam->timebase == EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else /* Local bus */
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }

    (void)EMIOS_ValidateChannel(busSelect, &restBusSelect);
    if (((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restBusSelect) & EMIOS_FILTER_MCB) != EMIOS_MASK_MCB_UP)
    {
        /* Wrong mode */
        ret = STATUS_ERROR;
    }
    else
    {
        /* Cleared UC configure registers */
        eMIOS[emiosGroup]->UC[restChannel].C = 0UL;                                        /* Disable channel pre-scaler (reset default) */
        EMIOS_SetUCRegA(emiosGroup, restChannel, opwmbParam->leadingEdgePlacement);
        EMIOS_SetUCRegB(emiosGroup, restChannel, opwmbParam->trailingEdgePlacement);
        EMIOS_SetUCRegCBsl(emiosGroup, restChannel, (uint32_t)opwmbParam->timebase);
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)opwmbParam->outputActiveMode);
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)opwmbParam->mode);
        EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, opwmbParam->internalPrescalerEn ? 1UL : 0UL);
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, restChannel, (uint32_t)opwmbParam->internalPrescaler);  /* Pre-scale channel clock by internalPrescaler +1 */
        EMIOS_SetUCRegC2UCPRECLK(emiosGroup, restChannel, 0UL);                                      /* Prescaler clock selected */
#else
        EMIOS_SetUCRegCUcpre(emiosGroup, restChannel, (uint32_t)opwmbParam->internalPrescaler);         /* Pre-scale channel clock by internalPrescaler +1 */
#endif
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitCenterAlignDeadTimeMode
 * Description   : Initial Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode.
 * This operation mode generates a center aligned PWM with dead time insertion to the leading or trailing edge.
 * Allow smooth output signal generation when changing duty cycle and deadtime values.
 *
 * The time base selected for a channel configured to OPWMCB mode should be a channel configured to MCB Up/Down mode.
 * It is recommended to start the MCB channel time base after the OPWMCB mode is entered
 * in order to avoid missing A matches at the very first duty cycle.
 *
 * The internal counter runs in the internal prescaler ratio, while the selected time base
 * may be running in a different prescaler ratio.
 *
 *END**************************************************************************/
static status_t EMIOS_DRV_PWM_InitCenterAlignDeadTimeMode(uint8_t emiosGroup,
                                                          uint8_t channel,
                                                          emios_opwmcb_param_t *opwmcbParam)
{
    status_t status       = STATUS_SUCCESS;
    uint8_t restChannel   = 0U;
    uint8_t restBusSelect = 0U;
    uint8_t busSelect     = 0U;
    uint8_t mode          = 0U;
    uint32_t newDutyCycle = 0UL;

    /* Check channel */
    (void)EMIOS_ValidateChannel(channel, &restChannel);

    /* Validate timebase: must be external counter, MCB Up mode */
    if (opwmcbParam->timebase == EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (opwmcbParam->timebase == EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else if (opwmcbParam->timebase == EMIOS_BUS_SEL_BCDE)
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }
    else
    {
        /* Wrong counter bus : choosing internal counter bus ? */
        status = STATUS_ERROR;
    }

    if (status == STATUS_SUCCESS)
    {
        (void)EMIOS_ValidateChannel(busSelect, &restBusSelect);
        mode = (uint8_t)((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restBusSelect) & EMIOS_FILTER_MCB);

    #ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
        /* Valid Opwfmb with channels supported */
        if (EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_OPWMCB) == false)
        {
            /* Channel not support for this mode */
             status = STATUS_ERROR;
        }
        else
    #endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
        {
            /* Check bus select and valid Opwmcb with channels supported */
            if (mode != EMIOS_MASK_MCB_UPDOWN)
            {
                /* Wrong counter bus */
                status = STATUS_ERROR;
            }
            else
            {
                /* Calibration ideal duty value */
                newDutyCycle = EMIOS_DRV_PWM_CalibrationDutyCycle(emiosGroup, busSelect, (uint32_t)opwmcbParam->outputActiveMode, opwmcbParam->idealDutyCycle);
                /* Cleared UC configure registers */
                eMIOS[emiosGroup]->UC[restChannel].C = 0UL;                                        /* Disable channel pre-scaler (reset default) */
                EMIOS_SetUCRegA(emiosGroup, restChannel, newDutyCycle);
                EMIOS_SetUCRegB(emiosGroup, restChannel, opwmcbParam->deadTime);
                EMIOS_SetUCRegCBsl(emiosGroup, restChannel, (uint32_t)opwmcbParam->timebase);
                EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)opwmcbParam->outputActiveMode);
                EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)opwmcbParam->mode);
                EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, opwmcbParam->internalPrescalerEn ? 1UL: 0UL);
            #if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
                EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, restChannel, (uint32_t)opwmcbParam->internalPrescaler); /* Pre-scale channel clock by internalPrescaler +1 */
                EMIOS_SetUCRegC2UCPRECLK(emiosGroup, restChannel, 0UL);                                      /* Prescaler clock selected */
            #else
                EMIOS_SetUCRegCUcpre(emiosGroup, restChannel, (uint32_t)opwmcbParam->internalPrescaler);         /* Pre-scale channel clock by internalPrescaler +1 */
            #endif
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetPeriodCheck
 * Description   : Check setup period value  in OPWFMB, OPWMB and OPWMCB mode.
 * Return true/false.
 *END**************************************************************************/
static bool EMIOS_DRV_PWM_SetPeriodCheck(uint32_t mode,
                                         uint32_t newPeriod)
{
    bool tmpCheck = true;

    switch (mode)
    {
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2:
            if ((newPeriod > EMIOS_OPWFMB_MAX_CNT_VAL) || \
                (newPeriod <= EMIOS_OPWFMB_MIN_CNT_VAL))
            {
                tmpCheck = false;
            }
            break;
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
            if ((newPeriod > EMIOS_OPWMCB_MAX_CNT_VAL) || \
                (newPeriod <= EMIOS_OPWMCB_MIN_CNT_VAL))
            {
                tmpCheck = false;
            }
            break;
        default :
            /* Mode not supported */
            tmpCheck = false;
            break;
    }

    return tmpCheck;
}

/******************************************************************************
* API
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_InitMode
 * Description   : Initialize PWM mode.
 * For all mode operation: Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
   in the eMIOS UC Control register.
 * Select main mode
 *     - EMIOS OPWFMB
 *       Initialize Output Pulse Width and Frequency Modulation Buffered (OPWFMB) Mode.
 *       This mode provides waveforms with variable duty cycle and frequency. The internal channel counter
 *       is automatically selected as the time base when this mode is selected.
 *
 *     - EMIOS OPWMB
 *       Initialize Output Pulse Width Modulation Buffered (OPWMB) Mode.
 *       OPWMB mode is used to generate pulses with programmable leading and trailing edge placement.
 *       An external counter driven in MCB Up mode must be selected from one of the counter buses.
 *       opwmbParam defines the first edge and the second edge. The output signal polarity is defined
 *       by outputActiveMode in opwmbParam.
 *
 *     - EMIOS OPWMCB
 *       Initialize Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode.
 *       This operation mode generates a center aligned PWM with dead time insertion to the leading or trailing edge.
 *       Allow smooth output signal generation when changing duty cycle and deadtime values.
 *
 *     - EMIOS OPWMT
 *       Initialize Output Pulse Width Modulation with Trigger (OPWMT) Mode
 *       OPWMT mode is intended to support the generation of Pulse Width Modulation signals where
 *       the period is not modified while the signal is being output, but where the duty cycle will
 *       be varied and must not create glitches. The mode is intended to be used in conjunction with
 *       other channels executing in the same mode and sharing a common timebase. It will support each
 *       channel with a fixed PWM leading edge position with respect to the other channels and the
 *       ability to generate a trigger signal at any point in the period that can be output from
 *       the module to initiate activity in other parts of the SoC such as starting ADC conversions.
 * Implements    : EMIOS_DRV_PWM_InitMode_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_PWM_InitMode(uint8_t emiosGroup,
                                uint8_t channel,
                                const emios_pwm_param_t *pwmParam)
{
    status_t ret        = STATUS_SUCCESS;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(pwmParam != NULL);
    DEV_ASSERT(eMIOS[emiosGroup]->UC[restChannel].C == 0UL); /* Check that device was initialized */
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    /* Enabled output update for channel */
    EMIOS_DRV_EnableAChOutputUpdate(emiosGroup, channel);
    /* Check mode to initialization */
    switch (pwmParam->mode)
    {
        case EMIOS_MODE_OPWFMB_FLAGX1: /* OPWFMB mode */
        case EMIOS_MODE_OPWFMB_FLAGX2:
        {
            emios_opwfm_param_t opwfm_param;
            opwfm_param.mode                = pwmParam->mode;
            opwfm_param.dutyCycleCount      = pwmParam->dutyCycleCount;
            opwfm_param.internalPrescaler   = pwmParam->internalPrescaler;
            opwfm_param.internalPrescalerEn = pwmParam->internalPrescalerEn;
            opwfm_param.outputActiveMode    = pwmParam->outputActiveMode;
            opwfm_param.periodCount         = pwmParam->periodCount;
            /* Validate opwfmb parameter */
            DEV_ASSERT((opwfm_param.periodCount <= EMIOS_OPWFMB_MAX_CNT_VAL) && \
                       (opwfm_param.periodCount > EMIOS_OPWFMB_MIN_CNT_VAL));
            DEV_ASSERT(opwfm_param.dutyCycleCount <= opwfm_param.periodCount);

            ret = EMIOS_DRV_PWM_InitPeriodDutyCycleMode(emiosGroup, channel, &opwfm_param);
        }
            break;
        case EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1: /* OPWMCB mode */
        case EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
        {
            emios_opwmcb_param_t opwmcb_param;
            opwmcb_param.mode                = pwmParam->mode;
            opwmcb_param.deadTime            = pwmParam->deadTime;
            opwmcb_param.idealDutyCycle      = pwmParam->idealDutyCycle;
            opwmcb_param.internalPrescaler   = pwmParam->internalPrescaler;
            opwmcb_param.internalPrescalerEn = pwmParam->internalPrescalerEn;
            opwmcb_param.outputActiveMode    = pwmParam->outputActiveMode;
            opwmcb_param.timebase            = pwmParam->timebase;
            /* Validate opwmcb parameter */
            DEV_ASSERT((opwmcb_param.idealDutyCycle <= EMIOS_OPWMCB_MAX_CNT_VAL) && \
                       (opwmcb_param.deadTime <= EMIOS_OPWMCB_MAX_CNT_VAL));

            ret = EMIOS_DRV_PWM_InitCenterAlignDeadTimeMode(emiosGroup, channel, &opwmcb_param);
        }
            break;
        case EMIOS_MODE_OPWMB_FLAGX1: /* OPWMB mode */
        case EMIOS_MODE_OPWMB_FLAGX2:
        {
            emios_opwmb_param_t opwmb_param;
            if (EMIOS_DRV_PWM_InitOpwmb(emiosGroup, channel, pwmParam, &opwmb_param) == STATUS_SUCCESS)
            {
                /* Validate opwmb parameter */
                DEV_ASSERT((opwmb_param.leadingEdgePlacement <= opwmb_param.trailingEdgePlacement) && \
                           (opwmb_param.trailingEdgePlacement <= EMIOS_DATA_REG_MAX_VAL));

                ret = EMIOS_DRV_PWM_InitEdgePlacementMode(emiosGroup, channel, &opwmb_param);
            }
            else
            {
                ret = STATUS_ERROR; /* Operation failed, wrong counter bus */
            }
        }
            break;
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
        case EMIOS_MODE_OPWMT: /* OPWMT mode */
        {
            emios_opwmt_param_t opwmt_param;
            if (EMIOS_DRV_PWM_InitOpwmt(emiosGroup, channel, pwmParam, &opwmt_param) == STATUS_SUCCESS)
            {
                DEV_ASSERT((opwmt_param.trailingEdgePlacement >= opwmt_param.leadingEdgePlacement) && \
                           (opwmt_param.trailingEdgePlacement <= (uint32_t)EMIOS_DATA_REG_MAX_VAL));

                ret = EMIOS_DRV_PWM_InitTriggerMode(emiosGroup, channel, &opwmt_param);
            }
            else
            {
                ret = STATUS_ERROR; /* Operation failed, wrong counter bus */
            }
        }
            break;
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */
        default :
        {
            /* Main mode selected is wrong */
            ret = STATUS_ERROR;
        }
            break;
    } /* switch (pwmParam->mode) */

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_ForcePWMMatchLeadingEdge
 * Description   : Force the output flipflop to the level corresponding to a match on Leading edge
 *
 * In Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode
 *      FORCMA has different behaviors depending upon the selected dead time
 *      insertion mode, lead or trail. In lead dead time insertion FORCMA force a transition
 *      in the output flipflop to the opposite of EDPOL. In trail dead time insertion the
 *      output flip-flop is forced to the value of EDPOL bit.
 *      FORCMA bit set does not set the internal time-base to 0x1 as a regular A1 match.
 * Implements    : EMIOS_DRV_PWM_ForcePWMMatchLeadingEdge_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_ForcePWMMatchLeadingEdge(uint8_t emiosGroup,
                                            uint8_t channel)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               ((temp & (uint32_t)EMIOS_FILTER_OPWMCB) == (uint32_t)EMIOS_MASK_OPWMCB) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2));
#else
    DEV_ASSERT(((temp & (uint32_t)EMIOS_FILTER_OPWMCB) == (uint32_t)EMIOS_MASK_OPWMCB) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2));
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */
#ifndef DEV_ERROR_DETECT
    (void)temp;
#endif

    EMIOS_SetUCRegCForcma(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_ForcePWMMatchTrailingEdge
 * Description   : Force the output flipflop to the level corresponding to a match on Trailing edge
 *
 * In Center Aligned Output Pulse Width Modulation with Dead Time Insertion Buffered (OPWMCB) Mode
 *       If FORCMB bit is set, the output flip-flop value depends upon the selected dead time
 *       insertion mode. In lead dead time insertion FORCMB forces the output flip-flop to transition to EDPOL
 *       bit value. In trail dead time insertion the output flip-flop is forced to the opposite of EDPOL bit value.
 * Implements    : EMIOS_DRV_PWM_ForcePWMMatchTrailingEdge_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_ForcePWMMatchTrailingEdge(uint8_t emiosGroup,
                                             uint8_t channel)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               (((uint8_t)temp & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2));
#else
    DEV_ASSERT((((uint8_t)temp & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2));
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */
#ifndef DEV_ERROR_DETECT
    (void)temp;
#endif

    EMIOS_SetUCRegCForcmb(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetPeriod
 * Description   : Get period of waveforms in PWM mode.
 * Implements    : EMIOS_DRV_PWM_GetPeriod_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetPeriod(uint8_t emiosGroup,
                                 uint8_t channel)
{
    uint32_t ret        = 0UL;
    uint32_t temp       = 0UL;
    uint8_t busSelect   = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    uint32_t cBusSelect = 0UL;
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    switch (temp)
    {
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2:          /* OPWFMB mode */
            ret = EMIOS_GetUCRegB(emiosGroup, restChannel);
            break;
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
        case (uint32_t)EMIOS_MODE_OPWMT:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX2: /* OPWMCB mode or OPWMB or OPWMT mode */
#else
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX2: /* OPWMCB mode or OPWMB mode */
#endif
            /* Get current bus select */
            cBusSelect = EMIOS_GetUCRegCBsl(emiosGroup, restChannel);
            switch (cBusSelect)
            {
                case (uint32_t)EMIOS_BUS_SEL_A:
                    busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
                    break;
#if FEATURE_EMIOS_BUS_F_SELECT
                case (uint32_t)EMIOS_BUS_SEL_F:
                    busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
                    break;
#endif
                case (uint32_t)EMIOS_BUS_SEL_BCDE:
                    busSelect = (uint8_t)(channel & 0xF8U);
                    break;
                default :
                    /* Unsupported for OPWMB mode */
                    DEV_ASSERT(((temp != (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) && (temp != (uint32_t)EMIOS_MODE_OPWMB_FLAGX2)));
                    break;
            }
            /* Get counter period */
            ret = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect);
            break;
        default :
            /* Did not support mode */
            DEV_ASSERT(false);
            break;
    }

    (void)cBusSelect;
    /* Return period value */
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetPeriod
 * Description   : Setup period of waveforms in OPWFMB, OPWMB and OPWMCB mode.
 * With PWM mode, the OPWFMB(Output Pulse Width and Frequency Modulation Buffered), OPWMB or OPWMCB can set
 * period. All other PWM modes can not, because OPWFMB mode using internal counter. All other
 * modes use external timebase and their period is timebase period.
 * Implements    : EMIOS_DRV_PWM_SetPeriod_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_SetPeriod(uint8_t emiosGroup,
                             uint8_t channel,
                             uint32_t newPeriod)
{
    uint8_t restChannel   = 0U;
    uint8_t busSelect     = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    uint32_t modeSelected = 0UL;
    bool restValidate     = EMIOS_ValidateChannel(channel, &restChannel);
    uint32_t mode         = EMIOS_GetUCRegCMode(emiosGroup, restChannel);
    bool restMode         = EMIOS_DRV_PWM_SetPeriodCheck(mode, newPeriod);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
    DEV_ASSERT(restMode == true);
#else
    (void)restMode;
    (void)restValidate;
#endif
    /* Check PWM mode */
    switch (mode)
    {
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2:
            EMIOS_SetUCRegB(emiosGroup, restChannel, newPeriod);
            break;
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
            /* Check bus selected */
            modeSelected = EMIOS_GetUCRegCBsl(emiosGroup, restChannel);
            switch (modeSelected)
            {
                case (uint32_t)EMIOS_BUS_SEL_A:
                    busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
                    break;
#if FEATURE_EMIOS_BUS_F_SELECT
                case (uint32_t)EMIOS_BUS_SEL_F:
                    busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
                    break;
#endif
                case (uint32_t)EMIOS_BUS_SEL_BCDE:
                    busSelect = (uint8_t)(channel & 0xF8U);
                    break;
                default : /* Internal bus select */
                    busSelect = channel;
                    /* Unsupported for OPWMB mode */
                    DEV_ASSERT(((mode != (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) && (mode != (uint32_t)EMIOS_MODE_OPWMB_FLAGX2)));
                    break;
            }
            /* Set counter period */
            (void)EMIOS_DRV_MC_SetCounterPeriod(emiosGroup, busSelect, newPeriod);
            break;
        default :
            /* Do not support this mode */
            DEV_ASSERT(false);
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetDutyCycle
 * Description   : Get duty cycle of waveforms in PWM mode. Duty cycle is time ON in a period,
 *                 this value may be difference with duty value configured previously if prescaler ratio of the
 *                 timebase counter bus difference prescaler ratio of the internal counter.
 * Implements    : EMIOS_DRV_PWM_GetDutyCycle_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetDutyCycle(uint8_t emiosGroup,
                                    uint8_t channel)
{
    uint32_t ret        = 0UL;
    uint8_t busSelect   = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    uint32_t cmode      = 0UL;
    uint32_t ARegVal    = 0UL;
    uint32_t BRegVal    = 0UL;
    uint8_t restChannel = 0U;
    uint32_t cBusSelect = 0UL;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    /* Get currently mode */
    cmode      = EMIOS_GetUCRegCMode(emiosGroup, restChannel);
    ARegVal    = EMIOS_GetUCRegA(emiosGroup, restChannel);
    BRegVal    = EMIOS_GetUCRegB(emiosGroup, restChannel);
    cBusSelect = EMIOS_GetUCRegCBsl(emiosGroup, restChannel);
    /* Check current bus selected */
    switch (cBusSelect)
    {
        case (uint32_t)EMIOS_BUS_SEL_A:
            busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
            break;
#if FEATURE_EMIOS_BUS_F_SELECT
        case (uint32_t)EMIOS_BUS_SEL_F:
            busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
            break;
#endif
        case (uint32_t)EMIOS_BUS_SEL_BCDE:
            busSelect = (uint8_t)(channel & 0xF8U);
            break;
        default : /* Internal bus unsupported for OPWMB mode */
            DEV_ASSERT(((cmode != (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) && (cmode != (uint32_t)EMIOS_MODE_OPWMB_FLAGX2)));
            break;
    }
    /* Check current PWM mode */
    switch (cmode)
    {
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2:
            ret = EMIOS_DRV_PWM_GetDutyOpwfmb(emiosGroup, channel, ARegVal, BRegVal);
            break;
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
            if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
            {
                /* 0% duty cycle, The output disable feature, if enabled, causes the output flip-flop to transition to the
                   EDPOL inverted state, RM Page 1036/4083 MPC5748G_RM_Rev5_RC */
                ret = 0UL;
            }
            else if ((cmode == (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1) || \
                     (cmode == (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2))
            {
                ret = EMIOS_DRV_PWM_GetDutyLead(emiosGroup, busSelect, ARegVal, BRegVal, EMIOS_GetUCRegCEdpol(emiosGroup, restChannel));
            }
            else /* Trailing edge dead-time insertion */
            {
                ret = EMIOS_DRV_PWM_GetDutyTrail(emiosGroup, busSelect, ARegVal, BRegVal, EMIOS_GetUCRegCEdpol(emiosGroup, restChannel));
            }
            break; /* End of OPWMCB mode */
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX2:
            ret = EMIOS_DRV_PWM_GetDutyOpwmb(emiosGroup, channel, busSelect, ARegVal, BRegVal);
            break;
#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
        case (uint32_t)EMIOS_MODE_OPWMT:
            ret = EMIOS_DRV_PWM_GetDutyOpwmt(emiosGroup, channel, busSelect, ARegVal, BRegVal);
            break;
#endif
        default : /* Unsupported this mode */
            DEV_ASSERT(false);
            break;
    }
    /* Return duty cycle value */
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetDutyCycle
 * Description   : Setup duty cycle of waveforms in OPWFMB, OPWMB, OPWMCB mode.
 * For OPWFMB mode operation: Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
 * in the eMIOS UC Control register.
 * Duty cycle should be not greater period value. When set duty cycle value greater period value
 * (and do not over 16 bits counter register) 100% duty cycle signal generated.

 * For OPWMCB mode operation: Set Ideal Duty Cycle value of waveforms in OPWMCB mode. New ideal duty cycle
 * should be no greater than 0xFFFF (16 bits) and greater than 0.
 * For this mode operation: Ideal Duty cycle always is high level percentage of the period and it does not depend by value of the Edge Polarity bit
   in the eMIOS UC Control register.

 * Implements    : EMIOS_DRV_PWM_SetDutyCycle_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_PWM_SetDutyCycle(uint8_t emiosGroup,
                                    uint8_t channel,
                                    uint32_t newDutyCycle)
{
    status_t ret        = STATUS_SUCCESS;
    uint8_t restChannel = 0U;
    uint8_t busSelect   = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    uint32_t mode       = EMIOS_GetUCRegCMode(emiosGroup, restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(newDutyCycle <= EMIOS_DATA_REG_MAX_VAL);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    /* Check PWM mode */
    switch (mode)
    {
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX1: /* OPWFMB mode */
        case (uint32_t)EMIOS_MODE_OPWFMB_FLAGX2:
        {
            /* Check output update disabled */
            if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
            {
                ret = STATUS_ERROR;
            }
            else
            {
                if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_NEGATIVE_PULSE)
                {
                    if (newDutyCycle < EMIOS_GetUCRegB(emiosGroup, restChannel))
                    {
                        EMIOS_SetUCRegA(emiosGroup, restChannel, newDutyCycle);
                    }
                    else
                    {
                        /* new duty cycle >= period */
                        EMIOS_SetUCRegA(emiosGroup, restChannel, EMIOS_GetUCRegB(emiosGroup, restChannel)); /* 100% duty cycle */
                    }
                }
                else
                {
                    if (newDutyCycle < EMIOS_GetUCRegB(emiosGroup, restChannel))
                    {
                        EMIOS_SetUCRegA(emiosGroup, restChannel, EMIOS_GetUCRegB(emiosGroup, restChannel) - newDutyCycle);
                    }
                    else
                    {
                        /* new duty cycle >= period */
                        EMIOS_SetUCRegA(emiosGroup, restChannel, 0UL); /* 100% duty cycle */
                    }
                }
            } /* End of check output update disabled */
        }
            break; /* End of OPWFMB mode */
        case (uint32_t)EMIOS_MODE_OPWMB_FLAGX1: /* OPWMB mode */
        {
            if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_A)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
            }
#if FEATURE_EMIOS_BUS_F_SELECT
            else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_F)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
            }
#endif
            else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_BCDE)
            {
                busSelect = (uint8_t)(channel & 0xF8U);
            }
            else
            {
                /* Not support internal bus */
                DEV_ASSERT(false);
            }
            /* Check output update disabled */
            if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, channel) == true)
            {
                ret = STATUS_ERROR;
            }
            else
            {
                ret = EMIOS_DRV_PWM_SetLeadingEdgePlacement(emiosGroup, channel, 1UL);
            }

            if (ret == STATUS_SUCCESS)
            {
                if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_NEGATIVE_PULSE)
                {
                    if (newDutyCycle < EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect))
                    {
                        EMIOS_DRV_PWM_SetTrailingEdgePlacement(emiosGroup, channel, ((EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) + 1UL) - newDutyCycle));
                    }
                    else /* new duty cycle >= period */
                    {
                        EMIOS_DRV_PWM_SetTrailingEdgePlacement(emiosGroup, channel, 1UL); /* 100% duty cycle */
                    }
                }
                else
                {
                    if (newDutyCycle < EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect))
                    {
                        EMIOS_DRV_PWM_SetTrailingEdgePlacement(emiosGroup, channel, (newDutyCycle + 1UL));
                    }
                    else /* new duty cycle >= period */
                    {
                        EMIOS_DRV_PWM_SetTrailingEdgePlacement(emiosGroup, channel, (EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect) + 1UL)); /* 100% duty cycle */
                    }
                }
            }
        }
            break; /* End of OPWMB mode */
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX1: /* OPWMCB mode */
        case (uint32_t)EMIOS_MODE_OPWMCB_TRAIL_EDGE_DEADTIME_FLAGX2:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX1:
        case (uint32_t)EMIOS_MODE_OPWMCB_LEAD_EDGE_DEADTIME_FLAGX2:
        {
            /* Validate opwmcb mode */
            DEV_ASSERT(((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restChannel) & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB);
            DEV_ASSERT(newDutyCycle <= EMIOS_OPWMCB_MAX_CNT_VAL);

            if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_A)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
            }
#if FEATURE_EMIOS_BUS_F_SELECT
            else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_F)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
            }
#endif
            else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_BCDE)
            {
                busSelect = (uint8_t)(channel & 0xF8U);
            }
            else
            {
                busSelect = channel;
            }
            /* Check output update disabled */
            if (EMIOS_DRV_IsOutputUpdateDisabled(emiosGroup, busSelect) == true)
            {
                ret = STATUS_ERROR;
            }
            else
            {
                ret = EMIOS_DRV_PWM_SetCenterAlignIdealDutyCycle(emiosGroup, channel, newDutyCycle);
            }
        }
            break; /* End of OPWMCB mode */
        default:
        {
            /* Unsupported this mode */
            ret = STATUS_ERROR;
        }
            break;
    }
    /* return  status result */
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetLeadingEdgePlacement
 * Description   : Get Leading edge position of waveforms in OPWMB & OPWMT mode.
 * Implements    : EMIOS_DRV_PWM_GetLeadingEdgePlacement_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetLeadingEdgePlacement(uint8_t emiosGroup,
                                               uint8_t channel)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#else
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */
#ifndef DEV_ERROR_DETECT
    (void)temp;
#endif

    return EMIOS_GetUCRegA(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetLeadingEdgePlacement
 * Description   : Set Leading edge position of waveforms in OPWMB & OPWMT mode.
 * With OPWMB mode when user write leading edge position is 0, then generating a 0 % duty cycle
 * signal (EDPOL = 0) or 100 % duty cycle (EDPOL = 1)
 * Implements    : EMIOS_DRV_PWM_SetLeadingEdgePlacement_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_PWM_SetLeadingEdgePlacement(uint8_t emiosGroup,
                                               uint8_t channel,
                                               uint32_t newLeadingEdgePlacement)
{
    status_t status     = STATUS_SUCCESS;
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#else
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#endif
#ifndef DEV_ERROR_DETECT
    (void)temp;
#endif
    /* Validate opwmb parameter */
    if (newLeadingEdgePlacement <= EMIOS_GetUCRegB(emiosGroup, restChannel))
    {
        status = EMIOS_DRV_PWM_SetLeadingEdgeCheck(emiosGroup, channel, newLeadingEdgePlacement);
        if (status == STATUS_SUCCESS)
        {
            EMIOS_SetUCRegA(emiosGroup, restChannel, newLeadingEdgePlacement);
        }
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetTrailingEdgePlacement
 * Description   : Get Trailing edge position of waveforms in OPWMB & OPWMT mode.
 * Implements    : EMIOS_DRV_PWM_GetTrailingEdgePlacement_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetTrailingEdgePlacement(uint8_t emiosGroup,
                                                uint8_t channel)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#else
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#endif
#ifndef DEV_ERROR_DETECT
    (void)temp;
#endif

    return EMIOS_GetUCRegB(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetTrailingEdgePlacement
 * Description   : Set Trailing edge position of waveforms in OPWMB & OPWMT mode.
 * When set Trailing edge to a value greater than maximum value of the selected time base,
 * a 100 % duty cycle(EDEPOL = 1) or 0 % duty cycle( EDEPOL = 0) signal generated.
 * Implements    : EMIOS_DRV_PWM_SetTrailingEdgePlacement_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_SetTrailingEdgePlacement(uint8_t emiosGroup,
                                            uint8_t channel,
                                            uint32_t newTrailingEdgePlacement)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMT) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#else
    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX1) || \
               (temp == (uint32_t)EMIOS_MODE_OPWMB_FLAGX2));
#endif
#ifndef DEV_ERROR_DETECT
    (void) temp;
#endif

    /* Validate parameter */
    DEV_ASSERT(newTrailingEdgePlacement <= EMIOS_DATA_REG_MAX_VAL);
    DEV_ASSERT(newTrailingEdgePlacement >= EMIOS_GetUCRegA(emiosGroup, restChannel));

    EMIOS_SetUCRegB(emiosGroup, restChannel, newTrailingEdgePlacement);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetCenterAlignDeadTime
 * Description   : Set Dead time value of waveforms in OPWMCB mode. New dead time should
 * be no greater than 0xFFFF (16 bits).
 * Implements    : EMIOS_DRV_PWM_SetCenterAlignDeadTime_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_SetCenterAlignDeadTime(uint8_t emiosGroup,
                                          uint8_t channel,
                                          uint32_t newDeadTime)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    DEV_ASSERT(((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restChannel) & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB);

     /* Validate opwmcb parameter */
    DEV_ASSERT(newDeadTime <= EMIOS_OPWMCB_MAX_CNT_VAL);

    EMIOS_SetUCRegB(emiosGroup, restChannel, newDeadTime);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetCenterAlignDeadTime
 * Description   : Get Dead time value of waveforms in OPWMCB mode.
 * Implements    : EMIOS_DRV_PWM_GetCenterAlignDeadTime_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetCenterAlignDeadTime(uint8_t emiosGroup,
                                              uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    DEV_ASSERT(((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restChannel) & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB);

    return EMIOS_GetUCRegB(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetCenterAlignIdealDutyCycle
 * Description   : Get Ideal Duty Cycle value of waveforms in OPWMCB mode. Ideal Duty cycle is time ON in a period,
 *                 this value may be difference with Ideal duty value configured previously.
 * Implements    : EMIOS_DRV_PWM_GetCenterAlignIdealDutyCycle_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetCenterAlignIdealDutyCycle(uint8_t emiosGroup,
                                                    uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    DEV_ASSERT(((uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restChannel) & EMIOS_FILTER_OPWMCB) == EMIOS_MASK_OPWMCB);
    uint32_t tmpIdealDutyCycle    = ((EMIOS_GetUCRegA(emiosGroup, restChannel) - 1UL) << 1UL);
    uint32_t tmpPeriodValue       = 0UL;
    uint32_t tmpGetIdealDutyCycle = 0UL;
    uint8_t  busSelect            = 0U;

    if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_A)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
    }
#if FEATURE_EMIOS_BUS_F_SELECT
    else if (EMIOS_GetUCRegCBsl(emiosGroup, restChannel) == (uint32_t)EMIOS_BUS_SEL_F)
    {
        busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
    }
#endif
    else
    {
        busSelect = (uint8_t)(channel & 0xF8U);
    }

    tmpPeriodValue = EMIOS_DRV_MC_GetCounterPeriod(emiosGroup, busSelect);

    if (EMIOS_GetUCRegCEdpol(emiosGroup, restChannel) == (uint32_t)EMIOS_POSITIVE_PULSE)
    {
        if (tmpIdealDutyCycle >= tmpPeriodValue)
        {
            tmpGetIdealDutyCycle = 0UL; /* 0% duty cycle */
        }
        else
        {
            /* 100% duty cycle when tmpIdealDutyCycle = 0 */
            tmpGetIdealDutyCycle = tmpPeriodValue - tmpIdealDutyCycle;
        }
    }
    else
    {
        if (tmpIdealDutyCycle >= tmpPeriodValue)
        {
            tmpGetIdealDutyCycle = tmpPeriodValue; /* 100% duty cycle */
        }
        else
        {
            tmpGetIdealDutyCycle = tmpIdealDutyCycle;
        }
    }

    return tmpGetIdealDutyCycle;
}

#if FEATURE_EMIOS_MODE_OPWMT_SUPPORT
/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_SetTriggerPlacement
 * Description   : Set Trigger placement value in OPWMT mode. New trigger placement
 * should be no larger than 0xFFFFFF(24 bits).
 * Implements    : EMIOS_DRV_PWM_SetTriggerPlacement_Activity
 *END**************************************************************************/
void EMIOS_DRV_PWM_SetTriggerPlacement(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t newTriggerPlacement)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    DEV_ASSERT(EMIOS_GetUCRegCMode(emiosGroup, restChannel) == (uint32_t)EMIOS_MODE_OPWMT);

    /* Validate opwmt parameter */
    DEV_ASSERT(newTriggerPlacement <= EMIOS_DATA_REG_MAX_VAL);
    EMIOS_SetUCRegALTA(emiosGroup, restChannel, newTriggerPlacement);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_PWM_GetTriggerPlacement
 * Description   : Get Trigger placement value in OPWMT mode
 * Implements    : EMIOS_DRV_PWM_GetTriggerPlacement_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_PWM_GetTriggerPlacement(uint8_t emiosGroup,
                                           uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif
    DEV_ASSERT(EMIOS_GetUCRegCMode(emiosGroup, restChannel) == (uint32_t)EMIOS_MODE_OPWMT);

    return EMIOS_GetUCRegALTA(emiosGroup, restChannel);
}
#endif /* FEATURE_EMIOS_MODE_OPWMT_SUPPORT */

/*******************************************************************************
* EOF
******************************************************************************/
