/*
 * Copyright 2017 - 2019 NXP
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
 * Violates MISRA 2012 Advisory Rule 8.13,  Pointer parameter 'icParam' could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include "emios_ic_driver.h"
#include "emios_mc_driver.h"
#include "emios_hw_access.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IC_InitModeWidth
 * Description   : initializer, Input mode : Width
 *END**************************************************************************/
static status_t EMIOS_DRV_IC_InitModeWidth(uint8_t emiosGroup,
                                           uint8_t channel,
                                           const emios_input_capture_param_t *icParam)
{
    status_t ret;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid Input Pulse Width Measurement with channels supported */
    if (EMIOS_ValidateMode(emiosGroup, channel, (uint8_t)EMIOS_GMODE_IPWM) == false)
    {
        ret = STATUS_ERROR;
    }
    else
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Configure registers */
        if (icParam->inputCaptureMode == EMIOS_PERIOD_ON_MEASUREMENT)
        {
            EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)EMIOS_POSITIVE_PULSE);
        }
        else
        {
            EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)EMIOS_NEGATIVE_PULSE);
        }

    #ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
        if ((EMIOS_ValidateInternalCnt(emiosGroup, channel) == false)
        && (icParam->timebase == EMIOS_BUS_SEL_INTERNAL))
        {
            /* Choose internal counter, this channel mode should be Type X or G */
            ret = STATUS_ERROR;
        }
        else
    #endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
        {
            /* Choose external counter */
            ret = STATUS_SUCCESS;
            EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)EMIOS_MODE_PULSE_WIDTH);
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IC_InitModePeriod
 * Description   : initializer, Input mode : Period
 *END**************************************************************************/
static status_t EMIOS_DRV_IC_InitModePeriod(uint8_t emiosGroup,
                                            uint8_t channel,
                                            const emios_input_capture_param_t *icParam)
{
    status_t ret;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid Input Period Measurement with channels supported */
    if (EMIOS_ValidateMode(emiosGroup, channel, (uint8_t)EMIOS_GMODE_IPM) == false)
    {
        ret = STATUS_ERROR;
    }
    else
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Validate 4 system clock cycles period at least in non by-pass mode */
        /* Configure registers */
        if (icParam->inputCaptureMode == EMIOS_RISING_EDGE_PERIOD_MEASUREMENT)
        {
            EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, EMIOS_RISING_EDGE);
        }
        else
        {
            EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, EMIOS_FALLING_EDGE);
        }

    #ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
        if ((EMIOS_ValidateInternalCnt(emiosGroup, channel) == false)
        && (icParam->timebase == EMIOS_BUS_SEL_INTERNAL))
        {
            /* Choose internal counter, this channel mode should be Type X or G */
            ret = STATUS_ERROR;
        }
        else
    #endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
        {
            /* Choose external counter */
            ret = STATUS_SUCCESS;
            EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)EMIOS_MODE_PERIOD);
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IC_InitModeSaic
 * Description   : initializer, Input mode : Saic
 *END**************************************************************************/
static status_t EMIOS_DRV_IC_InitModeSaic(uint8_t emiosGroup,
                                          uint8_t channel,
                                          const emios_input_capture_param_t *icParam)
{
    status_t ret;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    /* Configure registers */
    eMIOS[emiosGroup]->UC[restChannel].C = 0UL; /* Disable channel pre-scaler (reset default) */
    EMIOS_SetUCRegCEdsel(emiosGroup, restChannel, ((((uint8_t)(icParam->inputCaptureMode) & 0x02U) == 0U) ? 0UL : 1UL));
    EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, ((((uint8_t)(icParam->inputCaptureMode) & 0x01U) == 0U) ? 0UL : 1UL));

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Validate internal counter with selected channel */
    if ((EMIOS_ValidateInternalCnt(emiosGroup, channel) == false)
    && (icParam->timebase == EMIOS_BUS_SEL_INTERNAL))
    {
        /* Choose internal counter, this channel mode should be Type X or G */
        ret = STATUS_ERROR;
    }
    else
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Choose external counter */
        ret = STATUS_SUCCESS;
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)EMIOS_MODE_SAIC);
    }

    return ret;
}

/******************************************************************************
* API
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IC_InitInputCaptureMode
 * Description   : Initialize Input Measurement Mode or Single Action Input Capture mode
 * - Input Measurement Mode
 * This mode allows the measurement of the width of a positive or negative pulse or period of an input signal by
 * capturing two consecutive rising edges or two consecutive falling edges.
 * In period measurement mode: Successive input captures are done on consecutive edges of the same polarity.
 * The input signal must have at least four system clock cycles period in order to be properly
 * captured by the synchronization logic at the channel input even if the input filter is in by-pass mode.
 * - Single Action Input Capture mode
 * In SAIC mode, when a triggering event occurs on the input pin, the value on the selected time base is captured.
 * Implements    : EMIOS_DRV_IC_InitInputCaptureMode_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_IC_InitInputCaptureMode(uint8_t emiosGroup,
                                           uint8_t channel,
                                           const emios_input_capture_param_t *icParam)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(icParam != NULL);
    DEV_ASSERT(icParam->mode == EMIOS_MODE_IC);
    status_t ret;

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    uint32_t temp;

    /* Get mode of operation of the Unified Channel */
    temp = EMIOS_GetUCRegCMode( emiosGroup, restChannel);

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    bool validMCMode = false;
    bool validMCBMode = false;

    /* Valid MC or MCB with channel supported */
    validMCMode  = EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_MC);
    validMCBMode = EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_MCB);

    DEV_ASSERT((eMIOS[emiosGroup]->UC[restChannel].C == 0UL) ||
               (((validMCMode == true) || (validMCBMode == true)) && (temp == (uint32_t)EMIOS_MODE_MCB_UP_COUNTER_INT_CLK)) ||
               (temp == (uint32_t)EMIOS_MODE_PULSE_WIDTH) ||
               (temp == (uint32_t)EMIOS_MODE_PERIOD) ||
               (temp == (uint32_t)EMIOS_MODE_SAIC));
#else
    /* Check that device was initialized */
    DEV_ASSERT((eMIOS[emiosGroup]->UC[restChannel].C == 0UL) ||
               (temp == (uint32_t)EMIOS_MODE_MCB_UP_COUNTER_INT_CLK) ||
               (temp == (uint32_t)EMIOS_MODE_PULSE_WIDTH) ||
               (temp == (uint32_t)EMIOS_MODE_PERIOD) ||
               (temp == (uint32_t)EMIOS_MODE_SAIC));
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
#endif /* if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    if ((icParam->inputCaptureMode == EMIOS_PERIOD_ON_MEASUREMENT) ||
        (icParam->inputCaptureMode == EMIOS_PERIOD_OFF_MEASUREMENT))
    {
        ret = EMIOS_DRV_IC_InitModeWidth(emiosGroup, channel, icParam);
    }
    else if ((icParam->inputCaptureMode == EMIOS_RISING_EDGE_PERIOD_MEASUREMENT) ||
             (icParam->inputCaptureMode == EMIOS_FALLING_EDGE_PERIOD_MEASUREMENT))
    {
        ret = EMIOS_DRV_IC_InitModePeriod(emiosGroup, channel, icParam);
    }
    else
    {
        ret = EMIOS_DRV_IC_InitModeSaic(emiosGroup, channel, icParam);
    }

    if (ret == STATUS_SUCCESS)
    {
        EMIOS_SetUCRegCBsl(emiosGroup, restChannel, (uint32_t)icParam->timebase);
        EMIOS_SetUCRegCIf(emiosGroup, restChannel, (uint32_t)icParam->filterInput);
        EMIOS_SetUCRegCFck(emiosGroup, restChannel, icParam->filterEn ? 0UL : 1UL);
        EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, 1U);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IC_GetLastMeasurement
 * Description   : Get last measurement value in input capture measurement mode or
 * get Input capture value in SAIC mode.
 * Implements    : EMIOS_DRV_IC_GetLastMeasurement_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_IC_GetLastMeasurement(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t * const retValue)
{
    uint8_t counterBus;
    uint32_t temp, busSelect;
    status_t retStatus = STATUS_SUCCESS;
    uint8_t restChannel = 0U;
    uint8_t restBusSelect = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(retValue != NULL);

    /* Get mode of operation of the Unified Channel */
    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    /* Checking mode was input capture mode */
    if((temp != (uint32_t)EMIOS_MODE_PERIOD) && (temp != (uint32_t)EMIOS_MODE_PULSE_WIDTH) && (temp != (uint32_t)EMIOS_MODE_SAIC))
    {
        retStatus = STATUS_ERROR;
    }
    else
    {
        if (temp == EMIOS_MODE_SAIC)
        {
            *retValue = EMIOS_GetUCRegA(emiosGroup, restChannel);
        }
        else
        {
            uint32_t secondCnt = EMIOS_GetUCRegA(emiosGroup, restChannel);
            uint32_t firstCnt = EMIOS_GetUCRegB(emiosGroup, restChannel);

            /* Normal Case */
            if (firstCnt <= secondCnt)
            {
                *retValue = secondCnt - firstCnt;
            }
            else
            {
                /* Get bus select */
                busSelect = EMIOS_GetUCRegCBsl(emiosGroup, restChannel);

                /* Get counter bus driven */
                counterBus = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;

                switch (busSelect)
                {
                    case (uint32_t)EMIOS_BUS_SEL_A:
                        counterBus = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
                        break;
                #if FEATURE_EMIOS_BUS_F_SELECT
                    case (uint32_t)EMIOS_BUS_SEL_F:
                        counterBus = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
                        break;
                #endif
                    case (uint32_t)EMIOS_BUS_SEL_BCDE:
                        counterBus = (uint8_t)(channel & 0xF8U);
                        break;
                    default:
                        counterBus = channel;
                        break;
                }

                /* Check channel */
                (void)EMIOS_ValidateChannel(counterBus, &restBusSelect);

                /* if counter bus selected is Up mode */
                if (counterBus != channel)
                {
                    *retValue = (EMIOS_GetUCRegA(emiosGroup, restBusSelect) - firstCnt) + secondCnt;
                }
                else
                {
                    *retValue = (EMIOS_MC_MAX_CNT_VAL - firstCnt) + secondCnt;
                }
            }
        }
    }

    return retStatus;
}

/*******************************************************************************
* EOF
******************************************************************************/
