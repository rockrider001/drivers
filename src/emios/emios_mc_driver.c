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
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter 'mcbParam' could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 */

#include "emios_mc_driver.h"
#include "emios_oc_driver.h"
#include "emios_hw_access.h"

/*******************************************************************************
 * Private API declaration
 ******************************************************************************/
/*!
 * brief Initialize eMIOS Modulus Counter (MC) mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] mcParam A pointer to the MC configuration structure
 * return operation status
 *        - STATUS_SUCCESS   :  Operation was successful.
 *        - STATUS_ERROR     :  Operation failed, invalid input value. This status only has for MPC5748G and MPC5746C.
 */
static status_t EMIOS_DRV_MC_InitMcMode(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_mc_mode_param_t *mcParam);

/*!
 * brief Update new period value for MC mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] newPeriod The new Period value
 */
static void EMIOS_DRV_MC_SetMcPeriodValue(uint8_t emiosGroup,
                                              uint8_t channel,
                                              uint32_t newPeriod);

/*!
 * brief Initialize Modulus Counter Buffered (MCB) Mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] mcbParam A pointer to the MCB configuration structure
 * return operation status
 *        - STATUS_SUCCESS    :  Operation was successful.
 *        - STATUS_ERROR      :  Operation failed, invalid input value. This status only has for MPC5748G and MPC5746C.
 */
static status_t EMIOS_DRV_MC_InitMcbMode(uint8_t emiosGroup,
                                         uint8_t channel,
                                         const emios_mc_mode_param_t *mcbParam);

/*!
 * brief Update new period value for MCB mode
 *
 * param[in] emiosGroup The eMIOS group id
 * param[in] channel The channel in this eMIOS group
 * param[in] newPeriod The new Period value
 */
static void EMIOS_DRV_MC_SetMcbPeriodValue(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint32_t newPeriod);

/******************************************************************************
* Public API
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_InitCounterMode
 * Description   : Initialize Modulus Counter (MC) Mode.
 * The MC mode can be used to provide a time base for a counter bus or as a general purpose timer.
 * Implements    : EMIOS_DRV_MC_InitCounterMode_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_MC_InitCounterMode(uint8_t emiosGroup,
                                      uint8_t channel,
                                      const emios_mc_mode_param_t *mcParam)
{
    status_t ret        = STATUS_SUCCESS;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(mcParam != NULL);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    if ((((uint8_t)mcParam->mode & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP) ||
        (((uint8_t)mcParam->mode & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN))
    {
        /* Init mc mode */
        ret = EMIOS_DRV_MC_InitMcMode(emiosGroup, restChannel, mcParam);
    }
    else if ((((uint8_t)mcParam->mode & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
             (((uint8_t)mcParam->mode & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN))
    {
        /* Init mcb mode */
        ret = EMIOS_DRV_MC_InitMcbMode(emiosGroup, restChannel, mcParam);
    }
    else
    {
        ret = STATUS_ERROR;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_SetCounterPeriod
 * Description   : Set Period for Modulus Counter (MC) mode.
 * Updates Period in MC mode may cause loss of match in the current cycle if the transfer.
 * occurs near the match. In this case, the counter may rollover and resume operation in the next cycle.
 * Implements    : EMIOS_DRV_MC_SetCounterPeriod_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_MC_SetCounterPeriod(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t newPeriod)
{
    status_t ret        = STATUS_SUCCESS;
    uint8_t tempMode    = 0U;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    tempMode = (uint8_t)EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    if (((tempMode & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP) ||
        ((tempMode & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN)) /* MC mode*/
    {
        EMIOS_DRV_MC_SetMcPeriodValue(emiosGroup, restChannel, newPeriod);
    }
    else if (((tempMode & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
             ((tempMode & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN)) /* MCB mode*/
    {
        EMIOS_DRV_MC_SetMcbPeriodValue(emiosGroup, restChannel, newPeriod);
    }
    else
    {
        ret = STATUS_EMIOS_WRONG_MODE;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_GetCounterPeriod
 * Description   : Get Period of Modulus Counter (MC) mode.
 * Implements    : EMIOS_DRV_MC_GetCounterPeriod_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_MC_GetCounterPeriod(uint8_t emiosGroup,
                                       uint8_t channel)
{
    uint32_t ret        = 0UL;
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
    if ((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP)  /* Counter up mode */
    {
        /* Return period value */
        ret = EMIOS_GetUCRegA(emiosGroup, restChannel);                               /* MC mode: A register value will be "period" */
    }
    else if ((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN)                    /* Counter up/down mode */
    {
        /* Return period value */
        ret = (EMIOS_GetUCRegA(emiosGroup, restChannel) << 1UL);                      /* MC mode: A register value will be 2*"period" */
    }
    else if (((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
             (temp == (uint32_t)EMIOS_MODE_SAOC))                                 /* Modulus counter buffer up mode */
    {
        /* Return period value */
        ret = EMIOS_GetUCRegA(emiosGroup, restChannel);                               /* MCB mode: A register value will be "period" */
    }
    else if ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN)                  /* Modulus counter buffer up/down mode */
    {
        /* Return period value */
        ret = ((EMIOS_GetUCRegA(emiosGroup, restChannel) << 1UL)- 2UL);               /* MCB mode: A register value will be "period"/2 +1 */
    }
    else
    {
        /* Wrong mode */
        DEV_ASSERT(false);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_CounterRead
 * Description   : Get current counter value.
 * Note that in modulus counter up/down mode, counter value moves up then comes down.
 * In the counter up mode, counter value moves up only.
 * Implements    : EMIOS_DRV_MC_CounterRead_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_MC_CounterRead(uint8_t emiosGroup,
                                  uint8_t channel)
{
    uint32_t temp       = 0UL;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT(((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP) ||
               ((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN) ||
               (temp == (uint32_t)EMIOS_MODE_SAOC));
    (void) temp;

    return EMIOS_GetUCRegCNT(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_CounterStart
 * Description   : Start counter by enable channel clock. Required somethings prior to counter start
 * - EMIOS_DRV_InitGlobal
 * - EMIOS_DRV_EnableGlobalEmios
 * Implements    : EMIOS_DRV_MC_CounterStart_Activity
 *END**************************************************************************/
void EMIOS_DRV_MC_CounterStart(uint8_t emiosGroup,
                               uint8_t channel)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT(((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP) ||
               ((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN));
    (void) temp;
    EMIOS_DRV_ChannelEnableClk(emiosGroup, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_CounterStop
 * Description   : Stop counter by disable channel clock.
 * Implements    : EMIOS_DRV_MC_CounterStop_Activity
 *END**************************************************************************/
void EMIOS_DRV_MC_CounterStop(uint8_t emiosGroup,
                              uint8_t channel)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(restValidate == true);
#else
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT(((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP) ||
               ((temp & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UPDOWN) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP) ||
               ((temp & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UPDOWN));
    (void) temp;
    EMIOS_DRV_ChannelDisableClk(emiosGroup, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_InitMcMode
 * Description   : Initialize Modulus Counter (MC) Mode.
 * The MC mode can be used to provide a time base for a counter bus or as a general purpose timer.
 *
 *END**************************************************************************/
static status_t EMIOS_DRV_MC_InitMcMode(uint8_t emiosGroup,
                                        uint8_t channel,
                                        const emios_mc_mode_param_t *mcParam)
{
    status_t status = STATUS_SUCCESS;
#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid MC with channel supported */
    if (EMIOS_ValidateMode(emiosGroup, channel, (uint8_t)EMIOS_GMODE_MC) == false)
    {
         status = STATUS_ERROR;
    }
    else
#endif /* FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Cleared UC configure registers */
        eMIOS[emiosGroup]->UC[channel].C = 0UL;                                         /* Disable channel pre-scaler (reset default) */

        if (((uint8_t)mcParam->mode & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP)             /* Counter up mode */
        {
            /* Validate period value */
            DEV_ASSERT((mcParam->period > EMIOS_MC_MIN_CNT_VAL) && \
                       (mcParam->period <= EMIOS_MC_MAX_CNT_VAL));

            /* Calculate period */
            EMIOS_SetUCRegA(emiosGroup, channel, mcParam->period);                      /* MC mode: A register value will be "period" */
        }
        else /* Counter up/down mode */
        {
            /* Validate period value */
            DEV_ASSERT((mcParam->period > EMIOS_MC_MIN_CNT_VAL) && \
                       ((mcParam->period <= (EMIOS_MC_MAX_CNT_VAL*2UL)) && \
                       ((mcParam->period & 0x01UL) == 0UL)));                           /* period must be even number */

            /* Calculate period */
            EMIOS_SetUCRegA(emiosGroup, channel, (uint32_t)(mcParam->period >> 1UL));   /* MC mode: A register value will be "period"/2 */
        }
        /* Setup parammeter for external clock source */
        if (((uint8_t)mcParam->mode & 0x01U) == 1U)
        {
            EMIOS_SetUCRegCEdsel(emiosGroup, channel, ((((uint8_t)mcParam->triggerMode & 0x02U) == 0U) ? 0UL : 1UL));
            EMIOS_SetUCRegCEdpol(emiosGroup, channel, ((((uint8_t)mcParam->triggerMode & 0x01U) == 0U) ? 0UL : 1UL));
            EMIOS_SetUCRegCIf(emiosGroup, channel, (uint32_t)mcParam->filterInput);
            EMIOS_SetUCRegCFck(emiosGroup, channel, mcParam->filterEn ? 0UL : 1UL);
        }
        EMIOS_SetUCRegCUcpren(emiosGroup, channel, mcParam->internalPrescalerEn ? 1UL : 0UL);     /* Enable pre-scaler */
        EMIOS_SetUCRegCMode(emiosGroup, channel, (uint32_t)mcParam->mode);                        /* Modulus Counter (Up ctr) */
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, channel, (uint32_t)mcParam->internalPrescaler);      /* Pre-scale channel clock by internalPrescaler +1 */
        EMIOS_SetUCRegC2UCPRECLK(emiosGroup, channel, 0UL);                                       /* Prescaler clock selected */
#else
        EMIOS_SetUCRegCUcpre(emiosGroup, channel, (uint32_t)mcParam->internalPrescaler);          /* Pre-scale channel clock by internalPrescaler +1 */
#endif
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_SetMcPeriodValue
 * Description   : Set Period for Modulus Counter (MC) mode.
 * Updates Period in MC mode may cause loss of match in the current cycle if the transfer
 * occurs near the match. In this case, the counter may rollover and resume operation in the next cycle.
 *
 *END**************************************************************************/
static void EMIOS_DRV_MC_SetMcPeriodValue(uint8_t emiosGroup,
                                          uint8_t channel,
                                          uint32_t newPeriod)
{
    if ((EMIOS_GetUCRegCMode(emiosGroup, channel) & EMIOS_FILTER_MC) == EMIOS_MASK_MC_UP)  /* Counter up mode */
    {
        /* Validate period value */
        DEV_ASSERT((newPeriod > EMIOS_MC_MIN_CNT_VAL) && \
                   (newPeriod <= EMIOS_MC_MAX_CNT_VAL));

        /* Calculate period */
        EMIOS_SetUCRegA(emiosGroup, channel, newPeriod);                                   /* MC mode: A register value will be "period" clocks */
    }
    else /* Counter up/down mode */
    {
        /* Validate period value */
        DEV_ASSERT((newPeriod > EMIOS_MC_MIN_CNT_VAL) && \
                   ((newPeriod <= (EMIOS_MC_MAX_CNT_VAL*2UL)) && \
                   ((newPeriod & 0x01UL) == 0UL)));                                        /* period must be even number */

        /* Calculate period */
        EMIOS_SetUCRegA(emiosGroup, channel, (uint32_t)(newPeriod >> 1UL));                /* MC mode: A register value will be 2*"period" clocks */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_InitMcbMode
 * Description   : Initialize Modulus Counter Buffered (MCB) Mode.
 * The MCB mode provides a time base which can be shared with other channels through the
 * internal counter buses. This mode allowing smooth transitions between cycles when changing
 * period value.
 *END**************************************************************************/
static status_t EMIOS_DRV_MC_InitMcbMode(uint8_t emiosGroup,
                                         uint8_t channel,
                                         const emios_mc_mode_param_t *mcbParam)
{
    status_t status = STATUS_SUCCESS;
#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid MCB with channel supported */
    if (EMIOS_ValidateMode(emiosGroup, channel, (uint8_t)EMIOS_GMODE_MCB) == false)
    {
         status = STATUS_ERROR;
    }
    else
#endif /* FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
    {
        /* Cleared UC configure registers */
        eMIOS[emiosGroup]->UC[channel].C = 0UL;                                         /* Disable channel pre-scaler (reset default) */

        if (((uint8_t)mcbParam->mode & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP)          /* Counter up mode */
        {
            /* Validate period value */
            DEV_ASSERT((mcbParam->period > EMIOS_MCB_MIN_CNT_VAL) && \
                       (mcbParam->period <= EMIOS_MCB_MAX_CNT_VAL));

            /* Calculate period */
            EMIOS_SetUCRegA(emiosGroup, channel, mcbParam->period);                     /* MCB mode: A register value will be "period" clocks */
        }
        else /* Counter up/down mode */
        {
            /* Validate period value */
            DEV_ASSERT((mcbParam->period > EMIOS_MCB_MIN_CNT_VAL) && \
                       ((mcbParam->period <= ((EMIOS_MCB_MAX_CNT_VAL*2UL) - 2UL)) && \
                       ((mcbParam->period & 0x01UL) == 0UL)));                             /* period must be even number */

            /* Calculate period */
            /* MCB mode: A register value will be "period"/2 +1 */
            EMIOS_SetUCRegA(emiosGroup, channel, (uint32_t)((mcbParam->period >> 1UL) + 1UL));
        }
        /* Setup parameter for external clock source */
        if (((uint8_t)mcbParam->mode & 0x01U) == 1U)
        {
            EMIOS_SetUCRegCEdsel(emiosGroup, channel, ((((uint8_t)mcbParam->triggerMode & 0x02U) == 0U) ? 0UL : 1UL));
            EMIOS_SetUCRegCEdpol(emiosGroup, channel, ((((uint8_t)mcbParam->triggerMode & 0x01U) == 0U) ? 0UL : 1UL));
            EMIOS_SetUCRegCIf(emiosGroup, channel, (uint32_t)mcbParam->filterInput);
            EMIOS_SetUCRegCFck(emiosGroup, channel, mcbParam->filterEn ? 0UL : 1UL);
        }
        EMIOS_SetUCRegCUcpren(emiosGroup, channel, mcbParam->internalPrescalerEn ? 1UL : 0UL);   /* Enable pre-scaler */
        EMIOS_SetUCRegCMode(emiosGroup, channel, (uint32_t)mcbParam->mode);                      /* Modulus Counter Buffered (Up ctr) */
#if defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegC2UCEXTPRE(emiosGroup, channel, (uint32_t)mcbParam->internalPrescaler);    /* Pre-scaler channel clock by internalPrescaler +1 */
        EMIOS_SetUCRegC2UCPRECLK(emiosGroup, channel, 0UL);                                      /* Pre-scaler clock selected */
#else
        EMIOS_SetUCRegCUcpre(emiosGroup, channel, (uint32_t)mcbParam->internalPrescaler);         /* Pre-scaler channel clock by internalPrescaler +1 */
#endif
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_MC_SetMcbPeriodValue
 * Description   : Update Period for MCB mode. Period can be written at any time
 * within cycle n in order to be used in cycle n+1.
 *
 *END**************************************************************************/
static void EMIOS_DRV_MC_SetMcbPeriodValue(uint8_t emiosGroup,
                                           uint8_t channel,
                                           uint32_t newPeriod)
{
    if ((EMIOS_GetUCRegCMode(emiosGroup, channel) & EMIOS_FILTER_MCB) == EMIOS_MASK_MCB_UP)
    {
         /* Validate period value */
        DEV_ASSERT((newPeriod > EMIOS_MCB_MIN_CNT_VAL) && (newPeriod <= EMIOS_MCB_MAX_CNT_VAL));

        /* Calculate period */
        /* MCB mode: A register value will be "period" clocks */
        EMIOS_SetUCRegA(emiosGroup, channel, newPeriod);
    }
    else /* MCB_UPDOWN mode */
    {
        /* Validate period value */
        /* period must be even number */
        DEV_ASSERT((newPeriod > EMIOS_MCB_MIN_CNT_VAL) && \
                   ((newPeriod <= ((EMIOS_MCB_MAX_CNT_VAL*2UL) - 2UL)) && \
                   ((newPeriod & 0x01UL) == 0UL)));

        /* Calculate period */
        /* MCB mode: A register value will be "period" clocks */
        EMIOS_SetUCRegA(emiosGroup, channel, (uint32_t)((newPeriod >> 1UL) + 1UL));
    }
}

/*******************************************************************************
* EOF
******************************************************************************/
