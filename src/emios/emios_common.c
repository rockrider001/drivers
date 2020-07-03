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
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter 'commonParam' could be declared as pointing to const.
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
 * Violates MISRA 2012 Advisory Rule 11.4,  conversion between a pointer and integer type.
 * This is required for working with the common initialize function in initialized counter bus function.
 * Counter bus can run with many differrent mode.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, conversion from pointer to void to pointer to other type
 * This is required for working with the common initialize function in initialized counter bus function.
 * Counter bus can run with many differrent mode.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from unsigned int to pointer.
 * This is required for working with the common initialize function in initialized counter bus function.
 * Counter bus can run with many differrent mode.
 *
 */

#include "emios_common.h"
#include "emios_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
eMIOS_Type* const eMIOS[EMIOS_NUMBER_GROUP_MAX] = eMIOS_BASE_PTRS;

/******************************************************************************
* API
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_InitGlobal
 * Description   : Setup basic parameters for a eMIOS group
 * Divide input system clock to module by clkDivVal. When put the eMIOS into low power mode.
 * The MDIS bit stops the clock of the block, except the access to registers EMIOSMCR,
 * EMIOSOUDIS and EMIOSUCDIS.
 * Select the clock divider value for the global prescaler in range (1-256). Divide input
 * system clock to module by clkDivVal.
 * Implements    : EMIOS_DRV_InitGlobal_Activity
 *END**************************************************************************/
void EMIOS_DRV_InitGlobal(uint8_t emiosGroup,
                          const emios_common_param_t *commonParam)
{
    DEV_ASSERT(commonParam != NULL);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT((commonParam->clkDivVal > 0U) && (commonParam->clkDivVal <= (EMIOS_GPRE_MAX_VAL + 1U)));

    eMIOS_MCR_SET_GPREN(emiosGroup, 0UL);                                            /* Disable global pre-scaler (reset default) */

    /* Check if the global prescaler is enabled */
    if (commonParam->enableGlobalPrescaler == true)
    {
        eMIOS_MCR_SET_GPRE(emiosGroup, commonParam->clkDivVal - 1UL);                    /* Divide 80 MHz clock to module by clkDivVal */
    }
    else
    {
        eMIOS_MCR_SET_GPRE(emiosGroup, 0UL);
    }

    eMIOS_MCR_SET_FRZ(emiosGroup, commonParam->allowDebugMode ? 1UL : 0UL);          /* Freeze channel registers in debug mode if channel FREN=0 */
    eMIOS_MCR_SET_MDIS(emiosGroup, commonParam->lowPowerMode ? 1UL : 0UL);           /* Enter low power mode or normal mode */
    eMIOS_MCR_SET_GPREN(emiosGroup, commonParam->enableGlobalPrescaler ? 1UL : 0UL); /* Setup enable global pre-scaler or not */
#if defined(FEATURE_EMIOS_STAC_CLIENT)
    eMIOS_MCR_SET_ETB(emiosGroup, commonParam->enableExternalTimeBase ? 1UL : 0UL);  /* Setup enable external timebase or not */
    eMIOS_MCR_SET_SRV(emiosGroup, commonParam->serverTimeSlot);                      /* Select the address of a specific STAC server to which the STAC is assigned */
#else
    eMIOS_MCR_SET_ETB(emiosGroup, 0UL);                                              /* Enabled external timebase, counter bus A assigned to Unified Channel */
#endif
    eMIOS_MCR_SET_GTBE(emiosGroup, commonParam->enableGlobalTimeBase ? 1UL : 0UL);   /* Setup enable global timebase or not */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EnableGlobalEmios
 * Description   : Enable Global Emios group
 * Implements    : EMIOS_DRV_EnableGlobalEmios_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EnableGlobalEmios(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    /* Enable global prescaler */
    eMIOS_MCR_SET_GPREN(emiosGroup, 1UL);
    /* Enable global timebase */
    eMIOS_MCR_SET_GTBE(emiosGroup, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_DisableGlobalEmios
 * Description   : Disable Global Emios group
 * Implements    : EMIOS_DRV_DisableGlobalEmios_Activity
 *END**************************************************************************/
 void EMIOS_DRV_DisableGlobalEmios(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    /* Disable global prescaler */
    eMIOS_MCR_SET_GPREN(emiosGroup, 0UL);
    /* Disable global timebase */
    eMIOS_MCR_SET_GTBE(emiosGroup, 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EnterLowPowerMode
 * Description   : Put the eMIOS into low power mode. The MDIS bit stops the clock of the block,
 * except the access to registers EMIOSMCR, EMIOSOUDIS and EMIOSUCDIS.
 * Implements    : EMIOS_DRV_EnterLowPowerMode_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EnterLowPowerMode(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    eMIOS_MCR_SET_MDIS(emiosGroup, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EscLowPowerMode
 * Description   : Escape the eMIOS low power mode, put into normal mode. Clock is running,
 * except the access to registers EMIOSMCR, EMIOSOUDIS and EMIOSUCDIS.
 * Implements    : EMIOS_DRV_EscLowPowerMode_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EscLowPowerMode(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    eMIOS_MCR_SET_MDIS(emiosGroup, 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsLowPowerMode
 * Description   : Get state of eMIOS is low power mode or not.
 * Implements    : EMIOS_DRV_IsLowPowerMode_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsLowPowerMode(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return (eMIOS_MCR_GET_MDIS(emiosGroup) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsAllowDebugMode
 * Description   : Get state of Allow Debug Mode
 * Implements    : EMIOS_DRV_IsAllowDebugMode_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsAllowDebugMode(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return (eMIOS_MCR_GET_FRZ(emiosGroup) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsGlobalPresEnabled
 * Description   : Get state of Global Prescaler enable or not.
 * Implements    : EMIOS_DRV_IsGlobalPresEnabled_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsGlobalPresEnabled(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return (eMIOS_MCR_GET_GPREN(emiosGroup) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsGlobalTimebaseEn
 * Description   : Get status of Global Time Base Enable bit(GTBE) in MCR register
 * Implements    : EMIOS_DRV_IsGlobalTimebaseEn_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsGlobalTimebaseEn(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return (eMIOS_MCR_GET_GTBE(emiosGroup) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ReadFlagState
 * Description   : Get status of UC Flag bit from GFLAG register
 * Implements    : EMIOS_DRV_ReadFlagState_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_ReadFlagState(uint8_t emiosGroup,
                              uint8_t channel)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return EMIOS_GetFLAGReg(emiosGroup, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsOutputUpdateDisabled
 * Description   : Read Output Update Disable state of the channel in OUDIS register
 * Implements    : EMIOS_DRV_IsOutputUpdateDisabled_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsOutputUpdateDisabled(uint8_t emiosGroup,
                                       uint8_t channel)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    return EMIOS_GetOudisReg(emiosGroup, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_DisableOutputUpdate
 * Description   : Set Output Update Disable for the channel in OUDIS register
 * Implements    : EMIOS_DRV_DisableOutputUpdate_Activity
 *END**************************************************************************/
 void EMIOS_DRV_DisableOutputUpdate(uint8_t emiosGroup,
                                    uint8_t channel)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(channel < EMIOS_NUMBER_CHANNEL_MAX);

    eMIOS[emiosGroup]->OUDIS = eMIOS[emiosGroup]->OUDIS | (1UL << channel); /* NOTE Big endien */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EnableAChOutputUpdate
 * Description   : Set Output Update Enable for the channel in OUDIS register
 * Implements    : EMIOS_DRV_EnableAChOutputUpdate_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EnableAChOutputUpdate(uint8_t emiosGroup,
                                      uint8_t channel)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(channel < EMIOS_NUMBER_CHANNEL_MAX);

    eMIOS[emiosGroup]->OUDIS =eMIOS[emiosGroup]->OUDIS & ((uint32_t)( 1UL << channel) ^ 0xFFFFFFFFUL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EnableAllChOutputUpdate
 * Description   : Set Output Update Enable for all channels in OUDIS register
 * Implements    : EMIOS_DRV_EnableAllChOutputUpdate_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EnableAllChOutputUpdate(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    EMIOS_SetOudisReg(emiosGroup, 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_InitGpioMode
 * Description   : In GPIO mode, all input capture and output compare functions of the Unified Channel
 * are disabled, and the internal counter is cleared and disabled. All control bits remain accessible.
 * When changing MODE[0:6], the application software must first enter GPIO mode to reset the Unified Channel's
 * internal functions properly. if the application software does not do this,it could lead to
 * invalid and unexpected output compare or input capture results or the FLAGs being set
 * incorrectly.
 * In case initialize for output:
 * when triggerMoe is EMIOS_TRIGGER_EDGE_ANY or EMIOS_TRIGGER_EDGE_FALLING, output pin will set to 0.
 * when triggerMoe is EMIOS_TRIGGER_EDGE_RISING, output pin will set to 1.
 * Implements    : EMIOS_DRV_InitGpioMode_Activity
 *END**************************************************************************/
 void EMIOS_DRV_InitGpioMode(uint8_t emiosGroup,
                             uint8_t channel,
                             const emios_gpio_mode_param_t *gpioParam)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
    DEV_ASSERT(gpioParam != NULL);
    DEV_ASSERT((gpioParam->mode == EMIOS_MODE_GPIO_INPUT) || (gpioParam->mode == EMIOS_MODE_GPIO_OUTPUT));
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    /* Configure registers */
    if (gpioParam->mode == EMIOS_MODE_GPIO_INPUT)
    {
        EMIOS_SetUCRegCEdsel(emiosGroup, restChannel, ((((uint8_t)(gpioParam->triggerMode) & 0x02U) == 0U) ? 0UL : 1UL));
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, ((((uint8_t)(gpioParam->triggerMode) & 0x01U) == 0U) ? 0UL : 1UL));
        EMIOS_SetUCRegCIf(emiosGroup, restChannel, (uint32_t)gpioParam->filterInput);
        EMIOS_SetUCRegCFck(emiosGroup, restChannel, gpioParam->filterEn ? 0UL : 1UL);
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)EMIOS_MODE_GPIO_INPUT);
    }
    else
    {
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, ((((uint8_t)(gpioParam->triggerMode) & 0x01U) == 0U) ? 0UL : 1UL));
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)EMIOS_MODE_GPIO_OUTPUT);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelEnterDebugMode
 * Description   : To set a channel enters freeze state, should be setting
 * EMIOS_DRV_AllowEnterDebugMode first.
 * Implements    : EMIOS_DRV_ChannelEnterDebugMode_Activity
 *END**************************************************************************/
 status_t EMIOS_DRV_ChannelEnterDebugMode(uint8_t emiosGroup,
                                          uint8_t channel)
{
    status_t ret = STATUS_ERROR;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    if (eMIOS_MCR_GET_FRZ(emiosGroup) == 1UL)
    {
        EMIOS_SetUCRegCFren(emiosGroup, restChannel, 1UL);

        ret = STATUS_SUCCESS;
    }
    else
    {
        ret = STATUS_EMIOS_ENABLE_GLOBAL_FRZ;
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelEscDebugMode
 * Description   : Release a channel from freeze state
 * Implements    : EMIOS_DRV_ChannelEscDebugMode_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelEscDebugMode(uint8_t emiosGroup,
                                    uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    EMIOS_SetUCRegCFren(emiosGroup, restChannel, 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelSetupOutputDisable
 * Description   : Setup Output Disable
 *                 eMIOS can disable it's output using it's disable inputs.
 *                 Disable inputs[3:0] are driven from the flag out bits[11:8].
 * Implements    : EMIOS_DRV_ChannelSetupOutputDisable_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelSetupOutputDisable(uint8_t emiosGroup,
                                          uint8_t channel,
                                          bool isOutputDisable,
                                          emios_output_dis_sel_t odissl)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    if (isOutputDisable == true)
    {
        EMIOS_SetUCRegCOdis(emiosGroup, restChannel, 1UL);
        EMIOS_SetUCRegCOdissl(emiosGroup, restChannel, (uint32_t)odissl);
    }
    else
    {
        EMIOS_SetUCRegCOdis(emiosGroup, restChannel, 0UL);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelEnableDMARequest
 * Description   : Enable DMA request
 * Implements    : EMIOS_DRV_ChannelEnableDMARequest_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelEnableDMARequest(uint8_t emiosGroup,
                                        uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    EMIOS_SetUCRegCFen(emiosGroup, restChannel, 1UL);
    EMIOS_SetUCRegCDma(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelEnableInterruptRequest
 * Description   : Disable DMA request (assigned to interrupt request)
 * Implements    : EMIOS_DRV_ChannelEnableInterruptRequest_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelEnableInterruptRequest(uint8_t emiosGroup,
                                              uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    EMIOS_SetUCRegCFen(emiosGroup, restChannel, 1UL);
    EMIOS_SetUCRegCDma(emiosGroup, restChannel, 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsDMAMode
 * Description   : Get state of DMA request enable bit. Get TRUE if eMIOS channel is DMA request mode.
 * Implements    : EMIOS_DRV_IsDMAMode_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsDMAMode(uint8_t emiosGroup,
                          uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    return (EMIOS_GetUCRegCDma(emiosGroup, restChannel) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
*
* Function Name : EMIOS_DRV_ChannelDisableIsrRequest
* Description   : Disallow the Unified Channel FLAG bit to generate an interrupt signal or
* a DMA request signal.
* Implements    : EMIOS_DRV_ChannelDisableIsrRequest_Activity
*END**************************************************************************/
 void EMIOS_DRV_ChannelDisableIsrRequest(uint8_t emiosGroup,
                                         uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    EMIOS_SetUCRegCFen(emiosGroup, restChannel, 0UL);
}

/*FUNCTION**********************************************************************
*
* Function Name : EMIOS_DRV_ChannelEnableIsrRequest
* Description   : Allow the Unified Channel FLAG bit to generate an interrupt signal or
* a DMA request signal.
* Implements    : EMIOS_DRV_ChannelEnableIsrRequest_Activity
*END**************************************************************************/
 void EMIOS_DRV_ChannelEnableIsrRequest(uint8_t emiosGroup,
                                        uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    EMIOS_SetUCRegCFen(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
*
* Function Name : EMIOS_DRV_ReadInputPinState
* Description   : Reflects the input pin state after being filtered and synchronized.
* Implements    : EMIOS_DRV_ReadInputPinState_Activity
*END**************************************************************************/
 bool EMIOS_DRV_ReadInputPinState(uint8_t emiosGroup,
                                  uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    return (EMIOS_GetUCRegSUcin(emiosGroup, restChannel) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
*
* Function Name : EMIOS_DRV_ReadOutputPinState
* Description   : Reflects the output pin state.
* Implements    : EMIOS_DRV_ReadOutputPinState_Activity
*END**************************************************************************/
 bool EMIOS_DRV_ReadOutputPinState(uint8_t emiosGroup,
                                   uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    return (EMIOS_GetUCRegSUcout(emiosGroup, restChannel) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_DeInitChannel
 * Description   : Reset eMIOS channel to GPIO mode (reset default)
 * Implements    : EMIOS_DRV_DeInitChannel_Activity
 *END**************************************************************************/
 void EMIOS_DRV_DeInitChannel(uint8_t emiosGroup,
                              uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    /* Enable channel if it is disabled by disable clock */
    EMIOS_DRV_ChannelEnableClk(emiosGroup, channel);
    /* Disable channel pre-scaler (reset default) */
    eMIOS[emiosGroup]->UC[restChannel].C = 0UL;
    /* Reset An and Bn registers */
    eMIOS[emiosGroup]->UC[restChannel].A = 0UL;
    eMIOS[emiosGroup]->UC[restChannel].B = 0UL;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ClrFlagState
 * Description   : Clear FLAG bit. FLAG bit is set when an input capture or a match event in the
 * comparators occurred.
 * Implements    : EMIOS_DRV_ClrFlagState_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ClrFlagState(uint8_t emiosGroup,
                             uint8_t channel)
 {
     uint8_t restChannel = 0U;
     bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
     DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
     DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

     EMIOS_SetUCRegSFlag(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_SetOutputLevel
 * Description   : This function is used to sets the action executed on a compare
 * match value to set output pin, clear output pin, toggle output pin.
 *
 * Implements    : EMIOS_DRV_SetOutputLevel_Activity
 *END**************************************************************************/
void EMIOS_DRV_SetOutputLevel(uint8_t emiosGroup,
                              uint8_t channel,
                              bool edgeSel,
                              bool edgePol)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    /* Set the channel output mode */
    EMIOS_SetUCRegCEdsel((uint8_t)emiosGroup, restChannel, edgeSel);
    EMIOS_SetUCRegCEdpol((uint8_t)emiosGroup, restChannel, edgePol);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_IsChannelClkDisabled
 * Description   : Read state of Disable Channel register bit
 * Implements    : EMIOS_DRV_IsChannelClkDisabled_Activity
 *END**************************************************************************/
 bool EMIOS_DRV_IsChannelClkDisabled(uint8_t emiosGroup,
                                     uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    bool tmp = false;

#if defined(FEATURE_EMIOS_UC_DISABLE)
    tmp = (((eMIOS_MCR_GET_UCDIS(emiosGroup) >> channel) & 0x01UL) == 0UL) ? false : true;
#else
    tmp = (((eMIOS[emiosGroup]->UC[restChannel].C & eMIOS_C_UCPREN_MASK) >> eMIOS_C_UCPREN_SHIFT) == 0UL) ? true : false;
#endif

    return tmp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelDisableClk
 * Description   : Disable indivisual channel by stopping its respective clock.
 *                 Note that Control register will be locked, it's mean user
 *                 can not update any parameters in this register.
 * Implements    : EMIOS_DRV_ChannelDisableClk_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelDisableClk(uint8_t emiosGroup,
                                  uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#if defined(FEATURE_EMIOS_UC_DISABLE)
    eMIOS[emiosGroup]->UCDIS = eMIOS[emiosGroup]->UCDIS | ( 1UL << channel);
#else
    EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, 0UL);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_ChannelEnableClk
 * Description   : Enable the channel clock
 * Implements    : EMIOS_DRV_ChannelEnableClk_Activity
 *END**************************************************************************/
 void EMIOS_DRV_ChannelEnableClk(uint8_t emiosGroup,
                                 uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#if defined(FEATURE_EMIOS_UC_DISABLE)
    eMIOS[emiosGroup]->UCDIS = eMIOS[emiosGroup]->UCDIS & ((uint32_t)( 1UL << channel) ^ 0xFFFFFFFFUL);
#else
    EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, 1UL);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_EnableAllChannelClk
 * Description   : Enable all of the channels clock
 * Implements    : EMIOS_DRV_EnableAllChannelClk_Activity
 *END**************************************************************************/
 void EMIOS_DRV_EnableAllChannelClk(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

#if defined(FEATURE_EMIOS_UC_DISABLE)
    eMIOS_MCR_SET_UCDIS(emiosGroup, 0UL);
#else
    uint8_t channel = 0U;
    uint8_t channelNumbers[FEATURE_EMIOS_CH_COUNT] = PWMPAL_INDEX_2_HW_CHANNELS;

    for (channel = 0U; channel < FEATURE_EMIOS_CH_COUNT; channel++)
    {
        EMIOS_DRV_ChannelEnableClk(emiosGroup, channelNumbers[channel]);
    }
    /* Enabled global clock */
    eMIOS_MCR_SET_GPREN(emiosGroup, 1UL);
#endif
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OutputDisable
 * Description   : This function is used to disable each channel output by its clock.
 * The output after call this function is depended on its last logic level.
 * It is a private function and is called in PAL layer.
 * Implements    : EMIOS_DRV_OutputDisable_Activity
 *END**************************************************************************/
void EMIOS_DRV_OutputDisable(uint8_t emiosGroup,
                             uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#if defined(FEATURE_EMIOS_UC_DISABLE)
    uint32_t val = eMIOS_MCR_GET_UCDIS(emiosGroup);

    val |= (uint32_t)(1UL << channel);
    /* Disable pin output */
    eMIOS_MCR_SET_UCDIS(emiosGroup, val);
#else
    EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, 0UL);
#endif
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OutputEnable
 * Description   : This function is used to enable the channel output by its
 * respective clock.
 * It is a private function and is called in PAL layer.
 * Implements    : EMIOS_DRV_OutputEnable_Activity
 *END**************************************************************************/
void EMIOS_DRV_OutputEnable(uint8_t emiosGroup,
                            uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

#if defined(FEATURE_EMIOS_UC_DISABLE)
    uint32_t val = 0U;
    val = eMIOS_MCR_GET_UCDIS(emiosGroup);
    val &= ~((uint32_t)(1UL << channel));
    /* Enable pin output */
    eMIOS_MCR_SET_UCDIS(emiosGroup, val);
#else
    EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, 1UL);
#endif
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_GetCurrentOutputPin
 * Description   : This function gets current status of each output pin
 * in output mode.
 * It is a private function and is called in PAL layer.
 * Implements    : EMIOS_DRV_GetCurrentOutputPin_Activity
 *END**************************************************************************/
bool EMIOS_DRV_GetCurrentOutputPin(uint8_t emiosGroup,
                                   uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    return (EMIOS_GET_REG(eMIOS[emiosGroup]->UC[restChannel].S, eMIOS_S_UCOUT_MASK, eMIOS_S_UCOUT_SHIFT));
}
/*******************************************************************************
* EOF
********************************************************************************/

