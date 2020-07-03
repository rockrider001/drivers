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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 18.1, Index value of IGF->CHb array never touches
 * the beyond end of data because it was checked with size of the array.
 * So, elements of the array are always available.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */
#include <stddef.h>
#include "igf_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Define state structures for IGF */
static IGF_Type * const g_igfStatePtr[IGF_INSTANCE_COUNT] = IGF_BASE_PTRS;
/* Max ID channel of each instance */
static const uint8_t MAX_CHANNEL_INSTANCE[IGF_INSTANCE_COUNT] = FEATURE_IGF_MAX_CHANNEL_INSTANCE_ARRAY_DEFINE;
/*******************************************************************************
 * INTERNAL FUNCTIONS
 ******************************************************************************/
static status_t IGF_DRV_InitChannel(uint32_t instance, const igf_ch_param_t * configChannelPtr);
static void IGF_DRV_DeInitChannel(uint32_t instance, const uint8_t channelId);
static void IGF_DRV_SetExternalPrescaler(uint32_t instance,uint8_t channelId);

/*FUNCTION**********************************************************************
 *
 * Function Name : IGF_DRV_Init
 * Description   : Initializes input glitch filter according to user input.
 * This function performs the actual implementation-specific initialization
 * based on the provided input filter type configurations.
 * In the instance IGF_0, the EMIOS0_4 and EMIOS0_5 are not supported
 * because the input channels are not connected directly from pads.
 * They are connected from the DSPI_B module.
 * In the instance IGF_1, the EMIOS1_6 and EMIOS1_7 are not supported
 * because the input channels are not connected directly from pads.
 * They are connected from the DSPI_D module.
 *
 * Implements : IGF_DRV_Init_Activity
 *
 *END**************************************************************************/
status_t IGF_DRV_Init(uint32_t instance, const igf_config_t * configPtr)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(configPtr != NULL);
    status_t status = STATUS_SUCCESS;
    uint8_t index = 0U;

    for (index = 0U; index <= MAX_CHANNEL_INSTANCE[instance];index++)
    {
        /* Disable channel before initializes configuration. */
        IGF_DRV_DisableChannel(instance,index);
        if ((index != FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0) && (index != FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1))
        {
            IGF_DRV_SetExternalPrescaler(instance,index);
        }
    }

    for (index = 0U; index < configPtr->configsNumber;index++)
    {
        status = IGF_DRV_InitChannel(instance,&(configPtr->igfChConfig[index]));
        if (status != STATUS_SUCCESS)
        {
            break;
        }
    }

    /* Enable the filter channel even when user initializes correctly. */
    if (status == STATUS_SUCCESS)
    {
        /* Enable channel Id which initializes */
        for (index = 0U; index < configPtr->configsNumber;index++)
        {
            IGF_DRV_EnableChannel(instance,(&configPtr->igfChConfig[index])->channel);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : IGF_DRV_DeInit
 * Description   : De-initialize a input glitch filter instance.
 *
 * Implements : IGF_DRV_DeInit_Activity
 *
 *END**************************************************************************/
void IGF_DRV_DeInit(uint32_t instance)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    uint8_t index = 0U;

    for (index = 0U; index <= MAX_CHANNEL_INSTANCE[instance];index++)
    {
        /* Disable filter of channel */
        IGF_DRV_DisableChannel(instance,index);
        IGF_DRV_DeInitChannel(instance,index);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : IGF_DRV_InitChannel
 * Description   : Initializes input glitch filter for a channel.
 * This function performs the actual implementation-specific initialization
 * based on the provided input filter for each channel which calls by the
 * IGF_DRV_Init function.
 *
 *END**************************************************************************/
static status_t IGF_DRV_InitChannel(uint32_t instance, const igf_ch_param_t * configChannelPtr)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(configChannelPtr != NULL);

    IGF_Type * base = g_igfStatePtr[instance];
    status_t status = STATUS_SUCCESS;
    uint8_t channelId = configChannelPtr->channel;

    if (channelId > MAX_CHANNEL_INSTANCE[instance])
    {
        status = STATUS_ERROR;
    }
    else
    {
        if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
        {
            base->MCR_CH0 &= ~(IGF_MCR_CH0_RFM_MASK | IGF_MCR_CH0_FFM_MASK | IGF_MCR_CH0_FGEN_MASK |
                               IGF_MCR_CH0_POL_MASK | IGF_MCR_CH0_PSSEL_MASK | IGF_MCR_CH0_IMM_MASK |
                               IGF_MCR_CH0_FOL_MASK | IGF_MCR_CH0_FOH_MASK | IGF_MCR_CH0_FBP_MASK |
                               IGF_MCR_CH0_FRZ_MASK | IGF_MCR_CH0_MDIS_MASK);
            base->RTHR_CH0 &= IGF_RTHR_CH0_RTH_MASK;
            base->PRESR_CH0 &= ~IGF_PRESR_CH0_FPRE_MASK;

            base->MCR_CH0 |= IGF_MCR_CH0_FRZ((configChannelPtr->allowDebugMode == true) ? 1UL : 0UL) |
                             IGF_MCR_CH0_POL((configChannelPtr->invertOutput == true) ? 1UL : 0UL)|
                             IGF_MCR_CH0_IMM(configChannelPtr->edgePropagation) |
                             IGF_MCR_CH0_PSSEL(0UL) |
                             IGF_MCR_CH0_FFM(configChannelPtr->filterModeFallingEdge) |
                             IGF_MCR_CH0_RFM(configChannelPtr->filterModeRisingEdge);

            base->PRESR_CH0 |= IGF_PRESR_CH0_FPRE(configChannelPtr->prescalerValue);
            base->FTHR |= IGF_FTHR_FTH(configChannelPtr->fallingEdgeThreshold);
            base->RTHR_CH0 |= IGF_RTHR_CH0_RTH(configChannelPtr->risingEdgeThreshold);
        }
        else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
        {
            base->MCR_CH32 &= ~(IGF_MCR_CH32_RFM_MASK | IGF_MCR_CH32_FFM_MASK | IGF_MCR_CH32_FGEN_MASK |
                                IGF_MCR_CH32_POL_MASK | IGF_MCR_CH32_PSSEL_MASK | IGF_MCR_CH32_IMM_MASK |
                                IGF_MCR_CH32_FOL_MASK | IGF_MCR_CH32_FOH_MASK | IGF_MCR_CH32_FBP_MASK |
                                IGF_MCR_CH32_FRZ_MASK | IGF_MCR_CH32_MDIS_MASK);
            base->RTHR_CH32 &= IGF_RTHR_CH32_RTH_MASK;
            base->PRESR_CH32 &= ~IGF_PRESR_CH32_FPRE_MASK;

            base->MCR_CH32 |= IGF_MCR_CH32_FRZ((configChannelPtr->allowDebugMode == true) ? 1UL : 0UL) |
                              IGF_MCR_CH32_POL((configChannelPtr->invertOutput == true) ? 1UL : 0UL) |
                              IGF_MCR_CH32_IMM(configChannelPtr->edgePropagation) |
                              IGF_MCR_CH32_PSSEL(0UL) |
                              IGF_MCR_CH32_RFM(configChannelPtr->filterModeRisingEdge);
            base->PRESR_CH32 |= IGF_PRESR_CH32_FPRE(configChannelPtr->prescalerValue);
            base->RTHR_CH32 |= IGF_RTHR_CH32_RTH(configChannelPtr->risingEdgeThreshold);
        }
        else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
        {
            base->CHa[channelId - 1U].MCR &= ~(IGF_MCR_RFM_MASK | IGF_MCR_FFM_MASK | IGF_MCR_FGEN_MASK |
                                               IGF_MCR_POL_MASK | IGF_MCR_PSSEL_MASK | IGF_MCR_IMM_MASK |
                                               IGF_MCR_FOL_MASK | IGF_MCR_FOH_MASK | IGF_MCR_FBP_MASK |
                                               IGF_MCR_FRZ_MASK | IGF_MCR_MDIS_MASK);
            base->CHa[channelId - 1U].RTHR &= IGF_RTHR_RTH_MASK;

            base->CHa[channelId - 1U].MCR |= IGF_MCR_FRZ((configChannelPtr->allowDebugMode == true) ? 1UL : 0UL) |
                                             IGF_MCR_POL((configChannelPtr->invertOutput == true) ? 1UL : 0UL) |
                                             IGF_MCR_IMM(configChannelPtr->edgePropagation) |
                                             IGF_MCR_PSSEL(1UL) |
                                             IGF_MCR_RFM(configChannelPtr->filterModeRisingEdge);
            base->CHa[channelId - 1U].RTHR |= IGF_RTHR_RTH(configChannelPtr->risingEdgeThreshold);
        }
        else
        {
            base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR &= ~(IGF_MCR_RFM_MASK | IGF_MCR_FFM_MASK | IGF_MCR_FGEN_MASK |
                                                                           IGF_MCR_POL_MASK | IGF_MCR_PSSEL_MASK | IGF_MCR_IMM_MASK |
                                                                           IGF_MCR_FOL_MASK | IGF_MCR_FOH_MASK | IGF_MCR_FBP_MASK |
                                                                           IGF_MCR_FRZ_MASK | IGF_MCR_MDIS_MASK);
            base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].RTHR &= IGF_RTHR_RTH_MASK;

            base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR |= IGF_MCR_FRZ((configChannelPtr->allowDebugMode == true) ? 1UL : 0UL) |
                                                                         IGF_MCR_POL((configChannelPtr->invertOutput == true) ? 1UL : 0UL) |
                                                                         IGF_MCR_PSSEL(1UL) |
                                                                         IGF_MCR_RFM(configChannelPtr->filterModeRisingEdge);
            base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].RTHR |= IGF_RTHR_RTH(configChannelPtr->risingEdgeThreshold);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : IGF_DRV_DeInitChannel
 * Description   : De-initializes input glitch filter for a channel. This function
 * will be called by the IGF_DRV_DeInit function.
 *
 *END**************************************************************************/
static void IGF_DRV_DeInitChannel(uint32_t instance, const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);

    IGF_Type * base = g_igfStatePtr[instance];

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        base->MCR_CH0 = 0UL;
        base->PRESR_CH0 = 0UL;
        base->FTHR = 0UL;
        base->RTHR_CH0 = 0UL;
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        base->MCR_CH32 = 0UL;
        base->PRESR_CH32 = 0UL;
        base->RTHR_CH32 = 0UL;
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        base->CHa[channelId - 1U].MCR = 0UL;
        base->CHa[channelId - 1U].RTHR = 0UL;
    }
    else
    {
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR = 0UL;
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].RTHR = 0UL;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : IGF_DRV_SetExternalPrescaler
 * Description   : Configure extenal as the prescaler selection of channel.
 *
 *END**************************************************************************/
static void IGF_DRV_SetExternalPrescaler(uint32_t instance,uint8_t channelId)
{
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0U;

    if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->CHa[channelId - 1U].MCR;
        regValue |= IGF_MCR_PSSEL(1UL);
        base->CHa[channelId - 1U].MCR = regValue;
    }
    else
    {
        regValue = base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR;
        regValue |= IGF_MCR_PSSEL(1UL);
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR = regValue;
    }

}

/*FUNCTION***************************************************************************
 *
 * Function Name : IGF_DRV_EnableChannel
 * Description   : This function enable or disable the filter operation.
 *                 The filter is enabled the filtering selected types
 *                 are applied to the rising and falling edges of the input signal.
 *                 When user disable the cluster filter channel of group, all of channel
 *                 in the group will be disabled because that all are turned off internal
 *                 counter prescaler
 *
 * Implements : IGF_DRV_EnableChannel_Activity
 *
 *END********************************************************************************/
void IGF_DRV_EnableChannel(uint32_t instance,const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    IGF_Type * base = g_igfStatePtr[instance];

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        base->MCR_CH0 |= IGF_MCR_CH0_FGEN(1UL);
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        base->MCR_CH32 |= IGF_MCR_CH32_FGEN(1UL);
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        base->CHa[channelId - 1U].MCR |= IGF_MCR_FGEN(1UL);
    }
    else
    {
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR |= IGF_MCR_FGEN(1UL);
    }
}

/*FUNCTION***************************************************************************
 *
 * Function Name : IGF_DRV_DisableChannel
 * Description   : This function disable the filter operation.
 *                 The filter is disabled the filtering selected types
 *                 are applied to the rising and falling edges of the input signal.
 *                 When user disable the cluster filter channel of group, all of channel
 *                 in the group will be disabled because that all are turned off internal
 *                 counter prescaler.
 *
 * Implements : IGF_DRV_DisableChannel_Activity
 *
 *END********************************************************************************/
void IGF_DRV_DisableChannel(uint32_t instance,const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0U;

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        regValue = base->MCR_CH0;
        regValue &= ~IGF_MCR_CH0_FGEN_MASK;
        base->MCR_CH0 = regValue;
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->MCR_CH32;
        regValue &= ~IGF_MCR_CH32_FGEN_MASK;
        base->MCR_CH32 = regValue;
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->CHa[channelId - 1U].MCR;
        regValue &= ~IGF_MCR_FGEN_MASK;
        base->CHa[channelId - 1U].MCR = regValue;
    }
    else
    {
        regValue = base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR;
        regValue &= ~IGF_MCR_FGEN_MASK;
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR = regValue;
    }
}
/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_GetEnableChannel
 * Description   : This function will check the filtering is enabled or disabled.
 *
 * Implements : IGF_DRV_GetEnableChannel_Activity
 *
 *END***************************************************************************************/
bool IGF_DRV_GetEnableChannel(uint32_t instance, const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    const IGF_Type * const base = g_igfStatePtr[instance];
    uint32_t reagValue = 0U;

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        reagValue = ((base->MCR_CH0 & IGF_MCR_CH0_FGEN_MASK) >> IGF_MCR_CH0_FGEN_SHIFT);
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        reagValue = ((base->MCR_CH32 & IGF_MCR_CH32_FGEN_MASK) >> IGF_MCR_CH32_FGEN_SHIFT);
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        reagValue = ((base->CHa[channelId - 1U].MCR & IGF_MCR_FGEN_MASK) >> IGF_MCR_FGEN_SHIFT);
    }
    else
    {
        reagValue = ((base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR & IGF_MCR_FGEN_MASK) >> IGF_MCR_FGEN_SHIFT);
    }

    return ((reagValue == 0UL) ? false : true);
}

/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_SetRisingFilterModeChannel
 * Description   : This function will set the rising filter mode of channel.
 *
 * Implements : IGF_DRV_SetRisingFilterModeChannel_Activity
 *
 *END***************************************************************************************/
void IGF_DRV_SetRisingFilterModeChannel(uint32_t instance, const uint8_t channelId, filter_type_t mode)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0UL;
    bool enableCh = IGF_DRV_GetEnableChannel(instance,channelId);

    /* Disable the channel before user selects the rising filter mode. */
    IGF_DRV_DisableChannel(instance,channelId);
    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        regValue = base->MCR_CH0;
        regValue &= ~IGF_MCR_CH0_RFM_MASK;
        regValue |= IGF_MCR_CH0_RFM(mode);
        base->MCR_CH0 = regValue;
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->MCR_CH32;
        regValue &= ~IGF_MCR_CH32_RFM_MASK;
        regValue |= IGF_MCR_CH32_RFM(mode);
        base->MCR_CH32 = regValue;
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->CHa[channelId - 1U].MCR;
        regValue &= ~IGF_MCR_RFM_MASK;
        regValue |= IGF_MCR_RFM(mode);
        base->CHa[channelId - 1U].MCR = regValue;
    }
    else
    {
        regValue = base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR;
        regValue &= ~IGF_MCR_RFM_MASK;
        regValue |= IGF_MCR_RFM(mode);
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR = regValue;
    }
    if (enableCh == true)
    {
        IGF_DRV_EnableChannel(instance,channelId);
    }
}

/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_SetFallingFilterModeChannel
 * Description   : This function will set the falling filter mode of channel 0.
 *
 * Implements : IGF_DRV_SetFallingFilterModeChannel_Activity
 *
 *END***************************************************************************************/
void IGF_DRV_SetFallingFilterModeChannel(uint32_t instance, filter_type_t mode)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0UL;
    bool enableCh = IGF_DRV_GetEnableChannel(instance,FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0);

    /* Disable the channel before user selects the rising filter mode. */
    IGF_DRV_DisableChannel(instance,FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0);
    regValue = base->MCR_CH0;
    regValue &= ~IGF_MCR_CH0_FFM_MASK;
    regValue |= IGF_MCR_CH0_FFM(mode);
    base->MCR_CH0 = regValue;

    if (enableCh == true)
    {
        IGF_DRV_EnableChannel(instance,FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0);
    }
}

/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_GetRisingFilterModeChannel
 * Description   : This function will check the current rising filter mode of channel.
 *
 * Implements : IGF_DRV_GetRisingFilterModeChannel_Activity
 *
 *END***************************************************************************************/
filter_type_t IGF_DRV_GetRisingFilterModeChannel(uint32_t instance, const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    const IGF_Type * const base = g_igfStatePtr[instance];
    uint32_t regValue = 0U;
    filter_type_t mode = FILTER_BY_PASS;

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        regValue = ((base->MCR_CH0 & IGF_MCR_CH0_RFM_MASK) >> IGF_MCR_CH0_RFM_SHIFT);
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = ((base->MCR_CH32 & IGF_MCR_CH32_RFM_MASK) >> IGF_MCR_CH32_RFM_SHIFT);
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = ((base->CHa[channelId - 1U].MCR & IGF_MCR_RFM_MASK) >> IGF_MCR_RFM_SHIFT);
    }
    else
    {
        regValue = ((base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR & IGF_MCR_RFM_MASK) >> IGF_MCR_RFM_SHIFT);
    }

    switch (regValue)
    {
        case 0U:
            mode = FILTER_BY_PASS;
            break;
        case 1U:
            mode = FILTER_WINDOWING;
            break;
        case 2U:
            mode = FILTER_INTERGRATING;
            break;
        case 3U:
            mode = FILTER_INTERGRATING_HOLD;
            break;
        default:
            /* Nothing to do */
            break;
    }

    return mode;
}

/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_GetFallingFilterModeChannel
 * Description   : This function will check the current falling filter mode of channel.
 *
 * Implements : IGF_DRV_GetFallingFilterModeChannel_Activity
 *
 *END***************************************************************************************/
filter_type_t IGF_DRV_GetFallingFilterModeChannel(uint32_t instance, const uint8_t channelId)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    const IGF_Type * const base = g_igfStatePtr[instance];
    uint32_t regValue = 0U;
    filter_type_t mode = FILTER_BY_PASS;

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        regValue = ((base->MCR_CH0 & IGF_MCR_CH0_FFM_MASK) >> IGF_MCR_CH0_FFM_SHIFT);
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = ((base->MCR_CH32 & IGF_MCR_CH32_FFM_MASK) >> IGF_MCR_CH32_FFM_SHIFT);
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = ((base->CHa[channelId - 1U].MCR & IGF_MCR_FFM_MASK) >> IGF_MCR_FFM_SHIFT);
    }
    else
    {
        regValue = ((base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].MCR & IGF_MCR_FFM_MASK) >> IGF_MCR_FFM_SHIFT);
    }

    switch (regValue)
    {
        case 0U:
            mode = FILTER_BY_PASS;
            break;
        case 1U:
            mode = FILTER_WINDOWING;
            break;
        case 2U:
            mode = FILTER_INTERGRATING;
            break;
        case 3U:
            mode = FILTER_INTERGRATING_HOLD;
            break;
        default:
            /* Nothing to do */
            break;
    }

    return mode;
}

/*FUNCTION********************************************************************************
 *
 * Function Name : IGF_DRV_SetRisingThreshold
 * Description   : This function will set value the filter counter threshold when a rising
 *                 edge is being filtered.
 *
 * Implements : IGF_DRV_SetRisingThreshold_Activity
 *
 *END**************************************************************************************/
void IGF_DRV_SetRisingThreshold(uint32_t instance, const uint8_t channelId, const uint32_t value)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    DEV_ASSERT(channelId <= MAX_CHANNEL_INSTANCE[instance]);
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0UL;

    if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_0)
    {
        regValue = base->RTHR_CH0;
        regValue &= ~IGF_RTHR_CH0_RTH_MASK;
        regValue |= IGF_RTHR_RTH(value);
        base->RTHR_CH0 = regValue;
    }
    else if (channelId == FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->RTHR_CH32;
        regValue &= ~IGF_RTHR_CH32_RTH_MASK;
        regValue |= IGF_RTHR_RTH(value);
        base->RTHR_CH32 = regValue;
    }
    else if (channelId < FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1)
    {
        regValue = base->CHa[channelId - 1U].RTHR;
        regValue &= ~IGF_RTHR_RTH_MASK;
        regValue |= IGF_RTHR_RTH(value);
        base->CHa[channelId - 1U].RTHR = regValue;
    }
    else
    {
        regValue = base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].RTHR;
        regValue &= ~IGF_RTHR_RTH_MASK;
        regValue |= IGF_RTHR_RTH(value);
        base->CHb[channelId % (FEATURE_IGF_CLUSTER_CHANNEL_GROUP_1 + 1U)].RTHR = regValue;
    }
}

/*FUNCTION**********************************************************************************
 *
 * Function Name : IGF_DRV_SetFallingThreshold
 * Description   : This function will set value the filter counter threshold when a falling
 *                 edge is being filtered. This one only sets falling threshold value
 *                 for channel 0.
 *
 * Implements : IGF_DRV_SetFallingThreshold_Activity
 *
 *END***************************************************************************************/
void IGF_DRV_SetFallingThreshold(uint32_t instance, const uint32_t value)
{
    DEV_ASSERT(instance < IGF_INSTANCE_COUNT);
    IGF_Type * base = g_igfStatePtr[instance];
    uint32_t regValue = 0UL;

    regValue = base->FTHR;
    regValue &= ~IGF_FTHR_FTH_MASK;
    regValue |= IGF_RTHR_RTH(value);
    base->FTHR = regValue;
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
