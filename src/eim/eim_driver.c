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
 * @file eim_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned int.
 * The cast is required to initialize a pointer with an unsigned int define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 */

#include "eim_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for EIM instances */
static EIM_Type * const s_eimBase[] = EIM_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_DRV_Init
 * Description   : Initializes the module and configures the channels.
 * Implements    : EIM_DRV_Init_Activity
 *
 *END**************************************************************************/
void EIM_DRV_Init(uint32_t instance,
                  uint8_t channelCnt,
                  const eim_user_channel_config_t *channelConfigArr)
{
    DEV_ASSERT(instance < EIM_INSTANCE_COUNT);
    DEV_ASSERT(channelCnt > 0U);
    DEV_ASSERT(channelCnt <= EIM_EICHDn_COUNT);
    DEV_ASSERT(channelConfigArr != NULL);
    EIM_Type * base = s_eimBase[instance];
    uint8_t index;

    /* Configures the channels */
    for (index = 0U; index < channelCnt; index++)
    {
        EIM_DRV_ConfigChannel(instance, &channelConfigArr[index]);
    }
    /* Enables the module */
    EIM_Enable(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_DRV_Deinit
 * Description   : De-initializes the module.
 * Implements    : EIM_DRV_Deinit_Activity
 *
 *END**************************************************************************/
void EIM_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < EIM_INSTANCE_COUNT);
    EIM_Type * base = s_eimBase[instance];

    /* Disables the module */
    EIM_Disable(base);
    /* Resets the channel descriptors and disables the channels */
    EIM_InitChannel(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_DRV_ConfigChannel
 * Description   : Configures a channel with provided settings.
 * Implements    : EIM_DRV_ConfigChannel_Activity
 *
 *END**************************************************************************/
void EIM_DRV_ConfigChannel(uint32_t instance,
                           const eim_user_channel_config_t *userChannelConfig)
{
    DEV_ASSERT(instance < EIM_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig != NULL);
    DEV_ASSERT(userChannelConfig->channel < EIM_EICHDn_COUNT);
    EIM_Type * base = s_eimBase[instance];

    /* First disable the channel then configure it */
    EIM_EnableChannelCmd(base, userChannelConfig->channel, false);
    /* Configures check-bit mask for the channel */
#if defined(FEATURE_EIM_CHECKBITMASK_32BIT)
    EIM_SetCheckBitMask(base, userChannelConfig->channel, userChannelConfig->checkBitMask);
#else
    EIM_SetCheckBitMask(base, userChannelConfig->channel, (uint32_t) userChannelConfig->checkBitMask);
#endif
    /* Configures data mask(s) for the channel */
#if defined(FEATURE_EIM_DATAMASK1)
    EIM_SetDataMask(base, userChannelConfig->channel, userChannelConfig->dataMask, userChannelConfig->dataMask1);
#else
    EIM_SetDataMask(base, userChannelConfig->channel, userChannelConfig->dataMask, 0UL);
#endif
    /* Configures the channel operation status (enabled/disabled) */
    EIM_EnableChannelCmd(base, userChannelConfig->channel, userChannelConfig->enable);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_DRV_GetChannelConfig
 * Description   : Returns the configuration of the provided channel.
 * Implements    : EIM_DRV_GetChannelConfig_Activity
 *
 *END**************************************************************************/
void EIM_DRV_GetChannelConfig(uint32_t instance,
                              uint8_t channel,
                              eim_user_channel_config_t *channelConfig)
{
    DEV_ASSERT(instance < EIM_INSTANCE_COUNT);
    DEV_ASSERT(channel < EIM_EICHDn_COUNT);
    DEV_ASSERT(channelConfig != NULL);
    const EIM_Type * base = s_eimBase[instance];
    uint32_t tmpDataMask0 = 0UL;
    uint32_t tmpDataMask1 = 0UL;

    /* Fills the configuration structure with the module parameters set for the channel */
    channelConfig->channel = channel;
#if defined(FEATURE_EIM_CHECKBITMASK_32BIT)
    channelConfig->checkBitMask = EIM_GetCheckBitMask(base, channel);
#else
    channelConfig->checkBitMask = (uint8_t) EIM_GetCheckBitMask(base, channel);
#endif
    EIM_GetDataMask(base, channel, &tmpDataMask0, &tmpDataMask1);
    channelConfig->dataMask = tmpDataMask0;
#if defined(FEATURE_EIM_DATAMASK1)
    channelConfig->dataMask1 = tmpDataMask1;
#endif
    channelConfig->enable = EIM_IsChannelEnabled(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_DRV_GetDefaultConfig
 * Description   : Returns a default configuration for the provided channel.
 * Implements    : EIM_DRV_GetDefaultConfig_Activity
 *
 *END**************************************************************************/
void EIM_DRV_GetDefaultConfig(uint8_t channel,
                              eim_user_channel_config_t *channelConfig)
{
    DEV_ASSERT(channelConfig != NULL);
    DEV_ASSERT(channel < EIM_EICHDn_COUNT);

    /* Fills the configuration structure with the default module parameters for the channel */
    channelConfig->channel = channel;
    channelConfig->checkBitMask = EIM_CHECKBITMASK_DEFAULT;
    channelConfig->dataMask = EIM_DATAMASK_DEFAULT;
#if defined(FEATURE_EIM_DATAMASK1)
    channelConfig->dataMask1 = EIM_DATAMASK_DEFAULT;
#endif
    channelConfig->enable = true;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
