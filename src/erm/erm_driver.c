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
/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
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
 */

#include <stddef.h>
#include "erm_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_Init
 * Description   : Initializes the module
 * Implements    : ERM_DRV_Init_Activity
 *
 *END**************************************************************************/
void ERM_DRV_Init(uint32_t instance,
                  uint8_t channelCnt,
                  const erm_user_config_t * userConfigArr)
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);
    DEV_ASSERT(userConfigArr != NULL);
    DEV_ASSERT(channelCnt > 0U);
    DEV_ASSERT(channelCnt <= ERM_EARn_COUNT);
    uint8_t i;

    /* Initializes the module */
    ERM_Init(instance);

    /* Configure interrupt notification */
    for (i = 0U; i < channelCnt; i++)
    {
        ERM_DRV_SetInterruptConfig(instance, userConfigArr[i].channel, *userConfigArr[i].interruptCfg);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_Deinit
 * Description   : De-initializes the module
 * Implements    : ERM_DRV_Deinit_Activity
 *
 *END**************************************************************************/
void ERM_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);

    /* De-initialize and reset to default configuration */
    ERM_Init(instance);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_SetInterruptConfig
 * Description   : Configures the interrupt notifications
 * Implements    : ERM_DRV_SetInterruptConfig_Activity
 *
 *END**************************************************************************/
void ERM_DRV_SetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t interruptCfg)
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);
    DEV_ASSERT(channel < ERM_EARn_COUNT);

    /* Set the interrupt notifications based on the provided configuration */
    ERM_EnableEventInterrupt(instance, channel, ERM_EVENT_SINGLE_BIT, interruptCfg.enableSingleCorrection);
    ERM_EnableEventInterrupt(instance, channel, ERM_EVENT_NON_CORRECTABLE, interruptCfg.enableNonCorrectable);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_GetInterruptConfig
 * Description   : Returns the configured interrupt notifications
 * Implements    : ERM_DRV_GetInterruptConfig_Activity
 *
 *END**************************************************************************/
void ERM_DRV_GetInterruptConfig(uint32_t instance,
                                uint8_t channel,
                                erm_interrupt_config_t * const interruptPtr)
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);
    DEV_ASSERT(channel < ERM_EARn_COUNT);
    DEV_ASSERT(interruptPtr != NULL);

    /* Fill the interrupt configuration with the enabled interrupt notifications */
    interruptPtr->enableSingleCorrection = ERM_IsEventInterruptEnabled(instance, channel, ERM_EVENT_SINGLE_BIT);
    interruptPtr->enableNonCorrectable = ERM_IsEventInterruptEnabled(instance, channel, ERM_EVENT_NON_CORRECTABLE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_ClearEvent
 * Description   : Clears the record of an error event and the corresponding
 * interrupt notification
 * Implements    : ERM_DRV_ClearEvent_Activity
 *
 *END**************************************************************************/
void ERM_DRV_ClearEvent(uint32_t instance,
                        uint8_t channel,
                        erm_ecc_event_t eccEvent)
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);
    DEV_ASSERT(channel < ERM_EARn_COUNT);

    /* Clear the event record for the specified event type */
    if (eccEvent == ERM_EVENT_SINGLE_BIT)
    {
        ERM_ClearEventSingle(instance, channel);
    }
    else if (eccEvent == ERM_EVENT_NON_CORRECTABLE)
    {
        ERM_ClearEventDouble(instance, channel);
    }
    else
    {
        /* Do nothing - MISRA */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_DRV_GetErrorDetail
 * Description   : Returns the type of the last occurred ECC event and
 * its related information
 * Implements    : ERM_DRV_GetErrorDetail_Activity
 *
 *END**************************************************************************/
erm_ecc_event_t ERM_DRV_GetErrorDetail(uint32_t instance,
                                       uint8_t channel,
                                       uint32_t * addressPtr
#ifdef FEATURE_ERM_ECC_BIT_POS
                                     , uint32_t * bitPosPtr
#endif
                                       )
{
    DEV_ASSERT(instance < ERM_INSTANCE_COUNT);
    DEV_ASSERT(channel < ERM_EARn_COUNT);
    DEV_ASSERT(addressPtr != NULL);
#ifdef FEATURE_ERM_ECC_BIT_POS
    DEV_ASSERT(bitPosPtr != NULL);
#endif
    const ERM_Type * base = s_ermBase[instance];
    erm_ecc_event_t eccEvent;

    /* Check which type of event was detected */
    if (ERM_IsEventDetected(instance, channel, ERM_EVENT_SINGLE_BIT) != false)
    {
        eccEvent = ERM_EVENT_SINGLE_BIT;
    }
    else if (ERM_IsEventDetected(instance, channel, ERM_EVENT_NON_CORRECTABLE) != false)
    {
        eccEvent = ERM_EVENT_NON_CORRECTABLE;
    }
    else
    {
        eccEvent = ERM_EVENT_NONE;
    }

    /* Return details about the error event detected */
    *addressPtr = ERM_GetLastErrorAddress(base, channel);
#ifdef FEATURE_ERM_ECC_BIT_POS
    *bitPosPtr = ERM_GetLastErrorBitPosition(base, channel);
#endif

    return eccEvent;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
