/*
 * Copyright 2017-2018 NXP
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
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or
 * different essential type.
 * This is required by the conversion of a unsigned type into a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a unsigned type into a enum type.
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
#include "wkpu_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for WKPU instances. */
static WKPU_Type * const s_wkpuBase[] = WKPU_BASE_PTRS;

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Local functions
 ******************************************************************************/
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_InitNMI
 * Description   : This function sets the NMI configuration.
 * Note that: The user must make sure that the NMI configuration must have configure for
 * core 0, the filter only configured at this core, other cores will be ignored
 *
 * Implements    : WKPU_DRV_InitNMI_Activity
 *END**************************************************************************/
status_t WKPU_DRV_InitNMI(uint32_t instance,
                          uint8_t coreCnt,
                          const wkpu_nmi_cfg_t * pNmiConfig)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(coreCnt > 0U);
    DEV_ASSERT(coreCnt <= FEATURE_WKPU_NMI_NUM_CORES);
    DEV_ASSERT(pNmiConfig != NULL);

    WKPU_Type * base = s_wkpuBase[instance];
    status_t retVal = STATUS_SUCCESS;
    uint32_t coreShift = 0U;
    uint8_t coreNumber;
    uint8_t i;

    retVal = WKPU_DRV_DeinitNMI(instance);

    if(STATUS_SUCCESS == retVal)
    {
        for (i = 0U; i < coreCnt; i++)
        {
            coreNumber = (uint8_t)pNmiConfig[i].core;
            coreShift = coreNumber * (uint32_t)FEATURE_WKPU_CORE_OFFSET_SIZE;

            /* Configure destination source */
            WKPU_SetNMIDestinationSrc(base, coreShift, (uint8_t)pNmiConfig[i].destinationSrc);

            /* Configure wake-up request */
            WKPU_SetNMIWakeupRequest(base, coreShift, pNmiConfig[i].wkpReqEn);

            /* Only set filter for Core 0 */
            if (coreNumber == 0UL)
            {
                /* Configure glitch filter */
                WKPU_SetNMIFilter(base, coreShift, pNmiConfig[i].filterEn);
            }

            /* Configure edge events */
            WKPU_SetNMIRisingEdge(base, coreShift, (uint8_t)pNmiConfig[i].edgeEvent & 0x01U);
            WKPU_SetNMIFallingEdge(base, coreShift, (uint8_t)pNmiConfig[i].edgeEvent & 0x02U);

            /* Configure lock */
            WKPU_SetNMIConfigLock(base, coreShift, pNmiConfig[i].lockEn);
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_DeinitNMI
 * Description   : This function de-initializes NMI of the WKPU module.
 * Reset NMI configuration, disable Wake-up, clear filter enable, edge events enable
 *
 * Implements    : WKPU_DRV_DeinitNMI_Activity
 *END**************************************************************************/
status_t WKPU_DRV_DeinitNMI(uint32_t instance)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);

    uint8_t i;
    uint32_t coreShift = 0U;
    status_t retVal = STATUS_SUCCESS;
    static const wkpu_core_t coreNumber[FEATURE_WKPU_NMI_NUM_CORES] = FEATURE_WKPU_CORE_ARRAY;

    WKPU_Type * base = s_wkpuBase[instance];

    for (i = 0U; i < FEATURE_WKPU_NMI_NUM_CORES; i++)
    {
        coreShift = (uint32_t)coreNumber[i] * FEATURE_WKPU_CORE_OFFSET_SIZE;

        if (WKPU_IsNMIConfigLock(base, coreShift) == false)
        {
            /* Clear status flag and overrun status flag */
            WKPU_ClearStatusFlag(base, (WKPU_NSR_NIF0_MASK | WKPU_NSR_NOVF0_MASK) >> coreShift);
            /* Clear edge events */
            WKPU_SetNMIRisingEdge(base, coreShift, WKPU_EDGE_NONE);
            WKPU_SetNMIFallingEdge(base, coreShift, WKPU_EDGE_NONE);

            /* Only set filter for Core 0 */
            if (WKPU_CORE0 == coreNumber[i])
            {
                /* Disable glitch filter */
                WKPU_SetNMIFilter(base, coreShift, false);
            }

            /* Disable wake-up request */
            WKPU_SetNMIWakeupRequest(base, coreShift, false);

        #ifdef FEATURE_WKPU_SUPPORT_NONE_REQUEST
            /* Configure destination source */
            WKPU_SetNMIDestinationSrc(base, coreShift, WKPU_NMI_NONE);
        #endif
        }
        else
        {
            retVal = STATUS_ERROR;
            break;
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_GetNMIDefaultConfig
 * Description   : This function gets NMI default configuration.
 * Note that: The user need provides an array have maximum 3 elements of NMI configuration.
 *
 * Implements    : WKPU_DRV_GetNMIDefaultConfig_Activity
 *END**************************************************************************/
void WKPU_DRV_GetNMIDefaultConfig(wkpu_nmi_cfg_t * const pNmiConfig)
{
    DEV_ASSERT(pNmiConfig != NULL);

    uint8_t i;
    wkpu_core_t coreNumber[FEATURE_WKPU_NMI_NUM_CORES] = FEATURE_WKPU_CORE_ARRAY;

    for (i = 0U; i < FEATURE_WKPU_NMI_NUM_CORES; i++)
    {
        pNmiConfig[i].core = coreNumber[i];
        pNmiConfig[i].destinationSrc = WKPU_NMI_NON_MASK_INT;
        pNmiConfig[i].wkpReqEn = true;
        pNmiConfig[i].filterEn = false;
        pNmiConfig[i].edgeEvent = WKPU_EDGE_BOTH;
        pNmiConfig[i].lockEn = false;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_ClearNMIFlag
 * Description   : This function clears the NMI (status or overrun) flag for each core.
 *
 * Implements    : WKPU_DRV_ClearNMIFlag_Activity
 *END**************************************************************************/
void WKPU_DRV_ClearNMIFlag(uint32_t instance,
                           wkpu_core_t core,
                           wkpu_status_flag_t flag)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    WKPU_Type * base = s_wkpuBase[instance];

    if (flag == WKPU_FLAG_STATUS)
    {
        WKPU_ClearStatusFlag(base, WKPU_NSR_NIF0_MASK >> ((uint8_t)core * FEATURE_WKPU_CORE_OFFSET_SIZE));
    }
    else
    {
        WKPU_ClearStatusFlag(base, WKPU_NSR_NOVF0_MASK >> ((uint8_t)core * FEATURE_WKPU_CORE_OFFSET_SIZE));
    }
}

#ifdef FEATURE_WKPU_SUPPORT_INTERRUPT_REQUEST
/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_InitInterrupt
 * Description   : This function initializes interrupt WKPU driver based on user configuration input.
 * The channelCnt takes values between 1 and the maximum channel count supported by the hardware.
 *
 * Implements    : WKPU_DRV_InitInterrupt_Activity
 *END**************************************************************************/
status_t WKPU_DRV_InitInterrupt(uint32_t instance,
                                uint8_t channelCnt,
                                const wkpu_interrupt_cfg_t * pInterruptConfig)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(pInterruptConfig != NULL);
    DEV_ASSERT(channelCnt > 0U);
    DEV_ASSERT(channelCnt <= FEATURE_WKPU_MAX_CHANNEL_COUNT);
    uint8_t i;

    /* Configure for Interrupt */
    for (i = 0U; i < channelCnt; i++)
    {
        WKPU_DRV_SetInterruptConfig(instance, &pInterruptConfig[i]);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_DeinitInterrupt
 * Description   : This function de-initializes the interrupt WKPU module.
 * Reset interrupt configuration, disable IRQ and Wake-up, clear filter enable,
 * pull-up enable, edge events enable.
 *
 * Implements    : WKPU_DRV_DeinitInterrupt_Activity
 *END**************************************************************************/
status_t WKPU_DRV_DeinitInterrupt(uint32_t instance)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];

    /* Disable interrupt request */
    WKPU_EnableInterrupt(base, 0xFFFFFFFFUL, false);

    /* Disable Wakeup/Interrupt Filter Enable Register  */
    WKPU_EnableFilter(base, 0xFFFFFFFFUL, false);

    /* Disable Wakeup/Interrupt Pull-up Enable Register */
    WKPU_EnablePull(base, 0xFFFFFFFFUL, false);

    /* Disable edge events registers */
    WKPU_EnableRisingEdge(base, 0xFFFFFFFFUL, false);
    WKPU_EnableFallingEdge(base, 0xFFFFFFFFUL, false);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_GetInterruptDefaultConfig
 * Description   : This function gets interrupt default configuration.
 * Note that: The user need provides an array have 32 elements of interrupt configuration.
 *
 * Implements    : WKPU_DRV_GetInterruptDefaultConfig_Activity
 *END**************************************************************************/
void WKPU_DRV_GetInterruptDefaultConfig(wkpu_interrupt_cfg_t * const pInterruptConfig)
{
    DEV_ASSERT(pInterruptConfig != NULL);

    uint8_t i;

    for (i = 0U; i < FEATURE_WKPU_MAX_CHANNEL_COUNT; i++)
    {
        pInterruptConfig[i].hwChannel = i;
        pInterruptConfig[i].edgeEvent = WKPU_EDGE_BOTH;
        pInterruptConfig[i].filterEn = false;
        pInterruptConfig[i].pullEn = false;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_SetInterruptConfig
 * Description   : This function sets the interrupt configuration based on
 * interrupt configuration input.
 *
 * Implements    : WKPU_DRV_SetInterruptConfig_Activity
 *END**************************************************************************/
void WKPU_DRV_SetInterruptConfig(uint32_t instance,
                                 const wkpu_interrupt_cfg_t * pInterruptConfig)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(pInterruptConfig != NULL);

    WKPU_Type * base = s_wkpuBase[instance];
    uint8_t hwChannel = pInterruptConfig->hwChannel;
    uint32_t channelMask = 1UL << hwChannel;

    /* Disable interrupt request */
    WKPU_EnableInterrupt(base, channelMask, false);

    /* Set Wakeup/Interrupt Filter Enable Register */
    WKPU_EnableFilter(base, channelMask, pInterruptConfig->filterEn);

    /* Set Wakeup/Interrupt Pull-up Enable Register */
    WKPU_EnablePull(base, channelMask, pInterruptConfig->pullEn);

    /* Set edge events enable registers */
    WKPU_DRV_SetInterruptEdgeEvent(instance, hwChannel, pInterruptConfig->edgeEvent);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_ClearInterruptConfig
 * Description   : This function clears interrupt configuration, disable IRQ and Wake-up,
 * clear filter enable, pull-up enable, edge events enable.
 *
 * Implements    : WKPU_DRV_ClearInterruptConfig_Activity
 *END**************************************************************************/
void WKPU_DRV_ClearInterruptConfig(uint32_t instance,
                                   uint8_t hwChannel)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(hwChannel < FEATURE_WKPU_MAX_CHANNEL_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];
    uint32_t channelMask = 1UL << hwChannel;

    /* Disable interrupt request */
    WKPU_EnableInterrupt(base, channelMask, false);

    /* Disable Wakeup/Interrupt Filter Enable Register */
    WKPU_EnableFilter(base, channelMask, false);

    /* Disable Wakeup/Interrupt Pull-up Enable Register */
    WKPU_EnablePull(base, channelMask, false);

    /* Clear edge events enable registers */
    WKPU_DRV_SetInterruptEdgeEvent(instance, hwChannel, WKPU_EDGE_NONE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_SetInterruptNormalMode
 * This function sets interrupt normal mode(enable) for the WKPU,
 * enable interrupt and wake-up
 *
 * Implements    : WKPU_DRV_SetInterruptNormalMode_Activity
 *END**************************************************************************/
void WKPU_DRV_SetInterruptNormalMode(uint32_t instance,
                                     uint8_t hwChannel)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(hwChannel < FEATURE_WKPU_MAX_CHANNEL_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];

    /* Enable IRQ interrupt */
    WKPU_EnableInterrupt(base, 1UL << hwChannel, true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_SetInterruptSleepMode
 * Description   : This function sets sleep mode for the WKPU or disable interrupt
 * and wake-up.
 *
 * Implements    : WKPU_DRV_SetInterruptSleepMode_Activity
 *END**************************************************************************/
void WKPU_DRV_SetInterruptSleepMode(uint32_t instance,
                                    uint8_t hwChannel)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(hwChannel < FEATURE_WKPU_MAX_CHANNEL_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];

    /* Disable interrupt request */
    WKPU_EnableInterrupt(base, 1UL << hwChannel, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_SetInterruptEdgeEvent
 * Description   : This function sets edge events for each channel of the WKPU.
 *
 * Implements    : WKPU_DRV_SetInterruptEdgeEvent_Activity
 *END**************************************************************************/
void WKPU_DRV_SetInterruptEdgeEvent(uint32_t instance,
                                    uint8_t hwChannel,
                                    wkpu_edge_event_t edge)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(hwChannel < FEATURE_WKPU_MAX_CHANNEL_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];

    switch (edge)
    {
        case WKPU_EDGE_RISING:
            WKPU_EnableRisingEdge(base, 1UL << hwChannel, true);
            WKPU_EnableFallingEdge(base, 1UL << hwChannel, false);
            break;
        case WKPU_EDGE_FALLING:
            WKPU_EnableRisingEdge(base, 1UL << hwChannel, false);
            WKPU_EnableFallingEdge(base, 1UL << hwChannel, true);
            break;
        case WKPU_EDGE_NONE:
            WKPU_EnableRisingEdge(base, 1UL << hwChannel, false);
            WKPU_EnableFallingEdge(base, 1UL << hwChannel, false);
            break;
        case WKPU_EDGE_BOTH:
        default:
            /* fall-through */
            WKPU_EnableRisingEdge(base, 1UL << hwChannel, true);
            WKPU_EnableFallingEdge(base, 1UL << hwChannel, true);
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_ClearInterruptFlag
 * Description   : This function clears interrupt flag for channel mask.
 *
 * Implements    : WKPU_DRV_ClearInterruptFlag_Activity
 *END**************************************************************************/
void WKPU_DRV_ClearInterruptFlag(uint32_t instance,
                                 uint8_t hwChannel)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(hwChannel < FEATURE_WKPU_MAX_CHANNEL_COUNT);

    WKPU_Type * base = s_wkpuBase[instance];

    /* Clear interrupt flags */
    WKPU_ClearInterruptFlag(base, 1UL << hwChannel);
}
#endif

#ifdef FEATURE_WKPU_SUPPORT_RESET_REQUEST
/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_InitReset
 * Description   : This function sets reset configuration of the WKPU based on
 * reset configuration input.
 *
 * Implements    : WKPU_DRV_InitReset_Activity
 *END**************************************************************************/
status_t WKPU_DRV_InitReset(uint32_t instance,
                            const wkpu_reset_cfg_t * pResetConfig)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    DEV_ASSERT(pResetConfig != NULL);

    WKPU_Type * base = s_wkpuBase[instance];
    status_t retVal = STATUS_SUCCESS;
    uint8_t coreNumber = FEATURE_WKPU_RESET_POSITION;
    uint32_t coreShift = coreNumber * (uint32_t)FEATURE_WKPU_CORE_OFFSET_SIZE;

    /* Reset register to default */
    retVal = WKPU_DRV_DeinitReset(instance);
    /* Check status for de-initiating function */
    if (STATUS_SUCCESS == retVal)
    {

        /* Configure reset destination source */
        WKPU_SetNMIDestinationSrc(base, coreShift, (uint8_t)pResetConfig->destinationSrc);

        /* Configure reset wake-up request */
        WKPU_SetNMIWakeupRequest(base, coreShift, pResetConfig->wkpReqEn);

        /* Configure glitch filter of NMI */
        WKPU_SetNMIFilter(base, 0U, true);

        /* Configure reset edge events */
        WKPU_SetNMIRisingEdge(base, coreShift, (uint8_t)pResetConfig->edgeEvent & 0x01U);
        WKPU_SetNMIFallingEdge(base, coreShift, (uint8_t)pResetConfig->edgeEvent & 0x02U);

        /* Configure reset lock */
        WKPU_SetNMIConfigLock(base, coreShift, pResetConfig->lockEn);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_DeinitReset
 * Description   : This function de-initializes the Reset WKPU module. Reset configuration,
 * disable Wake-up, edge events enable and lock enable.
 *
 * Implements    : WKPU_DRV_DeinitReset_Activity
 *END**************************************************************************/
status_t WKPU_DRV_DeinitReset(uint32_t instance)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);

    uint32_t coreShift = 0U;
    status_t retVal = STATUS_SUCCESS;

    WKPU_Type * base = s_wkpuBase[instance];

    coreShift = (uint32_t)FEATURE_WKPU_RESET_POSITION * FEATURE_WKPU_CORE_OFFSET_SIZE;

    if (WKPU_IsNMIConfigLock(base, coreShift) == false)
    {
        /* Clear status flag and overrun status flag */
        WKPU_ClearStatusFlag(base, (WKPU_NSR_NIF0_MASK | WKPU_NSR_NOVF0_MASK) >> coreShift);
        /* Clear edge events */
        WKPU_SetNMIRisingEdge(base, coreShift, 0U);
        WKPU_SetNMIFallingEdge(base, coreShift, 0U);
        /* Disable wake-up request */
        WKPU_SetNMIWakeupRequest(base, coreShift, false);
        /* Configure destination source */
        WKPU_SetNMIDestinationSrc(base, coreShift, WKPU_NMI_NONE);
    }
    else
    {
        retVal = STATUS_ERROR;
    }


    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_GetResetDefaultConfig
 * Description   : This function gets Reset default configuration.
 *
 * Implements    : WKPU_DRV_GetResetDefaultConfig_Activity
 *END**************************************************************************/
void WKPU_DRV_GetResetDefaultConfig(wkpu_reset_cfg_t * const pResetConfig)
{
    DEV_ASSERT(pResetConfig != NULL);

    pResetConfig->destinationSrc = WKPU_RESET_REQ_RGM;
    pResetConfig->wkpReqEn = true;
    pResetConfig->edgeEvent = WKPU_EDGE_BOTH;
    pResetConfig->lockEn = false;

}


/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_DRV_ClearResetFlag
 * Description   : This function clears the reset (status or overrun) flag.
 *
 * Implements    : WKPU_DRV_ClearResetFlag_Activity
 *END**************************************************************************/
void WKPU_DRV_ClearResetFlag(uint32_t instance,
                             wkpu_status_flag_t flag)
{
    DEV_ASSERT(instance < WKPU_INSTANCE_COUNT);
    WKPU_Type * base = s_wkpuBase[instance];

    if (flag == WKPU_FLAG_STATUS)
    {
        WKPU_ClearStatusFlag(base, WKPU_NSR_RIF_MASK);
    }
    else
    {
        WKPU_ClearStatusFlag(base, WKPU_NSR_ROVF_MASK);
    }
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
