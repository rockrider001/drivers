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
 * @file etimer_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, auto variable
 * Taking address of near auto variable.
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, essential type
 * Expression assigned to a narrower or different essential type.
 *
 *  @section [global]
 * Violates MISRA 2012 Required Rule 10.7, other operand.
 * Composite expression with smaller essential type than other operand.
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
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * This is required because it makes the code easier to read.
 *
 */

#include <stddef.h>
#include "etimer_driver.h"
#include "etimer_hw_access.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for ETIMER instances */
static ETIMER_Type * const s_etimerBase[ETIMER_INSTANCE_COUNT] = ETIMER_BASE_PTRS;

/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_Init
 * Description   : Initializes ETIMER module.
 * This function Initializes the ETIMER module with default values.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other ETIMER driver function.
 *
 * Implements    : ETIMER_DRV_Init_Activity
 *END**************************************************************************/
void ETIMER_DRV_Init(const uint16_t instance)
{
    /* register pointer */
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);

    base = s_etimerBase[instance];
    /* Resets ETIMER module */
    ETIMER_Reset(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_Deinit
 * Description   : De-initializes ETIMER module.
 * This function disables ETIMER module.
 * In order to use the ETIMER module again, ETIMER_DRV_Init must be called.
 *
 * Implements    : ETIMER_DRV_Deinit_Activity
 *END**************************************************************************/
void ETIMER_DRV_Deinit(const uint16_t instance)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);

    base = s_etimerBase[instance];

    /* Stops timer channel from counting */
    ETIMER_Reset(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfig
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfig_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfig(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    config->countMode = ETIMER_CNTMODE_PRIMARY;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;
    config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_1;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;
    config->countDirection = ETIMER_COUNT_UP;
    config->compareValues[0] = 0;
    config->compareValues[1] = 0;
    config->compareLoading[0] = ETIMER_CLC_NEVER;
    config->compareLoading[1] = ETIMER_CLC_NEVER;
    config->compareOutputControl = ETIMER_OUTMODE_SOFTWARE;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
    config->captureWords = 0;
    config->interruptEnableMask = 0;
    config->oneshotCapture = false;
    config->countOnce = false;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
    config->countLength = false;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultDmaChannel
 * Description   : This function gets the default configuration structure for DMA access
 *
 * Implements    : ETIMER_DRV_GetDefaultDmaChannel_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultDmaChannel(etimer_dma_channel_t *config){

    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    config->enable = false;
    config->request = ETIMER_DMA_CH0_CAPT1;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_InitChannel
 * Description   : Initializes ETIMER channel.
 * This function initializes the ETIMER timers by using a channel, this function
 * configures timer channel chaining, timer channel mode, timer channel period,
 * interrupt generation, trigger source, trigger select, reload on trigger,
 * stop on interrupt and start on trigger.
 * The timer channel number and its configuration structure shall be passed as arguments.
 * Timer channels do not start counting by default after calling this function.
 * The function ETIMER_DRV_StartTimerChannels must be called to start the timer channel counting.
 * In order to re-configures the period, call the ETIMER_DRV_SetTimerPeriodByUs or
 * ETIMER_DRV_SetTimerPeriodByCount.
 *
 * Implements    : ETIMER_DRV_InitChannel_Activity
 *END**************************************************************************/
void ETIMER_DRV_InitChannel(const uint16_t instance,
                            const uint16_t channel,
                            const etimer_user_channel_config_t * const userChannelConfig)
{
    ETIMER_Type * base;
    uint16_t ctrl1_cntmode=0;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig != NULL);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* if channel runs, do not init */
    DEV_ASSERT((base->ENBL & (1UL << channel)) == 0U);

    /* stop the channel activity */
    base->CH[channel].CTRL1 &= ~ETIMER_CTRL1_CNTMODE_MASK;
    /* stop all interrupts */
    base->CH[channel].INTDMA &= (uint16_t)(~(ETIMER_CH_IRQ_SOURCE_MASK));
    /* clear all interrupt flags */
    base->CH[channel].STS = ETIMER_CH_IRQ_FLAGS_MASK;

    /* reset channel to default state */
    ETIMER_Reset_Channel(base, channel);

    /* Master and coChannel*/
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_MSTR(userChannelConfig->masterMode ? (uint16_t)1U : (uint16_t)0U);
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_COFRC(userChannelConfig->coChannelForce ? (uint16_t)1U : (uint16_t)0U);
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_COINIT(userChannelConfig->coChannelInit);
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_VAL(userChannelConfig->coChannelVal ? (uint16_t)1U : (uint16_t)0U);

    /* redundant mode */
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_RDNT(userChannelConfig->redundant ? (uint16_t)1U : (uint16_t)0U);

    /* DIR, PRISRC, SECSRC, SIPS, OPS, OEN */
    ETIMER_DIR_PRISRC_SECSRC_SIPS(base, channel, userChannelConfig);

    /* COMPx */
    ETIMER_COMPx(base, channel, userChannelConfig);

    /* CLC1 and CLC2 and other CCCTRL */
    ETIMER_CLC1_CLC2_CCCTRL(base, channel, userChannelConfig);

    /* OUTMODE */
    ETIMER_OUTMODE(base, channel, userChannelConfig);

    /* INTDMA */
    base->CH[channel].INTDMA |= (userChannelConfig->interruptEnableMask & ETIMER_CH_IRQ_SOURCE_MASK);

    /* CNTMODE and ONCE and COUNT LENGTH */
    ctrl1_cntmode |= ETIMER_CTRL1_CNTMODE(userChannelConfig->countMode) | \
                     ETIMER_CTRL1_ONCE(userChannelConfig->countOnce ? (uint16_t)1U : (uint16_t)0U) |
                     ETIMER_CTRL1_LENGTH(userChannelConfig->countLength ? (uint16_t)1U : (uint16_t)0U);

    /* Configure input filter */
    base->CH[channel].FILT |= ETIMER_FILT_FILT_CNT(userChannelConfig->inputFilter.samples) | \
                              ETIMER_FILT_FILT_PER(userChannelConfig->inputFilter.rate);

    /* start the channel activity */
    base->CH[channel].CTRL1 |= ctrl1_cntmode;

    /* everything is ok if we reached this point */
    return;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_DmaInitRequest
 * Description   : Initializes a single DMA request channel.
 * This function should setup and enable or disable a single DMA channel
 * which is available to the ETIMER instance.
 * The request number and its configuration structure shall be passed as arguments.
 * Check for each DREQ the DMAMUX source slot in eDMA.
 *
 *
 * Implements    : ETIMER_DRV_DmaInitRequest_Activity
 *END**************************************************************************/
void ETIMER_DRV_DmaInitRequest(const uint16_t instance,
                               const uint16_t dmaRequestNr,
                               const etimer_dma_channel_t * const singleDmaConfig)
{
    /* register pointer */
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(dmaRequestNr < ETIMER_DREQ_COUNT);

    base = s_etimerBase[instance];
    base->DREQ[dmaRequestNr] = 0;

    /* check if enabled  */
    if(singleDmaConfig->enable)
    {
        base->DREQ[dmaRequestNr] = ETIMER_DREQ_DREQ0(singleDmaConfig->request);
        base->DREQ[dmaRequestNr] |= ETIMER_DREQ_DREQ0_EN_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_ChannelDebugBehaviour
 * Description   : Sets debugging options.
 * This function set the way each channel behaves when using a HW debugger.
 * The timer channel number and its configuration structure shall be passed as arguments.
 *
 * The actions are (etimer_debug_mode_t) :
 *   ETIMER_DEBUG_DEFAULT = Continue with normal operation during debug mode
 *   ETIMER_DEBUG_HALT    = Halt channel counter during debug mode
 *   ETIMER_DEBUG_OFLAG_LOW  = Force OFLAG to logic 0 (prior to consideration of the OPS bit) during debug mode
 *   ETIMER_DEBUG_COMBINED   =  Both halt counter and force OFLAG to 0 during debug mode
 *
 * Implements    : ETIMER_DRV_ChannelDebugBehaviour_Activity
 *END**************************************************************************/
void ETIMER_DRV_ChannelDebugBehaviour(const uint16_t instance,
                                      const uint16_t channel,
                                      const etimer_debug_mode_t debug)
{
    /* register pointer */
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];
    /* debug mode set */
    base->CH[channel].CTRL3 &= ~ETIMER_CTRL3_DBGEN_MASK;
    base->CH[channel].CTRL3 |= ETIMER_CTRL3_DBGEN(debug);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_StartTimerChannels
 * Description   : Starts timer channel counting.
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * Implements    : ETIMER_DRV_StartTimerChannels_Activity
 *END**************************************************************************/
void ETIMER_DRV_StartTimerChannels(const uint16_t instance,
                                   const uint16_t mask)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << ETIMER_CH_COUNT));

    base = s_etimerBase[instance];
    /* Starts timer channel counting */
    base->ENBL |= mask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_StopTimerChannels
 * Description   : Stop timer channel from counting.
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the ETIMER_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * Implements    : ETIMER_DRV_StopTimerChannels_Activity
 *END**************************************************************************/
void ETIMER_DRV_StopTimerChannels(const uint16_t instance,
                                uint16_t mask)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(mask < (1UL << ETIMER_CH_COUNT));

    base = s_etimerBase[instance];
    /* Stops timer channel from counting */
    base->ENBL &= ~(mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetTimerTicks
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : ETIMER_DRV_SetTimerTicks_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetTimerTicks(const uint16_t instance,
                              const uint16_t channel,
                              const uint16_t count)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);
    DEV_ASSERT(count < ETIMER_CH_MAX_TIMER_COUNT);

    base = s_etimerBase[instance];
    /* Sets the timer channel period in count unit */
    base->CH[channel].CNTR = count;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetTimerTicks
 * Description   : Gets the current timer channel counting value in count.
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * Implements    : ETIMER_DRV_GetTimerTicks_Activity
 *END**************************************************************************/
uint16_t ETIMER_DRV_GetTimerTicks(const uint16_t instance,
                                  const uint16_t channel)
{
    const ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];
    /* Sets the timer channel counts */
    return (base->CH[channel].CNTR);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetTimerTicksBuffered
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : ETIMER_DRV_SetTimerTicksBuffered_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetTimerTicksBuffered(const uint16_t instance,
                                      const uint16_t channel,
                                      const uint16_t count)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);
    DEV_ASSERT(count < ETIMER_CH_MAX_TIMER_COUNT);

    base = s_etimerBase[instance];
    /* Sets the timer channel period in count unit */
    base->CH[channel].LOAD = count;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_EnableInterruptSource
 * Description   : This function sets the interrupt enable bits of timer channels.
 * All unset interrupt sources in the mask will keep their current state.
 *
 * Implements    : ETIMER_DRV_EnableInterruptSource_Activity
 *END**************************************************************************/
void ETIMER_DRV_EnableInterruptSource(const uint16_t instance,
                                      const uint16_t mask,
                                      const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    base->CH[channel].INTDMA |= mask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_DisableInterruptSource
 * Description   : This function disables the interrupt source for a timer channel.
 * All set interrupt sources in the mask will keep their current state.
 *
 * Implements    : ETIMER_DRV_DisableInterruptSource_Activity
 *END**************************************************************************/
void ETIMER_DRV_DisableInterruptSource(const uint16_t instance,
                                       const uint16_t mask,
                                       const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    base->CH[channel].INTDMA &= ~(mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_EnableDisableInterruptSources
 * Description   :  Enables and disables the interrupt sources for a timer channel.
 * This function sets the interrupt enable bits of timer channel and clears
 * all other interrupt sources which are not set through this operation.
 *
 * Implements    : ETIMER_DRV_EnableDisableInterruptSources_Activity
 *END**************************************************************************/
void ETIMER_DRV_EnableDisableInterruptSources(const uint16_t instance,
                                              const uint16_t mask,
                                              const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    base->CH[channel].INTDMA = mask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetInterruptStatus
 * Description   : This function gets the current interrupt flags of selected channels.
 * The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 *
 * Implements    : ETIMER_DRV_GetInterruptStatus_Activity
 *END**************************************************************************/
uint16_t ETIMER_DRV_GetInterruptStatus(const uint16_t instance,
                                       const uint16_t mask,
                                       const uint16_t channel)
{
    const ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];
    /* Gets the interrupt flag for timer channels */
    return (base->CH[channel].STS) & mask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_ClearInterruptStatus
 * Description   : Clears the interrupt flag of timer channels.
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * Implements    : ETIMER_DRV_ClearInterruptStatus_Activity
 *END**************************************************************************/
void ETIMER_DRV_ClearInterruptStatus(const uint16_t instance,
                                     const uint16_t mask,
                                     const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];
    /* Clears the interrupt flag for timer channels */
    base->CH[channel].STS = mask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetInterruptNumber
 * Description   : Clears the interrupt flag of timer channels.
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 * Implements    : ETIMER_DRV_GetInterruptNumber_Activity
 *END**************************************************************************/
IRQn_Type ETIMER_DRV_GetInterruptNumber(const uint16_t instance,
                                        const etimer_irq_vector_t vector)
{
    /* table for IRQ vectors */
    static const IRQn_Type etimerIrqId[ETIMER_INSTANCE_COUNT][ETIMER_CH_MAX_IRQS] = ETIMER_IRQS;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT((uint16_t)vector < ETIMER_CH_MAX_IRQS);
    DEV_ASSERT(etimerIrqId[instance][vector] != NotAvail_IRQn);

    return (etimerIrqId[instance][vector]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetCompareThreshold
 * Description   : Sets the timer channel compare registers in count unit.
 * This function sets the timer channel two compare registers.
 * The values will be loaded immediately, glitches may occur.
 *
 * Implements    : ETIMER_DRV_SetCompareThreshold_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetCompareThreshold(const uint16_t instance,
                                    const uint16_t channel,
                                    const uint16_t comp1,
                                    const uint16_t comp2)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set compare values */
    base->CH[channel].COMP1 = comp1;
    base->CH[channel].COMP2 = comp2;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetCompareThresholdBuffered
 * Description   : Sets the timer channel compare registers in count unit.
 * This function sets the timer channel two compare registers.
 * The values will be loaded after the current count overflows or underflows.
 *
 * Implements    : ETIMER_DRV_SetCompareThresholdBuffered_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetCompareThresholdBuffered(const uint16_t instance,
                                            const uint16_t channel,
                                            const uint16_t cmpld1,
                                            const uint16_t cmpld2)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set compare values using buffered registers */
    base->CH[channel].CMPLD1 = cmpld1;
    base->CH[channel].CMPLD2 = cmpld2;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetCompareThresholdBuffered
 * Description   : Reads the buffered compare values
 * This function gets the two compare registers of a timer channel
 *
 * Implements    : ETIMER_DRV_GetCompareThresholdBuffered_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetCompareThresholdBuffered(const uint16_t instance,
                                            const uint16_t channel,
                                            uint16_t * const cmpld1,
                                            uint16_t * const cmpld2)
{
    const ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set compare values using buffered registers */
    if(cmpld1 != NULL) { *cmpld1 = base->CH[channel].CMPLD1; }
    if(cmpld2 != NULL) { *cmpld2 = base->CH[channel].CMPLD2; }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_ReloadOnCompare
 * Description   : Set the value to which the counter shall be reset to.
 * This function sets the timer channel LOAD register.
 * The value will be loaded after underflow/overflow or after an compare event.
 * This function also disables or enables timer rollover.
 * When rollover is disabled, the counter will reset to loaded value upon
 * the first compare event.
 *
 * Implements    : ETIMER_DRV_ReloadOnCompare_Activity
 *END**************************************************************************/
extern void ETIMER_DRV_ReloadOnCompare(const uint16_t instance,
                                    const uint16_t channel,
                                    const uint16_t loadValue,
                                    const bool rolloverDisable)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set compare values */
    base->CH[channel].LOAD = loadValue;
    /* disable rollover (overflow or underflow)  */
    if(rolloverDisable == true )
    {
        base->CH[channel].CTRL1 |= ETIMER_CTRL1_LENGTH_MASK;
    }
    else
    {
        base->CH[channel].CTRL1 &= ~(ETIMER_CTRL1_LENGTH_MASK);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_OutputPinEnable
 * Description   : Enable the output from ETIMER channel to the external pin.
 * This function only allows the output from the ETIMER channel to be present
 * on the output pin selected without affecting any other functionality.
 * The user can route the output internally to another channel by configuring the
 * proper variables.
 *
 * Implements    : ETIMER_DRV_OutputPinEnable_Activity
 *END**************************************************************************/
void ETIMER_DRV_OutputPinEnable(const uint16_t instance,
                                const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* enable output */
    base->CH[channel].CTRL2 |=  ETIMER_CTRL2_OEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_OutputPinDisable
 * Description   : Disable the output from ETIMER channel to the external pin.
 * This function disables the output pin signal, which disables signal propagation
 * to the outside world through GPIO muxer without affecting any other functionality .
 * The external pin is configured as an input after calling this function.
 *
 * Implements    : ETIMER_DRV_OutputPinDisable_Activity
 *END**************************************************************************/
void ETIMER_DRV_OutputPinDisable(const uint16_t instance,
                                 const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* disable output */
    base->CH[channel].CTRL2 &=  ~(ETIMER_CTRL2_OEN_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetOutputFunction
 * Description   : Selects the way the output signal behaves.
 * There are several ways the output signal can behave, this will blindly set one of them.
 * Careful! This function bypasses driver mode checks !
 *
 * Implements    : ETIMER_DRV_SetOutputFunction_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetOutputFunction(const uint16_t instance,
                                    const uint16_t channel,
                                    const etimer_outmode_t outputMode)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set compare values */
    base->CH[channel].CTRL2 = ( ( base->CH[channel].CTRL2 & (~(ETIMER_CTRL2_OUTMODE_MASK | ETIMER_CTRL2_FORCE_MASK)) ) \
            | ETIMER_CTRL2_OUTMODE(outputMode) );
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_ForceOutputLogicLevel
 * Description   : Will set the output to 1 or 0 logic level.
 * Set the output signal to 1 or 0 logic level overwriting any other output setting.
 * Careful! This function set compareOutputControl to ETIMER_OUTMODE_SOFTWARE to maintain
 *          the force logic level of the output pin.
 *          To set previous compareOutputControl you must use ETIMER_DRV_SetOutputFunction
 * Careful! This function bypasses all sanity checks !
 *
 * Implements    : ETIMER_DRV_ForceOutputLogicLevel_Activity
 *END**************************************************************************/
void ETIMER_DRV_ForceOutputLogicLevel(const uint16_t instance,
                                    const uint16_t channel,
                                    const bool outputLogicLevel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

   base = s_etimerBase[instance];

   uint16_t polarity = ((base->CH[channel].CTRL2 & ETIMER_CTRL2_OPS_MASK) != 0u) ? (uint16_t)1u : (uint16_t)0u;
   uint16_t level;
   uint16_t ctrl2 = base->CH[channel].CTRL2;

   ctrl2 &= (~(ETIMER_CTRL2_OUTMODE_MASK | ETIMER_CTRL2_FORCE_MASK | ETIMER_CTRL2_VAL_MASK));

   if(polarity == (uint16_t)ETIMER_POLARITY_POSITIVE)
   {
       level = outputLogicLevel ? (uint16_t)1u : (uint16_t)0u;
   }
   else
   {
       level = outputLogicLevel ? (uint16_t)0u : (uint16_t)1u;
   }

   /* force logic */
   ctrl2 |= (ETIMER_CTRL2_OUTMODE(ETIMER_OUTMODE_SOFTWARE) | ETIMER_CTRL2_FORCE_MASK | ETIMER_CTRL2_VAL(level) );
   base->CH[channel].CTRL2  = ctrl2;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_IsTimerRunning
 * Description   : This function checks if the channel is running,
 * input clock is not gated and the internal counter is configured in a valid state.
 *
 * Implements    : ETIMER_DRV_IsTimerRunning_Activity
 *END**************************************************************************/
bool ETIMER_DRV_IsTimerRunning(const uint16_t instance,
                               const uint16_t channel)
{
    const ETIMER_Type * base;
    bool ret = false;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);
    DEV_ASSERT(channel < (1UL << ETIMER_CH_COUNT));

    base = s_etimerBase[instance];

    /* channel run enabled */
    if( (base->ENBL & (1UL << channel)) != 0U)
    {
        /* channel mode other than stop */
        if ( (((uint16_t)(base->CH[channel].CTRL1 & ETIMER_CTRL1_CNTMODE_MASK)) >> ETIMER_CTRL1_CNTMODE_SHIFT) != ((uint16_t) ETIMER_CNTMODE_NOP) )
        {
            /* this timer should be running, if it doesn't then the config is wrong */
            ret = true;
        }
    }
    /* default answer: nothing runs */
    return (ret);
}

#ifdef FEATURE_ETIMER_HAS_WATCHDOG
/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_SetWatchdog
 * Description   : Sets the watchdog counter in ticks.
 *
 * Implements    : ETIMER_DRV_SetWatchdog_Activity
 *END**************************************************************************/
void ETIMER_DRV_SetWatchdog(const uint16_t instance,
                            const uint16_t channel,
                            const uint32_t watchdog)
{
    /* ETIMER watchdog is only available on some peripherals */
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    /* set watchdog registers */
    base->WDTOL = (uint16_t) (watchdog & 0xffffU);
    base->WDTOH = (uint16_t) ((watchdog >> 16U) & 0xffffU);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_DisableWatchdog
 * Description   : Disable the watchdog counter for a channel
 *  This is a convenience function for the user which uses ETIMER_DRV_SetWatchdog
 *  with the watchdog parameter set to ETIMER_WATCHDOG_DISABLE_VALUE.
 *
 * Implements    : ETIMER_DRV_DisableWatchdog_Activity
 *END**************************************************************************/
void ETIMER_DRV_DisableWatchdog(const uint16_t instance,
                                const uint16_t channel)
{
    ETIMER_DRV_SetWatchdog(instance, channel, ETIMER_WATCHDOG_DISABLE_VALUE);
}

#endif /* FEATURE_ETIMER_HAS_WATCHDOG */

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetCaptureValue
 * Description   : Reads the values captured upon capture event.
 * The captureWordsToRead parameter will limit how many words will be read
 * from the HW FIFO buffer.
 * Captured words which are not read remain in the HW FIFO buffer.
 *
 * Implements    : ETIMER_DRV_GetCaptureValue_Activity
 *END**************************************************************************/
uint16_t ETIMER_DRV_GetCaptureValue(const uint16_t instance,
                                    const uint16_t channel,
                                    const uint16_t captureRegister,
                                    uint16_t * const captureResultBuffer,
                                    const uint8_t captureWordsToRead)
{
    const ETIMER_Type * base;
    uint8_t i=0;
    uint8_t cw=0;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);
    DEV_ASSERT(captureRegister < ETIMER_NUM_CAPTURE_REGISTERS);
    DEV_ASSERT(captureResultBuffer != NULL);
    DEV_ASSERT(captureWordsToRead <= ETIMER_NUM_CAPTURE_WORDS);
    DEV_ASSERT(captureWordsToRead != 0U);

    base = s_etimerBase[instance];
    /* what register */
    switch(captureRegister)
    {
    case 0:
        /* get captured words */
        cw = ((base->CH[channel].CTRL3 & ETIMER_CTRL3_C1FCNT_MASK) >> ETIMER_CTRL3_C1FCNT_SHIFT);
        /* anything to read */
        if(cw != 0U)
        {
            /* Get CAPT1 */
            for(i=0; (i<captureWordsToRead) && (i<cw); i++)
            {
                captureResultBuffer[i] = base->CH[channel].CAPT1;
            }
        }
        break;
    case 1:
        /* get captured words */
        cw = ((base->CH[channel].CTRL3 & ETIMER_CTRL3_C2FCNT_MASK) >> ETIMER_CTRL3_C2FCNT_SHIFT);
        /* anything to read */
        if(cw != 0U)
        {
            /* Get CAPT2 */
            for(i=0; (i<captureWordsToRead) && (i<cw); i++)
            {
                captureResultBuffer[i] = base->CH[channel].CAPT2;
            }
        }
        break;
    default:
        /* do nothing */
        break;
    }

    return (cw);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetCaptureConfigWords
 * Description   : Returns after how many captured words will the peripheral
 * signal that we have captured something.
 *
 * Implements    : ETIMER_DRV_GetCaptureConfigWords_Activity
 *END**************************************************************************/
uint16_t ETIMER_DRV_GetCaptureConfigWords(const uint16_t instance,
                                          const uint16_t channel)
{
    const ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    return ((base->CH[channel].CCCTRL & ETIMER_CCCTRL_CFWM_MASK) >> ETIMER_CCCTRL_CFWM_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetCaptureWords
 * Description   : Returns how many words are in the capture FIFO buffer
 *
 * Implements    : ETIMER_DRV_GetCaptureWords_Activity
 *END**************************************************************************/
uint16_t ETIMER_DRV_GetCaptureWords(const uint16_t instance,
                                    const uint16_t channel,
                                    const uint16_t captureRegister)
{
    const ETIMER_Type * base;
    uint16_t ret=0;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);
    DEV_ASSERT(captureRegister < ETIMER_NUM_CAPTURE_REGISTERS);

    base = s_etimerBase[instance];

    /* what register */
    switch(captureRegister)
    {
    case 0:
        /* get captured words */
        ret = ((base->CH[channel].CTRL3 & ETIMER_CTRL3_C1FCNT_MASK) >> ETIMER_CTRL3_C1FCNT_SHIFT);
        break;
    case 1:
        /* get captured words */
        ret = ((base->CH[channel].CTRL3 & ETIMER_CTRL3_C2FCNT_MASK) >> ETIMER_CTRL3_C2FCNT_SHIFT);
        break;
    default:
        /* do nothing */
        break;
    }
    return (ret);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_StartCapture
 * Description   : Start input capture.
 *
 * Implements    : ETIMER_DRV_StartCapture_Activity
 *END**************************************************************************/
void ETIMER_DRV_StartCapture(const uint16_t instance,
                             const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    base->CH[channel].CCCTRL |= ETIMER_CCCTRL_ARM_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_StopCapture
 * Description   : Stop any ongoing input capture. Words already captured
 * remain in the HW FIFO. If there is no ongoing capture, calling this function
 * will have no effect.
 *
 * Implements    : ETIMER_DRV_StopCapture_Activity
 *END**************************************************************************/
void ETIMER_DRV_StopCapture(const uint16_t instance,
                            const uint16_t channel)
{
    ETIMER_Type * base;

    DEV_ASSERT(instance < ETIMER_INSTANCE_COUNT);
    DEV_ASSERT(channel < ETIMER_CH_COUNT);

    base = s_etimerBase[instance];

    base->CH[channel].CCCTRL &= ~ETIMER_CCCTRL_ARM_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigOneshot
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigOneshot_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigOneshot(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);
    /* Fixed: These are fixed values, should not be changed */
    config->countMode = ETIMER_CNTMODE_SECONDARY_AS_TRIGGER;
    config->compareOutputControl = ETIMER_OUTMODE_COMP1_SET_SECIN_CLEAR;
    config->countDirection = ETIMER_COUNT_UP;
    config->countLength = true;

    /* Application specific: These values are application specific and should be updated after calling this function */
    config->compareValues[0] = 0;
    config->compareLoading[0] = ETIMER_CLC_FROM_CMPLD1_WHEN_COMP1;
    config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_1;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;

    /* Optional: These values are optional and can be changed by the application */
    config->interruptEnableMask = 0;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;

    /* Unused: These values are not relevant for the current mode and should not be changed */
    config->countOnce = false;
    config->oneshotCapture = false;
    config->captureWords = 0;
    config->compareValues[1] = 0;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
    config->compareLoading[1] = ETIMER_CLC_NEVER;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigPulseOutput
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigPulseOutput_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigPulseOutput(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

   /* Fixed: These are fixed values, should not be changed */
   config->countMode = ETIMER_CNTMODE_PRIMARY;
   config->compareOutputControl = ETIMER_OUTMODE_COUNT_CLK;
   config->countOnce = true;
   config->countDirection = ETIMER_COUNT_UP;
   config->countLength = false;

   /* Application specific: These values are application specific and should be updated after calling this function */
   config->compareValues[0] = 0;
   config->compareLoading[0] = ETIMER_CLC_NEVER;
   config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_2;
   config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;

   /* Optional: These values are optional and can be changed by the application */
   config->interruptEnableMask = 0;
   config->inputFilter.samples = ETIMER_FILT_CNT_3;
   config->inputFilter.rate = 0;
   config->outputPin.enable = false;
   config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;

   /* Unused: These values are not relevant for the current mode and should not be changed */
   config->oneshotCapture = false;
   config->captureWords = 0;
   config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
   config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
   config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
   config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;
   config->compareLoading[1] = ETIMER_CLC_NEVER;
   config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP;
   config->compareValues[1] = 0;
   config->redundant = false;
   config->masterMode = false;
   config->coChannelForce = false;
   config->coChannelVal = false;
   config->coChannelInit = ETIMER_COINIT_NONE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigFixedFreqPwm
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigFixedFreqPwm_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigFixedFreqPwm(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    /* Fixed: These are fixed values, should not be changed */
    config->countMode = ETIMER_CNTMODE_PRIMARY;
    config->countLength = false;
    config->countOnce = false;
    config->compareOutputControl = ETIMER_OUTMODE_COMP_SET_OVF_CLEAR;
    config->countDirection = ETIMER_COUNT_UP;

    /* Application specific: These values are application specific and should be updated after calling this function */
    config->compareValues[0] = 0;
    config->compareLoading[0] = ETIMER_CLC_NEVER;
    config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_1;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;

    /* Optional: These values are optional and can be changed by the application */
    config->interruptEnableMask = 0;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;

    /* Unused: These values are not relevant for the current mode and should not be changed */
    config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->compareValues[1] = 0;
    config->compareLoading[1] = ETIMER_CLC_NEVER;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
    config->captureWords = 0;
    config->oneshotCapture = false;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigVariableFreqPwm
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigVariableFreqPwm_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigVariableFreqPwm(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    /* Fixed: These are fixed values, should not be changed */
    config->countMode = ETIMER_CNTMODE_PRIMARY;
    config->countLength = true;
    config->countOnce = false;
    config->compareOutputControl = ETIMER_OUTMODE_COMPARE_ALT_TOGGLE;
    config->countDirection = ETIMER_COUNT_UP;

    /* Application specific: These values are application specific and should be updated after calling this function */
    config->compareValues[0] = 0;
    config->compareValues[1] = 0;
    config->compareLoading[0] = ETIMER_CLC_NEVER;
    config->compareLoading[1] = ETIMER_CLC_NEVER;
    config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_1;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;

    /* Optional: These values are optional and can be changed by the application */
    config->interruptEnableMask = 0;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP;

    /* Unused: These values are not relevant for the current mode and should not be changed */
    config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
    config->captureWords = 0;
    config->oneshotCapture = false;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigModuleCountingDirection
 * Description   : This function gets the default configuration structure with default settings
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigModuleCountingDirection_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigModuleCountingDirection(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    /* Fixed: These are fixed values, should not be changed */
    config->countMode = ETIMER_CNTMODE_SECONDARY_AS_DIRECTION;
    config->countLength = false;
    config->countOnce = false;
    config->compareLoading[0] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP2;
    config->compareLoading[1] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP1;

    /* Application specific: These values are application specific and should be updated after calling this function */
    config->compareValues[0] = 0;
    config->compareValues[1] = 0;
    config->primaryInput.source = ETIMER_IN_SRC_CLK_DIV_1;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_DOWN;
    config->secondaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;

    /* Optional: These values are optional and can be changed by the application */
    config->interruptEnableMask = 0;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;

    /* Unused: These values are not relevant for the current mode and should not be changed */
    config->compareOutputControl = ETIMER_OUTMODE_SOFTWARE;
    config->countDirection = ETIMER_COUNT_UP;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
    config->captureWords = 0;
    config->oneshotCapture = false;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ETIMER_DRV_GetDefaultChannelConfigModuleCountingQuadrature
 * Description   : This function gets the default configuration structure with default settings
 *
 *
 * Implements    : ETIMER_DRV_GetDefaultChannelConfigModuleCountingQuadrature_Activity
 *END**************************************************************************/
void ETIMER_DRV_GetDefaultChannelConfigModuleCountingQuadrature(etimer_user_channel_config_t *config)
{
    /* Checks input parameter is not NULL */
    DEV_ASSERT(config != NULL);

    /* Fixed: These are fixed values, should not be changed */
    config->countMode = ETIMER_CNTMODE_QUADRATURE;
    config->countLength = false;
    config->countOnce = false;
    config->compareLoading[0] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP2;
    config->compareLoading[1] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP1;
    config->countDirection = ETIMER_COUNT_UP;

    /* Application specific: These values are application specific and should be updated after calling this function */
    config->compareValues[0] = 0;
    config->compareValues[1] = 0;
    config->primaryInput.source = ETIMER_IN_SRC_CNT0_IN;
    config->primaryInput.polarity = ETIMER_POLARITY_POSITIVE;
    config->compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_DOWN;
    config->secondaryInput.source = ETIMER_IN_SRC_CNT1_IN;
    config->secondaryInput.polarity = ETIMER_POLARITY_POSITIVE;

    /* Optional: These values are optional and can be changed by the application */
    config->interruptEnableMask = 0;
    config->inputFilter.samples = ETIMER_FILT_CNT_3;
    config->inputFilter.rate = 0;
    config->outputPin.enable = false;
    config->outputPin.polarity = ETIMER_POLARITY_POSITIVE;

    /* Unused: These values are not relevant for the current mode and should not be changed */
    config->compareOutputControl = ETIMER_OUTMODE_SOFTWARE;
    config->captureControl[0] = ETIMER_CPTMODE_DISABLED;
    config->captureControl[1] = ETIMER_CPTMODE_DISABLED;
    config->captureWords = 0;
    config->oneshotCapture = false;
    config->redundant = false;
    config->masterMode = false;
    config->coChannelForce = false;
    config->coChannelVal = false;
    config->coChannelInit = ETIMER_COINIT_NONE;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
