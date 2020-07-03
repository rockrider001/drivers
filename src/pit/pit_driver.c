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
 * @file pit_driver.c
 *
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7, Composite expression with smaller
 * essential type than other operand.
 * This is required to avoid short time overflow in current lifetime calculation.
 */

#include <stddef.h>
#include "pit_driver.h"
#include "pit_hw_access.h"
#include "interrupt_manager.h"
#include "clock_manager.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for PIT instances */
static PIT_Type * const s_pitBase[] = PIT_BASE_PTRS;
/* Table to save PIT standard timer indexes in clock map for clock configuration */
static const clock_names_t s_pitClkNames[PIT_INSTANCE_COUNT] = PIT_CLOCK_NAMES;
/* PIT standard timer functional clock variable which will be updated in some driver functions */
static uint32_t s_pitSourceClock[PIT_INSTANCE_COUNT] = {0U};

#if FEATURE_PIT_HAS_RTI_CHANNEL
/* Table to save RTI timer channel indexes in clock map for clock configuration */
static const clock_names_t s_rtiClkNames[PIT_INSTANCE_COUNT] = RTI_CLOCK_NAMES;
/* RTI timer functional clock variable which will be updated in some driver functions */
static uint32_t s_rtiSourceClock[PIT_INSTANCE_COUNT] = {0U};
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */


/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetDefaultConfig
 * Description   : This function gets default PIT module configuration structure.
 *
 * Implements    : PIT_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void PIT_DRV_GetDefaultConfig(pit_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->enableStandardTimers = true;
#if FEATURE_PIT_HAS_RTI_CHANNEL
    config->enableRTITimer = false;
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    config->stopRunInDebug = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetDefaultChanConfig
 * Description   : This function gets default timer channel configuration structure.
 *
 * Implements    : PIT_DRV_GetDefaultChanConfig_Activity
 *END**************************************************************************/
void PIT_DRV_GetDefaultChanConfig(pit_channel_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->hwChannel = 0U;
    config->periodUnit = PIT_PERIOD_UNITS_COUNTS;
    config->period = 0U;
    config->enableChain = false;
    config->enableInterrupt = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_Init
 * Description   : Initializes PIT module.
 * This function enables the standard timer and real time interrupt timer,
 * configures PIT module operation in Debug mode. The PIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other PIT driver function
 *
 * Implements    : PIT_DRV_Init_Activity
 *END**************************************************************************/
void PIT_DRV_Init(const uint32_t instance,
                  const pit_config_t * const config)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    PIT_Type * base = s_pitBase[instance];
    status_t clkErr;
    /* Gets current functional clock frequency of PIT standard timer */
    clkErr = CLOCK_SYS_GetFreq(s_pitClkNames[instance], &s_pitSourceClock[instance]);
    /* Checks the functional clock of PIT standard timer */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_pitSourceClock[instance] > 0U);

    /* Enables functional clock of standard timer */
    if (config->enableStandardTimers)
    {
        PIT_EnableTimer(base, 0U);
    }

#if FEATURE_PIT_HAS_RTI_CHANNEL
    /* Enables functional clock of RTI timer */
    if (config->enableRTITimer)
    {
        PIT_EnableTimer(base, 1U);
    }
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    /* Sets PIT operation in Debug mode*/
    PIT_SetTimerStopRunInDebugCmd(base, config->stopRunInDebug);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_Deinit
 * Description   : De-initializes PIT module.
 * This function disables PIT timer and set all register to default value.
 * In order to use the PIT module again, PIT_DRV_Init must be called.
 *
 * Implements    : PIT_DRV_Deinit_Activity
 *END**************************************************************************/
void PIT_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    PIT_Type * base = s_pitBase[instance];
    uint8_t channelNum = PIT_TIMER_COUNT;

#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
    switch (instance)
    {
#ifdef PIT_PECULIAR_INSTANCE_1
        case PIT_PECULIAR_INSTANCE_1:
             channelNum = PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1;
             break;
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
        case PIT_PECULIAR_INSTANCE_2:
             channelNum = PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2;
             break;
#endif
        default:
             channelNum = PIT_TIMER_COUNT;
             break;
    }
#endif
    /* Set control, load, status registers to default value */
    PIT_Reset(base, channelNum);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_InitChannel
 * Description   : Initializes PIT channel.
 * This function initializes the PIT timers by using a channel. Pass in the timer channel
 * configuration structure. Timer channels do not start counting by default after calling this
 * function. The function PIT_DRV_StartChannel must be called to start the timer channel counting.
 * Call the PIT_DRV_SetTimerPeriodByUs to re-set the period.
 *
 * Implements    : PIT_DRV_InitChannel_Activity
 *END**************************************************************************/
status_t PIT_DRV_InitChannel(const uint32_t instance,
                             const pit_channel_config_t * const chnlConfig)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(chnlConfig != NULL);
    DEV_ASSERT(!((chnlConfig->hwChannel == 0U) && (chnlConfig->enableChain)));
    DEV_ASSERT(chnlConfig->hwChannel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((chnlConfig->hwChannel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((chnlConfig->hwChannel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * base = s_pitBase[instance];
    const IRQn_Type pitIrqId[PIT_INSTANCE_COUNT][PIT_CHANNEL_COUNT] = PIT_IRQS;
    status_t retVal = STATUS_SUCCESS;
    /* Setups the timer channel chaining  */
    if (chnlConfig->hwChannel != PIT_RTICHANNEL_INDEX)
    {
        PIT_SetTimerChainCmd(base, chnlConfig->hwChannel, chnlConfig->enableChain);
    }

    if (chnlConfig->periodUnit == PIT_PERIOD_UNITS_MICROSECONDS)
    {
        /* Setups timer channel period in microsecond unit */
        retVal = PIT_DRV_SetTimerPeriodByUs(instance, chnlConfig->hwChannel, chnlConfig->period);
    }
    else
    {
        /* Setups timer channel period in count unit */
        PIT_DRV_SetTimerPeriodByCount(instance, chnlConfig->hwChannel, chnlConfig->period);
    }

    if (retVal == STATUS_SUCCESS)
    {
        /* Setups interrupt generation for timer channel */
        if (chnlConfig->enableInterrupt)
        {
            /* Enables interrupt generation */
            PIT_DRV_EnableChannelInterrupt(instance, chnlConfig->hwChannel);
            INT_SYS_EnableIRQ(pitIrqId[instance][chnlConfig->hwChannel]);
        }
        else
        {
            /* Disables interrupt generation */
            PIT_DRV_DisableChannelInterrupt(instance, chnlConfig->hwChannel);
            INT_SYS_DisableIRQ(pitIrqId[instance][chnlConfig->hwChannel]);
        }
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_ConfigChannel
 * Description   : This function sets the timer channel period in microseconds or
 * count base on period unit argument.
 *
 * Implements    : PIT_DRV_ConfigChannel_Activity
 *END**************************************************************************/
status_t PIT_DRV_ConfigChannel(const uint32_t instance,
                               const uint8_t channel,
                               const uint32_t period,
                               const pit_period_units_t periodUnit)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    status_t retVal = STATUS_SUCCESS;
    if (periodUnit == PIT_PERIOD_UNITS_MICROSECONDS)
    {
        /* Setups timer channel period in microsecond unit */
        retVal = PIT_DRV_SetTimerPeriodByUs(instance, channel, period);
    }
    else
    {
        /* Setups timer channel period in count unit */
        PIT_DRV_SetTimerPeriodByCount(instance, channel, period);
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_StartChannel
 * Description   : This function starts timer channel counting.
 * After calling this function, timer channel loads period value, count down to 0 and
 * then load the respective start value again. Each time a timer channel reaches 0,
 * it generates a trigger pulse and sets the timeout interrupt flag.
 *
 * Implements    : PIT_DRV_StartChannel_Activity
 *END**************************************************************************/
void PIT_DRV_StartChannel(const uint32_t instance,
                          const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        /* check if the timer is already running. If it is, we cannot use it. */
        DEV_ASSERT((base->RTI_TCTRL & PIT_RTI_TCTRL_TEN_MASK) == 0u);
        base->RTI_TCTRL |= PIT_RTI_TCTRL_TEN_MASK;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        /* check if the timer is already running. If it is, we cannot use it. */
        DEV_ASSERT((base->TIMER[channel].TCTRL & PIT_TCTRL_TEN_MASK) == 0u);
        base->TIMER[channel].TCTRL |= PIT_TCTRL_TEN_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_StopChannel
 * Description   : This function stops timer channel counting.
 * Timer channels reload their periods respectively after the next time they call
 * the PIT_DRV_StartChannel.
 *
 * Implements    : PIT_DRV_StopChannel_Activity
 *END**************************************************************************/
void PIT_DRV_StopChannel(const uint32_t instance,
                         const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        base->RTI_TCTRL &= ~PIT_RTI_TCTRL_TEN_MASK;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        base->TIMER[channel].TCTRL &= ~PIT_TCTRL_TEN_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_SetTimerPeriodByUs
 * Description   : Sets timer channel period in microseconds unit.
 * This function sets the timer channel period in microseconds.
 * The period range depends on the frequency of the PIT source clock. If the required period
 * is out of range, use the lifetime timer if applicable.
 * This function is only valid for one single channel. If channels are chained together,
 * the period here makes no sense.
 *
 * Implements    : PIT_DRV_SetTimerPeriodByUs_Activity
 *END**************************************************************************/
status_t PIT_DRV_SetTimerPeriodByUs(const uint32_t instance,
                                    const uint8_t channel,
                                    const uint32_t periodUs)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    status_t clkErr;
    status_t retVal = STATUS_SUCCESS;
    uint64_t count;
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        /* Gets current functional clock frequency of RTI timer */
        clkErr = CLOCK_SYS_GetFreq(s_rtiClkNames[instance], &s_rtiSourceClock[instance]);
        /* Checks the functional clock of RTI timer */
        (void)clkErr;
        DEV_ASSERT(clkErr == STATUS_SUCCESS);
        DEV_ASSERT(s_rtiSourceClock[instance] > 0U);
        /* Convert the microsecond value to count value */
        count = ((uint64_t)periodUs) * s_rtiSourceClock[instance];
        count = (count / 1000000U) - 1U;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        /* Gets current functional clock frequency of PIT standard timer */
        clkErr = CLOCK_SYS_GetFreq(s_pitClkNames[instance], &s_pitSourceClock[instance]);
        /* Checks the functional clock of PIT standard timer */
        (void)clkErr;
        DEV_ASSERT(clkErr == STATUS_SUCCESS);
        DEV_ASSERT(s_pitSourceClock[instance] > 0U);
        /* Convert the microsecond value to count value */
        count = ((uint64_t)periodUs) * s_pitSourceClock[instance];
        count = (count / 1000000U) - 1U;
    }
    /* Checks whether the count is valid */
    if (count <= PIT_LDVAL_TSV_MASK)
    {
        /* Sets the timer channel period in count unit */
        PIT_DRV_SetTimerPeriodByCount(instance, channel, (uint32_t)count);
    }
    else
    {
        retVal = STATUS_ERROR;
    }
    return retVal;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetCurrentTimerUs
 * Description   : Gets current timer channel counting value in microseconds unit.
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time. Make sure the
 * running time does not exceed the timer period. The time stamp returned is
 * down-counting.
 *
 * Implements    : PIT_DRV_GetCurrentTimerUs_Activity
 *END**************************************************************************/
uint64_t PIT_DRV_GetCurrentTimerUs(const uint32_t instance,
                                   const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    status_t clkErr;
    uint64_t currentTime;
    /* Gets current timer channel counting value */
    currentTime = PIT_DRV_GetCurrentTimerCount(instance, channel);
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        /* Gets current functional clock frequency of RTI timer */
        clkErr = CLOCK_SYS_GetFreq(s_rtiClkNames[instance], &s_rtiSourceClock[instance]);
        (void)clkErr;
        DEV_ASSERT(s_rtiSourceClock[instance] > 0U);
        /* Converts counting value to microseconds unit */
        currentTime = (currentTime * 1000000U) / s_rtiSourceClock[instance];
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        /* Gets current functional clock frequency of PIT standard timer */
        clkErr = CLOCK_SYS_GetFreq(s_pitClkNames[instance], &s_pitSourceClock[instance]);
        (void)clkErr;
        DEV_ASSERT(s_pitSourceClock[instance] > 0U);
        /* Converts counting value to microseconds unit */
        currentTime = (currentTime * 1000000U) / s_pitSourceClock[instance];
    }
    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_SetTimerPeriodByCount
 * Description   : This function sets the timer channel period in count unit.
 * Timer channel begin counting from the value set by this function.
 * The counter period of a running timer channel can be modified by first stopping
 * the timer channel, setting a new load value, and starting the timer channel again. If
 * channel are not restarted, the new value is loaded after the next trigger
 * event. Note that The RTI channel must not be set to a value lower than 32 cycles,
 * otherwise interrupts may be lost, as it takes several cycles to clear the RTI interrupt
 *
 * Implements    : PIT_DRV_SetTimerPeriodByCount_Activity
 *END**************************************************************************/
void PIT_DRV_SetTimerPeriodByCount(const uint32_t instance,
                                   const uint8_t channel,
                                   const uint32_t count)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        base->RTI_LDVAL = count;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        base->TIMER[channel].LDVAL = count;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetCurrentTimerCount
 * Description   : This function returns the real-time timer channel counting value,
 * the value in a range from 0 to timer channel period
 *
 * Implements    : PIT_DRV_GetCurrentTimerCount_Activity
 *END**************************************************************************/
uint32_t PIT_DRV_GetCurrentTimerCount(const uint32_t instance,
                                      const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    const PIT_Type * base = s_pitBase[instance];
    uint32_t currentCount;
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        currentCount = base->RTI_CVAL;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        currentCount = base->TIMER[channel].CVAL;
    }
    return currentCount;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_SetLifetimeTimerCount
 * Description   : Build the 64-bit lifetimer.
 * The lifetime timer is a 64-bit timer which chains timer channel 0 and timer channel 1 together
 * with the start value of both channels is set to the maximum value(0xFFFFFFFF).
 * The period of lifetime timer is equal to the "period of
 * timer 0 * period of timer 1".
 *
 * Implements : PIT_DRV_SetLifetimeTimerCount_Activity
 *END**************************************************************************/
void PIT_DRV_SetLifetimeTimerCount(const uint32_t instance)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);

    PIT_Type * const base = s_pitBase[instance];

    /* Setup timer channel 1 for maximum counting period */
    base->TIMER[1U].LDVAL = PIT_LDVAL_TSV_MASK;
    /* Disable timer channel 1 interrupt */
    base->TIMER[1U].TCTRL &= ~PIT_TCTRL_TIE_MASK;
    /* Chain timer channel 1 to timer channel 0 */
    base->TIMER[1U].TCTRL |= PIT_TCTRL_CHN_MASK;
    /* Start timer channel 1 */
    base->TIMER[1U].TCTRL |= PIT_TCTRL_TEN_MASK;
    /* Setup timer channel 0 for maximum counting period */
    base->TIMER[0U].LDVAL = PIT_LDVAL_TSV_MASK;
    /* Start timer channel 0 */
    base->TIMER[0U].TCTRL = PIT_TCTRL_TEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetLifetimeTimerCount
 * Description   : Read current lifefime counter value.
 * The Lifetime timer is 64-bit timer which chains timer 0 and timer 1 together.
 * The period of lifetime timer equals to "period of timer 0 * period of timer 1".
 * This feature returns an absolute time stamp in count. The time stamp
 * value does not exceed the timer period. The timer is up-counting.
 * Calling PIT_DRV_SetLifetimeTimerCount to use this timer.
 *
 * Implements : PIT_DRV_GetLifetimeTimerCount_Activity
 *END**************************************************************************/
uint64_t PIT_DRV_GetLifetimeTimerCount(const uint32_t instance)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);

    const PIT_Type * const base = s_pitBase[instance];
    uint32_t valueH;
    uint32_t valueL;
    uint64_t lifeTimeValue = 0U;

#if PIT_INSTANCE_HAS_NOT_LIFETIME_TIMER
    if (base != PIT_INSTANCE_BASE_HAS_NOT_LIFETIME_TIMER)
#endif
    {
        /* LTMR64H should be read before LTMR64L */
        valueH = base->LTMR64H;
        valueL = base->LTMR64L;
        lifeTimeValue = (~(((uint64_t)valueH << 32U) + (uint64_t)(valueL)));
    }
    return lifeTimeValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetLifetimeTimerUs
 * Description   : Read current lifetime value in microseconds unit.
 * This feature returns an absolute time stamp in microseconds. The time stamp
 * value does not exceed the timer period. The timer is up-counting.
 *
 * Implements : PIT_DRV_GetLifetimeTimerUs_Activity
 *END**************************************************************************/
uint64_t PIT_DRV_GetLifetimeTimerUs(const uint32_t instance)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);

    status_t clkErr;
    uint64_t currentTime;

    /* Get current lifetime timer count, and reverse it to up-counting.*/
    currentTime = PIT_DRV_GetLifetimeTimerCount(instance);
    /* Gets current functional clock frequency of PIT standard timer */
    clkErr = CLOCK_SYS_GetFreq(s_pitClkNames[instance], &s_pitSourceClock[instance]);
    (void)clkErr;
    DEV_ASSERT(s_pitSourceClock[instance] > 0U);
    /* Convert count numbers to microseconds unit.*/
    /* Note: using currentTime * 1000 rather than 1000000 to avoid short time overflow. */
    currentTime = (currentTime * 1000U) / (s_pitSourceClock[instance] / 1000U);
    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_EnableChannelInterrupt
 * Description   : This function allows enabling interrupt generation of timer channel
 * when timeout occurs.
 *
 * Implements : PIT_DRV_EnableChannelInterrupt_Activity
 *END**************************************************************************/
void PIT_DRV_EnableChannelInterrupt(const uint32_t instance,
                                        const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        base->RTI_TCTRL |= PIT_RTI_TCTRL_TIE_MASK;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        base->TIMER[channel].TCTRL |= PIT_TCTRL_TIE_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_DisableChannelInterrupt
 * Description   : This function allows disabling interrupt generation of timer channel
 * when timeout occurs.
 *
 * Implements : PIT_DRV_DisableChannelInterrupt_Activity
 *END**************************************************************************/
void PIT_DRV_DisableChannelInterrupt(const uint32_t instance,
                                     const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        base->RTI_TCTRL &= ~PIT_RTI_TCTRL_TIE_MASK;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        base->TIMER[channel].TCTRL &= ~PIT_TCTRL_TIE_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_GetCurrentTimerUs
 * Description   : This function gets the current interrupt flag of timer channels.
 * Every time the timer channel counts to 0, this flag is set.
 *
 * Implements : PIT_DRV_GetStatusFlags_Activity
 *END**************************************************************************/
uint32_t PIT_DRV_GetStatusFlags(const uint32_t instance,
                                const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    const PIT_Type * const base = s_pitBase[instance];
    uint32_t retVal = 0U;
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        retVal = (base->RTI_TFLG & PIT_RTI_TFLG_TIF_MASK);
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        retVal = (base->TIMER[channel].TFLG & PIT_TFLG_TIF_MASK);
    }
    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_DRV_ClearStatusFlags
 * Description   : This function clears the timer interrupt flag after a timeout event
 * occurs.
 *
 * Implements : PIT_DRV_ClearStatusFlags_Activity
 *END**************************************************************************/
void PIT_DRV_ClearStatusFlags(const uint32_t instance,
                              const uint8_t channel)
{
    DEV_ASSERT(instance < PIT_INSTANCE_COUNT);
    DEV_ASSERT(channel < PIT_CHANNEL_COUNT);
#if FEATURE_PIT_HAS_PECULIAR_INSTANCE
#ifdef PIT_PECULIAR_INSTANCE_1
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_1) && (instance == PIT_PECULIAR_INSTANCE_1)));
#endif
#ifdef PIT_PECULIAR_INSTANCE_2
    DEV_ASSERT(!((channel >= PIT_CHAN_NUM_OF_PECULIAR_INSTANCE_2) && (instance == PIT_PECULIAR_INSTANCE_2)));
#endif
#endif

    PIT_Type * const base = s_pitBase[instance];
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (channel == PIT_RTICHANNEL_INDEX)
    {
        base->RTI_TFLG = PIT_RTI_TFLG_TIF_MASK;
    }
    else
#endif /* FEATURE_PIT_HAS_RTI_CHANNEL */
    {
        base->TIMER[channel].TFLG = PIT_TFLG_TIF_MASK;
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
