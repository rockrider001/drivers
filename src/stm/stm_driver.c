/*
 * Copyright 2017 NXP.
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
 * @file stm_driver.c
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
 * Violates MISRA 2012 Advisory Rule 8.9, An object should be defined at block
 * scope if its identifier only appears in a single function.
 * An object with static storage duration declared at block scope cannot be
 * accessed directly from outside the block.
 */

#include <stddef.h>
#include "stm_driver.h"
#include "clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The maximum value of compare register */
#define STM_COMPARE_MAX (0xFFFFFFFFU)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for STM instances. */
static STM_Type * const s_stmBase[STM_INSTANCE_COUNT] = STM_BASE_PTRS;
/*! @brief STM functional clock variable which will be updated in some driver functions */
static uint32_t s_stmClockSrcFreq[STM_INSTANCE_COUNT] = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_Init
 * Description   : Initializes the STM module.
 * This function initializes STM module base on the members of the stm_config_t structure
 * with the desired values. Including clock source for module, prescaler, allow counter to
 * be stopped in debug mode and start-value for common counter register.
 *
 * Implements    : STM_DRV_Init_Activity
 *END**************************************************************************/
void STM_DRV_Init(const uint32_t instance,
                  const stm_config_t * const config)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    STM_Type * const base = s_stmBase[instance];

    uint32_t cr;
    /* Configure clock source selection, prescaler, runs in stop mode */
    cr = STM_CR_CPS(config->clockPrescaler) |
#if FEATURE_STM_HAS_CLOCK_SELECTION
         STM_CR_CSL(config->clockSource) |
#endif /* FEATURE_STM_HAS_CLOCK_SELECTION */
         STM_CR_FRZ(config->stopInDebugMode ? 1UL : 0UL);
    base->CR = cr;
    /* Set start-value for counter register */
    base->CNT = config->startValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_Deinit
 * Description   : De-Initializes the STM module.
 * This function resets all control registers and registers of each channel to default values
 * (Reference Manual Resets).
 * This function should only be called if user wants to stop all channels (not only one channel).
 * System clock is always enabled for STM module, and doesn't have any option to disable clock.
 *
 * Implements    : STM_DRV_Deinit_Activity
 *END**************************************************************************/
void STM_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    uint8_t i;
    STM_Type * const base = s_stmBase[instance];

    /* Disable counter and reset counter registers */
    base->CR = 0x0U;
    base->CNT = 0x0U;
    /* Reset all channels to default */
    for (i = 0; i < STM_CHANNEL_COUNT; i++)
    {
        base->CHANNEL[i].CCR = 0x0U;
        base->CHANNEL[i].CIR = 0x1U;
        base->CHANNEL[i].CMP = 0x0U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_GetDefaultConfig
 * Description   : Gets the default configuration structure of STM with default settings.
 * This function initializes the hardware configuration structure to default values
 * (Reference Manual Resets).
 * This function should be called before configuring the hardware feature by STM_DRV_Init()
 * function, otherwise all members be written by user.
 * This function insures that all members are written with safe values, but the user still can
 * modify the desired members.
 *
 * Implements    : STM_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void STM_DRV_GetDefaultConfig(stm_config_t * const config)
{
    DEV_ASSERT(config != NULL);
#if FEATURE_STM_HAS_CLOCK_SELECTION
    /* Select clock source */
    config->clockSource = STM_CLOCK_SYSTEM;
#endif /* FEATURE_STM_HAS_CLOCK_SELECTION */
    /* Divide STM clock by 1 */
    config->clockPrescaler = 0U;
    /* Counter continues to run in debug mode */
    config->stopInDebugMode = false;
    /* Value start for common counter register */
    config->startValue = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_InitChannel
 * Description   : Initializes the STM channel module with a structure.
 * This function initializes STM channel module base on the members of the stm_channel_config_t
 * structure for each channel with the desired values. Including channel selected and compare-value
 * for that channel. This function is useful when using PEx tool.
 *
 * Implements    : STM_DRV_InitChannel_Activity
 *END**************************************************************************/
void STM_DRV_InitChannel(const uint32_t instance,
                         const stm_channel_config_t * const config)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(config->channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];

    /* Compare value for channel selected */
    base->CHANNEL[config->channel].CMP = config->compareValue;
    /* Enable channel */
    base->CHANNEL[config->channel].CCR = STM_CCR_CEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_ConfigChannel
 * Description   : Configures the STM channel module with parameters.
 * This function initializes the desired settings for each channel.
 * This function is the same STM_DRV_InitChannel() function about feature. But it is required
 * for user to have more options when configure the channel.
 *
 * Implements    : STM_DRV_ConfigChannel_Activity
 *END**************************************************************************/
void STM_DRV_ConfigChannel(const uint32_t instance,
                           const uint8_t channel,
                           const uint32_t compareValue)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Compare value for channel selected */
    base->CHANNEL[channel].CMP = compareValue;
    /* Enable channel */
    base->CHANNEL[channel].CCR = STM_CCR_CEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_EnableChannel
 * Description   : Enables the channel selected.
 * This function enables channel selected. The feature in this function is contained
 * in STM_DRV_InitChannel() also, so after calling that function then no need to call this
 * function for the first time. It is called when a channel is disable momentarily and
 *  user wants to enable channel again.
 *
 * Implements    : STM_DRV_EnableChannel_Activity
 *END**************************************************************************/
void STM_DRV_EnableChannel(const uint32_t instance,
                           const uint8_t channel)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Enable channel */
    base->CHANNEL[channel].CCR |= STM_CCR_CEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_DisableChannel
 * Description   : Disables the channel selected.
 * This function disables channel selected. There is no channel interrupt request is generated
 * after calling this function.
 *
 * Implements    : STM_DRV_DisableChannel_Activity
 *END**************************************************************************/
void STM_DRV_DisableChannel(const uint32_t instance,
                            const uint8_t channel)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Disable channel */
    base->CHANNEL[channel].CCR &= ~STM_CCR_CEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_SetStartValueCount
 * Description   : Sets start-value for Counter register.
 * This function sets start-value for common Counter register. There is only one counter
 * for all channels and the feature in this function is contained in STM_DRV_Init() also,
 * after calling that function then no need to call this function for the first time.
 * It is called when user wants to set a new start-value to run again instead of calling
 * STM_DRV_Init(), the action calls STM_DRV_Init() will reduce performance of module.
 *
 * Implements    : STM_DRV_SetStartValueCount_Activity
 *END**************************************************************************/
void STM_DRV_SetStartValueCount(const uint32_t instance,
                                const uint32_t startValue)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Set start-value for counter register */
    base->CNT = startValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_StartTimer
 * Description   : Starts timer counter.
 * This function enables common Timer Counter and starts running.
 *
 * Implements    : STM_DRV_StartTimer_Activity
 *END**************************************************************************/
void STM_DRV_StartTimer(const uint32_t instance)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Timer counter is started */
    base->CR |= STM_CR_TEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_StopTimer
 * Description   : Stops timer counter.
 * This function disables common Timer Counter and stop counting.
 *
 * Implements    : STM_DRV_StopTimer_Activity
 *END**************************************************************************/
void STM_DRV_StopTimer(const uint32_t instance)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Timer counter is stopped */
    base->CR &= ~STM_CR_TEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_ComputeTicksByUs
 * Description   : Computes the number of ticks from microseconds.
 * This function computes the number of ticks from microseconds.
 * The number of ticks depends on the frequency and counter prescaler of the STM source clock.
 * User has to configure the frequency and counter prescaler suitable by themself before calling
 * this function to have valid the number of ticks.
 *
 * Implements    : STM_DRV_ComputeTicksByUs_Activity
 *END**************************************************************************/
status_t STM_DRV_ComputeTicksByUs(const uint32_t instance,
                                  const uint32_t periodUs,
                                  uint32_t * const ticks)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    const STM_Type * const base = s_stmBase[instance];
    status_t retStatus = STATUS_SUCCESS;
    status_t clkErr;
    uint32_t clkPrescaler;
    uint64_t tempTicks;
    uint32_t clkSelect;

#if FEATURE_STM_HAS_CLOCK_SELECTION
    const clock_names_t s_stmClkNames[] = STM_CLOCK_NAMES;
    /* Gets current clock selection */
    clkSelect = (base->CR & STM_CR_CSL_MASK) >> STM_CR_CSL_SHIFT;
#else
    const clock_names_t s_stmClkNames[] = STM_CLOCK_NAMES;
    clkSelect = 0x0U;
#endif
    /* Gets current clock prescaler */
    clkPrescaler = ((base->CR & STM_CR_CPS_MASK) >> STM_CR_CPS_SHIFT) + 1U;
    /* Gets current functional clock frequency of STM */
    clkErr = CLOCK_SYS_GetFreq(s_stmClkNames[clkSelect], &s_stmClockSrcFreq[instance]);
    /* Checks the functional clock of STM */
    (void)clkErr;
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    DEV_ASSERT(s_stmClockSrcFreq[instance] > 0U);

    /* The formula to convert the microsecond value to the number of tick */
    /* ticks = ((periodUs * ClockSrcFreq) / clkPrescaler) / 1000000 */
    tempTicks = (((uint64_t)periodUs * s_stmClockSrcFreq[instance]) / clkPrescaler) / 1000000U;

    if (tempTicks > STM_COMPARE_MAX)
    {
        /* The number of ticks is out of range of compare register */
        retStatus = STATUS_ERROR;
    }
    else
    {
        /* The number of ticks is in of range of compare register */
        *ticks = (uint32_t)tempTicks;
    }

    return retStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_IncrementTicks
 * Description   : Increases the number of ticks in compare register.
 * This function will compute the compare-value suitable and set that compare-value for compare
 * register to create a periodic event. To make sure about a periodic event, user should call
 * this function immediately after the event occurs.
 *
 * Implements    : STM_DRV_IncrementTicks_Activity
 *END**************************************************************************/
void STM_DRV_IncrementTicks(const uint32_t instance,
                            const uint8_t channel,
                            const uint32_t ticks)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];
    uint32_t curCompareValue;

    /* Gets current compare-value in compare register of channel */
    curCompareValue = base->CHANNEL[channel].CMP;

    if ((STM_COMPARE_MAX - curCompareValue) >= ticks)
    {
        /* The distance from current value to max of compare register is enough */
        base->CHANNEL[channel].CMP = curCompareValue + ticks;
    }
    else
    {
        /* The distance is not enough, calculates a new value for compare register */
        base->CHANNEL[channel].CMP = ticks - (STM_COMPARE_MAX - curCompareValue);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_GetStatusFlags
 * Description   : Gets status of timer interrupt flag.
 * This function returns the status of each channel selected. When common Counter Timer
 * is enabled and value in Counter Timer reaches to compare-value in Channel Compare register
 * then a channel interrupt request is generated.
 *
 * Implements    : STM_DRV_GetStatusFlags_Activity
 *END**************************************************************************/
uint32_t STM_DRV_GetStatusFlags(const uint32_t instance,
                                const uint8_t channel)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    const STM_Type * const base = s_stmBase[instance];
    /* Return status of channel */
    return base->CHANNEL[channel].CIR;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_ClearStatusFlags
 * Description   : Clears channel interrupt flag.
 * This function will clear the flag of channel selected by writing a 1 to bit flag
 * which user wants to clear. All efforts write 0 to bit flag has no effect.
 *
 * Implements    : STM_DRV_ClearStatusFlags_Activity
 *END**************************************************************************/
void STM_DRV_ClearStatusFlags(const uint32_t instance,
                              const uint8_t channel)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
    DEV_ASSERT(channel < STM_CHANNEL_COUNT);

    STM_Type * const base = s_stmBase[instance];
    /* Clear interrupt flag, write 1 to clear */
    base->CHANNEL[channel].CIR = STM_CIR_CIF_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : STM_DRV_GetCounterValue
 * Description   : Returns current counter module.
 * This function will return the counter value at the moment it is called.
 *
 * Implements    : STM_DRV_GetCounterValue_Activity
 *END**************************************************************************/
uint32_t STM_DRV_GetCounterValue(const uint32_t instance)
{
    DEV_ASSERT(instance < STM_INSTANCE_COUNT);
#if defined(ERRATA_E10200)
#if FEATURE_STM_HAS_CLOCK_SELECTION
#if (defined(CUSTOM_DEVASSERT) || defined(DEV_ERROR_DETECT))
    if((s_stmBase[instance]->CR & STM_CR_TEN_MASK) == STM_CR_TEN_MASK)
	{
		DEV_ASSERT(((s_stmBase[instance]->CR & STM_CR_CSL_MASK) >> STM_CR_CSL_SHIFT) != (uint32_t)STM_CLOCK_FXOSC);
	}
#endif /* (defined(CUSTOM_DEVASSERT) || defined(DEV_ERROR_DETECT)) */
#endif /* FEATURE_STM_HAS_CLOCK_SELECTION */
#endif /* defined(ERRATA_E10200) */

    const STM_Type * const base = s_stmBase[instance];
    /* Return current counter */
    return base->CNT;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
