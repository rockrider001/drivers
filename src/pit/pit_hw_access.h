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

#ifndef PIT_HW_ACCESS_H
#define PIT_HW_ACCESS_H

#include <stdbool.h>
#include "device_registers.h"
#include "pit_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Resets the PIT module.
 *
 * This function sets control, load and status registers to default value.
 *
 * @param[in] base PIT peripheral base address
 */
void PIT_Reset(PIT_Type * const base, const uint8_t channelNum);

/*!
 * @brief Enables the PIT timer.
 *
 * This function enables functional clock of PIT timer(standard or RTI timer) (Note: this function
 * does not un-gate the system clock gating control). It should be called before
 * setup any timer channel.
 *
 * @param[in] base PIT peripheral base address
 * @param[in] timerType Timer type
 *              - 0U: Enable standard timer
 *              - 1U: Enable RTI timer
 */
static inline void PIT_EnableTimer(PIT_Type * const base,
                                   const uint8_t timerType)
{
#if FEATURE_PIT_HAS_RTI_CHANNEL
    if (timerType != 0U)
    {
        base->MCR &= ~PIT_MCR_MDIS_RTI_MASK;
    }
    else
#endif
    {
        (void)timerType;
        base->MCR &= ~PIT_MCR_MDIS_MASK;
    }
}

/*!
 * @brief Enables or disables the timer channel chain with the previous timer.
 *
 * When a timer channel has a chain mode enabled, it only counts after the previous
 * timer channel has expired. If the timer channel n-1 has counted down to 0, counter n
 * decrements the value by one. This allows the developers to chain timer channels together
 * and form a longer timer. The first timer channel (channel 0) cannot be chained to any
 * other timer channel
 *
 * @param[in] base PIT peripheral base address
 * @param[in] channel Timer channel number.
 * @param[in] enable Enable or disable timer channel chaining
 *              - true:  The channel is going to be chained with the previous channel
 *              - false: The channel isn't going to be chained with the previous channel
 */
static inline void PIT_SetTimerChainCmd(PIT_Type * const base,
                                        const uint8_t channel,
                                        const bool enable)
{
    DEV_ASSERT(channel < PIT_TIMER_COUNT);
    DEV_ASSERT(!((channel == 0U) && enable));

    if (enable)
    {
        base->TIMER[channel].TCTRL |= PIT_TCTRL_CHN_MASK;
    }
    else
    {
        base->TIMER[channel].TCTRL &= ~PIT_TCTRL_CHN_MASK;
    }
}

/*!
 * @brief Stop timer running on device debug mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. This is intended to aid software development,
 * allowing the developer to halt the processor, investigate the current state of
 * the system (for example, the current timer channel values), and continue the operation
 *
 * @param[in] base PIT peripheral base address
 * @param[in] stopRun PIT stop run in debug mode
 *        - True: PIT stop runs when the device enters debug mode
 *        - False: PIT continue to run when the device enters debug mode
 */
static inline void PIT_SetTimerStopRunInDebugCmd(PIT_Type * const base,
                                                 const bool stopRun)
{
    base->MCR &= ~PIT_MCR_FRZ_MASK;
    base->MCR |= PIT_MCR_FRZ(stopRun ? 1UL: 0UL);
}

#if defined(__cplusplus)
}
#endif

#endif /* PIT_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
