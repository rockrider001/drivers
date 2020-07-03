/*
 * Copyright 2017 NXP
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
 * @file etimer_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, not referenced
 * Global typedef not referenced in code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, not referenced
 * Macro not referenced in code.
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
 * Violates MISRA 2012 Required Rule 10.3, essential type
 * Expression assigned to a narrower or different essential type.
 *
 *  @section [global]
 * Violates MISRA 2012 Required Rule 10.7, other operand.
 * Composite expression with smaller essential type than other operand.
 */

#ifndef ETIMER_HW_ACCESS_H
#define ETIMER_HW_ACCESS_H

#include <stdbool.h>
#include "device_registers.h"
#include "etimer_driver.h"

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
 * @brief Resets the ETIMER channel.
 *
 * This function sets all ETIMER registers to reset value,
 * except the Module Control Register.
 *
 * @param[in] base ETIMER peripheral base address
 * @param[in] channel Timer channel number.
 */
static inline void ETIMER_Reset_Channel(ETIMER_Type * const base, const uint16_t channel)
{
	/* set default values as per manual in a logical way which should not generate unwanted behaviour */
	base->CH[channel].INTDMA   = 0x0000U;
	base->CH[channel].CTRL1    = 0x0000U;
	base->CH[channel].CTRL2    = 0x0000U;
	base->CH[channel].CTRL3    = 0x0F00U;
	base->CH[channel].CCCTRL   = 0x0000U;
	base->CH[channel].CMPLD1   = 0x0000U;
	base->CH[channel].CMPLD2   = 0x0000U;
	base->CH[channel].LOAD     = 0x0000U;
	base->CH[channel].COMP1    = 0x0000U;
	base->CH[channel].COMP2    = 0x0000U;
	base->CH[channel].CNTR     = 0x0000U;
	base->CH[channel].FILT     = 0x0000U;
	base->CH[channel].STS      = 0xFFFFU;
}

/*!
 * @brief Resets the ETIMER module.
 *
 * This function sets all ETIMER registers to reset value,
 * except the Module Control Register.
 *
 * @param[in] base ETIMER peripheral base address
 */
static inline void ETIMER_Reset(ETIMER_Type * const base)
{
    uint16_t i=0;
	/* disable all channels */
    base->ENBL &= ~ETIMER_ENBL_ENBL_MASK;
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
    /* disable watchdog */
    base->WDTOL = 0x0000;
    base->WDTOH = 0x0000;
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */
    /* reset DMA */
    for (i=0; i<ETIMER_DREQ_COUNT; i++)
    {
    	base->DREQ[i] = 0;
    }
    /* reset all channels */
    for (i=0; i<ETIMER_CH_COUNT; i++)
    {
    	ETIMER_Reset_Channel(base, i);
    }
}

/*!
 * @brief Sets DIR, PRISRC, SECSRC, SIPS, OPS, OEN in the ETIMER module.
 *
 * @param[in] base ETIMER peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to ETIMER channel configuration structure
 */
static inline void ETIMER_DIR_PRISRC_SECSRC_SIPS(ETIMER_Type * const base,
        const uint16_t channel,
        const etimer_user_channel_config_t * userChannelConfig)
{
	/* DIR & PRISRC */
	base->CH[channel].CTRL1 |=  ( ETIMER_CTRL1_DIR(userChannelConfig->countDirection) | \
			ETIMER_CTRL1_PRISRC(userChannelConfig->primaryInput.source) );
	/* PIPS */
	base->CH[channel].CTRL2 |=  ETIMER_CTRL2_PIPS(userChannelConfig->primaryInput.polarity);

	/* secondary source */
	base->CH[channel].CTRL1 |=  ETIMER_CTRL1_SECSRC(userChannelConfig->secondaryInput.source);
	/* secondary source polarity */
	base->CH[channel].CTRL2 |=  ETIMER_CTRL2_SIPS(userChannelConfig->secondaryInput.polarity);

	/* enable output and set polarity */
	if(userChannelConfig->outputPin.enable)
	{
		/* enable output and set polarity */
		base->CH[channel].CTRL2 |=  ETIMER_CTRL2_OPS(userChannelConfig->outputPin.polarity) | ETIMER_CTRL2_OEN_MASK;
	}
	else
	{
		/* just set polarity */
		base->CH[channel].CTRL2 |=  ETIMER_CTRL2_OPS(userChannelConfig->outputPin.polarity);
	}
}

/*!
 * @brief Sets COMP1 and COMP2 in the ETIMER module.
 *
 * @param[in] base ETIMER peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to ETIMER channel configuration structure
 */
static inline void ETIMER_COMPx(ETIMER_Type * const base,
        const uint16_t channel,
        const etimer_user_channel_config_t * userChannelConfig)
{
    /* COMPx */

    /* set compare values */
    base->CH[channel].COMP1 = userChannelConfig->compareValues[0];
    base->CH[channel].CMPLD1 = userChannelConfig->compareValues[0];

    /* set compare values */
    base->CH[channel].COMP2 = userChannelConfig->compareValues[1];
    base->CH[channel].CMPLD2 = userChannelConfig->compareValues[1];
}

/*!
 * @brief Sets CLC1 and CLC2 and other CCCTRL in the ETIMER module.
 *
 * @param[in] base ETIMER peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to ETIMER channel configuration structure
 */
static inline void ETIMER_CLC1_CLC2_CCCTRL(ETIMER_Type * const base,
        const uint16_t channel,
        const etimer_user_channel_config_t * userChannelConfig)
{
    base->CH[channel].CCCTRL |= ( ETIMER_CCCTRL_CFWM(userChannelConfig->captureWords) | \
            ETIMER_CCCTRL_ONESHOT(userChannelConfig->oneshotCapture ? (uint16_t)1U : (uint16_t)0U) | \
            ETIMER_CCCTRL_CPT1MODE(userChannelConfig->captureControl[0]) | \
            ETIMER_CCCTRL_CPT2MODE(userChannelConfig->captureControl[1])| \
            ETIMER_CCCTRL_CMPMODE(userChannelConfig->compareMode) | \
            ETIMER_CCCTRL_CLC2(userChannelConfig->compareLoading[1]) | \
            ETIMER_CCCTRL_CLC1 (userChannelConfig->compareLoading[0]) );
}

/*!
 * @brief Sets OUTMODE in the ETIMER module.
 *
 * @param[in] base ETIMER peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to ETIMER channel configuration structure
 */
static inline void ETIMER_OUTMODE(ETIMER_Type * const base,
        const uint16_t channel,
        const etimer_user_channel_config_t * userChannelConfig)
{
    /* Set OUTMODE  */
    base->CH[channel].CTRL2 |= ETIMER_CTRL2_OUTMODE(userChannelConfig->compareOutputControl);
}

/*******************************************************************************
 * API Private Extensions
 ******************************************************************************/


#if defined(__cplusplus)
}
#endif

#endif /* ETIMER_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
