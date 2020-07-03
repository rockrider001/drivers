/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#include "device_registers.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @file cmu_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 */

/*!
 * @ingroup cmu_hw_access
 * @defgroup cmu_hw_access
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/


/*!
 * @brief Sets CMU_0 XOSC divider
 */
static inline void CMU_SetDivider(CMU_Type * base, uint32_t rcdiv)
{
	uint32_t regValue = base->CSR;
	regValue &= (uint32_t)(~(CMU_CSR_RCDIV_MASK));
	regValue |= CMU_CSR_RCDIV(rcdiv);
	base->CSR = regValue;
}

/*!
 * @brief Enables/Disables the monitoring unit
 */
static inline void CMU_EnableCMU(CMU_Type * base, bool enable)
{
	uint32_t regValue = base->CSR;
	regValue &= (uint32_t)(~(CMU_CSR_CME_MASK));
	regValue |= CMU_CSR_CME(enable ? 1UL:0UL);
	base->CSR = regValue;
}

/*!
 * @brief Sets inferior limit of the monitored clock
 */
static inline void CMU_SetLowFreq(CMU_Type * base, uint32_t freq)
{
	uint32_t regValue = base->LFREFR;
	regValue &= (uint32_t)(~(CMU_LFREFR_LFREF_MASK));
	regValue |= CMU_LFREFR_LFREF(freq);
	base->LFREFR = regValue;
}

/*!
 * @brief Sets the superior limit of the monitored clock
 */
static inline void CMU_SetHighFreq(CMU_Type * base, uint32_t freq)
{
	uint32_t regValue = base->HFREFR;
	regValue &= (uint32_t)(~(CMU_HFREFR_HFREF_MASK));
	regValue |= CMU_HFREFR_HFREF(freq);
	base->HFREFR = regValue;
}

/*!
 * @brief Return CMU status
 */
static inline uint32_t CMU_GetStatus(const CMU_Type * base)
{
	uint32_t regValue = base->ISR;
	regValue &= (CMU_ISR_FLLI_MASK | CMU_ISR_FHHI_MASK | CMU_ISR_OLRI_MASK);
	return regValue;
}

/*!
 * @brief Clean ISR register
 */
static inline void CMU_CleanStatusRegister(CMU_Type * base)
{
	uint32_t regValue = base->ISR;
	regValue = (uint32_t)(CMU_ISR_FLLI_MASK | CMU_ISR_FHHI_MASK | CMU_ISR_OLRI_MASK);
	base->ISR = regValue;
}


#if defined(__cplusplus)
}
#endif /* __cplusplus*/


/*! @}*/

/*******************************************************************************
 * EOF
 ******************************************************************************/
