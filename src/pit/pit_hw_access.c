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
 * @file pit_hw_access.c
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
 */

#include "pit_hw_access.h"

/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PIT_Reset
 * Description   : Set control, load and status registers to default value.
 * This function sets control, load and status registers to default value.
 *
 *END**************************************************************************/
void PIT_Reset(PIT_Type * const base, const uint8_t channelNum)
{
    uint32_t i;
    uint32_t mask;

    for (i = 0U; i < channelNum; i++)
    {
        base->TIMER[i].TCTRL = PIT_TCTRL_TEN(0U) | PIT_TCTRL_TIE(0U) | PIT_TCTRL_CHN(0U);
        base->TIMER[i].LDVAL = PIT_LDVAL_TSV(0U);
        base->TIMER[i].TFLG = PIT_TFLG_TIF_MASK;
    }
#if PIT_MCR_MDIS_DEFAULT
    mask = PIT_MCR_MDIS(PIT_MCR_MDIS_DEFAULT) | PIT_MCR_FRZ(0U);
#else
    mask = PIT_MCR_MDIS(1U) | PIT_MCR_FRZ(0U);
#endif

#if FEATURE_PIT_HAS_RTI_CHANNEL
#ifdef PIT_PECULIAR_INSTANCE_HAS_NOT_RTI_CHANNEL
    /*Check if PIT peculiar instance has RTI channel or not then reset RTI channel value*/
    if(base != PIT_PECULIAR_INSTANCE_BASE)
#endif
    {
        base->RTI_TCTRL = PIT_RTI_TCTRL_TEN(0U) | PIT_RTI_TCTRL_TIE(0U);
        base->RTI_LDVAL = PIT_RTI_LDVAL_TSV(0U);
        base->RTI_TFLG = PIT_RTI_TFLG_TIF_MASK;
#if PIT_MCR_MDIS_RTI_DEFAULT
        mask = mask | PIT_MCR_MDIS_RTI(PIT_MCR_MDIS_RTI_DEFAULT);
#else
        mask = mask | PIT_MCR_MDIS_RTI(1U);
#endif
    }
#endif
    base->MCR = mask;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
