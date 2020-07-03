/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * These are symbols weak symbols defined in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, external could be made static.
 * The functions are called by the interrupt controller when the appropriate event
 * occurs.
 */


#include <stddef.h>
#include "srx_irq.h"
#include "device_registers.h"
#include "interrupt_manager.h"
#include "srx_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef FEATURE_SRX_HAS_COMBINED_IRQ

/**
 * Array containing interrupt mappings.
 */
static const IRQn_Type s_srxInterruptMappings[SRX_INSTANCE_COUNT][SRX_CHANNEL_COUNT] = FEATURE_SRX_IRQS;

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_EnableIRQ
 * Description   : Enabler for SRX_DRV_IRQ_EnableIRQ interrupt
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_EnableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id)
{
    (void)id; /* Shared */
    INT_SYS_EnableIRQ(s_srxInterruptMappings[instance][channel]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_DisableIRQ
 * Description   : Disabler for SRX_DRV_IRQ_DisableIRQ interrupt
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_DisableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id)
{
    (void)id; /* Shared */
    INT_SYS_DisableIRQ(s_srxInterruptMappings[instance][channel]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_Dispatch
 * Description   : Interrupt dispatcher
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_Dispatch(const uint32_t instance, const uint32_t channel)
{
    /* Dispatch according to event type */
    if (SRX_DRV_HW_GetFastRxStatus(instance, channel))
    {
        SRX_DRV_IRQ_FastHandler(instance, channel);
    }
    else if(SRX_DRV_HW_GetSlowRxStatus(instance, channel))
    {
        SRX_DRV_IRQ_SlowHandler(instance, channel);
    }
    else
    {
        /* Fall through, error */
        SRX_DRV_IRQ_RxErrHandler(instance, channel);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH0_IRQHandler
 * Description   : Handler for SRX0_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH1_IRQHandler
 * Description   : Handler for SRX0_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH2_IRQHandler
 * Description   : Handler for SRX0_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH3_IRQHandler
 * Description   : Handler for SRX0_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH4_IRQHandler
 * Description   : Handler for SRX0_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH5_IRQHandler
 * Description   : Handler for SRX0_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH6_IRQHandler
 * Description   : Handler for SRX0_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX0_CH7_IRQHandler
 * Description   : Handler for SRX0_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX0_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(0u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#if (SRX_INSTANCE_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH0_IRQHandler
 * Description   : Handler for SRX1_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH1_IRQHandler
 * Description   : Handler for SRX1_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH2_IRQHandler
 * Description   : Handler for SRX1_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH3_IRQHandler
 * Description   : Handler for SRX1_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH4_IRQHandler
 * Description   : Handler for SRX1_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH5_IRQHandler
 * Description   : Handler for SRX1_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH6_IRQHandler
 * Description   : Handler for SRX1_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX1_CH7_IRQHandler
 * Description   : Handler for SRX1_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX1_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(1u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 1u) */

#if (SRX_INSTANCE_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH0_IRQHandler
 * Description   : Handler for SRX2_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH1_IRQHandler
 * Description   : Handler for SRX2_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH2_IRQHandler
 * Description   : Handler for SRX2_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH3_IRQHandler
 * Description   : Handler for SRX2_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH4_IRQHandler
 * Description   : Handler for SRX2_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH5_IRQHandler
 * Description   : Handler for SRX2_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH6_IRQHandler
 * Description   : Handler for SRX2_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX2_CH7_IRQHandler
 * Description   : Handler for SRX2_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX2_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(2u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 2u) */

#if (SRX_INSTANCE_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH0_IRQHandler
 * Description   : Handler for SRX3_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH1_IRQHandler
 * Description   : Handler for SRX3_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH2_IRQHandler
 * Description   : Handler for SRX3_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH3_IRQHandler
 * Description   : Handler for SRX3_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH4_IRQHandler
 * Description   : Handler for SRX3_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH5_IRQHandler
 * Description   : Handler for SRX3_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH6_IRQHandler
 * Description   : Handler for SRX3_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX3_CH7_IRQHandler
 * Description   : Handler for SRX3_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX3_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(3u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 3u) */

#if (SRX_INSTANCE_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH0_IRQHandler
 * Description   : Handler for SRX4_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH1_IRQHandler
 * Description   : Handler for SRX4_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH2_IRQHandler
 * Description   : Handler for SRX4_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH3_IRQHandler
 * Description   : Handler for SRX4_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH4_IRQHandler
 * Description   : Handler for SRX4_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH5_IRQHandler
 * Description   : Handler for SRX4_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH6_IRQHandler
 * Description   : Handler for SRX4_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX4_CH7_IRQHandler
 * Description   : Handler for SRX4_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX4_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(4u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#endif /* (SRX_INSTANCE_COUNT > 4u) */

#if (SRX_INSTANCE_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH0_IRQHandler
 * Description   : Handler for SRX5_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH1_IRQHandler
 * Description   : Handler for SRX5_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH2_IRQHandler
 * Description   : Handler for SRX5_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH3_IRQHandler
 * Description   : Handler for SRX5_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH4_IRQHandler
 * Description   : Handler for SRX5_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH5_IRQHandler
 * Description   : Handler for SRX5_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH6_IRQHandler
 * Description   : Handler for SRX5_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX5_CH7_IRQHandler
 * Description   : Handler for SRX5_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX5_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(5u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#endif /* (SRX_INSTANCE_COUNT > 5u) */

#if (SRX_INSTANCE_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH0_IRQHandler
 * Description   : Handler for SRX6_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH1_IRQHandler
 * Description   : Handler for SRX6_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH2_IRQHandler
 * Description   : Handler for SRX6_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH3_IRQHandler
 * Description   : Handler for SRX6_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH4_IRQHandler
 * Description   : Handler for SRX6_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH5_IRQHandler
 * Description   : Handler for SRX6_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH6_IRQHandler
 * Description   : Handler for SRX6_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX6_CH7_IRQHandler
 * Description   : Handler for SRX6_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX6_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(6u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#endif /* (SRX_INSTANCE_COUNT > 6u) */

#if (SRX_INSTANCE_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH0_IRQHandler
 * Description   : Handler for SRX7_CH0_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH0_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH1_IRQHandler
 * Description   : Handler for SRX7_CH1_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH1_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH2_IRQHandler
 * Description   : Handler for SRX7_CH2_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH2_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH3_IRQHandler
 * Description   : Handler for SRX7_CH3_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH3_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH4_IRQHandler
 * Description   : Handler for SRX7_CH4_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH4_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH5_IRQHandler
 * Description   : Handler for SRX7_CH5_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH5_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH6_IRQHandler
 * Description   : Handler for SRX7_CH6_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH6_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX7_CH7_IRQHandler
 * Description   : Handler for SRX7_CH7_IRQn interrupt
 *
 *END**************************************************************************/
void SRX7_CH7_IRQHandler(void)
{
    SRX_DRV_IRQ_Dispatch(7u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#endif /* (SRX_INSTANCE_COUNT > 7u) */

#else

/**
 * Types of interrupts (Fast/Slow/Err)
 */
#define SRX_IRQ_MAP_TYPES (3u)

/**
 * Array containing interrupt mappings.
 */
static const IRQn_Type s_srxInterruptMappings[SRX_INSTANCE_COUNT][SRX_CHANNEL_COUNT][SRX_IRQ_MAP_TYPES] = FEATURE_SRX_IRQS;


/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_EnableIRQ
 * Description   : Enabler for SRX_DRV_IRQ_EnableIRQ interrupt
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_EnableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id)
{
    INT_SYS_EnableIRQ(s_srxInterruptMappings[instance][channel][id]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_DisableIRQ
 * Description   : Disabler for SRX_DRV_IRQ_DisableIRQ interrupt
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_DisableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id)
{
    INT_SYS_DisableIRQ(s_srxInterruptMappings[instance][channel][id]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_0_IRQHandler
 * Description   : Handler for SENT0_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_0_IRQHandler
 * Description   : Handler for SENT0_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_0_IRQHandler
 * Description   : Handler for SENT0_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_1_IRQHandler
 * Description   : Handler for SENT0_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_1_IRQHandler
 * Description   : Handler for SENT0_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_1_IRQHandler
 * Description   : Handler for SENT0_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_2_IRQHandler
 * Description   : Handler for SENT0_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_2_IRQHandler
 * Description   : Handler for SENT0_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_2_IRQHandler
 * Description   : Handler for SENT0_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_3_IRQHandler
 * Description   : Handler for SENT0_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_3_IRQHandler
 * Description   : Handler for SENT0_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_3_IRQHandler
 * Description   : Handler for SENT0_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_4_IRQHandler
 * Description   : Handler for SENT0_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_4_IRQHandler
 * Description   : Handler for SENT0_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_4_IRQHandler
 * Description   : Handler for SENT0_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_5_IRQHandler
 * Description   : Handler for SENT0_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_5_IRQHandler
 * Description   : Handler for SENT0_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_5_IRQHandler
 * Description   : Handler for SENT0_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_6_IRQHandler
 * Description   : Handler for SENT0_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_6_IRQHandler
 * Description   : Handler for SENT0_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_6_IRQHandler
 * Description   : Handler for SENT0_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Fast_7_IRQHandler
 * Description   : Handler for SENT0_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(0u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_Slow_7_IRQHandler
 * Description   : Handler for SENT0_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(0u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT0_RxErr_7_IRQHandler
 * Description   : Handler for SENT0_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT0_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(0u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#if (SRX_INSTANCE_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_0_IRQHandler
 * Description   : Handler for SENT1_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_0_IRQHandler
 * Description   : Handler for SENT1_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_0_IRQHandler
 * Description   : Handler for SENT1_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_1_IRQHandler
 * Description   : Handler for SENT1_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_1_IRQHandler
 * Description   : Handler for SENT1_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_1_IRQHandler
 * Description   : Handler for SENT1_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_2_IRQHandler
 * Description   : Handler for SENT1_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_2_IRQHandler
 * Description   : Handler for SENT1_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_2_IRQHandler
 * Description   : Handler for SENT1_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_3_IRQHandler
 * Description   : Handler for SENT1_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_3_IRQHandler
 * Description   : Handler for SENT1_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_3_IRQHandler
 * Description   : Handler for SENT1_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_4_IRQHandler
 * Description   : Handler for SENT1_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_4_IRQHandler
 * Description   : Handler for SENT1_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_4_IRQHandler
 * Description   : Handler for SENT1_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_5_IRQHandler
 * Description   : Handler for SENT1_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_5_IRQHandler
 * Description   : Handler for SENT1_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_5_IRQHandler
 * Description   : Handler for SENT1_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_6_IRQHandler
 * Description   : Handler for SENT1_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_6_IRQHandler
 * Description   : Handler for SENT1_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_6_IRQHandler
 * Description   : Handler for SENT1_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Fast_7_IRQHandler
 * Description   : Handler for SENT1_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(1u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_Slow_7_IRQHandler
 * Description   : Handler for SENT1_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(1u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT1_RxErr_7_IRQHandler
 * Description   : Handler for SENT1_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT1_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(1u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 1u) */

#if (SRX_INSTANCE_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_0_IRQHandler
 * Description   : Handler for SENT2_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_0_IRQHandler
 * Description   : Handler for SENT2_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_0_IRQHandler
 * Description   : Handler for SENT2_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_1_IRQHandler
 * Description   : Handler for SENT2_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_1_IRQHandler
 * Description   : Handler for SENT2_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_1_IRQHandler
 * Description   : Handler for SENT2_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_2_IRQHandler
 * Description   : Handler for SENT2_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_2_IRQHandler
 * Description   : Handler for SENT2_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_2_IRQHandler
 * Description   : Handler for SENT2_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_3_IRQHandler
 * Description   : Handler for SENT2_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_3_IRQHandler
 * Description   : Handler for SENT2_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_3_IRQHandler
 * Description   : Handler for SENT2_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_4_IRQHandler
 * Description   : Handler for SENT2_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_4_IRQHandler
 * Description   : Handler for SENT2_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_4_IRQHandler
 * Description   : Handler for SENT2_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_5_IRQHandler
 * Description   : Handler for SENT2_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_5_IRQHandler
 * Description   : Handler for SENT2_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_5_IRQHandler
 * Description   : Handler for SENT2_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_6_IRQHandler
 * Description   : Handler for SENT2_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_6_IRQHandler
 * Description   : Handler for SENT2_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_6_IRQHandler
 * Description   : Handler for SENT2_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Fast_7_IRQHandler
 * Description   : Handler for SENT2_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(2u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_Slow_7_IRQHandler
 * Description   : Handler for SENT2_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(2u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT2_RxErr_7_IRQHandler
 * Description   : Handler for SENT2_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT2_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(2u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 2u) */

#if (SRX_INSTANCE_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_0_IRQHandler
 * Description   : Handler for SENT3_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_0_IRQHandler
 * Description   : Handler for SENT3_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_0_IRQHandler
 * Description   : Handler for SENT3_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_1_IRQHandler
 * Description   : Handler for SENT3_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_1_IRQHandler
 * Description   : Handler for SENT3_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_1_IRQHandler
 * Description   : Handler for SENT3_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_2_IRQHandler
 * Description   : Handler for SENT3_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_2_IRQHandler
 * Description   : Handler for SENT3_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_2_IRQHandler
 * Description   : Handler for SENT3_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_3_IRQHandler
 * Description   : Handler for SENT3_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_3_IRQHandler
 * Description   : Handler for SENT3_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_3_IRQHandler
 * Description   : Handler for SENT3_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_4_IRQHandler
 * Description   : Handler for SENT3_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_4_IRQHandler
 * Description   : Handler for SENT3_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_4_IRQHandler
 * Description   : Handler for SENT3_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_5_IRQHandler
 * Description   : Handler for SENT3_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_5_IRQHandler
 * Description   : Handler for SENT3_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_5_IRQHandler
 * Description   : Handler for SENT3_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_6_IRQHandler
 * Description   : Handler for SENT3_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_6_IRQHandler
 * Description   : Handler for SENT3_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_6_IRQHandler
 * Description   : Handler for SENT3_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Fast_7_IRQHandler
 * Description   : Handler for SENT3_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(3u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_Slow_7_IRQHandler
 * Description   : Handler for SENT3_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(3u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT3_RxErr_7_IRQHandler
 * Description   : Handler for SENT3_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT3_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(3u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 3u) */

#if (SRX_INSTANCE_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_0_IRQHandler
 * Description   : Handler for SENT4_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_0_IRQHandler
 * Description   : Handler for SENT4_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_0_IRQHandler
 * Description   : Handler for SENT4_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_1_IRQHandler
 * Description   : Handler for SENT4_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_1_IRQHandler
 * Description   : Handler for SENT4_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_1_IRQHandler
 * Description   : Handler for SENT4_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_2_IRQHandler
 * Description   : Handler for SENT4_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_2_IRQHandler
 * Description   : Handler for SENT4_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_2_IRQHandler
 * Description   : Handler for SENT4_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_3_IRQHandler
 * Description   : Handler for SENT4_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_3_IRQHandler
 * Description   : Handler for SENT4_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_3_IRQHandler
 * Description   : Handler for SENT4_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_4_IRQHandler
 * Description   : Handler for SENT4_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_4_IRQHandler
 * Description   : Handler for SENT4_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_4_IRQHandler
 * Description   : Handler for SENT4_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_5_IRQHandler
 * Description   : Handler for SENT4_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_5_IRQHandler
 * Description   : Handler for SENT4_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_5_IRQHandler
 * Description   : Handler for SENT4_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_6_IRQHandler
 * Description   : Handler for SENT4_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_6_IRQHandler
 * Description   : Handler for SENT4_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_6_IRQHandler
 * Description   : Handler for SENT4_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Fast_7_IRQHandler
 * Description   : Handler for SENT4_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(4u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_Slow_7_IRQHandler
 * Description   : Handler for SENT4_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(4u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT4_RxErr_7_IRQHandler
 * Description   : Handler for SENT4_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT4_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(4u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 4u) */

#if (SRX_INSTANCE_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_0_IRQHandler
 * Description   : Handler for SENT5_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_0_IRQHandler
 * Description   : Handler for SENT5_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_0_IRQHandler
 * Description   : Handler for SENT5_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_1_IRQHandler
 * Description   : Handler for SENT5_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_1_IRQHandler
 * Description   : Handler for SENT5_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_1_IRQHandler
 * Description   : Handler for SENT5_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_2_IRQHandler
 * Description   : Handler for SENT5_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_2_IRQHandler
 * Description   : Handler for SENT5_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_2_IRQHandler
 * Description   : Handler for SENT5_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_3_IRQHandler
 * Description   : Handler for SENT5_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_3_IRQHandler
 * Description   : Handler for SENT5_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_3_IRQHandler
 * Description   : Handler for SENT5_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_4_IRQHandler
 * Description   : Handler for SENT5_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_4_IRQHandler
 * Description   : Handler for SENT5_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_4_IRQHandler
 * Description   : Handler for SENT5_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_5_IRQHandler
 * Description   : Handler for SENT5_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_5_IRQHandler
 * Description   : Handler for SENT5_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_5_IRQHandler
 * Description   : Handler for SENT5_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_6_IRQHandler
 * Description   : Handler for SENT5_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_6_IRQHandler
 * Description   : Handler for SENT5_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_6_IRQHandler
 * Description   : Handler for SENT5_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Fast_7_IRQHandler
 * Description   : Handler for SENT5_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(5u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_Slow_7_IRQHandler
 * Description   : Handler for SENT5_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(5u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT5_RxErr_7_IRQHandler
 * Description   : Handler for SENT5_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT5_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(5u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 5u) */

#if (SRX_INSTANCE_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_0_IRQHandler
 * Description   : Handler for SENT6_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_0_IRQHandler
 * Description   : Handler for SENT6_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_0_IRQHandler
 * Description   : Handler for SENT6_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_1_IRQHandler
 * Description   : Handler for SENT6_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_1_IRQHandler
 * Description   : Handler for SENT6_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_1_IRQHandler
 * Description   : Handler for SENT6_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_2_IRQHandler
 * Description   : Handler for SENT6_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_2_IRQHandler
 * Description   : Handler for SENT6_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_2_IRQHandler
 * Description   : Handler for SENT6_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_3_IRQHandler
 * Description   : Handler for SENT6_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_3_IRQHandler
 * Description   : Handler for SENT6_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_3_IRQHandler
 * Description   : Handler for SENT6_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_4_IRQHandler
 * Description   : Handler for SENT6_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_4_IRQHandler
 * Description   : Handler for SENT6_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_4_IRQHandler
 * Description   : Handler for SENT6_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_5_IRQHandler
 * Description   : Handler for SENT6_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_5_IRQHandler
 * Description   : Handler for SENT6_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_5_IRQHandler
 * Description   : Handler for SENT6_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_6_IRQHandler
 * Description   : Handler for SENT6_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_6_IRQHandler
 * Description   : Handler for SENT6_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_6_IRQHandler
 * Description   : Handler for SENT6_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Fast_7_IRQHandler
 * Description   : Handler for SENT6_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(6u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_Slow_7_IRQHandler
 * Description   : Handler for SENT6_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(6u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT6_RxErr_7_IRQHandler
 * Description   : Handler for SENT6_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT6_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(6u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */


#endif /* (SRX_INSTANCE_COUNT > 6u) */

#if (SRX_INSTANCE_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_0_IRQHandler
 * Description   : Handler for SENT7_Fast_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_0_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_0_IRQHandler
 * Description   : Handler for SENT7_Slow_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_0_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 0u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_0_IRQHandler
 * Description   : Handler for SENT7_RxErr_0_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_0_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 0u);
}

#if (SRX_CHANNEL_COUNT > 1u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_1_IRQHandler
 * Description   : Handler for SENT7_Fast_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_1_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_1_IRQHandler
 * Description   : Handler for SENT7_Slow_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_1_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 1u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_1_IRQHandler
 * Description   : Handler for SENT7_RxErr_1_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_1_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 1u);
}
#endif /* (SRX_CHANNEL_COUNT > 1u) */

#if (SRX_CHANNEL_COUNT > 2u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_2_IRQHandler
 * Description   : Handler for SENT7_Fast_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_2_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_2_IRQHandler
 * Description   : Handler for SENT7_Slow_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_2_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 2u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_2_IRQHandler
 * Description   : Handler for SENT7_RxErr_2_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_2_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 2u);
}
#endif /* (SRX_CHANNEL_COUNT > 2u) */

#if (SRX_CHANNEL_COUNT > 3u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_3_IRQHandler
 * Description   : Handler for SENT7_Fast_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_3_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_3_IRQHandler
 * Description   : Handler for SENT7_Slow_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_3_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 3u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_3_IRQHandler
 * Description   : Handler for SENT7_RxErr_3_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_3_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 3u);
}
#endif /* (SRX_CHANNEL_COUNT > 3u) */

#if (SRX_CHANNEL_COUNT > 4u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_4_IRQHandler
 * Description   : Handler for SENT7_Fast_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_4_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_4_IRQHandler
 * Description   : Handler for SENT7_Slow_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_4_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 4u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_4_IRQHandler
 * Description   : Handler for SENT7_RxErr_4_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_4_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 4u);
}
#endif /* (SRX_CHANNEL_COUNT > 4u) */

#if (SRX_CHANNEL_COUNT > 5u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_5_IRQHandler
 * Description   : Handler for SENT7_Fast_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_5_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_5_IRQHandler
 * Description   : Handler for SENT7_Slow_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_5_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 5u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_5_IRQHandler
 * Description   : Handler for SENT7_RxErr_5_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_5_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 5u);
}
#endif /* (SRX_CHANNEL_COUNT > 5u) */

#if (SRX_CHANNEL_COUNT > 6u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_6_IRQHandler
 * Description   : Handler for SENT7_Fast_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_6_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_6_IRQHandler
 * Description   : Handler for SENT7_Slow_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_6_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 6u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_6_IRQHandler
 * Description   : Handler for SENT7_RxErr_6_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_6_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 6u);
}
#endif /* (SRX_CHANNEL_COUNT > 6u) */

#if (SRX_CHANNEL_COUNT > 7u)
/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Fast_7_IRQHandler
 * Description   : Handler for SENT7_Fast_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Fast_7_IRQHandler(void)
{
    SRX_DRV_IRQ_FastHandler(7u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_Slow_7_IRQHandler
 * Description   : Handler for SENT7_Slow_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_Slow_7_IRQHandler(void)
{
    SRX_DRV_IRQ_SlowHandler(7u, 7u);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SENT7_RxErr_7_IRQHandler
 * Description   : Handler for SENT7_RxErr_7_IRQn interrupt
 *
 *END**************************************************************************/
void SENT7_RxErr_7_IRQHandler(void)
{
    SRX_DRV_IRQ_RxErrHandler(7u, 7u);
}
#endif /* (SRX_CHANNEL_COUNT > 7u) */

#endif /* (SRX_INSTANCE_COUNT > 7u) */

#endif

/*******************************************************************************
* EOF
*******************************************************************************/




