/*
 * Copyright 2019 NXP
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

#include "sipi_irq.h"

/*!
 * @file sipi_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * These functions are the implementations of weak symbols defined in the startup
 * file for interrupt handlers.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 */

#ifdef SIPI_ORED_INT_LINES
/* SIPI Interrupt vectors */
IRQn_Type sipi_irqs[SIPI_IRQ_COUNT] = {SIPI0_IRQn};

/* SIPI irq handler */
void SIPI0_IRQHandler(void)
{
    uint32_t sipiRarAckrMask = SIPI_CSR0_RAR_MASK | SIPI_CSR0_ACKR_MASK;

    /**** CHANNELS READ/ACK INTERRUPTS ****/
    /* Check channel 0 */
    if ((SIPI->CSR0 & sipiRarAckrMask) > 0U)
    {
        SIPI0_Ch0ResponseOrAckIrqHandler();
    }
    /* Check channel 1 */
    if ((SIPI->CSR1 & sipiRarAckrMask) > 0U)
    {
        SIPI0_Ch1ResponseOrAckIrqHandler();
    }
    /* Check channel 2 */
    if ((SIPI->CSR2 & sipiRarAckrMask) > 0U)
    {
        SIPI0_Ch2ResponseOrAckIrqHandler();
    }
    /* Check channel 3 */
    if ((SIPI->CSR3 & sipiRarAckrMask) > 0U)
    {
        SIPI0_Ch3ResponseOrAckIrqHandler();
    }

    /**** CHANNELS ERROR INTERRUPTS ****/
    if (SIPI->ERR > 0U)
    {
        SIPI0_ErrorIrqHandler();
    }

    /**** GLOBAL CRC ERROR INTERRUPT ****/
    if (SIPI_IsCrcErrIntEnabled(s_sipi_instances[0]))
    {
        if ((SIPI->SR & SIPI_SR_GCRCE_MASK) > 0U)
        {
            SIPI_ClearGlobalCrcErrFlag(s_sipi_instances[0]);
            SIPI_CrcErrHandler(0U);
        }
    }

    /**** TRIGGER OR MAX COUNT REACHED INTERRUPT ****/
    SIPI0_TriggerOrMaxCountReachedIrqHandler();
}
#endif  /* SIPI_ORED_INT_LINES */

#ifdef SIPI_SEPARATED_INT_LINES
/* SIPI Interrupt vectors */
IRQn_Type sipi_irqs[SIPI_IRQ_COUNT] = {SIPI0_IRQn, SIPI1_IRQn, SIPI2_IRQn, SIPI3_IRQn, SIPI4_IRQn, SIPI5_IRQn, SIPI6_IRQn};
#endif /* SIPI_SEPARATED_INT_LINES */

/* SIPI channel error irq handler */
void SIPI0_ErrorIrqHandler(void)
{
    /* Check channel 0 */
    if ((SIPI->ERR & SIPI_CH0_ERR_MASK) > 0U)
    {
        /* Timeout error channel 0 */
        if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_TIMEOUT_IRQ))
        {
            if (SIPI_GetChnTimeoutErrFlag(s_sipi_instances[0], 0U))
            {
                SIPI_ClearChnTimeoutErrFlag(s_sipi_instances[0], 0U);
                SIPI_ChnTimeoutErrHandler(0U, 0U);
                return;
            }
        }
        /* Transaction ID error channel 0 */
        if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_TRANS_ID_ERR_IRQ))
        {
            if (SIPI_GetChnTransactionIdErrFlag(s_sipi_instances[0], 0U))
            {
                SIPI_ClearChnTransactionIdErrFlag(s_sipi_instances[0], 0U);
                SIPI_ChnTransIdErrHandler(0U, 0U);
                return;
            }
        }
        /* ACK error channel 0 */
        if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_ACK_ERR_IRQ))
        {
            if (SIPI_GetChnAckErrFlag(s_sipi_instances[0], 0U))
            {
                SIPI_ClearChnAckErrFlag(s_sipi_instances[0], 0U);
                SIPI_ChnAckErrHandler(0U, 0U);
                return;
            }
        }
    }

    /* Check channel 1 */
    if ((SIPI->ERR & SIPI_CH1_ERR_MASK) > 0U)
    {
        /* Timeout error channel 1 */
        if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_TIMEOUT_IRQ))
        {
            if (SIPI_GetChnTimeoutErrFlag(s_sipi_instances[0], 1U))
            {
                SIPI_ChnTimeoutErrHandler(0U, 1U);
                SIPI_ClearChnTimeoutErrFlag(s_sipi_instances[0], 1U);
                return;
            }
        }
        /* Transaction ID error channel 1 */
        if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_TRANS_ID_ERR_IRQ))
        {
            if (SIPI_GetChnTransactionIdErrFlag(s_sipi_instances[0], 1U))
            {
                SIPI_ClearChnTransactionIdErrFlag(s_sipi_instances[0], 1U);
                SIPI_ChnTransIdErrHandler(0U, 1U);
                return;
            }
        }
        /* ACK error channel 1 */
        if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_ACK_ERR_IRQ))
        {
            if (SIPI_GetChnAckErrFlag(s_sipi_instances[0], 1U))
            {
                SIPI_ClearChnAckErrFlag(s_sipi_instances[0], 1U);
                SIPI_ChnAckErrHandler(0U, 1U);
                return;
            }
        }
    }

    /* Check channel 2 */
    if ((SIPI->ERR & SIPI_CH2_ERR_MASK) > 0U)
    {
        /* Timeout error channel 2 */
        if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_TIMEOUT_IRQ))
        {
            if (SIPI_GetChnTimeoutErrFlag(s_sipi_instances[0], 2U))
            {
                SIPI_ChnTimeoutErrHandler(0U, 2U);
                SIPI_ClearChnTimeoutErrFlag(s_sipi_instances[0], 2U);
                return;
            }
        }
        /* Transaction ID error channel 2 */
        if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_TRANS_ID_ERR_IRQ))
        {
            if (SIPI_GetChnTransactionIdErrFlag(s_sipi_instances[0], 2U))
            {
                SIPI_ClearChnTransactionIdErrFlag(s_sipi_instances[0], 2U);
                SIPI_ChnTransIdErrHandler(0U, 2U);
                return;
            }
        }
        /* ACK error channel 2 */
        if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_ACK_ERR_IRQ))
        {
            if (SIPI_GetChnAckErrFlag(s_sipi_instances[0], 2U))
            {
                SIPI_ClearChnAckErrFlag(s_sipi_instances[0], 2U);
                SIPI_ChnAckErrHandler(0U, 2U);
                return;
            }
        }
    }

    /* Check channel 3 */
    if ((SIPI->ERR & SIPI_CH3_ERR_MASK) > 0U)
    {
        /* Timeout error channel 3 */
        if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_TIMEOUT_IRQ))
        {
            if (SIPI_GetChnTimeoutErrFlag(s_sipi_instances[0], 3U))
            {
                SIPI_ChnTimeoutErrHandler(0U, 3U);
                SIPI_ClearChnTimeoutErrFlag(s_sipi_instances[0], 3U);
                return;
            }
        }
        /* Transaction ID error channel 3 */
        if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_TRANS_ID_ERR_IRQ))
        {
            if (SIPI_GetChnTransactionIdErrFlag(s_sipi_instances[0], 3U))
            {
                SIPI_ClearChnTransactionIdErrFlag(s_sipi_instances[0], 3U);
                SIPI_ChnTransIdErrHandler(0U, 3U);
                return;
            }
        }
        /* ACK error channel 3 */
        if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_ACK_ERR_IRQ))
        {
            if (SIPI_GetChnAckErrFlag(s_sipi_instances[0], 3U))
            {
                SIPI_ClearChnAckErrFlag(s_sipi_instances[0], 3U);
                SIPI_ChnAckErrHandler(0U, 3U);
                return;
            }
        }
    }
}

/* SIPI crc error irq handler */
void SIPI0_CrcErrorIrqHandler(void)
{
    SIPI_ClearGlobalCrcErrFlag(s_sipi_instances[0]);
    SIPI_CrcErrHandler(0U);
}

/* SIPI channel 0 response or ack irq handler */
void SIPI0_Ch0ResponseOrAckIrqHandler(void)
{
    /* Check channel 0 read answer received */
    if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_READ_ANSWER_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 0U, SIPI_READ_ANSWER_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 0U, SIPI_READ_ANSWER_FLAG);
            SIPI_ChnReadAnswerHandler(0U, 0U);
            return;
        }
    }

    /* Check channel 0 ack */
    if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_ACK_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 0U, SIPI_ACK_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 0U, SIPI_ACK_FLAG);
            SIPI_ChnAckHandler(0U, 0U);
            return;
        }
    }
}

/* SIPI channel 1 response or ack irq handler */
void SIPI0_Ch1ResponseOrAckIrqHandler(void)
{
    /* Check channel 1 read answer received */
    if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_READ_ANSWER_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 1U, SIPI_READ_ANSWER_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 1U, SIPI_READ_ANSWER_FLAG);
            SIPI_ChnReadAnswerHandler(0U, 1U);
            return;
        }
    }

    /* Check channel 1 ack */
    if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_ACK_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 1U, SIPI_ACK_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 1U, SIPI_ACK_FLAG);
            SIPI_ChnAckHandler(0U, 1U);
            return;
        }
    }
}

/* SIPI channel 2 response or ack irq handler */
void SIPI0_Ch2ResponseOrAckIrqHandler(void)
{
    /* Check channel 2 read answer received */
    if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_READ_ANSWER_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 2U, SIPI_READ_ANSWER_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 2U, SIPI_READ_ANSWER_FLAG);
            SIPI_ChnReadAnswerHandler(0U, 2U);
            return;
        }
    }

    /* Check channel 2 ack */
    if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_ACK_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 2U, SIPI_ACK_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 2U, SIPI_ACK_FLAG);
            SIPI_ChnAckHandler(0U, 2U);
            return;
        }
    }
}

/* SIPI channel 3 response or ack irq handler */
void SIPI0_Ch3ResponseOrAckIrqHandler(void)
{
    /* Check channel 3 read answer received */
    if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_READ_ANSWER_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 3U, SIPI_READ_ANSWER_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 3U, SIPI_READ_ANSWER_FLAG);
            SIPI_ChnReadAnswerHandler(0U, 3U);
            return;
        }
    }

    /* Check channel 3 ack */
    if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_ACK_IRQ))
    {
        if (SIPI_GetChannelFlag(0U, 3U, SIPI_ACK_FLAG))
        {
            SIPI_ClearChannelFlag(0U, 3U, SIPI_ACK_FLAG);
            SIPI_ChnAckHandler(0U, 3U);
            return;
        }
    }
}

/* SIPI trigger or max count reached irq handler */
void SIPI0_TriggerOrMaxCountReachedIrqHandler(void)
{
    /* Max count reached */
    if (SIPI_IsMaxCountReachedIntEnabled(s_sipi_instances[0]))
    {
        if (SIPI_GetMaxCountReachedFlag(s_sipi_instances[0]))
        {
            SIPI_ClearMaxCountReachedFlag(s_sipi_instances[0]);
            SIPI_MaxCountReachedHandler(0U);
            return;
        }
    }

    /* Trigger channel 0 */
    if (SIPI_IsChannelInterruptEnabled(0U, 0U, SIPI_TRIGGER_IRQ))
    {
        if (SIPI_GetChnTriggerEventFlag(s_sipi_instances[0], 0U))
        {
            SIPI_ClearChnTriggerEventFlag(s_sipi_instances[0], 0U);
            SIPI_ChnTriggerHandler(0U, 0U);
            return;
        }
    }

    /* Trigger channel 1 */
    if (SIPI_IsChannelInterruptEnabled(0U, 1U, SIPI_TRIGGER_IRQ))
    {
        if (SIPI_GetChnTriggerEventFlag(s_sipi_instances[0], 1U))
        {
            SIPI_ClearChnTriggerEventFlag(s_sipi_instances[0], 1U);
            SIPI_ChnTriggerHandler(0U, 1U);
            return;
        }
    }

    /* Trigger channel 2 */
    if (SIPI_IsChannelInterruptEnabled(0U, 2U, SIPI_TRIGGER_IRQ))
    {
        if (SIPI_GetChnTriggerEventFlag(s_sipi_instances[0], 2U))
        {
            SIPI_ClearChnTriggerEventFlag(s_sipi_instances[0], 2U);
            SIPI_ChnTriggerHandler(0U, 2U);
            return;
        }
    }

    /* Trigger channel 3 */
    if (SIPI_IsChannelInterruptEnabled(0U, 3U, SIPI_TRIGGER_IRQ))
    {
        if (SIPI_GetChnTriggerEventFlag(s_sipi_instances[0], 3U))
        {
            SIPI_ClearChnTriggerEventFlag(s_sipi_instances[0], 3U);
            SIPI_ChnTriggerHandler(0U, 3U);
            return;
        }
    }
}
