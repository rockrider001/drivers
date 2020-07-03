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

#ifndef SIPI_HW_ACCESS_H
#define SIPI_HW_ACCESS_H

/*!
 * @file sipi_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7,  Composite expression with smaller essential type than other operand.
 * The expression is safe as the calculation cannot overflow.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (wider essential type for the destination).
 * The value needs to be assigned to a 32-bit register.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 */

#include "device_registers.h"
#include "status.h"
#include "zipwire_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Arrays referencing channel registers.
 */
extern __IO uint32_t * CCR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];
extern __IO uint32_t * CSR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];
extern __IO uint32_t * CIR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];
extern __IO uint32_t * CTOR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];
extern __IO uint32_t * CAR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];
extern __IO uint32_t * CDR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];

/*!
 * @brief Channel interrupts.
 */
typedef enum
{
    SIPI_ACK_ERR_IRQ        = SIPI_CIR0_ACKIE_MASK,
    SIPI_TRANS_ID_ERR_IRQ   = SIPI_CIR0_TIDIE_MASK,
    SIPI_TIMEOUT_IRQ        = SIPI_CIR0_TOIE_MASK,
    SIPI_TRIGGER_IRQ        = SIPI_CIR0_TCIE_MASK,
    SIPI_READ_ANSWER_IRQ    = SIPI_CIR0_RAIE_MASK,
    SIPI_ACK_IRQ            = SIPI_CIR0_WAIE_MASK
} sipi_channel_interrupt_t;

/*!
 * @brief Channel interrupt flags.
 */
typedef enum
{
    SIPI_ACK_FLAG            = SIPI_CSR0_ACKR_MASK,
    SIPI_TRIGGER_FLAG        = SIPI_CSR0_TID_MASK,
    SIPI_READ_ANSWER_FLAG    = SIPI_CSR0_RAR_MASK
} sipi_channel_flag_t;

/*******************************************************************************
 * CODE
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables/disables SIPI module.
 * @param[in] base - SIPI base pointer.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_Enable(SIPI_Type * base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_MOEN_MASK);
    regValTemp |= SIPI_MCR_MOEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Enables/disables target functionality for SIPI module.
 * @param[in] base - SIPI base pointer.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_TargetEnable(SIPI_Type * base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_TEN_MASK);
    regValTemp |= SIPI_MCR_TEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Enables/disables a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_EnableChannel(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_CHEN_MASK);
    regValTemp |= SIPI_CCR0_CHEN(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables/disables DMA functionality for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_EnableChannelDma(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_DEN_MASK);
    regValTemp |= SIPI_CCR0_DEN(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Sets the transfer word length for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] size - transfer size (8/16/32 bits).
 */
static inline void SIPI_SetChannelWordLength(uint8_t instance, uint8_t channel, zipwire_transfer_size_t size)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_WL_MASK);
    regValTemp |= SIPI_CCR0_WL((uint8_t)size);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables ID transfer request for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelIdTransferRequest(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_IDT_MASK);
    regValTemp |= SIPI_CCR0_IDT(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables read request for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelReadRequest(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_RRT_MASK);
    regValTemp |= SIPI_CCR0_RRT(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables write request for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelWriteRequest(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_WRT_MASK);
    regValTemp |= SIPI_CCR0_WRT(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables streaming write request for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelStreamingWriteRequest(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_ST_MASK);
    regValTemp |= SIPI_CCR0_ST(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Enables trigger request for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelTriggerCommand(uint8_t instance, uint8_t channel, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CCR[instance][channel];
    regValTemp &= ~(SIPI_CCR0_TC_MASK);
    regValTemp |= SIPI_CCR0_TC(enable ? 1UL : 0UL);
    *CCR[instance][channel] = regValTemp;
}

/*!
 * @brief Sets the timeout value for a SIPI channel.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] timeout - timeout value.
 */
static inline void SIPI_SetChannelTimeout(uint8_t instance, uint8_t channel, uint8_t timeout)
{
    uint32_t regValTemp;

    regValTemp = *CTOR[instance][channel];
    regValTemp &= ~(SIPI_CTOR0_TOR_MASK);
    regValTemp |= SIPI_CTOR0_TOR(timeout);
    *CTOR[instance][channel] = regValTemp;
}

/*!
 * @brief Configures the channel interrupts.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] irq - interrupt request.
 * @param[in] enable - true -> enable, false -> disable.
 */
static inline void SIPI_SetChannelInterrupt(uint8_t instance, uint8_t channel, sipi_channel_interrupt_t irq, bool enable)
{
    uint32_t regValTemp;

    regValTemp = *CIR[instance][channel];
    regValTemp &= ~((uint32_t)irq);
    if (enable)
    {
        regValTemp |= (uint32_t)irq;
    }
    *CIR[instance][channel] = regValTemp;
}

/*!
 * @brief Returns whether the channel interrupt is enabled.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] irq - interrupt request.
 * @return - true -> interrupt enabled, false -> interrupt disabled.
 */
static inline bool SIPI_IsChannelInterruptEnabled(uint8_t instance, uint8_t channel, sipi_channel_interrupt_t irq)
{
    return ((*CIR[instance][channel] & ((uint32_t)irq)) > 0U);
}

/*!
 * @brief Clears the channel interrupt flags.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] flag - interrupt flag.
 */
static inline void SIPI_ClearChannelFlag(uint8_t instance, uint8_t channel, sipi_channel_flag_t flag)
{
    *CSR[instance][channel] = (uint32_t)flag;
}

/*!
 * @brief Checks the channel interrupt flags.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] flag - interrupt flag.
 * @return - true if flag is set, false otherwise.
 */
static inline bool SIPI_GetChannelFlag(uint8_t instance, uint8_t channel, sipi_channel_flag_t flag)
{
    return ((*CSR[instance][channel] & (uint32_t)flag) > 0U);
}

/*!
 * @brief Writes the channel address register.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] address - target address value.
 */
static inline void SIPI_SetChannelAddr(uint8_t instance, uint8_t channel, uint32_t address)
{
    *CAR[instance][channel] = address;
}

/*!
 * @brief Writes the channel data register.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @param[in] data - data to be transmitted.
 */
static inline void SIPI_SetChannelData(uint8_t instance, uint8_t channel, uint32_t data)
{
    *CDR[instance][channel] = data;
}

/*!
 * @brief Returns the channel data register value.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @return - received data.
 */
static inline uint32_t SIPI_GetChannelData(uint8_t instance, uint8_t channel)
{
    return *CDR[instance][channel];
}

/*!
 * @brief Performs a soft reset of the SIPI module.
 * @param[in] base - SIPI base pointer.
 * @return - error code.
 */
static inline status_t SIPI_SoftReset(SIPI_Type * base)
{
    volatile uint32_t timeout;
    timeout = SIPI_RESET_TIMEOUT;
    base->MCR |= SIPI_MCR_SR_MASK;
    while ((base->MCR & SIPI_MCR_SR_MASK) > 0U)
    {
        timeout--;
        if (timeout < 1U)
        {
            return STATUS_TIMEOUT;
        }
    }
    return STATUS_SUCCESS;
}

/*!
 * @brief Clears the global CRC error flag.
 * @param[in] base - SIPI base pointer.
 */
static inline void SIPI_ClearGlobalCrcErrFlag(SIPI_Type * base)
{
    base->SR = SIPI_SR_GCRCE_MASK;
}

/*!
 * @brief Clears the maximum count reached interrupt flag.
 * @param[in] base - SIPI base pointer.
 */
static inline void SIPI_ClearMaxCountReachedFlag(SIPI_Type * base)
{
    base->SR = SIPI_SR_MCR_MASK;
}

/*!
 * @brief Returns the value of the maximum count reached interrupt flag.
 * @param[in] base - SIPI base pointer.
 * @return - flag value.
 */
static inline bool SIPI_GetMaxCountReachedFlag(const SIPI_Type * base)
{
    return ((base->SR & SIPI_SR_MCR_MASK) > 0U);
}

/*!
 * @brief Clears the channel trigger event flag.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 */
static inline void SIPI_ClearChnTriggerEventFlag(SIPI_Type * base, uint8_t channel)
{
    base->SR = ((uint32_t)(1U << (((uint8_t)SIPI_SR_TE_SHIFT) + channel)));
}

/*!
 * @brief Returns the channel trigger event flag value.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @return - flag value.
 */
static inline bool SIPI_GetChnTriggerEventFlag(const SIPI_Type * base, uint8_t channel)
{
    return ((base->SR & (1U << (((uint8_t)SIPI_SR_TE_SHIFT) + channel))) > 0U);
}

/*!
 * @brief Clears the channel timeout error flag.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 */
static inline void SIPI_ClearChnTimeoutErrFlag(SIPI_Type * base, uint8_t channel)
{
    base->ERR = ((uint32_t)(((uint8_t)SIPI_ERR_TOE0_MASK) << (channel << 3U)));
}

/*!
 * @brief Returns the channel timeout error flag value.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @return - flag value.
 */
static inline bool SIPI_GetChnTimeoutErrFlag(const SIPI_Type * base, uint8_t channel)
{
    return ((base->ERR & (((uint8_t)SIPI_ERR_TOE0_MASK) << (channel << 3U))) > 0U);
}

/*!
 * @brief Clears the channel transaction ID error flag.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 */
static inline void SIPI_ClearChnTransactionIdErrFlag(SIPI_Type * base, uint8_t channel)
{
    base->ERR = ((uint32_t)(((uint8_t)SIPI_ERR_TIDE0_MASK) << (channel << 3U)));
}

/*!
 * @brief Returns the channel transaction ID error flag value.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @return - flag value.
 */
static inline bool SIPI_GetChnTransactionIdErrFlag(const SIPI_Type * base, uint8_t channel)
{
    return ((base->ERR & (((uint8_t)SIPI_ERR_TIDE0_MASK) << (channel << 3U))) > 0U);
}

/*!
 * @brief Clears the channel ACK error flag.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 */
static inline void SIPI_ClearChnAckErrFlag(SIPI_Type * base, uint8_t channel)
{
    base->ERR = ((uint32_t)(((uint8_t)SIPI_ERR_ACKE0_MASK) << (channel << 3U)));
}

/*!
 * @brief Returns the channel ACK error flag value.
 * @param[in] base - SIPI base pointer.
 * @param[in] channel - channel number.
 * @return - flag value.
 */
static inline bool SIPI_GetChnAckErrFlag(const SIPI_Type * base, uint8_t channel)
{
    return ((base->ERR & (((uint8_t)SIPI_ERR_ACKE0_MASK) << (channel << 3U))) > 0U);
}

/*!
 * @brief Moves SIPI to INIT state.
 * @param[in] base - SIPI base pointer.
 */
static inline void SIPI_EnterInitMode(SIPI_Type * base)
{
    base->MCR |= SIPI_MCR_INIT_MASK;
}

/*!
 * @brief Moves SIPI out of INIT state, back to NORMAL mode.
 * @param[in] base - SIPI base pointer.
 */
static inline void SIPI_ExitInitMode(SIPI_Type * base)
{
    base->MCR &= ~SIPI_MCR_INIT_MASK;
}

/*!
 * @brief Configures maximum count reached interrupt.
 * @param[in] base - SIPI base pointer.
 * @param[in] enable - true - enable irq, false - disable irq.
 */
static inline void SIPI_SetMaxCountReachedInt(SIPI_Type * base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_MCRIE_MASK);
    regValTemp |= SIPI_MCR_MCRIE(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Returns whether maximum count reached interrupt is enabled.
 * @param[in] base - SIPI base pointer.
 * @return - true -> interrupt enabled, false -> interrupt disabled
 */
static inline bool SIPI_IsMaxCountReachedIntEnabled(const SIPI_Type * base)
{
    return ((base->MCR & SIPI_MCR_MCRIE_MASK) > 0U);
}

/*!
 * @brief Configures global crc error interrupt.
 * @param[in] base - SIPI base pointer.
 * @param[in] enable - true - enable irq, false - disable irq.
 */
static inline void SIPI_SetCrcErrInt(SIPI_Type * base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_CRCIE_MASK);
    regValTemp |= SIPI_MCR_CRCIE(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

#ifdef SIPI_ORED_INT_LINES
/*!
 * @brief Retruns whether global crc error interrupt is enabled.
 * @param[in] base - SIPI base pointer.
 * @return - true -> interrupt enabled, false -> interrupt disabled
 */
static inline bool SIPI_IsCrcErrIntEnabled(const SIPI_Type * base)
{
    return ((base->MCR & SIPI_MCR_CRCIE_MASK) > 0U);
}
#endif

/*!
 * @brief Configures address increment/decrement field.
 * @param[in] base - SIPI base pointer.
 * @param[in] addrOffset - address offset value.
 */
static inline void SIPI_SetAddrOffset(SIPI_Type * base, uint8_t addrOffset)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_AID_MASK);
    regValTemp |= SIPI_MCR_AID(addrOffset);
    base->MCR = regValTemp;
}

/*!
 * @brief Configures the timeout clock prescaler value.
 * @param[in] base - SIPI base pointer.
 * @param[in] addrOffset - prescaler value.
 */
static inline void SIPI_SetTimeoutClockPrescaler(SIPI_Type * base, uint16_t prescaler)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(SIPI_MCR_PRSCLR_MASK);
    regValTemp |= SIPI_MCR_PRSCLR(prescaler);
    base->MCR = regValTemp;
}

#if defined(__cplusplus)
}
#endif

#endif /* SIPI_HW_ACCESS_H */
