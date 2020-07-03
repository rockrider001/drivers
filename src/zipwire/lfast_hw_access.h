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

#ifndef LFAST_HW_ACCESS_H
#define LFAST_HW_ACCESS_H

/*!
 * @file lfast_hw_access.h
 */

#include "device_registers.h"
#include "status.h"
#include "zipwire_driver.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief LFAST Master initialization.
 *
 * Initializes the LFAST master interface
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] preDiv - LFAST PLL reference clock divider.
 * @param[in] fbDiv - Feedback Division factor for LFAST PLL VCO output clock.
 * @param[in] speedMode - low-speed/high-speed.
 * @param[in] lsClkDiv - low-speed clock input (sysclk/2 or sysclk/4).
 * @param[in] timeout - cycles allowed for the synchronization to complete.
 *                      A value of zero passed for the timeout parameter is disregarded by the
 *                      driver; the master will wait forever for the responses from the slave.
 * @param[in] syncAttempts - Number of attempts for the master to synchronize with the slave;
 *                           A value of zero for this parameter is equivalent to an infinite
 *                           number of attempts; the LFAST master will wait forever for the
 *                           slave to confirm it's status.
 * @return - error code
 */
status_t LFAST_MasterInit(LFAST_Type *base,
                          lfast_pll_refclk_div_t preDiv,
                          uint8_t fbDiv,
                          lfast_speed_mode_t speedMode,
                          lfast_low_speed_clk_t lsClkDiv,
                          uint32_t timeout,
                          uint32_t attempts);

/*!
 * @brief LFAST Slave initialization.
 *
 * Initializes the LFAST slave interface
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] preDiv - LFAST PLL reference clock divider.
 * @param[in] fbDiv - Feedback Division factor for LFAST PLL VCO output clock.
 * @param[in] speedMode - low-speed/high-speed.
 * @param[in] lsClkDiv - low-speed clock input (sysclk/2 or sysclk/4).
 * @param[in] timeout - cycles allowed for the synchronization to complete.
 *                      A value of zero passed for the timeout parameter is disregarded by the
 *                      driver; the slave will wait forever for the commands from the master.
 * @return - error code
 */
status_t LFAST_SlaveInit(LFAST_Type *base,
                         lfast_pll_refclk_div_t preDiv,
                         uint8_t fbDiv,
                         lfast_speed_mode_t speedMode,
                         lfast_low_speed_clk_t lsClkDiv,
                         uint32_t timeout);

/*!
 * @brief LFAST Soft reset.
 *
 * Performs a soft reset of the LFAST module; causes a reset of the LFAST module,
 * all the registers are reset to their default values and all the FIFOs are flushed.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_Reset(LFAST_Type *base)
{
    base->MCR |= LFAST_MCR_DRFRST_MASK;
    while((base->MCR & LFAST_MCR_DRFRST_MASK) > 0U) {};
}

/*!
 * @brief Configures the role: master/slave.
 *
 * Enables either master or slave functionality.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] role - LFAST role (master/slave).
 */
static inline void LFAST_SetRole(LFAST_Type *base, lfast_role_t role)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_MSEN_MASK);
    regValTemp |= LFAST_MCR_MSEN(role);
    base->MCR = regValTemp;
}

/*!
 * @brief Low speed mode clock configuration.
 *
 * Selects the fraction of LFAST clock in Low Speed Select mode.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] clk  - clock division factor (lfast_clk/2 or lfast_clk/4).
 */
static inline void LFAST_SetLowSpeedClk(LFAST_Type *base, lfast_low_speed_clk_t clk)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_LSSEL_MASK);
    regValTemp |= LFAST_MCR_LSSEL(clk);
    base->MCR = regValTemp;
}

/*!
 * @brief Rx speed mode configuration.
 *
 * Selects the speed mode for the master rx interface.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] mode - speed mode: low/high.
 */
static inline void LFAST_SetRxSpeedMode(LFAST_Type *base, lfast_speed_mode_t mode)
{
    uint32_t regValTemp;

    regValTemp = base->SCR;
    regValTemp &= ~(LFAST_SCR_RDR_MASK);
    regValTemp |= LFAST_SCR_RDR(((uint8_t)mode));
    base->SCR = regValTemp;
}

/*!
 * @brief Returns Rx speed mode configuration.
 *
 * Returns the speed mode for the master rx interface.
 *
 * @param[in] base - LFAST base pointer.
 * @return - true - high speed, false - low speed.
 */
static inline bool LFAST_GetRxSpeedMode(const LFAST_Type *base)
{
    return ((base->GSR & LFAST_GSR_DRSM_MASK) > 0U);
}

/*!
 * @brief Tx speed mode configuration.
 *
 * Selects the speed mode for the master tx interface.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] mode - speed mode: low/high.
 */
static inline void LFAST_SetTxSpeedMode(LFAST_Type *base, lfast_speed_mode_t mode)
{
    uint32_t regValTemp;

    regValTemp = base->SCR;
    regValTemp &= ~(LFAST_SCR_TDR_MASK);
    regValTemp |= LFAST_SCR_TDR(((uint8_t)mode));
    base->SCR = regValTemp;
}

/*!
 * @brief Returns Tx speed mode configuration.
 *
 * Returns the speed mode for the master tx interface.
 *
 * @param[in] base - LFAST base pointer.
 * @return - true - high speed, false - low speed.
 */
static inline bool LFAST_GetTxSpeedMode(const LFAST_Type *base)
{
    return ((base->GSR & LFAST_GSR_LDSM_MASK) > 0U);
}

/*!
 * @brief Enables data rate changes through ICLC frames.
 *
 * Selects whether speed mode is controlled by sw or through ICLC frames.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> data rate controlled by ICLC, false -> data rate controlled by sw.
 */
static inline void LFAST_EnableIclcSpeedModeChange(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->SCR;
    regValTemp &= ~(LFAST_SCR_DRMD_MASK);
    regValTemp |= LFAST_SCR_DRMD(enable ? 1UL : 0UL);
    base->SCR = regValTemp;
}

/*!
 * @brief Enables automatic ping response.
 *
 * Selects whether an automatic response should be sent to a ping frame.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> ping response automatically sent, false -> ping response not automatically sent.
 */
static inline void LFAST_EnableAutomaticPingResponse(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->PICR;
    regValTemp &= ~(LFAST_PICR_PNGAUTO_MASK);
    regValTemp |= LFAST_PICR_PNGAUTO(enable ? 1UL : 0UL);
    base->PICR = regValTemp;
}

/*!
 * @brief Enables/disables LFAST transfers.
 *
 * Enables/disables LFAST operation.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable LFAST, false -> disable LFAST.
 */
static inline void LFAST_Enable(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_DRFEN_MASK);
    regValTemp |= LFAST_MCR_DRFEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Enables/disables LFAST transmitter.
 *
 * Enables/disables LFAST tx operation.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable tx, false -> disable tx.
 */
static inline void LFAST_EnableTx(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_TXEN_MASK);
    regValTemp |= LFAST_MCR_TXEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Checks whether LFAST transmitter is enabled.
 *
 * Returns whether the TX interface has been enabled (used for slave interface
 * enabled by ICLC frames).
 *
 * @param[in] base - LFAST base pointer.
 * @return true - tx interface enabled, false - tx interface disabled.
 */
static inline bool LFAST_IsTxEnabled(const LFAST_Type *base)
{
    return ((base->MCR & LFAST_MCR_TXEN_MASK) > 0U);
}

/*!
 * @brief Enables/disables LFAST receiver.
 *
 * Enables/disables LFAST rx operation.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable rx, false -> disable rx.
 */
static inline void LFAST_EnableRx(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_RXEN_MASK);
    regValTemp |= LFAST_MCR_RXEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Enables/disables LFAST tx arbiter.
 *
 * Enables/disables LFAST the tx block arbiter and framer.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable tx arbiter, false -> disable tx arbiter.
 */
static inline void LFAST_EnableTxArbitrer(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_TXARBD_MASK);
    regValTemp |= LFAST_MCR_TXARBD(enable ? 0UL : 1UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Enables/disables LFAST data frames.
 *
 * Enables/disables LFAST data frame transmission and reception.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable data frames, false -> disable data frames.
 */
static inline void LFAST_EnableData(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->MCR;
    regValTemp &= ~(LFAST_MCR_DATAEN_MASK);
    regValTemp |= LFAST_MCR_DATAEN(enable ? 1UL : 0UL);
    base->MCR = regValTemp;
}

/*!
 * @brief Configures data rate change delay.
 *
 * Defines the number of cycles of needed by the Tx interface data rate change
 * controller to switch from one speed mode to another.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] cyclesCount - data rate change delay (number of cycles).
 */
static inline void LFAST_SetRateChangeDelay(LFAST_Type *base, uint8_t cyclesCount)
{
    DEV_ASSERT (cyclesCount <= 0xFU);
    uint32_t regValTemp;

    regValTemp = base->RCDCR;
    regValTemp &= ~(LFAST_RCDCR_DRCNT_MASK);
    regValTemp |= LFAST_RCDCR_DRCNT(cyclesCount);
    base->RCDCR = regValTemp;
}

/*!
 * @brief Configures feedback division factor for VCO output clock.
 *
 * This function sets the feedback division factor for VCO output clock.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] factor - division factor (accepted range: 0xB-0x1F).
 */
static inline void LFAST_SetPllFeedbackDivisionFactor(LFAST_Type *base, uint8_t factor)
{
    DEV_ASSERT (factor > 0xAU);
    DEV_ASSERT (factor < 0x20U);
    uint32_t regValTemp;

    regValTemp = base->PLLCR;
    regValTemp &= ~(LFAST_PLLCR_FBDIV_MASK);
    regValTemp |= LFAST_PLLCR_FBDIV(factor);
    base->PLLCR = regValTemp;
}

/*!
 * @brief Configures division factor for PLL Reference Clock input.
 *
 * This function sets the division factor for PLL Reference Clock input.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] factor - division factor (accepted range: 0x0-0x3).
 */
static inline void LFAST_SetPllReferenceDivisionFactor(LFAST_Type *base, uint8_t factor)
{
    DEV_ASSERT (factor < 0x4U);
    uint32_t regValTemp;

    regValTemp = base->PLLCR;
    regValTemp &= ~(LFAST_PLLCR_PREDIV_MASK);
    regValTemp |= LFAST_PLLCR_PREDIV(factor);
    base->PLLCR = regValTemp;
}

/*!
 * @brief Turns on he PLL.
 *
 * This function sets the sw signal to turn on the PLL.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_TurnOnPll(LFAST_Type *base)
{
    base->PLLCR |= LFAST_PLLCR_SWPON_MASK;
}

/*!
 * @brief Enables/disables ICLC frame sequence.
 *
 * This function should be called whenever the software is performing
 * a series of ICLC frame transfers.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] enable - true -> enable ICLC sequence, false -> disable ICLC sequence.
 */
static inline void LFAST_EnableIclcSequence(LFAST_Type *base, bool enable)
{
    uint32_t regValTemp;

    regValTemp = base->ICR;
    regValTemp &= ~(LFAST_ICR_ICLCSEQ_MASK);
    regValTemp |= LFAST_ICR_ICLCSEQ(enable ? 1UL : 0UL);
    base->ICR = regValTemp;
}

/*!
 * @brief Requests an ICLC frame..
 *
 * This function initiates the transfer of ICLC frame by the LFAST master.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_IclcFrameRequest(LFAST_Type *base)
{
    base->ICR |= LFAST_ICR_SNDICLC_MASK;
}

/*!
 * @brief Configures ICLC payload.
 *
 * This function sets the ICLC frame payload.
 *
 * @param[in] base - LFAST base pointer.
 * @param[in] payload - ICLC frame payload.
 */
static inline void LFAST_SetIclcPayload(LFAST_Type *base, uint8_t payload)
{
    uint32_t regValTemp;

    regValTemp = base->ICR;
    regValTemp &= ~(LFAST_ICR_ICLCPLD_MASK);
    regValTemp |= LFAST_ICR_ICLCPLD(payload);
    base->ICR = regValTemp;
}

/*!
 * @brief Checks whether the PLL is active.
 *
 * This function returns whether the PLL is enabled or disabled.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - pll active, false - pll inactive.
 */
static inline bool LFAST_PllActive(const LFAST_Type *base)
{
    return (!((base->PLLLSR & LFAST_PLLLSR_PLLDIS_MASK) > 0U));
}

/*!
 * @brief Checks whether the PLL is locked.
 *
 * This function returns whether the PLL is locked.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - pll locked, false - pll unlocked.
 */
static inline bool LFAST_PllLocked(const LFAST_Type *base)
{
    return ((base->PLLLSR & LFAST_PLLLSR_PLDCR_MASK) > 0U);
}

/*!
 * @brief Checks whether the PLL is locked.
 *
 * This function returns whether the PLL is locked.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - flag is set, false - flag not set.
 */
static inline bool LFAST_IclcFrameTransmittedFlag(const LFAST_Type *base)
{
    return ((base->TISR & LFAST_TISR_TXICLCF_MASK) > 0U);
}

/*!
 * @brief Checks ICLC frame transmitted flag.
 *
 * This function returns the value of the 'ICLC frame transmitted' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcFrameTransmittedFlag(LFAST_Type *base)
{
    base->TISR = LFAST_TISR_TXICLCF_MASK;
}

/*!
 * @brief Checks ICLC ping frame request flag.
 *
 * This function returns the value of the 'ICLC ping frame request received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - flag is set, false - flag not set.
 */
static inline bool LFAST_IclcPingFrameRequestReceivedFlag(const LFAST_Type *base)
{
    return ((base->RIISR & LFAST_RIISR_ICPRF_MASK) > 0U);
}

/*!
 * @brief Clears ICLC ping frame request flag.
 *
 * This function clears the 'ICLC ping frame request received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcPingFrameRequestReceivedFlag(LFAST_Type *base)
{
    base->RIISR = LFAST_RIISR_ICPRF_MASK;
}

/*!
 * @brief Checks ICLC ping frame response successful flag.
 *
 * This function returns the value of the 'ICLC frame response successful' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - flag is set, false - flag not set.
 */
static inline bool LFAST_IclcPingFrameResponseSuccessfulFlag(const LFAST_Type *base)
{
    return ((base->RIISR & LFAST_RIISR_ICPSF_MASK) > 0U);
}

/*!
 * @brief Clears ICLC ping frame response successful flag.
 *
 * This function clears the 'ICLC frame response successful' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcPingFrameResponseSuccessfulFlag(LFAST_Type *base)
{
    base->RIISR = LFAST_RIISR_ICPSF_MASK;
}

/*!
 * @brief Clears ICLC frame PLL on flag.
 *
 * This function clears the 'ICLC frame for PLL on received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcPllOnReceivedFlag(LFAST_Type *base)
{
    base->RIISR = LFAST_RIISR_ICPONF_MASK;
}

/*!
 * @brief Checks ICLC frame for slaves tx fast mode switch flag.
 *
 * This function returns the value of the 'ICLC frame for LFAST Slaves Tx Interface fast
 * mode switch received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - flag is set, false - flag not set.
 */
static inline bool LFAST_IclcSlaveTxFastModeReceivedFlag(const LFAST_Type *base)
{
    return ((base->RIISR & LFAST_RIISR_ICTFF_MASK) > 0U);
}

/*!
 * @brief Clears ICLC frame for slaves tx fast mode switch flag.
 *
 * This function clears  the 'ICLC frame for LFAST Slaves Tx Interface fast
 * mode switch received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcSlaveTxFastModeReceivedFlag(LFAST_Type *base)
{
    base->RIISR = LFAST_RIISR_ICTFF_MASK;
}

/*!
 * @brief Checks ICLC frame for slaves rx fast mode switch flag.
 *
 * This function returns the value of the 'ICLC frame for LFAST Slaves rx Interface fast
 * mode switch received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 * @return true - flag is set, false - flag not set.
 */
static inline bool LFAST_IclcSlaveRxFastModeReceivedFlag(const LFAST_Type *base)
{
    return ((base->RIISR & LFAST_RIISR_ICRFF_MASK) > 0U);
}

/*!
 * @brief Clears ICLC frame for slaves rx fast mode switch flag.
 *
 * This function clears the 'ICLC frame for LFAST Slaves rx Interface fast
 * mode switch received' interrupt flag.
 *
 * @param[in] base - LFAST base pointer.
 */
static inline void LFAST_ClearIclcSlaveRxFastModeReceivedFlag(LFAST_Type *base)
{
    base->RIISR = LFAST_RIISR_ICRFF_MASK;
}

#if defined(__cplusplus)
}
#endif

#endif /* LFAST_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
