/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef SRX_HW_ACCESS_H
#define SRX_HW_ACCESS_H

#include <stddef.h>
#include "srx_driver.h"
#include "device_registers.h"

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(ERRATA_E7425) /* Enable only when we have this active */
/**
 * @brief Workarounds for E7425.
 *
 * This function implements the workarounds required for E7425
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] instance The SRX peripheral binstance
 * @param[in] channel The requested channel
 * @return true if handler yields a legitimate event
 */
bool  SRX_DRV_HW_ErrataE7425Workaroud(const uint32_t instance, const uint32_t channel);
#endif /* defined(ERRATA_E7425) */

/**
 * @brief Set timestamp prescaler
 *
 * This function sets the timestamp prescaler value
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] value The prescaler value
 */
void SRX_DRV_HW_SetTimestampPrescaler(const uint32_t instance, const uint8_t value);

#ifdef FEATURE_SRX_DMA_HAS_FIFO
/**
 * @brief Set FIFO length
 *
 * This function sets the FIFO watermark level.
 * The level describes the number of valid messages that are to
 * be stored in the FIFO queue before a DMA transfer is triggered.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] value FIFO length
 */
void SRX_DRV_HW_SetFifoWm(const uint32_t instance, const uint8_t value);

/**
 * @brief Set FIFO state
 *
 * Sets the enable state (active/inactive) for the DMA FIFO
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] state Requested state
 */
void SRX_DRV_HW_SetFifoState(const uint32_t instance, const bool state);
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

/**
 * @brief Set channel prescaler
 *
 * Sets the channel prescaler.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] prescaler Prescaler value
 */
void SRX_DRV_HW_SetChannelPrescaler(const uint32_t instance, const uint8_t channel, const uint16_t prescaler);

/**
 * @brief Set channel compensation
 *
 * Sets the channel compensation state (enable / disable).
 * This enables automatic compesation for deviations in the receive clock.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] state Requested state
 */
void SRX_DRV_HW_SetChannelCompensation(const uint32_t instance, const uint8_t channel, const bool state);

/**
 * @brief Set bus idle count
 *
 * Sets the value for the Bus Idle Count paramter.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] count Parameter value
 */
void SRX_DRV_HW_SetBusIdleCnt(const uint32_t instance, const uint8_t channel, const srx_diag_idle_cnt_cfg_t count);

/**
 * @brief Set calibration range
 *
 * Sets the value for the Calibration Range parameter.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] range Parameter value
 */
void SRX_DRV_HW_SetCalRng(const uint32_t instance, const uint8_t channel, const srx_diag_calib_pulse_var_cfg_t range);

/**
 * @brief Set Pause pulse check type
 *
 * Sets the value for the Pause Pulse Check paramter.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] check Parameter value
 */
void SRX_DRV_HW_SetPpChkSel(const uint32_t instance, const uint8_t channel, const srx_diag_pulse_cfg_t check);

/**
 * @brief Set pause pulse
 *
 * Enables / disables the detection of pause pulses.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] stat Parameter value
 */
void SRX_DRV_HW_SetPausePulseEnable(const uint32_t instance, const uint8_t channel, const srx_diag_pause_pulse_cfg_t stat);

/**
 * @brief Set succesive calibration check
 *
 * Sets the value of the Successive Calibration Check parameter
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] type Parameter value
 */
void SRX_DRV_HW_SetSuccCalChk(const uint32_t instance, const uint8_t channel, const srx_diag_succ_cal_check_cfg_t type);

/**
 * @brief Set slow CRC type
 *
 * Sets the value of the Slow Message CRC Type parameter
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] type Parameter value
 */
void SRX_DRV_HW_SetSlowCrcType(const uint32_t instance, const uint8_t channel, const srx_msg_crc_t type);

/**
 * @brief Set fast CRC type
 *
 * Sets the value of the Slow Message CRC Type parameter
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] type Parameter value
 */
void SRX_DRV_HW_SetFastCrcType(const uint32_t instance, const uint8_t channel, const srx_msg_crc_t type);

/**
 * @brief Enable inclusion of status in CRC
 *
 * Sets the Include Status Nibble in CRC calculation parameter.
 * This is valid only for Fast messages.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] status Parameter value
 */
void SRX_DRV_HW_SetFastCrcIncStatus(const uint32_t instance, const uint8_t channel, const bool status);

/**
 * @brief Disable Fast CRC
 *
 * Enables / Disables CRC verification for Fast messages.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] status Parameter value
 */
void SRX_DRV_HW_SetFastDisableCrc(const uint32_t instance, const uint8_t channel, const bool status);

/**
 * @brief Set Fast number of nibbles
 *
 * Configures the number of nibbles for the Fast channel.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] num Nibbles count
 */
void SRX_DRV_HW_SetFastNumNibbles(const uint32_t instance, const uint8_t channel, const uint8_t num);

/**
 * @brief Enable Fast DMA
 *
 * Enables / disables DMA transfers for the given Fast channel.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetFastDma(const uint32_t instance, const uint8_t channel, const bool enable);

/**
 * @brief Enable Slow DMA
 *
 * Enables / disables DMA transfers for the given Slow channel.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetSlowDma(const uint32_t instance, const uint8_t channel, const bool enable);

/**
 * @brief Set channel status
 *
 * Enables / disables reception for the given channels.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetChannelStatus(const uint32_t instance, const uint8_t channel, const bool enable);

/**
 * @brief Set peripheral status
 *
 * Enables / disables reception for the given channels.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetPeripheralStatus(const uint32_t instance, const bool enable);

/**
 * @brief Set event config
 *
 * Enables events received in a mask. These events will signal
 * diagnostics notifications to the application.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] events Requested events
 */
void SRX_DRV_HW_SetEventConfig(const uint32_t instance, const uint8_t channel, const srx_event_t events);

/**
 * @brief Get active events
 *
 * Returns current active diagnostics events
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @return Active events
 */
srx_event_t SRX_DRV_HW_GetActiveEvents(const uint32_t instance, const uint8_t channel);

/**
 * @brief Clear active events
 *
 * Clears the flags for the events in the given mask.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] events List of events
 */
void SRX_DRV_HW_ClearActiveEvents(const uint32_t instance, const uint8_t channel, const srx_event_t events);

/**
 * @brief Set fast interrupt status
 *
 * Enables / disables Interrupts for the Rx
 * event on the Slow channel.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetFastRxInterrupt(const uint32_t instance, const uint8_t channel, const bool enable);

/**
 * @brief Set slow interrupt status
 *
 * Enables / disables Interrupts for the Rx
 * event on the Slow channel.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[in] enable Requested status
 */
void SRX_DRV_HW_SetSlowRxInterruptStatus(const uint32_t instance, const uint8_t channel, const bool enable);

/**
 * @brief Get Fast Rx status
 *
 * Returns current Rx status for the Fast channel.
 * This will return true if there is a new message present in the buffer.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @return true if new message in buffer
 */
bool SRX_DRV_HW_GetFastRxStatus(const uint32_t instance, const uint32_t channel);

/**
 * @brief Get Fast Rx status
 *
 * Returns current Rx status for the Slow channel.
 * This will return true if there is a new message present in the buffer.
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @return true if new message in buffer
 */
bool SRX_DRV_HW_GetSlowRxStatus(const uint32_t instance, const uint32_t channel);

/**
 * @brief Get Fast raw message
 *
 * Returns current Fast message in the raw format (unformatted).
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[out] Pointer to a Raw message structure
 */
void SRX_DRV_HW_GetFastRawMsg(const uint32_t instance, const uint8_t channel, srx_raw_msg_t * rawMsg);

/**
 * @brief Get Slow raw message
 *
 * Returns current Slow message in the raw format (unformatted).
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @param[in] channel The requested channel
 * @param[out] Pointer to a Raw message structure
 */
void SRX_DRV_HW_GetSlowRawMsg(const uint32_t instance, const uint8_t channel, srx_raw_msg_t * rawMsg);

/**
 * @brief Convert Fast Raw to Normal
 *
 * Converts an unformatted (raw) Fast message into a formatted one.
 *
 * @param[out] Pointer to a Normal message structure
 * @param[in] Pointer to a Raw message structure
 */
void SRX_DRV_HW_ConvertFastRaw(srx_fast_msg_t * msg, const srx_raw_msg_t * rawMsg);

/**
 * @brief Convert Slow Raw to Normal
 *
 * Converts an unformatted (raw) Slow message into a formatted one.
 *
 * @param[out] Pointer to a Normal message structure
 * @param[in] Pointer to a Raw message structure
 */
void SRX_DRV_HW_ConvertSlowRaw(srx_slow_msg_t * msg, const srx_raw_msg_t * rawMsg);

/**
 * @brief Resets the peripheral
 *
 * Disables the peripheral and brings it's register into a default state
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 */
void SRX_DRV_HW_ResetPeripheral(const uint32_t instance);

/**
 * @brief Gets DMA start address
 *
 * Returns the start address for the Fast DMA buffer
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @return Buffer base address
 */
volatile const uint32_t * SRX_DRV_HW_GetSlowDmaRegStartAddr(const uint32_t instance);

/**
 * @brief Gets DMA start address
 *
 * Returns the start address for the Slow DMA buffer
 *
 * @param[in] baseAddr The SRX peripheral base address pointer
 * @return Buffer base address
 */
volatile const uint32_t * SRX_DRV_HW_GetFastDmaRegStartAddr(const uint32_t instance);

#if defined(__cplusplus)
}
#endif

#endif /* SRX_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
