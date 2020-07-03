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

#ifndef SRX_IRQ_H
#define SRX_IRQ_H

#include <stddef.h>
#include "device_registers.h"

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * IRQ IDs
 */
typedef enum {
    SRX_IRQ_FAST = 0,
    SRX_IRQ_SLOW = 1,
    SRX_IRQ_ERROR = 2
} srx_interrupt_id_t;

/**
 * @brief Enable interrupt by mapping
 *
 * Platform specific
 *
 * @param[in] instance Peripheral instance
 * @param[in] channel Communication channel
 * @param[in] irqMap Interrupt map
 */
void SRX_DRV_IRQ_EnableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id);

/**
 * @brief Disable interrupt by mapping
 *
 * Platform specific
 *
 * @param[in] instance Peripheral instance
 * @param[in] channel Communication channel
 * @param[in] irqMap Interrupt map
 */
void SRX_DRV_IRQ_DisableIRQ(const uint32_t instance, const uint32_t channel, const srx_interrupt_id_t id);

/**
 * @brief Driver side interrupt
 *
 * Fast Rx Event.
 * Gets called from the low level handler
 * with instance and channel as parameter
 *
 * @param[in] instance Peripheral instance
 * @param[in] channel Communication channel
 */
void SRX_DRV_IRQ_FastHandler(const uint32_t instance, const uint32_t channel);

/**
 * @brief Driver side interrupt
 *
 * Slow Rx Event.
 * Gets called from the low level handler
 * with instance and channel as parameter
 *
 * @param[in] instance Peripheral instance
 * @param[in] channel Communication channel
 */
void SRX_DRV_IRQ_SlowHandler(const uint32_t instance, const uint32_t channel);

/**
 * @brief Driver side interrupt
 *
 * Rx Error Event.
 * Gets called from the low level handler
 * with instance and channel as parameter
 *
 * @param[in] instance Peripheral instance
 * @param[in] channel Communication channel
 */
void SRX_DRV_IRQ_RxErrHandler(const uint32_t instance, const uint32_t channel);

#if defined(__cplusplus)
}
#endif

#endif /* SRX_IRQ_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
