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

#ifndef SIPI_IRQ_H
#define SIPI_IRQ_H

#include "sipi_hw_access.h"

/* Array holding SIPI base pointers */
extern SIPI_Type * s_sipi_instances[SIPI_INSTANCE_COUNT];
/* SIPI Interrupt vectors */
extern IRQn_Type sipi_irqs[SIPI_IRQ_COUNT];

/*******************************************************************************
 * Driver handlers prototypes
 ******************************************************************************/
void SIPI_CrcErrHandler(uint8_t instance);
void SIPI_MaxCountReachedHandler(uint8_t instance);
void SIPI_ChnTimeoutErrHandler(uint8_t instance, uint8_t channel);
void SIPI_ChnTransIdErrHandler(uint8_t instance, uint8_t channel);
void SIPI_ChnAckErrHandler(uint8_t instance, uint8_t channel);
void SIPI_ChnReadAnswerHandler(uint8_t instance, uint8_t channel);
void SIPI_ChnAckHandler(uint8_t instance, uint8_t channel);
void SIPI_ChnTriggerHandler(uint8_t instance, uint8_t channel);

/*******************************************************************************
 *  Default interrupt handlers signatures
 ******************************************************************************/
void SIPI0_ErrorIrqHandler(void);
void SIPI0_CrcErrorIrqHandler(void);
void SIPI0_Ch0ResponseOrAckIrqHandler(void);
void SIPI0_Ch1ResponseOrAckIrqHandler(void);
void SIPI0_Ch2ResponseOrAckIrqHandler(void);
void SIPI0_Ch3ResponseOrAckIrqHandler(void);
void SIPI0_TriggerOrMaxCountReachedIrqHandler(void);
#ifdef SIPI_ORED_INT_LINES
void SIPI0_IRQHandler(void);
#endif

#endif /* SIPI_IRQ_H */
