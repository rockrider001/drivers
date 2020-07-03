/*
 * Copyright 2018 NXP
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

/**
 * @page misra_violations MISRA-C:2012 violations
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
 * Violates MISRA 2012 Required Rule 13.1, side effect in initializer
 * Array is initialized with addresses to volatile memory, pointing to registers on which read operations have no side effects.
 *
 */

#include "eqadc_hw_access.h"
#include "device_registers.h"

/*!
 * @defgroup eqadc_group EQADC Hardware Access
 * @brief EQADC Group
 * @{
 */

/*******************************************************************************
 * Variables definitions
 ******************************************************************************/

__IO uint32_t * const CFCR[EQADC_INSTANCE_COUNT][3u] = {
    {&(EQADC_0->CFCR0), &(EQADC_0->CFCR1), &(EQADC_0->CFCR2)},
    {&(EQADC_1->CFCR0), &(EQADC_1->CFCR1), &(EQADC_1->CFCR2)}
};

__IO uint32_t * const IDCR[EQADC_INSTANCE_COUNT][3u] = {
    {&(EQADC_0->IDCR0), &(EQADC_0->IDCR1), &(EQADC_0->IDCR2)},
    {&(EQADC_1->IDCR0), &(EQADC_1->IDCR1), &(EQADC_1->IDCR2)}
};

__I uint32_t * const CFxR[EQADC_INSTANCE_COUNT][EQADC_CFPR_COUNT] = {
    {&(EQADC_0->CF0R[0]), &(EQADC_0->CF1R[0]), &(EQADC_0->CF2R[0]), &(EQADC_0->CF3R[0]), &(EQADC_0->CF4R[0]), &(EQADC_0->CF5R[0])},
    {&(EQADC_1->CF0R[0]), &(EQADC_1->CF1R[0]), &(EQADC_1->CF2R[0]), &(EQADC_1->CF3R[0]), &(EQADC_1->CF4R[0]), &(EQADC_1->CF5R[0])}
};

__I uint32_t * const RFxR[EQADC_INSTANCE_COUNT][EQADC_RFPR_COUNT] = {
    {&(EQADC_0->RF0R[0]), &(EQADC_0->RF1R[0]), &(EQADC_0->RF2R[0]), &(EQADC_0->RF3R[0]), &(EQADC_0->RF4R[0]), &(EQADC_0->RF5R[0])},
    {&(EQADC_1->RF0R[0]), &(EQADC_1->RF1R[0]), &(EQADC_1->RF2R[0]), &(EQADC_1->RF3R[0]), &(EQADC_1->RF4R[0]), &(EQADC_1->RF5R[0])}
};

__IO uint32_t * const CFTCR[EQADC_INSTANCE_COUNT][3u] = {
    {&(EQADC_0->CFTCR0), &(EQADC_0->CFTCR1), &(EQADC_0->CFTCR2)},
    {&(EQADC_1->CFTCR0), &(EQADC_1->CFTCR1), &(EQADC_1->CFTCR2)}
};

__IO uint32_t * const SIU_TBG_CR[EQADC_INSTANCE_COUNT] = {SIU->TBG_CR_A, SIU->TBG_CR_B};

const uint32_t TCB_MASK[EQADC_CFPR_COUNT] = {
    EQADC_CFSSR0_CFS0_TCB0_MASK,
    EQADC_CFSSR0_CFS1_TCB0_MASK,
    EQADC_CFSSR0_CFS2_TCB0_MASK,
    EQADC_CFSSR0_CFS3_TCB0_MASK,
    EQADC_CFSSR0_CFS4_TCB0_MASK,
    EQADC_CFSSR0_CFS5_TCB0_MASK
};

const uint8_t TCB_SHIFT[EQADC_CFPR_COUNT] = {
    EQADC_CFSSR0_CFS0_TCB0_SHIFT,
    EQADC_CFSSR0_CFS1_TCB0_SHIFT,
    EQADC_CFSSR0_CFS2_TCB0_SHIFT,
    EQADC_CFSSR0_CFS3_TCB0_SHIFT,
    EQADC_CFSSR0_CFS4_TCB0_SHIFT,
    EQADC_CFSSR0_CFS5_TCB0_SHIFT
};

/*! @} */

/*******************************************************************************
 * EOF
 ******************************************************************************/
