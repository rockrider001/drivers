/*
 * Copyright 2018-2019 NXP
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

#ifndef EQADC_HW_ACCESS_H
#define EQADC_HW_ACCESS_H

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
 */

#include <stdint.h>
#include <stdbool.h>

#include "device_registers.h"

/*!
 * @defgroup eqadc_group EQADC Hardware Access
 * @brief EQADC Group
 * @{
 */

/*******************************************************************************
 * Variables declarations
 ******************************************************************************/
extern __IO uint32_t * const CFCR[EQADC_INSTANCE_COUNT][3u];

extern __IO uint32_t * const IDCR[EQADC_INSTANCE_COUNT][3u];

extern __I uint32_t * const CFxR[EQADC_INSTANCE_COUNT][EQADC_CFPR_COUNT];

extern __I uint32_t * const RFxR[EQADC_INSTANCE_COUNT][EQADC_RFPR_COUNT];

extern __IO uint32_t * const CFTCR[EQADC_INSTANCE_COUNT][3u];

extern __IO uint32_t * const SIU_TBG_CR[EQADC_INSTANCE_COUNT];

extern const uint32_t TCB_MASK[EQADC_CFPR_COUNT];

extern const uint8_t TCB_SHIFT[EQADC_CFPR_COUNT];


/*******************************************************************************
 * Function definitions
 ******************************************************************************/

 #if defined (__cplusplus)
extern "C" {
#endif



/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoOpMode
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoOpMode(const uint32_t instance, const uint8_t cfifoIdx, const uint32_t opMode)
{
    uint8_t cfcrIdx = cfifoIdx / 2u;

    if((cfifoIdx % 2u) == 0u)
    {
        *(CFCR[instance][cfcrIdx]) &= ~EQADC_CFCR0_MODE0_MASK;
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_MODE0(opMode);
    }
    else
    {
        *(CFCR[instance][cfcrIdx]) &= ~EQADC_CFCR0_MODE1_MASK;
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_MODE1(opMode);
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetCfifoOpMode
 * Description   :
 *
 *END*************************************************************************/
static inline uint32_t EQADC_GetCfifoOpMode(const uint32_t instance, const uint8_t cfifoIdx)
{
    uint8_t cfcrIdx = cfifoIdx / 2u;
    uint32_t opMode;

    if((cfifoIdx % 2u) == 0u)
    {
        opMode = (*(CFCR[instance][cfcrIdx]) & EQADC_CFCR0_MODE0_MASK) >> EQADC_CFCR0_MODE0_SHIFT;
    }
    else
    {
        opMode = (*(CFCR[instance][cfcrIdx]) & EQADC_CFCR0_MODE1_MASK) >> EQADC_CFCR0_MODE1_SHIFT;
    }

    return opMode;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoInv
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoInv(const uint32_t instance, const uint8_t cfifoIdx)
{
    uint8_t cfcrIdx = cfifoIdx / 2u;

    if((cfifoIdx % 2u) == 0u)
    {
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_CFINV0_MASK;
    }
    else
    {
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_CFINV1_MASK;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoSSE
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoSSE(const uint32_t instance, const uint32_t cfifoIdx)
{
    uint32_t cfcrIdx = cfifoIdx / 2u;

    if((cfifoIdx % 2u) == 0u)
    {
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_SSE0_MASK;
    }
    else
    {
        *(CFCR[instance][cfcrIdx]) |= EQADC_CFCR0_SSE1_MASK;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoSSEPair
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoSSEPair(const uint32_t instance, const uint32_t cfifoIdx)
{
    uint32_t cfcrIdx = cfifoIdx / 2u;

    *(CFCR[instance][cfcrIdx]) |= (EQADC_CFCR0_SSE0_MASK | EQADC_CFCR0_SSE1_MASK);
}



/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetCfifoCFS
 * Description   :
 *
 *END*************************************************************************/
static inline uint32_t EQADC_GetCfifoCFS(const EQADC_Type * const base, const uint32_t cfifoIdx)
{
    uint32_t cfs = 0u;

    switch(cfifoIdx)
    {
        case 0u:
            cfs = (base->CFSR & EQADC_CFSR_CFS0_MASK) >> EQADC_CFSR_CFS0_SHIFT;
            break;
        case 1u:
            cfs = (base->CFSR & EQADC_CFSR_CFS1_MASK) >> EQADC_CFSR_CFS1_SHIFT;
            break;
        case 2u:
            cfs = (base->CFSR & EQADC_CFSR_CFS2_MASK) >> EQADC_CFSR_CFS2_SHIFT;
            break;
        case 3u:
            cfs = (base->CFSR & EQADC_CFSR_CFS3_MASK) >> EQADC_CFSR_CFS3_SHIFT;
            break;
        case 4u:
            cfs = (base->CFSR & EQADC_CFSR_CFS4_MASK) >> EQADC_CFSR_CFS4_SHIFT;
            break;
        case 5u:
            cfs = (base->CFSR & EQADC_CFSR_CFS5_MASK) >> EQADC_CFSR_CFS5_SHIFT;
            break;
        default:
            /* no operation - invalid value for cfifoIdx */
            break;
    }

    return cfs;
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_ClearFifoIDCR
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_ClearFifoIDCR(const uint32_t instance, const uint32_t fifoIdx, const uint16_t mask)
{
    uint32_t idcrIdx = fifoIdx / 2u;

    if((fifoIdx % 2u) == 0u)
    {
        *(IDCR[instance][idcrIdx]) &= ~((uint32_t)mask << 16u);
    }
    else
    {
        *(IDCR[instance][idcrIdx]) &= ~((uint32_t)mask);
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetFifoIDCR
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetFifoIDCR(const uint32_t instance, const uint32_t fifoIdx, const uint16_t mask)
{
    uint32_t idcrIdx = fifoIdx / 2u;

    if((fifoIdx % 2u) == 0u)
    {
        *(IDCR[instance][idcrIdx]) |= ((uint32_t)mask << 16u);
    }
    else
    {
        *(IDCR[instance][idcrIdx]) |= (uint32_t)mask;
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetFifoIDCR
 * Description   :
 *
 *END*************************************************************************/
static inline uint16_t EQADC_GetFifoIDCR(const uint32_t instance, const uint32_t fifoIdx)
{
    uint32_t idcrIdx = fifoIdx / 2u;
    uint16_t res = 0u;

    if((fifoIdx % 2u) == 0u)
    {
        res = (uint16_t)((*(IDCR[instance][idcrIdx]) >> 16u) & 0xFFFFu);
    }
    else
    {
        res = (uint16_t)(*(IDCR[instance][idcrIdx]) & 0xFFFFu);
    }

    return res;
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoTrig
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoTrig(const uint32_t instance, const uint32_t cfifoIdx, const uint8_t value)
{
    static __IO uint32_t * const ISEL_CFIFO_01[EQADC_INSTANCE_COUNT] = {&(SIU->ISEL5), &(SIU->ISEL7)};
    static __IO uint32_t * const ISEL_CFIFO_2345[EQADC_INSTANCE_COUNT] = {&(SIU->ISEL4), &(SIU->ISEL6)};

    switch (cfifoIdx) {
        case 0u:
            *(ISEL_CFIFO_01[instance]) &= ~SIU_ISEL5_CTSEL0_A_MASK;
            *(ISEL_CFIFO_01[instance]) |= SIU_ISEL5_CTSEL0_A(value);
            break;
        case 1u:
            *(ISEL_CFIFO_01[instance]) &= ~SIU_ISEL5_CTSEL1_A_MASK;
            *(ISEL_CFIFO_01[instance]) |= SIU_ISEL5_CTSEL1_A(value);
            break;
        case 2u:
            *(ISEL_CFIFO_2345[instance]) &= ~SIU_ISEL4_CTSEL2_A_MASK;
            *(ISEL_CFIFO_2345[instance]) |= SIU_ISEL4_CTSEL2_A(value);
            break;
        case 3u:
            *(ISEL_CFIFO_2345[instance]) &= ~SIU_ISEL4_CTSEL3_A_MASK;
            *(ISEL_CFIFO_2345[instance]) |= SIU_ISEL4_CTSEL3_A(value);
            break;
        case 4u:
            *(ISEL_CFIFO_2345[instance]) &= ~SIU_ISEL4_CTSEL4_A_MASK;
            *(ISEL_CFIFO_2345[instance]) |= SIU_ISEL4_CTSEL4_A(value);
            break;
        case 5u:
            *(ISEL_CFIFO_2345[instance]) &= ~SIU_ISEL4_CTSEL5_A_MASK;
            *(ISEL_CFIFO_2345[instance]) |= SIU_ISEL4_CTSEL5_A(value);
            break;
        default:
            /* no op */
            break;
    }
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetAdvanceTrig
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetAdvanceTrig(const uint32_t instance, const uint8_t value)
{
    SIU_Type * const siuBase = SIU;

    switch(instance)
    {
        case 0u:
            siuBase->ISEL9 &= ~SIU_ISEL9_EETSEL0ADV_A_MASK;
            siuBase->ISEL9 |= SIU_ISEL9_EETSEL0ADV_A(value);
            break;
        case 1u:
            siuBase->ISEL9 &= ~SIU_ISEL9_EETSEL0ADV_B_MASK;
            siuBase->ISEL9 |= SIU_ISEL9_EETSEL0ADV_B(value);
            break;
        default:
            /*no op */
            break;
    }
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetCfifoCFxR
 * Description   :
 *
 *END*************************************************************************/
static inline uint32_t EQADC_GetCfifoCFxR(const uint32_t instance, const uint32_t cfifoIdx, const uint32_t entryIdx)
{
    return (CFxR[instance][cfifoIdx][entryIdx] & EQADC_CF0R_CFIFO0_DATAW_MASK);
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetRfifoRFxR
 * Description   :
 *
 *END*************************************************************************/
static inline uint16_t EQADC_GetRfifoRFxR(const uint32_t instance, const uint32_t rfifoIdx, const uint32_t entryIdx)
{
    return (uint16_t) (RFxR[instance][rfifoIdx][entryIdx] & EQADC_RF0R_RFIFO0_DATAW_MASK);
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetCfifoTC_CF
 * Description   :
 *
 *END*************************************************************************/
static inline uint16_t EQADC_GetCfifoTC_CF(const uint32_t instance, const uint32_t entryIdx)
{
    uint32_t cftcrIdx = entryIdx / 2u;
    uint32_t tc_cf;
    uint16_t numTC;

    if((entryIdx % 2u) == 0u)
    {
        tc_cf = (*(CFTCR[instance][cftcrIdx]) & EQADC_CFTCR0_TC_CF0_MASK) >> EQADC_CFTCR0_TC_CF0_SHIFT;
    }
    else
    {
        tc_cf = (*(CFTCR[instance][cftcrIdx]) & EQADC_CFTCR0_TC_CF1_MASK) >> EQADC_CFTCR0_TC_CF1_SHIFT;
    }
    numTC = (uint16_t) tc_cf;

    return numTC;
}


/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_SetCfifoTC_CF
 * Description   :
 *
 *END*************************************************************************/
static inline void EQADC_SetCfifoTC_CF(const uint32_t instance, const uint32_t entryIdx, const uint16_t value)
{
    uint32_t cftcrIdx = entryIdx / 2u;

    if((entryIdx % 2u) == 0u)
    {
        *(CFTCR[instance][cftcrIdx]) &= ~EQADC_CFTCR0_TC_CF0_MASK;
        *(CFTCR[instance][cftcrIdx]) |= EQADC_CFTCR0_TC_CF0(value);
    }
    else
    {
        *(CFTCR[instance][cftcrIdx]) &= ~EQADC_CFTCR0_TC_CF1_MASK;
        *(CFTCR[instance][cftcrIdx]) |= EQADC_CFTCR0_TC_CF1(value);
    }
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetCfifoTCB
 * Description   :
 *
 *END*************************************************************************/
static inline uint8_t EQADC_GetCfifoTCB(const EQADC_Type * const base,
                                        const uint32_t cfifoIdx,
                                        const uint32_t adcIdx)
{
    uint32_t cfssr = 0u;
    uint32_t tcb = 0u;

    switch(adcIdx)
    {
        case 0u:
            cfssr = base->CFSSR0;
            break;
        case 1u:
            cfssr = base->CFSSR1;
            break;
        default:
            DEV_ASSERT(false); /* invalid adcIdx value */
            break;
    }

    tcb = (cfssr & TCB_MASK[cfifoIdx]) >> TCB_SHIFT[cfifoIdx];

    return ((uint8_t) tcb);
}




#if defined (__cplusplus)
}
#endif

/*! @} */

#endif /* EQADC_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
