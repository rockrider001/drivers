/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
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
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variable is required for hw access extension so it must remain global.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to store base addresses of the registers of ERM module instances,
 * the hardware abstraction uses unsigned integers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address (base address of the module/module register).
 *
 */

#ifndef ERM_HW_ACCESS_H
#define ERM_HW_ACCESS_H

/*! @file erm_hw_access.h */

#include "erm_driver.h"

/*!
 * erm_hw_access ERM Hardware Access
 * @details This section describes the programming interface of the ERM Hardware Access.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The number of bits between two different channels in configuration/status register */
#define ERM_CHANNELS_OFFSET_SIZE (4U)
/* The number of channels per configuration/status register */
#define ERM_NUM_OF_CHANNELS_PER_REG (8U)
/* Mask of the first bitfield for non correctable error in configuration/status register */
#define ERM_NCE_START_MASK       ERM_CR0_ENCIE0_MASK
/* Mask of the first bitfield for single bit correction in configuration/status register */
#define ERM_SBC_START_MASK       ERM_CR0_ESCIE0_MASK

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for ERM instances */
static ERM_Type * const s_ermBase[FEATURE_ERM_INST_COUNT] = ERM_BASE_PTRS;
/* Table of Configuration Register base addresses for each ERM instance */
static __IO uint32_t * const s_ermConfigRegBases[FEATURE_ERM_INST_COUNT][FEATURE_ERM_CRn_SRn_COUNT] = FEATURE_ERM_CRn_BASE_ADDRS;
/* Table of Status Register base addresses for each ERM instance */
static __IO uint32_t * const s_ermStatusRegBases[FEATURE_ERM_INST_COUNT][FEATURE_ERM_CRn_SRn_COUNT] = FEATURE_ERM_SRn_BASE_ADDRS;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name ERM HAL API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief HW initialization for the module
 *
 * This function initializes the module to default configuration,
 * the configuration register is reset (interrupt notifications are disabled)
 * for all channels and the status register is reset (event records are cleared).
 *
 * @param[in] instance ERM peripheral instance number
 */
void ERM_Init(uint32_t instance);

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_EnableEventInterrupt
 * Description   : Configures interrupt notification bitfields in CR register
 *
 *END**************************************************************************/
static inline void ERM_EnableEventInterrupt(uint32_t instance,
                                            uint8_t channel,
                                            erm_ecc_event_t eccEvent,
                                            bool enable)
{
    uint32_t selectedConfigRegVal = *s_ermConfigRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG];

    /* Enable interrupt notification bitfield based on ECC event type */
    if (eccEvent == ERM_EVENT_SINGLE_BIT)
    {
        if (enable)
        {
            selectedConfigRegVal |= (ERM_SBC_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
        }
        else
        {
            selectedConfigRegVal &= ~(ERM_SBC_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
        }
    }
    else if (eccEvent == ERM_EVENT_NON_CORRECTABLE)
    {
        if (enable)
        {
            selectedConfigRegVal |= (ERM_NCE_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
        }
        else
        {
            selectedConfigRegVal &= ~(ERM_NCE_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
        }
    }
    else
    {
        /* Do nothing - MISRA */
    }

    /* Write to register */
    *s_ermConfigRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG] = selectedConfigRegVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_IsEventInterruptEnabled
 * Description   : Returns whether the interrupt notification bitfield for
 * the provided channel is set in CR register
 *
 *END**************************************************************************/
static inline bool ERM_IsEventInterruptEnabled(uint32_t instance,
                                               uint8_t channel,
                                               erm_ecc_event_t eccEvent)
{
    uint32_t retVal = 0UL;
    uint32_t selectedConfigRegVal = *s_ermConfigRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG];

    /* Check if interrupt notification bitfield is set based on ECC event type */
    if (eccEvent == ERM_EVENT_SINGLE_BIT)
    {
        retVal = selectedConfigRegVal & (ERM_SBC_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
    }
    else if (eccEvent == ERM_EVENT_NON_CORRECTABLE)
    {
        retVal = selectedConfigRegVal & (ERM_NCE_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
    }
    else
    {
        /* Do nothing - MISRA */
    }

    return (retVal != 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_IsEventDetected
 * Description   : Returns whether the ECC event bitfield for the provided
 * channel is set in SR register
 *
 *END**************************************************************************/
static inline bool ERM_IsEventDetected(uint32_t instance,
                                       uint8_t channel,
                                       erm_ecc_event_t eccEvent)
{
    uint32_t retVal = 0UL;
    uint32_t selectedStatusRegVal = *s_ermStatusRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG];

    /* Check if ECC event occurred is set based on ECC event type */
    if (eccEvent == ERM_EVENT_SINGLE_BIT)
    {
        retVal = selectedStatusRegVal & (ERM_SBC_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
    }
    else if (eccEvent == ERM_EVENT_NON_CORRECTABLE)
    {
        retVal = selectedStatusRegVal & (ERM_NCE_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE));
    }
    else
    {
        /* Do nothing - MISRA */
    }

    return (retVal != 0UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_ClearEventSingle
 * Description   : Clears event record bitfield for single bit correction events
 * in SR register
 *
 *END**************************************************************************/
static inline void ERM_ClearEventSingle(uint32_t instance,
                                        uint8_t channel)
{
    *s_ermStatusRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG] = ERM_SBC_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE);
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    (void)(*s_ermStatusRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG]);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_ClearEventDouble
 * Description   : Clears event record bitfield for non correctable error events
 * in SR register
 *
 *END**************************************************************************/
static inline void ERM_ClearEventDouble(uint32_t instance,
                                        uint8_t channel)
{
    *s_ermStatusRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG] = ERM_NCE_START_MASK >> ((channel % ERM_NUM_OF_CHANNELS_PER_REG) * ERM_CHANNELS_OFFSET_SIZE);
#ifdef ERRATA_E9005
    /* Read-after-write sequence to guarantee required serialization of memory operations */
    (void)(*s_ermStatusRegBases[instance][channel / ERM_NUM_OF_CHANNELS_PER_REG]);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_GetLastErrorAddress
 * Description   : Returns detected error address information from EAR register
 *
 *END**************************************************************************/
static inline uint32_t ERM_GetLastErrorAddress(const ERM_Type * const base,
                                               uint8_t channel)
{
    #if(ERM_EARn_COUNT >= 2u)
        return base->EARn[channel].EAR;
    #else
        return base->EARn[channel];
    #endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ERM_GetLastErrorBitPosition
 * Description   : Returns detected error bit information from SYN register
 *
 *END**************************************************************************/
#ifdef FEATURE_ERM_ECC_BIT_POS
static inline uint32_t ERM_GetLastErrorBitPosition(const ERM_Type * const base,
                                                                uint8_t channel)
{
    return (((base->EARn[channel].SYN) & ERM_SYN_SYNDROME_MASK) >> ERM_SYN_SYNDROME_SHIFT);
}
#endif /* FEATURE_ERM_ECC_BIT_POS */
/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* ERM_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
