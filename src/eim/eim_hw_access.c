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

/*!
 * @file eim_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 */

#include "eim_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of channel types */
static const uint8_t s_eimChannelType[EIM_EICHDn_COUNT] = FEATURE_EIM_CH_TYPE;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_InitChannel
 * Description   : Resets the enable channel and channel descriptors registers.
 *
 *END**************************************************************************/
void EIM_InitChannel(EIM_Type * const base)
{
    uint8_t channel;

    for(channel = 0; channel < EIM_EICHDn_COUNT; channel++)
    {
        EIM_EnableChannelCmd(base, channel, false);
        EIM_SetCheckBitMask(base, channel, 0U);
        EIM_SetDataMask(base, channel, 0U, 0U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_SetCheckBitMask
 * Description   : Configures the check bit mask register bitfields.
 *
 *END**************************************************************************/
void EIM_SetCheckBitMask(EIM_Type * const base,
                                    uint8_t channel,
                                    uint32_t checkBitMask)
{
    switch (s_eimChannelType[channel])
    {
    #ifdef FEATURE_EIM_CH_TYPE_0
        case FEATURE_EIM_CH_TYPE_0:
        base->EICHDn[channel].WORD0 = FEATURE_EIM_CH_TYPE_0_WORD0_WRITE(checkBitMask);
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_1
        case FEATURE_EIM_CH_TYPE_1:
        base->EICHDn[channel].WORD0 = FEATURE_EIM_CH_TYPE_1_WORD0_WRITE(checkBitMask);
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_2
        case FEATURE_EIM_CH_TYPE_2:
        base->EICHDn[channel].WORD0 = FEATURE_EIM_CH_TYPE_2_WORD0_WRITE(checkBitMask);
        break;
    #endif
        default:
        DEV_ASSERT(false);
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_GetCheckBitMask
 * Description   : Returns the value of check bit mask bitfield.
 *
 *END**************************************************************************/
uint32_t EIM_GetCheckBitMask(const EIM_Type * const base,
                                            uint8_t channel)
{
    uint32_t tmpCheckBitMask = 0UL;
    switch (s_eimChannelType[channel])
    {
    #ifdef FEATURE_EIM_CH_TYPE_0
        case FEATURE_EIM_CH_TYPE_0:
        tmpCheckBitMask = FEATURE_EIM_CH_TYPE_0_WORD0_READ(base->EICHDn[channel].WORD0);
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_1
        case FEATURE_EIM_CH_TYPE_1:
        tmpCheckBitMask = FEATURE_EIM_CH_TYPE_1_WORD0_READ(base->EICHDn[channel].WORD0);
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_2
        case FEATURE_EIM_CH_TYPE_2:
        tmpCheckBitMask = FEATURE_EIM_CH_TYPE_2_WORD0_READ(base->EICHDn[channel].WORD0);
        break;
    #endif
        default:
        DEV_ASSERT(false);
        break;
    }
    return tmpCheckBitMask;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_SetDataMask
 * Description   : Configures the data mask(s) register bitfields.
 *
 *END**************************************************************************/
void EIM_SetDataMask(EIM_Type * const base,
                                uint8_t channel,
                                uint32_t dataMask,
                                uint32_t dataMask1)
{
    switch (s_eimChannelType[channel])
    {
    #ifdef FEATURE_EIM_CH_TYPE_0
        case FEATURE_EIM_CH_TYPE_0:
        base->EICHDn[channel].WORD1 = FEATURE_EIM_CH_TYPE_0_WORD1_WRITE(dataMask);
        #if (FEATURE_EIM_CH_TYPE_0_NUM_OF_DATA_WORDS > 1U)
        base->EICHDn[channel].WORD2 = FEATURE_EIM_CH_TYPE_0_WORD2_WRITE(dataMask1);
        #endif
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_1
        case FEATURE_EIM_CH_TYPE_1:
        base->EICHDn[channel].WORD1 = FEATURE_EIM_CH_TYPE_1_WORD1_WRITE(dataMask);
        #if (FEATURE_EIM_CH_TYPE_1_NUM_OF_DATA_WORDS > 1U)
        base->EICHDn[channel].WORD2 = FEATURE_EIM_CH_TYPE_1_WORD2_WRITE(dataMask1);
        #endif
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_2
        case FEATURE_EIM_CH_TYPE_2:
        base->EICHDn[channel].WORD1 = FEATURE_EIM_CH_TYPE_2_WORD1_WRITE(dataMask);
        #if (FEATURE_EIM_CH_TYPE_2_NUM_OF_DATA_WORDS > 1U)
        base->EICHDn[channel].WORD2 = FEATURE_EIM_CH_TYPE_2_WORD2_WRITE(dataMask1);
        #endif
        break;
    #endif
        default:
        DEV_ASSERT(false);
        break;
    }
    /* Avoid compiler warnings */
    (void)dataMask;
    (void)dataMask1;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_GetDataMask
 * Description   : Returns the value of data mask(s) bitfield.
 *
 *END**************************************************************************/
void EIM_GetDataMask(const EIM_Type * const base,
                                    uint8_t channel,
                                    uint32_t *dataMask,
                                    uint32_t *dataMask1)
{
    *dataMask = 0UL;
    *dataMask1 = 0UL;
    switch (s_eimChannelType[channel])
    {
    #ifdef FEATURE_EIM_CH_TYPE_0
        case FEATURE_EIM_CH_TYPE_0:
        *dataMask = FEATURE_EIM_CH_TYPE_0_WORD1_READ(base->EICHDn[channel].WORD1);
        #if (FEATURE_EIM_CH_TYPE_0_NUM_OF_DATA_WORDS > 1U)
        *dataMask1 = FEATURE_EIM_CH_TYPE_0_WORD2_READ(base->EICHDn[channel].WORD2);
        #endif
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_1
        case FEATURE_EIM_CH_TYPE_1:
        *dataMask = FEATURE_EIM_CH_TYPE_1_WORD1_READ(base->EICHDn[channel].WORD1);
        #if (FEATURE_EIM_CH_TYPE_1_NUM_OF_DATA_WORDS > 1U)
        *dataMask1 = FEATURE_EIM_CH_TYPE_1_WORD2_READ(base->EICHDn[channel].WORD2);
        #endif
        break;
    #endif
    #ifdef FEATURE_EIM_CH_TYPE_2
        case FEATURE_EIM_CH_TYPE_2:
        *dataMask = FEATURE_EIM_CH_TYPE_2_WORD1_READ(base->EICHDn[channel].WORD1);
        #if (FEATURE_EIM_CH_TYPE_2_NUM_OF_DATA_WORDS > 1U)
        *dataMask1 = FEATURE_EIM_CH_TYPE_2_WORD2_READ(base->EICHDn[channel].WORD2);
        #endif
        break;
    #endif
        default:
        DEV_ASSERT(false);
        break;
    }
}

/*******************************************************************************
* EOF
******************************************************************************/
