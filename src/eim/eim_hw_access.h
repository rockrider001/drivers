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
 * @file eim_hw_access.h
 */

#ifndef EIM_HW_ACCESS_H
#define EIM_HW_ACCESS_H

#include "eim_driver.h"

/*!
 * @brief Error Injection Module Hardware Access.
 * EIM HW ACCESS provides low level APIs for reading and writing register bit-fields
 * belonging to the EIM module.
 * @{
 */

/*******************************************************************************
 * Definitions
 *****************************************************************************/

/* The position of the most significant bit in Error Injection Channel Enable register */
#define POS_MSB_EIM_EICHEN    (31U)

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name EIM HW ACCESS API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Resets the enable channel and channel descriptors registers.
 *
 * This function disables all channels and clears all check-bit mask
 * and data mask bitfields.
 *
 * @param[in] base EIM peripheral base address
 */
void EIM_InitChannel(EIM_Type * const base);

/*!
 * @brief Configures the check bit mask register bitfields.
 *
 * This function configures the check bit mask bitfield of the channel given as argument.
 *
 * @param[in] base EIM peripheral base address
 * @param[in] channel EIM channel number
 * @param[in] checkBitMask Check-bit mask bitfield value
 */
void EIM_SetCheckBitMask(EIM_Type * const base,
                                    uint8_t channel,
                                    uint32_t checkBitMask);

/*!
 * @brief Returns the value of check bit mask bitfield.
 *
 * This function returns the check bit mask bitfield value of the channel given as argument.
 *
 * @param[in] base EIM peripheral base address
 * @param[in] channel EIM channel number
 * @return Check-bit mask bitfield value
 */
uint32_t EIM_GetCheckBitMask(const EIM_Type * const base,
                                            uint8_t channel);

/*!
 * @brief Configures the data mask(s) register bitfields.
 *
 * This function configures the data mask bitfield of the channel given as argument.
 *
 * @param[in] base EIM peripheral base address
 * @param[in] channel EIM channel number
 * @param[in] dataMask Data mask bitfield value
 * @param[in] dataMask Data mask1 bitfield value
 */
void EIM_SetDataMask(EIM_Type * const base,
                                uint8_t channel,
                                uint32_t dataMask,
                                uint32_t dataMask1);

/*!
 * @brief Returns the value of data mask(s) bitfield.
 *
 * This function returns the data mask(s) bitfield value of the channel given as argument.
 *
 * @param[in] base EIM peripheral base address
 * @param[in] channel EIM channel number
 * @param[out] dataMask Data mask bitfield value
 * @param[out] dataMask1 Data mask1 bitfield value
 */
void EIM_GetDataMask(const EIM_Type * const base,
                                    uint8_t channel,
                                    uint32_t *dataMask,
                                    uint32_t *dataMask1);

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_Enable
 * Description   : Sets Global Error Injection Enable bitfield to enable the module.
 *
 *END**************************************************************************/
static inline void EIM_Enable(EIM_Type * const base)
{
    base->EIMCR |= EIM_EIMCR_GEIEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_Disable
 * Description   : Clears Global Error Injection Enable bitfield to disable the module.
 *
 *END**************************************************************************/
static inline void EIM_Disable(EIM_Type * const base)
{
    base->EIMCR &= ~EIM_EIMCR_GEIEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_EnableChannelCmd
 * Description   : Sets/Clears Error Injection Channel Enable bitfield for a channel.
 *
 *END**************************************************************************/
static inline void EIM_EnableChannelCmd(EIM_Type * const base,
                                        uint8_t channel,
                                        bool enable)
{
    uint32_t temp;

    temp = base->EICHEN;
    temp &= ~(1UL << (POS_MSB_EIM_EICHEN - channel));
    temp |= (enable ? 1UL : 0UL) << (POS_MSB_EIM_EICHEN - channel);
    base->EICHEN = temp;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EIM_EnableChannelCmd
 * Description   : Returns whether the Error Injection Channel Enable bitfield
 * is set for a channel.
 *
 *END**************************************************************************/
static inline bool EIM_IsChannelEnabled(const EIM_Type * const base,
                                        uint8_t channel)
{
    return ((base->EICHEN & (1UL << (POS_MSB_EIM_EICHEN - channel))) != 0UL);
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* EIM_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
