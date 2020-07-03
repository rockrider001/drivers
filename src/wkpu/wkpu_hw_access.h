/*
 * Copyright 2017 NXP
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

#ifndef WKPU_HW_ACCESS_H
#define WKPU_HW_ACCESS_H

/*! @file wkpu_hw_access.h */

#include "wkpu_driver.h"

/*!
 * wkpu_hw_access WKPU Hardware Access
 * @details This section describes the programming interface of the WKPU Hardware Access.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name WKPU Hardware Access
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Sets NMI destination source
 *
 * This function sets NMI destination source
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] value The destination source
 */
static inline void WKPU_SetNMIDestinationSrc(WKPU_Type * const base,
                                             uint8_t position,
                                             uint8_t value)
{
    uint32_t tmp = base->NCR;
    tmp &= ~(WKPU_NCR_NDSS0_MASK >> position);
    tmp |= WKPU_NCR_NDSS0(value) >> position;

    base->NCR = tmp;
}

/*!
 * @brief Sets NMI wakeup request
 *
 * This function sets NMI wakeup request
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] enable Enable or disable NMI wakeup request
 */
static inline void WKPU_SetNMIWakeupRequest(WKPU_Type * const base,
                                            uint8_t position,
                                            bool enable)
{
    uint32_t tmp = base->NCR;
    tmp &= ~(WKPU_NCR_NWRE0_MASK >> position);
    tmp |= WKPU_NCR_NWRE0(enable ? 1UL : 0UL) >> position;

    base->NCR = tmp;
}

/*!
 * @brief Sets NMI filter
 *
 * This function sets NMI filter
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] enable Enable or disable NMI filter
 */
static inline void WKPU_SetNMIFilter(WKPU_Type * const base,
                                     uint8_t position,
                                     bool enable)
{
    uint32_t tmp = base->NCR;
    tmp &= ~(WKPU_NCR_NFE0_MASK >> position);
    tmp |= WKPU_NCR_NFE0(enable ? 1UL : 0UL) >> position;

    base->NCR = tmp;
}

/*!
 * @brief Sets NMI rising edge
 *
 * This function sets NMI rising edge
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] value The value of falling edge
 */
static inline void WKPU_SetNMIRisingEdge(WKPU_Type * const base,
                                         uint8_t position,
                                         uint8_t value)
{
    if (value != 0U)
    {
        base->NCR |= WKPU_NCR_NREE0_MASK >> position;
    }
    else
    {
        base->NCR &= ~(WKPU_NCR_NREE0_MASK >> position);
    }
}

/*!
 * @brief Sets NMI falling edge
 *
 * This function sets NMI falling edge
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] value The value of falling edge
 */
static inline void WKPU_SetNMIFallingEdge(WKPU_Type * const base,
                                          uint8_t position,
                                          uint8_t value)
{
    if (value != 0U)
    {
        base->NCR |= WKPU_NCR_NFEE0_MASK >> position;
    }
    else
    {
        base->NCR &= ~(WKPU_NCR_NFEE0_MASK >> position);
    }
}

/*!
 * @brief Sets NMI/Reset configuration lock
 *
 * This function sets NMI/Reset configuration lock
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @param[in] enable Enable or disable NMI falling edge
 */
static inline void WKPU_SetNMIConfigLock(WKPU_Type * const base,
                                         uint8_t position,
                                         bool enable)
{
    base->NCR |= WKPU_NCR_NLOCK0(enable ? 1UL : 0UL) >> position;
}

/*!
 * @brief Gets NMI/Reset configuration lock
 *
 * This function gets NMI/Reset configuration lock
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] position The position of source
 * @return Status of configuration lock
 */
static inline bool WKPU_IsNMIConfigLock(const WKPU_Type * base,
                                        uint8_t position)
{
    return (base->NCR & (WKPU_NCR_NLOCK0_MASK >> position)) != 0U;
}

/*!
 * @brief Clears status flag
 *
 * This function clears status flag
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 */
static inline void WKPU_ClearStatusFlag(WKPU_Type * const base,
                                        uint32_t channelMask)
{
    /* Clear status flag */
    base->NSR = channelMask;
}

#ifdef FEATURE_WKPU_SUPPORT_INTERRUPT_REQUEST
/*!
 * @brief Enables or disables interrupt request
 *
 * This function enables or disables interrupt request
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables interrupt request
 */
static inline void WKPU_EnableInterruptRequest(WKPU_Type * const base,
                                               uint32_t channelMask,
                                               bool enable)
{
    /* Enable interrupt request */
    if (enable)
    {
        base->IRER |= channelMask;
    }
    /* Disable interrupt request */
    else
    {
        base->IRER &= ~channelMask;
    }
}

/*!
 * @brief Enables or disables wakeup request
 *
 * This function enables or disables wakeup request
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables wakeup request
 */
static inline void WKPU_EnableWakeupRequest(WKPU_Type * const base,
                                            uint32_t channelMask,
                                            bool enable)
{
    /* Enable wake-up request */
    if (enable)
    {
        base->WRER |= channelMask;
    }
    /* Disable wake-up request */
    else
    {
        base->WRER &= ~channelMask;
    }
}

/*!
 * @brief Clears interrupt flag
 *
 * This function clears interrupt flag
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 */
static inline void WKPU_ClearInterruptFlag(WKPU_Type * const base,
                                           uint32_t channelMask)
{
    /* Clear ISR flag */
    base->WISR = channelMask;
}

/*!
 * @brief Enables or disables rising edge event
 *
 * This function enables or disables rising edge event
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables rising edge event
 */
static inline void WKPU_EnableRisingEdge(WKPU_Type * const base,
                                         uint32_t channelMask,
                                         bool enable)
{
    /* Enables Wakeup/Interrupt Rising edge event enable Register */
    if (enable)
    {
        base->WIREER |= channelMask;
    }
    /* Disables Wakeup/Interrupt Rising edge event enable Register */
    else
    {
        base->WIREER &= ~channelMask;
    }
}

/*!
 * @brief Enables or disables falling edge event
 *
 * This function enables or disables falling edge event
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables falling edge event
 */
static inline void WKPU_EnableFallingEdge(WKPU_Type * const base,
                                          uint32_t channelMask,
                                          bool enable)
{
    /* Enables Wakeup/Interrupt Falling edge event enable Register */
    if (enable)
    {
        base->WIFEER |= channelMask;
    }
    /* Disables Wakeup/Interrupt Falling edge event enable Register */
    else
    {
        base->WIFEER &= ~channelMask;
    }
}

/*!
 * @brief Enables or disables filter
 *
 * This function enables or disables filter
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables filter
 */
static inline void WKPU_EnableFilter(WKPU_Type * const base,
                                     uint32_t channelMask,
                                     bool enable)
{
    /* Enables Wakeup/Interrupt Filter Enable Register */
    if (enable)
    {
        base->WIFER |= channelMask;
    }
    /* Disables Wakeup/Interrupt Filter Enable Register */
    else
    {
        base->WIFER &= ~channelMask;
    }
}

/*!
 * @brief Enables or disables request enable
 *
 * This function enables or disables request enable
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables pull
 */
static inline void WKPU_EnablePull(WKPU_Type * const base,
                                   uint32_t channelMask,
                                   bool enable)
{
    if (enable)
    {
        base->WIPER |= channelMask;
    }
    else
    {
        base->WIPER &= ~channelMask;
    }
}

/*!
 * @brief Enables interrupt and wake-up for channel mask
 *
 * This function enable interrupt and wake-up for channel mask
 *
 * @param[in] base The WKPU peripheral base address
 * @param[in] channelMask The channel mask
 * @param[in] enable Enables or disables interrupt
 */
void WKPU_EnableInterrupt(WKPU_Type * const base,
                          uint32_t channelMask,
                          bool enable);
#endif

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* WKPU_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
