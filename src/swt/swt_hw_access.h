/*
 * Copyright 2017 NXP.
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
 * @file swt_hw_access.h
 */

#ifndef SWT_HW_ACCESS_H
#define SWT_HW_ACCESS_H

#include "swt_driver.h"

/*!
 * @brief Software Watchdog Timer Hardware Access layer.
 * @{
 */

/*******************************************************************************
 * API
 *******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Unlocks SWT registers.
 * This function unlocks the SWT register
 * - The soft lock is cleared by writing the unlock sequence to the service register.
 * - The hard lock can only be cleared by a reset.
 *
 * @param[in] base The SWT peripheral base address
 * @return operation status:
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to the SWT was locked by hard lock.
 */
status_t SWT_UnLock(SWT_Type * const base);

/*!
 * @brief Locks SWT registers.
 * This function locks the SWT register
 * When locked, the SWT_CR, SWT_TO, SWT_WN and SWT_SK registers are read-only.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] lockConfig The configuration lock bits
 *            - SWT_UNLOCK   : Unlock
 *            - SWT_SOFTLOCK : Soft lock
 *            - SWT_HARDLOCK : Hard lock
 */
void SWT_Lock(SWT_Type * const base,
              swt_lock_t lockConfig);

/*!
 * @brief Resets the SWT.
 * This function resets all configuration to default and disable the SWT timer.
 *
 * @param[in] base The SWT peripheral base address
 */
void SWT_Reset(SWT_Type * const base);

/*!
 * @brief Configures the SWT.
 * This function configures the SWT by user configuration.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] userConfigPtr Pointer to the SWT user configuration structure
 * @return operation status
 *         - STATUS_SUCCESS : Operation was successful.
 *         - STATUS_ERROR   : Operation failed due to timeout value less than minimum timeout
 *                            or the SWT was locked by hard lock.
 */
status_t SWT_Config(SWT_Type * const base,
                    const swt_user_config_t * const userConfigPtr);

/*!
 * @brief Enables the SWT.
 * This function enables the SWT.
 *
 * @param[in] base The SWT peripheral base address
 */
static inline void SWT_Enable(SWT_Type * const base)
{
    /* Enables the SWT */
    base->CR |= SWT_CR_WEN_MASK;
}

/*!
 * @brief Disables the SWT.
 * This function disables the SWT.
 *
 * @param[in] base The SWT peripheral base address
 */
static inline void SWT_Disable(SWT_Type * const base)
{
    /* Enables the SWT */
    base->CR &= ~SWT_CR_WEN_MASK;
}

/*!
 * @brief Checks whether the SWT is enabled.
 * This function checks whether the SWT is enabled.
 *
 * @param[in] base The SWT peripheral base address
 * @return The SWT enable status
 */
static inline bool SWT_IsEnable(const SWT_Type * const base)
{
    /* Checks whether the SWT is enabled */
    return ((base->CR & SWT_CR_WEN_MASK) != 0U);
}

/*!
 * @brief Sets timeout value.
 * This function sets timeout value.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] value The timeout value
 */
static inline void SWT_SetTimeoutValue(SWT_Type * const base,
                                       uint32_t value)
{
    /* Sets timeout value */
    base->TO = value;
}

/*!
 * @brief Enables/Disables window mode.
 * This function enables/disables window mode.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] enable State
 *            - true  : Enable window mode
 *            - false : Disable window mode
 */
static inline void SWT_EnableWindow(SWT_Type * const base,
                                    bool enable)
{
    /* Enables/Disables window mode */
    base->CR = (base->CR & ~SWT_CR_WND_MASK) | SWT_CR_WND(enable ? 1UL : 0UL);
}

/*!
 * @brief Sets window value.
 * This function sets window value.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] value The window value
 */
static inline void SWT_SetWindowValue(SWT_Type * const base,
                                      uint32_t value)
{
    /* Sets window value */
    base->WN = value;
}

/*!
 * @brief Sets service mode.
 * This function sets service mode.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] mode Service mode
 *            - SWT_FS_SEQ_MODE : Fixed Service Sequence mode
 *            - SWT_KS_SEQ_MODE : Keyed Service Sequence mode
 *            - SWT_FA_EXE_MODE : Fixed Address Execution mode
 *            - SWT_IA_EXE_MODE : Incremental Address Execution mode
 */
static inline void SWT_SetServiceMode(SWT_Type * const base,
                                      swt_service_mode_t mode)
{
    /* Sets service mode */
    base->CR = (base->CR & ~SWT_CR_SMD_MASK) | SWT_CR_SMD(mode);
}

/*!
 * @brief Gets current service mode.
 * This function gets current service mode.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] mode Service mode
 * @return Service mode
 *         - SWT_FS_SEQ_MODE : Fixed Service Sequence mode
 *         - SWT_KS_SEQ_MODE : Keyed Service Sequence mode
 *         - SWT_FA_EXE_MODE : Fixed Address Execution mode
 *         - SWT_IA_EXE_MODE : Incremental Address Execution mode
 */
swt_service_mode_t SWT_GetServiceMode(const SWT_Type * const base);

/*!
 * @brief Sets service key.
 * This function sets service key.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] serviceKey The initial service key
 */
static inline void SWT_SetServiceKey(SWT_Type * const base,
                                     uint16_t serviceKey)
{
    /* Sets initial service key */
    base->SK = (base->SK & ~SWT_SK_SK_MASK) | SWT_SK_SK(serviceKey);
}

/*!
 * @brief Generates the next key to service the SWT.
 * This function generates the next key used to service the SWT in keyed service sequence mode.
 * The next key value to be written to the SWT_SR is (17*SK+3) mod 2^16.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] serviceKey The initial service key
 * @return Service key
 */
static inline uint16_t SWT_ServiceKeyGen(const SWT_Type * const base)
{
    /* Generates the next key used to service the SWT */
    return ((uint16_t)((((base->SK & SWT_SK_SK_MASK) >> SWT_SK_SK_SHIFT) * 17U) + 3U));
}

/*!
 * @brief Sets service command.
 * This function sets service command.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] serviceKey The service key
 */
static inline void SWT_SetServiceCmd(SWT_Type * const base,
                                     uint16_t serviceKey)
{
    base->SR = (base->SR & ~SWT_SR_WSC_MASK) | SWT_SR_WSC(serviceKey);
}

/*!
 * @brief Enables/Disables interrupt.
 * This function enables/disables interrupt.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] enable State
 *            - true  : enable interrupt
 *            - false : disable interrupt
 */
static inline void SWT_EnableInt(SWT_Type * const base,
                                 bool enable)
{
    /* Enables/Disables SWT interrupt */
    base->CR = (base->CR & ~SWT_CR_ITR_MASK) | SWT_CR_ITR(enable ? 1UL : 0UL);
}

/*!
 * @brief Gets counter value.
 * This function gets counter value.
 *
 * @param[in] base The SWT peripheral base address
 * @return The counter value
 */
static inline uint32_t SWT_GetCounter(const SWT_Type * const base)
{
    return (base->CO);
}

/*!
 * @brief Write and read interrupt register follow mask.
 * This function write to interrupt register and read again follow mask.
 *
 * @param[in] base The SWT peripheral base address
 * @param[in] mask The location clears (write 0)
 * @param[in] value The value to write
 * @return The interrupt register value
 */
uint32_t SWT_WriteReadIntReg(SWT_Type * const base,
                             uint32_t mask,
                             uint32_t value);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* SWT_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
