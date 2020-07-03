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

/*!
 * @file crc_c55_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Unusual pointer cast (incompatible
 * indirect types).
 * The cast is to required access address register and write it.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer
 * to object type and a pointer to a different object type.
 * This cast is to required write 8bit or 16bit data into corresponding address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const/volatile
 * from a pointer or reference.
 *
 */

#ifndef CRC_C55_HW_ACCESS_H
#define CRC_C55_HW_ACCESS_H

#include "crc_driver.h"

/*!

 * @brief Cyclic Redundancy Check Hardware Access.
 *
 * This section describes the programming interface of the CRC C55 HW ACCESS.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name CRC C55 HW ACCESS API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the CRC module
 *
 * This function initializes the module to default configuration
 * (Default polynomial: 0x1021U,
 * Swap byte-wise CRC_INP input data: false,
 * Swap bit-wise CRC_INP input data: true,
 * Swap CRC_OUTP content: false,
 * Inversion CRC_OUTP content: false,
 * No complement of checksum read,
 * CRC_BITS_16_CCITT).
 *
 * @param[in] base The CRC peripheral base address
 */
void CRC_Init(CRC_Type * const base);

/*!
 * @brief Sets the CRC transpose type for writes
 *
 * This function sets the CRC transpose type for writes
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] transp The CRC input transpose type
 */
void CRC_SetWriteTranspose(CRC_Type * const base,
                           crc_transpose_t transp);

/*!
 * @brief Gets the CRC transpose type for writes
 *
 * This function gets the CRC transpose type for writes
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC input transpose type for writes
 */
crc_transpose_t CRC_GetWriteTranspose(const CRC_Type * const base);

/*!
 * @brief Returns the current result of the CRC calculation
 *
 * This function returns the current result of the CRC calculation
 *
 * @param[in] base The CRC peripheral base address
 * @return Result of CRC calculation
 */
uint32_t CRC_GetCrcResult(const CRC_Type * const base);

/*!
 * @brief Gets the current CRC result
 *
 * This function gets the 32 bits of the current CRC result from the data register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns the current CRC result
 */
static inline uint32_t CRC_GetDataOutp(const CRC_Type * const base)
{
    return base->OUTP;
}

/*!
 * @brief Sets the 32 bits of CRC data register
 *
 * This function sets the 32 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 */
static inline void CRC_SetDataReg(CRC_Type * const base,
                                  uint32_t value)
{
    base->INP = value;
}

/*!
 * @brief Sets the lower 16 bits of CRC data register
 *
 * This function sets the lower 16 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 */
static inline void CRC_SetDataLReg(CRC_Type * const base,
                                   uint16_t value)
{
    *(uint16_t *)&base->INP = value;
}

/*!
 * @brief Sets the Lowest 8 bits of CRC data register
 *
 * This function sets the Lowest 8 bits of CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] value New data for CRC computation
 */
static inline void CRC_SetDataLLReg(CRC_Type * const base,
                                    uint8_t value)
{
    *(uint8_t *)&base->INP = value;
}

/*!
 * @brief Gets seed of the CRC data register
 *
 * This function Gets seed of the CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @return Returns seed for CRC computation
 */
static inline uint32_t CRC_GetDataReg(const CRC_Type * const base)
{
    return base->CSTAT;
}

/*!
 * @brief Sets seed of the CRC data register
 *
 * This function sets seed of the CRC data register
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] seedvalue New seed for CRC computation
 */
static inline void CRC_SetSeedReg(CRC_Type * const base,
                                  uint32_t seedvalue)
{
    base->CSTAT = seedvalue;
}

/*!
 * @brief Gets the CRC polynomial
 *
 * This function gets the CRC polynomial
 *
 * @param[in] base The CRC peripheral base address
 * @return CRC polynomial
 *            - CRC_BITS_16_CCITT  : CRC-CCITT polynomial
 *            - CRC_BITS_32        : CRC-32 polynomial
 *            - CRC_BITS_8         : CRC-8 polynomial
 *            - CRC_BITS_8_H2F     : CRC-8-H2F Autosar polynomial
 */
static inline crc_mode_polynomial_t CRC_GetPolyReg(const CRC_Type * const base)
{
    crc_mode_polynomial_t retVal;
    uint32_t temp = ((base->CFG & CRC_CFG_POLYG_MASK) >> CRC_CFG_POLYG_SHIFT);

    switch(temp)
    {
        case 1U:
            retVal = CRC_BITS_32;
            break;
        case 2U:
            retVal = CRC_BITS_8;
            break;
#if (FEATURE_CRC_BITS_8_H2F == 1U)
        case 3U:
            retVal = CRC_BITS_8_H2F;
            break;
#endif
        default:
            retVal = CRC_BITS_16_CCITT;
            break;
    }

    return retVal;
}

/*!
 * @brief Sets the CRC polynomial
 *
 * This function sets the CRC polynomial
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] polynomial The CRC polynomial
 *            - CRC_BITS_16_CCITT   : CRC-CCITT polynomial
 *            - CRC_BITS_32         : CRC-32 polynomial
 *            - CRC_BITS_8          : CRC-8 polynomial
 *            - CRC_BITS_8_H2F      : CRC-8-H2F Autosar polynomial
 */
static inline void CRC_SetPolyReg(CRC_Type * const base,
                                  crc_mode_polynomial_t polynomial)
{
    uint32_t ctrlTemp = base->CFG;

    ctrlTemp &= ~(CRC_CFG_POLYG_MASK);
    ctrlTemp |= CRC_CFG_POLYG(polynomial);
    base->CFG = ctrlTemp;
}

/*!
 * @brief Gets the CRC Swap byte-wise
 *
 * This function gets Swap CRC_INP byte-wise
 *
 * @param[in] base The CRC peripheral base address
 * @return Swap CRC_INP byte-wise
 */
static inline bool CRC_GetSwapBytewise(const CRC_Type * const base)
{
    return ((base->CFG & CRC_CFG_SWAP_BYTEWISE_MASK) >> CRC_CFG_SWAP_BYTEWISE_SHIFT) != 0U;
}

/*!
 * @brief Sets the CRC Swap byte-wise
 *
 * This function sets Swap CRC_INP byte-wise
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable or disable Swap CRC_INP byte-wise
 */
static inline void CRC_SetSwapBytewise(CRC_Type * const base,
                                       bool enable)
{
    uint32_t ctrlTemp = base->CFG;

    ctrlTemp &= ~(CRC_CFG_SWAP_BYTEWISE_MASK);
    ctrlTemp |= CRC_CFG_SWAP_BYTEWISE(enable ? 1UL : 0UL);
    base->CFG = ctrlTemp;
}

/*!
 * @brief Gets the CRC Swap bit-wise
 *
 * This function gets Swap CRC_INP bit-wise
 *
 * @param[in] base The CRC peripheral base address
 * @return Swap CRC_INP bit-wise
 */
static inline bool CRC_GetSwapBitwise(const CRC_Type * const base)
{
    return ((base->CFG & CRC_CFG_SWAP_BITWISE_MASK) >> CRC_CFG_SWAP_BITWISE_SHIFT) != 0U;
}

/*!
 * @brief Sets the CRC Swap bit-wise
 *
 * This function sets Swap CRC_INP bit-wise
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable or disable Swap CRC_INP bit-wise
 */
static inline void CRC_SetSwapBitwise(CRC_Type * const base,
                                      bool enable)
{
    uint32_t ctrlTemp = base->CFG;

    ctrlTemp &= ~(CRC_CFG_SWAP_BITWISE_MASK);
    ctrlTemp |= CRC_CFG_SWAP_BITWISE(enable ? 1UL : 0UL);
    base->CFG = ctrlTemp;
}

/*!
 * @brief Gets the CRC transpose CRC_OUTP
 *
 * This function gets transpose CRC_OUTP content
 *
 * @param[in] base The CRC peripheral base address
 * @return Swap bit CRC_OUTP content
 */
static inline bool CRC_GetReadTranspose(const CRC_Type * const base)
{
    return ((base->CFG & CRC_CFG_SWAP_MASK) >> CRC_CFG_SWAP_SHIFT) != 0U;
}

/*!
 * @brief Sets the CRC transpose
 *
 * This function sets transpose CRC_OUTP content
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable or disable swap CRC_OUTP content
 */
static inline void CRC_SetReadTranspose(CRC_Type * const base,
                                        bool enable)
{
    uint32_t ctrlTemp = base->CFG;

    ctrlTemp &= ~(CRC_CFG_SWAP_MASK);
    ctrlTemp |= CRC_CFG_SWAP(enable ? 1UL : 0UL);
    base->CFG = ctrlTemp;
}

/*!
 * @brief Gets the CRC Inversion
 *
 * This function gets Inversion CRC_OUTP content
 *
 * @param[in] base The CRC peripheral base address
 * @return Complement or inversion CRC_OUTP content
 */
static inline bool CRC_GetFXorMode(const CRC_Type * const base)
{
    return (base->CFG & CRC_CFG_INV_MASK) != 0U;
}

/*!
 * @brief Sets the CRC Inversion content
 *
 * This function sets Inversion CRC_OUTP content
 *
 * @param[in] base The CRC peripheral base address
 * @param[in] enable Enable or disable Inversion CRC_OUTP content
 */
static inline void CRC_SetFXorMode(CRC_Type * const base,
                                   bool enable)
{
    uint32_t ctrlTemp = base->CFG;

    ctrlTemp &= ~(CRC_CFG_INV_MASK);
    ctrlTemp |= CRC_CFG_INV_MASK & (enable ? 1UL : 0UL);
    base->CFG = ctrlTemp;
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* CRC_C55_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
