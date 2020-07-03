/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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

#if !defined(CGM_HW_ACCESS_H)
#define CGM_HW_ACCESS_H

#include "device_registers.h"
#include <stdbool.h>
#include <stddef.h>

/*!
 * @file cgm_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 */

/*!
 * @ingroup cgm_hw_access
 * @defgroup cgm_hw_access
 * @{
 */




#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief Gets system clock first divider status
 *
 * This function gets system clock first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock first divider status
 */
static inline bool CGM_GetSC_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC0_DE_MASK
    return (base->SC_DC0 & MC_CGM_SC_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock second divider status
 *
 * This function gets system clock second divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock second divider status
 */
static inline bool CGM_GetSC_DC1_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC1_DE_MASK
    return (base->SC_DC1 & MC_CGM_SC_DC1_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock 3rd divider status
 *
 * This function gets system clock 3rd divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 3d divider status
 */
static inline bool CGM_GetSC_DC2_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC2_DE_MASK
    return (base->SC_DC2 & MC_CGM_SC_DC2_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock 4th divider status
 *
 * This function gets system clock 4th divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 4th divider status
 */
static inline bool CGM_GetSC_DC3_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC3_DE_MASK
    return (base->SC_DC3 & MC_CGM_SC_DC3_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock 5th divider status
 *
 * This function gets system clock 5th divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 5th divider status
 */
static inline bool CGM_GetSC_DC4_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC4_DE_MASK
    return (base->SC_DC4 & MC_CGM_SC_DC4_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock 6th divider status
 *
 * This function gets system clock 6th divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 6th divider status
 */
static inline bool CGM_GetSC_DC5_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC5_DE_MASK
    return (base->SC_DC5 & MC_CGM_SC_DC5_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets system clock 7th divider status
 *
 * This function gets system clock 7th divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 6th divider status
 */
static inline bool CGM_GetSC_DC6_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_SC_DC6_DE_MASK
    return (base->SC_DC6 & MC_CGM_SC_DC6_DE_MASK) != 0U;
#else
    return false;
#endif
}


/*!
 * @brief Gets system clock first divider value
 *
 * This function gets system clock first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock first divider value
 */
static inline uint32_t CGM_GetSC_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC0_DIV_MASK) && defined(MC_CGM_SC_DC0_DIV_SHIFT)
    return ((base->SC_DC0 & MC_CGM_SC_DC0_DIV_MASK) >> MC_CGM_SC_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock second divider value
 *
 * This function gets system clock second divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock second divider value
 */
static inline uint32_t CGM_GetSC_DC1_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC1_DIV_MASK) && defined(MC_CGM_SC_DC1_DIV_SHIFT)
    return ((base->SC_DC1 & MC_CGM_SC_DC1_DIV_MASK) >> MC_CGM_SC_DC1_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock third divider value
 *
 * This function gets system clock third divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock third divider value
 */
static inline uint32_t CGM_GetSC_DC2_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC2_DIV_MASK) && defined(MC_CGM_SC_DC2_DIV_SHIFT)
    return ((base->SC_DC2 & MC_CGM_SC_DC2_DIV_MASK) >> MC_CGM_SC_DC2_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock 4th divider value
 *
 * This function gets system clock 4th divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 4th divider value
 */
static inline uint32_t CGM_GetSC_DC3_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC3_DIV_MASK) && defined(MC_CGM_SC_DC3_DIV_SHIFT)
    return ((base->SC_DC3 & MC_CGM_SC_DC3_DIV_MASK) >> MC_CGM_SC_DC3_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock 5th divider value
 *
 * This function gets system clock 5th divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 5th divider value
 */
static inline uint32_t CGM_GetSC_DC4_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC4_DIV_MASK) && defined(MC_CGM_SC_DC4_DIV_SHIFT)
    return ((base->SC_DC4 & MC_CGM_SC_DC4_DIV_MASK) >> MC_CGM_SC_DC4_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock 6th divider value
 *
 * This function gets system clock 6th divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 6th divider value
 */
static inline uint32_t CGM_GetSC_DC5_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC5_DIV_MASK) && defined(MC_CGM_SC_DC5_DIV_SHIFT)
    return ((base->SC_DC5 & MC_CGM_SC_DC5_DIV_MASK) >> MC_CGM_SC_DC5_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets system clock 7th divider value
 *
 * This function gets system clock 7th divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return system clock 7th divider value
 */
static inline uint32_t CGM_GetSC_DC6_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_SC_DC6_DIV_MASK) && defined(MC_CGM_SC_DC6_DIV_SHIFT)
    return ((base->SC_DC6 & MC_CGM_SC_DC6_DIV_MASK) >> MC_CGM_SC_DC6_DIV_SHIFT);
#else
    return 0U;
#endif
}


/*!
 * @brief Sets CGM AC0 selector
 *
 * This function sets CGM AC0 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC0(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC0_SC_SELCTL)
    base->AC0_SC = MC_CGM_AC0_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC0 1st divider
 *
 * This function sets CGM AC0 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC0_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC0_DC0_DIV) && defined(MC_CGM_AC0_DC0_DE)
    base->AC0_DC0 = (MC_CGM_AC0_DC0_DIV(divValue) |
                     MC_CGM_AC0_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC0 2nd divider
 *
 * This function sets CGM AC0 2nd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC0_DC1(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC0_DC1_DIV) && defined(MC_CGM_AC0_DC1_DE)
    base->AC0_DC1 = (MC_CGM_AC0_DC1_DIV(divValue) |
            MC_CGM_AC0_DC1_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC0 3rd divider
 *
 * This function sets CGM AC0 3rd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC0_DC2(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC0_DC2_DIV) && defined(MC_CGM_AC0_DC2_DE)
    base->AC0_DC2 = (MC_CGM_AC0_DC2_DIV(divValue) |
                     MC_CGM_AC0_DC2_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC0 4th divider
 *
 * This function sets CGM AC0 4th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC0_DC3(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC0_DC3_DIV) && defined(MC_CGM_AC0_DC3_DE)
    base->AC0_DC3 = (MC_CGM_AC0_DC3_DIV(divValue) |
                     MC_CGM_AC0_DC3_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC0 5th divider
 *
 * This function sets CGM AC0 5th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC0_DC4(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC0_DC4_DIV) && defined(MC_CGM_AC0_DC4_DE)
    base->AC0_DC4 = (MC_CGM_AC0_DC4_DIV(divValue) |
                     MC_CGM_AC0_DC4_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC1 selector
 *
 * This function sets CGM AC1 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC1(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC1_SC_SELCTL)
    base->AC1_SC = MC_CGM_AC1_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC1 1st divider
 *
 * This function sets CGM AC1 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC1_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC1_DC0_DIV) && defined(MC_CGM_AC1_DC0_DE)
    base->AC1_DC0 = (MC_CGM_AC1_DC0_DIV(divValue) |
                     MC_CGM_AC1_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC1 2nd divider
 *
 * This function sets CGM AC1 2nd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC1_DC1(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC1_DC1_DIV) && defined(MC_CGM_AC1_DC1_DE)
    base->AC1_DC1 = (MC_CGM_AC1_DC1_DIV(divValue) |
            MC_CGM_AC1_DC1_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC2 selector
 *
 * This function sets CGM AC2 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC2(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC2_SC_SELCTL)
    base->AC2_SC = MC_CGM_AC2_SC_SELCTL(selector);
#endif

}

/*!
 * @brief Sets CGM AC1 1st divider
 *
 * This function sets CGM AC1 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC2_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC2_DC0_DIV) && defined(MC_CGM_AC2_DC0_DE)
    base->AC2_DC0 = (MC_CGM_AC2_DC0_DIV(divValue) |
                     MC_CGM_AC2_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC3 selector
 *
 * This function sets CGM AC3 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC3(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;
#if defined(MC_CGM_AC3_SC_SELCTL)
    base->AC3_SC = MC_CGM_AC3_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC4 selector
 *
 * This function sets CGM AC4 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC4(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;
#if defined(MC_CGM_AC4_SC_SELCTL)
    base->AC4_SC = MC_CGM_AC4_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC5 selector
 *
 * This function sets CGM AC5 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC5(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;
#if defined(MC_CGM_AC5_SC_SELCTL)
    base->AC5_SC = MC_CGM_AC5_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC5 1st divider
 *
 * This function sets CGM AC5 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC5_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC5_DC0_DIV) && defined(MC_CGM_AC5_DC0_DE)
    base->AC5_DC0 = (MC_CGM_AC5_DC0_DIV(divValue) |
                     MC_CGM_AC5_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC5 2nd divider
 *
 * This function sets CGM AC5 2nd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC5_DC1(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC5_DC1_DIV) && defined(MC_CGM_AC5_DC1_DE)
    base->AC5_DC1 = (MC_CGM_AC5_DC1_DIV(divValue) |
                     MC_CGM_AC5_DC1_DE(enable ? 1UL : 0UL));
#endif
}


/*!
 * @brief Sets CGM AC6 selector
 *
 * This function sets CGM AC6 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC6(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC6_SC_SELCTL)
    base->AC6_SC = MC_CGM_AC6_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC6 1st divider
 *
 * This function sets CGM AC6 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC6_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC6_DC0_DIV) && defined(MC_CGM_AC6_DC0_DE)
    base->AC6_DC0 = (MC_CGM_AC6_DC0_DIV(divValue) |
                     MC_CGM_AC6_DC0_DE(enable ? 1UL : 0UL));
#endif
}


/*!
 * @brief Sets CGM AC7 selector
 *
 * This function sets CGM AC7 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC7(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC7_SC_SELCTL)
    base->AC7_SC = MC_CGM_AC7_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC7 1st divider
 *
 * This function sets CGM AC7 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC7_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC7_DC0_DIV) && defined(MC_CGM_AC7_DC0_DE)
    base->AC7_DC0 = (MC_CGM_AC7_DC0_DIV(divValue) |
                     MC_CGM_AC7_DC0_DE(enable ? 1UL : 0UL));
#endif
}



/*!
 * @brief Sets CGM AC8 selector
 *
 * This function sets CGM AC8 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC8(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC8_SC_SELCTL)
    base->AC8_SC = MC_CGM_AC8_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC8 1st divider
 *
 * This function sets CGM AC8 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC8_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC8_DC0_DIV) && defined(MC_CGM_AC8_DC0_DE)
    base->AC8_DC0 = (MC_CGM_AC8_DC0_DIV(divValue) |
                     MC_CGM_AC8_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC9 selector
 *
 * This function sets CGM AC9 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC9(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC9_SC_SELCTL)
    base->AC9_SC = MC_CGM_AC9_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC9 1st divider
 *
 * This function sets CGM AC9 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC9_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC9_DC0_DIV) && defined(MC_CGM_AC9_DC0_DE)
    base->AC9_DC0 = (MC_CGM_AC9_DC0_DIV(divValue) |
                     MC_CGM_AC9_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC10 selector
 *
 * This function sets CGM AC10 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC10(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC10_SC_SELCTL)
    base->AC10_SC = MC_CGM_AC10_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC10 1st divider
 *
 * This function sets CGM AC10 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC10_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC10_DC0_DIV) && defined(MC_CGM_AC10_DC0_DE)
    base->AC10_DC0 = (MC_CGM_AC10_DC0_DIV(divValue) |
                     MC_CGM_AC10_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC11 selector
 *
 * This function sets CGM AC11 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC11(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC11_SC_SELCTL)
    base->AC11_SC = MC_CGM_AC11_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC11 1st divider
 *
 * This function sets CGM AC11 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC11_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC11_DC0_DIV) && defined(MC_CGM_AC11_DC0_DE)
    base->AC11_DC0 = (MC_CGM_AC11_DC0_DIV(divValue) |
                     MC_CGM_AC11_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC12 selector
 *
 * This function sets CGM AC12 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC12(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC12_SC_SELCTL)
    base->AC12_SC = MC_CGM_AC12_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC12 1st divider
 *
 * This function sets CGM AC12 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC12_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC12_DC0_DIV) && defined(MC_CGM_AC12_DC0_DE)
    base->AC12_DC0 = (MC_CGM_AC12_DC0_DIV(divValue) |
                     MC_CGM_AC12_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC13 selector
 *
 * This function sets CGM AC13 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC13(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC13_SC_SELCTL)
    base->AC13_SC = MC_CGM_AC13_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC13 1st divider
 *
 * This function sets CGM AC13 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC13_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC13_DC0_DIV) && defined(MC_CGM_AC13_DC0_DE)
    base->AC13_DC0 = (MC_CGM_AC13_DC0_DIV(divValue) |
                     MC_CGM_AC13_DC0_DE(enable ? 1UL : 0UL));
#endif
}

/*!
 * @brief Sets CGM AC14 selector
 *
 * This function sets CGM AC14 selector
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] selector Selector value.
 */
static inline void CGM_SetAC14(MC_CGM_Type * base, uint32_t selector)
{
    (void)base;
    (void)selector;

#if defined(MC_CGM_AC14_SC_SELCTL)
    base->AC14_SC = MC_CGM_AC14_SC_SELCTL(selector);
#endif
}

/*!
 * @brief Sets CGM AC14 1st divider
 *
 * This function sets CGM AC14 1st divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetAC14_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if defined(MC_CGM_AC14_DC0_DIV) && defined(MC_CGM_AC14_DC0_DE)
    base->AC14_DC0 = (MC_CGM_AC14_DC0_DIV(divValue) |
                     MC_CGM_AC14_DC0_DE(enable ? 1UL : 0UL));
#endif
}


/*!
 * @brief Gets CGM AC0 selector
 *
 * This function gets CGM AC0 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC0 clock selector value
 */
static inline uint32_t CGM_GetAC0_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_SS_SELSTAT_MASK) && defined(MC_CGM_AC0_SS_SELSTAT_SHIFT)
    return ((base->AC0_SC & MC_CGM_AC0_SS_SELSTAT_MASK) >> MC_CGM_AC0_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC1 selector
 *
 * This function gets CGM AC1 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC1 clock selector value
 */
static inline uint32_t CGM_GetAC1_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC1_SS_SELSTAT_MASK) && defined(MC_CGM_AC1_SS_SELSTAT_SHIFT)
    return ((base->AC1_SC & MC_CGM_AC1_SS_SELSTAT_MASK) >> MC_CGM_AC1_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}


/*!
 * @brief Gets CGM AC2 selector
 *
 * This function gets CGM AC2 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC2 clock selector value
 */
static inline uint32_t CGM_GetAC2_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC2_SS_SELSTAT_MASK) && defined(MC_CGM_AC2_SS_SELSTAT_SHIFT)
    return ((base->AC2_SC & MC_CGM_AC2_SS_SELSTAT_MASK) >> MC_CGM_AC2_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC3 selector
 *
 * This function gets CGM AC3 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC3 clock selector value
 */
static inline uint32_t CGM_GetAC3_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC3_SS_SELSTAT_MASK) && defined(MC_CGM_AC3_SS_SELSTAT_SHIFT)
    return ((base->AC3_SC & MC_CGM_AC3_SS_SELSTAT_MASK) >> MC_CGM_AC3_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC4 selector
 *
 * This function gets CGM AC4 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC4 clock selector value
 */
static inline uint32_t CGM_GetAC4_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC4_SS_SELSTAT_MASK) && defined(MC_CGM_AC4_SS_SELSTAT_SHIFT)
    return ((base->AC4_SC & MC_CGM_AC4_SS_SELSTAT_MASK) >> MC_CGM_AC4_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC5 selector
 *
 * This function gets CGM AC5 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC5 clock selector value
 */
static inline uint32_t CGM_GetAC5_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC5_SS_SELSTAT_MASK) && defined(MC_CGM_AC5_SS_SELSTAT_SHIFT)
    return ((base->AC5_SC & MC_CGM_AC5_SS_SELSTAT_MASK) >> MC_CGM_AC5_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC6 selector
 *
 * This function gets CGM AC6 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC7 clock selector value
 */
static inline uint32_t CGM_GetAC6_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC6_SC_SELCTL_MASK) && defined(MC_CGM_AC6_SS_SELSTAT_SHIFT)
    return ((base->AC6_SC & MC_CGM_AC6_SC_SELCTL_MASK) >> MC_CGM_AC6_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC7 selector
 *
 * This function gets CGM AC7 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC7 clock selector value
 */
static inline uint32_t CGM_GetAC7_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC7_SS_SELSTAT_MASK) && defined(MC_CGM_AC7_SS_SELSTAT_SHIFT)
    return ((base->AC7_SC & MC_CGM_AC7_SS_SELSTAT_MASK) >> MC_CGM_AC7_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC8 selector
 *
 * This function gets CGM AC8 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC8 clock selector value
 */
static inline uint32_t CGM_GetAC8_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC8_SS_SELSTAT_MASK) && defined(MC_CGM_AC8_SS_SELSTAT_SHIFT)
    return ((base->AC8_SC & MC_CGM_AC8_SS_SELSTAT_MASK) >> MC_CGM_AC8_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC9 selector
 *
 * This function gets CGM AC9 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC9 clock selector value
 */
static inline uint32_t CGM_GetAC9_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC9_SS_SELSTAT_MASK) && defined(MC_CGM_AC9_SS_SELSTAT_SHIFT)
    return ((base->AC9_SC & MC_CGM_AC9_SS_SELSTAT_MASK) >> MC_CGM_AC9_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}




/*!
 * @brief Gets CGM AC10 selector
 *
 * This function gets CGM AC10 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC10 clock selector value
 */
static inline uint32_t CGM_GetAC10_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC10_SS_SELSTAT_MASK) && defined(MC_CGM_AC10_SS_SELSTAT_SHIFT)
    return ((base->AC10_SC & MC_CGM_AC10_SS_SELSTAT_MASK) >> MC_CGM_AC10_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}


/*!
 * @brief Gets CGM AC11 selector
 *
 * This function gets CGM AC11 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC11 clock selector value
 */
static inline uint32_t CGM_GetAC11_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC11_SS_SELSTAT_MASK) && defined(MC_CGM_AC11_SS_SELSTAT_SHIFT)
    return ((base->AC11_SC & MC_CGM_AC11_SS_SELSTAT_MASK) >> MC_CGM_AC11_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC12 selector
 *
 * This function gets CGM AC12 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC12 clock selector value
 */
static inline uint32_t CGM_GetAC12_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC12_SS_SELSTAT_MASK) && defined(MC_CGM_AC12_SS_SELSTAT_SHIFT)
    return ((base->AC12_SC & MC_CGM_AC12_SS_SELSTAT_MASK) >> MC_CGM_AC12_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets CGM AC13 selector
 *
 * This function gets CGM AC13 selector
 *
 * @param[in] base Register base address for the CGM instance.
 * @return AC13 clock selector value
 */
static inline uint32_t CGM_GetAC13_SelValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC13_SS_SELSTAT_MASK) && defined(MC_CGM_AC13_SS_SELSTAT_SHIFT)
    return ((base->AC13_SC & MC_CGM_AC13_SS_SELSTAT_MASK) >> MC_CGM_AC13_SS_SELSTAT_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 first divider value
 *
 * This function gets auxiliary selector clock 0 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider value
 */
static inline uint32_t CGM_GetAC0_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_DC0_DIV_MASK) && defined(MC_CGM_AC0_DC0_DIV_SHIFT)
    return ((base->AC0_DC0 & MC_CGM_AC0_DC0_DIV_MASK) >> MC_CGM_AC0_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 second divider value
 *
 * This function gets auxiliary selector clock 0 second divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 second divider value
 */
static inline uint32_t CGM_GetAC0_DC1_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_DC1_DIV_MASK) && defined(MC_CGM_AC0_DC1_DIV_SHIFT)
    return ((base->AC0_DC1 & MC_CGM_AC0_DC1_DIV_MASK) >> MC_CGM_AC0_DC1_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 third divider value
 *
 * This function gets auxiliary selector clock 0 third divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 third divider value
 */
static inline uint32_t CGM_GetAC0_DC2_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_DC2_DIV_MASK) && defined(MC_CGM_AC0_DC2_DIV_SHIFT)
    return ((base->AC0_DC2 & MC_CGM_AC0_DC2_DIV_MASK) >> MC_CGM_AC0_DC2_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 fourth divider value
 *
 * This function gets auxiliary selector clock 0 fourth divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 fourth divider value
 */
static inline uint32_t CGM_GetAC0_DC3_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_DC3_DIV_MASK) && defined(MC_CGM_AC0_DC3_DIV_SHIFT)
    return ((base->AC0_DC3 & MC_CGM_AC0_DC3_DIV_MASK) >> MC_CGM_AC0_DC3_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 first divider value
 *
 * This function gets auxiliary selector clock 0 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider value
 */
static inline uint32_t CGM_GetAC0_DC4_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC0_DC4_DIV_MASK) && defined(MC_CGM_AC0_DC4_DIV_SHIFT)
    return ((base->AC0_DC4 & MC_CGM_AC0_DC4_DIV_MASK) >> MC_CGM_AC0_DC4_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 1 first divider value
 *
 * This function gets auxiliary selector clock 1 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 1 first divider value
 */
static inline uint32_t CGM_GetAC1_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC1_DC0_DIV_MASK) && defined(MC_CGM_AC1_DC0_DIV_SHIFT)
    return ((base->AC1_DC0 & MC_CGM_AC1_DC0_DIV_MASK) >> MC_CGM_AC1_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 1 second divider value
 *
 * This function gets auxiliary selector clock 1 second divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 1 second divider value
 */
static inline uint32_t CGM_GetAC1_DC1_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC1_DC1_DIV_MASK) && defined(MC_CGM_AC1_DC1_DIV_SHIFT)
    return ((base->AC1_DC1 & MC_CGM_AC1_DC1_DIV_MASK) >> MC_CGM_AC1_DC1_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 2 first divider value
 *
 * This function gets auxiliary selector clock 2 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 2 first divider value
 */
static inline uint32_t CGM_GetAC2_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC2_DC0_DIV_MASK) && defined(MC_CGM_AC2_DC0_DIV_SHIFT)
    return ((base->AC2_DC0 & MC_CGM_AC2_DC0_DIV_MASK) >> MC_CGM_AC2_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 5 first divider value
 *
 * This function gets auxiliary selector clock 5 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 5 first divider value
 */
static inline uint32_t CGM_GetAC5_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC5_DC0_DIV_MASK) && defined(MC_CGM_AC5_DC0_DIV_SHIFT)
    return ((base->AC5_DC0 & MC_CGM_AC5_DC0_DIV_MASK) >> MC_CGM_AC5_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 5 first divider value
 *
 * This function gets auxiliary selector clock 5 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 5 second divider value
 */
static inline uint32_t CGM_GetAC5_DC1_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC5_DC1_DIV_MASK) && defined(MC_CGM_AC5_DC1_DIV_SHIFT)
    return ((base->AC5_DC1 & MC_CGM_AC5_DC1_DIV_MASK) >> MC_CGM_AC5_DC1_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 6 first divider value
 *
 * This function gets auxiliary selector clock 6 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 6 first divider value
 */
static inline uint32_t CGM_GetAC6_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC6_DC0_DIV_MASK) && defined(MC_CGM_AC6_DC0_DIV_SHIFT)
    return ((base->AC6_DC0 & MC_CGM_AC6_DC0_DIV_MASK) >> MC_CGM_AC6_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}


/*!
 * @brief Gets auxiliary selector clock 7 first divider value
 *
 * This function gets auxiliary selector clock 7 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 7 first divider value
 */
static inline uint32_t CGM_GetAC7_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC7_DC0_DIV_MASK) && defined(MC_CGM_AC7_DC0_DIV_SHIFT)
    return ((base->AC7_DC0 & MC_CGM_AC7_DC0_DIV_MASK) >> MC_CGM_AC7_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 8 first divider value
 *
 * This function gets auxiliary selector clock 8 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 8 first divider value
 */
static inline uint32_t CGM_GetAC8_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC8_DC0_DIV_MASK) && defined(MC_CGM_AC8_DC0_DIV_SHIFT)
    return ((base->AC8_DC0 & MC_CGM_AC8_DC0_DIV_MASK) >> MC_CGM_AC8_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 9 first divider value
 *
 * This function gets auxiliary selector clock 9 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 9 first divider value
 */
static inline uint32_t CGM_GetAC9_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC9_DC0_DIV_MASK) && defined(MC_CGM_AC9_DC0_DIV_SHIFT)
    return ((base->AC9_DC0 & MC_CGM_AC9_DC0_DIV_MASK) >> MC_CGM_AC9_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 10 first divider value
 *
 * This function gets auxiliary selector clock 10 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 10 first divider value
 */
static inline uint32_t CGM_GetAC10_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC10_DC0_DIV_MASK) && defined(MC_CGM_AC10_DC0_DIV_SHIFT)
    return ((base->AC10_DC0 & MC_CGM_AC10_DC0_DIV_MASK) >> MC_CGM_AC10_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 11 first divider value
 *
 * This function gets auxiliary selector clock 11 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 11 first divider value
 */
static inline uint32_t CGM_GetAC11_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC11_DC0_DIV_MASK) && defined(MC_CGM_AC11_DC0_DIV_SHIFT)
    return ((base->AC11_DC0 & MC_CGM_AC11_DC0_DIV_MASK) >> MC_CGM_AC11_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 12 first divider value
 *
 * This function gets auxiliary selector clock 12 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 12 first divider value
 */
static inline uint32_t CGM_GetAC12_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC12_DC0_DIV_MASK) && defined(MC_CGM_AC12_DC0_DIV_SHIFT)
    return ((base->AC12_DC0 & MC_CGM_AC12_DC0_DIV_MASK) >> MC_CGM_AC12_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 13 first divider value
 *
 * This function gets auxiliary selector clock 13 first divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 13 first divider value
 */
static inline uint32_t CGM_GetAC13_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC13_DC0_DIV_MASK) && defined(MC_CGM_AC13_DC0_DIV_SHIFT)
    return ((base->AC13_DC0 & MC_CGM_AC13_DC0_DIV_MASK) >> MC_CGM_AC13_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary selector clock 0 second divider value
 *
 * This function gets auxiliary selector clock 0 second divider value
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 second divider value
 */
static inline uint32_t CGM_GetAC14_DC0_DividerValue(const MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_AC14_DC0_DIV_MASK) && defined(MC_CGM_AC14_DC0_DIV_SHIFT)
    return ((base->AC14_DC0 & MC_CGM_AC14_DC0_DIV_MASK) >> MC_CGM_AC14_DC0_DIV_SHIFT);
#else
    return 0U;
#endif
}

/*!
 * @brief Gets auxiliary clock 0 first divider status
 *
 * This function gets auxiliary clock 0 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider status
 */
static inline bool CGM_GetAC0_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC0_DC0_DE_MASK
    return (base->AC0_DC0 & MC_CGM_AC0_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 0 second divider status
 *
 * This function gets auxiliary clock 0 second divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 second divider status
 */
static inline bool CGM_GetAC0_DC1_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC0_DC1_DE_MASK
    return (base->AC0_DC1 & MC_CGM_AC0_DC1_DE_MASK) != 0U;
#else
    return false;
#endif
}


/*!
 * @brief Gets auxiliary clock 0 third divider status
 *
 * This function gets auxiliary clock 0 third divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 third divider status
 */
static inline bool CGM_GetAC0_DC2_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC0_DC2_DE_MASK
    return (base->AC0_DC2 & MC_CGM_AC0_DC2_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 0 fourth divider status
 *
 * This function gets auxiliary clock 0 fourth divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 fourth divider status
 */
static inline bool CGM_GetAC0_DC3_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC0_DC3_DE_MASK
    return (base->AC0_DC3 & MC_CGM_AC0_DC3_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 0 fifth divider status
 *
 * This function gets auxiliary clock 0 fifth divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 fifth divider status
 */
static inline bool CGM_GetAC0_DC4_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC0_DC4_DE_MASK
    return (base->AC0_DC4 & MC_CGM_AC0_DC4_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 1 first divider status
 *
 * This function gets auxiliary clock 1 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 1 first divider status
 */
static inline bool CGM_GetAC1_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC1_DC0_DE_MASK
    return (base->AC1_DC0 & MC_CGM_AC1_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}


/*!
 * @brief Gets auxiliary clock 1 second divider status
 *
 * This function gets auxiliary clock 1 second divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 1 second divider status
 */
static inline bool CGM_GetAC1_DC1_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC1_DC1_DE_MASK
    return (base->AC1_DC1 & MC_CGM_AC1_DC1_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 2 first divider status
 *
 * This function gets auxiliary clock 2 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 2 first divider status
 */
static inline bool CGM_GetAC2_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC2_DC0_DE_MASK
    return (base->AC2_DC0 & MC_CGM_AC2_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 5 first divider status
 *
 * This function gets auxiliary clock 5 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 5 second divider status
 */
static inline bool CGM_GetAC5_DC1_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC5_DC1_DE_MASK
    return (base->AC5_DC1 & MC_CGM_AC5_DC1_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 5 first divider status
 *
 * This function gets auxiliary clock 5 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider status
 */
static inline bool CGM_GetAC5_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC5_DC0_DE_MASK
    return (base->AC5_DC0 & MC_CGM_AC5_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 6 first divider status
 *
 * This function gets auxiliary clock 6 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider status
 */
static inline bool CGM_GetAC6_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC6_DC0_DE_MASK
    return (base->AC6_DC0 & MC_CGM_AC6_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 7 first divider status
 *
 * This function gets auxiliary clock 7 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider status
 */
static inline bool CGM_GetAC7_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC7_DC0_DE_MASK
    return (base->AC7_DC0 & MC_CGM_AC7_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 8 first divider status
 *
 * This function gets auxiliary clock 8 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 0 first divider status
 */
static inline bool CGM_GetAC8_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC8_DC0_DE_MASK
    return (base->AC8_DC0 & MC_CGM_AC8_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 9 first divider status
 *
 * This function gets auxiliary clock 9 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 9 first divider status
 */
static inline bool CGM_GetAC9_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC9_DC0_DE_MASK
    return (base->AC9_DC0 & MC_CGM_AC9_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 10 first divider status
 *
 * This function gets auxiliary clock 10 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 10 first divider status
 */
static inline bool CGM_GetAC10_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC10_DC0_DE_MASK
    return (base->AC10_DC0 & MC_CGM_AC10_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}


/*!
 * @brief Gets auxiliary clock 11 first divider status
 *
 * This function gets auxiliary clock 11 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 11 first divider status
 */
static inline bool CGM_GetAC11_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC11_DC0_DE_MASK
    return (base->AC11_DC0 & MC_CGM_AC11_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 12 first divider status
 *
 * This function gets auxiliary clock 12 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 12 first divider status
 */
static inline bool CGM_GetAC12_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC12_DC0_DE_MASK
    return (base->AC12_DC0 & MC_CGM_AC12_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 13 first divider status
 *
 * This function gets auxiliary clock 13 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 13 first divider status
 */
static inline bool CGM_GetAC13_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC13_DC0_DE_MASK
    return (base->AC13_DC0 & MC_CGM_AC13_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}

/*!
 * @brief Gets auxiliary clock 14 first divider status
 *
 * This function gets auxiliary clock 14 first divider status
 *
 * @param[in] base Register base address for the CGM instance.
 * @return auxiliary selector clock 14 first divider status
 */
static inline bool CGM_GetAC14_DC0_Status(const MC_CGM_Type * base)
{
    (void)base;

#ifdef MC_CGM_AC13_DC0_DE_MASK
    return (base->AC14_DC0 & MC_CGM_AC14_DC0_DE_MASK) != 0U;
#else
    return false;
#endif
}


/*!
 * @brief Sets CGM clock output 0
 *
 * This function sets CGM clock output 0
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] selector Selector value.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetClkout0(MC_CGM_Type * base, bool enable, uint32_t selector, uint32_t divider)
{
    (void)base;
    (void)enable;
    (void)selector;
    (void)divider;

#if defined(MC_CGM_CLKOUT0_SC_SELCTL) && defined(MC_CGM_CLKOUT0_DC_DIV) && defined(MC_CGM_CLKOUT0_DC_DE)
    if(enable)
    {
        base->CLKOUT0_SC = MC_CGM_CLKOUT0_SC_SELCTL(selector);
        base->CLKOUT0_DC = MC_CGM_CLKOUT0_DC_DIV(divider) |
                           MC_CGM_CLKOUT0_DC_DE(1UL);
    }
    else
    {
        base->CLKOUT0_SC = 0UL;
        base->CLKOUT0_DC = 0UL
    }
#endif
}

/*!
 * @brief Sets CGM clock output 1
 *
 * This function sets CGM clock output 1
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] selector Selector value.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetClkout1(MC_CGM_Type * base, bool enable, uint32_t selector, uint32_t divider)
{
    (void)base;
    (void)enable;
    (void)selector;
    (void)divider;

#if defined(MC_CGM_CLKOUT1_SC_SELCTL) && defined(MC_CGM_CLKOUT1_DC_DIV) && defined(MC_CGM_CLKOUT1_DC_DE)
    if(enable)
    {
        base->CLKOUT1_SC = MC_CGM_CLKOUT1_SC_SELCTL(selector);
        base->CLKOUT1_DC = MC_CGM_CLKOUT1_DC_DIV(divider) |
                           MC_CGM_CLKOUT1_DC_DE(1UL);
    }
    else
    {
        base->CLKOUT1_SC = 0UL;
        base->CLKOUT1_DC = 0UL;
    }
#endif
}


/*!
 * @brief Gets CGM clock output 1 status
 *
 * This function gets CGM clock output 1 status
 *
 * @param[in] base     Register base address for the CGM instance.
*/
static inline bool CGM_GetClkout1_Status(MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_CLKOUT1_DC_DE_MASK)
	return ((base->CLKOUT1_DC & MC_CGM_CLKOUT1_DC_DE_MASK) != 0U);
#else
	return false;
#endif
}

/*!
 * @brief Gets CGM clock output 1 selector
 *
 * This function gets CGM clock output 1 selector
 *
 * @param[in] base     Register base address for the CGM instance.
*/
static inline uint32_t CGM_GetClkout1_SelValue(MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_CLKOUT1_SC_SELCTL_MASK) && defined(MC_CGM_CLKOUT1_SC_SELCTL_SHIFT)
	return ((base->CLKOUT1_SC & MC_CGM_CLKOUT1_SC_SELCTL_MASK) >> MC_CGM_CLKOUT1_SC_SELCTL_SHIFT);
#else
	return 0U;
#endif
}

/*!
 * @brief Gets CGM clock output 1 divider value
 *
 * This function gets CGM clock output 1 divider value
 *
 * @param[in] base     Register base address for the CGM instance.
*/
static inline uint32_t CGM_GetClkout1_DividerValue(MC_CGM_Type * base)
{
    (void)base;

#if defined(MC_CGM_CLKOUT1_DC_DIV_MASK) && defined(MC_CGM_CLKOUT1_DC_DIV_SHIFT)
	return ((base->CLKOUT1_DC & MC_CGM_CLKOUT1_DC_DIV_MASK) >> MC_CGM_CLKOUT1_DC_DIV_SHIFT);
#else
	return 0U;
#endif
}



/*!
 * @brief Sets CGM system clock first divider
 *
 * This function sets CGM system clock first divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC0(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER0_MASK) != 0U
    base->SC_DC0 = MC_CGM_SC_DC0_DIV(divValue) |
                   MC_CGM_SC_DC0_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 2nd divider
 *
 * This function sets CGM system clock 2nd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC1(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER1_MASK) != 0U
    base->SC_DC1 = MC_CGM_SC_DC1_DIV(divValue) |
                   MC_CGM_SC_DC1_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 3rd divider
 *
 * This function sets CGM system clock 3rd divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC2(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER2_MASK) != 0U
    base->SC_DC2 = MC_CGM_SC_DC2_DIV(divValue) |
                   MC_CGM_SC_DC2_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 4th divider
 *
 * This function sets CGM system clock 4th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC3(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER3_MASK) != 0U
    base->SC_DC3 = MC_CGM_SC_DC3_DIV(divValue) |
    		       MC_CGM_SC_DC3_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 5th divider
 *
 * This function sets CGM system clock 5th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC4(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER4_MASK) != 0U
    base->SC_DC4 = MC_CGM_SC_DC4_DIV(divValue) |
    		MC_CGM_SC_DC4_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 6th divider
 *
 * This function sets CGM system clock 6th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC5(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER5_MASK) != 0U
    base->SC_DC5 = MC_CGM_SC_DC5_DIV(divValue) |
                   MC_CGM_SC_DC5_DE(enable ? 1UL : 0UL);
#endif
}

/*!
 * @brief Sets CGM system clock 7th divider
 *
 * This function sets CGM system clock 7th divider
 *
 * @param[in] base     Register base address for the CGM instance.
 * @param[in] enable   Enable divider.
 * @param[in] divValue Divider value.
 */
static inline void CGM_SetSC_DC6(MC_CGM_Type * base, bool enable, uint32_t divValue)
{
    (void)base;
    (void)enable;
    (void)divValue;

#if (SYSTEM_CLOCK_DIVIDERS & SYSTEM_CLOCK_DIVIDER6_MASK) != 0U
    base->SC_DC6 = MC_CGM_SC_DC6_DIV(divValue) |
                   MC_CGM_SC_DC6_DE(enable ? 1UL : 0UL);
#endif
}

#if defined(MCB)
/*!
 * @brief Sets clkouts in MCB
 *
 * This function sets clkouts in MCB
 *
 * @param[in] base     Register base address for the MCB instance.
 * @param[in] clkout0  clkout0.
 * @param[in] clkout1  clkout1.
 */
static inline void MCB_SetClkout0Clkout1(MCB_Type * base, uint32_t clkout0, uint32_t clkout1)
{
#if defined(MCB_CLKOUT_SEL_CLKOUT0) && defined(MCB_CLKOUT_SEL_CLKOUT1)
    uint32_t value = base->CLKOUT_SEL;
    value &= ~MCB_CLKOUT_SEL_CLKOUT0_MASK;
    value |= MCB_CLKOUT_SEL_CLKOUT0(clkout0);
    value &= ~MCB_CLKOUT_SEL_CLKOUT1_MASK;
    value |= MCB_CLKOUT_SEL_CLKOUT1(clkout1);
    base->CLKOUT_SEL = value;
#else
	(void)base;
    (void)clkout0;
    (void)clkout1;
#endif
}


static inline uint32_t MCB_GetClkout0Clkout1(MCB_Type *base, uint32_t instance)
{

	(void)base;
	uint32_t selector = 0U;

	switch (instance)
	{

	case 0U:
#if defined(MCB_CLKOUT_SEL_CLKOUT0_MASK) && defined(MCB_CLKOUT_SEL_CLKOUT0_SHIFT)
	selector = ((base->CLKOUT_SEL & MCB_CLKOUT_SEL_CLKOUT0_MASK) >> MCB_CLKOUT_SEL_CLKOUT0_SHIFT);
#else
	selector = 0U;
#endif
	break;

	case 1U:
#if defined(MCB_CLKOUT_SEL_CLKOUT1_MASK) && defined(MCB_CLKOUT_SEL_CLKOUT1_SHIFT)
	selector = ((base->CLKOUT_SEL & MCB_CLKOUT_SEL_CLKOUT1_MASK) >> MCB_CLKOUT_SEL_CLKOUT1_SHIFT);
#else
	selector = 0U;
#endif
	break;

	default:
		selector = 0U;
		break;
	}
	return selector;
}
#endif


#if defined(__cplusplus)
}
#endif /* __cplusplus*/


/*! @}*/

#endif /* CGM_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
