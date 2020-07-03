/*
 * Copyright 2017-2018 NXP.
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
 * @file fccu_hw_access.h
 */

#ifndef FCCU_HW_ACCESS_H
#define FCCU_HW_ACCESS_H

#include "fccu_driver.h"

/*!
 * @brief Fault Collection and Control Unit Hardware Access layer.
 * @{
 */

/*******************************************************************************
* API
*******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/* FCCU Control */

#if FEATURE_FCCU_FILTER_EN
/*!
 * @brief Set Filter Bypass Setting
 * @param[in] base          FCCU base pointer
 */
static inline void FCCU_SetFilterByPass(FCCU_Type * const base,
                                        bool enable)
{
    base->CTRL = (base->CTRL & ~FCCU_CTRL_FILTER_BYPASS_MASK)
               | FCCU_CTRL_FILTER_BYPASS(enable ? 1UL : 0UL);
}

/*!
 * @brief Set Filter Width Setting
 * @param[in] base      FCCU base pointer
 * @param[in] width     filter width
 */
static inline void FCCU_SetFilterWidth(FCCU_Type * const base,
                                       fccu_filterwidth_t width)
{
    base->CTRL = (base->CTRL & ~FCCU_CTRL_FILTER_WIDTH_MASK)
               | FCCU_CTRL_FILTER_WIDTH(width);
}
#endif /* FEATURE_FCCU_FILTER_EN */

/*!
 * @brief Set Debug Mode Setting
 * @param[in] base      FCCU base pointer
 * @param[in] enable    enable/disable debug mode
 */
static inline void FCCU_SetDebugMode(FCCU_Type * const base,
                                     bool enable)
{
    base->CTRL = (base->CTRL & ~FCCU_CTRL_DEBUG_MASK)
               | FCCU_CTRL_DEBUG(enable ? 1UL : 0UL);
}

/*!
 * @brief Get Operation Status
 * @param[in] base      FCCU base pointer
 * @return Operation Status
 */
static inline fccu_op_status_t FCCU_GetOperationStatus(const FCCU_Type * const base)
{
    fccu_op_status_t status;
    switch ((base->CTRL & FCCU_CTRL_OPS_MASK) >> FCCU_CTRL_OPS_SHIFT)
    {
        case 0U:
            status = FCCU_OP_IDLE;
            break;
        case 1U:
            status = FCCU_OP_INPROGRESS;
            break;
        case 2U:
            status = FCCU_OP_ABORTED;
            break;
        default:
            status = FCCU_OP_SUCCESSFUL;
            break;
    }

    return status;
}

/*!
 * @brief Set Operation
 * @param[in] base      FCCU base pointer
 * @param[in] state     Operation to run
 */
static inline void FCCU_SetOperationRun(FCCU_Type * const base,
                                        fccu_run_op_t state)
{
    base->CTRL = (base->CTRL & ~FCCU_CTRL_OPR_MASK)
               | FCCU_CTRL_OPR(state);
}

/*!
 * @brief Set Control Key
 * @param[in] base      FCCU base pointer
 * @param[in] key       Value of Key
 */
static inline void FCCU_SetControlKey(FCCU_Type * const base,
                                      uint32_t key)
{
    base->CTRLK = key;
}

/* FCCU Configuration */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
/*!
 * @brief Set Fault Toggle Time
 * @param[in] base      FCCU base pointer
 * @param[in] value     Value of Toggle Time
 */
static inline void FCCU_SetFaultOutputTogglingTime(FCCU_Type * const base,
                                                   uint8_t value)
{
    base->CFG = (base->CFG & ~FCCU_CFG_TGL_MASK) | FCCU_CFG_TGL(value);
}
#endif
/*!
 * @brief Set Fault Output Active
 * @param[in] base      FCCU base pointer
 * @param[in] enable    enable/disable
 */
static inline void FCCU_SetFaultOutputActivate(FCCU_Type * const base,
                                               bool enable)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FCCU_SET_AFTER_RESET_MASK)
               | FCCU_CFG_FCCU_SET_AFTER_RESET(enable ? 1UL : 0UL);
}

/*!
 * @brief Set Fault Output Control
 * @param[in] base      FCCU base pointer
 * @param[in] control   Type of control
 */
static inline void FCCU_SetFaultOutputControl(FCCU_Type * const base,
                                              fccu_fo_control_t control)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FCCU_SET_CLEAR_MASK)
               | FCCU_CFG_FCCU_SET_CLEAR(control);
}

#if FEATURE_FCCU_RCC_EN
/*!
 * @brief Set Self-check FCCU’s redundancy control checker 
 * @param[in] base     FCCU base pointer
 * @param[in] setRcc   Set FCCU’s redundancy control checker
 */
static inline void FCCU_SetRedundancyControlChecker(FCCU_Type * const base,
                                                    fccu_rcc_config_t setRcc)
{
    /* Set FCCU's redundancy control checker 0 and 1 */
    base->CFG = (base->CFG & ~(FCCU_CFG_RCCE0_MASK | FCCU_CFG_RCCE1_MASK)) | (((uint32_t)setRcc & 3UL) << FCCU_CFG_RCCE0_SHIFT);
}
#endif

#if !defined(FEATURE_FCCU_FOP_SUPPORT)
/*!
 * @brief Set Fault Output Prescaler Extension
 * @param[in] base      FCCU base pointer
 * @param[in] enable    Enable/Disable
 */
static inline void FCCU_SetFaultOutputPrescalerExtension(FCCU_Type * const base,
                                                         bool enable)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FOPE_MASK)
               | FCCU_CFG_FOPE(enable ? 1UL : 0UL);
}

/*!
 * @brief Set Fault Output Prescaler
 * @param[in] base       FCCU base pointer
 * @param[in] value      Value of prescaler
 */
static inline void FCCU_SetFaultOutputPrescaler(FCCU_Type * const base,
                                                uint8_t value)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FOP_MASK)
               | FCCU_CFG_FOP(value);
}
#endif /* FEATURE_FCCU_FOP_SUPPORT */

#if FEATURE_FCCU_OPEN_DRAIN_EN
/*!
 * @brief Set Fault Output Open Drain
 * @param[in] base      FCCU base pointer
 * @param[in] enable    Enable -> OpenDrain  / Disable -> PushPull
 */
static inline void FCCU_SetOpenDrain(FCCU_Type * const base,
                                     bool enable)
{
    base->CFG = (base->CFG & ~FCCU_CFG_OD_MASK)
               | FCCU_CFG_OD(enable ? 1UL : 0UL);
}
#endif /* FEATURE_FCCU_OPEN_DRAIN_EN */

#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
/*!
 * @brief Set Fault Output Mode Extension
 * @param[in] base      FCCU base pointer
 * @param[in] mode      Type of mode behavior
 */
static inline void FCCU_SetFaultOutputExtension(FCCU_Type * const base,
                                                fccu_fo_extension_t mode)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FOME_MASK)
               | FCCU_CFG_FOME(mode);
}
#endif /* FEATURE_FCCU_IRQ_EN_MASK */

/*!
 * @brief Set Fault Output Switch Mode
 * @param[in] base      FCCU base pointer
 * @param[in] mode      Enable -> Fast / Disable -> Slow
 */
static inline void FCCU_SetFaultOutputSwitchingMode(FCCU_Type * const base,
                                                    bool mode)
{
    base->CFG = (base->CFG & ~FCCU_CFG_SM_MASK)
               | FCCU_CFG_SM(mode ? 1UL : 0UL);
}

/*!
 * @brief Set Fault Output Switch Mode
 * @param[in] base       FCCU base pointer
 * @param[in] polarity
 */
static inline void FCCU_SetFaultOutputPolaritySelection(FCCU_Type * const base,
                                                        bool polarity)
{
    base->CFG = (base->CFG & ~FCCU_CFG_PS_MASK)
               | FCCU_CFG_PS(polarity ? 1UL : 0UL);
}

/*!
 * @brief Set Fault Output Mode
 * @param[in] base       FCCU base pointer
 * @param[in] mode       Type of operation mode
 */
static inline void FCCU_SetFaultOutputMode(FCCU_Type * const base,
                                           fccu_fo_mode_t mode)
{
    base->CFG = (base->CFG & ~FCCU_CFG_FOM_MASK)
               | FCCU_CFG_FOM(mode);
}

/* Noncritical Fault Configuration */

/*!
 * @brief Set NCF_CFG
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 * @param[in] config        Type of Recovery fault
 */
void FCCU_SetNCFConfig(FCCU_Type * const base,
                       uint8_t faultIndex,
                       fccu_ncf_config_t config);

/*!
 * @brief Sets NCFS_CFG
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 * @param[in] config        Type of Reset for fault
 */
void FCCU_SetNCFStatusConfig(FCCU_Type * const base,
                             uint8_t faultIndex,
                             fccu_ncfs_config_t config);

/*!
 * @brief Gets NCF_CFG
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 * @return true -> if fault set
 *         false-> if fault not set
 */
bool FCCU_GetNCFStatus(const FCCU_Type * const base,
                       uint8_t faultIndex);

/*!
 * @brief Clear Fault NCF_CFG
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 */
void FCCU_ClearNCFStatus(FCCU_Type * const base,
                         uint8_t faultIndex);

/*!
 * @brief Get Fault Status NCF_S
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 * @return  true -> fault triggered
 *          false -> fault not triggered
 */
bool FCCU_GetNCFsStatus(const FCCU_Type * const base,
                        uint8_t faultIndex);

/*!
 * @brief Clear Fault Status NCF_S
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 */
void FCCU_ClearNCFsStatus(FCCU_Type * const base,
                          uint8_t faultIndex);

/*!
 *  @brief Set NonCritical Fault Key
 *
 *  @param[in] base FCCU base pointer
 *  @param[in] key  value of key
 *
 */
static inline void FCCU_SetNCFKey(FCCU_Type * const base,
                                  uint32_t key)
{
    base->NCFK = key;
}

/*!
 * @brief Enable Fault to trigger Alarm\Fault State NCF_E
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 */
void FCCU_SetNCFEnable(FCCU_Type * const base,
                       uint8_t faultIndex,
                       bool enable);

/*!
 * @brief Time Out Fault to trigger Alarm -> Fault State NCF_TOE
 * @param[in] base          FCCU base pointer
 * @param[in] faultIndex    Number of fault
 */
void FCCU_SetNCFTimeoutEnable(FCCU_Type * const base,
                             uint8_t faultIndex,
                             bool enable);

/*!
 *  @brief Set Time Out Alarm to Fault
 *
 *  @param[in] base  FCCU base pointer
 *  @param[in] value Value of timer
 *
 */
static inline void FCCU_SetNCFTimeout(FCCU_Type * const base,
                                      uint32_t value)
{
    base->NCF_TO = value;
}

/*!
 *  @brief Set Configuration Time Allow
 *
 *  @param[in] base  FCCU base pointer
 *  @param[in] value Value of timer
 *
 */
static inline void FCCU_SetTimerInterval(FCCU_Type * const base,
                                         uint8_t value)
{
    base->CFG_TO = (base->CFG_TO & ~FCCU_CFG_TO_TO_MASK)
                 | FCCU_CFG_TO_TO(value);
}

/*!
 *  @brief Get FCCU Status
 *
 *  @param[in] base FCCU base pointer
 *  @return  State of FCCU
 */
static inline fccu_status_t FCCU_GetStatus(const FCCU_Type * const base)
{
    fccu_status_t status;

    switch ((base->STAT & FCCU_STAT_STATUS_MASK) >> FCCU_STAT_STATUS_SHIFT)
    {
        case 0U:
            status = FCCU_STATUS_NORMAL;
            break;
        case 1U:
            status = FCCU_STATUS_CONFIGURATION;
            break;
        case 2U:
            status = FCCU_STATUS_ALARM;
            break;
        default:
            status = FCCU_STATUS_FAULT;
            break;
    }

    return status;
}

/* FCCU Freeze status */
/*!
 * @brief Return Freeze Status
 * @param[in] base          FCCU base pointer
 * @param[in] type          Type of freeze status register
 * @return Fault number
 */
uint8_t FCCU_GetFreezeStatus(const FCCU_Type * const base,
                             fccu_freeze_type_t type);

/* Fake fault */
/*!
 *  @brief Trigger Fake NonCritical Fault
 *
 *  @param[in] base       FCCU base pointer
 *  @param[in] faultIndex    Number of fault
 */
static inline void FCCU_SetNCFFake(FCCU_Type * const base,
                                   uint8_t faultIndex)
{
    base->NCFF = (base->NCFF & ~FCCU_NCFF_FNCFC_MASK)
               | FCCU_NCFF_FNCFC(faultIndex);
}

/* FCCU IRQ */

/*!
 * @brief Get Interrupt Status
 * @param[in] base          FCCU base pointer
 * @param[in] intStatus     Type of interrupt
 * @return true -> interrupt type triggered
 *         false -> interrupt type not trigger
 */
bool FCCU_GetIntStatus(const FCCU_Type * const base,
                       fccu_int_status_t intStatus);

/*!
 * @brief Clear Interrupt Status
 * @param[in] base          FCCU base pointer
 * @param[in] intStatus     Type of interrupt
 */
void FCCU_ClearIntFlag(FCCU_Type * const base,
                       fccu_int_status_t intStatus);

/*!
 * @brief Enable Global Interrupts
 * @param[in] base          FCCU base pointer
 * @param[in] intStatus     Type of interrupt
 */
void FCCU_SetIntEnable(FCCU_Type * const base,
                       fccu_int_status_t intStatus,
                       bool enable);

/*!
 * @brief Enable Interrupt Alarm
 * @param[in] base          FCCU base pointer
 * @param[in] intStatus     Type of Alarm interrupt
 * @param[in] faultIndex    Number of fault
 * @param[in] enable        enable/disable
 */
void FCCU_SetIntSource(FCCU_Type * const base,
                       fccu_int_status_t intStatus,
                       uint8_t faultIndex,
                       bool enable);

/* FCCU Timer */
/*!
 *  @brief Get Alarm/ Watchdog Timer
 *
 *  @param[in] base FCCU base pointer
 *  @return value of timer counter
 *
 */
static inline uint32_t FCCU_GetCounterValue(const FCCU_Type * const base)
{
    return base->XTMR;
}

#if FEATURE_FCCU_CONTROL_MODE_EN
/* FCCU Mode */
/*!
 * @brief Get Mode Controller Status
 * @param[in] base          FCCU base pointer
 * @param[in] recentMode    Recent Mode Value
 * @param[out] infoPtr      pointer to save location structure
 */
void FCCU_GetModeControllerStatus(const FCCU_Type * const base,
                                  fccu_chip_mode_t recentMode,
                                  fccu_mode_info_t * infoPtr);
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

/* FCCU Configuration Lock */
/*!
 *  @brief Set Transient Lock
 *
 *  @param[in] base   FCCU base pointer
 *  @param[in] enable Enable / Disable
 *
 */
static inline void FCCU_SetTransientConfigLock(FCCU_Type * const base,
                                               bool enable)
{
    base->TRANS_LOCK = (base->TRANS_LOCK & FCCU_TRANS_LOCK_TRANSKEY_MASK)
                     | FCCU_TRANS_LOCK_TRANSKEY(enable ? 0UL : FEATURE_FCCU_TRANS_UNLOCK);
}

/*!
 *  @brief Set Permanent Lock
 *
 *  @param[in] base FCCU base pointer
 *
 */
static inline void FCCU_SetPermanentConfigLock(FCCU_Type * const base)
{
    base->PERMNT_LOCK = (base->PERMNT_LOCK & FCCU_PERMNT_LOCK_PERMNTKEY_MASK)
                     | FCCU_PERMNT_LOCK_PERMNTKEY(FEATURE_FCCU_PERMNT_LOCK);
}

/* FCCU Delta T */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
/*!
 *  @brief Set IRQDMA trigger time
 *
 *  @param[in] base  FCCU base pointer
 *  @param[in] value trigger time
 *
 */
static inline void FCCU_SetIRQDMATriggerTime(FCCU_Type * const base,
                                             uint16_t value)
{
    base->DELTA_T = (base->DELTA_T & ~FCCU_DELTA_T_T_TRG_MASK)
                  | FCCU_DELTA_T_T_TRG(value);
}
#endif

/*!
 *  @brief Set Bistable Minimum Fault-Output Faulty Interval
 *
 *  @param[in] base  FCCU base pointer
 *  @param[in] value
 *
 */
static inline void FCCU_SetBiFOFaultyInterval(FCCU_Type * const base,
                                              uint16_t value)
{
    base->DELTA_T = (base->DELTA_T & ~FCCU_DELTA_T_DELTA_T_MASK)
                  | FCCU_DELTA_T_DELTA_T(value);
}

#if defined (__cplusplus)
}
#endif

/*! @} */

#endif /* FCCU_HW_ACCESS_H */
/*******************************************************************************
* EOF
*******************************************************************************/
