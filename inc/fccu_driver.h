/*
 * Copyright 2017-2019 NXP.
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
 * @file fccu_driver.h
 */

#ifndef FCCU_DRIVER_H
#define FCCU_DRIVER_H

#include "status.h"
#include "device_registers.h"
#include "interrupt_manager.h"

/* */
/* */
/* */

/*!
 * @defgroup fccu_drv fccu Driver
 * @ingroup fccu
 * @brief Fault Collection and Control Unit Peripheral Driver.
 * @addtogroup fccu_drv
 * @{
 */

/*******************************************************************************
* Definitions
*******************************************************************************/

/* Time Out Value in milliseconds for running an operation */
#define FCCU_TIME_DELAY           (10U)

/*!
 * @brief Implements Clear All Faults Symbol
 * Implements : FEATURE_FCCU_CLEAR_ALL_FAULTS_Class
 */
#define FEATURE_FCCU_CLEAR_ALL_FAULTS          (uint8_t)(FEATURE_FCCU_MAX_FAULTS_NO + 1U)


/*!
 * @brief Timer Types
 * Implements : fccu_xtmr_type_t_Class
 */
typedef enum
{
    FCCU_XTMR_ALARM  = 0U, /*!< Timer Set for Alarm State */
    FCCU_XTMR_CONFIG = 1U, /*!< Timer Set for Configuration State */
    FCCU_XTMR_ETMR   = 2U  /*!< Timer Set for Fault State */
} fccu_xtmr_type_t;
/*!
 * @brief Lock Types of Configuration
 * Implements : fccu_lock_type_t_Class
 */
typedef enum
{
    FCCU_LOCK_TYPE_NO_LOCK   = 0U,   /*!< No lock on the configuration */
    FCCU_LOCK_TYPE_TEMPORARY = 1U,   /*!< Temporary lock on configuration */
    FCCU_LOCK_TYPE_PERMANENT = 2U    /*!< Permanent lock on configuration */
} fccu_lock_type_t;

#if FEATURE_FCCU_FILTER_EN
/*!
 * @brief Filter Wide of Configuration
 * Implements : fccu_filterwidth_t_Class
 */
typedef enum
{
    FCCU_FILTERWIDTH_UP_TO_50_US  = 0U,    /*!< Filters glitches up to 50 us */
    FCCU_FILTERWIDTH_UP_TO_75_US  = 1U,    /*!< Filters glitches up to 75 us */
    FCCU_FILTERWIDTH_UP_TO_100_US = 2U     /*!< Filters glitches up to 100 us */
} fccu_filterwidth_t;
#endif /* FEATURE_FCCU_FILTER_EN */

/*!
 * @brief State of an executed Operation
 * Implements : fccu_op_status_t_Class
 */
typedef enum
{
    FCCU_OP_IDLE       = 0U,    /*!< FCCU Operation Idle */
    FCCU_OP_INPROGRESS = 1U,    /*!< FCCU Operation In progress */
    FCCU_OP_ABORTED    = 2U,    /*!< FCCU Operation Aborted */
    FCCU_OP_SUCCESSFUL = 3U     /*!< FCCU Operation Successful */
} fccu_op_status_t;

/*!
 * @brief Operations that can be executed by FCCU
 * Implements : fccu_run_op_t_Class
 */
typedef enum
{
    FCCU_RUN_OP0     = 0U,     /*!< No operation */
    FCCU_RUN_OP1     = 1U,     /*!< Put FCCU in Configuration state */
    FCCU_RUN_OP2     = 2U,     /*!< Put FCCU in Normal state */
    FCCU_RUN_OP3     = 3U,     /*!< Capture FCCU state and the states of the EOUT signals */
    FCCU_RUN_OP4     = 4U,     /*!< Read the FCCU frozen status flags Normal - Alarm */
    FCCU_RUN_OP5     = 5U,     /*!< Read the FCCU frozen status flags Alarm - Fault */
    FCCU_RUN_OP6     = 6U,     /*!< Read the FCCU frozen status flags Normal - Fault */
    FCCU_RUN_OP7     = 7U,     /*!< Read the FCCU frozen status flags Fault - Alarm */
#if FEATURE_FCCU_RCC_EN
    FCCU_RUN_OP8     = 8U,     /*!< Read the FCCU frozen status flags Self-checking RCC */
#endif
    FCCU_RUN_OP10    = 10U,    /*!< Read the Noncritical Fault Status */
    FCCU_RUN_OP12    = 12U,    /*!< Do not initiate this operation; it is automatically initiated by FCCU.
                                *   A Noncritical Fault Status register status clear operation is in progress*/
    FCCU_RUN_OP13    = 13U,    /*!< Clear the freeze status registers */
    FCCU_RUN_OP14    = 14U,    /*!< Do not initiate this operation; it is automatically initiated by FCCU.
                                *   A Configuration-state timeout is in progress */
    FCCU_RUN_OP15    = 15U,    /*!< Set the operation status */
    FCCU_RUN_OP17    = 17U,    /*!< Read the Alarm-state timer */
    FCCU_RUN_OP19    = 19U,    /*!< Read the Configuration-state timer */
    FCCU_RUN_OP20    = 20U,    /*!< Read the Error Pin low counter value */
    FCCU_RUN_OP31    = 31U     /*!< Load configuration data from the NVMCFG signals to certain fields
                                *    in the Configuration register, but without resetting FCCU (for testing and debugging) */
} fccu_run_op_t;


/*!
 * @brief Fault Output Control for EOUT Signals
 * Implements : fccu_fo_control_t_Class
 */
typedef enum
{
    FCCU_FO_CONTROLLED_BY_FSM      = 0U,    /*!< Controlled by the FSM */
    FCCU_FO_ALLWAY_LOW             = 1U,    /*!< Always low */
    FCCU_FO_HIGH_UNTIL_FALUT_OCCUR = 3U     /*!< High until a fault occurs on a channel, regardless of whether
                                             *    that fault is disabled; thereafter, controlled by the FSM */
} fccu_fo_control_t;

#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
/*!
 * @brief Fault Output Mode Extension for EOUT Signals
 * Implements : fccu_fo_extension_t_Class
 */
typedef enum
{
    FCCU_FO_EXT_DEFAULT = 0U,    /*!< Current behavior */
    FCCU_FO_EXT_OPTION1 = 1U,    /*!< HW interrupt trigger asserted by the FCCU
                                  *    when EOUT reaches T_Trig value when in NORMAL/ALARM state */
    FCCU_FO_EXT_OPTION2 = 2U,    /*!< DMA request asserted by the FCCU
                                  *    when EOUT reaches T_Trig value when in NORMAL/ALARM state */
    FCCU_FO_EXT_OPTION3 = 3U     /*!< Only if in Dual-Rail mode. EOUT protocol is SW controlled in NORMAL/ALARM state
                                  *    and HW controlled in FAULT state (dual rail, in phase toggling with programmable delay)
                                  *    following a HW interrupt trigger asserted by the FCCU
                                  *    when EOUT reaches T_Trig value when in NORMAL/ALARM state */
} fccu_fo_extension_t;
#endif /* FEATURE_FCCU_IRQ_EN_MASK */

/*!
 * @brief Fault Output Mode for EOUT Signals
 * Implements : fccu_fo_mode_t_Class
 */
typedef enum
{
    FCCU_FO_DUAL_RAIL      = 0U,    /*!< Dual-Rail (default state) */
    FCCU_FO_TIME_SWITCHING = 1U,    /*!< Time-Switching */
    FCCU_FO_BISTABLE       = 2U,    /*!< Bistable */
    FCCU_FO_TEST0          = 5U,    /*!< Test 0 (controlled by the FCCU_EINOUT register;
                                     *    EOUT1 is an output; EOUT0 is an input) */
    FCCU_FO_TEST1          = 6U,    /*!< Test 1 (controlled by the FCCU_EINOUT register;
                                     *    EOUT1 and EOUT0 are both outputs) */
    FCCU_FO_TEST2          = 7U     /*!< Test 2 (controlled by the FCCU_EINOUT register;
                                     *    EOUT1 is an input; EOUT0 is an output) */
} fccu_fo_mode_t;

/*!
 * @brief Type of recover for NonCritical Faults
 * Implements : fccu_ncf_config_t_Class
 */
typedef enum
{
    FCCU_NCF_HW_REC_FAULT = 0U,    /*!< Hardware-recoverable fault */
    FCCU_NCF_SW_REC_FAULT = 1U     /*!< Software-recoverable fault */
} fccu_ncf_config_t;

/*!
 * @brief Type of Reset for NonCritical Faults
 * Implements : fccu_ncfs_config_t_Class
 */
typedef enum
{
    FCCU_NCFS_NO_RESET    = 0U,    /*!< No reset reaction */
    FCCU_NCFS_SHORT_RESET = 1U,    /*!< Short functional reset request pulse */
    FCCU_NCFS_LONG_RESET  = 2U     /*!< Long functional reset request pulse */
} fccu_ncfs_config_t;

/*!
 * @brief EOUT IO Control Phase Type
 * Describe Field EOUTX from FCCU_EINOUT register
 * Implements : fccu_fo_phase_t_Class
 */
typedef enum
{
    FCCU_FO_OPPOSITE_PHASE_10 = 0U,    /*!< Opposite phase: EOUT1 = 1 xor PS, EOUT0 = 0 xor PS */
    FCCU_FO_OPPOSITE_PHASE_01 = 1U,    /*!< Opposite phase: EOUT1 = 0 xor PS, EOUT0 = 1 xor PS */
    FCCU_FO_IN_PHASE_00       = 2U,    /*!< In phase: EOUT1 = 0 xor PS, EOUT0 = 0 xor PS */
    FCCU_FO_IN_PHASE_11       = 3U     /*!< In phase: EOUT1 = 1 xor PS, EOUT0 = 1 xor PS */
} fccu_fo_phase_t;

/*!
 * @brief State of the FCCU Module
 * Implements : fccu_status_t_Class
 */
typedef enum
{
    FCCU_STATUS_NORMAL        = 0U,    /*!< FCCU Normal */
    FCCU_STATUS_CONFIGURATION = 1U,    /*!< FCCU Configuration */
    FCCU_STATUS_ALARM         = 2U,    /*!< FCCU Alarm */
    FCCU_STATUS_FAULT         = 3U,    /*!< FCCU Fault */
} fccu_status_t;

/*!
 * @brief Type of Freeze Status
 * Implements : fccu_freeze_type_t_Class
 */
typedef enum
{
    FCCU_FRZ_NORMAL_ALARM   = 0U,    /*!< The state transition from the NORMAL state to the ALARM state */
    FCCU_FRZ_ALARM_FAULT    = 1U,    /*!< The state transition from the ALARM state to the FAULT state  */
    FCCU_FRZ_NORMAL_FAULT   = 2U,    /*!< The state transition from the NORMAL state to the FAULT state */
    FCCU_FRZ_FAULT_ALARM    = 3U,    /*!< The state transition from the FAULT state to the ALARM state  */
#if FEATURE_FCCU_RCC_EN
    FCCU_FRZ_SELF_CHECK_RCC = 4U     /*!< The state Self-checking reaction (interrupt) from RCC         */
#endif
} fccu_freeze_type_t;

/*!
 * @brief Status of Interrupt Source
 * Implements : fccu_int_status_t_Class
 */
typedef enum
{
    FCCU_INT_TIMEOUT = 0U,    /*!< Status bit asserted when EOUT counter = T_TRG in NORMAL/ALARM mode */
    FCCU_INT_ALARM   = 1U,    /*!< Interrupt status bit asserted when EOUT counter = T_TRG in NORMAL/ALARM mode */
    FCCU_INT_NMI     = 2U,    /*!< NMI Interrupt Status */
    FCCU_INT_EOUT    = 3U,    /*!< Alarm Interrupt Status */
    FCCU_INT_DMA     = 4U     /*!< Configuration Timeout Status */
} fccu_int_status_t;

#if FEATURE_FCCU_CONTROL_MODE_EN
/*!
 * @brief Recent Chip Mode
 * Implements : fccu_chip_mode_t_Class
 */
typedef enum
{
    FCCU_CHIPMODE_MOST_RECENT        = 0U,    /*!< The most recent (current) chip mode */
    FCCU_CHIPMODE_SECOND_MOST_RECENT = 1U,    /*!< The second most recent chip mode */
    FCCU_CHIPMODE_THIRD_MOST_RECENT  = 2U,    /*!< The third most recent chip mode */
    FCCU_CHIPMODE_FOURTH_MOST_RECENT = 3U     /*!< The fourth most recent chip mode */
} fccu_chip_mode_t;
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

/*!
 * @brief IRQ Configuration Mode Supported
 * Implements : fccu_irq_en_type_t_Class
 */
typedef enum
{
    FCCU_IRQ_EN_NOIRQ            = 0U,  /*!< No Interrupts are configured */
    FCCU_IRQ_EN_CGF_TO           = 2U,  /*!< Time Out Interrupt configured */
    FCCU_IRQ_EN_EOUT             = 8U,  /*!< EOUT interrupt configured */
    FCCU_IRQ_EN_EOUT_WITH_CFG_TO = 9U   /*!< EOUT and Time Out Interrupt are configured */
} fccu_irq_en_type_t;

/*!
 * @brief Status of Interrupt Source Reported By State FCCU IRQ Structure
 * Implements : fccu_irq_status_t_Class
 */
typedef enum
{
    FCCU_ISR_NO          = 0U,   /*!< Status ISR No Interrupt Triggered */
    FCCU_ISR_IRQ_EOUT    = 1U,   /*!< Status ISR Eout Interrupt Triggered */
    FCCU_ISR_IRQ_NMI     = 2U,   /*!< Status ISR NMI Interrupt Triggered */
    FCCU_ISR_IRQ_ALARM   = 3U,   /*!< Status ISR Alarm Interrupt Triggered */
    FCCU_ISR_IRQ_TIMEOUT = 4U    /*!< Status ISR TimeOut Interrupt Triggered */
} fccu_irq_status_t;

/*!
 * @brief State of Semaphore
 * Implements : fccu_semaphore_t_Class
 */
typedef enum
{
    FCCU_SEMAPHORE_UNLOCK = 0U, /*!< State of Semaphore Unlocked */
    FCCU_SEMAPHORE_LOCK   = 1U  /*!< State of Semaphore Locked */
} fccu_semaphore_t;

#if FEATURE_FCCU_RCC_EN
/*!
 * @brief State of FCCU's redundancy control checker
 * Implements : fccu_rcc_config_t_Class
 */
typedef enum
{
    FCCU_RCC_NO   = 0U, /*!< Disable FCCU's RCC0 and RCC1        */
    FCCU_RCC0_EN  = 1U, /*!< Enable FCCU's RCC0 and disable RCC1 */
    FCCU_RCC1_EN  = 2U, /*!< Enable FCCU's RCC1 and disable RCC0 */
    FCCU_RCC12_EN = 3U  /*!< Enable FCCU's RCC0 and RCC1         */
} fccu_rcc_config_t;
#endif /* FEATURE_FCCU_RCC_EN */

#if FEATURE_FCCU_CONTROL_MODE_EN
/*!
 * @brief Structure for Recent Chip Mode
 * Implements : fccu_mode_info_t_Class
 */
typedef struct
{
    bool valid;             /*!< Access error master */
    bool faultStatus;       /*!< Access error master overrun */
    uint8_t mode;           /*!< Access error attributes */
} fccu_mode_info_t;
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

/*!
 * @brief Structure for State of ISR for FCCU
 * Implements : fccu_isr_state_t_Class
 */
typedef struct
{
    volatile fccu_semaphore_t  semaphore;                           /*!< Status of Semaphore */
             fccu_irq_status_t eventType;                           /*!< Source of interrupt trigger */
             uint8_t           ncfConfigNb;                         /*!< Number of configured NonCritical Faults */
             void          ( * callBackFunction )( void * param );  /*!< Callback Global Isr Function */
             void            * callbackParam;                       /*!< Parameter for Callback Global Isr Function */
} fccu_isr_state_t;

/*!
 * @brief Fault State Flags for NonCritical Faults
 * Implements : fccu_faults_flags_t_Class
 */
typedef struct
{
    uint32_t faultsFlags[FCCU_NCF_S_COUNT]; /*!< Faults Flags */
} fccu_faults_flags_t;


/*!
 * @brief EOUT configuration
 * Implements : fccu_eout_config_t_Class
 */
typedef struct
{
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
    uint8_t toggleTime;             /*!< Eout Toggle Time Measured in us, Max 128            */
    fccu_fo_extension_t extMode;    /*!< Fault Output Mode Extension Field From FCCU_CFG     */
    uint16_t triggerTimeIRQDMA;     /*!< IRQDMA trigger time                                 */
#endif
    bool activate;                  /*!< False -> EOUT Signals are High Impedance (inactive),
                                     *    True -> EOUT Signals driven by FCCU (active)       */
    fccu_fo_control_t control;      /*!< Apply when EOUT Active, FSM control State By FCCU   */
#if FEATURE_FCCU_RCC_EN
    fccu_rcc_config_t setRcc;       /*!< FCCU's redundancy control checker                   */
#endif
#if FEATURE_FCCU_OPEN_DRAIN_EN
    bool openDrain;                 /*!< False -> Push-Pull, True -> Open Drain of EOUT pins */
#endif
    bool switchMode;                /*!< False -> Slow, True -> Fast */
    bool polarity;                  /*!< Polarity Selection : False -> EOUT1 high, EOUT0 low
                                     *                         True -> EOUT1 low , EOUT0 high */
    fccu_fo_mode_t mode;            /*!< Fault Output Mode */
#if !defined(FEATURE_FCCU_FOP_SUPPORT)
    uint8_t prescaler;              /*!< Check the value and modify accordingly to implementation FOPE & FOP bit fields
                                     *    Max Allow Value for Prescaler is 127    */
#endif
    fccu_fo_phase_t phase;          /*!< EOUT SW or HW control accordingly to FSM  */
    uint16_t deltaFaultInterval;    /*!< Bistable Minimum Fault-Output (EOUT) Faulty Interval */
} fccu_eout_config_t;

/*!
 * @brief Global Driver FCCU Config
 * Implements : fccu_control_t_Class
 */
typedef struct
{
#if FEATURE_FCCU_FILTER_EN
    bool                   filterBypass;                    /*!< Activate Filter Bypass for FCCU */
    fccu_filterwidth_t     filterWidth;                     /*!< Set filter width */
#endif
    bool                   debugEnable;                     /*!< Activate Debug */
    fccu_irq_en_type_t     irqEnableType;                   /*!< Type of interrupt */
    uint32_t               ncfTimeout;                      /*!< Time Out Value TO*T(16Mhz) */
    fccu_eout_config_t   * configRun;                       /*!< Pointer to Configuration behavior for NCF EOUT and TO  */
    fccu_lock_type_t       lockType;                        /*!< Lock FCCU after Configuration */
    uint8_t                ncfConfigNumber;                 /*!< No of NCF configured */
    void               ( * callbackIsr )( void * param );   /*!< Global ISR Callback */
    void                 * callbackIsrParam;                /*!< Global ISR Callback Parameter*/
} fccu_control_t;


/*!
 * @brief Non Critical Fault Config
 * Implements : fccu_config_ncf_t_Class
 */
typedef struct
{
    uint8_t             functionID;                   /*!< Function Identification No by NonCritical Fault */
    fccu_ncf_config_t   hwSwRecovery;                 /*!< Type Of Recovery HW or SW  */
    fccu_ncfs_config_t  reset;                        /*!< reset Behavior */
    bool                timeoutEnable;                /*!< Activate Timeout for NonCritical Fault */
    uint8_t             reactionType;                 /*!< Set Reaction Type from fccu_int_status_t  */
    void            ( * callback )( void * param ) ;  /*!< Applicable only for Reaction Type FCCU_NCF_REACTION_IRQ_ALARM else NULL*/
    void              * callbackParam;                /*!< Alarm Callback Parameter */
} fccu_config_ncf_t;
/*******************************************************************************
* API
*******************************************************************************/
/*!
 * @name fccu Driver API
 * @{
 */
#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @brief Clear NonCritical Faults Status
 *
 * Need to clear the fault register NCF_S by setting bit position corresponding with
 * fault faultIndex. Any value higher than last faultNo will clear all faults
 * For MPC5777C: fault 41 cannot clear if Safety core is disabled, fault 42 cannot clear if Checker core or RCCUs are disabled.
 *
 * @param[in] instance          number of FCCU peripheral to clear fault.
 * @param[in] faultIdx          fault to clear or all faults mask.
 * @return  STATUS_SUCCESS in case of success clearing fault(s)
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_ClearFaults(const uint32_t instance,
                              uint8_t faultIdx);

/*!
 * @brief Read NonCritical Faults Status
 *
 * Update and Read NonCritical Faults Triggered.
 *
 * @param[in]   instance    number of FCCU instance.
 * @param[out]  param       pointer to the structure where to save the faults.
 */
void FCCU_DRV_ReadFaults(const uint32_t instance,
                         fccu_faults_flags_t * param);

/*!
 * @brief Run Operation issued by user.
 *
 * In some case operations need to unlock the FCCU to allow to execute it
 * An failsafe timeout protection for execution of operation is implemented
 *
 * @param[in] instance    number of FCCU instance.
 * @param[in] operation   operation to run.
 */
void FCCU_DRV_RunOperation(const uint32_t instance,
                           const fccu_run_op_t operation);

/*!
 * @brief Locks the configuration
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] lock       type of lock to apply.
 */
void FCCU_DRV_LockConfig(const uint32_t instance,
                         const fccu_lock_type_t lock);

/*!
 * @brief Set Event for triggered interrupt in FCCU IRQ Status
 *
 * This function sets the event without taking care of the semaphore status
 * in case of lock will override the status and set the new one and
 * unlocks the semaphore after operation completed.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] state      type of interrupt event to save
 */
void FCCU_DRV_IrqStatusSetEvent(const uint32_t  instance,
                                const fccu_irq_status_t state);

/*!
 * @brief Get Event for triggered interrupt in FCCU IRQ Status
 *
 * Checks the state of semaphore and returns the interrupt event.
 * An failsafe timeout protection for execution of operation is implemented
 * in case the semaphore is locked and prevents access to the FCCU IRQ Status structure.
 *
 * @param[in]   instance   instance number of FCCU peripheral.
 * @param[out]  state   pointer to the location where to save the event.
 */
void FCCU_DRV_IrqStatusGetEvent(const uint32_t  instance,
                                fccu_irq_status_t * const state);

/*!
 * @brief Checks the installed Callbacks in case of alarm
 *
 * In case of interrupt from Alarm source checks the triggered source fault
 * and If configured callback function then call it.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 */
void FCCU_DRV_IrqAlarmCallback(const uint32_t instance);

/*!
 * @brief Global Interrupt FCCU Handler
 *
 * In case of interrupt checks the triggered source for FCCU
 * and update FCCU IRQ Status with corresponding event and if a global callBackFunction
 * configured call it.
  * If a NCF NMI is enabled then it will trigger to a non-maskable interrupt(NMI).
 *
 * @param[in] instance   instance number of FCCU peripheral.
 */
void FCCU_DRV_IrqHandler(const uint32_t instance);

/*!
 * @brief Update and Read FCCU Status
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @return FCCU_STATUS_NORMAL          if FCCU is in normal mode
 *         FCCU_STATUS_CONFIGURATION   if FCCU is in configuration mode
 *         FCCU_STATUS_ALARM           if FCCU is in Alarm state
 *         FCCU_STATUS_FAULT           if FCCU is in Fault state
 */
fccu_status_t FCCU_DRV_GetConfigState(const uint32_t instance);

/*!
 * @brief Get default value of the configuration for FCCU module
 *
 * This function will get default one non-critical fault and one control configuration.
 * Non-critical fault ID default value is 2U, base on other derivatives maybe select fault injection(fake fault) suitable.
 *
 * @param[out] controlConfig pointer to configuration structure for FCCU
 * @param[out] ncfConfig     pointer to configuration structure for NonCritical Faults
 */
void FCCU_DRV_GetDefaultConfig(fccu_control_t * controlConfig,
                               fccu_config_ncf_t * ncfConfig);

/*!
 * @brief Initialize FCCU Module and NonCritical Faults
 *
 * Will clear all errors to permit enter in config mode and  normal mode
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] config    pointer to configuration structure for FCCU
 * @param[in] param     pointer to configuration structure for NonCritical Faults
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_Init(const uint32_t instance,
                       fccu_control_t config,
                       fccu_config_ncf_t const ** param);

/*!
 * @brief Set FCCU Module in configuration mode
 *
 * Will clear all errors to permit enter in configuration mode, and lock the configuration
 * as user set it.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_SetConfigState(const uint32_t instance);

/*!
 * @brief Set FCCU Module in normal mode
 *
 * Will clear errors if FCCU is in Alarm or Fault State.
 * For MPC5777C: When the FCCU state is Fault or Alarm then FCCU cannot enter Normal state if Safety core or Checker core or RCCUs are disabled. And this function
 * will return STATUS_TIMEOUT or STATUS_FCCU_ERROR_SET_NORMAL.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_SetNormalState(const uint32_t instance);

/*!
 * @brief Set Time Out Interval that permits the FCCU module to be set in configuration mode
 *
 * If is in configuration mode will force it in Normal Mode and will clear all faults.
 * User need to put again in configuration mode if need it the FCCU module
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] value      amount of time that FCCU can be in configuration mode
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_CONFIG_TIMEOUT in case of FCCU fails to set timer value
 */
status_t FCCU_DRV_SetStateTimer(const uint32_t instance,
                                uint8_t value);

/*!
 * @brief Set and configure NonCritical Faults
 *
 * Will clear all errors to permit enter in config mode and return to normal mode
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] param     pointer to configuration structure for NonCritical Faults
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_SetNcfConfig(const uint32_t instance,
                               fccu_config_ncf_t const * param);

/*!
 * @brief Get the value of freeze status registers
 *
 * Will update and return the value of freeze register selected
 *
 * @param[in]  instance   instance number of FCCU peripheral.
 * @param[in]  type       type of register to update and get the value
 * @param[out] value      pointer to where to save the value of the freeze register selected
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_UPDATE_FREEZ in case of FCCU fails to update the value of selected register
 */
status_t FCCU_DRV_GetFreezeStatus(const uint32_t instance,
                                  const fccu_freeze_type_t type,
                                  uint8_t * const value);

/*!
 * @brief Clear of freeze status registers
 *
 * Will clear and update to 0 all values of freeze status registers. Exception FCCU_SCFS register on MPC5746R, the RCCx checkers must be disabled
 * (FCCU_CFG.RCCEx = 00b) during a clear operation otherwise the RCCSx flags are asserted again.
 *
 * @param[in]  instance   instance number of FCCU peripheral.
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_UPDATE_FREEZ in case of FCCU fails to update the all freeze registers
 */
status_t FCCU_DRV_ClearAllFreezeStatus(const uint32_t instance);

/*!
 * @brief Update the configuration of EOUT signals
 *
 * This function will set the EOUT configuration that need to be done in FCCU Config State.
 * If pointer to configuration is NULL then will disable the EOUT signals (High Impedance).
 * In case FCCU is in normal State will try to put in Config state if no errors
 * detected and the put back FCCU in normal state, in case of Alarm & Fault State
 * the function will report error.
 * For MPC5777C: When the FCCU state is Normal then FCCU cannot enter Configuration state if Safety core or Checker core or RCCUs are disabled. And this function
 * will return STATUS_FCCU_ERROR_FAULT_DETECTED or STATUS_FCCU_ERROR_SET_NORMAL or STATUS_FCCU_ERROR_SET_CONFIG.
 *
 * @param[in]  instance   instance number of FCCU peripheral.
 * @param[in]  param      pointer to the configuration structure containing EOUT settings
 * @return  STATUS_SUCCESS in case of success setting EOUT configuration
 *          STATUS_FCCU_ERROR_FAULT_DETECTED in case of detection of NCF errors
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 *          STATUS_TIMEOUT in case of a timeout operation
 *
 */
status_t FCCU_DRV_EoutSet(const uint32_t instance,
                          const fccu_eout_config_t * const param);

/*!
 * @brief Update the Timer Value and return it
 *
 * This function will update and return if the correct state correspond for running timer
 * based on the timer type selected by user.
 * @param[in]  instance   instance number of FCCU peripheral.
 * @param[in]  type       type of timer to read based on fccu_xtmr_type_t enum structure
 * @return    uint32_t value of timer
 */
uint32_t FCCU_DRV_GetXtmrValue(const uint32_t instance,
                               const fccu_xtmr_type_t type);

/*!
 * @brief Set NonCritical Fault Fake for test
 *
 * This function will inject a NonCritical Fault in the peripheral for propose of testing the
 * behavior of NonCritical Faults configured to allow an emulation of triggered fault set.
 * @param[in]  instance   instance number of FCCU peripheral.
 * @param[in]  ncfValue   trigger fake fault number
 */
void FCCU_DRV_SetNcfFake(const uint32_t instance,
                         const uint8_t ncfValue);

/*!
 * @brief De-initialize FCCU Module and NonCritical Faults
 *
 * Will clear all errors to permit enter in config mode and  normal mode
 * The Module Register Values will be the reset values as chip specific
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @return  STATUS_SUCCESS in case of success setting of FCCU
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 *          STATUS_TIMEOUT in case of a timeout operation
 */
status_t FCCU_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Disable a NonCritical Fault Number
 *
 * Will Update and Check status of the NonCritical Fault number to be disabled, and report if NCF is
 * triggered or in case of a timeout operation.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] faultIdx   fault to disable.
 * @return  STATUS_SUCCESS in case of success disable of FCCU NonCritical Fault
 *          STATUS_FCCU_ERROR_FAULT_DETECTED in case of detection of NCF triggered source
 *          STATUS_TIMEOUT in case of a timeout operation
 *          STATUS_FCCU_ERROR_SET_NORMAL in case of FCCU fails to return to normal mode
 *          STATUS_FCCU_ERROR_SET_CONFIG in case of FCCU fails to enter in configuration mode
 */
status_t FCCU_DRV_DisableFault(const uint32_t instance,
                               uint8_t faultIdx);

#if FEATURE_FCCU_CONTROL_MODE_EN
/*!
 * @brief Return Selected ChipMode
 *
 * This function capture chip modes—as defined by the MC_ME module
 * based on the last event selected.
 *
 * @param[in] instance   instance number of FCCU peripheral.
 * @param[in] recentMode last event wanted to be saved
 * @param[out] infoPtr   pointer to the saving mode info structure
 */
void FCCU_DRV_GetChipMode(const uint32_t instance,
                       fccu_chip_mode_t recentMode,
                       fccu_mode_info_t * const infoPtr);
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

#if (FCCU_INSTANCE_COUNT > 0U)
/*!
 * @brief Wrapper for Global Interrupt FCCU Handler
 *
 */
void FCCU0_IRQ_Handler(void);

#endif


/*! @} */

#if defined (__cplusplus)
}

#endif

/*! @} */

#endif /* FCCU_DRIVER_H */
/*******************************************************************************
* EOF
*******************************************************************************/
