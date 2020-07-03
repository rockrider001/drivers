/*
 * Copyright 2017-2019 NXP
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

#ifndef FLEXPWM_DRIVER_H
#define FLEXPWM_DRIVER_H

/*! @file */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro defines a bitmask or shifting value used to access register bit-fields.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * This increases ease of use: allows users to access the corresponding field in the register
 * using an already defined type.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @addtogroup flexpwm_driver Flexible Pulse Width Modulator Driver
 * @ingroup flexpwm
 * @brief Flexible Pulse Width Modulator Driver
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

 /*! Used to indicate that a particular pin in the sub-module does NOT  output a PWM signal */
#define FLEXPWM_NO_PWM_OUT_SIGNAL      (0U)
/*! @brief Number of capture words that the peripheral can store */
#define FLEXPWM_NUM_DEEP_FIFO_CAPTVAL  (4U)

/*! @brief Macros for selecting the FLEXPWM fault input. */
#define FLEXPWM_FAULT_INPUT_0            ( 1U << 0U )    /*! Mask for selecting fault input 0 */
#define FLEXPWM_FAULT_INPUT_1            ( 1U << 1U )    /*! Mask for selecting fault input 1 */
#define FLEXPWM_FAULT_INPUT_2            ( 1U << 2U )    /*! Mask for selecting fault input 2 */
#define FLEXPWM_FAULT_INPUT_3            ( 1U << 3U )    /*! Mask for selecting fault input 3 */
#define FLEXPWM_FAULT_INPUT_ALL          ( FLEXPWM_FAULT_INPUT_0 | FLEXPWM_FAULT_INPUT_1 | FLEXPWM_FAULT_INPUT_2 | FLEXPWM_FAULT_INPUT_3 )

/*!
 * @brief PWM submodules.
 * Implements : flexpwm_module_t_Class
 */
typedef enum
{
    FlexPwmModule0 = 0U,                     /*!< Sub-module 0. */
    FlexPwmModule1 = 1U,                     /*!< Sub-module 1. */
    FlexPwmModule2 = 2U,                     /*!< Sub-module 2. */
    FlexPwmModule3 = 3U                      /*!< Sub-module 3. */
} flexpwm_module_t;

/*!
 * @brief PWM value registers.
 * Implements : flexpwm_val_regs_t_Class
 */
typedef enum
{
    FlexPwmVAL0 = 0U,        /*!< PWM VAL0 reg. */
    FlexPwmVAL1 = 1U,        /*!< PWM VAL1 reg. */
    FlexPwmVAL2 = 2U,        /*!< PWM VAL2 reg. */
    FlexPwmVAL3 = 3U,        /*!< PWM VAL3 reg. */
    FlexPwmVAL4 = 4U,        /*!< PWM VAL4 reg. */
    FlexPwmVAL5 = 5U         /*!< PWM VAL5 reg. */
} flexpwm_val_regs_t;

/*!
 * @brief PWM clock source selection.
 * Implements : flexpwm_clock_src_t_Class
 */
typedef enum
{
    ClkSrcPwmPeriphClk  = 0U,     /*!< The peripheral clock is used as the clock. */
    ClkSrcPwmExtClk     = 1U,     /*!< EXT_CLK is used as the clock. */
    ClkSrcPwm0Clk       = 2U      /*!< Clock of the submodule 0 (AUX_CLK) is used as the source clock. */
} flexpwm_clock_src_t;

/*!
 * @brief PWM prescaler factor selection for clock source.
 * Implements : flexpwm_clock_ps_t_Class
 */
typedef enum
{
    PwmDividedBy1   = 0U,       /*!< PWM clock frequency = fclk/1. */
    PwmDividedBy2   = 1U,       /*!< PWM clock frequency = fclk/2. */
    PwmDividedBy4   = 2U,       /*!< PWM clock frequency = fclk/4. */
    PwmDividedBy8   = 3U,       /*!< PWM clock frequency = fclk/8. */
    PwmDividedBy16  = 4U,       /*!< PWM clock frequency = fclk/16. */
    PwmDividedBy32  = 5U,       /*!< PWM clock frequency = fclk/32. */
    PwmDividedBy64  = 6U,       /*!< PWM clock frequency = fclk/64. */
    PwmDividedBy128 = 7U        /*!< PWM clock frequency = fclk/128. */
} flexpwm_clock_ps_t;

/*!
 * @brief Options that can trigger a PWM FORCE_OUT.
 * Implements : flexpwm_force_output_trigger_t_Class
 */
typedef enum
{
    ForceOutputLocalForce    = 0U,  /*!< The local force signal, CTRL2[FORCE], from this sub-module is used to force updates. */
    ForceOutputMasterForce   = 1U,  /*!< The master force signal from sub-module 0 is used to force updates. */
    ForceOutputLocalReload   = 2U,  /*!< The local reload signal from this sub-module is used to force updates without regard to the state of LDOK. */
    ForceOutputMasterReload  = 3U,  /*!< The master reload signal from sub-module 0 is used to force updates if LDOK is set. */
    ForceOutputLocalSync     = 4U,  /*!< The local sync signal from this sub-module is used to force updates. */
    ForceOutputMasterSync    = 5U,  /*!< T`he master sync signal from submodule0 is used to force updates. */
    ForceOutputExternalForce = 6U   /*!< The external force signal, EXT_FORCE, from outside the PWM module causes updates. */
} flexpwm_force_output_trigger_t;

/*!
 * @brief PWM counter initialization options.
 * Implements : flexpwm_init_src_t_Class
 */
typedef enum
{
    InitSrcLocalSync    = 0U,   /*!< Local sync (PWMX) causes initialization. */
    InitSrcMasterReload = 1U,   /*!< Master reload from sub-module 0 causes initialization. */
    InitSrcMasterSync   = 2U,   /*!< Master sync from sub-module 0 causes initialization. */
    InitSrcExtSync      = 3U    /*!< EXT_SYNC causes initialization. */
} flexpwm_init_src_t;

/*!
 * @brief PWM load frequency selection.
 * Implements : flexpwm_load_frequency_t_Class
 */
typedef enum
{
    PwmLoadEvery1Oportunity  = 0U,      /*!< Every 1 PWM opportunity. */
    PwmLoadEvery2Oportunity  = 1U,      /*!< Every 2 PWM opportunities. */
    PwmLoadEvery3Oportunity  = 2U,      /*!< Every 3 PWM opportunities. */
    PwmLoadEvery4Oportunity  = 3U,      /*!< Every 4 PWM opportunities. */
    PwmLoadEvery5Oportunity  = 4U,      /*!< Every 5 PWM opportunities. */
    PwmLoadEvery6Oportunity  = 5U,      /*!< Every 6 PWM opportunities. */
    PwmLoadEvery7Oportunity  = 6U,      /*!< Every 7 PWM opportunities. */
    PwmLoadEvery8Oportunity  = 7U,      /*!< Every 8 PWM opportunities. */
    PwmLoadEvery9Oportunity  = 8U,      /*!< Every 9 PWM opportunities. */
    PwmLoadEvery10Oportunity = 9U,      /*!< Every 10 PWM opportunities. */
    PwmLoadEvery11Oportunity = 10U,     /*!< Every 11 PWM opportunities. */
    PwmLoadEvery12Oportunity = 11U,     /*!< Every 12 PWM opportunities. */
    PwmLoadEvery13Oportunity = 12U,     /*!< Every 13 PWM opportunities. */
    PwmLoadEvery14Oportunity = 13U,     /*!< Every 14 PWM opportunities. */
    PwmLoadEvery15Oportunity = 14U,     /*!< Every 15 PWM opportunities. */
    PwmLoadEvery16Oportunity = 15U      /*!< Every 16 PWM opportunities. */
} flexpwm_load_frequency_t;

/*!
 * @brief Options available for the PWM A & B pair operation.
 * Implements : flexpwm_chnl_pair_operation_t_Class
 */
typedef enum
{
    FlexPwmComplementary     = 0U,   /*!< PWM A & PWM B are complementary channels. */
    FlexPwmIndependent       = 1U    /*!< PWM A & PWM B operation as 2 independent channels. */
} flexpwm_chnl_pair_operation_t;

/*!
 * @brief Source selection for the generation of complementary PWM pair output.
 * Implements : flexpwm_complementary_chnl_source_t_Class
 */
typedef enum
{
    FlexPwmComplementarySource23    =0u,    /*!< PWM23 is used as the source for the generation */
    FlexPwmComplementarySource45    =1U     /*!< PWM45 is used as the source for the generation */
} flexpwm_complementary_chnl_source_t;

/*!
 * @brief Options available on how to load the buffered-registers with new values.
 * Implements : flexpwm_reg_reload_t_Class
 */
typedef enum
{
    FlexPwmReloadImmediate              = 0U,   /*!< Buffered-registers get loaded with new values as soon as LDOK bit is set. */
    FlexPwmReloadPwmFullCycle           = 1U,   /*!< Registers loaded on a PWM full cycle. */
    FlexPwmReloadPwmHalfCycle           = 2U,   /*!< Registers loaded on a PWM half cycle. */
    FlexPwmReloadPwmHalfAndFullCycle    = 3U    /*!< Registers loaded on a PWM half & full cycle. */
} flexpwm_reg_reload_t;

/*!
 * @brief Options available to select reload signal for loading the buffered-registers with new values.
 * Implements : flexpwm_reg_reload_source_sel_t_Class
 */
typedef enum
{
    FLEXPWM_LOCAL_RELOAD_SIGNAL           = 0U,   /*!< The local RELOAD signal is used to reload registers. */
    FLEXPWM_MASTER_RELOAD_SIGNAL          = 1U    /*!< The master RELOAD signal (from submodule 0) is used to reload registers */
} flexpwm_reg_reload_source_sel_t;

/*!
 * @brief Interrupt enabling values for the FlexPWM module.
 * Implements : flexpwm_module_interrupt_t_Class
 */
typedef enum
{
    FLEXPWM_CMP_VAL0_INT_ENABLE     = 0x0001U,  /* Compare VAL0 interrupt          */
    FLEXPWM_CMP_VAL1_INT_ENABLE     = 0x0002U,  /* Compare VAL1 interrupt          */
    FLEXPWM_CMP_VAL2_INT_ENABLE     = 0x0004U,  /* Compare VAL2 interrupt          */
    FLEXPWM_CMP_VAL3_INT_ENABLE     = 0x0008U,  /* Compare VAL3 interrupt          */
    FLEXPWM_CMP_VAL4_INT_ENABLE     = 0x0010U,  /* Compare VAL4 interrupt          */
    FLEXPWM_CMP_VAL5_INT_ENABLE     = 0x0020U,  /* Compare VAL5 interrupt          */
    FLEXPWM_CMP_ALL_VAL_INT_ENABLE  = 0x003FU,  /* Compare interrupt for all VALx  */
    FLEXPWM_RELOAD_INT_ENABLE       = 0x0040U,  /* Reload interrupt                */
    FLEXPWM_CAPTURE_X0_INT_ENABLE   = 0x0080U,  /* Capture X0 interrupt            */
    FLEXPWM_CAPTURE_X1_INT_ENABLE   = 0x0100U,  /* Capture X1 interrupt            */
    FLEXPWM_RELOAD_ERR_INT_ENABLE   = 0x0200U,  /* Reload error interrupt          */
}flexpwm_module_interrupt_t;

/*!
 * @brief Structure is used to hold the parameters to configure a PWM module.
 * Implements : flexpwm_module_setup_t_Class
 */
typedef struct
{
    flexpwm_init_src_t cntrInitSel;                             /*!< Option to initialize the counter. */
    flexpwm_clock_src_t clkSrc;                                 /*!< Clock source for the counter. */
    flexpwm_clock_ps_t prescaler;                               /*!< Pre-scaler to divide down the clock. */
    uint32_t clockFreq;                                         /*!< Configured clock frequency for sub-module in clock ticks. */
    flexpwm_chnl_pair_operation_t chnlPairOper;                 /*!< Channel pair in independent or complementary mode. */
    flexpwm_complementary_chnl_source_t complementarySourceSel; /*!< Source selection for the PWM complementary pair generation. */
    flexpwm_reg_reload_t reloadLogic;                           /*!< PWM Reload logic setup. */
    flexpwm_reg_reload_source_sel_t reloadSource;               /*!< Reload source select*/
    flexpwm_load_frequency_t reloadFreq;                        /*!< Specifies when to reload, used when user's choice is not immediate reload. */
    flexpwm_force_output_trigger_t forceTrig;                   /*!< Specify which signal will trigger a FORCE_OUT. */
} flexpwm_module_setup_t;

/*!
 * @brief FlexPWM Signal Type options.
 * Implements : flexpwm_signal_type_t_Class
 */
typedef enum
{
    FlexPwmCentreAligned    = 0U,   /*!< Centre-aligned PWM     */
    FlexPwmEdgeAligned      = 1U,   /*!< Edge aligned PWM       */
    FlexPwmPhaseShifted     = 2U,   /*!< Phase shifted PWM      */
    FlexPwmDoubleSwitching  = 3U   /*!< Double switching PWM   */
} flexpwm_signal_type_t;

/*!
 * @brief PWM output state during fault conditions
 * Implements : flexpwm_fault_state_t_Class
 */
typedef enum
{
    FLEXPWM_OUTPUT_STATE_LOGIC_0    = 0U,    /*!< Output is forced to logic 0 state prior to consideration of output polarity control */
    FLEXPWM_OUTPUT_STATE_LOGIC_1    = 1U,    /*!< Output is forced to logic 1 state prior to consideration of output polarity control */
    FLEXPWM_OUTPUT_STATE_TRISTATED  = 2U     /*!< Output is tristated */
} flexpwm_fault_state_t;

/*!
 * @brief Configuration structure for the user to define the PWM signal characteristics.
 * Implements : flexpwm_module_signal_setup_t_Class
 */
typedef struct
{
    uint16_t pwmPeriod;                 /*!< PWM period specified in ticks. */
    flexpwm_signal_type_t flexpwmType;  /*!< PWM type, edge, centre, phase shifted or double switching. */
    uint16_t pwmAPulseWidth;            /*!< PWM A pulse width specified in ticks. Specify FLEXPWM_NO_PWM_OUT_SIGNAL if no PWM output on this pin. */
    uint16_t pwmBPulseWidth;            /*!< PWM B pulse width specified in ticks. Specify FLEXPWM_NO_PWM_OUT_SIGNAL if no PWM output on this pin. */
    uint16_t pwmAShift;                 /*!< PWM A phase shift*/
    uint16_t pwmBShift;                 /*!< PWM B phase shift*/
    bool pwmAOuten;                     /*!< PWM A output enabled*/
    bool pwmBOuten;                     /*!< PWM B output enabled*/
    bool pwmXOuten;                     /*!< PWM X output enabled*/
    bool pwmAPolarity;                  /*!< true: if output is to be inverted; false: if no output inversion. */
    bool pwmBPolarity;                  /*!< true: if output is to be inverted; false: if no output inversion. */
    bool pwmXPolarity;                  /*!< true: if output is to be inverted; false: if no output inversion. */
    uint8_t pwmAFaultDisableMask;           /*!< Specify the fault input pin for the PWMA output disable.
                                                 The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    flexpwm_fault_state_t pwmAfaultState;   /*!< Specify the fault state for the PWM output during fault conditions. */
    uint8_t pwmBFaultDisableMask;           /*!< Specify the fault input pin for the PWMB output disable.
                                                 The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    flexpwm_fault_state_t pwmBfaultState;   /*!< Specify the fault state for the PWM output during fault conditions. */
    uint8_t pwmXFaultDisableMask;           /*!< Specify the fault input pin for the PWMX output disable.
                                                 The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    flexpwm_fault_state_t pwmXfaultState;   /*!< Specify the fault state for the PWM output during fault conditions. */
} flexpwm_module_signal_setup_t;

/*!
 * @brief Structure is used to hold the parameters to configure a PWM fault protection.
 * Implements : flexpwm_fault_protection_config_t_Class
 */
typedef struct
{
    uint8_t filterPeriod;        /*!< Specify the sampling period (in peripheral clock cycles) of the fault pin input filter */
    uint8_t filterCount;         /*!< Specify the number of consecutive samples that must agree prior to the input filter accepting
                                      an input transition */
    bool glitchStretchEnable;    /*!< Used to enable the fault glitch stretching logic */
    uint8_t faultLevelMask;      /*!< Select the logic 1 to be active logic level of the individual fault inputs
                                      The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    uint8_t autoFaultClearMask;  /*!< Enables automatic clearing of faults. The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    uint8_t failSafeModeMask;    /*!< Enables the safety mode during manual fault clearing. The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
    uint8_t fullCycleMask;       /*!< PWM outputs are re-enabled only at the start of a full cycle after a fault condition
                                      The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
#if (FEATURE_FLEXPWM_HAS_COMBINATIONAL_PATH == 1U)
    uint8_t combinationalMask;   /*!< Disable the direct combinational path from the fault inputs to the PWM outputs.
                                      The mask can be set using FLEXPWM_FAULT_INPUT_ defines */
#endif
} flexpwm_fault_protection_config_t;

/*!
 * @brief FlexPWM source selection for the input capture circuit.
 * Implements : flexpwm_input_capture_select_t_Class
 */
typedef enum
{
    FLEXPWM_RAW_PWM_INPUT   = 0U,   /*!< PWM input signal selected as source. */
    FLEXPWM_OUTPUT_EDGE     = 1U    /*!< Output of edge counter/compare selected as source. */
} flexpwm_input_capture_select_t;

/*!
 * @brief FlexPWM options for edge selection for the input capture.
 * Implements : flexpwm_input_capture_edge_select_t_Class
 */
typedef enum
{
    FLEXPWM_CAPTURE_EDGE_DISABLED   = 0U,   /*!< Edge capturing disabled.   */
    FLEXPWM_CAPTURE_FALLING_EDGE    = 1U,   /*!< Capture on falling edges.  */
    FLEXPWM_CAPTURE_RISING_EDGE     = 2U,   /*!< Capture on rising edges.   */
    FLEXPWM_CAPTURE_EITHER_EDGE     = 3U    /*!< Capture on either edge.    */
} flexpwm_input_capture_edge_select_t;

/*!
 * @brief FlexPWM running modes of the input capture circuit.
 * Implements : flexpwm_input_capture_mode_t_Class
 */
typedef enum
{
    FLEXPWM_CAPTURE_FREE_RUNNIG = 0U,   /*!< Input capture runs in free running mode.   */
    FLEXPWM_CAPTURE_ONE_SHOT    = 1U    /*!< Input capture runs in one shot mode.       */
} flexpwm_input_capture_mode_t;

/*!
 * @brief FlexPWM capture edge counter.
 * Implements : flexpwm_input_capture_edge_counter_t_Class
 */
typedef enum
{
    FLEXPWM_CAPTURE_EDGE_CNT_DISABLED   = 0U,   /*!< Edge counter disabled.     */
    FLEXPWM_CAPTURE_EDGE_CNT_ENABLED    = 1U    /*!< Edge counter enabled.      */
} flexpwm_input_capture_edge_counter_t;

/*!
 * @brief FlexPWM deadtime control available counter registers.
 * Implements : flexpwm_deadtime_counter_t_Class
 */
typedef enum
{
    FLEXPWM_DEADTIME_COUNTER_0  = 0U,   /*!< Abstract of DTCNT0. */
    FLEXPWM_DEADTIME_COUNTER_1  = 1U    /*!< Abstract of DTCNT1. */
} flexpwm_deadtime_counter_t;

/*!
 * @brief FlexPWM Input capture counter registers
 * Implements : flexpwm_input_capture_counter_t_Class
 */
typedef enum
{
    FLEXPWM_INPUT_CAPTURE_COUNTER_0  = 0U,   /*!< Abstract of CVAL0 and CVAL0CYC. */
    FLEXPWM_INPUT_CAPTURE_COUNTER_1  = 1U    /*!< Abstract of CVAL1 and CVAL1CYC. */
} flexpwm_input_capture_counter_t;

/*!
 * @brief FlexPWM input capture configuration structure.
 * Implements : flexpwm_input_capture_config_t_Class
 */
typedef struct
{
    uint8_t watermark;                                  /*!< Watermark for the capture FIFOs */
    flexpwm_input_capture_select_t inputSelect;         /*!< Source selection for the input capture circuit. */
    flexpwm_input_capture_edge_select_t edgeSelect0;    /*!< Edge selection for input capturing for the capture 0 circuit. */
    flexpwm_input_capture_edge_select_t edgeSelect1;    /*!< Edge selection for input capturing for the capture 0 circuit. */
    flexpwm_input_capture_mode_t oneShot;               /*!< Mode selection for the input capture circuit. */
    uint8_t edgeCompareVal;                             /*!< Number of edges to be counted before triggering an edge event. */
    flexpwm_input_capture_edge_counter_t edgeCntEn;     /*!< Edge counter enabling */
} flexpwm_input_capture_config_t;

/*! @brief Macros for selecting the FLEXPWM interrupt flags from the STS register. */
#define FLEXPWM_RUF_INT_FLAG            ( FlexPWM_STS_RUF_MASK )
#define FLEXPWM_REF_INT_FLAG            ( FlexPWM_STS_REF_MASK )
#define FLEXPWM_RF_INT_FLAG             ( FlexPWM_STS_RF_MASK )
#define FLEXPWM_CFX1_INT_FLAG           ( FlexPWM_STS_CFX1_MASK )
#define FLEXPWM_CFX0_INT_FLAG           ( FlexPWM_STS_CFX0_MASK )
#define FLEXPWM_CMPF_VAL0_INT_FLAG      ( 0x1u )
#define FLEXPWM_CMPF_VAL1_INT_FLAG      ( 0x2u )
#define FLEXPWM_CMPF_VAL2_INT_FLAG      ( 0x4u )
#define FLEXPWM_CMPF_VAL3_INT_FLAG      ( 0x8u )
#define FLEXPWM_CMPF_VAL4_INT_FLAG      ( 0x10u )
#define FLEXPWM_CMPF_VAL5_INT_FLAG      ( 0x20u )
#define FLEXPWM_CMPF_INT_FLAG           ( FlexPWM_STS_CMPF_MASK )
#define FLEXPWM_ALL_INT_FLAG            ( FlexPWM_STS_RUF_MASK | FlexPWM_STS_REF_MASK | FlexPWM_STS_RF_MASK |\
                                         FlexPWM_STS_CFX1_MASK | FlexPWM_STS_CFX0_MASK | FlexPWM_STS_CMPF_MASK )

/*! @brief Macros for selecting the FLEXPWM signal source for Software controlled output. */
#define FLEXPWM_OUT23_3_SOURCE            ( FlexPWM_SWCOUT_OUT23_3_SHIFT )
#define FLEXPWM_OUT45_3_SOURCE            ( FlexPWM_SWCOUT_OUT45_3_SHIFT )
#define FLEXPWM_OUT23_2_SOURCE            ( FlexPWM_SWCOUT_OUT23_2_SHIFT )
#define FLEXPWM_OUT45_2_SOURCE            ( FlexPWM_SWCOUT_OUT45_2_SHIFT )
#define FLEXPWM_OUT23_1_SOURCE            ( FlexPWM_SWCOUT_OUT23_1_SHIFT )
#define FLEXPWM_OUT45_1_SOURCE            ( FlexPWM_SWCOUT_OUT45_1_SHIFT )
#define FLEXPWM_OUT23_0_SOURCE            ( FlexPWM_SWCOUT_OUT23_0_SHIFT )
#define FLEXPWM_OUT45_0_SOURCE            ( FlexPWM_SWCOUT_OUT45_0_SHIFT )

/*! @brief Macros for selecting the FLEXPWM deadtime source logic from DTSRCSEL. */
#define FLEXPWM_DEADTIME_SEL23_3_SOURCE   ( FlexPWM_DTSRCSEL_SEL23_3_SHIFT )
#define FLEXPWM_DEADTIME_SEL45_3_SOURCE   ( FlexPWM_DTSRCSEL_SEL45_3_SHIFT )
#define FLEXPWM_DEADTIME_SEL23_2_SOURCE   ( FlexPWM_DTSRCSEL_SEL23_2_SHIFT )
#define FLEXPWM_DEADTIME_SEL45_2_SOURCE   ( FlexPWM_DTSRCSEL_SEL45_2_SHIFT )
#define FLEXPWM_DEADTIME_SEL23_1_SOURCE   ( FlexPWM_DTSRCSEL_SEL23_1_SHIFT )
#define FLEXPWM_DEADTIME_SEL45_1_SOURCE   ( FlexPWM_DTSRCSEL_SEL45_1_SHIFT )
#define FLEXPWM_DEADTIME_SEL23_0_SOURCE   ( FlexPWM_DTSRCSEL_SEL23_0_SHIFT )
#define FLEXPWM_DEADTIME_SEL45_0_SOURCE   ( FlexPWM_DTSRCSEL_SEL45_0_SHIFT )
#define FLEXPWM_DEADTIME_GEN_PWM_SIGNAL   ( 0U )
#define FLEXPWM_DEADTIME_INV_PWM_SIGNAL   ( 1U )
#define FLEXPWM_DEADTIME_SW_PWM_SIGNAL    ( 2U )
#define FLEXPWM_DEADTIME_EXT_PWM_SIGNAL   ( 3U )

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Deinitialize an instance of the FlexPWM module
 *
 * The function deinitializes an instance of the FlexPWM module restoring all the
 * registers to their reset values.
 *
 * @param[in] instance      Instance number of the PWM module.
 */
void FLEXPWM_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Create a default configuration for the FlexPWM module
 *
 * This function shall populate the structures needed for initializing the
 * driver with the default values
 * The default configuration that will be created will have the following
 * characteristics:
 *      - Clock from Peripheral clock, not divided in the module
 *      - 2 channels configured in canter-aligned, independent and centre-aligned mode
 *      - the period will be of 10000 clock ticks
 *      - the duty cycle of the first signal will be 50%
 *      - the duty cycle of the second signal will be 25%
 *
 * NOTE: The user must allocate the structures before calling the function
 *
 * @param[in] moduleSetupParams     A pointer to the module settings structure
 *                                  that will be populated by the function
 * @param[in] signalParams          A pointer to the signal settings structure
 *                                  that will be populated by the function
 */
void FLEXPWM_DRV_GetDefaultConfig(flexpwm_module_setup_t * const moduleSetupParams,
                                  flexpwm_module_signal_setup_t * const signalParams);

/*!
 * @brief Sets up the PWM signals from the FlewPWM module.
 *
 * The function initializes the sub-module according to the parameters passed in by the user. The function
 * also sets up the value compare registers to match the PWM signal requirements.
 * If the dead time insertion logic is enabled, the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     The FlexPWM sub-module that is configured
 * @param[in] moduleSetupParams  The initialization values used to set up the sub-module
 * @param[in] signalParams  Signal parameters which generate the sub-modules PWM signals
 */
void FLEXPWM_DRV_SetupPwm(const uint32_t instance, const flexpwm_module_t subModule,
                          flexpwm_module_setup_t * const moduleSetupParams,
                          const flexpwm_module_signal_setup_t * const signalParams);

/*!
 * @brief Updates the PWM signal settings.
 *
 * The function updates the PWM signal to the new value that is passed in.
 * If the dead time insertion logic is enabled then the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     The FlexPWM sub-module that is configured
 * @param[in] signalParams  Signal parameters which generate the sub-modules PWM signals
 */

void FLEXPWM_DRV_UpdatePwmSignal(const uint32_t instance, const flexpwm_module_t subModule,
                                 const flexpwm_module_signal_setup_t * const signalParams);

/*!
 * @brief Updates the PWM signal period.
 *
 * The function updates the PWM signal period to the new value that is passed in.
 * It also updates the mid cycle reload point to half the PWM period.
 * If the dead time insertion logic is enabled then the pulse period is reduced by the
 * dead time period specified by the user.
 *
 * @param[in] instance          Instance number of the PWM module
 * @param[in] subModule         The FlexPWM sub-module that is configured
 * @param[in] pwmPeriod         Signal period
 */

void FLEXPWM_DRV_UpdatePwmPeriod(const uint32_t instance, const flexpwm_module_t subModule,
                                 const uint32_t pwmPeriod);

/*!
 * @brief Updates the PWM pulse width(duty cycle).
 *
 * The function updates the PWM pulse width to the new value that is passed in.
 *
 * @param[in] instance              Instance number of the PWM module
 * @param[in] subModule             The FlexPWM sub-module that is configured
 * @param[in] pwmAPulseWidth        PWM A signal pulse width
 * @param[in] pwmBPulseWidth        PWM B signal pulse width
 * @param[in] pwmType               Signal type
 *
 */

void FLEXPWM_DRV_UpdatePulseWidth(const uint32_t instance, const flexpwm_module_t subModule, const uint16_t pwmAPulseWidth,
                                  const uint16_t pwmBPulseWidth, const flexpwm_signal_type_t pwmType);

/*!
 * @brief Updates the PWM pulse width(duty cycle).
 *
 * This function set the mid cycle reload value(VAL0).
 *
 * @param[in] instance              Instance number of the PWM module
 * @param[in] subModule             The FlexPWM sub-module that is configured
 * @param[in] value                 The new value for mid cycle reload(VAL0)
 */
void FLEXPWM_DRV_UpdateMidCycleReload(const uint32_t instance, const flexpwm_module_t subModule,
                                      const uint16_t value);
/*!
 * @brief Updates the values from the modules registers.
 *
 * This function allows the user to trigger an update in the registers values.
 * The user may write in all of the registers of the module but an update won't
 * be made until this function is called.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     The FlexPWM sub-module mask that need to be updated
 */

void FLEXPWM_DRV_LoadCommands(const uint32_t instance, const uint32_t subModules);

/*!
 * @brief Clears the LDOK bits for the sub-modules specified in the provided mask.
 *
 * This function allows the user to manually clear the Load OK bits for the
 * sub-modules specified in the subModuleMask parameter. This is needed when
 * the trigger for the automatically hardware cleating is not triggered.
 * One situation in which this happens is when a sub-modules reload signal is
 * taken from the master reload signal.
 *
 * @param[in] instance        Instance number of the PWM module.
 * @param[in] subModuleMask   The FlexPWM sub-modules mask for which the LDOK bits should be cleared
 */
void FLEXPWM_DRV_ClearLDOK(const uint32_t instance, const uint32_t subModuleMask);

/*!
 * @brief Returns the current sub-module counter value
 *
 * This function will return the current sub-module counter value from the CNT
 * register.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the counter value is requested.
 *
 * @return uint16_t             Value of the counter register.
 */
uint16_t FLEXPWM_DRV_GetCounterValue(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Returns the current sub-module period.
 *
 * This function will return the current sub-module PWM period.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the counter value is requested.
 *
 * @return uint16_t             Value of the sub-module PWM period.
 */
uint16_t FLEXPWM_DRV_GetPeriod(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Returns the value in one of the sub-module value registers.
 *
 * This function will return the current value in one of the sub-module value registers.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the counter value is requested.
 * @param[in] valReg            Value(VALx) register for which the value is requested.
 *
 * @return uint16_t             Value of the sub-module requested VALx register.
 */
uint16_t FLEXPWM_DRV_GetCmpRegValue(const uint32_t instance, const flexpwm_module_t subModule,
                                    const flexpwm_val_regs_t valReg);

/*!
 * @brief Set a new value in one of the sub-module value registers.
 *
 * This function will set a new value in one of the sub-module value registers.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the counter value should be updated.
 * @param[in] valReg            Value(VALx) register to be updated.
 * @param[in] newVal            The new value to be set in the value(VALx) register.
 */
void FLEXPWM_DRV_SetCmpRegValue(const uint32_t instance, const flexpwm_module_t subModule,
                                const flexpwm_val_regs_t valReg, const uint16_t newValue);

/*!
 * @brief Enables or disables the PWM output trigger.
 *
 * This function allows the user to enable or disable the PWM trigger. The PWM has 2 triggers. The trigger 0
 * is activated when the counter matches VAL 0, VAL 2, or VAL 4 register. The trigger 1 is activated
 * when the counter matches VAL 1, VAL 3, or VAL 5.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     The FlexPWM submodule that is configured
 * @param[in] trigger       Trigger number that the user wants to activate
 * @param[in] activate      Enable or disable the trigger
 */
void FLEXPWM_DRV_SetTriggerCmd(const uint32_t instance, const flexpwm_module_t subModule,
                               const flexpwm_val_regs_t trigger, const bool activate);

/*!
 * @brief Sets the PWM trigger value.
 *
 * This function sets the value in the compare register that generates a trigger.
 * Note that the user must make sure that the value of the register being modified is not currently used to generate
 * a PWM signal.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     The FlexPWM sub-module that is configured
 * @param[in] trigger       Trigger number that we wish to configure
 * @param[in] triggerVal    Trigger value
 */
void FLEXPWM_DRV_SetTriggerVal(const uint32_t instance, const flexpwm_module_t subModule,
                               const flexpwm_val_regs_t trigger, const uint16_t triggerVal);

/*!
 * @brief Starts the PWM counter.
 *
 * This function starts the PWM sub-module counters.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     Sub-modules to start; 4 bit value, 1-bit for each submodule
 */
void FLEXPWM_DRV_CounterStart(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Stops the PWM counter.
 *
 * This function stops the the PWM sub-module counters.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     Sub-modules to stop;  4 bit value, 1-bit for each submodule
 */
void FLEXPWM_DRV_CounterStop(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Force external output of a sub-module.
 *
 * This function forces the external output of a sub-module to one of the available options depending
 * on the selected source for the signal.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] subModule     Sub-module for which the signal is forced
 * @param[in] forceInit     true to trigger a 'Force Out' event
 * @param[in] forceEnable   enables/disables Force Initialization action by forceInit
 * @param[in] forceTrig     source of the forced output signal
 */
void FLEXPWM_DRV_ForcePwmOutput(const uint32_t instance, const flexpwm_module_t subModule,
                                const bool forceInit, const bool forceEnable,
                                const flexpwm_force_output_trigger_t forceTrig);

/*!
 * @brief Enables the submodule interrupts.
 *
 * This function should enable interrupts for a submodule.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 * @param[in] interruptMask     A mask where each bit means that the corresponding interrupt is enabled
 *                              The interrupt mask should be constructed using flexpwm_module_interrupt_t
 */
void FLEXPWM_DRV_EnableModuleInterrupts(const uint32_t instance, const flexpwm_module_t subModule,
                                        const uint16_t interruptMask);

/*!
 * @brief Disables the submodule interrupts.
 *
 * This function should disable interrupts for a submodule.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 * @param[in] interruptMask     A mask where each bit means that the corresponding interrupt is disabled
 *                              The interrupt mask should be constructed using flexpwm_module_interrupt_t
 */
void FLEXPWM_DRV_DisableModuleInterrupts(const uint32_t instance, const flexpwm_module_t subModule,
                                         const uint16_t interruptMask);

/*!
 * @brief Configures the input capture circuit.
 *
 * This function configures the input capture circuit
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 * @param[in] inputCapture      The configuration structure for the input capture circuit
 */
void FLEXPWM_DRV_ConfigureInputCapture(const uint32_t instance, const flexpwm_module_t subModule,
                                       const flexpwm_input_capture_config_t * const inputCapture);

/*!
 * @brief Starts the input capturing
 *
 * This function starts the input capture event.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_StartInputCapture(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Stops the input capturing
 *
 * This function stops the input capture event.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_StopInputCapture(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Returns the number of edges counted to the moment
 *
 * This function returns the number of edges counted by the input capture circuit
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced.
 * @return uint32_t             Number of edges counted to the moment.
 */
uint32_t FLEXPWM_DRV_GetEdgeCounter(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Configures the edge compare value.
 *
 * This function configures the input capture number of edges which should be counted
 * before triggering a capture event.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 * @param[in] value             The edge counter value to be set.
 */
void FLEXPWM_DRV_SetEdgeCompare(const uint32_t instance, const flexpwm_module_t subModule,
                                const uint16_t value);

/*!
 * @brief Enables the edge counter.
 *
 * This function enables the edge counter.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_EnableEdgeCounting(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Disables the edge counter.
 *
 * This function disables the edge counter.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_DisableEdgeCounting(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Returns the captured value
 *
 * This function populates the user given array with the values in
 * the capture register FIFO for which the capture has taken place.
 * The function will read as many words as the user specifies in the
 * noWords parameter if they exist in the FIFO.
 * Note: The user must allocate the memory pointed by registerValue
 * accordingly dimensioned with the number of words.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the capture value is requested.
 * @param[in] counterRegister   Counter register to be read.
 * @param[out] registerValue    User given array in which the capture words will be put.
 * @param[in] noWords           Number of words to be read.
 */
void FLEXPWM_DRV_GetCaptureValue(const uint32_t instance,
                                 const flexpwm_module_t subModule,
                                 const flexpwm_input_capture_counter_t counterRegister,
                                 uint16_t * const registerValue,
                                 const uint8_t noWords);

/*!
 * @brief Returns the captured number of cycles
 *
 * This function returns the number of cycles for which the capture has taken place.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the capture value is requested.
 * @param[in] counterRegister   Cycle counter register to be returned.
 * @return uint16_t             Number of cycles corresponding to the captured value.
 */
uint16_t FLEXPWM_DRV_GetCaptureCycle(const uint32_t instance, const flexpwm_module_t subModule,
                                     const flexpwm_input_capture_counter_t counterRegister);

/*!
 * @brief Modifies the input capture channel mode.
 *
 * This function modifies the capturing moment for one of the input capture circuits
 * specified in the parameters
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the capture value is requested.
 * @param[in] chnType           Channel type to which the circuit shall change.
 * @param[in] chnNumber         Input capture circuit selection.
 */
void FLEXPWM_DRV_SetChannelMode(const uint32_t instance, const flexpwm_module_t subModule,
                                const flexpwm_input_capture_edge_select_t edgeSelect,
                                const flexpwm_input_capture_counter_t counterRegister);

/*!
 * @brief Enables the debug mode.
 *
 * This function enables the debug mode. When debug mode is enabled the
 * PWM will continue to run even if the chip is in debug mode.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_DebugEnable(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Disables the debug mode.
 *
 * This function disables the debug mode. This means that when the chip
 * is in debug mode the PWM outputs will be disabled until debug mode is exited
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 */
void FLEXPWM_DRV_DebugDisable(const uint32_t instance, const flexpwm_module_t subModule);

/*!
 * @brief Sets the deadtime counter value.
 *
 * This function sets the deadtime counter value.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced
 * @param[in] value             The value for the Deadtime counter
 */
void FLEXPWM_DRV_SetDeadtime(const uint32_t instance, const flexpwm_module_t subModule, const uint16_t value,
                             const flexpwm_deadtime_counter_t counterRegister);
/*!
 * @brief Selects the source for the deadtime logic.
 *
 * This function selects from the possible sources to overrided the signal
 * that goes to the deadtime logic.
 * For the source to be configured please use the defined macros
 * (i.e. FLEXPWM_DEADTIME_SEL23_3_SOURCE). For the override option selection
 * please also use the defined macros (i.e. FLEXPWM_DEADTIME_SW_PWM_SIGNAL).
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] index             Bit-field for override configuration of
                                the generated PWM signal
 * @param[in] value             Option for selecting the desired override
 */
void FLEXPWM_DRV_SelectDeadtimeSource(const uint32_t instance, const uint16_t index,
                                      const uint16_t value);

/*!
 * @brief Selects the software controlled output.
 *
 * This function selects the value to be supplied to the deadtime
 * generator instead of the PWM signal.
 * For selecting the index of the bit-field please use the defined
 * macros (i.e. FlexPWM_SWCOUT_OUT23_3_SHIFT).
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] index             Index of the value to be overwritten from
 *                              the source selection register
 * @param[in] value             Value to be written at the index
 */
void FLEXPWM_DRV_SelectSwCtrlOutput(const uint32_t instance, const uint32_t index,
                                    const uint32_t value);

/*!
 * @brief Masks the PWM signal for a mask of submodules.
 *
 * This function masks the PWM signals for a mask of submodules.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] pwmAMask          Mask of the submodules for which PWMA should be masked
 * @param[in] pwmBMask          Mask of the submodules for which PWMB should be masked
 * @param[in] pwmXMask          Mask of the submodules for which PWMX should be masked
 */
void FLEXPWM_DRV_MaskOutput(const uint32_t instance, const uint16_t pwmAMask,
                            const uint16_t pwmBMask, const uint16_t pwmXMask);

/*!
 * @brief Forces the masking of the PWM signals.
 *
 * This function forces the masking of the PWM signals based on the mask provided.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] mask              Mask for the submodules that need to be forced updated.
 */
void FLEXPWM_DRV_ForceApplyMask(const uint32_t instance, const uint32_t mask);

/*!
 * @brief Reads the interrupt flags
 *
 * This function reads the interrupt flags based on the mask provided. To use the mask
 * you should use the defines specially created to map on the bit-fields of the Status register.
 * To specify the bit-fields please use the defined macros (i.e. FLEXPWM_RUF_INT_FLAG).
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced.
 * @param[in] interruptFlags    Mask for the interrupt flags to be read.
 * @return uint32_t             Bit-mask with values for the FLEXPWM interrupt flags.
 */
uint32_t FLEXPWM_DRV_GetInterruptFlag(const uint32_t instance, const flexpwm_module_t subModule,
                                      const uint32_t interruptFlag);

/*!
 * @brief Clears the interrupt flags
 *
 * This function clears the interrupt flags based on the mask provided. To use the mask
 * you should use the defines specially created to map on the bit-fields of the Status register.
 * To specify the bit-fields please use the defined macros (i.e. FLEXPWM_RUF_INT_FLAG).
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] subModule         Sub-module for which the signal is forced.
 * @param[in] interruptFlags    Mask for the interrupt flags to be cleared.
 */
void FLEXPWM_DRV_ClearInterruptFlag(const uint32_t instance, const flexpwm_module_t subModule,
                                    const uint16_t interruptFlag);

/*!
 * @brief Sets up the PWM fault protection.
 *
 * The function configures the fault protection according to the configuration structure passed in by the user.
 *
 * @param[in] instance         Instance number of the PWM module.
 * @param[in] faultProtection  Pointer to fault protection configuration structure
 */
void FLEXPWM_DRV_SetupFaultProtection(const uint32_t instance,
                                      const flexpwm_fault_protection_config_t * const faultProtection);

/*!
 * @brief Sets the input fault filter period.
 *
 * This function sets the fault filter period for the input capture circuit.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] period        The period for the input fault filter
 */
void FLEXPWM_DRV_SetFaultFilterPeriod(const uint32_t instance, const uint32_t period);

/*!
 * @brief Sets the input fault filter counter.
 *
 * This function sets the fault filter counter for the number of consecutive samples
 * that must agree prior to the input filter accepting an input transition.
 *
 * @param[in] instance      Instance number of the PWM module.
 * @param[in] counter       The counter for the input fault filter
 */
void FLEXPWM_DRV_SetFaultFilterCounter(const uint32_t instance, const uint32_t counter);

/*!
 * @brief Enables the fault glitch stretching for the input capture circuit.
 *
 * This function enables the fault glitch stretching for the input capture circuit.
 *
 * @param[in] instance      Instance number of the PWM module.
 */
void FLEXPWM_DRV_EnableFaultGlitchStretch(const uint32_t instance);

/*!
 * @brief Disables the fault glitch stretching for the input capture circuit.
 *
 * This function disables the fault glitch stretching for the input capture circuit.
 *
 * @param[in] instance      Instance number of the PWM module.
 */
void FLEXPWM_DRV_DisableFaultGlitchStretch(const uint32_t instance);

/*!
 * @brief Enable the fault input interrupt
 *
 * This function enables fault interrupt requests that are set to '1' in the faultInputMask.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to enable multiple interrupt requests.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] faultInputMask    Mask for the fault input interrupt to be enabled.
 */
void FLEXPWM_DRV_EnableFaultInterrupt(const uint32_t instance,
                                      const uint8_t faultInputMask);

/*!
 * @brief Disable the fault input interrupt
 *
 * This function disables fault interrupt requests that are set to '1' in the faultInputMask.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to disable multiple interrupt requests.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] faultInputMask    Mask for fault input interrupt to be disable.
 */
void FLEXPWM_DRV_DisableFaultInterrupt(const uint32_t instance,
                                       const uint8_t faultInputMask);

/*!
 * @brief Clears the fault flags
 *
 * This function clears fault flags corresponding to the bits enabled in the faultInputMask input parameter.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to clear multiple fault flags.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] faultInputMask    Mask for the fault flags to be cleared.
 */
void FLEXPWM_DRV_ClearFaultFlags(const uint32_t instance,
                                 const uint8_t faultInputMask);

/*!
 * @brief Gets the interrupt flags
 *
 * This function gets fault flags corresponding to the bits enabled in the faultInputMask input parameter.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to get multiple fault flag.
 *
 * @param[in] instance          Instance number of the PWM module.
 * @param[in] faultInputMask    Mask for the fault flags to be get.
 */
uint32_t FLEXPWM_DRV_GetFaultFlags(const uint32_t instance,
                                   const uint8_t faultInputMask);

/*!
 * @brief Simulates fault condition
 *
 * This function simulates a fault condition. Calling this function causes a simulated fault to be
 * sent into all of the fault filters.
 *
 * @param[in] instance       Instance number of the PWM module.
 * @param[in] faultState     The fault state.
 */
void FLEXPWM_DRV_SimulateFault(const uint32_t instance, const bool faultState);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup flexpwm */

#endif /* FLEXPWM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
