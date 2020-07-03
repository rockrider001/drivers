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

/*!
 * @file etimer_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, not referenced
 * Macro not referenced in code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 */

#ifndef ETIMER_DRIVER_H
#define ETIMER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "status.h"

#include "device_registers.h"

/*!
 * @addtogroup etimer_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max period in counts
 *  Each counter is 16-bit, but you can chain multiple counters  */
#define ETIMER_CH_MAX_TIMER_COUNT                  (0xFFFFU)

#ifdef FEATURE_ETIMER_HAS_WATCHDOG
/*! @brief Watchdog disabled value
 * To disable the watchdog set it with this value  */
#define ETIMER_WATCHDOG_DISABLE_VALUE                   (0x0000UL)
/*! @brief Max value the watchdog can count to  */
#define ETIMER_WATCHDOG_MAX_VALUE                       (0xFFFFFFFFUL)
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */

/* ETIMER_NUM macros below are gathered into a single requirement
 *
 * Implements : ETIMER_NUM_MACROS_Class
 */

/*! @brief Number of compare registers */
#define ETIMER_NUM_COMPARE_REGISTERS                    (2UL)

/*! @brief Number of capture registers */
#define ETIMER_NUM_CAPTURE_REGISTERS                    (2UL)

/*! @brief Number of capture words that the peripheral can store */
#define ETIMER_NUM_CAPTURE_WORDS                        ((1U<<ETIMER_CCCTRL_CFWM_WIDTH)-1U)

/*! @brief Channel enable masks.
 * Use these masks wherever you want to select the channel.
 * Some functions accept ORing these to address multiple channels at once.
 *
 * Implements : ETIMER_ENABLE_BITMASK_Class
 */
#define ETIMER_ENABLE_CH0                          (0x0001U)
#define ETIMER_ENABLE_CH1                          (0x0002U)
#define ETIMER_ENABLE_CH2                          (0x0004U)
#define ETIMER_ENABLE_CH3                          (0x0008U)
#define ETIMER_ENABLE_CH4                          (0x0010U)
#define ETIMER_ENABLE_CH5                          (0x0020U)

/*! @brief All available interrupt sources set, can be used for enable/disable of all interrupt sources  */
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
#define ETIMER_CH_IRQ_SOURCE_MASK                  (ETIMER_INTDMA_TCFIE_MASK | ETIMER_INTDMA_TCF1IE_MASK | ETIMER_INTDMA_TCF2IE_MASK | \
                                                    ETIMER_INTDMA_TOFIE_MASK | ETIMER_INTDMA_IELFIE_MASK | ETIMER_INTDMA_IEHFIE_MASK | \
                                                    ETIMER_INTDMA_ICF1IE_MASK | ETIMER_INTDMA_ICF2IE_MASK | ETIMER_INTDMA_RCFIE_MASK | \
                                                    ETIMER_INTDMA_WDFIE_MASK | ETIMER_INTDMA_CMPLD1DE_MASK | ETIMER_INTDMA_CMPLD2DE_MASK | \
                                                    ETIMER_INTDMA_ICF1DE_MASK | ETIMER_INTDMA_ICF2DE_MASK)
#else
#define ETIMER_CH_IRQ_SOURCE_MASK                  (ETIMER_INTDMA_TCFIE_MASK | ETIMER_INTDMA_TCF1IE_MASK | ETIMER_INTDMA_TCF2IE_MASK | \
                                                    ETIMER_INTDMA_TOFIE_MASK | ETIMER_INTDMA_IELFIE_MASK | ETIMER_INTDMA_IEHFIE_MASK | \
                                                    ETIMER_INTDMA_ICF1IE_MASK | ETIMER_INTDMA_ICF2IE_MASK | ETIMER_INTDMA_RCFIE_MASK | \
                                                    ETIMER_INTDMA_CMPLD1DE_MASK | ETIMER_INTDMA_CMPLD2DE_MASK | \
                                                    ETIMER_INTDMA_ICF1DE_MASK | ETIMER_INTDMA_ICF2DE_MASK)
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */
/*! @brief Check all available interrupt flags set, can be used for check/clear of all interrupt flags  */
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
#define ETIMER_CH_IRQ_FLAGS_MASK                   (ETIMER_STS_WDF_MASK | ETIMER_STS_RCF_MASK | ETIMER_STS_ICF2_MASK | \
                                                    ETIMER_STS_ICF1_MASK | ETIMER_STS_IEHF_MASK | ETIMER_STS_IELF_MASK | \
                                                    ETIMER_STS_TOF_MASK | ETIMER_STS_TCF2_MASK | ETIMER_STS_TCF1_MASK | \
                                                    ETIMER_STS_TCF_MASK)
#else
#define ETIMER_CH_IRQ_FLAGS_MASK                   (ETIMER_STS_RCF_MASK | ETIMER_STS_ICF2_MASK | \
                                                    ETIMER_STS_ICF1_MASK | ETIMER_STS_IEHF_MASK | ETIMER_STS_IELF_MASK | \
                                                    ETIMER_STS_TOF_MASK | ETIMER_STS_TCF2_MASK | ETIMER_STS_TCF1_MASK | \
                                                    ETIMER_STS_TCF_MASK)
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */

/*! @brief Interrupt set sources masks, direct mapped to bits in register
 *
 * Implements : ETIMER_CH_IRQ_SOURCE_BITMASK_Class
 */
#define ETIMER_CH_IRQ_SOURCE_ICF2DE                (ETIMER_INTDMA_ICF2DE_MASK)    /*!< Input Capture 2 Flag DMA Enable */
#define ETIMER_CH_IRQ_SOURCE_ICF1DE                (ETIMER_INTDMA_ICF1DE_MASK)    /*!< Input Capture 1 Flag DMA Enable */
#define ETIMER_CH_IRQ_SOURCE_CMPLD2DE              (ETIMER_INTDMA_CMPLD2DE_MASK)  /*!< Comparator Load Register 2 Flag DMA Enable */
#define ETIMER_CH_IRQ_SOURCE_CMPLD1DE              (ETIMER_INTDMA_CMPLD1DE_MASK)  /*!< Comparator Load Register 1 Flag DMA Enable */
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
#define ETIMER_CH_IRQ_SOURCE_WDFIE                 (ETIMER_INTDMA_WDFIE_MASK)     /*!< Watchdog Flag Interrupt Enable */
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */
#define ETIMER_CH_IRQ_SOURCE_RCFIE                 (ETIMER_INTDMA_RCFIE_MASK)     /*!< Redundant Channel Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_ICF2IE                (ETIMER_INTDMA_ICF2IE_MASK)    /*!< Input Capture 2 Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_ICF1IE                (ETIMER_INTDMA_ICF1IE_MASK)    /*!< Input Capture 1 Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_IEHFIE                (ETIMER_INTDMA_IEHFIE_MASK)    /*!< Input Edge High Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_IELFIE                (ETIMER_INTDMA_IELFIE_MASK)    /*!< Input Edge Low Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_TOFIE                 (ETIMER_INTDMA_TOFIE_MASK)     /*!< Timer Overflow Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_TCF2IE                (ETIMER_INTDMA_TCF2IE_MASK)    /*!< Timer Compare 2 Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_TCF1IE                (ETIMER_INTDMA_TCF1IE_MASK)    /*!< Timer Compare 1 Flag Interrupt Enable */
#define ETIMER_CH_IRQ_SOURCE_TCFIE                 (ETIMER_INTDMA_TCFIE_MASK)     /*!< Timer Compare Flag Interrupt Enable */

/*! @brief Interrupt flags masks, direct mapped to bits in register
 *
 * Implements : ETIMER_CH_IRQ_FLAGS_BITMASK_Class
 */
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
#define ETIMER_CH_IRQ_FLAGS_WDF                (ETIMER_STS_WDF_MASK)   /*!< Watchdog Time-out Flag.
                                                                            This bit is set when the watchdog times out by counting down to zero. */
#endif
#define ETIMER_CH_IRQ_FLAGS_RCF                (ETIMER_STS_RCF_MASK)   /*!< Redundant Channel Flag.
                                                                            This field is set to 1 when there is a miscompare between this channel's
                                                                            OFLAG value and the OFLAG value of the corresponding redundant channel. */
#define ETIMER_CH_IRQ_FLAGS_ICF2               (ETIMER_STS_ICF2_MASK)  /*!< Input Capture 2 Flag.
                                                                            This bit is set when an input capture event (as defined by CPT2MODE) occurs
                                                                            and the word count of the CAPT1 FIFO exceeds the value of the CFWM field. */
#define ETIMER_CH_IRQ_FLAGS_ICF1               (ETIMER_STS_ICF1_MASK)  /*!< Input Capture 1 Flag.
                                                                            This bit is set when an input capture event (as defined by CPT1MODE) occurs
                                                                            and the word count of the CAPT1 FIFO exceeds the value of the CFWM field. */
#define ETIMER_CH_IRQ_FLAGS_IEHF               (ETIMER_STS_IEHF_MASK)  /*!< Input Edge High Flag.
                                                                            This bit is set when a positive input transition occurs. */
#define ETIMER_CH_IRQ_FLAGS_IELF               (ETIMER_STS_IELF_MASK)  /*!< Input Edge Low Flag.
                                                                            This bit is set when a negative input transition occurs. */
#define ETIMER_CH_IRQ_FLAGS_TOF                (ETIMER_STS_TOF_MASK)   /*!< Timer Overflow Flag.
                                                                            This bit is set when the counter rolls over its maximum value. */
#define ETIMER_CH_IRQ_FLAGS_TCF2               (ETIMER_STS_TCF2_MASK)  /*!< Timer Compare 2 Flag.
                                                                            This bit is set when a successful compare occurs with COMP2. */
#define ETIMER_CH_IRQ_FLAGS_TCF1               (ETIMER_STS_TCF1_MASK)  /*!< Timer Compare 1 Flag.
                                                                            This bit is set when a successful compare occurs with COMP1. */
#define ETIMER_CH_IRQ_FLAGS_TCF                (ETIMER_STS_TCF_MASK)   /*!< Timer Compare Flag.
                                                                            This bit is set when a successful compare occurs. */

/*!
 * @brief Counter functional mode.
 *
 * This is used to set the conter mode
 *
 * Implements : etimer_cntmode_t_Class
 */
typedef enum
{
    ETIMER_CNTMODE_NOP                      = 0x00U,  /*!< No operation, stopped  */
    ETIMER_CNTMODE_PRIMARY                  = 0x01U,  /*!< Count rising OR falling edges of primary source */
    ETIMER_CNTMODE_PRIMARY_BOTH_EDGES       = 0x02U,  /*!< Count rising AND falling edges of primary source */
    ETIMER_CNTMODE_PRI_WHILE_SEC_HI         = 0x03U,  /*!< Count rising edges of primary source while secondary input is active high */
    ETIMER_CNTMODE_QUADRATURE               = 0x04U,  /*!< Quadrature count mode, use both primary and secondary */
    ETIMER_CNTMODE_SECONDARY_AS_DIRECTION   = 0x05U,  /*!< Count rising OR falling edges of primary source, secondary source specifies direction  */
    ETIMER_CNTMODE_SECONDARY_AS_TRIGGER     = 0x06U,  /*!< Edge of secondary source triggers primary count till compare */
    ETIMER_CNTMODE_CASCADE                  = 0x07U,  /*!< Cascaded counter mode, up/down  */
}  etimer_cntmode_t;

/*!
 * @brief Interrupt vectors available ETIMER instance
 *
 * This enumerates all IRQ vectors available for one ETIMER instance.
 * Use this enums to pass to interrupt manager.
 *
 * Implements : etimer_irq_vector_t_Class
 */
typedef enum
{
    ETIMER_IRQ_CH0           = 0U,              /*!< eTimer Channel 0 */
    ETIMER_IRQ_CH1           = 1U,              /*!< eTimer Channel 1 */
    ETIMER_IRQ_CH2           = 2U,              /*!< eTimer Channel 2 */
    ETIMER_IRQ_CH3           = 3U,              /*!< eTimer Channel 3 */
    ETIMER_IRQ_CH4           = 4U,              /*!< eTimer Channel 4 */
    ETIMER_IRQ_CH5           = 5U,              /*!< eTimer Channel 5 */
#if ETIMER_CH_COUNT > 6U
    ETIMER_IRQ_CH6           = 6U,              /*!< eTimer Channel 6 */
    ETIMER_IRQ_CH7           = 7U,              /*!< eTimer Channel 7 */
#endif
#ifdef FEATURE_ETIMER_HAS_WATCHDOG
    ETIMER_IRQ_WTIF          = ETIMER_CH_MAX_IRQS - 2U,              /*!< eTimer Watchdog */
#endif
    ETIMER_IRQ_RCF           = ETIMER_CH_MAX_IRQS - 1U              /*!< eTimer RCF - always last*/
} etimer_irq_vector_t;

/*!
 * @brief Input source options.
 *
 * Each channel has two inputs, one Primary and one Secondary.
 * The Primary input must always be set to use a signal which is valid to the selected mode.
 * The Secondary input setup is ignored in any mode which does not use it.
 *
 * Implements : etimer_input_source_t_Class
 */
typedef enum
{
    ETIMER_IN_SRC_CNT0_IN      = 0x00U,  /*!< Use Counter 0 Input pin as input source  */
    ETIMER_IN_SRC_CNT1_IN      = 0x01U,  /*!< Use Counter 1 Input pin as input source  */
    ETIMER_IN_SRC_CNT2_IN      = 0x02U,  /*!< Use Counter 2 Input pin as input source  */
    ETIMER_IN_SRC_CNT3_IN      = 0x03U,  /*!< Use Counter 3 Input pin as input source  */
    ETIMER_IN_SRC_CNT4_IN      = 0x04U,  /*!< Use Counter 4 Input pin as input source  */
    ETIMER_IN_SRC_CNT5_IN      = 0x05U,  /*!< Use Counter 5 Input pin as input source  */
    ETIMER_IN_SRC_AUX0_IN      = 0x08U,  /*!< Use Auxiliary Input Pin 0 as input source  */
    ETIMER_IN_SRC_AUX1_IN      = 0x09U,  /*!< Use Auxiliary Input Pin 1 as input source  */
    ETIMER_IN_SRC_AUX2_IN      = 0x0aU,  /*!< Use Auxiliary Input Pin 2 as input source  */
    ETIMER_IN_SRC_AUX3_IN      = 0x0bU,  /*!< Use Auxiliary Input Pin 3 as input source  */
    ETIMER_IN_SRC_AUX4_IN      = 0x0cU,  /*!< Use Auxiliary Input Pin 4 as input source  */
    ETIMER_IN_SRC_CNT0_OUT     = 0x10U,  /*!< Use Counter 0 Output as input source  */
    ETIMER_IN_SRC_CNT1_OUT     = 0x11U,  /*!< Use Counter 1 Output as input source  */
    ETIMER_IN_SRC_CNT2_OUT     = 0x12U,  /*!< Use Counter 2 Output as input source  */
    ETIMER_IN_SRC_CNT3_OUT     = 0x13U,  /*!< Use Counter 3 Output as input source  */
    ETIMER_IN_SRC_CNT4_OUT     = 0x14U,  /*!< Use Counter 4 Output as input source  */
    ETIMER_IN_SRC_CNT5_OUT     = 0x15U,  /*!< Use Counter 5 Output as input source  */
    ETIMER_IN_SRC_CLK_DIV_1    = 0x18U,  /*!< Use Bus Clock divided by 1 as input source  */
    ETIMER_IN_SRC_CLK_DIV_2    = 0x19U,  /*!< Use Bus Clock divided by 2 as input source  */
    ETIMER_IN_SRC_CLK_DIV_4    = 0x1aU,  /*!< Use Bus Clock divided by 4 as input source  */
    ETIMER_IN_SRC_CLK_DIV_8    = 0x1bU,  /*!< Use Bus Clock divided by 8 as input source  */
    ETIMER_IN_SRC_CLK_DIV_16   = 0x1cU,  /*!< Use Bus Clock divided by 16 as input source  */
    ETIMER_IN_SRC_CLK_DIV_32   = 0x1dU,  /*!< Use Bus Clock divided by 32 as input source  */
    ETIMER_IN_SRC_CLK_DIV_64   = 0x1eU,  /*!< Use Bus Clock divided by 64 as input source  */
    ETIMER_IN_SRC_CLK_DIV_128  = 0x1fU,  /*!< Use Bus Clock divided by 128 as input source  */
} etimer_input_source_t;

/*!
 * @brief Polarity type for input signals.
 *
 * This is used to select the polarity of the input signals.
 * A negative polarity will negate the input signal before passing it to the ETIMER hardware.
 *
 * Implements : etimer_input_polarity_t_Class
 */
typedef enum
{
    ETIMER_POLARITY_POSITIVE = 0x00U, /*!< Input signal sampled on positive edge */
    ETIMER_POLARITY_NEGATIVE = 0x01U  /*!< Input signal sampled on negative edge */
} etimer_input_polarity_t;

/*!
 * @brief Counting direction.
 *
 * This is used to determine which direction the ETIMER channel will count.
 * Some mode ignore this setting.
 *
 * Implements : etimer_count_direction_t_Class
 */
typedef enum
{
    ETIMER_COUNT_UP     = 0x00U, /*!< Counter value will increase */
    ETIMER_COUNT_DOWN   = 0x01U  /*!< Counter value will decrease */
} etimer_count_direction_t;

/*!
 * @brief Input filter consecutive samples.
 *
 * This is used for glitch filtering on input signals.
 * An input signal state must be the same for at least this number of samples before passing it to the hardware.
 * You cannot set less than ETIMER_FILT_CNT_3 samples.
 *
 * Implements : etimer_filt_cnt_t_Class
 */
typedef enum
{
    ETIMER_FILT_CNT_3  = 0x00U, /*!< Require 3 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_4  = 0x01U, /*!< Require 4 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_5  = 0x02U, /*!< Require 5 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_6  = 0x03U, /*!< Require 6 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_7  = 0x04U, /*!< Require 7 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_8  = 0x05U, /*!< Require 8 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_9  = 0x06U, /*!< Require 9 consecutive samples to be of equal value */
    ETIMER_FILT_CNT_10 = 0x07U, /*!< Require 10 consecutive samples to be of equal value */
} etimer_filt_cnt_t;

/*!
 * @brief Compare load control type.
 *
 * This determines automatic loading of counter registers when a compare event happens.
 * Values to be loaded are stored in special buffer registers.
 *
 * Implements : etimer_clc_t_Class
 */
typedef enum
{
    ETIMER_CLC_NEVER                              = 0x00U,  /*!< Disabled  */
    ETIMER_CLC_FROM_CMPLD1_WHEN_COMP1             = 0x02U,  /*!< Load COMPx with CMPLD1 upon successful compare with the value in COMP1  */
    ETIMER_CLC_FROM_CMPLD1_WHEN_COMP2             = 0x03U,  /*!< Load COMPx with CMPLD1 upon successful compare with the value in COMP2  */
    ETIMER_CLC_FROM_CMPLD2_WHEN_COMP1             = 0x04U,  /*!< Load COMPx with CMPLD2 upon successful compare with the value in COMP1  */
    ETIMER_CLC_FROM_CMPLD2_WHEN_COMP2             = 0x05U,  /*!< Load COMPx with CMPLD2 upon successful compare with the value in COMP2  */
    ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP1         = 0x06U,  /*!< Load CNTR with CMPLD upon successful compare with the value in COMP1  */
    ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP2         = 0x07U,  /*!< Load CNTR with CMPLD upon successful compare with the value in COMP2  */
} etimer_clc_t;

/*!
 * @brief Output compare flag control type.
 *
 * This determine how the output pin behaves, thus influencing output compare mode.
 *
 * Implements : etimer_outmode_t_Class
 */
typedef enum
{
    ETIMER_OUTMODE_SOFTWARE               = 0x00U,  /*!< OFLAG is software controlled  */
    ETIMER_OUTMODE_COMPARE_CLEAR          = 0x01U,  /*!< Clear OFLAG output on successful compare (COMP1 or COMP2)  */
    ETIMER_OUTMODE_COMPARE_SET            = 0x02U,  /*!< Set OFLAG output on successful compare (COMP1 or COMP2)  */
    ETIMER_OUTMODE_COMPARE_TOGGLE         = 0x03U,  /*!< Toggle OFLAG output on successful compare (COMP1 or COMP2)  */
    ETIMER_OUTMODE_COMPARE_ALT_TOGGLE     = 0x04U,  /*!< Toggle OFLAG output using alternating compare registers  */
    ETIMER_OUTMODE_COMP1_SET_SECIN_CLEAR  = 0x05U,  /*!< Set on compare with COMP1, cleared on secondary source input edge  */
    ETIMER_OUTMODE_COMP2_SET_SECIN_CLEAR  = 0x06U,  /*!< Set on compare with COMP2, cleared on secondary source input edge  */
    ETIMER_OUTMODE_COMP_SET_OVF_CLEAR     = 0x07U,  /*!< Set on compare, cleared on counter roll-over  */
    ETIMER_OUTMODE_COMP1_SET_COMP2_CLEAR  = 0x08U,  /*!< Set on successful compare on COMP1, clear on successful compare on COMP2  */
    ETIMER_OUTMODE_ACTIVE_SET             = 0x09U,  /*!< Asserted while counter is active, cleared when counter is stopped.  */
    ETIMER_OUTMODE_UP_SET_DOWN_CLEAR      = 0x0aU,  /*!< Asserted when counting up, cleared when counting down  */
    ETIMER_OUTMODE_COUNT_CLK               = 0x0fU,  /*!< Enable gated clock output while counter is active  */
} etimer_outmode_t;

/*!
 * @brief Compare mode control.
 *
 * This will determine what registers to use when comparing
 *
 * Implements : etimer_cmpmode_t_Class
 */
typedef enum
{
    ETIMER_CMPMODE_COMP1_UP_COMP2_UP             = 0x00U,  /*!< COMP1 register is used when the counter is counting up;
                                                                COMP2 register is used when the counter is counting up  */
    ETIMER_CMPMODE_COMP1_DOWN_COMP2_UP           = 0x01U,  /*!< COMP1 register is used when the counter is counting down;
                                                                COMP2 register is used when the counter is counting up */
    ETIMER_CMPMODE_COMP1_UP_COMP2_DOWN           = 0x02U,  /*!< COMP1 register is used when the counter is counting up;
                                                                COMP2 register is used when the counter is counting down */
    ETIMER_CMPMODE_COMP1_DOWN_COMP2_DOWN         = 0x03U,  /*!< COMP1 register is used when the counter is counting down;
                                                                COMP2 register is used when the counter is counting down */
} etimer_cmpmode_t;


/*!
 * @brief Capture functional mode.
 *
 * This will determine how input capture will work.
 *
 * Implements : etimer_cptmode_t_Class
 */
typedef enum
{
    ETIMER_CPTMODE_DISABLED     = 0x00U,  /*!< Disabled  */
    ETIMER_CPTMODE_FALLING      = 0x01U,  /*!< Capture falling edges */
    ETIMER_CPTMODE_RISING       = 0x02U,  /*!< Capture rising edges */
    ETIMER_CPTMODE_BOTH         = 0x03U,  /*!< Capture any edge */
} etimer_cptmode_t;

/*!
 * @brief oneshotmode.
 *
 * This will determine Co-channel Initialization.
 *
 * Implements : etimer_cochannelinitialization_t_Class
 */
typedef enum
{
    ETIMER_COINIT_NONE         = 0x00U,  /*!< Other channels cannot force re-initialization of this channel */
    ETIMER_COINIT_LOAD         = 0x01U,  /*!< Other channels may force a re-initialization of this channel's counter using the LOAD reg. */
    ETIMER_COINIT_CMPLD        = 0x02U,  /*!< Other channels may force a re-initialization of this channel's counter with the CMPLD2 reg when this channel is counting down or the CMPLD1 reg when this channel is counting up. */
}etimer_cochannelinitialization_t;

/*!
 * @brief DMA Request Select.
 *
 * This is used to set a DMA Request to use a DMA channel
 *
 * Implements : etimer_dma_req_select_t_Class
 */
typedef enum
{
    ETIMER_DMA_CH0_CAPT1   = 0x00U, /*!< Channel 0 CAPT1 DMA read request */
    ETIMER_DMA_CH0_CAPT2   = 0x01U, /*!< Channel 0 CAPT2 DMA read request */
    ETIMER_DMA_CH0_CMPLD1  = 0x02U, /*!< Channel 0 CMPLD1 DMA write request */
    ETIMER_DMA_CH0_CMPLD2  = 0x03U, /*!< Channel 0 CMPLD2 DMA write request */
    ETIMER_DMA_CH1_CAPT1   = 0x04U, /*!< Channel 1 CAPT1 DMA read request */
    ETIMER_DMA_CH1_CAPT2   = 0x05U, /*!< Channel 1 CAPT2 DMA read request */
    ETIMER_DMA_CH1_CMPLD1  = 0x06U, /*!< Channel 1 CMPLD1 DMA write request */
    ETIMER_DMA_CH1_CMPLD2  = 0x07U, /*!< Channel 1 CMPLD2 DMA write request */
    ETIMER_DMA_CH2_CAPT1   = 0x08U, /*!< Channel 2 CAPT1 DMA read request */
    ETIMER_DMA_CH2_CAPT2   = 0x09U, /*!< Channel 2 CAPT2 DMA read request */
    ETIMER_DMA_CH2_CMPLD1  = 0x0AU, /*!< Channel 2 CMPLD1 DMA write request */
    ETIMER_DMA_CH2_CMPLD2  = 0x0BU, /*!< Channel 2 CMPLD2 DMA write request */
    ETIMER_DMA_CH3_CAPT1   = 0x0CU, /*!< Channel 3 CAPT1 DMA read request */
    ETIMER_DMA_CH3_CAPT2   = 0x0DU, /*!< Channel 3 CAPT2 DMA read request */
    ETIMER_DMA_CH3_CMPLD1  = 0x0EU, /*!< Channel 3 CMPLD1 DMA write request */
    ETIMER_DMA_CH3_CMPLD2  = 0x0FU, /*!< Channel 3 CMPLD2 DMA write request */
    ETIMER_DMA_CH4_CAPT1   = 0x10U, /*!< Channel 4 CAPT1 DMA read request */
    ETIMER_DMA_CH4_CAPT2   = 0x11U, /*!< Channel 4 CAPT2 DMA read request */
    ETIMER_DMA_CH4_CMPLD1  = 0x12U, /*!< Channel 4 CMPLD1 DMA write request */
    ETIMER_DMA_CH4_CMPLD2  = 0x13U, /*!< Channel 4 CMPLD2 DMA write request */
    ETIMER_DMA_CH5_CAPT1   = 0x14U, /*!< Channel 5 CAPT1 DMA read request */
    ETIMER_DMA_CH5_CAPT2   = 0x15U, /*!< Channel 5 CAPT2 DMA read request */
    ETIMER_DMA_CH5_CMPLD1  = 0x16U, /*!< Channel 5 CMPLD1 DMA write request */
    ETIMER_DMA_CH5_CMPLD2  = 0x17U, /*!< Channel 5 CMPLD2 DMA write request */
} etimer_dma_req_select_t;

/*!
 * @brief Debug behavior.
 *
 * This sets the way eTimer behaves when debug is enabled and a breakpoint is hit
 *
 * Implements : etimer_debug_mode_t_Class
 */
typedef enum
{
    ETIMER_DEBUG_DEFAULT    = 0x00U, /*!< Continue with normal operation during debug mode */
    ETIMER_DEBUG_HALT       = 0x01U, /*!< Halt channel counter during debug mode */
    ETIMER_DEBUG_OFLAG_LOW  = 0x02U, /*!< Force OFLAG to logic 0 (prior to consideration of the OPS bit) during debug mode */
    ETIMER_DEBUG_COMBINED   = 0x03U, /*!< Both halt counter and force OFLAG to 0 during debug mode */
} etimer_debug_mode_t;

/*!
 * @brief ETIMER DMA channel structure.
 *
 * This is used to set the DMA channel request type and enable/disable.
 *
 * Implements : etimer_dma_channel_t_Class
 */
typedef struct
{
    bool                    enable;      /*!< Enable this DMA channel if true */
    etimer_dma_req_select_t request;     /*!< Select which type of request is connected to this DMA channel  */
} etimer_dma_channel_t;

/*!
 * @brief ETIMER input configuration structure.
 *
 * This is used to configure input glitch filtering
 *
 * Implements : etimer_input_filter_config_t_Class
 */
typedef struct
{
    etimer_filt_cnt_t    samples;      /*!< Number of consecutive samples that the input signal must have the same value */
    uint8_t              rate;         /*!< The rate at which the input is samples.
                                            Represents the sampling period (in peripheral clock cycles) of the eTimer input signal.  */
} etimer_input_filter_config_t;

/*!
 * @brief ETIMER input configuration structure.
 *
 * This is used to configure an input
 *
 * Implements : etimer_input_config_t_Class
 */
typedef struct
{
     etimer_input_source_t    source;      /*!< The source of the input signal */
     etimer_input_polarity_t  polarity;    /*!< The polarity of the input signal */
} etimer_input_config_t;

/*!
 * @brief ETIMER output configuration structure.
 *
 * This is used to configure the output signal
 *
 * Implements : etimer_output_config_t_Class
 */
typedef struct
{
     bool                     enable;      /*!< Enable the output signal */
     etimer_input_polarity_t  polarity;    /*!< The polarity of the output signal */
} etimer_output_config_t;

/*! @brief Structure to configure the channel timer
 *
 * This structure holds the configuration settings for one ETIMER channel
 *
 * Implements : etimer_user_channel_config_t_Class
 */
typedef struct
{
    etimer_cntmode_t                    countMode;                                       /*!< Operation mode of timer channel */
    etimer_input_filter_config_t        inputFilter;                                     /*!< Input glitch filter configuration  */
    etimer_input_config_t               primaryInput;                                    /*!< Primary input of the channel    */
    etimer_input_config_t               secondaryInput;                                  /*!< Secondary input of the channel  */
    etimer_output_config_t              outputPin;                                       /*!< Output pin of the channel  */
    etimer_count_direction_t            countDirection;                                  /*!< Which direction to count  */
    bool                                countLength;                                     /*!< Count until compare, then reinitialize.*/
    uint16_t                            compareValues  [ETIMER_NUM_COMPARE_REGISTERS];   /*!< Initial compare values, to be written in COMPx registers */
    etimer_clc_t                        compareLoading [ETIMER_NUM_COMPARE_REGISTERS];   /*!< Control what to do when a compare event occurs  */
    etimer_outmode_t                    compareOutputControl;                            /*!< Set the way output compare flag works, see documentation  */
    etimer_cmpmode_t                    compareMode;                                     /*!< Set how the COMPx registers are used for comparing  */
    etimer_cptmode_t                    captureControl [ETIMER_NUM_CAPTURE_REGISTERS];   /*!< Control the operation of the capture module  */
    uint8_t                             captureWords;                                    /*!< Set how many capture events to occur before setting the corresponding capture flag  */
    uint16_t                            interruptEnableMask;                             /*!< Enable interrupts for each channel, directly mapped to INTDMA register */
    bool                                oneshotCapture;                                  /*!< This bit selects between free running a and one shot mode for the input capture circuitry */
    bool                                countOnce;                                       /*!< Selects continuous or one shot counting mode */
    bool                                redundant;                                       /*!< Enables redundant channel checking between adjacent channels */
    bool                                masterMode;                                      /*!< Timer channel can be assigned as a Master */
    bool                                coChannelForce;                                  /*!< Co-channel OFLAG Force */
    bool                                coChannelVal;                                    /*!< This determines the value of the OFLAG output signal when COFRC signal occurs. */
    etimer_cochannelinitialization_t    coChannelInit;                                   /*!< Co-channel Initialization */
} etimer_user_channel_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Initializes the ETIMER module.
 *
 * This function resets ETIMER module, enables the ETIMER module.
 * This function affects all ETIMER channels.
 * This function should be called before calling any other ETIMER driver function.
 *
 * @param[in] instance ETIMER module instance number.
 */
void ETIMER_DRV_Init(const uint16_t instance);

/*!
 * @brief De-Initializes the ETIMER module.
 *
 * This function disables ETIMER module.
 * This function affects all ETIMER channels.
 * In order to use the ETIMER module again, ETIMER_DRV_Init must be called.
 *
 * @param[in] instance ETIMER module instance number
 */
void ETIMER_DRV_Deinit(const uint16_t instance);

/*!
 * @brief Gets the default configuration structure
 *
 * This function gets the default configuration structure with default settings
 *
 * @param[out] config The configuration structure
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_PRIMARY
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3
 *          - rate = 0
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1
 *          - polarity = ETIMER_POLARITY_POSITIVE
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN
 *          - polarity = ETIMER_POLARITY_POSITIVE
 *      - outputPin:
 *          - enable = false
 *          - polarity = ETIMER_POLARITY_POSITIVE
 *      - countDirection = ETIMER_COUNT_UP
 *      - countLength = false
 *      - compareValues:
 *          - [0] = 0
 *          - [1] = 0
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_NEVER
 *          - [1] = ETIMER_CLC_NEVER
 *      - compareOutputControl = ETIMER_OUTMODE_SOFTWARE
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED
 *          - [1] = ETIMER_CPTMODE_DISABLED
 *      - captureWords = 0
 *      - interruptEnableMask = 0
 *      - oneshotCapture = false
 *      - countOnce = false
 *      - redundant = false
 *      - masterMode = false
 *      - coChannelForce = false
 *      - coChannelVal = false
 *      - coChannelInit = ETIMER_COINIT_NONE
 */
void ETIMER_DRV_GetDefaultChannelConfig(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for ONE SHOT mode
 *
 * This function gets the default configuration structure with default settings for ONE SHOT mode
 *
 * @param[out] config The configuration structure
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_SECONDARY_AS_TRIGGER **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - outputPin:
 *          - enable = false  **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP **Fixed**
 *      - countLength = true **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Optional**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_FROM_CMPLD1_WHEN_COMP1 **Application specific**
 *          - [1] = ETIMER_CLC_NEVER **Optional**
 *      - compareOutputControl = ETIMER_OUTMODE_COMP1_SET_SECIN_CLEAR **Fixed**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP
 *      - captureControl:
 *          - [0] = ETIMER_CLC_FROM_CMPLD1_WHEN_COMP1
 *          - [1] = ETIMER_CPTMODE_DISABLED
 *      - captureWords = 0 **Optional**
 *      - interruptEnableMask = 0
 *      - oneshotCapture = false **Optional**
 *      - countOnce = false **Fixed**
 *      - redundant = false
 *      - masterMode = false
 *      - coChannelForce = false
 *      - coChannelVal = false
 *      - coChannelInit = ETIMER_COINIT_NONE
 */
void ETIMER_DRV_GetDefaultChannelConfigOneshot(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for PULSE-OUTPUT mode
 *
 * This function gets the default configuration structure with default settings for PULSE-OUTPUT mode
 *
 * @param[out] config The configuration structure.
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_PRIMARY **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Not used**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Not used**
 *      - outputPin:
 *          - enable = false  **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP **Fixed**
 *      - countLength = false **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Not used**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_NEVER **Application specific**
 *          - [1] = ETIMER_CLC_NEVER **Not used**
 *      - compareOutputControl = ETIMER_OUTMODE_COUNT_CLK **Fixed**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP **Not used**
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED **Not used**
 *          - [1] = ETIMER_CPTMODE_DISABLED **Not used**
 *      - captureWords = 0 **Not used**
 *      - interruptEnableMask = 0 **Optional**
 *      - oneshotCapture = false **Not used**
 *      - countOnce = false **Fixed**
 *      - redundant = false **Not used**
 *      - masterMode = false **Not used**
 *      - coChannelForce = false **Not used**
 *      - coChannelVal = false **Not used**
 *      - coChannelInit = ETIMER_COINIT_NONE **Not used**
 *
 */
void ETIMER_DRV_GetDefaultChannelConfigPulseOutput(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for FIXED-FREQUENCY PWM mode
 *
 * This function gets the default configuration structure with default settings for FIXED-FREQUENCY PWM mode
 *
 * @param[out] config The configuration structure.
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_PRIMARY **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Not used**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Not used**
 *      - outputPin:
 *          - enable = false **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP **Fixed**
 *      - countLength = false **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Not used**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_NEVER **Application specific**
 *          - [1] = ETIMER_CLC_NEVER **Not used**
 *      - compareOutputControl = ETIMER_OUTMODE_COMP_SET_OVF_CLEAR  **Fixed**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP **Not used**
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED **Not used**
 *          - [1] = ETIMER_CPTMODE_DISABLED **Not used**
 *      - captureWords = 0 **Not used**
 *      - interruptEnableMask = 0 **Optional**
 *      - oneshotCapture = false **Not used**
 *      - countOnce = false **Fixed**
 *      - redundant = false **Not used**
 *      - masterMode = false **Not used**
 *      - coChannelForce = false **Not used**
 *      - coChannelVal = false **Not used**
 *      - coChannelInit = ETIMER_COINIT_NONE **Not used**
 *
 */
void ETIMER_DRV_GetDefaultChannelConfigFixedFreqPwm(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for VARIABLE-FREQUENCY PWM mode
 *
 * This function gets the default configuration structure with default settings for VARIABLE-FREQUENCY PWM mode
 *
 * @param[out] config The configuration structure.
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_PRIMARY **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Not used**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Not used**
 *      - outputPin:
 *          - enable = false  **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP **Fixed**
 *      - countLength = true **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Application specific**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_NEVER **Application specific**
 *          - [1] = ETIMER_CLC_NEVER **Application specific**
 *      - compareOutputControl = ETIMER_OUTMODE_COMPARE_ALT_TOGGLE **Fixed**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_UP **Optional**
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED **Not used**
 *          - [1] = ETIMER_CPTMODE_DISABLED **Not used**
 *      - captureWords = 0 **Not used**
 *      - interruptEnableMask = 0 **Optional**
 *      - oneshotCapture = false **Not used**
 *      - countOnce = false **Fixed**
 *      - redundant = false **Not used**
 *      - masterMode = false **Not used**
 *      - coChannelForce = false **Not used**
 *      - coChannelVal = false **Not used**
 *      - coChannelInit = ETIMER_COINIT_NONE **Not used**
 *
 */
void ETIMER_DRV_GetDefaultChannelConfigVariableFreqPwm(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for MODULO COUNTING with direction mode
 *
 * This function gets the default configuration structure with default settings for MODULO COUNTING with direction mode
 *
 * @param[out] config The configuration structure.
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_SECONDARY_AS_DIRECTION **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - outputPin:
 *          - enable = false  **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP  **Not used**
 *      - countLength = false **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Application specific**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP2 **Fixed**
 *          - [1] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP1 **Fixed**
 *      - compareOutputControl = ETIMER_OUTMODE_SOFTWARE **Not used**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_DOWN **Fixed**
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED **Not used**
 *          - [1] = ETIMER_CPTMODE_DISABLED **Not used**
 *      - captureWords = 0 **Not used**
 *      - interruptEnableMask = 0 **Optional**
 *      - oneshotCapture = false **Not used**
 *      - countOnce = false **Fixed**
 *      - redundant = false **Not used**
 *      - masterMode = false **Not used**
 *      - coChannelForce = false **Not used**
 *      - coChannelVal = false **Not used**
 *      - coChannelInit = ETIMER_COINIT_NONE **Not used**
 *
 */
void ETIMER_DRV_GetDefaultChannelConfigModuleCountingDirection(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for MODULO COUNTING quadrature mode
 *
 * This function gets the default configuration structure with default settings for MODULO COUNTING quadrature mode
 *
 * @param[out] config The configuration structure.
 *  The initialized members:
 *      - countMode = ETIMER_CNTMODE_SECONDARY_AS_DIRECTION **Fixed**
 *      - inputFilter:
 *          - samples = ETIMER_FILT_CNT_3 **Optional**
 *          - rate = 0 **Optional**
 *      - primaryInput:
 *          - source = ETIMER_IN_SRC_CLK_DIV_1 **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - secondaryInput:
 *          - source = ETIMER_IN_SRC_CNT0_IN **Application specific**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Application specific**
 *      - outputPin:
 *          - enable = false  **Optional**
 *          - polarity = ETIMER_POLARITY_POSITIVE **Optional**
 *      - countDirection = ETIMER_COUNT_UP  **Not used**
 *      - countLength = false **Fixed**
 *      - compareValues:
 *          - [0] = 0 **Application specific**
 *          - [1] = 0 **Application specific**
 *      - compareLoading:
 *          - [0] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP2 **Fixed**
 *          - [1] = ETIMER_CLC_CNTR_WITH_CMPLD_WHEN_COMP1 **Fixed**
 *      - compareOutputControl = ETIMER_OUTMODE_SOFTWARE **Not used**
 *      - compareMode = ETIMER_CMPMODE_COMP1_UP_COMP2_DOWN **Fixed**
 *      - captureControl:
 *          - [0] = ETIMER_CPTMODE_DISABLED **Not used**
 *          - [1] = ETIMER_CPTMODE_DISABLED **Not used**
 *      - captureWords = 0 **Not used**
 *      - interruptEnableMask = 0 **Optional**
 *      - oneshotCapture = false **Not used**
 *      - countOnce = false **Fixed**
 *      - redundant = false **Not used**
 *      - masterMode = false **Not used**
 *      - coChannelForce = false **Not used**
 *      - coChannelVal = false **Not used**
 *      - coChannelInit = ETIMER_COINIT_NONE **Not used**
 *
 */
void ETIMER_DRV_GetDefaultChannelConfigModuleCountingQuadrature(etimer_user_channel_config_t *config);

/*!
 * @brief Gets the default configuration structure for DMA access
 *
 * This function gets the default configuration structure with default settings for DMA access
 *
 * @param[out] config The configuration structure
 */
void ETIMER_DRV_GetDefaultDmaChannel(etimer_dma_channel_t *config);

/*!
 * @brief Initializes the ETIMER channel.
 *
 * This function initializes the ETIMER channels. This function sets the following:
 * - driver mode;
 * - hardware counter mode;
 * - input glitch filter, one setting for both inputs;
 * - Primary input source;
 * - Secondary input source;
 * - output pin configuration;
 * - counting direction;
 * - initial compare values, for both COMPx registers;
 * - how to use the compare registers;
 * - loading from buffer registers;
 * - the way output signal is generated;
 * - how many input capture values should be kept before issuing an input capture event signal;
 * - what interrupt sources to enable;
 * The timer channel number and its configuration structure shall be passed as arguments.
 * Timer channels do not start counting by default after calling this function.
 * The function ETIMER_DRV_StartTimerChannels must be called to start the timer channel counting.
 * Parts of configuration structure are ignored in some combinations
 * of timer mode with internal hardware counter counter mode.
 *
 * This is an example demonstrating how to define a ETIMER channel configuration structure:
   @code
   etimer_user_channel_config_t etimerTestInit =
	{
        .countMode = ETIMER_CNTMODE_PRIMARY,
		.primaryInput=
		{
				ETIMER_IN_SRC_CLK_DIV_4,
				ETIMER_POLARITY_POSITIVE,
		},
		.secondaryInput=
		{
				ETIMER_IN_SRC_CNT1_IN,
				ETIMER_POLARITY_NEGATIVE,
		},
		.inputFilter=
		{
				ETIMER_FILT_CNT_3,
				0,
		},
		.outputPin=
		{
			true,
			ETIMER_POLARITY_POSITIVE,
		},
		.compareLoading=
		{
			ETIMER_CLC_FROM_CMPLD1_WHEN_COMP1,
			ETIMER_CLC_FROM_CMPLD2_WHEN_COMP2,
		},
		.compareValues=
		{
			0x1000,
	        0xffff,
		},
		.countDirection=ETIMER_COUNT_UP,
		.compareOutputControl=ETIMER_OUTMODE_SOFTWARE,
		.compareMode=ETIMER_CMPMODE_COMP1_UP_COMP2_UP,
		.captureControl
		{
			ETIMER_CPTMODE_DISABLED,
			ETIMER_CPTMODE_DISABLED,
		}
		.captureWords=0,
		.interruptEnableMask=ETIMER_CH_IRQ_SOURCE_TOFIE,
        .oneshot = ETIMER_MODE_FREE_RUNNING,
        .once = false,
        .redundant= false,
        .coChannelForce = false,
        .coChannelInit = ETIMER_COINIT_NONE,
        .masterMode = false,
	};
   @endcode
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel ETIMER channel number
 * @param[in] userChannelConfig Pointer to ETIMER channel configuration structure
 */
void ETIMER_DRV_InitChannel(const uint16_t instance,
                            const uint16_t channel,
                            const etimer_user_channel_config_t * const userChannelConfig);

/*!
 * @brief Initializes specified DMA channel for the ETIMER module.
 *
 * This configures ETIMER DMA access for the given DMA request channel.
 * The ETIMER DMA Request configuration structure shall be passed as argument.
 * It is recommended to call this function before starting any channels.
 *
 * This is an example demonstrating how to define a ETIMER DMA Channel configuration structure:
   @code
   etimer_dma_channel_t etimerTestDmaInitChannel =
   {
        .enable = true,
        .request = ETIMER_DMA_CH0_CAPT1
   };
   @endcode
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] dmaRequestNr ETIMER DMA Request channel
 * @param[in] singleDmaConfig Pointer to ETIMER DMA Request configuration structure
 */
void ETIMER_DRV_DmaInitRequest(const uint16_t instance,
                               const uint16_t dmaRequestNr,
                               const etimer_dma_channel_t * const singleDmaConfig);

/*!
 * @brief Controls the way each channel behaves when using a HW debugger.
 *
 * @param[in] instance ETIMER module instance number.
 * @param[in] channel ETIMER channel number.
 * @param[in] debug Enumeration of debug behavior modes, choose only one.
 */
void ETIMER_DRV_ChannelDebugBehaviour(const uint16_t instance,
                                      const uint16_t channel,
                                      const etimer_debug_mode_t debug);

/*! @} */

/*!
 * @name Channel start/stop control functions
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, the timer channels are going operate according to the way they were
 * previously configured.
 * No checks are done for unusable modes or other mode depended conditions.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask Timer channels starting mask that decides which channels
 * will be started
 * - For example:
 *      - with mask = ETIMER_ENABLE_CH0 then channel 0 will be started
 *      - with mask = ETIMER_ENABLE_CH1 then channel 1 will be started
 *      - with mask = (ETIMER_ENABLE_CH0|ETIMER_ENABLE_CH1) then channel 0 and channel 1 will be started
 */
void ETIMER_DRV_StartTimerChannels(const uint16_t instance,
		                           const uint16_t mask);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the ETIMER_DRV_StartTimerChannels.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask Timer channels stopping mask that decides which channels
 * will be stopped
 * - For example:
 *      - with mask = ETIMER_ENABLE_CH0 then channel 0 will be stopped
 *      - with mask = ETIMER_ENABLE_CH1 then channel 1 will be stopped
 *      - with mask = (ETIMER_ENABLE_CH0|ETIMER_ENABLE_CH1) then channel 0 and channel 1 will be stopped
 */
void ETIMER_DRV_StopTimerChannels(const uint16_t instance,
		                          const uint16_t mask);

/*!
 * @brief Checks if the given channel is running.
 *
 * This function reads if the channel was enabled and if the internal hardware counter was set in a mode which enables it to count.
 * It does not check if the module and/or channel input clocks are actually running.
 *
 * @param[in] instance ETIMER module instance number.
 * @param[in] channel Timer channel number.
 * @return true - the channel is running, timer is counting;
 *         false - the channel is stopped, timer is not counting.
 */
bool ETIMER_DRV_IsTimerRunning(const uint16_t instance,
                               const uint16_t channel);


/*! @} */

/*!
 * @name Channel counting period
 * @{
 */

/*!
 * @brief Sets the timer channel period in count units.
 *
 * This function sets the timer channel period in count units. The value must be less or equal than ETIMER_CH_MAX_TIMER_COUNT.
 * This function will write directly to the hardware register, without any buffering or delay
 * which may introduce functional glitches.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
void ETIMER_DRV_SetTimerTicks(const uint16_t instance,
                              const uint16_t channel,
                              const uint16_t count);

/*!
 * @brief Gets the current timer channel counting value in counts.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to ETIMER_CH_MAX_TIMER_COUNT.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @return Current channel timer counting value read directly from the hardware register
 */
uint16_t ETIMER_DRV_GetTimerTicks(const uint16_t instance,
                                  const uint16_t channel);

/*!
 * @brief Sets the timer channel period in count units.
 *
 * This function sets the timer channel period in count units. The value must be less or equal than ETIMER_CH_MAX_TIMER_COUNT.
 * This function will write indirectly to the hardware register by using a buffer register and allowing the counter hardware
 * to load the new value whenever it want, usually at overflow or underflow, thus preventing functional glitches.
 * It is recommended to use this function to change the counter in output compare modes or PWM modes.
 * Buffering must be enabled and properly configured for this function to work.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
void ETIMER_DRV_SetTimerTicksBuffered(const uint16_t instance,
                                      const uint16_t channel,
							          const uint16_t count);
/*! @} */

/*!
 * @name Channel interrupts control
 * @{
 */

/*!
 * @brief Enables the interrupt source for a timer channel.
 *
 * This function sets the interrupt enable bits of timer channels.
 *
 * @note All unset interrupt sources in the mask will keep their current state.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask The interrupt source enable mask that decides which interrupt sources will
 * be enabled
 * @param[in] channel Timer channel number
 * - For example:
 *      - with mask = ETIMER_CH_IRQ_SOURCE_TCFIE then an interrupt will be generated when
 *                                               a successful compare event happens;
 *      - with mask = ETIMER_CH_IRQ_SOURCE_TOFIE then an interrupt will be generated when
 *                                               the counter overflows or underflows depending on the counting direction;
 *      - with mask = (ETIMER_CH_IRQ_SOURCE_TCFIE|ETIMER_CH_IRQ_SOURCE_TOFIE) then an interrupt will be generated when
 *                                               a successful compare event happens and another interrupt will be generated when
 *                                               the counter overflows or underflows depending on the counting direction;
 */
void ETIMER_DRV_EnableInterruptSource(const uint16_t instance,
                                      const uint16_t mask,
                                      const uint16_t channel);

/*!
 * @brief Disables the interrupt source for a timer channel.
 *
 * This function clears the interrupt enable bits of timer channels.
 *
 * @note All unset interrupt sources in the mask will keep their current state.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask The interrupt source enable mask that decides which interrupt sources will
 * be disabled
 * @param[in] channel Timer channel number *
 */
void ETIMER_DRV_DisableInterruptSource(const uint16_t instance,
                                       const uint16_t mask,
                                       const uint16_t channel);

/*!
 * @brief Enables and disables the interrupt sources for a timer channel.
 *
 * This function sets the interrupt enable bits of timer channel and clears
 * all other interrupt sources which are not set through this operation.
 *
 * @note All unset interrupt sources in the mask will be disabled, disregarding their current state.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag
 * @param[in] channel Timer channel number
 * - For example:
 *      - with mask = ETIMER_CH_IRQ_SOURCE_TCFIE then an interrupt will be generated ONLY when
 *                                               a successful compare event happens;
 *      - with mask = ETIMER_CH_IRQ_SOURCE_TOFIE then an interrupt will be generated ONLY when
 *                                               the counter overflows or underflows depending on the counting direction;
 *      - with mask = (ETIMER_CH_IRQ_SOURCE_TCFIE|ETIMER_CH_IRQ_SOURCE_TOFIE) then an interrupt will be generated ONLY when
 *                                               a successful compare event happens OR when the counter overflows or underflows
 *                                               depending on the counting direction;
 */
void ETIMER_DRV_EnableDisableInterruptSources(const uint16_t instance,
                                              const uint16_t mask,
                                              const uint16_t channel);

/*!
 * @brief Gets the current interrupt flag of timer channels.
 *
 * This function gets the current interrupt flags of selected channels.
 *
 * @param[in] instance ETIMER module instance number.
 * @param[in] mask The interrupt flag getting mask that decides which channels will
 * be got interrupt flag.
 * @param[in] channel Timer channel number
 * - For example:
 *      - with mask = ETIMER_CH_IRQ_FLAGS_TCF then only the state of the compare flag is returned;
 *      - with mask = ETIMER_CH_IRQ_FLAGS_TOF then only the state of the overflow flag is returned;
 *      - with mask = (ETIMER_CH_IRQ_FLAGS_TCF|ETIMER_CH_IRQ_FLAGS_TOF) then only
 *                                            the states of the overflow flag and of the compare flag
 *                                            are returned.
 */
uint16_t ETIMER_DRV_GetInterruptStatus(const uint16_t instance,
                                       const uint16_t mask,
                                       const uint16_t channel);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] mask The interrupt flag clearing mask that decides which channels will
 * be cleared interrupt flag
 * @param[in] channel Timer channel number
 * - For example:
 *      - with mask = ETIMER_CH_IRQ_FLAGS_TCF then only the state of the compare flag is cleared;
 *      - with mask = ETIMER_CH_IRQ_FLAGS_TOF then only the state of the overflow flag is cleared;
 *      - with mask = (ETIMER_CH_IRQ_FLAGS_TCF|ETIMER_CH_IRQ_FLAGS_TOF) then only
 *                                            the states of the overflow flag and of the compare flag
 *                                            are cleared.
 */
void ETIMER_DRV_ClearInterruptStatus(const uint16_t instance,
		                             const uint16_t mask,
                                     const uint16_t channel);
/*!
 * @brief Returns the system interrupt vector number.
 *
 * This function return the system interrupt vector number corresponding to the selected channel.
 * This number is to be used with interrupt_manager to set the interrupt functions.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] vector the ETIMER specific vector for which to get the system interrupt vector number
 * @return System interrupt vector number or NULL if it does not exist on the system.
 */
IRQn_Type ETIMER_DRV_GetInterruptNumber(const uint16_t instance,
		                                const etimer_irq_vector_t vector);

/*! @} */

/*!
 * @name Timer Output Compare functions
 * @{
 */

/*!
 * @brief Set both compare registers directly.
 *
 * This function sets the timer channel compare values for both compare register, COMP1 and COMP2.
 * The value must be less or equal than ETIMER_CH_MAX_TIMER_COUNT.
 * This function will write directly to the hardware register, without any buffering or delay
 * which may introduce functional glitches.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] comp1 Timer channel value to be written in COMP1 register
 * @param[in] comp2 Timer channel value to be written in COMP2 register
 */
void ETIMER_DRV_SetCompareThreshold(const uint16_t instance,
                                    const uint16_t channel,
									const uint16_t comp1,
									const uint16_t comp2);

/*!
 * @brief Set both compare registers directly.
 *
 * This function sets the timer channel compare values for both compare register, COMP1 and COMP2.
 * The value must be less or equal than ETIMER_CH_MAX_TIMER_COUNT.
 * This function will write indirectly to the hardware register by using a buffer register and allowing the counter hardware
 * to load the new value whenever it want, usually at overflow or underflow, thus preventing functional glitches.
 * It is recommended to use this function to change the counter in output compare modes or PWM modes.
 * Buffering must be enabled and properly configured for this function to work.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] cmpld1 Timer channel value to be written in COMP1 register, via CMPLD1 register
 * @param[in] cmpld2 Timer channel value to be written in COMP2 register, via CMPLD2 register
 */
void ETIMER_DRV_SetCompareThresholdBuffered(const uint16_t instance,
                                    const uint16_t channel,
									const uint16_t cmpld1,
									const uint16_t cmpld2);

/*!
 * @brief Enable signal output to external pin.
 *
 * This function enables the output pin signal, which enables signal propagation to the outside
 * world through GPIO muxer.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 */
void ETIMER_DRV_OutputPinEnable(const uint16_t instance,
                                const uint16_t channel);

/*!
 * @brief Disable signal output to external pin.
 *
 * This function disables the output pin signal, which disables signal propagation to the outside
 * world through GPIO muxer. The state of the output pin after calling this function is not defined
 * by this driver of the ETIMER instance.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 */
void ETIMER_DRV_OutputPinDisable(const uint16_t instance,
                                 const uint16_t channel);

/*! @} */

/*!
 * @name Timer Input Capture functions
 * @{
 */

/*!
 * @brief Start input capture.
 *
 * Enables the ETIMER channel to capture the input signal and store the captured counter value.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 */
void ETIMER_DRV_StartCapture(const uint16_t instance,
                             const uint16_t channel);

/*!
 * @brief Stop an input capture.
 *
 * Disables the ETIMER channel to capture the input signal. It will not delete previously captured values.
 * If there is no ongoing capture, calling this function will have no effect.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 */
void ETIMER_DRV_StopCapture(const uint16_t instance,
                            const uint16_t channel);

/*!
 * @brief Read a captured value, this does not account for timer overflows.
 *
 * Reads the values captured upon capture event, which are already stored in the HW FIFO.
 * The captureWordsToRead parameter will limit how many words will be read
 * from the HW FIFO buffer.
 * Captured words which are not read remain in the HW FIFO buffer.
 *
 * User must ensure that all words stored in the HW FIFO buffer are read. To do this,
 * captureResultBuffer should have ETIMER_NUM_CAPTURE_WORD elements and
 * captureWordsToRead should be set to ETIMER_NUM_CAPTURE_WORD
 *
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] captureRegister Select which capture register to read
 * @param[out] captureResultBuffer Pointer to a buffer were the read data is going to be stored
 * @param[in] captureWordsToRead How many captured words to read from the HW FIFO
 */
uint16_t ETIMER_DRV_GetCaptureValue(const uint16_t instance,
                                    const uint16_t channel,
									const uint16_t captureRegister,
									uint16_t * const captureResultBuffer,
									const uint8_t captureWordsToRead);

/*!
 * @brief Returns the maximum number of captured words that can be stored in the HW FIFO.
 *
 * Each capture register has a HW FIFO which can store up to several captured values.
 * This function will return the maximum number of captured words that can be stored in the HW FIFO,
 * as configured at Init. After capturing these words the peripheral will signal that we have
 * captured something, the event can trigger interrupts if set.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 */
uint16_t ETIMER_DRV_GetCaptureConfigWords(const uint16_t instance,
                                          const uint16_t channel);

/*!
 * @brief Returns how many captured words are stored in the HW FIFO.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] captureRegister Select which capture register to read
 */
uint16_t ETIMER_DRV_GetCaptureWords(const uint16_t instance,
                                    const uint16_t channel,
									const uint16_t captureRegister);

/*! @} */

#ifdef FEATURE_ETIMER_HAS_WATCHDOG
/*!
 * @name Timer Watchdog functions
 * @{
 */

/*!
 * @brief Sets the watchdog value directly.
 *
 * This will set the watchdog timer for the corresponding ETIMER instance to the given value.
 * Accepts ETIMER_WATCHDOG_DISABLE_VALUE or ETIMER_WATCHDOG_MAX_VALUE or any number in-between as values.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 * @param[in] watchdog Value to be written in the watchdog timer counter
 */
void ETIMER_DRV_SetWatchdog(const uint16_t instance,
                            const uint16_t channel,
                            const uint32_t watchdog);

/*!
 * @brief Stops the watchdog timer.
 *
 * This is a convenience function for the user which uses ETIMER_DRV_SetWatchdog
 * with the watchdog parameter set to ETIMER_WATCHDOG_DISABLE_VALUE.
 *
 * @param[in] instance ETIMER module instance number
 * @param[in] channel Timer channel number
 *
 *
 */
void ETIMER_DRV_DisableWatchdog(const uint16_t instance,
                                const uint16_t channel);

/*! @} */
#endif /* FEATURE_ETIMER_HAS_WATCHDOG */

/*!
 * @brief Force output to logic level
 *
 * Set the output signal to 1 or 0 logic level overwriting any other output setting.
 * Careful! This function set compareOutputControl to ETIMER_OUTMODE_SOFTWARE to maintain
 *          the force logic level of the output pin.
 *          To set previous compareOutputControl you must use ETIMER_DRV_SetOutputFunction
 *
 * Careful! This function bypasses all sanity checks !
 *
 * @param[in] instance ETIMER instance number
 * @param[in] channel Timer channel number
 * @param[in] outputLogicLevel output state to be forced
 */
void ETIMER_DRV_ForceOutputLogicLevel(const uint16_t instance,
                                    const uint16_t channel,
									const bool outputLogicLevel);

/*!
 * @brief Read the values stored in the buffer compare registers
 *
 * Read the values stored in the buffer compare registers cmpld1 and cmpld2
 *
 * @param[in] instance ETIMER instance number
 * @param[in] channel Timer channel number
 * @param[out] cmpld1 The value stored in CMPLD1 register
 * @param[out] cmpld2 The value stored in CMPLD2 register
 */
void ETIMER_DRV_GetCompareThresholdBuffered(const uint16_t instance,
                                            const uint16_t channel,
									        uint16_t * const cmpld1,
									        uint16_t * const cmpld2);

/*!
 * @brief Reset the counter to specified value.
 *
 * Set the value to which the counter shall be reset to.
 * This function sets the timer channel LOAD register.
 * The value will be loaded after underflow/overflow or after an compare event.
 * This function also disables or enables timer rollover.
 * When rollover is disabled, the counter will reset to loaded value upon
 * the first compare event.
 *
 * @param[in] instance ETIMER instance number
 * @param[in] channel Timer channel number
 * @param[in] loadValue The value to be loaded after rollover or counter reset
 * @param[in] rolloverDisable If true then the counter will be reset
 *                            to the loaded value on a successful compare
 */
void ETIMER_DRV_ReloadOnCompare(const uint16_t instance,
                                    const uint16_t channel,
									const uint16_t loadValue,
									const bool rolloverDisable);

/*!
 * @brief Set way the output behaves
 *
 * Selects the way the output signal behaves.
 * There are several ways the output signal can behave, this will blindly set one of them.
 * Careful! This function bypasses driver mode checks !
 *
 * @param[in] instance ETIMER instance number
 * @param[in] channel Timer channel number
 * @param[in] outputMode output behavior selector
 */
void ETIMER_DRV_SetOutputFunction(const uint16_t instance,
                                    const uint16_t channel,
									const etimer_outmode_t outputMode);


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* ETIMER_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
