/*
 * Copyright 2018-2019 NXP
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

#ifndef SDADC_DRIVER_H
#define SDADC_DRIVER_H

/*! @file */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro defines a bitmask used to access status flags.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"
#include "edma_driver.h"
/*!
 * @addtogroup sdadc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Macros for status flags
 *
 * These flags map to internal hardware flags in the status flag register.
 * ORing these macros to clear multiple flags.
 *
 */
#define SDADC_FLAG_DATA_FIFO_EMPTY              (SDADC_SFR_DFEF_MASK)   /*!< DATA FIFO is empty */
#define SDADC_FLAG_WDG_UPPER_THRES_CROSS_OVER   (SDADC_SFR_WTHH_MASK)   /*!< Watchdog Upper Threshold is cross Over */
#define SDADC_FLAG_WDG_LOWER_THRES_CROSS_OVER   (SDADC_SFR_WTHL_MASK)   /*!< Watchdog Lower Threshold is cross Over */
#define SDADC_FLAG_CONVERTED_DATA_VALID         (SDADC_SFR_CDVF_MASK)   /*!< Converted Data is Valid */
#define SDADC_FLAG_DATA_FIFO_OVERRUN            (SDADC_SFR_DFORF_MASK)  /*!< Data FIFO is overrun */
#define SDADC_FLAG_DATA_FIFO_FULL               (SDADC_SFR_DFFF_MASK)   /*!< Data FIFO is full */
#define SDADC_FLAG_STATUS_ALL                   (SDADC_FLAG_DATA_FIFO_EMPTY | SDADC_FLAG_WDG_UPPER_THRES_CROSS_OVER | \
                                                 SDADC_FLAG_WDG_LOWER_THRES_CROSS_OVER | SDADC_FLAG_CONVERTED_DATA_VALID | \
                                                 SDADC_FLAG_DATA_FIFO_OVERRUN | SDADC_FLAG_DATA_FIFO_FULL)

/*!
 * @brief Macros for setting DMA and Interrupt request generating
 *
 * These macros are directly mapped to bits in the RSER register.
 * ORing these macros to setting DMA and Interrupt request generating of multiple events
 *
 */
#define SDADC_EVENT_FIFO_FULL          (SDADC_RSER_DFFDIRE_MASK)   /*!< Data FIFO Full Event */
#define SDADC_EVENT_WDOG_CROSSOVER     (SDADC_RSER_WTHDIRE_MASK)   /*!< WDG Threshold Cross Over Event */
#define SDADC_EVENT_FIFO_OVERRUN       (SDADC_RSER_DFORIE_MASK)    /*!< Data FIFO Full Event, this macro is only used to enable interrupt */
#define SDADC_EVENT_CONV_DATA_VALID    (SDADC_RSER_CDVEE_MASK)     /*!< Data FIFO Full Event, this macto is only used to enable data valid event output */

/*! @brief The max of signed 16-bit conversion data, it is (2 ^ 15) - 1 */
#define SDADC_MAX_CONV_DATA                 (32767)
/*! @brief The min of signed 16-bit conversion data, it is -(2 ^ 15) */
#define SDADC_MIN_CONV_DATA                 (-32768)

/*!
 * @brief Programmable Decimation Rate
 *
 * This enum is used to configure the programmable Decimation Rate
 *
 * Implements : sdadc_decimation_rate_t_Class
 */
typedef enum
{
    SDADC_DECIMATION_RATE_24 = 0x00u,   /*!< Oversampling ratio is 24 */
    SDADC_DECIMATION_RATE_28 = 0x01u,   /*!< Oversampling ratio is 28 */
    SDADC_DECIMATION_RATE_32 = 0x02u,   /*!< Oversampling ratio is 32 */
    SDADC_DECIMATION_RATE_36 = 0x03u,   /*!< Oversampling ratio is 36 */
    SDADC_DECIMATION_RATE_40 = 0x04u,   /*!< Oversampling ratio is 40 */
    SDADC_DECIMATION_RATE_44 = 0x05u,   /*!< Oversampling ratio is 44 */
    SDADC_DECIMATION_RATE_48 = 0x06u,   /*!< Oversampling ratio is 48 */
    SDADC_DECIMATION_RATE_56 = 0x07u,   /*!< Oversampling ratio is 56 */
    SDADC_DECIMATION_RATE_64 = 0x08u,   /*!< Oversampling ratio is 64 */
    SDADC_DECIMATION_RATE_72 = 0x09u,   /*!< Oversampling ratio is 72 */
    SDADC_DECIMATION_RATE_75 = 0x0Au,   /*!< Oversampling ratio is 75 */
    SDADC_DECIMATION_RATE_80 = 0x0Bu,   /*!< Oversampling ratio is 80 */
    SDADC_DECIMATION_RATE_88 = 0x0Cu,   /*!< Oversampling ratio is 88 */
    SDADC_DECIMATION_RATE_96 = 0x0Du,   /*!< Oversampling ratio is 96 */
    SDADC_DECIMATION_RATE_112 = 0x0Eu,  /*!< Oversampling ratio is 112 */
    SDADC_DECIMATION_RATE_128 = 0x0Fu,  /*!< Oversampling ratio is 128 */
    SDADC_DECIMATION_RATE_144 = 0x10u,  /*!< Oversampling ratio is 144 */
    SDADC_DECIMATION_RATE_160 = 0x11u,  /*!< Oversampling ratio is 160 */
    SDADC_DECIMATION_RATE_176 = 0x12u,  /*!< Oversampling ratio is 176 */
    SDADC_DECIMATION_RATE_192 = 0x13u,  /*!< Oversampling ratio is 192 */
    SDADC_DECIMATION_RATE_224 = 0x14u,  /*!< Oversampling ratio is 224 */
    SDADC_DECIMATION_RATE_256 = 0x15u   /*!< Oversampling ratio is 256 */
} sdadc_decimation_rate_t;

/*!
 * @brief Programmable Gain
 *
 * This enum is used to configure the programmable gain
 *
 * Implements : sdadc_input_gain_t_Class
 */
typedef enum
{
    SDADC_INPUT_GAIN_1  = 0x00u,   /*!< Input gain is 1 */
    SDADC_INPUT_GAIN_2  = 0x01u,   /*!< Input gain is 2 */
    SDADC_INPUT_GAIN_4  = 0x02u,   /*!< Input gain is 4 */
    SDADC_INPUT_GAIN_8  = 0x03u,   /*!< Input gain is 8 */
    SDADC_INPUT_GAIN_16 = 0x07u    /*!< Input gain is 16 */
} sdadc_input_gain_t;

/*!
 * @brief Trigger Source Selection
 *
 * This enum is used to configure the trigger source selection
 *
 * Implements : sdadc_trigger_select_t_Class
 */
typedef enum
{
    SDADC_TRIGGER_DISABLE  = 0xFFu,          /*!< Trigger is disable. The configured instance will not be triggered by any trigger source */
#ifdef FEATURE_SDADC_HAS_INSTANCE_0
    SDADC_SDADC0_SWTRIGGER_SELECT = 0x00u,   /*!< SDADC_0 output software trigger is selected. The configured instance will be triggered by SDADC0 SW trigger output */
#endif
    SDADC_SDADC1_SWTRIGGER_SELECT = 0x01u,   /*!< SDADC_1 output software trigger is selected. The configured instance will be triggered by SDADC1 SW trigger output */
    SDADC_SDADC2_SWTRIGGER_SELECT = 0x02u,   /*!< SDADC_2 output software trigger is selected. The configured instance will be triggered by SDADC2 SW trigger output */
#if (FEATURE_SDADC_HAS_INSTANCE_NUMBER > 3U)
    SDADC_SDADC3_SWTRIGGER_SELECT = 0x03u,   /*!< SDADC_3 output software trigger is selected. The configured instance will be triggered by SDADC3 SW trigger output */
    SDADC_SDADC4_SWTRIGGER_SELECT = 0x04u,   /*!< SDADC_4 output software trigger is selected. The configured instance will be triggered by SDADC4 SW trigger output */
#endif
#ifndef FEATURE_SDADC_HAS_COMMON_TRIGGER_SELECTION /* MPC5746R_SERIES*/
    SDADC_ETPU_A_CHANNEL_31_TRIGGER_SELECT = 0x05u,  /*!< ETPU_A channel 31 output trigger is selected. The configured instance will be triggered by ETPU_A channel 31 output */
    SDADC_ETPU_A_CHANNEL_30_TRIGGER_SELECT = 0x06u,  /*!< ETPU_A channel 30 output trigger is selected. The configured instance will be triggered by ETPU_A channel 30 output */
    SDADC_ETPU_A_CHANNEL_29_TRIGGER_SELECT = 0x07u,  /*!< ETPU_A channel 29 output trigger is selected. The configured instance will be triggered by ETPU_A channel 29 output */
    SDADC_ETPU_A_CHANNEL_28_TRIGGER_SELECT = 0x08u,  /*!< ETPU_A channel 28 output trigger is selected. The configured instance will be triggered by ETPU_A channel 28 output */
    SDADC_ETPU_B_CHANNEL_31_TRIGGER_SELECT = 0x09u,  /*!< ETPU_B channel 31 output trigger is selected. The configured instance will be triggered by ETPU_B channel 31 output */
    SDADC_ETPU_B_CHANNEL_30_TRIGGER_SELECT = 0x0Au,  /*!< ETPU_B channel 30 output trigger is selected. The configured instance will be triggered by ETPU_B channel 30 output */
    SDADC_ETPU_B_CHANNEL_29_TRIGGER_SELECT = 0x0Bu,  /*!< ETPU_B channel 29 output trigger is selected. The configured instance will be triggered by ETPU_B channel 29 output */
    SDADC_ETPU_B_CHANNEL_28_TRIGGER_SELECT = 0x0Cu,  /*!< ETPU_B channel 28 output trigger is selected. The configured instance will be triggered by ETPU_B channel 28 output */
    SDADC_EMIOS0_CHANNEL_23_TRIGGER_SELECT = 0x0Du,  /*!< EMIOS0 channel 23 output trigger is selected. The configured instance will be triggered by EMIOS0 channel 23 output */
    SDADC_EMIOS0_CHANNEL_22_TRIGGER_SELECT = 0x0Eu,  /*!< EMIOS0 channel 22 output trigger is selected. The configured instance will be triggered by EMIOS0 channel 22 output */
    SDADC_EMIOS0_CHANNEL_21_TRIGGER_SELECT = 0x0Fu,  /*!< EMIOS0 channel 21 output trigger is selected. The configured instance will be triggered by EMIOS0 channel 21 output */
    SDADC_EMIOS0_CHANNEL_20_TRIGGER_SELECT = 0x10u,  /*!< EMIOS0 channel 20 output trigger is selected. The configured instance will be triggered by EMIOS0 channel 20 output */
    SDADC_EMIOS1_CHANNEL_23_TRIGGER_SELECT = 0x11u,  /*!< EMIOS1 channel 23 output trigger is selected. The configured instance will be triggered by EMIOS1 channel 23 output */
    SDADC_EMIOS1_CHANNEL_22_TRIGGER_SELECT = 0x12u,  /*!< EMIOS1 channel 22 output trigger is selected. The configured instance will be triggered by EMIOS1 channel 22 output */
    SDADC_EMIOS1_CHANNEL_21_TRIGGER_SELECT = 0x13u,  /*!< EMIOS1 channel 21 output trigger is selected. The configured instance will be triggered by EMIOS1 channel 21 output */
    SDADC_EMIOS1_CHANNEL_20_TRIGGER_SELECT = 0x14u,  /*!< EMIOS1 channel 20 output trigger is selected. The configured instance will be triggered by EMIOS1 channel 20 output */
#else /* MPC5777C_SERIES */
    SDADC_ETPU_A_CHANNEL_20_22_24_26_TRIGGER_SELECT = 0x05, /*!< ETPU_A channel 20, 22, 24, 26 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_A channel output */
    SDADC_ETPU_A_CHANNEL_21_23_25_27_TRIGGER_SELECT = 0x06, /*!< ETPU_A channel 21, 23, 25, 27 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_A channel output */
    SDADC_ETPU_B_CHANNEL_20_22_24_26_TRIGGER_SELECT = 0x07, /*!< ETPU_B channel 20, 22, 24, 26 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_B channel output */
    SDADC_ETPU_B_CHANNEL_21_23_25_27_TRIGGER_SELECT = 0x08, /*!< ETPU_B channel 21, 23, 25, 27 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_B channel output */
    SDADC_ETPU_C_CHANNEL_15_17_19_21_TRIGGER_SELECT = 0x09, /*!< ETPU_C channel 15, 17, 19, 21 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_C channel output */
    SDADC_ETPU_C_CHANNEL_16_18_20_22_TRIGGER_SELECT = 0x0A  /*!< ETPU_C channel 16, 18, 20, 22 output trigger is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                 The configured instance will be triggered by the corresponding ETPU_C channel output */
#endif
} sdadc_trigger_select_t;

/*!
 * @brief Trigger Edge Selection
 *
 * This enum is used to select the input trigger edge
 *
 * Implements : sdadc_trigger_edge_t_Class
 */
typedef enum
{
    SDADC_TRIGGER_FALLING_EDGE = 0x00u,   /*!< Falling edge of trigger input is selected */
    SDADC_TRIGGER_RISING_EDGE  = 0x01u,   /*!< Rising edge of trigger input is selected */
    SDADC_TRIGGER_BOTH_EDGE    = 0x02u    /*!< Both edges of trigger input are selected */
} sdadc_trigger_edge_t;

/*!
 * @brief DMA/Interrupt global gating input selection
 *
 * This enum is used to select the DMA/Interrupt global gating input
 *
 * Implements : sdadc_dmaint_gate_select_t_Class
 */
typedef enum
{
#ifndef FEATURE_SDADC_HAS_COMMON_DMAINT_GATE_SELECTION /* MPC5746R_SERIES*/
    SDADC_ETPU_A_CHANNEL_7_OUTPUT_SELECT  = 0x01, /*!< ETPU_A channel 7 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_A channel 7 output is asserted */
    SDADC_ETPU_A_CHANNEL_6_OUTPUT_SELECT  = 0x02, /*!< ETPU_A channel 6 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_A channel 6 output is asserted */
    SDADC_ETPU_A_CHANNEL_5_OUTPUT_SELECT  = 0x03, /*!< ETPU_A channel 5 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_A channel 5 output is asserted */
    SDADC_ETPU_A_CHANNEL_4_OUTPUT_SELECT  = 0x04, /*!< ETPU_A channel 4 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_A channel 4 output is asserted */
    SDADC_ETPU_B_CHANNEL_7_OUTPUT_SELECT  = 0x05, /*!< ETPU_B channel 7 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_B channel 7 output is asserted */
    SDADC_ETPU_B_CHANNEL_6_OUTPUT_SELECT  = 0x06, /*!< ETPU_B channel 6 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_B channel 6 output is asserted */
    SDADC_ETPU_B_CHANNEL_5_OUTPUT_SELECT  = 0x07, /*!< ETPU_B channel 5 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_B channel 5 output is asserted */
    SDADC_ETPU_B_CHANNEL_4_OUTPUT_SELECT  = 0x08, /*!< ETPU_B channel 4 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when ETPU_B channel 4 output is asserted */
    SDADC_EMIOS0_CHANNEL_23_OUTPUT_SELECT = 0x09, /*!< EMIOS0 channel 23 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when EMIOS0 channel 23 output is asserted */
    SDADC_EMIOS0_CHANNEL_22_OUTPUT_SELECT = 0x0A, /*!< EMIOS0 channel 22 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when EMIOS0 channel 22 output is asserted */
    SDADC_EMIOS1_CHANNEL_23_OUTPUT_SELECT = 0x0B, /*!< EMIOS1 channel 23 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when EMIOS1 channel 23 output is asserted */
    SDADC_EMIOS1_CHANNEL_22_OUTPUT_SELECT = 0x0C, /*!< EMIOS1 channel 22 output is selected.
                                                       All DMA/interrupt requests of the configured instance are qualified only when EMIOS1 channel 22 output is asserted */
#else /* MPC5777C_SERIES */
    SDADC_ETPU_A_CHANNEL_9_10_11_12_OUTPUT_SELECT  = 0x00, /*!< ETPU_A channel 9, 10, 11, 12 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_A channel output is asserted */
    SDADC_ETPU_A_CHANNEL_1_2_19_30_OUTPUT_SELECT   = 0x01, /*!< ETPU_A channel 1, 2, 19, 30 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_A channel output is asserted */
    SDADC_ETPU_B_CHANNEL_9_10_11_12_OUTPUT_SELECT  = 0x02, /*!< ETPU_B channel 9, 10, 11, 12 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_B channel output is asserted */
    SDADC_ETPU_B_CHANNEL_1_2_19_30_OUTPUT_SELECT   = 0x03, /*!< ETPU_B channel 1, 2, 19, 30 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_B channel output is asserted */
    SDADC_ETPU_C_CHANNEL_7_8_9_10_OUTPUT_SELECT    = 0x04, /*!< ETPU_C channel 7, 8, 9, 10 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_C channel output is asserted */
    SDADC_ETPU_C_CHANNEL_27_28_29_30_OUTPUT_SELECT = 0x05  /*!< ETPU_C channel 27, 28, 29, 30 output is selected to correspond to SDADC instance 1, 2, 3, 4.
                                                                All DMA/interrupt requests of the configured instance are qualified only when the corresponding ETPU_C channel output is asserted */
#endif
} sdadc_dmaint_gate_select_t;

/*!
 * @brief Analog Channel Selection
 *
 * This enum is used to select analog input channel
 *
 * Implements : sdadc_inputchannel_sel_t_Class
 */
typedef enum
{
    SDADC_CHAN_AN0_VREFN = 0x20u,    /*!< In Single-ended input mode, AN0 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN1_VREFN = 0x21u,    /*!< In Single-ended input mode, AN1 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN2_VREFN = 0x22u,    /*!< In Single-ended input mode, AN2 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN3_VREFN = 0x23u,    /*!< In Single-ended input mode, AN3 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN4_VREFN = 0x24u,    /*!< In Single-ended input mode, AN4 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN5_VREFN = 0x25u,    /*!< In Single-ended input mode, AN5 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN6_VREFN = 0x26u,    /*!< In Single-ended input mode, AN6 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN7_VREFN = 0x27u,    /*!< In Single-ended input mode, AN7 input channel is connected to INP terminal, VREFN is connected to INM terminal */
    SDADC_CHAN_AN0_VREFP2 = 0x30u,   /*!< In Single-ended input mode, AN0 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN1_VREFP2 = 0x31u,   /*!< In Single-ended input mode, AN1 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN2_VREFP2 = 0x32u,   /*!< In Single-ended input mode, AN2 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN3_VREFP2 = 0x33u,   /*!< In Single-ended input mode, AN3 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN4_VREFP2 = 0x34u,   /*!< In Single-ended input mode, AN4 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN5_VREFP2 = 0x35u,   /*!< In Single-ended input mode, AN5 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN6_VREFP2 = 0x36u,   /*!< In Single-ended input mode, AN6 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN7_VREFP2 = 0x37u,   /*!< In Single-ended input mode, AN7 input channel is connected to INP terminal, VREFP/2 is connected to INM terminal */
    SDADC_CHAN_AN0_AN1 = 0x00u,      /*!< In Differential input mode, AN0 and AN1 input channels are connected to INP and INM terminals respectively */
    SDADC_CHAN_AN2_AN3 = 0x01u,      /*!< In Differential input mode, AN2 and AN3 input channels are connected to INP and INM terminals respectively */
    SDADC_CHAN_AN4_AN5 = 0x02u,      /*!< In Differential input mode, AN4 and AN5 input channels are connected to INP and INM terminals respectively */
    SDADC_CHAN_AN6_AN7 = 0x03u,      /*!< In Differential input mode, AN6 and AN7 input channels are connected to INP and INM terminals respectively */
    SDADC_CHAN_VREFN_VREFN = 0x04u,   /*!< In Differential input mode, VREFN is connected to INP and INM terminals */
    SDADC_CHAN_VREFP2_VREFP2 = 0x05u, /*!< In Differential input mode, VREFP/2 is connected to INP and INM terminals */
    SDADC_CHAN_VREFP_VREFN = 0x06u,   /*!< In Differential input mode, VREFP and VREFN are connected to INP and INM terminals respectively */
    SDADC_CHAN_VREFN_VREFP = 0x07u,   /*!< In Differential input mode, VREFN and VREFP are connected to INP and INM terminals respectively */
} sdadc_inputchannel_sel_t;

/*!
 * @brief Structure for DMA parameters for moving data from a result fifo into system memory.
 *
 * This structure is used for configuring DMA parameters for moving data from a result fifo into system memory.
 *
 * Implements : sdadc_result_dma_config_t_Class
 */
typedef struct
{
    bool fifoFullDmaReq;         /*! Generate DMA request when data fifo full event happens */
    bool wdogOverDmaReq;         /*! Generate DMA request when watchdog cross over event happens */
    uint8_t dmaVirtualChan;      /*! DMA virtual channel to be used if SDADC generates the DMA request */
    uint16_t * destPtr;          /*! Pointer to the memory address where the results will be copied by the DMA, if DMA transfer is enabled */
    uint32_t destLength;         /*! Length (num of uint16_t words) of the destination buffer pointed by destPtr */
    edma_callback_t callback;    /*! Callback function called when the destination buffer has been filled by the DMA */
    void * callbackParam;        /*! Parameter to be passed to the callback function pointer */
    bool enHalfDestCallback;     /*! Enable the callback to be called when half of the destination buffer has been filled */
} sdadc_result_dma_config_t;

/*!
 * @brief Defines the converter configuration
 *
 * This structure is used to configure the SDADC converter
 *
 * Implements : sdadc_conv_config_t_Class
 */
typedef struct
{
    sdadc_inputchannel_sel_t channelSel;  /*!< Select analog input is connected to SDADC terminals.
                                               When Wrap Around mode is enabled, indicates the initial entry value for the first loop of the wraparound sequence */
    sdadc_decimation_rate_t decimaRate;   /*!< Programmable Decimation Rate */
    sdadc_input_gain_t inputGain;         /*!< Programmable Gain */
    sdadc_trigger_select_t trigSelect;    /*!< Trigger input selection */
    sdadc_trigger_edge_t trigEdge;        /*!< Trigger edge selection, this member is not influent if trigger selection is disabled */
    uint8_t outputSetDelay;               /*!< Output Settling Delay */
    bool highPassFilter;                  /*!< High Pass Filter Enable */
    bool wrapAroundEnable;                /*!< Enable Wrap-Around mode */
    sdadc_inputchannel_sel_t wraparound;  /*!< When Wrap Around mode is enabled, this indicates
                                               the maximum value of the wraparound counter */
    bool enableFifo;                      /*!< Enable FIFO */
    uint8_t fifoThreshold;                /*!< FIFO Threshold */
    bool stopInDebug;                     /*!< Enable stopping the SDADC conversions when the chip enters debug mode*/
    sdadc_result_dma_config_t * resultDmaConfig;   /*! Pointer to a configuration structure for DMA transferring from data fifo.
                                                       If DMA support is selected for data fifo full or watchdog crossover event, then the SDADC will issue DMA request
                                                       whenever Data Fifo Full Flag or Watchdog Crossover flags is set and the result will be transfered
                                                       to the corresponding configured destination buffer.
                                                       By default, DMA support is not selected for these events. */
} sdadc_conv_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the converter configuration structure
 *
 * This function initializes the members of the sdadc_conv_config_t
 * structure to default values which are most commonly used for SDADC.
 * This function should be called on a structure before using it to configure the converter with
 * SDADC_DRV_ConfigConverter(), otherwise all members must be written by the user.
 * The user can modify the desired members of the structure after calling this function.
 * The below is default configuration:
 *
 * - Decimation Rate: SDADC_DECIMATION_RATE_24
 * - Gain: SDADC_INPUT_GAIN_1
 * - Trigger selection: SDADC_TRIGGER_DISABLE
 * - Trigger edge: SDADC_TRIGGER_RISING_EDGE
 * - Enable FIFO: Enable
 * - FIFO Threshold : 16
 * - Output Settling Delay: 0xFF
 * - High Pass Filter: Disable
 * - Wrap-Around mode: Disable
 * - Wrap-around value: SDADC_CHAN_AN0_AN1
 * - Input channel select: SDADC_CHAN_AN0_VREFN
 * - Stop in debug: Enable
 * - DMA configuration: NULL
 *
 * @param[out] config configuration structure pointer
 */
void SDADC_DRV_GetConverterDefaultConfig(sdadc_conv_config_t * const config);

/*!
 * @brief Configures the converter with the given configuration structure
 *
 * This function configures the SDADC converter with the options
 * provided in the structure. Converter does not start by default after calling this
 * function. The SDADC_DRV_RefreshConversion() or SDADC_DRV_SwTriggerConv() function must be called to start conversions.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] config Configuration structure pointer
 */
void SDADC_DRV_ConfigConverter(const uint32_t instance,
                               const sdadc_conv_config_t * const config);

/*!
 * @brief Reset the SDADC
 *
 * This function resets the SDADC internal registers to their Reference Manual reset values.
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_Reset(const uint32_t instance);

/*!
 * @brief Power up the SDADC
 *
 * This function enables the SDADC block.
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_Powerup(const uint32_t instance);

/*!
 * @brief Power down the SDADC
 *
 * This function disables the SDADC, SDADC internal modulator placed in low consumption mode.
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_Powerdown(const uint32_t instance);

/*!
 * @brief Refresh SDADC conversion
 *
 * This function resets SDADC internal modulator to start a fresh conversion.
 * When the input trigger is disabled, this function must be call after changing converter configuration(gain, input channel, trigger, watchdog...).
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_RefreshConversion(const uint32_t instance);

/*!
 * @brief Setting input gain
 *
 * This function configures the gain to be applied to the analog input stage of the SDADC.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] gain The input gain
 */
void SDADC_DRV_SetInputGain(const uint32_t instance,
                            const sdadc_input_gain_t gain);

/*!
 * @brief Setting decimation rate
 *
 * This function configures the over-sampling ratio to be applied to support different passbands
 * with a fixed input sampling clock.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] rate The decimation rate
 */
void SDADC_DRV_SetDecimationRate(const uint32_t instance,
                                 const sdadc_decimation_rate_t rate);

/*!
 * @brief Setting Output Settling Delay
 *
 * This function configures the output settling delay to be applied to qualify the
 * converted output data coming from the SDADC.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] outputSettlingDelay The output settling delay value
 */
void SDADC_DRV_SetOutputSettlingDelay(const uint32_t instance,
                                      const uint8_t outputSettlingDelay);

/*!
 * @brief Enable analog input bias
 *
 * This function enables analog input bias, the analog input will be connected to
 * half-scale bias(VREFP/2).
 * Note that: The electrical settings of the pins(E.g: Pull-up, Pull-down) can impact to the biased input.
 * For example: If the pull-up of pin is enabled, the actual measured voltage of the biased input is bigger than half-scale(VREFP/2).
 *
 * @param[in] instance The SDADC instance number
 * @param[in] inputMask The mask of analog inputs to enable/disable bias
 * @param[in] enable Enable/disable analog input bias
 *            - True: enable analog input bias
 *            - False: disable analog input bias
 * - For example:
 *      - with inputMask = 0x01U and enable is "true", the input 0 will be enabled bias
 *      - with inputMask = 0x02U and enable is "true", the input 1 will be enabled bias
 *      - with inputMask = 0x03U and enable is "false",the input 0 and input 1 will be disable bias
 */
void SDADC_DRV_SetAnalogInputBias(const uint32_t instance,
                                  const uint8_t inputMask,
                                  const bool enable);

/*!
 * @brief Setting watchdog monitor
 *
 * This function configures the watch dog monitor with given parameters.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] wdgEnable Enable the watchdog monitor
 * @param[in] upperThreshold Watchdog upper threshold value
 * @param[in] lowerThreshold Watchdog lower threshold value
 */
void SDADC_DRV_SetWatchdog(const uint32_t instance,
                           const bool wdgEnable,
                           const int16_t upperThreshold,
                           const int16_t lowerThreshold);

/*!
 * @brief Setting Wrap Around mode
 *
 * This function configures the wraparound mechanism for conversion of programmed sequence of channels.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] wraparound The wraparound value
 * @param[in] enable Enable/disable wrap around mode
 *            - True: enable wrap around mode
 *            - False: disable wrap around mode
 */
void SDADC_DRV_SetWraparoundMode(const uint32_t instance,
                                 const sdadc_inputchannel_sel_t wraparound,
                                 const bool enable);

/*!
 * @brief Select input analog channel
 *
 * This function configures the connectivity of analog inputs to either positive or negative polarity
 * terminals of the SDADC. If wraparound mode is enabled, this function supports to configure
 * initial entry value for the first loop of the wraparound sequence.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] channel The input channel selection
 */
void SDADC_DRV_SelectInputChannel(const uint32_t instance,
                                  const sdadc_inputchannel_sel_t channel);

/*!
 * @brief Generate software trigger
 *
 * This function generates the trigger event output which can be used for triggering conversions.
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_SwTriggerConv(const uint32_t instance);

/*!
 * @brief Flush data Fifo
 *
 * This function flush data fifo, all data in the fifo will be erased.
 * This function should be called after SDADC is disabled.
 * This function is ignored if have no data in fifo.
 *
 * @param[in] instance The SDADC instance number
 */
void SDADC_DRV_FlushDataFifo(const uint32_t instance);

/*!
 * @brief Gets the conversion data in Fifo.
 *
 * This function gets the converted data from the fifo and put the data into data array.
 * The data will be consecutive popped out the fifo until the fifo is empty or the data array is full,
 * so the data array length should be big enough to contain all data.
 * Note that: This function automatically calibrates the converted data.
 * Please use the SDADC_DRV_GetRawConvDataFifo function to get raw data(uncalibrated data).
 *
 * @param[in] instance The SDADC instance number
 * @param[in] length The length of data array
 * @param[out] data The data array which contain the converted data
 * @return The actual number of values read from the FIFO
 */
uint8_t SDADC_DRV_GetConvDataFifo(const uint32_t instance,
                                  const uint8_t length,
                                  int16_t * const data);

/*!
 * @brief Gets the raw conversion data in Fifo.
 *
 * This function gets the raw converted data(uncalibrated data) from the fifo and put the data into data array.
 * The data will be consecutive popped out the fifo until the fifo is empty or the data array is full,
 * so the data array length should be big enough to contain all data.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] length The length of data array
 * @param[out] data The data array which contain the converted data
 * @return The actual number of values read from the FIFO
 */
uint8_t SDADC_DRV_GetRawConvDataFifo(const uint32_t instance,
                                     const uint8_t length,
                                     uint16_t * const data);

/*!
 * @brief Get the status flags
 *
 * This function returns the status flags of the SDADC.
 * Bitwise AND the returned value with the SDADC_FLAG_ defines to get a specific status flag.
 *
 * @param[in] instance The SDADC instance number
 * @return The status flags
 */
uint32_t SDADC_DRV_GetStatusFlags(const uint32_t instance);

/*!
 * @brief Clear the status flags
 *
 * This function clears the status flags that are set to '1' in the mask.
 * The mask input parameter can be set using SDADC_FLAG_ defines.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] mask Bit-mask of flags to clear
 *  - For example:
 *       - With mask = SDADC_FLAG_DATA_FIFO_FULL to clear Data FIFO Full Flag(DFFF).
 *       - With mask = (SDADC_FLAG_DATA_FIFO_FULL | SDADC_FLAG_DATA_FIFO_OVERRUN)
 *                      to clear Data FIFO Full Flag(DFFF) and Data FIFO Overrun Flag(DFORF).
 */
void SDADC_DRV_ClearStatusFlags(const uint32_t instance,
                                const uint32_t mask);

/*!
 * @brief Configure SDADC Global DMA/Interrupt gate
 *
 * This function configures SDADC Global DMA/Interrupt gate.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] select The DMA/Interrupt gating input
 * @param[in] enable Enable/disable global DMA/Interrupt gate
 *            - True: enable gate
 *            - False: disable gate
 */
void SDADC_DRV_SetGlobalDmaInterruptGate(const uint32_t instance,
                                         const sdadc_dmaint_gate_select_t select,
                                         const bool enable);

/*!
 * @brief Enables SDADC DMA request generating
 *
 * This function enables SDADC DMA requests that are set to '1' in the event_mask.
 * The event_mask input parameter can be set using SDADC_EVENT_ defines. Bitwise OR the macro defines
 * to enable multiple DMA requests.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] event_mask The bit-mask of SDADC events to enable DMA request generating
 *
 * - For example: To enable DMA request generating of Data FIFO Full Event, the mask must be
 *              "SDADC_EVENT_FIFO_FULL".
 */
void SDADC_DRV_EnableDmaEvents(const uint32_t instance,
                               const uint32_t event_mask);

/*!
 * @brief Enables SDADC interrupt request generating
 *
 * This function enables SDADC interrupt requests that are set to '1' in the event_mask.
 * The event_mask input parameter can be set using SDADC_EVENT_ defines. Bitwise OR the macro defines
 * to enable multiple interrupt requests.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] event_mask The bit-mask of SDADC events to enable interrupt request generating
 *
 * - For example: To enable interrupt request generating of Data FIFO Full Event, the mask must be
 *              "SDADC_EVENT_FIFO_FULL".
 */
void SDADC_DRV_EnableInterruptEvents(const uint32_t instance,
                                     const uint32_t event_mask);

/*!
 * @brief Disable SDADC DMA and interrupt request generating
 *
 * This function disable SDADC DMA and interrupt requests that are set to '1' in the event_mask.
 * The event_mask input parameter can be set using SDADC_EVENT_ defines. Bitwise OR the macro defines
 * to disable multiple interrupt/DMA requests.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] event_mask The bit-mask of SDADC event to disable DMA and interrupt request generating
 *
 * - For example: To disable DMA and interrupt request generating of Data FIFO Full Event, the mask must be
 *               "SDADC_EVENT_FIFO_FULL".
 */
void SDADC_DRV_DisableEvents(const uint32_t instance,
                             const uint32_t event_mask);

/*!
 * @brief Returns the interrupt number for the SDADC instance.
 *
 * This function returns the interrupt number for the specified SDADC instance.
 * Note that: depending on platform, there might be multiple SDADC instances which use the same IRQ number.
 *
 * @param[in] instance The SDADC instance number
 * @return The interrupt number (index) of the SDADC instance, used to configure the interrupt
 */
IRQn_Type SDADC_DRV_GetInterruptNumber(const uint32_t instance);

/*!
 * @brief Perform Gain Calibration of the SDADC
 *
 * This function performs a gain calibration of the SDADC. Gain calibration
 * should be run before using the SDADC converter or after the operating conditions
 * (particularly Vref) change significantly. The measured gain value is going be used to
 * nullify the gain errors in the data conversion.
 * The conversion number that is performed by calibration should be from 16 to 64. The higher the number
 * of conversion done, the higher the rejection of noise during the calibration.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] convNum The conversion number used for gain calibration
 */
void SDADC_DRV_GainCalibration(const uint32_t instance,
                               const uint8_t convNum);

/*!
 * @brief Perform Offset Calibration of the SDADC
 *
 * This function performs a offset calibration of the SDADC. Offset calibration
 * should be run before using the SDADC converter or after the operating conditions
 * (particularly Vref, input gain) change significantly.
 * The measured offset is going be used to nullify the offset error in the data conversion.
 * The offset calibration must be performed for each input gain changing since it is expected to
 * vary with input gain configuration of SDADC.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] userGain The input gain at which the offset calibration is perform
 */
void SDADC_DRV_OffsetCalibration(const uint32_t instance,
                                 const sdadc_input_gain_t userGain);

/*!
 * @brief Calibrate the converted data of the SDADC
 *
 * This function calibrates the uncalibrated converted data which is got by DMA or SDADC_DRV_GetRawConvDataFifo function.
 * In the case uncalibrated converted data which is got by DMA, this function should be called in the DMA callback.
 *
 * @param[in] instance The SDADC instance number
 * @param[in] uncalibratedBuffer The pointer to the converted data array which contains uncalibrated data.
 * @param[out] calibratedBuffer The pointer to the data array which contains data after calibrated.
 * @param[in] bufferLength The length of the data array which contains uncalibrated data.
 */
void SDADC_DRV_CalibrateDataSet(const uint32_t instance,
                                const uint16_t * const uncalibratedBuffer,
                                int32_t * const calibratedBuffer,
                                const uint32_t bufferLength);

#if defined (__cplusplus)
}
#endif

/*! @} */

#endif /* SDADC_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
