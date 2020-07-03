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

#ifndef CTU_DRIVER_H
#define CTU_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*! @file ctu_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros define bitmasks used to access different flags.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * Function-like macros are used to create bit-masks for input trigger select, based on trigger index
 *
 */

/*!
 * @addtogroup ctu_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Defines the CTU prescaler values
 *
 * Implements : ctu_prescaler_t_Class
 */
typedef enum {
    CTU_PRESCALER_1                 = 0u,   /*!< CTU clock prescaler value of 1. */
    CTU_PRESCALER_2                 = 1u,   /*!< CTU clock prescaler value of 2. */
    CTU_PRESCALER_3                 = 2u,   /*!< CTU clock prescaler value of 3. */
    CTU_PRESCALER_4                 = 3u    /*!< CTU clock prescaler value of 4. */
} ctu_prescaler_t;

/*!
 * @brief Defines the CTU Trigger Generator Subunit (TGS) operating modes
 *
 * Implements : ctu_tgs_mode_t_Class
 */
typedef enum {
    CTU_TGS_MODE_TRIGGERED          = 0u,   /*!< Trigger Generator Subunit (TGS) Triggered Mode. */
    CTU_TGS_MODE_SEQUENTIAL         = 1u    /*!< Trigger Generator Subunit (TGS) Sequential Mode. */
} ctu_tgs_mode_t;

/*!
 * @brief Defines the CTU output external trigger mode
 *
 * Implements : ctu_ext_trig_mode_t_Class
 */
typedef enum {
    CTU_EXT_TRIG_MODE_PULSE         = 0u,   /*!< Pulse mode for output external trigger */
    CTU_EXT_TRIG_MODE_TOGGLE        = 1u    /*!< Toggle mode for output external trigger */
} ctu_ext_trig_mode_t;

/*!
 * @brief Defines the CTU ADC command list operating mode
 *
 * Implements : ctu_adc_cmd_list_mode_t_Class
 */
typedef enum {
    CTU_ADC_CMD_LIST_MODE_STREAMING = 0u,   /*!< ADC Command List in streaming mode: no more than a list executing at a time. */
    CTU_ADC_CMD_LIST_MODE_PARALLEL  = 1u    /*!< ADC Command List in parallel mode: lists may execute in parallel, if targeting different ADCs.  */
} ctu_adc_cmd_list_mode_t;

/*!
 * @brief Defines the CTU signal edges for input selection
 *
 * Implements : ctu_input_edge_t_Class
 */
typedef enum {
    CTU_INPUT_EDGE_RISING           = 0u,   /*!< Input Selection Rising Edge */
    CTU_INPUT_EDGE_FALLING          = 1u,   /*!< Input Selection Falling Edge */
    CTU_INPUT_EDGE_BOTH             = 2u    /*!< Input Selection both rising and falling edges */
} ctu_input_edge_t;

/*!
 * @brief Defines the CTU input triggers
 *
 * This enumeration defines the CTU input triggers. Inputs may be assigned to
 * different trigger sources, depending on each CTU instance. Please refer
 * to device reference manual for actual assignments - chapter regarding TGSISR input assignments.
 *
 * Implements : ctu_input_trig_t_Class
 */
typedef enum {
    CTU_INPUT_TRIG_PWM_REL          = 0u,   /*!< Input trigger PWM Reload */
    CTU_INPUT_TRIG_PWM_ODD_0        = 1u,   /*!< Input trigger PWM ODD 0  */
    CTU_INPUT_TRIG_PWM_ODD_1        = 2u,   /*!< Input trigger PWM ODD 1  */
    CTU_INPUT_TRIG_PWM_ODD_2        = 3u,   /*!< Input trigger PWM ODD 2  */
    CTU_INPUT_TRIG_PWM_ODD_3        = 4u,   /*!< Input trigger PWM ODD 3  */
    CTU_INPUT_TRIG_PWM_EVEN_0       = 5u,   /*!< Input trigger PWM EVEN 0 */
    CTU_INPUT_TRIG_PWM_EVEN_1       = 6u,   /*!< Input trigger PWM EVEN 1 */
    CTU_INPUT_TRIG_PWM_EVEN_2       = 7u,   /*!< Input trigger PWM EVEN 2 */
    CTU_INPUT_TRIG_PWM_EVEN_3       = 8u,   /*!< Input trigger PWM EVEN 3 */
    CTU_INPUT_TRIG_RPWM_0           = 9u,   /*!< Input trigger RPWM 0     */
    CTU_INPUT_TRIG_RPWM_1           = 10u,  /*!< Input trigger RPWM 1     */
    CTU_INPUT_TRIG_RPWM_2           = 11u,  /*!< Input trigger RPWM 2     */
    CTU_INPUT_TRIG_RPWM_3           = 12u,  /*!< Input trigger RPWM 3     */
    CTU_INPUT_TRIG_ETIMER0_IN       = 13u,  /*!< Input trigger ETIMER 0   */
    CTU_INPUT_TRIG_ETIMER1_IN       = 14u,  /*!< Input trigger ETIMER 1   */
    CTU_INPUT_TRIG_EXT_IN           = 15u   /*!< Input trigger EXTERNAL   */
} ctu_input_trig_t;

/*!
 * @brief Defines the CTU conversion mode
 *
 * Implements : ctu_conv_mode_t_Class
 */
typedef enum {
    CTU_CONV_MODE_SINGLE            = 0u,   /*!< Single conversion mode - conversion will run on a single ADC instance */
    CTU_CONV_MODE_DUAL              = 1u    /*!< Dual conversion mode - conversion will run on two ADC instances simultaneously */
} ctu_conv_mode_t;

/*!
 * @brief Defines the available ADC ports connected to CTU
 *
 * Implements : ctu_adc_port_t_Class
 */
typedef enum {
    CTU_ADC_PORT_A                  = 0u,   /*!< ADC instance 0 */
    CTU_ADC_PORT_B                  = 1u    /*!< ADC instance 1 */
} ctu_adc_port_t;

/*!
 * @brief Defines the ADC self test algorithms available from CTU
 *
 * Implements : ctu_adc_st_alg_t_Class
 */
typedef enum {
    CTU_ADC_ST_ALG_S                = 0u,   /*!< ADC Self Test "S" algorithm */
    CTU_ADC_ST_ALG_C                = 2u,   /*!< ADC Self Test "C" algorithm */
    CTU_ADC_ST_ALG_S_C              = 3u    /*!< ADC Self Test "S + C" algorithm */
} ctu_adc_st_alg_t;

/*!
 * @brief Defines the possible data alignment for ADC conversion results
 * available from CTU result FIFOs
 *
 * Implements : ctu_data_align_t_Class
 */
typedef enum
{
    CTU_RESULT_ALIGN_RIGHT_UNSIGNED = 0u,   /*!< FRn: data is unsigned, right aligned */
    CTU_RESULT_ALIGN_LEFT_SIGNED    = 1u    /*!< FLn: data is signed, left aligned - leftmost bit is sign bit. */
} ctu_data_align_t;


/*!
* @brief Structure for configuring global CTU functionalities
*
* This structure is used for configuring global CTU functionalities
*
* Implements : ctu_config_t_Class
*/
typedef struct {
   ctu_tgs_mode_t tgsMode;                  /*!< Trigger Generator Subunit (TGS) Mode - double-buffered (TGSCR.TGS_M) */
   ctu_prescaler_t prescaler;               /*!< Input clock prescaler - double-buffered (TGSCR.PRES) */
   ctu_ext_trig_mode_t extTrigMode;         /*!< Trigger mode for external triggers: eTimers and EXT_TRG - double-buffered (TGSCR.ET_TM) */
   uint32_t inputTrigSelectMask;            /*!< Select input triggers: even/odd bits correspond to rising/falling edge. May be set using CTU_INPUT_TRIG_BITMASK_ macros. - double-buffered (TGSISR) */
   uint16_t tgsCounterCompareVal;           /*!< TGS Counter Compare Value - double-buffered (TGSCCR) */
   uint16_t tgsCounterReloadVal;            /*!< TGS Counter Reload Value - double-buffered (TGSCRR) */
   ctu_adc_cmd_list_mode_t adcCmdListMode;  /*!< Select mode of execution for ADC Command List (LISTCSR.PAR_LIST) */
   ctu_input_trig_t seqModeMrsInput;        /*!< Select which input to be used for MRS when Sequential Mode is enabled (see tgsMode) - double-buffered (TGSCR.MRS_SM) */
   ctu_input_edge_t seqModeMrsInputEdge;    /*!< Select which edge of the input signal (only rising or falling is allowed) to be used for MRS when SequenceMode is enabled - double-buffered (TGSCR.MRS_SM) */
   bool intEnMRS;                           /*!< MRS interrupt enable (IR.MRS_IE) */
   bool intEnError;                         /*!< Interrupt error enabled (IR.IEE) */
   bool dmaDoneGRE;                         /*!< Enable setting of General Reload Enable (GRE) when DMA-done occurs. (IR.DMA_DE) */
   bool dmaReqMRS;                          /*!< Enable issuing of DMA request when MRS occurs, if GRE is enabled. (IR.MRS_DMAE) */
   bool disableOutput;                      /*!< True/false - disable/enable the CTU outputs: pulses to other peripherals/pins, ADC commands or command streams. */
} ctu_config_t;


/*!
 * @brief Structure for configuring an internal trigger (TGS output trigger, input for SU).
 *
 * This structure is used for configuring the properties of an internal trigger (TGS output trigger, input for SU).
 *
 * Implements : ctu_trig_config_t_Class
 */
typedef struct {
    uint16_t compareVal;                /*!< Compare Value - double-buffered (TnCR) */
    uint8_t cmdListStartAdr;            /*!< First address of the command list associated with the internal trigger - (CLCRx) */
    uint8_t outputTrigEnMask;           /*!< Select the CTU output triggers enabled for each SU input trigger. To be used with CTU_OUTPUT_TRIG_ defines. - double-buffered (THCR) */
    bool intEn;                         /*!< Enable interrupt to be triggered when Compare Value is reached and TGS output trigger is set. (IR.Tn_IE) */
} ctu_trig_config_t;


/*!
 * @brief Structure for configuring a result FIFO.
 *
 * This structure is used for configuring a result FIFO.
 *
 * Implements : ctu_res_fifo_config_t_Class
 */
typedef struct {
    uint8_t fifoIntEnMask;              /*!< Mask for FIFO interrupt events. May be set using CTU_FIFO_ macros */
    uint8_t fifoThreshold;              /*!< The number of available results in the selected FIFO, which trigger the FIFO overflow condition - (FTH) */
    bool dmaEn;                         /*!< Enable DMA requests to be issued when the fifoThreshold is reached - (FDCR.DEn) */
} ctu_res_fifo_config_t;


/*!
 * @brief Structure for retrieving full result information for a conversion.
 *
 * This structure is used for retrieving full result information for a conversion.
 *
 * Implements : ctu_conv_result_t_Class
 */
typedef struct {
    ctu_adc_port_t adcPort;             /*!< ADC instance on which the conversion has executed */
    uint8_t adcChanNum;                 /*!< ADC channel number on which the conversion has executed */
    uint16_t convData;                  /*!< ADC conversion data result */
} ctu_conv_result_t;


/*!
 * @brief Structure for configuring a conversion command.
 *
 * This structure is used for configuring a conversion command.
 * The structure fields are a reunion of all fields available for the 3 command formats: A, B and C.
 * The actual format of the command is determined according to values for dedicated fields:
 * \n selfTestEn == false and convMode == CTU_CONV_MODE_SINGLE - selects command format A
 * \n selfTestEn == false and convMode == CTU_CONV_MODE_DUAL - selects command format B
 * \n selfTestEn == true - selects command format C
 *
 * Implements : ctu_adc_cmd_t_Class
 */
typedef struct {
    bool intEn;                         /*!< Enable interrupt request for command execution */
    uint8_t fifoIdx;                    /*!< Select result FIFO to store the conversion result */
    ctu_conv_mode_t convMode;           /*!< Select conversion mode - if selfTestEn==false, selects between format A and B */
    ctu_adc_port_t adcPort;             /*!< Select ADC instance to run the conversion - used in format A and C */
    uint8_t adcChanA;                   /*!< Select target channel number for ADC instance 0 (ADC port A) */
    uint8_t adcChanB;                   /*!< Select target channel number for ADC instance 1 (ADC port B) - used only in format B */
    bool selfTestEn;                    /*!< Enable Self Test command - CTU issues a self test command to ADC - selects format C */
    ctu_adc_st_alg_t selfTestAlg;       /*!< Select algorithm for self test command - used only in format C */
    uint8_t selfTestBurstSize;          /*!< Select burst size for self test command - used only in format C */
} ctu_adc_cmd_t;


/*!
 * @brief Structure for retrieving ADC command list status information.
 *
 * This structure is used for retrieving ADC command list status information.
 *
 * Implements : ctu_list_status_t_Class
 */
typedef struct {
    uint8_t listAddr;					/*!< Command address being executed when EFR.LIST_BE flag was set */
    bool listBlocked;					/*!< When set indicates that the address in listAddr had failed to issue an ADC cmd */
} ctu_list_status_t;

/*!
 * @brief Macros for selecting input triggers signal edges - mapped to TRGSISR register format.
 * Implements : CTU_INPUT_TRIG_BITMASK_Class
 */
#define CTU_INPUT_TRIG_BITMASK_RISING(INPUT_IDX)    ((uint32_t)1u << (2u * ((uint32_t)(INPUT_IDX))))          /*!< Select input triggers: even bits correspond to rising edge of the input idx */
#define CTU_INPUT_TRIG_BITMASK_FALLING(INPUT_IDX)   ((uint32_t)1u << ((2u * ((uint32_t)(INPUT_IDX))) + 1u))   /*!< Select input triggers: odd bits correspond to falling edge of the input idx */
#define CTU_INPUT_TRIG_BITMASK_BOTH(INPUT_IDX)      ((uint32_t)3u << (2u * ((uint32_t)(INPUT_IDX))))          /*!< Select input triggers: both rising and falling edges of the input idx  */
#define CTU_INPUT_TRIG_BITMASK_ALL                  (0xFFFFFFFFu)                                             /*!< Select input triggers: both rising and falling edges for all inputs  */


/*!
 * @brief Macros for selecting CTU outputs to be enabled - mapped to THCR register.
 * Implements : CTU_OUTPUT_TRIG_Class
 */
#define CTU_OUTPUT_TRIG_ADC_CMD_EN          (CTU_THCR1_T0_ADCE_MASK)
#define CTU_OUTPUT_TRIG_TIMER_1_EN          (CTU_THCR1_T0_T1E_MASK)
#if (CTU_ETIMER_COUNT >= 2u)
#define CTU_OUTPUT_TRIG_TIMER_2_EN          (CTU_THCR1_T0_T2E_MASK)
#else
#define CTU_OUTPUT_TRIG_TIMER_2_EN          (0u)
#endif /* (CTU_ETIMER_COUNT >= 2u) */
#if (CTU_ETIMER_COUNT >= 3u)
#define CTU_OUTPUT_TRIG_TIMER_3_EN          (CTU_THCR1_T0_T3E_MASK)
#else
#define CTU_OUTPUT_TRIG_TIMER_3_EN          (0u)
#endif /* (CTU_ETIMER_COUNT >= 3u) */
#if (CTU_ETIMER_COUNT >= 4u)
#define CTU_OUTPUT_TRIG_TIMER_4_EN          (CTU_THCR1_T0_T4E_MASK)
#else
#define CTU_OUTPUT_TRIG_TIMER_4_EN          (0u)
#endif /* (CTU_ETIMER_COUNT >= 4u) */
#define CTU_OUTPUT_TRIG_EXT_EN              (CTU_THCR1_T0_ETE_MASK)
#define CTU_OUTPUT_TRIG_EN                  (CTU_THCR1_T0_E_MASK)    /*! Enable the output for selected CTU trigger. If not set, all outputs are disabled. */

#define CTU_OUTPUT_TRIG_ALL_EN              (CTU_OUTPUT_TRIG_ADC_CMD_EN | CTU_OUTPUT_TRIG_TIMER_1_EN | CTU_OUTPUT_TRIG_TIMER_2_EN | \
                                             CTU_OUTPUT_TRIG_TIMER_3_EN | CTU_OUTPUT_TRIG_TIMER_4_EN | CTU_OUTPUT_TRIG_EXT_EN     | \
                                             CTU_OUTPUT_TRIG_EN)

/*!
 * @brief Macros for selecting CTU result FIFO status and interrupt flags - mapped to FST and FCR registers.
 * Implements : CTU_FIFO_Class
 */
#define CTU_FIFO_FULL                       (CTU_FST_FULL0_MASK)
#define CTU_FIFO_EMPTY                      (CTU_FST_EMP0_MASK)
#define CTU_FIFO_OVERFLOW                   (CTU_FST_OF0_MASK)
#define CTU_FIFO_OVERRUN                    (CTU_FST_OR0_MASK)
#define CTU_FIFO_ALL                        (CTU_FIFO_FULL | CTU_FIFO_EMPTY | CTU_FIFO_OVERFLOW | CTU_FIFO_OVERRUN)

/*!
 * @brief Macros for selecting CTU error flags - mapped to EFR register.
 * Implements : CTU_ERROR_FLAG_Class
 */
#define CTU_ERROR_FLAG_MRS_RE               (CTU_EFR_MRS_RE_MASK)
#define CTU_ERROR_FLAG_SM_TRIG_OVERRUN      (CTU_EFR_SM_TO_MASK)
#define CTU_ERROR_FLAG_INVALID_CMD          (CTU_EFR_ICE_MASK)
#define CTU_ERROR_FLAG_MRS_OVERRUN          (CTU_EFR_MRS_O_MASK)
#define CTU_ERROR_FLAG_TGS_SM_OVERRUN       (CTU_EFR_TGS_OSM_MASK)
#define CTU_ERROR_FLAG_ADC_OVERRUN          (CTU_EFR_ADC_OE_MASK)
#define CTU_ERROR_FLAG_TIMER_1_OVERRUN      (CTU_EFR_T1_OE_MASK)
#if (CTU_ETIMER_COUNT >= 2u)
#define CTU_ERROR_FLAG_TIMER_2_OVERRUN      (CTU_EFR_T2_OE_MASK)
#else
#define CTU_ERROR_FLAG_TIMER_2_OVERRUN      (0u)
#endif /* (CTU_ETIMER_COUNT >= 2u) */
#if (CTU_ETIMER_COUNT >= 3u)
#define CTU_ERROR_FLAG_TIMER_3_OVERRUN      (CTU_EFR_T3_OE_MASK)
#else
#define CTU_ERROR_FLAG_TIMER_3_OVERRUN      (0u)
#endif /* (CTU_ETIMER_COUNT >= 3u) */
#if (CTU_ETIMER_COUNT >= 4u)
#define CTU_ERROR_FLAG_TIMER_4_OVERRUN      (CTU_EFR_T4_OE_MASK)
#else
#define CTU_ERROR_FLAG_TIMER_4_OVERRUN      (0u)
#endif /* (CTU_ETIMER_COUNT >= 4u) */
#define CTU_ERROR_FLAG_ERRCMP               (CTU_EFR_ERRCMP_MASK)
#define CTU_ERROR_FLAG_EXT_TRIG_OVERRUN     (CTU_EFR_ET_OE_MASK)
#define CTU_ERROR_FLAG_SELF_TEST_RUNNING    (CTU_EFR_CS_MASK)
#define CTU_ERROR_FLAG_LIST_BUSY            (CTU_EFR_LIST_BE_MASK)

#define CTU_ERROR_FLAG_ALL                  (CTU_ERROR_FLAG_MRS_RE            | CTU_ERROR_FLAG_SM_TRIG_OVERRUN  | CTU_ERROR_FLAG_INVALID_CMD      | \
                                             CTU_ERROR_FLAG_MRS_OVERRUN       | CTU_ERROR_FLAG_TGS_SM_OVERRUN   | CTU_ERROR_FLAG_ADC_OVERRUN      | \
                                             CTU_ERROR_FLAG_TIMER_1_OVERRUN   | CTU_ERROR_FLAG_TIMER_2_OVERRUN  | CTU_ERROR_FLAG_TIMER_3_OVERRUN  | \
                                             CTU_ERROR_FLAG_TIMER_4_OVERRUN   | CTU_ERROR_FLAG_ERRCMP           | CTU_ERROR_FLAG_EXT_TRIG_OVERRUN | \
                                             CTU_ERROR_FLAG_SELF_TEST_RUNNING | CTU_ERROR_FLAG_LIST_BUSY)

/*!
* @brief Macros for selecting CTU interrupt status flags - mapped to IFR register.
* Implements : CTU_INTERRUPT_FLAG_Class
*/
#define CTU_INTERRUPT_FLAG_MRS              (CTU_IFR_MRS_I_MASK)    /*!< Set when Master Reload Signal is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_0           (CTU_IFR_T0_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_1           (CTU_IFR_T1_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_2           (CTU_IFR_T2_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_3           (CTU_IFR_T3_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_4           (CTU_IFR_T4_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_5           (CTU_IFR_T5_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_6           (CTU_IFR_T6_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_TRIG_7           (CTU_IFR_T7_I_MASK)     /*!< Set when the corresponding internal trigger is issued. */
#define CTU_INTERRUPT_FLAG_ADC_CMD          (CTU_IFR_ADC_I_MASK)    /*!< Set when a new ADC command is issued. */
#define CTU_INTERRUPT_FLAG_SERR_A           (CTU_IFR_SERR_A_MASK)   /*!< Duration of ADC 0 conversion is greater than the maximum configured. */
#define CTU_INTERRUPT_FLAG_SERR_B           (CTU_IFR_SERR_B_MASK)   /*!< Duration of ADC 1 conversion is greater than the maximum configured. */

#define CTU_INTERRUPT_FLAG_ALL              (CTU_INTERRUPT_FLAG_MRS     | CTU_INTERRUPT_FLAG_TRIG_0 | CTU_INTERRUPT_FLAG_TRIG_1 | \
                                             CTU_INTERRUPT_FLAG_TRIG_2  | CTU_INTERRUPT_FLAG_TRIG_3 | CTU_INTERRUPT_FLAG_TRIG_4 | \
                                             CTU_INTERRUPT_FLAG_TRIG_5  | CTU_INTERRUPT_FLAG_TRIG_6 | CTU_INTERRUPT_FLAG_TRIG_7 | \
                                             CTU_INTERRUPT_FLAG_ADC_CMD | CTU_INTERRUPT_FLAG_SERR_A | CTU_INTERRUPT_FLAG_SERR_B)


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

 /*!
  * @brief Configure the global functionalities of a CTU instance.
  *
  * This function configures the global functionalities of a CTU instance.
  *
  * @param[in] instance - the CTU instance number
  * @param[in] config - pointer to the input configuration structure
  * @return void
  */
void CTU_DRV_Config(const uint32_t instance, const ctu_config_t * const config);

/*!
 * @brief Get the default configuration for CTU.
 *
 * This function populates the configuration structure with the
 * default configuration for CTU.
 *
 * @param[out] config - pointer to the output configuration structure
 * @return void
 */
void CTU_DRV_GetDefaultConfig(ctu_config_t * const config);

/*!
 * @brief Reset all registers of a CTU instance to default values.
 *
 * This function resets all registers of a CTU instance to default values.
 * It stops and resets any running ADC command list.
 * Note: the function also enables the CTU output (default state is enabled).
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_Reset(const uint32_t instance);

/*!
 * @brief Enable the selected signal edge(s) for a CTU input trigger.
 *
 * This function enables the selected signal edge(s) for a CTU input trigger.
 * The affected register is double-buffered, so the new value will take effect only
 * after MRS occurs or CTU_DRV_SyncInputSelect() is called.
 * Please note that the set operation is cumulative, e.g. set falling edge, then set rising edge 
 * for same input trigger, results in both edges being enabled for that trigger. 
 * The same applies also when setting both edges, followed by set falling/rising.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] inputTrig - the selected input trigger
 * @param[in] inputEdge - the selected signal edge(s) of the input trigger
 * @return void
 */
void CTU_DRV_SetInputSelect(const uint32_t instance, const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge);

/*!
 * @brief Disable the selected signal edge(s) for a CTU input trigger.
 *
 * This function disables the selected signal edge(s) for a CTU input trigger.
 * The affected register is double-buffered, so the new value will take effect only
 * after MRS occurs or CTU_DRV_SyncInputSelect() is called.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] inputTrig - the selected input trigger
 * @param[in] inputEdge - the selected signal edge(s) of the input trigger
 * @return void
 */
void CTU_DRV_ClearInputSelect(const uint32_t instance, const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge);

/*!
 * @brief Load the new value of input trigger select - TGSISR (double buffered register).
 *
 * This function loads the new value of input trigger select - TGSISR (double buffered register).
 * The function waits until the TGSISR register is updated from the corresponding buffer register.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_SyncInputSelect(const uint32_t instance);


/*!
 * @brief Configure the properties of a single internal trigger.
 *
 * This function configures the properties of a single internal trigger.
 * By internal trigger, the CTU driver refers to the output triggers from TGS (Trigger Generator Subunit),
 * fed as input triggers to the SU (Scheduler Subunit). \n
 *
 * Note regarding members of ctu_trig_config_t: \n
 *  <compareVal> and <outputTrigEnMask>, correspond to double buffered registers,
 *  so configured values are loaded and take effect only after MRS occurs. \n
 *  <intEn> and <cmdListStartAdr> are not double-buffered, so the configured values
 *  take effect immediately after CTU_DRV_ConfigInternalTrigger() is called.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] internalTrigIdx - index of the internal trigger
 * @param[in] internalTrigConfig - pointer to the configuration structure for the internal trigger
 * @return void
 */
void CTU_DRV_ConfigInternalTrigger(const uint32_t instance, const uint32_t internalTrigIdx, const ctu_trig_config_t * const internalTrigConfig);

/*!
 * @brief Configure an array of ADC commands and marks accordingly the last command.
 *
 * This function configures an array of ADC commands and marks accordingly the last command. \n
 *
 * Note: the ADC command registers are double buffered,
 *  so configured values are loaded and take effect only after MRS occurs. \n
 *  The function doesn't take into account the reload mechanism (refer to device reference manual for details),
 *  so it needs to be handled externally - may use CTU_DRV_EnableGeneralReload().
 *
 * @param[in] instance - the CTU instance number
 * @param[in] startCmdListAdr - index of the starting element in the conversion list
 * @param[in] adcCmdArray - pointer to the array of the ADC command list
 * @param[in] numCmds - number of commands in the array of ADC command list
 * @return void
 */
void CTU_DRV_ConfigAdcCmdList(const uint32_t instance, const uint32_t startCmdListAdr, const ctu_adc_cmd_t * const adcCmdArray, uint8_t numCmds);

/*!
 * @brief Configure the properties of a selected result FIFO.
 *
 * This function configures the properties of a selected result FIFO.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] fifoConfig - pointer to the configuration structure for the result FIFO
 * @return void
 */
void CTU_DRV_ConfigResultFifo(const uint32_t instance, const uint32_t fifoIdx, const ctu_res_fifo_config_t * const fifoConfig);

/*!
 * @brief Return the conversion data result aligned according to the alignment parameter.
 *
 * This function returns the conversion data result aligned according
 * to the alignment parameter.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] alignment - left or right alignment of the result data
 * @return uint16_t - conversion data result
 */
uint16_t CTU_DRV_GetConvData(const uint32_t instance, const uint32_t fifoIdx, ctu_data_align_t alignment);

/*!
 * @brief Get the full conversion result information, with
 * the conversion data aligned according to the alignment parameter.
 *
 * This function gets the full conversion result information, with
 * the conversion data aligned according to the alignment parameter.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] alignment - left or right alignment of the result data
 * @param[out] result - pointer to the output structure containing full conversion result information
 * @return void
 */
void CTU_DRV_GetConvResult(const uint32_t instance, const uint32_t fifoIdx, const ctu_data_align_t alignment, ctu_conv_result_t * const result);

/*!
 * @brief Clear the status flags enabled in the bit-mask, for the selected result FIFO.
 *
 * This function clears the status flags enabled in the bit-mask, for the selected result FIFO.
 * The mask parameter can be set using the defines provided in the driver header file:
 * CTU_FIFO_ - e.g. CTU_FIFO_OVERRUN.  \n
 *
 * Note: CTU permits only for the FST.OR flags (CTU_FIFO_OVERRUN) to be cleared.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] statusFlagsMask - bit-mask selecting which FIFO status flags to be cleared - may be set using CTU_FIFO_ defines
 * @return void
 */
void CTU_DRV_ClearFifoStatusFlags(const uint32_t instance, const uint32_t fifoIdx, const uint32_t statusFlagsMask);

/*!
 * @brief Return bit-mask with all the status flags for a selected result FIFO.
 *
 * Returns bit-mask with all the status flags for a selected result FIFO.
 * Values for individual flags may be retrieved by masking the result with defines provided
 * in the driver header file: CTU_FIFO_ - e.g. CTU_FIFO_OVERFLOW
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @return uint32_t - bit-mask containing status flags for the selected FIFO
 */
uint32_t CTU_DRV_GetFifoStatusFlags(const uint32_t instance, const uint32_t fifoIdx);

/*!
 * @brief Enable the interrupts for the selected result FIFO, corresponding
 * to the bits set in mask parameter.
 *
 * This function enables the interrupts for the selected result FIFO, corresponding
 * to the bits set in mask parameter.
 * The mask parameter can be set using the defines provided in the driver header file:
 * CTU_FIFO_ - e.g. CTU_FIFO_FULL.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] intFlagsMask - bit-mask selecting which FIFO interrupts to be enabled - may be set using CTU_FIFO_ defines
 * @return void
 */
void CTU_DRV_EnableFifoInterrupts(const uint32_t instance, const uint32_t fifoIdx, const uint32_t intFlagsMask);

/*!
 * @brief Disable the interrupts for the selected result FIFO, corresponding
 * to the bits set in mask parameter.
 *
 * This function disables the interrupts for the selected result FIFO, corresponding
 * to the bits set in mask parameter.
 * The mask parameter can be set using the defines provided in the driver header file:
 * CTU_FIFO_ - e.g. CTU_FIFO_FULL.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] fifoIdx - index of the result FIFO
 * @param[in] intFlagsMask - bit-mask selecting which FIFO interrupts to be enabled - may be set using CTU_FIFO_ defines
 * @return void
 */
void CTU_DRV_DisableFifoInterrupts(const uint32_t instance, const uint32_t fifoIdx, const uint32_t intFlagsMask);

/*!
 * @brief Clear the error flags corresponding to the bits set in mask parameter.
 *
 * This function clears the error flags corresponding to the bits set in mask parameter.
 * The mask parameter can be set using the defines provided in the driver header file:
 * CTU_ERROR_FLAG_ - e.g. CTU_ERROR_FLAG_INVALID_CMD.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] flagsMask - bit-mask selecting which FIFO interrupts to be enabled - may be set using CTU_ERROR_FLAG_ defines
 * @return void
 */
void CTU_DRV_ClearErrorFlags(const uint32_t instance, const uint32_t flagsMask);

/*!
 * @brief Return bit-mask with values for all the CTU error flags.
 *
 * This function returns bit-mask with values for all the CTU error flags.
 * Values for individual flags may be retrieved by masking the result with defines provided
 * in the driver header file: CTU_ERROR_FLAG_ - e.g. CTU_ERROR_FLAG_MRS_RE.
 *
 * @param[in] instance - the CTU instance number
 * @return uint32_t - bit-mask with values for all the CTU error flags.
 */
uint32_t CTU_DRV_GetErrorFlags(const uint32_t instance);

/*!
 * @brief Clear the interrupt flags, corresponding to the bits set in mask parameter.
 *
 * This function clears the interrupt flags, corresponding to the bits set in mask parameter.
 * The mask parameter can be set using the defines provided in the driver header file:
 * CTU_INTERRUPT_FLAG_ - e.g. CTU_INTERRUPT_FLAG_MRS.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] flagsMask - bit-mask selecting which FIFO interrupts to be enabled - may be set using CTU_INTERRUPT_FLAG_ defines
 * @return void
 */
void CTU_DRV_ClearInterruptFlags(const uint32_t instance, const uint32_t flagsMask);

/*!
 * @brief Return bit-mask with values for all the CTU interrupt status flags.
 *
 * This function returns bit-mask with values for all the CTU interrupt status flags.
 * Values for individual flags may be retrieved by masking the result with defines provided
 * in the driver header file: CTU_INTERRUPT_FLAG_ - e.g. CTU_INTERRUPT_FLAG_MRS.
 *
 * @param[in] instance - the CTU instance number
 * @return uint32_t - bit-mask with values for all the CTU interrupt status flags.
 */
uint32_t CTU_DRV_GetInterruptFlags(const uint32_t instance);

/*!
 * @brief Generates from software an event on the selected internal trigger.
 *
 * Generates from software an event on the selected internal trigger,
 * provided as input to the Scheduler subunit (SU).
 * This bypasses any delays configured in Trigger Generator Subunit (TGS)
 * and can be used when the TGS Counter is running or stopped.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] internalTrigIdx - index of the internal trigger
 * @return void
 */
void CTU_DRV_SwInternalTrigger(const uint32_t instance, const uint32_t internalTrigIdx);

/*!
 * @brief Software trigger the Master Reload Signal.
 *
 * Software trigger the Master Reload Signal.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_SwMasterReloadSignal(const uint32_t instance);

/*!
 * @brief Enable CTU output.
 *
 * This function enables CTU output.
 * The output can be a pulse, an ADC command or a stream of ADC commands.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_EnableOutput(const uint32_t instance);

/*!
 * @brief Disable CTU output.
 *
 * This function disables CTU output.
 * The output can be a pulse, an ADC command or a stream of ADC commands.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_DisableOutput(const uint32_t instance);

/*!
 * @brief Set the on-time + guard-time duration for external triggers.
 *
 * This function sets the on-time + guard-time duration for external triggers.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] duration - value of the on-time + guard-time duration in CTU clock ticks
 * @return void
 */
void CTU_DRV_SetExternalTrigsDuration(const uint32_t instance, const uint8_t duration);

/*!
 * @brief Set the digital filter value.
 *
 * This function sets the digital filter value. If the set value is >0, then the function
 * implicitly enables the digital filter. Otherwise the function implicitly disables it.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] filterValue - number of times for which the external signal (EXT_IN)
 *  needs to be latched at 1/0,  so that it is considered 1/0
 * @return void
 */
void CTU_DRV_SetDigitalFilterValue(const uint32_t instance, const uint8_t filterValue);

/*!
 * @brief Set the maximum expected duration for an ADC conversion on the selected ADC instance.
 *
 * Sets the maximum expected duration for an ADC conversion on the selected ADC instance.
 * Checking of the ADC conversion duration needs to be enabled via dedicated function.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] adcIdx - target ADC instance
 * @param[in] duration - timing duration in which a conversion from the target ADC should finish execution
 * @return void
 */
void CTU_DRV_SetExpectedConvDuration(const uint32_t instance, const ctu_adc_port_t adcIdx, const uint16_t duration);

/*!
 * @brief Set the value of counter range register.
 *
 * This function sets the value of counter range register. The value is used to mask out the
 * related bits for expected conversion duration. For the exact formula, please refer to reference manual.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] range - timing range in which a conversion should occur
 * @return void
 */
void CTU_DRV_SetExpectedConvRange(const uint32_t instance, const uint16_t range);

/*!
 * @brief Enable checking of the ADC conversion duration, for the selected
 * ADC instance.
 *
 * Enables checking of the ADC conversion duration, for the selected
 * ADC instance. If a conversion takes a number of cycles out of the interval
 * defined by CTU_DRV_SetExpectedConvRange() and CTU_DRV_SetExpectedConvDuration()
 * an error flag is raised SERR_A/SERR_B (please refer to device reference manual
 * for details).
 *
 * @param[in] instance - the CTU instance number
 * @param[in] adcIdx - index of the target ADC instance
 * @return void
 */
void CTU_DRV_EnableCheckDuration(const uint32_t instance, const ctu_adc_port_t adcIdx);

/*!
 * @brief Disable checking of the ADC conversion duration, for the selected
* ADC instance.
 *
 * This function disables checking of the ADC conversion duration, for the selected
* ADC instance.
 *
 * @param[in] instance - the CTU instance number
 * @param[in] adcIdx - index of the target ADC instance
 * @return void
 */
void CTU_DRV_DisableCheckDuration(const uint32_t instance, const ctu_adc_port_t adcIdx);

/*!
 * @brief Enable General Reload flag (CR.GRE)
 *
 * This function enables General Reload flag (CR.GRE).
 * It needs to be called before MRS, after the last write to a double-buffered register.
 * It marks that the user has completed writing all the double-buffered registers for
 * the current control cycle, so an MRS event shall load all the new values in the double-buffered registers.
 * If CR.GRE is not set and at least a double-buffered register has been written during the current control cycle
 * (i.e. after the last MRS), the next MRS will not load the new values and an error flag will be raised (EFR.MRS_RE).
 * \n For more details please see reload mechanism described in device reference manual.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_EnableGeneralReload(const uint32_t instance);

/*!
 * @brief Disable General Reload (CR.GRE) - for details please see reload
 * mechanism described in device reference manual.
 *
 * This function disables General Reload (CR.GRE) - for details please see reload
 * mechanism described in device reference manual.
 *
 * NOTE: the functions waits until write to CGRE takes effect, to mitigate propagation delay in CTU module
 * and guarantee that any code called after CTU_DRV_DisableGeneralReload is executed after CGRE write takes effect (GRE bit is cleared).
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_DisableGeneralReload(const uint32_t instance);


/*!
 * @brief Get the list status information.
 *
 * This function gets the list status information.
 * Information for multiple lists is available when executing in parallel mode.
 *
 * @param[in] instance - the instance number
 * @param[in] listIdx - index of the list
 * @param[out] listStatus - pointer to list status information
 * @return status:
 * \n - STATUS_SUCCESS: returned information is valid
 * \n - STATUS_ERROR: list status information is not meaningful, because EFR.LIST_BE is not set.
 */
status_t CTU_DRV_GetListStatus(const uint32_t instance, const uint32_t listIdx, ctu_list_status_t * const listStatus);


/*!
 * @brief Reset CTU ADC command list control state machine.
 *
 * This function resets CTU ADC command list control state machine.
 * After a reset, the command list stops execution and waits for a new trigger event
 * in order to initiate a command list execution.
 *
 * @param[in] instance - the CTU instance number
 * @return void
 */
void CTU_DRV_ResetAdcCmdListState(const uint32_t instance);

/*! @}*/

#if defined (__cplusplus)
}
#endif

/*! @}*/


#endif /* CTU_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/


