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

#ifndef ADC_SAR_DRIVER_H
#define ADC_SAR_DRIVER_H

/* */
/* */
/* */

/*! @file */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro defines a bitmask used to access status flags.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * The macros are defined to simplify bitmask usage
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"
#include "status.h"

/*!
 * @addtogroup adc_sar_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Macros for status flags
 *
 * These flags map to internal hardware flags in different registers, but are
 * grouped together for convenience.
 *
 */
#define ADC_FLAG_CALIBRATED        ((uint32_t)1U << 0U)
#define ADC_FLAG_NORMAL_STARTED    ((uint32_t)1U << 1U)
#define ADC_FLAG_NORMAL_EOC        ((uint32_t)1U << 2U)
#define ADC_FLAG_NORMAL_ENDCHAIN   ((uint32_t)1U << 3U)
#define ADC_FLAG_INJECTED_STARTED  ((uint32_t)1U << 4U)
#define ADC_FLAG_INJECTED_EOC      ((uint32_t)1U << 5U)
#define ADC_FLAG_INJECTED_ENDCHAIN ((uint32_t)1U << 6U)
#define ADC_FLAG_INJECTED_ABORTED  ((uint32_t)1U << 7U)
#define ADC_FLAG_CTU_STARTED       ((uint32_t)1U << 8U)
#define ADC_FLAG_CTU_EOC           ((uint32_t)1U << 9U)
#define ADC_FLAG_AUTOCLOCKOFF      ((uint32_t)1U << 10U)

#define ADC_FLAG_ALL               (ADC_FLAG_CALIBRATED | \
                                    ADC_FLAG_NORMAL_STARTED | \
                                    ADC_FLAG_NORMAL_EOC | \
                                    ADC_FLAG_NORMAL_ENDCHAIN | \
                                    ADC_FLAG_INJECTED_STARTED | \
                                    ADC_FLAG_INJECTED_EOC | \
                                    ADC_FLAG_INJECTED_ENDCHAIN | \
                                    ADC_FLAG_INJECTED_ABORTED | \
                                    ADC_FLAG_CTU_STARTED | \
                                    ADC_FLAG_CTU_EOC | \
                                    ADC_FLAG_AUTOCLOCKOFF \
                                    )
#define ADC_NUM_OF_GROUP_CHN        3U

/*!
 * @brief Macros for watchdog status registers
 *
 * These macros help decode bit mask provided by the ADC_DRV_GetWdgThresholdFlags
 * function and also compose the mask to be provided to ADC_DRV_ClearWdgThresholdFlags
 *
 */
#define ADC_WDOG_REG_MASK_HIGH(registerIdx) ((uint32_t)((uint32_t)1u << ((registerIdx*2u) + 1u)))
#define ADC_WDOG_REG_MASK_LOW(registerIdx)  ((uint32_t)((uint32_t)1u << (registerIdx*2u)))

/*!
 * @brief Conversion mode selection (One-shot or Scan)
 *
 * This structure is used to configure the conversion mode
 *
 * Implements : adc_conv_mode_t_Class
 */
typedef enum
{
    ADC_CONV_MODE_ONESHOT = 0x00U,  /*!< One-shot conversion mode */
    ADC_CONV_MODE_SCAN = 0x01U      /*!< Scan conversion mode */
} adc_conv_mode_t;

/*!
 * @brief Converter input clock
 *
 * This structure is used to configure the converter input clock
 *
 * Implements : adc_clk_sel_t_Class
 */
typedef enum
{
    ADC_CLK_HALF_BUS = 0x00U,       /*!< Input clock is Bus clock/2 */
    ADC_CLK_FULL_BUS = 0x01U,       /*!< Input clock is Bus clock   */
#if FEATURE_ADC_HAS_CLKSEL_EXTENDED 
    ADC_CLK_QUARTER_BUS = 0x02U,    /*!< Input clock is Bus clock/4 */
#endif
} adc_clk_sel_t;

/*!
 * @brief Reference selection.
 *
 * This structure is used to configure the reference of the ADC.
 *
 * Implements : adc_ref_sel_t_Class
 */
typedef enum
{
    ADC_REF_VREFH = 0x00U           /*!< Reference is VREFH */
} adc_ref_sel_t;

#if FEATURE_ADC_HAS_CTU
/*!
 * @brief CTU Mode selection
 *
 * This structure is used to configure the mode in which CTU is used
 *
 * Implements : adc_ctu_mode_t_Class
 */
typedef enum
{
    ADC_CTU_MODE_DISABLED = 0x00U,  /*!< CTU Mode disabled */
    ADC_CTU_MODE_CONTROL = 0x02,    /*!< CTU is in Control Mode */
    ADC_CTU_MODE_TRIGGER = 0x03U    /*!< CTU is in Trigger Mode */
} adc_ctu_mode_t;
#endif

#if FEATURE_ADC_HAS_INJ_TRIGGER_SEL
/*!
 * @brief Injected Trigger selection
 *
 * This structure is used to configure the type of injected trigger
 *
 * Implements : adc_injected_edge_t_Class
 */
typedef enum
{
    ADC_INJECTED_EDGE_DISABLED = 0x00U, /*!< Injected trigger disabled */
    ADC_INJECTED_EDGE_FALLING = 0x02U,  /*!< Injected trigger on Falling Edge */
    ADC_INJECTED_EDGE_RISING = 0x03U,   /*!< Injected trigger on Rising Edge */
} adc_injected_edge_t;
#endif /* FEATURE_ADC_HAS_INJ_TRIGGER_SEL */

/*!
 * @brief External Trigger selection 
 *
 * This structure is used to configure the external trigger
 *
 * Implements : adc_ext_trigger_t_Class
 */
#if FEATURE_ADC_HAS_EXT_TRIGGER
typedef enum
{
    ADC_EXT_TRIGGER_DISABLED = 0x00U, /*!< External trigger disabled */
    ADC_EXT_TRIGGER_FALLING = 0x02U,  /*!< External trigger on Falling Edge */
    ADC_EXT_TRIGGER_RISING = 0x03U,   /*!< External trigger on Rising Edge */
} adc_ext_trigger_t;
#endif

/*!
 * @brief Conversion chain selection
 *
 * This structure is used to configure type of the conversion
 *
 * Implements : adc_conv_chain_t_Class
 */
typedef enum
{
    ADC_CONV_CHAIN_NORMAL = 0x00U,      /*!< Selects the "Normal" Conversion Chain */
    ADC_CONV_CHAIN_INJECTED = 0x01U,    /*!< Selects the "Injected" Conversion Chain */
    ADC_CONV_CHAIN_CTU = 0x02U          /*!< Selects the "CTU" Conversion Chain */
} adc_conv_chain_t;

/*!
 * @brief Data alignment selection
 *
 * This structure is used to configure data alignment
 *
 * Implements : adc_data_aligned_t_Class
 */
typedef enum
{
    ADC_DATA_ALIGNED_RIGHT = 0x00U,     /*!< Measured data is right-aligned */
    ADC_DATA_ALIGNED_LEFT = 0x01U       /*!< Measured data is left-aligned */
} adc_data_aligned_t;

/*!
 * @brief Clear DMA source
 *
 * This structure is used to configure source used to clear a DMA request
 *
 * Implements : adc_dma_clear_source_t_Class
 */
typedef enum {
    ADC_DMA_REQ_CLEAR_ON_ACK = 0U,      /*!< Clear DMA Request on Ack from DMA Controller */
    ADC_DMA_REQ_CLEAR_ON_READ = 1U,     /*!< Clear DMA Request on read of Data Registers */
} adc_dma_clear_source_t;

/*!
 * @brief Presampling Voltage selection
 *
 * This structure is used to configure the presampling voltage
 *
 * Implements : adc_presampling_source_t_Class
 */
typedef enum {
    ADC_PRESAMPLE_SRC0 = 0x00U,         /*!< Presampling from */
    ADC_PRESAMPLE_SRC1 = 0x01U,         /*!< Presampling from */
    ADC_PRESAMPLE_SRC2 = 0x02U,         /*!< Presampling from */
    ADC_PRESAMPLE_SRC3 = 0x03U          /*!< Presampling from */
} adc_presampling_source_t;

/*!
 * @brief Channel group selection
 *
 * This structure is used to select the group of ADC channels
 *
 * Implements : adc_chan_group_t_Class
 */
typedef enum {
    ADC_CHAN_GROUP_0 = 0U,           /*!< Channels Group (0-31) */
    ADC_CHAN_GROUP_1 = 1U,           /*!< Channels Group (32-63) */
    ADC_CHAN_GROUP_2 = 2U,           /*!< Channels Group (64-95) */
} adc_chan_group_t;

/*!
 * @brief Defines the converter configuration
 *
 * This structure is used to configure the ADC converter
 *
 * Implements : adc_conv_config_t_Class
 */
typedef struct
{
    adc_conv_mode_t convMode;   /*!< Conversion Mode (One-shot or Scan) */
    adc_clk_sel_t clkSelect;    /*!< Clock input */
    adc_ref_sel_t refSelect;    /*!< Reference selection  */
#if FEATURE_ADC_HAS_CTU
    adc_ctu_mode_t ctuMode;     /*!< CTU mode */
#endif
#if FEATURE_ADC_HAS_INJ_TRIGGER_SEL
    adc_injected_edge_t injectedEdge;   /*!< Injected Trigger selection */
#endif
#if FEATURE_ADC_HAS_EXT_TRIGGER
     adc_ext_trigger_t extTrigger;       /*!< External Trigger selection */
#endif
    uint8_t sampleTime0; /*!< Sample time for channels 0-31 */
    uint8_t sampleTime1;  /*!< Sample time for channels 32-63 */
    uint8_t sampleTime2;  /*!< Sample time for channels 64-95 */
    bool autoClockOff; /*!< Enable Auto Clock Off */
    bool overwriteEnable; /*!< Overwrite new conversion data over old data */
    adc_data_aligned_t dataAlign; /*!< Data alignment in conversion result register */
    uint8_t decodeDelay; /*!< Delay for decoding Input MUX channels */
    uint8_t powerDownDelay; /*!< Delay before entering Power Down */
} adc_conv_config_t;

/*!
 * @brief Defines the chain configuration
 *
 * This structure is used to configure the ADC Normal and Injected Chains
 *
 * Implements : adc_chain_config_t_Class
 */
typedef struct
{
	uint32_t chanMaskNormal[ADC_NUM_OF_GROUP_CHN];     /*!< Bit-mask used to configure Normal Chain */
	uint32_t chanMaskInjected[ADC_NUM_OF_GROUP_CHN];   /*!< Bit-mask used to configure Injected Chain */
    uint32_t interruptMask[ADC_NUM_OF_GROUP_CHN];      /*!< Bit-mask used to configure channels interrupts */
} adc_chain_config_t;


/*!
 * @brief Defines the data regarding a conversion, beyond the conversion data.
 *
 * This structure is used to return information about conversions beyond just conversion data
 *
 * Implements : adc_chan_result_t_Class
 */
typedef struct {
    uint8_t chnIdx;   /*!< ADC Channel Index */
    bool valid;       /*!< Data Valid Flag */
    bool overWritten; /*!< Data Overwritten Flag */
    uint16_t cdata;   /*!< Conversion Data */
} adc_chan_result_t;

/*!
 * @brief Defines the upper and lower thresholds for analog watchdog.
 *
 * This structure is used to configure the analog watchdog threshold registers.
 *
 * Implements : adc_wdg_threshold_values_t_Class
 */
typedef struct
{
	uint16_t lowThreshold;  /*!< Lower threshold */
	uint16_t highThreshold; /*!< Upper threshold */
	bool lowThresholdIntEn; /*!< Enable interrupt when lower threshold exceeded */
	bool highThresholdIntEn; /*!< Enable interrupt when upper threshold exceeded */
} adc_wdg_threshold_values_t;

/*!
 * @brief Set the threshold register used by a certain ADC channel
 *
 * This structure is used to assign a threshold register to a certain ADC channel.
 * The register stores the high and low thresholds for the analog watchdog.
 *
 * Implements : adc_wdg_chan_thresholds_t_Class
 */
typedef struct
{
	uint8_t chnIdx;      /*!< The index of the channel */
	uint8_t registerIdx;  /*!< The index of the register that store the thresholds */
} adc_wdg_chan_thresholds_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the converter configuration structure
 *
 * This function initializes the struct members to default (reset) values.
 *
 * @param[out] config configuration struct pointer
 */
void ADC_DRV_GetDefaultConfigConverter(adc_conv_config_t * const config);

/*!
 * @brief Configures the converter with the given configuration structure
 *
 * This function configures the ADC converter with the options
 * provided in the structure.
 *
 * @param[in] instance the instance number
 * @param[in] config configuration struct pointer
 */
void ADC_DRV_ConfigConverter(const uint32_t instance,
                             const adc_conv_config_t * const config);

/*!
 * @brief Configures the converter chains the given configuration structure
 *
 * This function configures the ADC Normal and Injected Chains with the options
 * provided in the structure.
 *
 * @param[in] instance the instance number
 * @param[in] config configuration struct pointer
 */
void ADC_DRV_ChainConfig(const uint32_t instance,
                         const adc_chain_config_t * const config);

/*!
 * @brief Reset the ADC
 *
 * This function resets the ADC internal registers to default values.
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_Reset(const uint32_t instance);

/*!
 * @brief Enable a channel
 *
 * This function enables a channel in a specified conversion chain
 *
 * @param[in] instance the instance number
 * @param[in] convChain conversion chain (Normal or Injected)
 * @param[in] chnIdx the index of the channel to enable
 */
void ADC_DRV_EnableChannel(const uint32_t instance,
                           const adc_conv_chain_t convChain,
                           const uint32_t chnIdx);

/*!
 * @brief Disable a channel
 *
 * This function disables a channel in a specified conversion chain
 *
 * @param[in] instance the instance number
 * @param[in] convChain conversion chain (Normal or Injected)
 * @param[in] chnIdx the index of the channel to enable
 */
void ADC_DRV_DisableChannel(const uint32_t instance,
                            const adc_conv_chain_t convChain,
                            const uint32_t chnIdx);

/*!
 * @brief Start conversion
 *
 * This function starts a conversion channel (Normal or Injected)
 *
 * @param[in] instance the instance number
 * @param[in] convChain conversion chain (Normal or Injected)
 */
void ADC_DRV_StartConversion(const uint32_t instance,
                             const adc_conv_chain_t convChain);

/*!
 * @brief Get the status flags
 *
 * This function returns the status flags of the ADC.
 *
 * @param[in] instance the instance number
 * @return the status flag bit-mask
 */
uint32_t ADC_DRV_GetStatusFlags(const uint32_t instance);

/*!
 * @brief Clear the status flags
 *
 * This function clears the status flags of the ADC.
 *
 * @param[in] instance the instance number
 * @param[in] mask bit-mask of flags to clear
 */
void ADC_DRV_ClearStatusFlags(const uint32_t instance,
                              const uint32_t mask);

/*!
 * @brief Get conversion results for a conversion chain
 *
 * This function gets the conversion results for the selected Conversion Chain.
 *
 * @param[in] instance the instance number
 * @param[in] convChain conversion chain (Normal, Injected or CTU)
 * @param[out] results the output buffer
 * @param[in] length the length of the buffer
 * @return the number of values written in the buffer (max length)
 */
uint32_t ADC_DRV_GetConvResultsToArray(const uint32_t instance,
                                       const adc_conv_chain_t convChain,
                                       uint16_t * const results,
                                       const uint32_t length);

/*!
 * @brief Perform Calibration of the ADC
 *
 * This function performs a calibration of the ADC. The maximum input clock 
 * frequency for the ADC is 80 MHz, checked with assertions if DEV_ASSERT is
 * enabled. After calibration, the ADC is left in Powerup state (PWDN bit is clear).
 *
 * @param[in] instance the instance number
 * @return the calibration result
 *  - STATUS_SUCCESS: calibration successful
 *  - STATUS_ERROR: calibration failed
 */
status_t ADC_DRV_DoCalibration(const uint32_t instance);

/*!
 * @brief Power up the ADC
 *
 * This function enables the ADC (disables the Power Down feature).
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_Powerup(const uint32_t instance);

/*!
 * @brief Power down the ADC
 *
 * This function disables the ADC (enables the Power Down feature).
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_Powerdown(const uint32_t instance);

/*!
 * @brief Enable ADC interrupts
 *
 * This function enables ADC interrupts.
 *
 * @param[in] instance the instance number
 * @param[in] interruptMask mask of interrupts to enable (of status flags)
 */
void ADC_DRV_EnableInterrupts(const uint32_t instance,
                              const uint32_t interruptMask);

/*!
 * @brief Disable ADC interrupts
 *
 * This function disables ADC interrupts.
 *
 * @param[in] instance the instance number
 * @param[in] interruptMask mask of interrupts to disable (of status flags)
 */
void ADC_DRV_DisableInterrupts(const uint32_t instance,
                               const uint32_t interruptMask);


/*!
 * @brief Enable ADC interrupt for a channel
 *
 * This function enables interrupt generation on End of Conversion event for a single channel.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_EnableChannelInterrupt(const uint32_t instance,
                                    const uint32_t chnIdx);

/*!
 * @brief Disable ADC interrupt for a channel
 *
 * This function disables interrupt generation on End of Conversion event for a single channel.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_DisableChannelInterrupt(const uint32_t instance,
                                     const uint32_t chnIdx);


/*!
 * @brief Set the Presampling Source for the channel group
 *
 * This function configures the Presampling Source for a channel group.
 *
 * @param[in] instance the instance number
 * @param[in] chanGroup the channel group
 * @param[in] presampleSource the presampling source
 */
void ADC_DRV_SetPresamplingSource(const uint32_t instance,
                                  const adc_chan_group_t chanGroup,
                                  const adc_presampling_source_t presampleSource);

/*!
 * @brief Enable Presampling on one channel
 *
 * This function enables the Presampling on one channel of the ADC.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_EnableChannelPresampling(const uint32_t instance,
                                      const uint32_t chnIdx);

/*!
 * @brief Disable Presampling on one channel
 *
 * This function disables the Presampling on one channel of the ADC.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_DisableChannelPresampling(const uint32_t instance,
                                       const uint32_t chnIdx);

/*!
 * @brief Enable Conversion Presampled Data
 *
 * This function enables bypass of the Sampling Phase, resulting in a conversion
 * of the presampled data. This is available only for channels that have presampling
 * enabled.
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_EnablePresampleConversion(const uint32_t instance);

/*!
 * @brief Disable Conversion of Presampled Data
 *
 * This function disables Sampling Phase bypass.
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_DisablePresampleConversion(const uint32_t instance);

/*!
 * @brief Enable DMA Requests
 *
 * This function enables requests to DMA from ADC
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_EnableDma(const uint32_t instance);

/*!
 * @brief Disable DMA Requests
 *
 * This function disables requests to DMA from ADC
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_DisableDma(const uint32_t instance);

/*!
 * @brief Enable DMA on one channel
 *
 * This function enables DMA requests triggered by End of Conversion event from
 * a selected channel.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_EnableChannelDma(const uint32_t instance,
                              const uint32_t chnIdx);

/*!
 * @brief Disable DMA on one channel
 *
 * This function disables DMA requests triggered by End of Conversion event from
 * a selected channel.
 *
 * @param[in] instance the instance number
 * @param[in] chnIdx the index of the channel
 */
void ADC_DRV_DisableChannelDma(const uint32_t instance,
                               const uint32_t chnIdx);

/*!
 * @brief Set DMA Request Clear Source
 *
 * This function selects the DMA Request Flag Clear Source.
 *
 * @param[in] instance the instance number
 * @param[in] dmaClear the clear source for DMA Requests (Ack from DMA Controller or
    read of data registers)
 */
void ADC_DRV_SetDmaClearSource(const uint32_t instance,
                               const adc_dma_clear_source_t dmaClear);

/*!
 * @brief Get conversion results for a conversion chain with extended information
 *
 * This function gets the conversion results for the selected Conversion Chain, with
 * extended information about each conversion result (channel index, valid an overwritten
 * properties and conversion data). This function should be used in case of configurations
 * with overlapping channel lists in different chains, resulting in overwrite of conversion
 * data when a higher priority chain is executed before all data was read.
 *
 * @param[in] instance the instance number
 * @param[in] convChain conversion chain (Normal, Injected or CTU)
 * @param[out] results the output buffer
 * @param[in] length the length of the buffer
 * @return the number of values written in the buffer (max length)
 */
uint32_t ADC_DRV_GetConvInfoToArray(const uint32_t instance,
                                    const adc_conv_chain_t convChain,
                                    adc_chan_result_t* const results,
                                    const uint32_t length);

/*!
 * @brief Abort ongoing conversion
 *
 * This function aborts an ongoing conversion.
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_AbortConversion(const uint32_t instance);

/*!
 * @brief Abort ongoing chain conversion
 *
 * This function aborts an ongoing chain of conversions.
 *
 * @param[in] instance the instance number
 */
void ADC_DRV_AbortChain(const uint32_t instance);

/*!
 * @brief Configure the analog watchdog
 *
 * This function configures the thresholds registers and set
 * the register used by the ADC channel
 *
 * @param [in] instance - the instance that will be configured
 * @param [in] numThresholds - number of thresholds registers that will be configured
 * @param [in] thresholdValuesArray - structures that contains the thresholds to configure the registers
 * @param [in] numChannels - the number of the channels that will be configured
 * @param [in] chanThresholdsArray - structures with the mapping between channel and the threshold register used
 */
void ADC_DRV_ConfigWdg(const uint32_t instance,
                       const uint8_t numThresholds,
                       const adc_wdg_threshold_values_t * const thresholdValuesArray,
                       const uint8_t numChannels,
                       const adc_wdg_chan_thresholds_t * const  chanThresholdsArray);

/*!
 * @brief Configure watchdog treshold register
 *
 * This function configures the high/low thresholds for a certain register.
 *
 * @param [in] instance - the instance number
 * @param [in] registerIdx - the index of the register
 * @param [in] thresholdValues - the threshold values
 */
void ADC_DRV_SetWdgThreshold(const uint32_t instance,
                             const uint8_t registerIdx,
                             const adc_wdg_threshold_values_t * const thresholdValues);

/*!
 * @brief Set watchdog threshold register
 *
 * This function set the threshold register used for a specific channel.
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 * @param [in] registerIdx - the index of the register
 */
void ADC_DRV_SetWdgChannelMapping(const uint32_t instance,
		                          const uint32_t chnIdx,
                                  const uint32_t registerIdx);

/*!
 * @brief Return the watchdog threshold flags for all the watchdog
 * channels (high and low thresholds).
 * @param [in] instance - the instance number
 * @return the bit mask for all the thresholds of all the channels. Macros ADC_WDOG_REG_MASK_HIGH
 *      and ADC_WDOG_REG_MASK_LOW can be used to decode the data
 */
uint32_t ADC_DRV_GetWdgThresholdFlags(const uint32_t instance);

/*!
 * @brief Clear the watchdog threshold flags.
 *
 * @param [in] instance - the instance number
 * @param [in] wdogRegMask - the mask containing the bits to clear.  Macros ADC_WDOG_REG_MASK_HIGH
 *      and ADC_WDOG_REG_MASK_LOW can be used to compose the mask.
 */
void ADC_DRV_ClearWdgThresholdFlags(const uint32_t instance,
                                    const uint32_t wdogRegMask);

/*!
 * @brief Enable analog watchdog
 *
 * This function enable analog watchdog for a specific channel.
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 */
void ADC_DRV_EnableChannelWdg(const uint32_t instance,
                              const uint32_t chnIdx);

/*!
 * @brief Disable analog watchdog
 *
 * This function disable analog watchdog for a specific channel.
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 */
void ADC_DRV_DisableChannelWdg(const uint32_t instance,
                               const uint32_t chnIdx);

/*!
 * @brief Return the status of a channel
 *
 * This function returns status of a channel.If the function return TRUE the value converted
 * is out of the range that was set for that channel.
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 * @return the status of the channel
 */
bool ADC_DRV_WdgIsChanOutOfRange(const uint32_t instance, const uint32_t chnIdx);

/*!
 * @brief Clear out of range flag
 *
 * This function clears the flag set when the converted value is out of range
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 */
void ADC_DRV_ClearWdgOutOfRangeFlag(const uint32_t instance,  const uint32_t chnIdx);

/*!
 * @brief Return the result of the conversion
 *
 * This function returns the result of the conversion for a single channel
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 */
uint16_t ADC_DRV_GetConvResult (const uint32_t instance, const uint32_t  chnIdx);


/*!
 * @brief Return the result and the status of the conversion
 *
 * This function returns the result and the status of the conversion
 * for a single channel
 *
 * @param [in] instance - the instance number
 * @param [in] chnIdx - the index of the channel
 * @param [out] result - pointer to the buffer where the result is written
 */
void ADC_DRV_GetConvInfo (const uint32_t instance,
                          const uint32_t  chnIdx,
                          adc_chan_result_t* const result);

#if defined (__cplusplus)
}
#endif

/*! @} */

#endif /* ADC_SAR_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
