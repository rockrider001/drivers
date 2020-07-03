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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type with
 * many values.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.4, Mismatched essential type categories for binary operator.
 * This is required by the average calculation of the data conversion.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.6, Composite expression assigned to a wider essential type
 * This is required by the conversion of a bit-field of a register into int16_t type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.7,  Composite expression with smaller essential type than other operand.
 * The expression is safe as the calculation cannot overflow.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 * (different essential type categories).
 * This is required by the conversion of a bit-field of a register into int16_t type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 18.1, Possible creation of out-of-bounds pointer.
 * The DEV_ASSERT checks the instance number to don't exceed the size of the base addresses array.
 */

#include <stddef.h>
#include "device_registers.h"
#include "sdadc_driver.h"
#include "clock_manager.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The position of the mode bit in the input channel enum */
#define SDADC_MODE_BIT_POSITION_IN_CHANNEL_ENUM       (5U)
/*! @brief The position of the common voltage bias selection bit in the input channel enum */
#define SDADC_VCOMSEL_BIT_POSITION_IN_CHANNEL_ENUM    (4U)
/*! @brief Defines for invalid DMA channel number */
#define SDADC_INVALID_DMA_VIRT_CHAN     (0xFFu)
/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for SDADC instances. */
static SDADC_Type * const s_sdadcBase[SDADC_INSTANCE_COUNT] = SDADC_BASE_PTRS;
/* The gain error variable which will be used by some functions */
static int32_t s_gainErr[SDADC_INSTANCE_COUNT] = SDADC_GAIN_ERROR;
/* The offset error in case of data conversion after calibration in
   "single ended mode with negative input = VSS_HV_ADR_D" */
static int16_t s_offsetErrVss[SDADC_INSTANCE_COUNT] = {0};
/* The offset error in case of data conversion after calibration in
   "differential mode" and "single ended mode with negative input =
   (VDD_HV_ADR_D – VSS_HV_ADR_D) / 2" */
static int16_t s_offsetErrVdd[SDADC_INSTANCE_COUNT] = {0};
/* Table of DMA channel number for SDADC instances. */
static uint8_t s_sdadcDmaVirtualChans[SDADC_INSTANCE_COUNT] = SDADC_DMA_CHANNEL_NUMBER;

/* Private functions definitions */
static void SDADC_ConfigResultDma(const uint32_t instance,
                                  const sdadc_result_dma_config_t * const resultDmaConfig);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GetConverterDefaultConfig
* Description   : This function initializes the members of the sdadc_conv_config_t
* structure to default values which are most commonly used for SDADC.
* This function should be called on a structure before using it to configure the converter with
* SDADC_DRV_ConfigConverter(), otherwise all members must be written by the user.
* The user can modify the desired members of the structure after calling this function.
*
* Implements    : SDADC_DRV_GetConverterDefaultConfig_Activity
* END**************************************************************************/
void SDADC_DRV_GetConverterDefaultConfig(sdadc_conv_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->decimaRate = SDADC_DECIMATION_RATE_24;
    config->inputGain = SDADC_INPUT_GAIN_1;
    config->trigSelect = SDADC_TRIGGER_DISABLE;
    config->trigEdge = SDADC_TRIGGER_RISING_EDGE;
    config->outputSetDelay = 0xFF;
    config->highPassFilter = false;
    config->wrapAroundEnable = false;
    config->wraparound = SDADC_CHAN_AN0_AN1;
    config->channelSel = SDADC_CHAN_AN0_VREFN;
    config->enableFifo = true;
    config->fifoThreshold = 15U;
    config->stopInDebug = true;
    config->resultDmaConfig = NULL;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_ConfigConverter
* Description   : This function configures the SDADC converter with the options
* provided in the structure.
*
* Implements    : SDADC_DRV_ConfigConverter_Activity
* END**************************************************************************/
void SDADC_DRV_ConfigConverter(const uint32_t instance,
                               const sdadc_conv_config_t * const config)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(!(config->wrapAroundEnable && (config->channelSel > config->wraparound)));
    DEV_ASSERT(!(config->enableFifo && (config->fifoThreshold > 15u)));
    DEV_ASSERT(config->outputSetDelay >= 16u);
#ifdef ERRATA_E6906
    DEV_ASSERT(config->outputSetDelay >= 23u);
#endif

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    clock_names_t sdadc_clocks[SDADC_INSTANCE_COUNT] = SDADC_CLOCKS;
    uint32_t sdadc_freq = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(sdadc_clocks[instance], &sdadc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;

    DEV_ASSERT((sdadc_freq >= SDADC_CLOCK_FREQ_MIN_RUNTIME) && (sdadc_freq <= SDADC_CLOCK_FREQ_MAX_RUNTIME));
#endif

    SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t mcr = 0u;
    uint32_t mode = (((uint32_t)config->channelSel & (1UL << SDADC_MODE_BIT_POSITION_IN_CHANNEL_ENUM)) != 0UL) ? 1UL : 0UL;
    uint32_t vcomsel = (((uint32_t)config->channelSel & (1UL << SDADC_VCOMSEL_BIT_POSITION_IN_CHANNEL_ENUM)) != 0UL) ? 1UL : 0UL;
    mcr |= SDADC_MCR_EN(1UL);
    mcr |= SDADC_MCR_MODE(mode);
    mcr |= SDADC_MCR_PDR(config->decimaRate);
    mcr |= SDADC_MCR_PGAN(config->inputGain);
    mcr |= SDADC_MCR_GECEN(0x1UL);
    mcr |= SDADC_MCR_HPFEN(config->highPassFilter);
    mcr |= SDADC_MCR_VCOMSEL(vcomsel);
    mcr |= SDADC_MCR_WRMODE(config->wrapAroundEnable);
    mcr |= SDADC_MCR_FRZ(config->stopInDebug);
    mcr |= SDADC_MCR_TRIGEDSEL(config->trigEdge);

    /* Find the selected value that will be wrote to MCR register for trigger selection */
    if (config->trigSelect != SDADC_TRIGGER_DISABLE)
    {
        uint32_t select = (uint32_t)config->trigSelect;

#ifndef FEATURE_SDADC_HAS_INSTANCE_0
        /* Calculate the selected value that be wrote to the MCR register.
           The start value of the TRIGSEL bit field of the MCR register is 0x0, but the start enum value is 0x1u,
           thus the value which will be wrote to MCR register must be the enum value minus 0x1u.*/
        select = ((uint32_t)config->trigSelect) - 0x1u;
#else
        if ((uint32_t)config->trigSelect >= SDADC_INSTANCE_COUNT)
        {
#ifndef FEATURE_SDADC_HAS_COMMON_TRIGGER_SELECTION
            uint32_t imcr[SDADC_INSTANCE_COUNT] = TRIGGER_IMCR_REGISTER_NUMBERS;
            /* Calculate the selected value that be wrote to IMCR register.
               The start value of the SSS bit field of the IMCR register is 0x1, but the start enum value is 0x05u,
               thus the value which will be wrote to IMCR register must be the enum value minus 0x4u.*/
            select = ((uint32_t)config->trigSelect) - 0x4u;
            /* Configure IMCR register to select detail input trigger source */
            SIUL2->IMCR[imcr[instance]] = SIUL2_IMCR_SSS(select);
            /* To configure input trigger source from eTPU or eMIOS, the 0x3 value will be wrote to the TRIGSEL bit field of the MCR register */
            select = 0x3u;
#endif
        }
#endif /* FEATURE_SDADC_HAS_INSTANCE_0 */

        mcr |= SDADC_MCR_TRIGSEL(select);

        /* In the case the SW trigger of the instance is used to trigger itself and the trigger input is enabled,
           writing to STKR register will generate double trigger to the instance, so wraparound mode operation will incorrect.
           Thus the trigger input should not be enabled in this case. */
        if (select == instance)
        {
            if (config->wrapAroundEnable)
            {
                mcr &= ~SDADC_MCR_TRIGEN_MASK;
            }
            else
            {
                mcr |= SDADC_MCR_TRIGEN_MASK;
            }
        }
        else
        {
            mcr |= SDADC_MCR_TRIGEN_MASK;
        }
    }
    else
    {
        mcr &= ~SDADC_MCR_TRIGEN_MASK;
    }
    /* Disable SDADC */
    base->MCR &= ~SDADC_MCR_EN_MASK;
     /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
    /* Make sure that the data fifo overrun flag is not set */
    base->SFR = SDADC_SFR_DFFF_MASK | SDADC_SFR_DFORF_MASK | SDADC_SFR_WTHL_MASK | SDADC_SFR_WTHH_MASK;
    /* Enable SDADC */
    base->MCR |= SDADC_MCR_EN_MASK;
    /* Configure output settling delay */
    base->OSDR = SDADC_OSDR_OSD(config->outputSetDelay);
    /* Configure the input analog channel and wrap around channel */
    base->CSR = SDADC_CSR_ANCHSEL(config->channelSel) | SDADC_CSR_ANCHSEL_WRAP(config->wraparound);
    /* Configure data fifo */
    base->FCR = SDADC_FCR_FE(config->enableFifo) | SDADC_FCR_FTHLD(config->fifoThreshold);
     /* Configure MCR register */
    base->MCR |= mcr;

    /* Configure DMA for transferring results from data fifo to system memory */
    if(config->resultDmaConfig != NULL)
    {
        DEV_ASSERT((config->resultDmaConfig->fifoFullDmaReq || config->resultDmaConfig->wdogOverDmaReq) != false);
        uint32_t eventMask = 0UL;
        eventMask |= (config->resultDmaConfig->fifoFullDmaReq == true) ? SDADC_EVENT_FIFO_FULL : 0UL;
        eventMask |= (config->resultDmaConfig->wdogOverDmaReq == true) ? SDADC_EVENT_WDOG_CROSSOVER : 0UL;

        /* Configure DMA */
        SDADC_ConfigResultDma(instance, config->resultDmaConfig);
        /* Configure SDADC to send DMA requests for data fifo full event */
        SDADC_DRV_EnableDmaEvents(instance, eventMask);
    }
    else
    {
        /* Do nothing */
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_Reset
* Description   : This function resets the SDADC internal registers to their Reference Manual reset values.
*
* Implements    : SDADC_DRV_Reset_Activity
* END**************************************************************************/
void SDADC_DRV_Reset(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    base->MCR = 0u;
    base->CSR = 0u;
    base->OSDR = 0u;
    base->RSER = 0u;
    base->FCR = 0u;
    base->WTHHLR = 0u;
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
    /* Clear all status flags */
    base->SFR = SDADC_SFR_DFFF_MASK | SDADC_SFR_DFORF_MASK | SDADC_SFR_WTHL_MASK | SDADC_SFR_WTHH_MASK;

    /* Set gain error and offset variables to default value */
    s_gainErr[instance] = 65536;
    s_offsetErrVss[instance] = 0;
    s_offsetErrVdd[instance] = 0;
    /* Stop DMA virtual channel and reset the SDADC state storing DMA virtual channel used */
    if(s_sdadcDmaVirtualChans[instance] != SDADC_INVALID_DMA_VIRT_CHAN)
    {
        status_t status;
        status = EDMA_DRV_StopChannel(s_sdadcDmaVirtualChans[instance]);
        (void) status;
        s_sdadcDmaVirtualChans[instance] = SDADC_INVALID_DMA_VIRT_CHAN;
    }

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_Powerup
* Description   : This function disables the SDADC block. SDADC internal modulator placed in low consumption mode
*
* Implements    : SDADC_DRV_Powerdown_Activity
* END**************************************************************************/
void SDADC_DRV_Powerdown(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

   base->MCR &= ~SDADC_MCR_EN_MASK;
   /* Clear the DFFDIRE bit to ensure safe operation*/
   base->RSER &= ~SDADC_RSER_DFFDIRE_MASK;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_Powerup
* Description   : This function enable the SDADC block.
*
* Implements    : SDADC_DRV_Powerup_Activity
* END**************************************************************************/
void SDADC_DRV_Powerup(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);


    SDADC_Type * const base = s_sdadcBase[instance];

    base->MCR |= SDADC_MCR_EN_MASK;

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_RefreshConversion
* Description   : This function resets SDADC internal modulator to start a fresh conversion.
* This function must be call after changing converter configuration(gain, input channel, trigger, watchdog...).
*
* Implements    : SDADC_DRV_RefreshConversion_Activity
* END**************************************************************************/
void SDADC_DRV_RefreshConversion(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    base->RKR = SDADC_RKR_RESET_KEY(0x5AF0u);

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetInputGain
* Description   : This function configures the gain to be applied to the analog input stage of the SDADC.
*
* Implements    : SDADC_DRV_SetInputGain_Activity
* END**************************************************************************/
void SDADC_DRV_SetInputGain(const uint32_t instance,
                            const sdadc_input_gain_t gain)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    base->MCR &= ~SDADC_MCR_PGAN_MASK;
    base->MCR |= SDADC_MCR_PGAN(gain);

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetDecimationRate
* Description   : This function configures the over-sampling ratio to be applied to support different passbands
* with a fixed input sampling clock.
*
* Implements    : SDADC_DRV_SetDecimationRate_Activity
* END**************************************************************************/
void SDADC_DRV_SetDecimationRate(const uint32_t instance,
                                 const sdadc_decimation_rate_t rate)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    base->MCR &= ~SDADC_MCR_PDR_MASK;
    base->MCR |= SDADC_MCR_PDR(rate);
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetOutputSettlingDelay
* Description   : This function configures the output settling delay to be applied to qualify the
* converted output data coming from the SDADC.
*
* Implements    : SDADC_DRV_SetOutputSettlingDelay_Activity
* END**************************************************************************/
void SDADC_DRV_SetOutputSettlingDelay(const uint32_t instance,
                                      const uint8_t outputSettlingDelay)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT(outputSettlingDelay >= 16u);
#ifdef ERRATA_E6906
    DEV_ASSERT(outputSettlingDelay >= 23u);
#endif
    SDADC_Type * const base = s_sdadcBase[instance];

    base->OSDR = SDADC_OSDR_OSD(outputSettlingDelay);
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetAnalogInputBias
* Description   : This function enables analog input bias, the analog input will be connected to
* half-scale bias(VREFP/2).
* Note that: The electrical settings of the pins(E.g: Pull-up, Pull-down) can impact to the biased input.
* For example: If the pull-up of pin is enabled, the actual measured voltage of the biased input is bigger than half-scale(VREFP/2).
*
* Implements    : SDADC_DRV_SetAnalogInputBias_Activity
* END**************************************************************************/
void SDADC_DRV_SetAnalogInputBias(const uint32_t instance,
                                  const uint8_t inputMask,
                                  const bool enable)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    if (enable)
    {
        /* Enable input bias */
        base->CSR |= SDADC_CSR_BIASEN(inputMask);
    }
    else
    {
        /* Disable input bias */
        base->CSR &= ~SDADC_CSR_BIASEN(inputMask);
    }

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetWatchdog
* Description   : This function configures the watch dog monitor with given parameters.
*
* Implements    : SDADC_DRV_SetWatchdog_Activity
* END**************************************************************************/
void SDADC_DRV_SetWatchdog(const uint32_t instance,
                           const bool wdgEnable,
                           const int16_t upperThreshold,
                           const int16_t lowerThreshold)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
#ifdef ERRATA_E8631
    DEV_ASSERT(lowerThreshold >= 0);
#endif
    SDADC_Type * const base = s_sdadcBase[instance];

    base->WTHHLR = SDADC_WTHHLR_THRL(lowerThreshold) | SDADC_WTHHLR_THRH(upperThreshold);

    if (wdgEnable)
    {
#ifdef ERRATA_E8710
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
        clock_names_t sdadc_clocks[SDADC_INSTANCE_COUNT] = SDADC_CLOCKS;
        uint32_t sdadc_freq = 0u;
        uint32_t fm_per_freq = 0u;
        status_t clk_status = CLOCK_SYS_GetFreq(sdadc_clocks[instance], &sdadc_freq);
        DEV_ASSERT(clk_status == STATUS_SUCCESS);
        clk_status = CLOCK_SYS_GetFreq(PBRIDGEx_CLK, &fm_per_freq);
        DEV_ASSERT(clk_status == STATUS_SUCCESS);
        (void) clk_status;
        /* FM Peripheral Clock frequency must be bigger than the sigma-delta ADC clock frequency */
        DEV_ASSERT(sdadc_freq < fm_per_freq);
#endif /* DEV_ERROR_DETECT */
#endif

        base->MCR |= SDADC_MCR_WDGEN_MASK;
    }
    else
    {
        base->MCR &= ~SDADC_MCR_WDGEN_MASK;
    }

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetWraparoundMode
* Description   : This function configures the wraparound mechanism for conversion of
* programmed sequence of channels
*
* Implements    : SDADC_DRV_SetWraparoundMode_Activity
* END**************************************************************************/
void SDADC_DRV_SetWraparoundMode(const uint32_t instance,
                                 const sdadc_inputchannel_sel_t wraparound,
                                 const bool enable)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    if (enable)
    {
        uint32_t trigSelect = (base->MCR & SDADC_MCR_TRIGSEL_MASK) >> SDADC_MCR_TRIGSEL_SHIFT;
        /* In the case the SW trigger of the instance is used to trigger itself and the trigger input is enabled,
           writing to STKR register will generate double trigger to the instance, so wraparound mode operation will incorrect.
           Thus the trigger input should not be enabled in this case. */
        if (trigSelect == instance)
        {
            /* Disable input trigger */
            base->MCR &= ~SDADC_MCR_TRIGEN_MASK;
        }
        else
        {
            /* Do nothing */
        }

        /* Enable wrap around mode */
        base->MCR |= SDADC_MCR_WRMODE_MASK;
    }
    else
    {
        /* Disable wrap around mode */
        base->MCR &= ~SDADC_MCR_WRMODE_MASK;
    }

    /* Set wrap around value */
    base->CSR &= ~SDADC_CSR_ANCHSEL_WRAP_MASK;
    base->CSR |= SDADC_CSR_ANCHSEL_WRAP(wraparound);
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SelectInputChannel
* Description   : This function configures the connectivity of analog inputs to either positive or negative polarity
* terminals of the SDADC. If wraparound mode is enabled, this function supports to configure
* initial entry value for the first loop of the wraparound sequence.
*
* Implements    : SDADC_DRV_SelectInputChannel_Activity
* END**************************************************************************/
void SDADC_DRV_SelectInputChannel(const uint32_t instance,
                                  const sdadc_inputchannel_sel_t channel)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t mode = (((uint32_t)channel & (1UL << SDADC_MODE_BIT_POSITION_IN_CHANNEL_ENUM)) != 0UL) ? 1UL : 0UL;
    uint32_t vcomsel = (((uint32_t)channel & (1UL << SDADC_VCOMSEL_BIT_POSITION_IN_CHANNEL_ENUM)) != 0UL) ? 1UL : 0UL;

    base->MCR &= ~(SDADC_MCR_MODE_MASK | SDADC_MCR_VCOMSEL_MASK);
    base->MCR |= SDADC_MCR_MODE(mode) | SDADC_MCR_VCOMSEL(vcomsel);

    base->CSR &= ~SDADC_CSR_ANCHSEL_MASK;
    base->CSR |= SDADC_CSR_ANCHSEL(channel);
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SwTriggerConv
* Description   : This function generates the trigger event output which can be used
* for triggering conversions.
*
* Implements    : SDADC_DRV_SwTriggerConv_Activity
* END**************************************************************************/
void SDADC_DRV_SwTriggerConv(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    base->STKR = SDADC_STKR_ST_KEY(0xFFFFu);
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_FlushDataFifo
* Description   : This function flush data fifo, all data in the fifo will be erased.
* This function is ignored if have no data in fifo.
*
* Implements    : SDADC_DRV_FlushDataFifo_Activity
* END**************************************************************************/
void SDADC_DRV_FlushDataFifo(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    const SDADC_Type * const base = s_sdadcBase[instance];

    bool fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;

    while(!fifoEmpty)
    {
        (void)base->CDR;
        fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GetConvDataFifo
* Description   : This function gets the converted data from the fifo and put the data into data array.
* The data will be consecutive popped out the fifo until the fifo is empty or the data array is full,
* so the data array length should be big enough to contain all data.
* Note that: This function automatically calibrate the converted data.
* Please use the SDADC_DRV_GetRawConvDataFifo function to get raw data(uncalibrated data)
*
* Implements    : SDADC_DRV_GetConvDataFifo_Activity
* END**************************************************************************/
uint8_t SDADC_DRV_GetConvDataFifo(const uint32_t instance,
                                  const uint8_t length,
                                  int16_t * const data)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT(data != NULL);

    const SDADC_Type * const base = s_sdadcBase[instance];
    bool fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;
    bool validFlag = ((base->SFR & SDADC_SFR_CDVF_MASK) != 0u) ? true : false;
    uint8_t num = 0;
    int32_t rawData = 0;
    /* Get current mode and common voltage setting */
    uint32_t mode = (base->MCR & SDADC_MCR_MODE_MASK) >> SDADC_MCR_MODE_SHIFT;
    bool vcomsel = ((base->MCR & SDADC_MCR_VCOMSEL_MASK) != 0u) ? true : false;

    while(!fifoEmpty && validFlag)
    {
        rawData = (int16_t)((base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT);

        /* The calibrated results should be calculated by equation: CAL_RES = RAW_RES/GCC + OCC */
        /* Nullify the gain error in the data conversion */
        rawData = (rawData * 65536) / s_gainErr[instance];

        /* Nullify the offset error in the data conversion */
        if ((mode == 0x1UL) && !vcomsel)
        {
            rawData = rawData + s_offsetErrVss[instance];
        }
        else
        {
            rawData = rawData + s_offsetErrVdd[instance];
        }

        if (rawData > SDADC_MAX_CONV_DATA)
        {
            rawData = SDADC_MAX_CONV_DATA;
        }
        else if (rawData < SDADC_MIN_CONV_DATA)
        {
            rawData = SDADC_MIN_CONV_DATA;
        }
        else
        {
            /* Do nothing */
        }

        data[num] = (int16_t)rawData;

        fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;
        validFlag = ((base->SFR & SDADC_SFR_CDVF_MASK) != 0u) ? true : false;
        num++;
        if (num >= length)
        {
            break;
        }
        else
        {
            /* Do nothing */
        }
    }

    return num;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GetRawConvDataFifo
* Description   : This function gets the raw converted data(uncalibrated data) from the fifo and
* put the data into data array.
* The data will be consecutive popped out the fifo until the fifo is empty or the data array is full,
* so the data array length should be big enough to contain all data
*
* Implements    : SDADC_DRV_GetRawConvDataFifo_Activity
* END**************************************************************************/
uint8_t SDADC_DRV_GetRawConvDataFifo(const uint32_t instance,
                                     const uint8_t length,
                                     uint16_t * const data)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT(data != NULL);

    const SDADC_Type * const base = s_sdadcBase[instance];
    bool fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;
    bool validFlag = ((base->SFR & SDADC_SFR_CDVF_MASK) != 0u) ? true : false;
    uint8_t num = 0;

    while(!fifoEmpty && validFlag)
    {
        /* Read the 32-bit data register */
        data[num] = (base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT;
        /* Check the status flags */
        fifoEmpty = ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u) ? true : false;
        validFlag = ((base->SFR & SDADC_SFR_CDVF_MASK) != 0u) ? true : false;
        num++;
        if (num >= length)
        {
            break;
        }
        else
        {
            /* Do nothing */
        }
    }

    return num;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GetStatusFlags
* Description   : This function returns the status flags of the SDADC.
* Bitwise AND the returned value with the SDADC_FLAG_ defines to get a specific status flag.
*
* Implements    : SDADC_DRV_GetStatusFlags_Activity
* END**************************************************************************/
uint32_t SDADC_DRV_GetStatusFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    const SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t flags = base->SFR;

    return flags;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_ClearStatusFlags
* Description   : This function clears the status flags that are set to '1' in the mask.
* The mask input parameter can be set using SDADC_FLAG_ defines.
*
* Implements    : SDADC_DRV_ClearStatusFlags_Activity
* END**************************************************************************/
void SDADC_DRV_ClearStatusFlags(const uint32_t instance,
                                const uint32_t mask)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT((mask & ~SDADC_FLAG_STATUS_ALL) == 0u);

    SDADC_Type * const base = s_sdadcBase[instance];

    /* Write-1-to-clear bits in the status register */
    base->SFR = mask;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_SetGlobalDmaInterruptGate
* Description   : This function configures SDADC Global DMA/Interrupt gate.
*
* Implements    : SDADC_DRV_SetGlobalDmaInterruptGate_Activity
* END**************************************************************************/
void SDADC_DRV_SetGlobalDmaInterruptGate(const uint32_t instance,
                                         const sdadc_dmaint_gate_select_t select,
                                         const bool enable)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];

    if (enable)
    {
        /* Enable Global DMA/Interrupt gate */
        base->RSER |=  SDADC_RSER_GDIGE_MASK;
    }
    else
    {
        /* Disable Global DMA/Interrupt gate */
        base->RSER &= ~SDADC_RSER_GDIGE_MASK;
    }

#ifndef FEATURE_SDADC_HAS_COMMON_DMAINT_GATE_SELECTION
    uint32_t imcr[SDADC_INSTANCE_COUNT] = GATE_IMCR_REGISTER_NUMBERS;
    /* Configure IMCR register to select detail input gate source */
    SIUL2->IMCR[imcr[instance]] = SIUL2_IMCR_SSS(select);
#else
    uint32_t shift = instance * SIU_SDGATE_SEL_SD_A_GATE_SEL_WIDTH;
    uint32_t mask = (uint32_t)SIU_SDGATE_SEL_SD_A_GATE_SEL_MASK << shift;

    /* Configure GATE_SEL register to select detail input gate source */
    SIU->SDGATE_SEL &= ~mask;
    SIU->SDGATE_SEL |= ((uint32_t)select << shift) & mask;
#endif

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_EnableDmaEvents
* Description   : This function enables SDADC DMA request generating.
*
* Implements    : SDADC_DRV_EnableDmaEvents_Activity
* END**************************************************************************/
void SDADC_DRV_EnableDmaEvents(const uint32_t instance,
                               const uint32_t event_mask)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT((event_mask & SDADC_EVENT_FIFO_OVERRUN) == 0U);
    DEV_ASSERT((event_mask & SDADC_EVENT_CONV_DATA_VALID) == 0U);
#ifdef ERRATA_E10415
    DEV_ASSERT((event_mask & SDADC_EVENT_WDOG_CROSSOVER) == 0U);
#endif

    SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t rser = 0U;

    rser |= ((event_mask & SDADC_EVENT_FIFO_FULL) != 0U) ? (SDADC_RSER_DFFDIRE_MASK | SDADC_RSER_DFFDIRS_MASK) : 0U;
    rser |= ((event_mask & SDADC_EVENT_WDOG_CROSSOVER) != 0U) ? (SDADC_RSER_WTHDIRE_MASK | SDADC_RSER_WTHDIRS_MASK): 0U;

    /* Enable DMA generating */
    base->RSER |=  rser;

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_EnableInterruptEvents
* Description   : This function enables SDADC interrupt request generating.
*
* Implements    : SDADC_DRV_EnableInterruptEvents_Activity
* END**************************************************************************/
void SDADC_DRV_EnableInterruptEvents(const uint32_t instance,
                                     const uint32_t event_mask)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t intEnable = 0U;
    uint32_t intSel = 0U;

    intEnable |= ((event_mask & SDADC_EVENT_FIFO_FULL) != 0U) ? SDADC_RSER_DFFDIRE_MASK : 0U;
    intEnable |= ((event_mask & SDADC_EVENT_WDOG_CROSSOVER) != 0U) ? SDADC_RSER_WTHDIRE_MASK : 0U;
    intEnable |= ((event_mask & SDADC_EVENT_FIFO_OVERRUN) != 0U) ? SDADC_RSER_DFORIE_MASK : 0U;
    intEnable |= ((event_mask & SDADC_EVENT_CONV_DATA_VALID) != 0U) ? SDADC_RSER_CDVEE_MASK : 0U;

    intSel |= ((event_mask & SDADC_EVENT_FIFO_FULL) != 0U) ? SDADC_RSER_DFFDIRS_MASK : 0U;
    intSel |= ((event_mask & SDADC_EVENT_WDOG_CROSSOVER) != 0U) ? SDADC_RSER_WTHDIRS_MASK : 0U;

    /* Select interrupt request */
    base->RSER &= ~intSel;

    /* Enable interrupt */
    base->RSER |=  intEnable;

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_DisableEvents
* Description   : This function disable SDADC DMA and interrupt request generating
*
* Implements    : SDADC_DRV_DisableEvents_Activity
* END**************************************************************************/
void SDADC_DRV_DisableEvents(const uint32_t instance,
                             const uint32_t event_mask)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t intDisable = 0U;

    intDisable |= ((event_mask & SDADC_EVENT_FIFO_FULL) != 0U) ? SDADC_RSER_DFFDIRE_MASK : 0U;
    intDisable |= ((event_mask & SDADC_EVENT_WDOG_CROSSOVER) != 0U) ? SDADC_RSER_WTHDIRE_MASK : 0U;
    intDisable |= ((event_mask & SDADC_EVENT_FIFO_OVERRUN) != 0U) ? SDADC_RSER_DFORIE_MASK : 0U;
    intDisable |= ((event_mask & SDADC_EVENT_CONV_DATA_VALID) != 0U) ? SDADC_RSER_CDVEE_MASK : 0U;
    /* Disable DMA and interrupt */
    base->RSER &= ~intDisable;

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GetInterruptNumber
* Description   : This function returns the interrupt number for the specified SDADC instance.
* Note that: depending on platform, there might be multiple SDADC instances which use the same IRQ number.
*
* Implements    : SDADC_DRV_GetInterruptNumber_Activity
* END**************************************************************************/
IRQn_Type SDADC_DRV_GetInterruptNumber(const uint32_t instance)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

    const IRQn_Type sdadcIrqId[SDADC_INSTANCE_COUNT] = SDADC_IRQS;
    IRQn_Type irqId = sdadcIrqId[instance];

    return irqId;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_GainCalibration
* Description   : This function performs a gain calibration of the SDADC. Gain calibration
* should be run before using the SDADC converter or after the operating conditions
* (particularly Vref) change significantly. The measured gain value is going be used to
* nullify the gain errors in the data conversion.
* The conversion number that is performed by calibration should be from 16 to 64. The higher the number
* of conversion done, the higher the rejection of noise during the calibration.
*
* Implements    : SDADC_DRV_GainCalibration_Activity
* END**************************************************************************/
void SDADC_DRV_GainCalibration(const uint32_t instance,
                               const uint8_t convNum)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT((convNum >= 16u) && (convNum <= 64u));

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    clock_names_t sdadc_clocks[SDADC_INSTANCE_COUNT] = SDADC_CLOCKS;
    uint32_t sdadc_freq = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(sdadc_clocks[instance], &sdadc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;
    DEV_ASSERT((sdadc_freq >= SDADC_CLOCK_FREQ_MIN_RUNTIME) && (sdadc_freq <= SDADC_CLOCK_FREQ_MAX_RUNTIME));
#endif

    SDADC_Type * const base = s_sdadcBase[instance];
    int32_t actualData = 0;
    int32_t dP = 0;
    int32_t dN = 0;
    uint8_t i;
    int32_t tmp = 0;

    /* Save the current state of MCR to restore the configuration after calibration */
    uint32_t currentMCR = base->MCR;
    /* Save the current state of CSR to restore the configuration after calibration */
    uint32_t currentCSR = base->CSR;
    /* Save the current state of OSDR to restore the configuration after calibration */
    uint32_t currentOSDR = base->OSDR;
    /* Save the current state of FCR to restore the configuration after calibration */
    uint32_t currentFCR = base->FCR;

    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
    /* Make sure that the data fifo overrun flag is not set */
    SDADC_DRV_ClearStatusFlags(instance, (SDADC_FLAG_DATA_FIFO_OVERRUN | SDADC_FLAG_DATA_FIFO_FULL));
    /* Enable SDADC */
    base->MCR |= SDADC_MCR_EN_MASK;
    /* Configure converter to perform gain calibration */
    base->MCR = SDADC_MCR_EN(1u) | SDADC_MCR_MODE(0u) | SDADC_MCR_GECEN(1u) | SDADC_MCR_HPFEN(0u) | SDADC_MCR_PGAN(0u);
    /* Output settling Time delay has to be set at least to 16 */
    base->OSDR = SDADC_OSDR_OSD(0xFFu);
    /* Select input channel mux to perform full positive scale calibration */
    base->CSR = SDADC_CSR_BIASEN(0u) | SDADC_CSR_ANCHSEL(0x6u);
    /* Enable the data fifo  */
    base->FCR = SDADC_FCR_FE(1u) | SDADC_FCR_FTHLD(15u);
    /* Start calibration */
    base->RKR = SDADC_RKR_RESET_KEY(0x5AF0u);

    for (i = 0u; i < convNum; i++)
    {
        /* Wait for conversion to finish */
        while ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u)
        {}
        tmp = (int16_t)((base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT);
        actualData = actualData + tmp;
        /* If DFORF is set, further datawords will not be received into FIFO even if sufficient room exists,
        so make sure the data fifo overrun flag is not set */
        base->SFR = SDADC_SFR_DFORF_MASK | SDADC_SFR_DFFF_MASK;
    }
    /* Calculate average of full positive scale data conversions */
    dP = (int32_t)(actualData / convNum);

    /* Perform full negative scale calibration */
    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
     /* Make sure that the data fifo overrun flag is not set */
    SDADC_DRV_ClearStatusFlags(instance, (SDADC_FLAG_DATA_FIFO_OVERRUN | SDADC_FLAG_DATA_FIFO_FULL));
     /* Enable SDADC */
    base->MCR |= SDADC_MCR_EN_MASK;
    /* Change the input channel mux to perform full negative scale calibration */
    base->CSR = SDADC_CSR_ANCHSEL(0x7u);
    /* Start calibration */
    base->RKR = SDADC_RKR_RESET_KEY(0x5AF0u);
    actualData = 0u;

    for (i = 0u; i < convNum; i++)
    {
        /* Wait for conversion to finish */
        while ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u)
        {}
        tmp = (int16_t)((base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT);
        actualData = actualData + tmp;
        /* If DFORF is set, further datawords will not be received into FIFO even if sufficient room exists,
        so make sure the data fifo overrun flag is not set */
        base->SFR = SDADC_SFR_DFORF_MASK | SDADC_SFR_DFFF_MASK;
    }
    /* Stop conversion */
    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Calculate average of full negative scale data conversions */
    dN = (int32_t)(actualData / convNum);
    /* Calculate the gain error */
    s_gainErr[instance] = dP - dN;

    /* Restore the configuration of OSDR register */
    base->OSDR = currentOSDR;
    /* Restore the configuration of CSR register */
    base->CSR = currentCSR;
    /* Restore the configuration of FCR register */
    base->FCR = currentFCR;
    /* Restore the configuration of MCR register */
    base->MCR = currentMCR;
}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_OffsetCalibration
* Description   : This function performs a offset calibration of the SDADC. Offset calibration
* should be run before using the SDADC converter or after the operating conditions
* (particularly Vref, input gain) change significantly.
* The measured offset is going be used to nullify the offset error in the data conversion.
* The offset calibration must be performed for each input gain changing since it is expected to
* vary with input gain configuration of SDADC.
*
* Implements    : SDADC_DRV_OffsetCalibration_Activity
* END**************************************************************************/
void SDADC_DRV_OffsetCalibration(const uint32_t instance,
                                 const sdadc_input_gain_t userGain)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    clock_names_t sdadc_clocks[SDADC_INSTANCE_COUNT] = SDADC_CLOCKS;
    uint32_t sdadc_freq = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(sdadc_clocks[instance], &sdadc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;
    DEV_ASSERT((sdadc_freq >= SDADC_CLOCK_FREQ_MIN_RUNTIME) && (sdadc_freq <= SDADC_CLOCK_FREQ_MAX_RUNTIME));
#endif

    SDADC_Type * const base = s_sdadcBase[instance];
    int16_t actualData = 0;
    int16_t expectedData = 0;

    /* Save the current state of MCR to restore the configuration after calibration */
    uint32_t currentMCR = base->MCR;
    /* Save the current state of CSR to restore the configuration after calibration */
    uint32_t currentCSR = base->CSR;
    /* Save the current state of OSDR to restore the configuration after calibration */
    uint32_t currentOSDR = base->OSDR;
    /* Configure converter to perform gain calibration */
    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
    /* Make sure that the data fifo overrun flag is not set */
    SDADC_DRV_ClearStatusFlags(instance, (SDADC_FLAG_DATA_FIFO_OVERRUN | SDADC_FLAG_DATA_FIFO_FULL));
    /* Enable SDADC */
    base->MCR |= SDADC_MCR_EN_MASK;
    /* Configure SDADC converter */
    base->MCR = SDADC_MCR_EN(1u) | SDADC_MCR_MODE(0u) | SDADC_MCR_GECEN(1u) | SDADC_MCR_HPFEN(0u) | SDADC_MCR_PGAN(userGain);
    /* Output settling Time delay has to be set at least to 16 */
    base->OSDR = SDADC_OSDR_OSD(0xFFu);
    /* Perform calibration in case of data conversion after calibration in "single ended mode with negative input = VSS_HV_ADR_D" */
    base->CSR = SDADC_CSR_BIASEN(0u) | SDADC_CSR_ANCHSEL(0x4u);
    /* Start calibration */
    base->RKR = SDADC_RKR_RESET_KEY(0x5AF0u);

    while ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u)
    {
        /* Wait for conversion to finish */
    }
    /* Get the conversion data */
    actualData = (int16_t)((base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT);
    /* Calculate the offset error */
    s_offsetErrVss[instance] = expectedData - actualData;

    /* Disable SDADC */
    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Flush data fifo to make sure the fifo does not contains data of previous conversions */
    SDADC_DRV_FlushDataFifo(instance);
    /* Make sure that the data fifo overrun flag is not set */
    SDADC_DRV_ClearStatusFlags(instance, (SDADC_FLAG_DATA_FIFO_OVERRUN | SDADC_FLAG_DATA_FIFO_FULL));
    /* Enable SDADC */
    base->MCR |= SDADC_MCR_EN_MASK;
    /* Perform calibration in case of data conversion after calibration in "differential mode" and
    "single ended mode with negative input = (VDD_HV_ADR_D – VSS_HV_ADR_D) / 2" */
    base->CSR = SDADC_CSR_ANCHSEL(0x5u);
    /* Start calibration */
    base->RKR = SDADC_RKR_RESET_KEY(0x5AF0u);
    actualData = 0;

    while ((base->SFR & SDADC_SFR_DFEF_MASK) != 0u)
    {
        /* Wait for conversion to finish */
    }
    /* Get the conversion data */
    actualData = (int16_t)((base->CDR & SDADC_CDR_CDATA_MASK) >> SDADC_CDR_CDATA_SHIFT);
    /* Calculate the offset error */
    s_offsetErrVdd[instance] = expectedData - actualData;
    /* Stop conversion */
    base->MCR &= ~SDADC_MCR_EN_MASK;
    /* Restore the configuration of OSDR register */
    base->OSDR = currentOSDR;
    /* Restore the configuration of CSR register */
    base->CSR = currentCSR;
    /* Restore the configuration of MCR register */
    base->MCR = currentMCR;

}

/*FUNCTION**********************************************************************
*
* Function Name : SDADC_DRV_CalibrateDataSet
* Description   : This function calibrates the uncalibrated converted data which is
* got by DMA or SDADC_DRV_GetRawConvDataFifo function.
* In the case uncalibrated converted data which is got by DMA, this function should be
* called in the DMA callback.
*
* Implements    : SDADC_DRV_CalibrateDataSet_Activity
* END**************************************************************************/
void SDADC_DRV_CalibrateDataSet(const uint32_t instance,
                                const uint16_t * const uncalibratedBuffer,
                                int32_t * const calibratedBuffer,
                                const uint32_t bufferLength)
{
    DEV_ASSERT(instance < SDADC_INSTANCE_COUNT);
    DEV_ASSERT(uncalibratedBuffer != NULL);
    DEV_ASSERT(calibratedBuffer != NULL);

    const SDADC_Type * const base = s_sdadcBase[instance];
    int32_t caliData = 0;
    uint32_t index;
    /* Get current mode and common voltage setting */
    uint32_t mode = (base->MCR & SDADC_MCR_MODE_MASK) >> SDADC_MCR_MODE_SHIFT;
    bool vcomsel = ((base->MCR & SDADC_MCR_VCOMSEL_MASK) != 0u) ? true : false;

    for(index = 0; index < bufferLength; index++)
    {
        caliData = (int16_t)uncalibratedBuffer[index];

        /* The calibrated results should be calculated by equation: CAL_RES = RAW_RES/GCC + OCC */
        /* Nullify the gain error in the data conversion */
        caliData = (caliData * 65536) / s_gainErr[instance];

        /* Nullify the offset error in the data conversion */
        if ((mode == 0x1UL) && !vcomsel)
        {
            caliData = caliData + s_offsetErrVss[instance];
        }
        else
        {
            caliData = caliData + s_offsetErrVdd[instance];
        }

        calibratedBuffer[index] = caliData;

    }

}

static void SDADC_ConfigResultDma(const uint32_t instance,
                                  const sdadc_result_dma_config_t * const resultDmaConfig)
{
    const SDADC_Type * const base = s_sdadcBase[instance];
    uint32_t srcAddr, destAddr, destSizeInBytes;

    srcAddr = (uint32_t)(&(base->CDR)) + 2u; /* +2u because read op needs to be done on CDATA, from the second 16bits in 32bits CDR register */
    destAddr = (uint32_t) resultDmaConfig->destPtr;
    destSizeInBytes = sizeof(resultDmaConfig->destPtr[0u]) * resultDmaConfig->destLength;

    /* Update SDADC state with the DMA virtual channel number for current rfifo */
    s_sdadcDmaVirtualChans[instance] = resultDmaConfig->dmaVirtualChan;

    /* Configure EDMA to transfer 1 result for each SDADC result.
     * After a number of transfers equal with the destination buffer length, EDMA shall call the registered callback (if not NULL) */
    edma_transfer_config_t tCfg;
    edma_loop_transfer_config_t tLoopCfg;
    tCfg.destAddr           = destAddr;
    tCfg.srcAddr            = srcAddr;
    tCfg.srcTransferSize    = EDMA_TRANSFER_SIZE_2B;
    tCfg.destTransferSize   = EDMA_TRANSFER_SIZE_2B;
    tCfg.srcOffset          = 0;
    tCfg.destOffset         = (int16_t)sizeof(resultDmaConfig->destPtr[0u]); /* After each transfer, the dest address is incremented with the size of uint16_t */
    tCfg.srcLastAddrAdjust  = 0;
    tCfg.destLastAddrAdjust = -(int32_t)destSizeInBytes; /* Circular dest buffer: after the dest buffer is filled, dest address is decremented with size in bytes. */
    tCfg.srcModulo          = EDMA_MODULO_OFF;
    tCfg.destModulo         = EDMA_MODULO_OFF;
    tCfg.minorByteTransferCount    = sizeof(resultDmaConfig->destPtr[0u]);
    tCfg.scatterGatherEnable       = false;
    tCfg.scatterGatherNextDescAddr = 0u;
    tCfg.interruptEnable           = true;
    tCfg.loopTransferConfig        = &tLoopCfg;

    tLoopCfg.majorLoopIterationCount = resultDmaConfig->destLength; /* Number of minor loops in a major loop */
    tLoopCfg.srcOffsetEnable         = false;
    tLoopCfg.dstOffsetEnable         = false;
    tLoopCfg.minorLoopOffset         = 0;
    tLoopCfg.minorLoopChnLinkEnable  = false;
    tLoopCfg.minorLoopChnLinkNumber  = 0u;
    tLoopCfg.majorLoopChnLinkEnable  = false;
    tLoopCfg.majorLoopChnLinkNumber  = 0u;

    status_t status;
    status = EDMA_DRV_ConfigLoopTransfer(resultDmaConfig->dmaVirtualChan, &tCfg);
    (void) status;

    if(resultDmaConfig->callback != NULL)
    {
        status = EDMA_DRV_InstallCallback(resultDmaConfig->dmaVirtualChan, resultDmaConfig->callback, resultDmaConfig->callbackParam);
        (void) status;

        EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_ERR_INT, true);
        EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_MAJOR_LOOP_INT, true);

        if(resultDmaConfig->enHalfDestCallback == true)
        {
            EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_HALF_MAJOR_LOOP_INT, true);
        }
        else
        {
            EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_HALF_MAJOR_LOOP_INT, false);
        }
    }
    else
    {
        EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_ERR_INT, false);
        EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_HALF_MAJOR_LOOP_INT, false);
        EDMA_DRV_ConfigureInterrupt(resultDmaConfig->dmaVirtualChan, EDMA_CHN_MAJOR_LOOP_INT, false);
    }

    status = EDMA_DRV_StartChannel(resultDmaConfig->dmaVirtualChan);
    (void) status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
