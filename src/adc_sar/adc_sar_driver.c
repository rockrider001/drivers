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
/**
 * @page misra_violations MISRA-C:2012 violations
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macro defines a bitmask used to access status flags.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 4.9, Function-like macro defined.
 * Function-like macros are used instead of inline functions in order to ensure
 * that the performance will not be decreased if the functions will not be
 * inlined by the compiler.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.4, more than one 'break' terminates loop.
 * 'break' is used to terminate the loops on different and unrelated conditions
 *
 */

#include <stddef.h>
#include "device_registers.h"
#include "adc_sar_driver.h"
#include "adc_sar_hw_access.h"

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
/* Clock Manager is a dependency only when DEV_ASSERT is enabled, used in:
  - ADC_DRV_DoCalibration
  - ADC_DRV_ConfigConverter
*/
#include "clock_manager.h"

/* local definitions on the limits of the clock frequency */
#define ADC_CLOCK_FREQ_MAX_RUNTIME     (80000000U)
#define ADC_CLOCK_FREQ_MAX_CALIBRATION (40000000U)

#endif

/*******************************************************************************
 * Pre-check
 ******************************************************************************/
 /* If the bad access proctection is not explicitly enabled, it is disabled */
#ifndef FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    #define FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL (0)
#endif

#ifndef FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    #define FEATURE_ADC_BAD_ACCESS_PROT_FEATURE (0)
#endif

#ifndef ADC_CWSELR_COUNT
    /* if ADC_CWSELR_COUNT is not defined,  it's likely because CWSELR are unrolled
    instead of in array form (FEATURE_ADC_HAS_CWSELR_UNROLLED == 1), assume the maximum
    number of 12 CWSELR regs */
    #define ADC_CWSELR_COUNT (12u)
#endif

/* define a maximum group number as 3. The ADC SAR has support for at most 96
channels mapped into 3 groups (sometimes referred as Internal, Precision and
External) of maximum 32 channels */
#define ADC_SAR_MAX_GROUP_NUM (3u)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for ADC instances. */
static ADC_Type * const s_adcBase[ADC_INSTANCE_COUNT] = ADC_BASE_PTRS;
#ifdef ADC_THRHLR_PER_INSTANCE_COUNT
static uint32_t const s_adcThrhlr_count[ADC_INSTANCE_COUNT] = ADC_THRHLR_PER_INSTANCE_COUNT;
#endif

#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
static const uint32_t ADC_CHAN_BITMAP[ADC_INSTANCE_COUNT][ADC_SAR_MAX_GROUP_NUM] = FEATURE_ADC_CHN_AVAIL_BITMAP;
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
static const uint32_t ADC_FEATURE_BITMAP[ADC_INSTANCE_COUNT] = FEATURE_ADC_FEAT_AVAIL_BITMAP;
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */

/*******************************************************************************
 * Defines
 ******************************************************************************/
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
/* ADC_INST_HAS_REG_<bitwidth> macros. Checks the availability of a
    register index based on the availability of the ADC channels
    for the particular instance on the platform.
    <bitwidth> represents the width of the bitfield associated with each
    ADC channel in the register series.
*/
#define ADC_INST_HAS_REG_1(inst,regIdx)   (ADC_CHAN_BITMAP[inst][regIdx] != 0u)
#define ADC_INST_HAS_REG_4(inst,regIdx)  ((ADC_CHAN_BITMAP[inst][regIdx / 4u] & (0xFFUL << ((regIdx % 4u) * 8u))) != 0u)
#define ADC_INST_HAS_REG_32(inst,regIdx) ((ADC_CHAN_BITMAP[inst][regIdx / 32u] & (0x01UL << (regIdx % 32u))) != 0u)

/* <bitwidth> = 1. Each bit in these registers correspons to one ADC channel */
#define ADC_INST_HAS_CEOCFRn(inst,n) (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_CIMRn(inst,n)   (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_DMARn(inst,n)   (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_PSRn(inst,n)    (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_CTRn(inst,n)    (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_NCMRn(inst,n)   (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_JCMRn(inst,n)   (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_CWENRn(inst,n)  (ADC_INST_HAS_REG_1(inst,n))
#define ADC_INST_HAS_AWORRn(inst,n)  (ADC_INST_HAS_REG_1(inst,n))

/* <bitwidth> = 4. Four bits in the register correspond to one ADC channel */
#define ADC_INST_HAS_CWSELRn(inst,x) (ADC_INST_HAS_REG_4(inst,x))

/* <bitwidth> = 32. The entire 4 byte register is associated with one ADC channel */
#define ADC_INST_HAS_CDRn(inst,n) (ADC_INST_HAS_REG_32(inst,n))
#define ADC_INST_HAS_CHANn(inst,n) (ADC_INST_HAS_REG_32(inst,n))

#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE

#define ADC_INST_HAS_DSDR(inst) ((ADC_FEATURE_BITMAP[inst] & (1UL << 0u)) != 0u)
#define ADC_INST_HAS_PSCR(inst) ((ADC_FEATURE_BITMAP[inst] & (1UL << 1u)) != 0u)
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
/*FUNCTION*********************************************************************
 *
 * Function Name : ADC_GetConvResults
 * Description   : Reads the conversion results to output arrays
 * Params:
 *  - instance : the ADC instance to read
 *  - convChain : the conversion chain (Normal, Injected or CTU)
 *  - resultsRaw : an uint16_t array to write only conversion data
 *  - resultsStruct : an adc_chan_result_t array to write detailed information
                      about each conversion result
 *  - length : the maximum size of resultsRaw and resultsStruct
 *
 * resultsRaw or resultsStruct can be NULL, but not both, as the function will
 * have nowhere to write the results.
 *
 *END*************************************************************************/
static uint32_t ADC_GetConvResults(const uint32_t instance,
                                   const adc_conv_chain_t convChain,
                                   uint16_t * const resultsRaw,
                                   adc_chan_result_t* const resultsStruct,
                                   const uint32_t length)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    /* resultsRaw and resultsStruct cannot be both NULL */
    DEV_ASSERT((resultsRaw != NULL) ||  (resultsStruct != NULL));

    uint32_t index = 0u;
    bool length_exceeded = false;
    ADC_Type * const base = s_adcBase[instance];
    uint32_t vectAdr;
    /* go through each channel group */
    for (vectAdr = 0u; vectAdr < ADC_CEOCFR_COUNT; vectAdr++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_CEOCFRn(instance, vectAdr))
        {
            continue; /* skip if the CEOCFR[vectAdr] is not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        if (base->CEOCFR[vectAdr] == 0U)
        {
            continue; /* skip if there are no complete conversions in the group */
        }
        uint32_t vectBit;
        /* go through each bit in the group, check if there is a completed conversion */
        for (vectBit = 0u; vectBit < 32u; vectBit++)
        {
            if ((base->CEOCFR[vectAdr] & (1UL << vectBit)) == 0U)
            {
                continue; /* skip if the channel does not have a conversion ready */
            }
            uint8_t chnIdx = (uint8_t)((vectAdr * 32u) + vectBit);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
            if (!ADC_INST_HAS_CDRn(instance, chnIdx))
            {
                continue; /* skip if the CDR register is not available */
            }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
            if (chnIdx >= ADC_CDR_COUNT)
            {
                /* terminate if we exceeded the maximum index of the channels */
                length_exceeded = true;
                break;
            }
            uint32_t cdr = base->CDR[chnIdx];
            if (ADC_CDR_RESULT((uint32_t)convChain) == (cdr & ADC_CDR_RESULT_MASK))
            {
                /* if the result type matches the one request by convChain,
                    write to the output array(s)
                */
                if (resultsRaw != NULL)
                {
                    resultsRaw[index] = (uint16_t)(cdr & ADC_CDR_CDATA_MASK) >> ADC_CDR_CDATA_SHIFT;
                }
                if (resultsStruct != NULL)
                {
                    resultsStruct[index].cdata = (uint16_t)(cdr & ADC_CDR_CDATA_MASK) >> ADC_CDR_CDATA_SHIFT;
                    resultsStruct[index].chnIdx = chnIdx;
                    resultsStruct[index].valid = ((cdr & ADC_CDR_VALID_MASK) != 0U) ? true : false;
                    resultsStruct[index].overWritten = ((cdr & ADC_CDR_OVERW_MASK) != 0U) ? true : false;
                }
                /* increment the current index and reset the CEOCFR flag */
                base->CEOCFR[vectAdr] = (1UL) << vectBit; /* w1c bit */
                index++;
                if (index >= length)
                {
                    /* We have filled the output buffer, exit de loop.
                     * Data may still exist in the result registers, which won't be reached.
                     */
                    length_exceeded = true;
                    break;
                }
            }
        }
        if (length_exceeded)
        {
            break;
        }
    }
    return index;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : ADC_ResetWdog
 * Description   : Reset and disable the Analog Watchdog feature
 *
 *END*************************************************************************/
static void ADC_ResetWdog(const uint32_t instance)
{
    uint8_t i;

    ADC_Type * const base = s_adcBase[instance];
    uint32_t thrhlr_count;
#ifdef ADC_THRHLR_PER_INSTANCE_COUNT
    thrhlr_count = s_adcThrhlr_count[instance];
#else
    thrhlr_count = ADC_THRHLR_COUNT;
#endif

    for (i = 0u; i < ADC_CWENR_COUNT; i++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_CWENRn(instance, i))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->CWENR[i] = 0u;
    }

    for (i = 0u; i < ADC_AWORR_COUNT; i++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_AWORRn(instance, i))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->AWORR[i] = 0xFFFFFFFFu; /* w1c bits */
    }

    for (i = 0u; i < thrhlr_count; i++)
    {
        ADC_WriteThresholds(base, i, 0xFFFFu, 0u);
    }

    for (i = 0u; i < ADC_CWSELR_COUNT; i++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_CWSELRn(instance, i))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        ADC_ResetWdogCWSELR(base, i);
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetDefaultConfigConverter
* Description   : This function initializes the members of the adc_conv_config_t
*  structure to default values (Reference Manual resets).
*
* Implements    : ADC_DRV_GetDefaultConfigConverter_Activity
* END**************************************************************************/
void ADC_DRV_GetDefaultConfigConverter(adc_conv_config_t * const config)
{
    DEV_ASSERT(config != NULL);
    config->convMode = ADC_CONV_MODE_ONESHOT;
    config->clkSelect = ADC_CLK_FULL_BUS;
    config->refSelect = ADC_REF_VREFH;
#if FEATURE_ADC_HAS_CTU
    config->ctuMode = ADC_CTU_MODE_DISABLED;
#endif
#if FEATURE_ADC_HAS_INJ_TRIGGER_SEL
    config->injectedEdge = ADC_INJECTED_EDGE_DISABLED;
#endif
#if FEATURE_ADC_HAS_EXT_TRIGGER
    config->extTrigger = ADC_EXT_TRIGGER_DISABLED;
#endif
    config->sampleTime0 = ADC_DEF_SAMPLE_TIME_0;
    config->sampleTime1 = ADC_DEF_SAMPLE_TIME_1;
    config->sampleTime2 = ADC_DEF_SAMPLE_TIME_2;
    config->autoClockOff = false;
    config->overwriteEnable = false;
    config->dataAlign = ADC_DATA_ALIGNED_RIGHT;
    config->decodeDelay = 0U;
    config->powerDownDelay = 0U;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ConfigConverter
* Description   : This function configures the ADC converter with the options
*  provided in the configuration structure.
*
* Implements    : ADC_DRV_ConfigConverter_Activity
* END**************************************************************************/
void ADC_DRV_ConfigConverter(const uint32_t instance,
                             const adc_conv_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
#if FEATURE_ADC_HAS_CTU
#if (FEATURE_ADC_HAS_CTU_TRIGGER_MODE == 0)
    /* test that the feature is enabled only on platforms that have it */
    DEV_ASSERT(config->ctuMode != ADC_CTU_MODE_TRIGGER);
#endif /* FEATURE_ADC_HAS_CTU */
#endif /* (FEATURE_ADC_HAS_CTU_TRIGGER_MODE == 0) */

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    static const clock_names_t adc_clocks[ADC_INSTANCE_COUNT] = ADC_CLOCKS;
    uint32_t adc_freq = 0U;
    status_t clk_status = CLOCK_SYS_GetFreq(adc_clocks[instance], &adc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    if (config->clkSelect == ADC_CLK_HALF_BUS) {
        adc_freq /= 2U; /* half bus speed during normal operation */
    }
    DEV_ASSERT(adc_freq > 0U);
    DEV_ASSERT(adc_freq <= ADC_CLOCK_FREQ_MAX_RUNTIME);
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t mcr = 0U;
    mcr |= ADC_MCR_MODE(config->convMode);
#if FEATURE_ADC_HAS_CLKSEL_EXTENDED
    switch (config->clkSelect)
    {
        case ADC_CLK_HALF_BUS:
            /* Half bus clock when MCR[ADCLKSE] = 0 and MCR[ADCLKDIV] = 0 */
            break;
        case ADC_CLK_FULL_BUS:
            /* Full bus clock when MCR[ADCLKSE] = 1 */
            mcr |= ADC_MCR_ADCLKSE(1u);
            break;
        case ADC_CLK_QUARTER_BUS:
            /* Full bus clock when MCR[ADCLKSE] = 0 and MCR[ADCLKDIV] = 1 */
            mcr |= ADC_MCR_ADCLKDIV(1u);
            break;
        default:
            /* no-op */
            break;
    }
#else
    mcr |= ADC_MCR_ADCLKSEL(config->clkSelect);
#endif
#ifdef ADC_MCR_REFSEL
    mcr |= ADC_MCR_REFSEL(config->refSelect);
#endif
    mcr |= ADC_MCR_ACKO((uint32_t)config->autoClockOff);
    mcr |= ADC_MCR_OWREN((uint32_t)config->overwriteEnable);
    mcr |= ADC_MCR_WLSIDE(config->dataAlign);

#if FEATURE_ADC_HAS_CTU
    switch(config->ctuMode)
    {
        case ADC_CTU_MODE_CONTROL:
            /* Already in CTU Control Mode CTU_MODE = 0 */
            mcr |= ADC_MCR_CTUEN(1U); /* Enable CTU */
            break;
#if FEATURE_ADC_HAS_CTU_TRIGGER_MODE
        case ADC_CTU_MODE_TRIGGER:
            mcr |= ADC_MCR_CTU_MODE(1U); /* Set CTU to Trigger Mode CTU_MODE = 1 */
            mcr |= ADC_MCR_CTUEN(1U); /* Enable CTU */
            break;
#endif
        case ADC_CTU_MODE_DISABLED:
            /* CTU is disabled (CTUEN = 0 and CTU_MODE = 0) */
            /* Pass through */
        default:
            /* no-op */
            break;
    }
#endif /* FEATURE_ADC_HAS_CTU */
#if FEATURE_ADC_HAS_INJ_TRIGGER_SEL
    switch(config->injectedEdge)
    {
        case ADC_INJECTED_EDGE_FALLING:
            /* Already on falling edge JEDGE = 0 */
            mcr |= ADC_MCR_JTRGEN(1U); /* enable Injected trigger */
            break;
        case ADC_INJECTED_EDGE_RISING:
            mcr |= ADC_MCR_JEDGE(1U); /* set to rising edge JEDGE = 1 */
            mcr |= ADC_MCR_JTRGEN(1U); /* enable Injected trigger */
            break;
        case ADC_INJECTED_EDGE_DISABLED:
            /* Injected trigger disabled (JTRGEN = 0 and JEDGE = 0) */
            /* Pass through */
        default:
            /* no-op */
            break;
    }
#endif /* FEATURE_ADC_HAS_INJ_TRIGGER_SEL */

#if FEATURE_ADC_HAS_EXT_TRIGGER
    switch(config->extTrigger)
    {
	    case ADC_EXT_TRIGGER_FALLING:
			mcr |= ADC_MCR_EDGE(0U);  /* set to falling edge EDGE = 0 */
		    mcr |= ADC_MCR_TRGEN(1U); /* enable External trigger */
		    break;
	    case ADC_EXT_TRIGGER_RISING:
		    mcr |= ADC_MCR_EDGE(1U);  /* set to rising edge EDGE = 1 */
		    mcr |= ADC_MCR_TRGEN(1U); /* enable external trigger */
		    break;
	    case ADC_EXT_TRIGGER_DISABLED:
		    /* External trigger disabled (TRGEN = 0 and EDGE = 0) */
		    /* Pass through */
	    default:
		   /* no-op */
		   break;
	}
#endif

    ADC_Powerdown(base);
    base->MCR = mcr;
    ADC_Powerup(base);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    if (ADC_INST_HAS_CTRn(instance, ADC_GROUP_0))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->CTR[ADC_GROUP_0] = ADC_CTR_INPSAMP(config->sampleTime0);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    if (ADC_INST_HAS_CTRn(instance, ADC_GROUP_1))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->CTR[ADC_GROUP_1] = ADC_CTR_INPSAMP(config->sampleTime1);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
#if ((ADC_CIMR_COUNT > 2U) || (ADC_CTR_COUNT > 2U))
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    if (ADC_INST_HAS_CTRn(instance, ADC_GROUP_2))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->CTR[ADC_GROUP_2] = ADC_CTR_INPSAMP(config->sampleTime2);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
#endif /* ((ADC_CIMR_COUNT > 2U) || (ADC_CTR_COUNT > 2U)) */

    base->PDEDR = ADC_PDEDR_PDED(config->powerDownDelay);

#ifdef ADC_DSDR_DSD
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_DSDR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        base->DSDR = ADC_DSDR_DSD(config->decodeDelay);
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
#endif /* ADC_DSDR_DSD */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ChainConfig
* Description   : This function configures the ADC chain feature with the options
*  provided in the configuration structure.
*
* Implements    : ADC_DRV_ChainConfig_Activity
* END**************************************************************************/
void ADC_DRV_ChainConfig(const uint32_t instance,
                         const adc_chain_config_t * const config)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    /* Enabling unavailable channels is forbidden */
    DEV_ASSERT(  (config->chanMaskNormal[0] & (~ADC_CHAN_BITMAP[instance][0])) == 0u);
    DEV_ASSERT(  (config->chanMaskNormal[1] & (~ADC_CHAN_BITMAP[instance][1])) == 0u);
    DEV_ASSERT(  (config->chanMaskNormal[2] & (~ADC_CHAN_BITMAP[instance][2])) == 0u);
    DEV_ASSERT((config->chanMaskInjected[0] & (~ADC_CHAN_BITMAP[instance][0])) == 0u);
    DEV_ASSERT((config->chanMaskInjected[1] & (~ADC_CHAN_BITMAP[instance][1])) == 0u);
    DEV_ASSERT((config->chanMaskInjected[2] & (~ADC_CHAN_BITMAP[instance][2])) == 0u);
    DEV_ASSERT(   (config->interruptMask[0] & (~ADC_CHAN_BITMAP[instance][0])) == 0u);
    DEV_ASSERT(   (config->interruptMask[1] & (~ADC_CHAN_BITMAP[instance][1])) == 0u);
    DEV_ASSERT(   (config->interruptMask[2] & (~ADC_CHAN_BITMAP[instance][2])) == 0u);
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */

    ADC_Type * const base = s_adcBase[instance];

    uint8_t k;
    for (k = 0U; k < ADC_NCMR_COUNT; k++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_NCMRn(instance, k))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->NCMR[k] = config->chanMaskNormal[k];
    }

    for (k = 0U; k < ADC_JCMR_COUNT; k++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_JCMRn(instance, k))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->JCMR[k] = config->chanMaskInjected[k];
    }

    for (k = 0U; k < ADC_CIMR_COUNT; k++)
    {
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (!ADC_INST_HAS_CIMRn(instance, k))
        {
            continue; /* skip register if it's not available */
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
        base->CIMR[k] = config->interruptMask[k];
    }

}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_Reset
* Description   : This function writes all the internal ADC registers with
*  their Reference Manual reset values.
*
* Implements    : ADC_DRV_Reset_Activity
* END**************************************************************************/
void ADC_DRV_Reset(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    adc_conv_config_t default_config;
    ADC_DRV_GetDefaultConfigConverter(&default_config);
    ADC_DRV_ConfigConverter(instance, &default_config);
    ADC_ResetWdog(instance);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableChannel
* Description   : This function enables a channel in a conversion chain (
*  ADC_CONV_CHAIN_NORMAL or ADC_CONV_CHAIN_INJECTED).
*
* Implements    : ADC_DRV_EnableChannel_Activity
* END**************************************************************************/
void ADC_DRV_EnableChannel(const uint32_t instance,
                           const adc_conv_chain_t convChain,
                           const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    switch (convChain)
    {
        case ADC_CONV_CHAIN_NORMAL:
            REG_BIT_SET32(&(base->NCMR[vectAdr]), (1UL << vectBit));
            break;
        case ADC_CONV_CHAIN_INJECTED:
            REG_BIT_SET32(&(base->JCMR[vectAdr]), (1UL << vectBit));
            break;
        default:
            DEV_ASSERT(false);
            break;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableChannel
* Description   : This function disables a channel from a conversion chain (ADC_CONV_CHAIN_NORMAL
*  or ADC_CONV_CHAIN_INJECTED).
*
* Implements    : ADC_DRV_DisableChannel_Activity
* END**************************************************************************/
void ADC_DRV_DisableChannel(const uint32_t instance,
                            const adc_conv_chain_t convChain,
                            const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif
    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    switch (convChain)
    {
        case ADC_CONV_CHAIN_NORMAL:
            REG_BIT_CLEAR32(&(base->NCMR[vectAdr]), (1UL << vectBit));
            break;
        case ADC_CONV_CHAIN_INJECTED:
            REG_BIT_CLEAR32(&(base->JCMR[vectAdr]), (1UL << vectBit));
            break;
        default:
            DEV_ASSERT(false);
            break;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_StartConversion
* Description   : This function starts a conversion chain (ADC_CONV_CHAIN_NORMAL
*  or ADC_CONV_CHAIN_INJECTED).
*
* Implements    : ADC_DRV_StartConversion_Activity
* END**************************************************************************/
void ADC_DRV_StartConversion(const uint32_t instance,
                             const adc_conv_chain_t convChain)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];
    switch (convChain)
    {
        case ADC_CONV_CHAIN_NORMAL:
            REG_BIT_SET32(&(base->MCR), ADC_MCR_NSTART(1U));
            break;
        case ADC_CONV_CHAIN_INJECTED:
            REG_BIT_SET32(&(base->MCR), ADC_MCR_JSTART(1U));
            break;
        default:
            DEV_ASSERT(false);
            break;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetStatusFlags
* Description   : This function returns the status flags of the ADC.
*
* Implements    : ADC_DRV_GetStatusFlags_Activity
* END**************************************************************************/
uint32_t ADC_DRV_GetStatusFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    const ADC_Type * const base = s_adcBase[instance];
    uint32_t msr = base->MSR;
    uint32_t isr = base->ISR;
    uint32_t flags = 0U;

    flags |= ((msr & ADC_MSR_CALIBRTD_MASK) != 0U) ? ADC_FLAG_CALIBRATED : 0U;
    flags |= ((msr & ADC_MSR_NSTART_MASK) != 0U) ? ADC_FLAG_NORMAL_STARTED : 0U;
    flags |= ((msr & ADC_MSR_JABORT_MASK) != 0U) ? ADC_FLAG_INJECTED_ABORTED : 0U;
    flags |= ((msr & ADC_MSR_JSTART_MASK) != 0U) ? ADC_FLAG_INJECTED_STARTED : 0U;
#if FEATURE_ADC_HAS_CTU
    flags |= ((msr & ADC_MSR_CTUSTART_MASK) != 0U) ? ADC_FLAG_CTU_STARTED : 0U;
#endif
    flags |= ((msr & ADC_MSR_ACKO_MASK) != 0U) ? ADC_FLAG_AUTOCLOCKOFF : 0U;
    flags |= ((isr & ADC_ISR_EOC_MASK) != 0U) ? ADC_FLAG_NORMAL_EOC : 0U;
    flags |= ((isr & ADC_ISR_ECH_MASK) != 0U) ? ADC_FLAG_NORMAL_ENDCHAIN : 0U;
    flags |= ((isr & ADC_ISR_JEOC_MASK) != 0U) ? ADC_FLAG_INJECTED_EOC : 0U;
    flags |= ((isr & ADC_ISR_JECH_MASK) != 0U) ? ADC_FLAG_INJECTED_ENDCHAIN : 0U;
#if FEATURE_ADC_HAS_CTU
    flags |= ((isr & ADC_ISR_EOCTU_MASK) != 0U) ? ADC_FLAG_CTU_EOC : 0U;
#endif

    return flags;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ClearStatusFlags
* Description   : This function clears the status flags that are set to '1' in
*  the mask.
*
* Implements    : ADC_DRV_ClearStatusFlags_Activity
* END**************************************************************************/
void ADC_DRV_ClearStatusFlags(const uint32_t instance,
                              const uint32_t mask)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];
    uint32_t isr_flags = 0U;
    isr_flags |= ((mask & ADC_FLAG_NORMAL_EOC) != 0U) ? ADC_ISR_EOC(1U) : 0U;
    isr_flags |= ((mask & ADC_FLAG_NORMAL_ENDCHAIN) != 0U) ? ADC_ISR_ECH(1U) : 0U;
    isr_flags |= ((mask & ADC_FLAG_INJECTED_EOC) != 0U) ? ADC_ISR_JEOC(1U) : 0U;
    isr_flags |= ((mask & ADC_FLAG_INJECTED_ENDCHAIN) != 0U) ? ADC_ISR_JECH(1U) : 0U;
#if FEATURE_ADC_HAS_CTU
    isr_flags |= ((mask & ADC_FLAG_CTU_EOC) != 0U) ? ADC_ISR_EOCTU(1U) : 0U;
#endif

    /* Write-1-to-clear bits in ISR register */
    base->ISR = isr_flags;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetConvResultsToArray
* Description   : This function reads the conversion result values for a conversion chain
*  (ADC_CONV_CHAIN_NORMAL, ADC_CONV_CHAIN_INJECTED or ADC_CONV_CHAIN_CTU).
*
* Implements    : ADC_DRV_GetConvResultsToArray_Activity
* END**************************************************************************/
uint32_t ADC_DRV_GetConvResultsToArray(const uint32_t instance,
                                       const adc_conv_chain_t convChain,
                                       uint16_t * const results,
                                       const uint32_t length)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    return ADC_GetConvResults(instance, convChain, results, NULL, length);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetConvInfoToArray
* Description   : This function gets the conversion results for the selected
*  Conversion Chain. It follows the same algorithm as ADC_DRV_GetConvResultsToArray,
*  but will copy some extra information to the output.
*
* Implements    : ADC_DRV_GetConvInfoToArray_Activity
* END**************************************************************************/
uint32_t ADC_DRV_GetConvInfoToArray(const uint32_t instance,
                                    const adc_conv_chain_t convChain,
                                    adc_chan_result_t* const results,
                                    const uint32_t length)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    return ADC_GetConvResults(instance, convChain, NULL, results, length);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DoCalibration
* Description   : This functions executes a calibration sequence. It is recommended
*  to run this sequence before using the ADC converter. The maximum clock frequency
*  for the calibration is 40 MHz, this function sets the ADCLKSEL bit resulting in
*  a maximum input clock frequency of 80 MHz.
*
* Implements    : ADC_DRV_DoCalibration_Activity
* END**************************************************************************/
status_t ADC_DRV_DoCalibration(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    clock_names_t adc_clocks[ADC_INSTANCE_COUNT] = ADC_CLOCKS;
    uint32_t adc_freq = 0U;
    status_t clk_status = CLOCK_SYS_GetFreq(adc_clocks[instance], &adc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    adc_freq /= (uint32_t) 2u; /* half bus speed will be used during calibration */
    DEV_ASSERT(adc_freq > 0U);
    DEV_ASSERT(adc_freq <= ADC_CLOCK_FREQ_MAX_CALIBRATION);
#endif

    status_t ret = STATUS_SUCCESS;

    ADC_Type * const base = s_adcBase[instance];
#if FEATURE_ADC_HAS_CALIBRATION_ALT
    /* Note: this mode implies FEATURE_ADC_HAS_CLKSEL_EXTENDED == 1 */
    ADC_Powerdown(base);
    /* Reset CLKSEL to 0x00 (will set to half bus speed) */
    uint32_t mcr = base->MCR; /* save the current state of MCR to restore ADCLKSEL */
    REG_BIT_CLEAR32(&(base->MCR), ADC_MCR_ADCLKSE_MASK | ADC_MCR_ADCLKDIV_MASK);
    ADC_Powerup(base);

    /* clear the bits and set to calibration values */
    base->MCR &= ~(ADC_MCR_TSAMP_MASK | ADC_MCR_NRSMPL_MASK | ADC_MCR_AVGEN_MASK);
    base->MCR |= ADC_MCR_NRSMPL(0x03u) | ADC_MCR_AVGEN(1U);
    /* start calibration */
    REG_BIT_SET32(&(base->MCR), ADC_MCR_CALSTART(1U));
    while ((base->MSR & ADC_MSR_CALBUSY_MASK) != 0U)
    {
        /* Wait for calibration to finish */
    }

    /* If the calibration failed, return error */
    if ((base->MSR & ADC_MSR_CALFAIL_MASK) != 0U)
    {
        ret = STATUS_ERROR;
    }

    /* restore the state of ADCLKSEL */
    REG_BIT_SET32(&(base->MCR), (uint32_t) (mcr & (ADC_MCR_ADCLKSE_MASK | ADC_MCR_ADCLKDIV_MASK)));

#else
    /* Note: this mode implies FEATURE_ADC_HAS_CLKSEL_EXTENDED == 0 */
    ADC_Powerdown(base);
    /* Reset CLKSEL to 0x00 (will set to half bus speed) */
    uint32_t mcr = base->MCR; /* save the current state of MCR to restore ADCLKSEL */
    REG_BIT_CLEAR32(&(base->MCR), ADC_MCR_ADCLKSEL_MASK);
    ADC_Powerup(base);
    uint32_t calbistreg = base->CALBISTREG;
    /* clear the bits and set to calibration values */
    calbistreg &= ~(ADC_CALBISTREG_TSAMP_MASK | ADC_CALBISTREG_NR_SMPL_MASK | ADC_CALBISTREG_AVG_EN_MASK | ADC_CALBISTREG_TEST_EN_MASK);
    calbistreg |= ADC_CALBISTREG_NR_SMPL(0x03u) | ADC_CALBISTREG_AVG_EN(1U);
    base->CALBISTREG = calbistreg;
    /* clear the calibration failed before a new calibration */
    REG_BIT_SET32(&(base->CALBISTREG), ADC_CALBISTREG_TEST_FAIL_MASK);
    /* start calibration */
    REG_BIT_SET32(&(base->CALBISTREG), ADC_CALBISTREG_TEST_EN(1U));
    while ((base->CALBISTREG & ADC_CALBISTREG_C_T_BUSY_MASK) != 0U)
    {
        /* Wait for calibration to finish */
    }

    /* If the calibration failed, return error */
    if ((base->CALBISTREG & ADC_CALBISTREG_TEST_FAIL_MASK) != 0U)
    {
        ret = STATUS_ERROR;
    }

    /* restore the state of ADCLKSEL */
    REG_BIT_SET32(&(base->MCR), (uint32_t) (mcr & ADC_MCR_ADCLKSEL_MASK));
#endif
    return ret;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_Powerup
* Description   : This function enables the ADC module (by clearing the Powerdown
*  bit).
*
* Implements    : ADC_DRV_Powerup_Activity
* END**************************************************************************/
void ADC_DRV_Powerup(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];
    ADC_Powerup(base);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_Powerdown
* Description   : This function disables the ADC module (by setting the Powerdown
*  bit).
*
* Implements    : ADC_DRV_Powerdown_Activity
* END**************************************************************************/
void ADC_DRV_Powerdown(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];
    ADC_Powerdown(base);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableInterrupts
* Description   : This function enables the ADC interrupts set to '1' in the
*  mask parameter.
*
* Implements    : ADC_DRV_EnableInterrupts_Activity
* END**************************************************************************/
void ADC_DRV_EnableInterrupts(const uint32_t instance,
                              const uint32_t interruptMask)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    uint32_t imr_flags = 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_NORMAL_EOC) != 0U) ? ADC_IMR_MSKEOC(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_NORMAL_ENDCHAIN) != 0U) ? ADC_IMR_MSKECH(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_INJECTED_EOC) != 0U) ? ADC_IMR_MSKJEOC(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_INJECTED_ENDCHAIN) != 0U) ? ADC_IMR_MSKJECH(1U) : 0U;
#if FEATURE_ADC_HAS_CTU
    imr_flags |= ((interruptMask & ADC_FLAG_CTU_EOC) != 0U) ? ADC_IMR_MSKEOCTU(1U) : 0U;
#endif
    REG_BIT_SET32(&(base->IMR), imr_flags);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableInterrupts
* Description   : This function disables the ADC interrupts set to '1' in the
*  mask parameter.
*
* Implements    : ADC_DRV_DisableInterrupts_Activity
* END**************************************************************************/
void ADC_DRV_DisableInterrupts(const uint32_t instance,
                               const uint32_t interruptMask)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    uint32_t imr_flags = 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_NORMAL_EOC) != 0U) ? ADC_IMR_MSKEOC(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_NORMAL_ENDCHAIN) != 0U) ? ADC_IMR_MSKECH(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_INJECTED_EOC) != 0U) ? ADC_IMR_MSKJEOC(1U) : 0U;
    imr_flags |= ((interruptMask & ADC_FLAG_INJECTED_ENDCHAIN) != 0U) ? ADC_IMR_MSKJECH(1U) : 0U;
#if FEATURE_ADC_HAS_CTU
    imr_flags |= ((interruptMask & ADC_FLAG_CTU_EOC) != 0U) ? ADC_IMR_MSKEOCTU(1U) : 0U;
#endif
    REG_BIT_CLEAR32(&(base->IMR), imr_flags);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableChannelInterrupt
* Description   : This function enables End-of-Conversion interrupt generation for
*  a single channel.
*
* Implements    : ADC_DRV_EnableChannelInterrupt_Activity
* END**************************************************************************/
void ADC_DRV_EnableChannelInterrupt(const uint32_t instance,
                                    const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    base->CIMR[vectAdr] |= 1UL << vectBit;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableChannelInterrupt
* Description   : This function disables End-of-Conversion interrupt generation for
*  a single channel.
*
* Implements    : ADC_DRV_DisableChannelInterrupt_Activity
* END**************************************************************************/
void ADC_DRV_DisableChannelInterrupt(const uint32_t instance,
                                     const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    base->CIMR[vectAdr] &= ~(1UL << vectBit);
}


/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_SetPresamplingSource
* Description   : This function configures the Presampling Source for a channel group (0-31, 32-63, 64 -95)
*
* Implements    : ADC_DRV_SetPresamplingSource_Activity
* END**************************************************************************/
void ADC_DRV_SetPresamplingSource(const uint32_t instance,
                                  const adc_chan_group_t chanGroup,
                                  const adc_presampling_source_t presampleSource)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_PSCR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        ADC_Type * const base = s_adcBase[instance];

        uint32_t pscr = base->PSCR;
        switch (chanGroup){
#ifdef ADC_PSCR_PREVAL0
            case ADC_CHAN_GROUP_0:
                pscr &= ~(ADC_PSCR_PREVAL0_MASK);
                pscr |= ADC_PSCR_PREVAL0((uint32_t)presampleSource);
                break;
#endif
#ifdef ADC_PSCR_PREVAL1
            case ADC_CHAN_GROUP_1:
                pscr &= ~(ADC_PSCR_PREVAL1_MASK);
                pscr |= ADC_PSCR_PREVAL1((uint32_t)presampleSource);
                break;
#endif
#ifdef ADC_PSCR_PREVAL2
            case ADC_CHAN_GROUP_2:
                pscr &= ~(ADC_PSCR_PREVAL2_MASK);
                pscr |= ADC_PSCR_PREVAL2((uint32_t)presampleSource);
                break;
#endif
            default:
                /* Not supported */
                DEV_ASSERT(false);
                break;
        }
        base->PSCR = pscr;
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableChannelPresampling
* Description   : This function enables the Presampling on one channel of the ADC.
*
* Implements    : ADC_DRV_EnableChannelPresampling_Activity
* END**************************************************************************/
void ADC_DRV_EnableChannelPresampling(const uint32_t instance,
                                      const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_PSCR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        ADC_Type * const base = s_adcBase[instance];

        uint32_t vectAdr = CHAN_2_VECT(chnIdx);
        uint32_t vectBit = CHAN_2_BIT(chnIdx);

        base->PSR[vectAdr] |= 1UL << vectBit;

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableChannelPresampling
* Description   : This function disables the Presampling on one channel of the ADC.
*
* Implements    : ADC_DRV_DisableChannelPresampling_Activity
* END**************************************************************************/
void ADC_DRV_DisableChannelPresampling(const uint32_t instance,
                                       const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_PSCR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        ADC_Type * const base = s_adcBase[instance];

        uint32_t vectAdr = CHAN_2_VECT(chnIdx);
        uint32_t vectBit = CHAN_2_BIT(chnIdx);

        base->PSR[vectAdr] &= ~(1UL << vectBit);
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnablePresampleConversion
* Description   : This function enables bypass of the Sampling Phase, resulting in a conversion
* of the presampled data. This is available only for channels that have presampling
* enabled.
*
* Implements    : ADC_DRV_EnablePresampleConversion_Activity
* END**************************************************************************/
void ADC_DRV_EnablePresampleConversion(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_PSCR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        ADC_Type * const base = s_adcBase[instance];

        REG_BIT_SET32(&(base->PSCR), ADC_PSCR_PRECONV(1U));
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisablePresampleConversion
* Description   : This function disables Sampling Phase bypass.
*
* Implements    : ADC_DRV_DisablePresampleConversion_Activity
* END**************************************************************************/
void ADC_DRV_DisablePresampleConversion(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    if (ADC_INST_HAS_PSCR(instance))
    {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
        ADC_Type * const base = s_adcBase[instance];

        REG_BIT_CLEAR32(&(base->PSCR), ADC_PSCR_PRECONV(1U));
#if FEATURE_ADC_BAD_ACCESS_PROT_FEATURE
    }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_FEATURE */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableDma
* Description   : This function enables requests to DMA from ADC
*
* Implements    : ADC_DRV_EnableDma_Activity
* END**************************************************************************/
void ADC_DRV_EnableDma(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    REG_BIT_SET32(&(base->DMAE), ADC_DMAE_DMAEN(1U));
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableDma
* Description   : This function disables requests to DMA from ADC
*
* Implements    : ADC_DRV_DisableDma_Activity
* END**************************************************************************/
void ADC_DRV_DisableDma(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    REG_BIT_CLEAR32(&(base->DMAE), ADC_DMAE_DMAEN(1U));
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableChannelDma
* Description   : This function enables DMA requests triggered by End of Conversion event from
*  a selected channel.
*
* Implements    : ADC_DRV_EnableChannelDma_Activity
* END**************************************************************************/
void ADC_DRV_EnableChannelDma(const uint32_t instance,
                              const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    base->DMAR[vectAdr] |= 1UL << vectBit;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableChannelDma
* Description   : This function disables DMA requests triggered by End of Conversion event from
*  a selected channel.
*
* Implements    : ADC_DRV_DisableChannelDma_Activity
* END**************************************************************************/
void ADC_DRV_DisableChannelDma(const uint32_t instance,
                               const uint32_t chnIdx)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
    uint32_t vectBit = CHAN_2_BIT(chnIdx);

    base->DMAR[vectAdr] &= ~(1UL << vectBit);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_SetDmaClearSource
* Description   : This function selects the DMA Request Flag Clear Source.
*
* Implements    : ADC_DRV_SetDmaClearSource_Activity
* END**************************************************************************/
void ADC_DRV_SetDmaClearSource(const uint32_t instance,
                               const adc_dma_clear_source_t dmaClear)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);

    ADC_Type * const base = s_adcBase[instance];

    uint32_t dmae = base->DMAE;
    dmae &= ~(ADC_DMAE_DCLR_MASK);
    dmae |= ADC_DMAE_DCLR((uint32_t)dmaClear);
    base->DMAE = dmae;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_AbortConversion
* Description   : This function aborts an ongoing conversion.
*
* Implements    : ADC_DRV_AbortConversion_Activity
* END**************************************************************************/
void ADC_DRV_AbortConversion(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    ADC_Type * const base = s_adcBase[instance];
    base->MCR |= ADC_MCR_ABORT(1U);

}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_AbortChain
* Description   : This function aborts an ongoing chain of conversions.
*
* Implements    : ADC_DRV_AbortChain_Activity
* END**************************************************************************/
void ADC_DRV_AbortChain(const uint32_t instance)
{
    DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    ADC_Type * const base = s_adcBase[instance];

    base->MCR |= ADC_MCR_ABORT_CHAIN(1U);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ConfigWdg
* Description   : This function configures the analog watchdog.
*
* Implements    : ADC_DRV_ConfigWdg_Activity
* * END**************************************************************************/
void ADC_DRV_ConfigWdg(const uint32_t instance,
                       const uint8_t numThresholds,
                       const adc_wdg_threshold_values_t * const thresholdValuesArray,
                       const uint8_t numChannels,
                       const adc_wdg_chan_thresholds_t * const  chanThresholdsArray)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
	DEV_ASSERT(thresholdValuesArray != NULL);
	DEV_ASSERT(chanThresholdsArray != NULL);
	DEV_ASSERT(numThresholds < ADC_THRHLR_COUNT);
	DEV_ASSERT(numChannels < ADC_CDR_COUNT);
#ifdef ADC_THRHLR_PER_INSTANCE_COUNT
    DEV_ASSERT(numThresholds < s_adcThrhlr_count[instance]);
#endif

	uint8_t index;
	ADC_Type * const base = s_adcBase[instance];

    uint32_t wtimr = base->WTIMR;
	for (index = 0; index < numThresholds; index++)
	{
		ADC_WriteThresholds(base, index,
                            thresholdValuesArray[index].highThreshold,
                            thresholdValuesArray[index].lowThreshold);
        if (thresholdValuesArray[index].highThresholdIntEn)
        {
            wtimr |= ADC_WDOG_REG_MASK_HIGH(index);
        }
        else
        {
            wtimr &= ~ADC_WDOG_REG_MASK_HIGH(index);
        }
        if (thresholdValuesArray[index].lowThresholdIntEn)
        {
            wtimr |= ADC_WDOG_REG_MASK_LOW(index);
        }
        else
        {
            wtimr &= ~ADC_WDOG_REG_MASK_LOW(index);
        }
	}
    base->WTIMR = wtimr;

	for (index = 0; index < numChannels; index++)
	{
		/* compute which of the 11 registers of 32 bit should be written for a specific channel*/
		uint32_t registerNumber = (uint32_t)chanThresholdsArray[index].chnIdx / 8U;
		/* compute which of the 8 fields(4 bits each field) of the register have to be written for a specific channel*/
		uint32_t fieldPosition = (uint32_t)chanThresholdsArray[index].chnIdx % 8U;

#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        if (ADC_INST_HAS_CWSELRn(instance, registerNumber))
        {
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */
            ADC_WriteChannelMapping(base, registerNumber, fieldPosition, chanThresholdsArray[index].registerIdx);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
        }
#endif /* FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL */

		ADC_DRV_EnableChannelWdg(instance, (uint32_t)chanThresholdsArray[index].chnIdx);
	}
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_SetWdgThreshold
* Description   : This function sets the watchdog thresholds for a certain register.
*
* Implements    : ADC_DRV_SetWdgThreshold_Activity
* END**************************************************************************/
void ADC_DRV_SetWdgThreshold(const uint32_t instance,
                             const uint8_t registerIdx,
                             const adc_wdg_threshold_values_t * const thresholdValues)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(thresholdValues != NULL);
    DEV_ASSERT(registerIdx < ADC_THRHLR_COUNT);
#ifdef ADC_THRHLR_PER_INSTANCE_COUNT
    DEV_ASSERT(registerIdx < s_adcThrhlr_count[instance]);
#endif

    ADC_Type * const base = s_adcBase[instance];

    ADC_WriteThresholds(base, registerIdx,
                        thresholdValues->highThreshold,
                        thresholdValues->lowThreshold);

    uint32_t wtimr = base->WTIMR;
    if (thresholdValues->highThresholdIntEn)
    {
        wtimr |= ADC_WDOG_REG_MASK_HIGH(registerIdx);
    }
    else
    {
        wtimr &= ~ADC_WDOG_REG_MASK_HIGH(registerIdx);
    }
    if (thresholdValues->lowThresholdIntEn)
    {
        wtimr |= ADC_WDOG_REG_MASK_LOW(registerIdx);
    }
    else
    {
        wtimr &= ~ADC_WDOG_REG_MASK_LOW(registerIdx);
    }
    base->WTIMR = wtimr;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_SetWdgChannelMapping
* Description   : This function set the threshold register used for a specific channel.
*
* Implements    : ADC_DRV_SetWdgChannelMapping_Activity
* END**************************************************************************/
void ADC_DRV_SetWdgChannelMapping(const uint32_t instance,
		                          const uint32_t chnIdx,
                                  const uint32_t registerIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
	DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
	DEV_ASSERT(registerIdx < ADC_THRHLR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

	ADC_Type * const base = s_adcBase[instance];

	/* find out which of the 11 registers of 32 bit should be written for a specific channel*/
	uint32_t registerNumber = chnIdx / 8U;
	/* find out which of the 8 fields(4 bits each field) of the register have to be written for a specific channel*/
	uint32_t fieldPosition =  chnIdx % 8U;

	ADC_WriteChannelMapping(base, registerNumber, fieldPosition, registerIdx);
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_EnableChannelWdg
* Description   : This function enable analog watchdog for a specific channel.
*
* Implements    : ADC_DRV_EnableChannelWdg_Activity
* END**************************************************************************/
void ADC_DRV_EnableChannelWdg(const uint32_t instance,
                              const uint32_t chnIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];
    uint32_t registerNumber = CHAN_2_VECT(chnIdx);
    uint32_t bitPosition = CHAN_2_BIT(chnIdx);

    base->CWENR[registerNumber] |= ((uint32_t)((1UL) << bitPosition));
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_DisableChannelWdg
* Description   : This function disable analog watchdog for a specific channel.
*
* Implements    : ADC_DRV_DisableChannelWdg_Activity
* END**************************************************************************/
void ADC_DRV_DisableChannelWdg(const uint32_t instance,
                               const uint32_t chnIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
	DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

	ADC_Type * const base = s_adcBase[instance];
	uint32_t registerNumber = CHAN_2_VECT(chnIdx);
	uint32_t bitPosition = CHAN_2_BIT(chnIdx);

	base->CWENR[registerNumber] &= ~((uint32_t)((1UL) << bitPosition));
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_WdgIsChanOutOfRange
* Description   : This function returns status of a channel.If the function returns
*   TRUE, the value converted is out of the range that was set for that channel.
*
* Implements    : ADC_DRV_WdgIsChanOutOfRange_Activity
* END**************************************************************************/
bool ADC_DRV_WdgIsChanOutOfRange(const uint32_t instance,
                                 const uint32_t chnIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    bool returnValue = false;
    ADC_Type const * const base = s_adcBase[instance];

    uint32_t registerNumber = CHAN_2_VECT(chnIdx);
    uint32_t bitPosition = CHAN_2_BIT(chnIdx);
    uint32_t mask = 1U;

    mask <<= bitPosition;
    if((base->AWORR[registerNumber] & mask) != 0U)
    {
    	returnValue = true;
    }

    return returnValue;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ClearWdgOutOfRangeFlag
* Description   : This function clears the flag set when the converted value is
* out of range
*
* Implements    : ADC_DRV_ClearWdgOutOfRangeFlag_Activity
* END**************************************************************************/
void ADC_DRV_ClearWdgOutOfRangeFlag(const uint32_t instance,
                                    const uint32_t chnIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
	DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

	ADC_Type * const base = s_adcBase[instance];

	uint32_t registerNumber = CHAN_2_VECT(chnIdx);
	uint32_t bitPosition = CHAN_2_BIT(chnIdx);
	uint32_t mask = 1U;

	mask <<= bitPosition;
	base->AWORR[registerNumber] = mask; /* w1c bit */
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetConvResult
* Description   : This function returns the result of the conversion for
* a single channel
*
* Implements    : ADC_DRV_GetConvResult_Activity
* END**************************************************************************/
uint16_t ADC_DRV_GetConvResult(const uint32_t instance,
                               const uint32_t chnIdx)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
	DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

	uint16_t result = 0U;
	ADC_Type * const base = s_adcBase[instance];

    uint32_t vectAdr = CHAN_2_VECT(chnIdx);
	uint32_t vectBit = CHAN_2_BIT(chnIdx);
    /* check if the the conversion is complete */
    if ((base->CEOCFR[vectAdr] & ((uint32_t)1U << vectBit)) != 0U)
    {
        /* check if the the conversion data is valid */
        uint32_t cdr = base->CDR[chnIdx];
        if (((cdr & ADC_CDR_VALID_MASK) != 0U))
        {
            /* the data is correct, store the result in and clear the flag */
            result = (uint16_t)(cdr & ADC_CDR_CDATA_MASK) >> ADC_CDR_CDATA_SHIFT;
            base->CEOCFR[vectAdr] = (1UL) << vectBit; /* w1c bit */
        }
    }

	return result;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetConvInfo
* Description   : This function returns the result and the status of
* the conversion for a single channel
*
* Implements    : ADC_DRV_GetConvInfo_Activity
* END**************************************************************************/
void ADC_DRV_GetConvInfo (const uint32_t instance,
                          const uint32_t  chnIdx,
                          adc_chan_result_t* const result)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    DEV_ASSERT(chnIdx < ADC_CDR_COUNT);
    DEV_ASSERT(result != NULL);
#if FEATURE_ADC_BAD_ACCESS_PROT_CHANNEL
    DEV_ASSERT(ADC_INST_HAS_CHANn(instance, chnIdx));
#endif

    ADC_Type * const base = s_adcBase[instance];

	uint32_t vectAdr = CHAN_2_VECT(chnIdx);
	uint32_t vectBit = CHAN_2_BIT(chnIdx);

    /* check if the the conversion is complete */
    if ((base->CEOCFR[vectAdr] & (1UL << vectBit)) != 0UL)
    {
        /* check if the the conversion data is valid */
        uint32_t cdr = base->CDR[chnIdx];
        if (((cdr & ADC_CDR_VALID_MASK) != 0U))
        {
            /* the data is correct, store the result in and clear the flag */
            result->chnIdx = (uint8_t)chnIdx;
            result->valid = true;
            result->overWritten = (((cdr & ADC_CDR_OVERW_MASK) >> ADC_CDR_OVERW_SHIFT) == 1U);
            result->cdata = (uint16_t)(cdr & ADC_CDR_CDATA_MASK) >> ADC_CDR_CDATA_SHIFT;

            base->CEOCFR[vectAdr] = (1UL) << vectBit; /* w1c bit */
        }
    }
    else
    {
        result->chnIdx = (uint8_t)chnIdx;
        result->valid = false;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_GetWdgThresholdFlags
* Description   : This function returns the status of the watchdog threshold
* interrupts flags (WTISR) as a single bit mask. The macros ADC_WDOG_REG_MASK_HIGH
* and ADC_WDOG_REG_MASK_LOW can be used to decode the data in the bit mask.
*
* Implements    : ADC_DRV_GetWdgThresholdFlags_Activity
* END**************************************************************************/
uint32_t ADC_DRV_GetWdgThresholdFlags(const uint32_t instance)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    const ADC_Type * const base = s_adcBase[instance];
    return base->WTISR;
}

/*FUNCTION**********************************************************************
*
* Function Name : ADC_DRV_ClearWdgThresholdFlags
* Description   : This function clears the watchdog threshold interrupt flags (WTISR)
* with a bit mask provided. The macros ADC_WDOG_REG_MASK_HIGH and ADC_WDOG_REG_MASK_LOW
* can be used to compose the bitmask.
*
* Implements    : ADC_DRV_ClearWdgThresholdFlags_Activity
* END**************************************************************************/
void ADC_DRV_ClearWdgThresholdFlags(const uint32_t instance,
                                    const uint32_t wdogRegMask)
{
	DEV_ASSERT(instance < ADC_INSTANCE_COUNT);
    ADC_Type * const base = s_adcBase[instance];
    base->WTISR = wdogRegMask; /* w1c bits */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
