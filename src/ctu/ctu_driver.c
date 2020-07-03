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
 * @file ctu_driver.c
 *
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
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 */

#include <stddef.h>
#include <stdbool.h>

#include "ctu_driver.h"
#include "device_registers.h"
#include "status.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Local function prototypes
 ******************************************************************************/
static uint32_t CTU_SelectInput(const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge);
static void CTU_ConfigSingleCmd(const uint32_t instance, const uint32_t pos, const ctu_adc_cmd_t * const adcCmd, const uint32_t lcFlagValue);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for CTU instances. */
static CTU_Type * const s_ctuBase[CTU_INSTANCE_COUNT] = CTU_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_Config
* Description   : This function configures the global functionalities of a CTU instance.
*
* Implements    : CTU_DRV_Config_Activity
* END**************************************************************************/
void CTU_DRV_Config(const uint32_t instance, const ctu_config_t * const config)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(config->seqModeMrsInputEdge < CTU_INPUT_EDGE_BOTH); /* A single edge can be selected for MRS input */

    CTU_Type * const base = s_ctuBase[instance];
    uint16_t tgscr = 0u, ir = 0u, offset = 0u;

    if(config->tgsMode == CTU_TGS_MODE_SEQUENTIAL)
    {
        tgscr |= CTU_TGSCR_TGS_M_MASK;
        offset = (config->seqModeMrsInputEdge == CTU_INPUT_EDGE_RISING) ? (uint16_t)0u : (uint16_t)1u;
        /* TGSCR.MRS_SM must be set with the index corresponding to entries in TGSISR */
        tgscr |= CTU_TGSCR_MRS_SM((2u * ((uint16_t) config->seqModeMrsInput)) + offset);
    }

    tgscr |= CTU_TGSCR_PRES(config->prescaler);
    tgscr |= (config->extTrigMode == CTU_EXT_TRIG_MODE_TOGGLE) ? CTU_TGSCR_ET_TM_MASK : 0u;

    ir |= (config->intEnMRS == true)   ? (uint16_t)CTU_IR_MRS_IE_MASK : (0u);
    ir |= (config->intEnError == true) ? (uint16_t)CTU_IR_IEE_MASK : (0u);
    ir |= (config->dmaDoneGRE == true) ? (uint16_t)CTU_IR_DMA_DE_MASK : (0u);
    ir |= (config->dmaReqMRS == true)  ? (uint16_t)CTU_IR_MRS_DMAE_MASK : (0u);

    base->IR = ir;

    base->TGSCR = tgscr;
    base->TGSCCR = CTU_TGSCCR_TGSCCV(config->tgsCounterCompareVal);
    base->TGSCRR = CTU_TGSCRR_TGSCRV(config->tgsCounterReloadVal);

    base->TGSISR = config->inputTrigSelectMask;
    CTU_DRV_SyncInputSelect(instance); /* Load the new value for TGSISR (double buffered reg) */

    base->LISTCSR |= (config->adcCmdListMode == CTU_ADC_CMD_LIST_MODE_PARALLEL) ? CTU_LISTCSR_PAR_LIST(1u) : 0u;

    base->CR |= (config->disableOutput == true) ? (uint16_t)CTU_CR_CTU_ODIS_MASK : (0u);
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetDefaultConfig
* Description   : This function populates the configuration structure with the
* default configuration for CTU.
*
* Implements    : CTU_DRV_GetDefaultConfig_Activity
* END**************************************************************************/
void CTU_DRV_GetDefaultConfig(ctu_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->tgsMode              = CTU_TGS_MODE_TRIGGERED;
    config->prescaler            = CTU_PRESCALER_1;
    config->extTrigMode          = CTU_EXT_TRIG_MODE_PULSE;
    config->inputTrigSelectMask  = 0u;
    config->tgsCounterCompareVal = 0u;
    config->tgsCounterReloadVal  = 0u;
    config->adcCmdListMode       = CTU_ADC_CMD_LIST_MODE_STREAMING;
    config->seqModeMrsInput      = CTU_INPUT_TRIG_PWM_REL;
    config->seqModeMrsInputEdge  = CTU_INPUT_EDGE_RISING;
    config->intEnMRS             = false;
    config->intEnError           = false;
    config->dmaDoneGRE           = false;
    config->dmaReqMRS            = false;
    config->disableOutput        = false;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_Reset
* Description   : This function resets all registers of a CTU instance to default values.
* It stops and resets any running ADC command list.
* Note: the function also enables the CTU output (default state).
*
* Implements    : CTU_DRV_Reset_Activity
* END**************************************************************************/
void CTU_DRV_Reset(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];
    uint8_t fifoIdx, idx;
    uint32_t convData, fifoEmptyMask;
    ctu_config_t config;
    uint32_t errFlags;
    ctu_tgs_mode_t tgsModeBeforeReset = CTU_TGS_MODE_TRIGGERED;

    /* Save TGS Mode state */
    if((base->TGSCR & CTU_TGSCR_TGS_M_MASK) != 0u)
    {
        tgsModeBeforeReset = CTU_TGS_MODE_SEQUENTIAL;
    }

    /* Disable all input triggers */
    base->TGSISR = 0u;
    CTU_DRV_SyncInputSelect(instance); /* Load the new value for TGSISR (double buffered reg) */

    CTU_DRV_DisableOutput(instance);

    /* Reset running ADC command list */
    CTU_DRV_ResetAdcCmdListState(instance);

    /* Reset general configurations to default */
    CTU_DRV_GetDefaultConfig(&config);
    CTU_DRV_Config(instance, &config);

    /* Reset internal trigger properties */
#if FEATURE_CTU_HAS_TCR_UNROLLED
    base->T0CR = 0u;
    base->T1CR = 0u;
    base->T2CR = 0u;
    base->T3CR = 0u;
    base->T4CR = 0u;
    base->T5CR = 0u;
    base->T6CR = 0u;
    base->T7CR = 0u;
#else
    for(idx = 0; idx < CTU_TCR_COUNT; idx++)
    {
        base->TCR[idx] = 0u;
    }
#endif

    base->CLCR1 = 0u;
    base->CLCR2 = 0u;
    base->THCR1 = 0u;
    base->THCR2 = 0u;

    /* Reset command list registers */
    for(idx = 0; idx < CTU_CHANNEL_COUNT; idx++)
    {
        base->CHANNEL[idx].CLR_A = 0u;
    }

    /* Reset FIFOs */
    base->FDCR = 0u;
    base->FTH  = 0u;
    /* Empty the result FIFOs without triggering additional interrupts */
    /* Assumes that there is no pending conversion data to be written in the FIFOs. */
    for(fifoIdx = 0u; fifoIdx < CTU_FR_COUNT; fifoIdx++)
    {
        CTU_DRV_DisableFifoInterrupts(instance, fifoIdx, CTU_FIFO_ALL);
        fifoEmptyMask = CTU_DRV_GetFifoStatusFlags(instance, fifoIdx) & CTU_FIFO_EMPTY;
        while(fifoEmptyMask == 0u)
        {
            convData = CTU_DRV_GetConvData(instance, fifoIdx, CTU_RESULT_ALIGN_RIGHT_UNSIGNED);
            (void) convData; /* Data is only read to empty the result FIFO */

            fifoEmptyMask = CTU_DRV_GetFifoStatusFlags(instance, fifoIdx) & CTU_FIFO_EMPTY;
        }
        /* Note that CTU_FIFO_EMPTY flags will remain set, because they reflect the state of the FIFOs
         * and cannot be reset by writing to them. */
        CTU_DRV_ClearFifoStatusFlags(instance, fifoIdx, CTU_FIFO_OVERRUN);
    }

    /* Reset the rest of registers to default */
    base->IR      = 0u;
    base->COTR    = 0u;

    base->DFR     = 0u;
    base->EXPAR   = CTU_EXPAR_EXPA_MASK;
    base->EXPBR   = CTU_EXPBR_EXPB_MASK;
    base->CNTRNGR = 0u;
    base->LISTCSR = 0u;

    base->CR      = 0u; /* Reset also CR.CTU_ODIS to enable the output */

    /* Clear error flags */
    CTU_DRV_ClearErrorFlags(instance, CTU_ERROR_FLAG_ALL);

    /* SW trigger MRS to make sure reset values get loaded to the double buffered registers
     * and FGRE flag is reset (refer to device reference manual for reload mechanism). */
    CTU_DRV_EnableGeneralReload(instance);
    CTU_DRV_SwMasterReloadSignal(instance);

    if(tgsModeBeforeReset == CTU_TGS_MODE_TRIGGERED)
    {
        /* In TRIGGERED mode, the Sw MRS starts the TGS Counter, also in the case when TGSCRR = TGSCRR (see Reference Manual chapter on TGS counter).
         * So need to wait until the TGS Counter stops.
         * In SEQUENTIAL mode, the Sw MRS doesn't start the TGS Counter at all. So, these procedures are not required. */
        do
        {
            errFlags = CTU_DRV_GetErrorFlags(instance);
        }
        while((errFlags & CTU_ERROR_FLAG_ERRCMP) == 0u);

        /* Clear error and interrupt flags which may have been set during the control cycle started by Sw MRS */
        CTU_DRV_ClearErrorFlags(instance, CTU_ERROR_FLAG_ALL);
        CTU_DRV_ClearInterruptFlags(instance, CTU_INTERRUPT_FLAG_ALL);
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SetInputSelect
* Description   : This function enables the selected signal edge(s) for a CTU input trigger.
* The affected register is double-buffered, so the new value will take effect only
* after MRS occurs or CTU_DRV_SyncInputSelect() is called.
* Please note that the set operation is cumulative, e.g. set falling edge, then set rising edge
* for same input trigger, results in both edges being enabled for that trigger.
*
* Implements    : CTU_DRV_SetInputSelect_Activity
* END**************************************************************************/
void CTU_DRV_SetInputSelect(const uint32_t instance, const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];
    uint32_t tgsisrMask = CTU_SelectInput(inputTrig, inputEdge);

    base->TGSISR |= tgsisrMask;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ClearInputSelect
* Description   : This function disables the selected signal edge(s) for a CTU input trigger.
* The affected register is double-buffered, so the new value will take effect only
* after MRS occurs or CTU_DRV_SyncInputSelect() is called.
*
* Implements    : CTU_DRV_ClearInputSelect_Activity
* END**************************************************************************/
void CTU_DRV_ClearInputSelect(const uint32_t instance, const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];
    uint32_t tgsisrMask = CTU_SelectInput(inputTrig, inputEdge);

    base->TGSISR &= ~tgsisrMask;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SyncInputSelect
* Description   : This function loads the new value of input trigger select - TGSISR (double buffered register).
* The function waits until the TGSISR register is updated from the corresponding buffer register.
*
* Implements    : CTU_DRV_SyncInputSelect_Activity
* END**************************************************************************/
void CTU_DRV_SyncInputSelect(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_TGSISR_RE_MASK;

    /* Wait until the TGSISR is updated from the corresponding buffer register (according to double-buffering mechanism).
     * TGSISR_RE is self negated - once set, remains set until TGSISR update is complete. */
    while((base->CR & CTU_CR_TGSISR_RE_MASK) != 0u) {};
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ConfigInternalTrigger
* Description   : This function configures the properties of a single internal trigger.
* By internal trigger, the CTU driver refers to the output triggers from TGS (Trigger Generator Subunit),
* fed as input triggers to the SU (Scheduler Subunit). \n
*
* Note regarding members of ctu_trig_config_t: \n
*  <compareVal> and <outputTrigEnMask>, correspond to double buffered registers,
*  so configured values are loaded and take effect only after MRS occurs. \n
*  <intEn> and <cmdListStartAdr> are not double-buffered, so the configured values
*  take effect immediately after CTU_DRV_ConfigInternalTrigger() is called.
*
* Implements    : CTU_DRV_ConfigInternalTrigger_Activity
* END**************************************************************************/
void CTU_DRV_ConfigInternalTrigger(const uint32_t instance, const uint32_t internalTrigIdx, const ctu_trig_config_t * const internalTrigConfig)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(internalTrigIdx < CTU_TCR_COUNT);
    DEV_ASSERT(internalTrigConfig != NULL);

    CTU_Type * const base = s_ctuBase[instance];
    uint32_t shift;

#if FEATURE_CTU_HAS_TCR_UNROLLED
    switch (internalTrigIdx) {
    case 0:
        base->T0CR = internalTrigConfig->compareVal;
        break;
    case 1:
        base->T1CR = internalTrigConfig->compareVal;
        break;
    case 2:
        base->T2CR = internalTrigConfig->compareVal;
        break;
    case 3:
        base->T3CR = internalTrigConfig->compareVal;
        break;
    case 4:
        base->T4CR = internalTrigConfig->compareVal;
        break;
    case 5:
        base->T6CR = internalTrigConfig->compareVal;
        break;
    case 6:
        base->T6CR = internalTrigConfig->compareVal;
        break;
    case 7:
        base->T7CR = internalTrigConfig->compareVal;
        break;

        default:
            DEV_ASSERT(false);
            break;
    }
#else
    base->TCR[internalTrigIdx] = internalTrigConfig->compareVal;
#endif

    if(internalTrigIdx < 4u) /* CLCR1 and THCR1 store info for internal triggers: #0 -> #3 */
    {
        shift = internalTrigIdx * CTU_CLCR1_T1_INDEX_SHIFT;
        /* Clear bits corresponding to Tn_INDEX bitfield - where 'n' is internalTrigIdx */
        base->CLCR1 &= ~((uint32_t)CTU_CLCR1_T0_INDEX_MASK << shift);
        /* Write new value */
        base->CLCR1 |= CTU_CLCR1_T0_INDEX(internalTrigConfig->cmdListStartAdr) << shift;

        shift = internalTrigIdx * CTU_THCR1_T1_ADCE_SHIFT;
        /* Clear bits corresponding to Tn bitfields - where 'n' is internalTrigIdx */
        base->THCR1 &= ~(CTU_OUTPUT_TRIG_ALL_EN << shift);
        /* Write new value */
        base->THCR1 |= (internalTrigConfig->outputTrigEnMask & CTU_OUTPUT_TRIG_ALL_EN) << shift;
    }
    else /* CLCR2 and THCR2 store info for internal triggers: #3 -> #7 */
    {
        shift = (internalTrigIdx - 4u) * CTU_CLCR2_T5_INDEX_SHIFT;
        /* Clear bits corresponding to Tn_INDEX bitfield - where 'n' is internalTrigIdx */
        base->CLCR2 &= ~((uint32_t)CTU_CLCR2_T4_INDEX_MASK << shift);
        /* Write new value */
        base->CLCR2 |= CTU_CLCR2_T4_INDEX(internalTrigConfig->cmdListStartAdr) << shift;

        shift = (internalTrigIdx - 4u) * CTU_THCR2_T5_ADCE_SHIFT;
        /* Clear bits corresponding to Tn bitfields - where 'n' is internalTrigIdx */
        base->THCR2 &= ~(CTU_OUTPUT_TRIG_ALL_EN << shift);
        /* Write new value */
        base->THCR2 |= (internalTrigConfig->outputTrigEnMask & CTU_OUTPUT_TRIG_ALL_EN) << shift;
    }

    shift = CTU_IR_T0_IE_SHIFT + internalTrigIdx;
    if(internalTrigConfig->intEn == true)
    {
        base->IR |= ((uint16_t)1u << shift);
    }
    else
    {
        base->IR &= ~((uint16_t)1u << shift);
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ConfigAdcCmdList
* Description   : This function configures an array of ADC commands and marks accordingly the last command. \n
*
* Note: the ADC command registers are double buffered,
*  so configured values are loaded and take effect only after MRS occurs. \n
*  The function doesn't take into account the reload mechanism (refer to device reference manual for details),
*  so it needs to be handled externally - may use CTU_DRV_EnableGeneralReload().
*
* Implements    : CTU_DRV_ConfigAdcCmdList_Activity
* END**************************************************************************/
void CTU_DRV_ConfigAdcCmdList(const uint32_t instance, const uint32_t startCmdListAdr, const ctu_adc_cmd_t * const adcCmdArray, uint8_t numCmds)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(startCmdListAdr < CTU_CHANNEL_COUNT);
    DEV_ASSERT(adcCmdArray != NULL);
    DEV_ASSERT(numCmds <= CTU_CHANNEL_COUNT);
    DEV_ASSERT(numCmds != 0u);

    CTU_Type * const base = s_ctuBase[instance];
    uint32_t idx, lcFlagFirstCmd;
    uint32_t pos = startCmdListAdr;

    /* Preserve value of LastCommand (LC) flag corresponding to the first command in the list,
     * because LC bit is ignored in the first command of a list. */
    lcFlagFirstCmd = ((uint32_t)base->CHANNEL[pos].CLR_A & CTU_CLR_A_LC_MASK) >> CTU_CLR_A_LC_SHIFT;
    CTU_ConfigSingleCmd(instance, pos, &(adcCmdArray[0]), lcFlagFirstCmd);

    for(idx = 1u; idx < numCmds; idx++)
    {
        pos = (startCmdListAdr + idx) % CTU_CHANNEL_COUNT;

        CTU_ConfigSingleCmd(instance, pos, &(adcCmdArray[idx]), 0u); /* Clear the LC bit for the rest of the commands */
    }

    /* Mark the last command in the list.
     * The last command (n) is defined by the LC bit of the next command (n+1) in the list.
     * Note that the command with LC=1 does not belong to the list and thus it is not executed.
     * Note also that the LC bit is ignored in the first command of a list. */
    pos = (pos + 1u) % CTU_CHANNEL_COUNT;
    base->CHANNEL[pos].CLR_A |= CTU_CLR_A_LC_MASK;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ConfigResultFifo
* Description   : This function configures the properties of a selected result FIFO.
*
* Implements    : CTU_DRV_ConfigResultFifo_Activity
* END**************************************************************************/
void CTU_DRV_ConfigResultFifo(const uint32_t instance, const uint32_t fifoIdx, const ctu_res_fifo_config_t * const fifoConfig)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);
    DEV_ASSERT(fifoConfig != NULL);

    CTU_Type * const base = s_ctuBase[instance];
    uint32_t fdcr;
    uint32_t shift;

    shift = CTU_FCR_FULL_EN1_SHIFT * fifoIdx;
    /* Clear bits corresponding to fifoIdx in FCR */
    base->FCR &= ~(CTU_FIFO_ALL << shift);
    /* Set new values for fifoIdx in FCR */
    base->FCR |= ((uint32_t)fifoConfig->fifoIntEnMask & CTU_FIFO_ALL) << shift;

    shift = CTU_FTH_TH1_SHIFT * fifoIdx;
    /* Clear bits corresponding to fifoIdx in FTH */
    base->FTH &= ~((uint32_t)CTU_FTH_TH0_MASK << shift);
    /* Set new values for fifoIdx in FTH */
    base->FTH |= ((uint32_t)fifoConfig->fifoThreshold & CTU_FTH_TH0_MASK) << shift;

    /* FDCR includes reserved w1c bitfields which will be set with 0s */
    fdcr = (uint32_t)(base->FDCR) & (uint32_t)(CTU_FDCR_DE0_MASK | CTU_FDCR_DE1_MASK | CTU_FDCR_DE2_MASK | CTU_FDCR_DE3_MASK);

    if(fifoConfig->dmaEn == true)
    {
        fdcr |= (uint32_t)CTU_FDCR_DE0_MASK << fifoIdx;
    }
    else
    {
        fdcr &= ~((uint32_t)CTU_FDCR_DE0_MASK << fifoIdx);
    }

    base->FDCR = (CTU_FDCR_TYPE) fdcr;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetConvData
* Description   : This function returns the conversion data result aligned according
* to the alignment parameter.
*
* Implements    : CTU_DRV_GetConvData_Activity
* END**************************************************************************/
uint16_t CTU_DRV_GetConvData(const uint32_t instance, const uint32_t fifoIdx, ctu_data_align_t alignment)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);

    const CTU_Type * const base = s_ctuBase[instance];
    uint16_t data = 0;

    switch(alignment)
    {
    case CTU_RESULT_ALIGN_RIGHT_UNSIGNED:
        data = (uint16_t) (base->FR[fifoIdx] & CTU_FR_DATA_MASK);
        break;
    case CTU_RESULT_ALIGN_LEFT_SIGNED:
        data = (uint16_t) (base->FL[fifoIdx] & CTU_FL_LA_DATA_MASK);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

    return data;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetConvResult
* Description   : This function gets the full conversion result information, with
* the conversion data aligned according to the alignment parameter.
*
* Implements    : CTU_DRV_GetConvResult_Activity
* END**************************************************************************/
void CTU_DRV_GetConvResult(const uint32_t instance, const uint32_t fifoIdx, const ctu_data_align_t alignment, ctu_conv_result_t * const result)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);
    DEV_ASSERT(result != NULL);

    const CTU_Type * const base = s_ctuBase[instance];
    uint32_t resInfo = 0;

    switch(alignment)
    {
    case CTU_RESULT_ALIGN_RIGHT_UNSIGNED:
        resInfo = base->FR[fifoIdx];
        result->convData = (uint16_t) (resInfo & CTU_FR_DATA_MASK);
        break;
    case CTU_RESULT_ALIGN_LEFT_SIGNED:
        resInfo = base->FL[fifoIdx];
        result->convData = (uint16_t) (resInfo & CTU_FL_LA_DATA_MASK);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

   result->adcPort    = ((resInfo & CTU_FR_ADC_MASK) != 0u) ? CTU_ADC_PORT_A : CTU_ADC_PORT_B;
   result->adcChanNum = (uint8_t) ((resInfo & CTU_FR_N_CH_MASK) >> CTU_FR_N_CH_SHIFT);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ClearFifoStatusFlags
* Description   : This function clears the status flags enabled in the bit-mask,
* for the selected result FIFO.
* The mask parameter can be set using the defines provided in the driver header file:
* CTU_FIFO_ - e.g. CTU_FIFO_OVERRUN.  \n
*
* Note: CTU permits only for the FST.OR flags (CTU_FIFO_OVERRUN) to be cleared.
*
* Implements    : CTU_DRV_ClearFifoStatusFlags_Activity
* END**************************************************************************/
void CTU_DRV_ClearFifoStatusFlags(const uint32_t instance, const uint32_t fifoIdx, const uint32_t statusFlagsMask)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);
    DEV_ASSERT((statusFlagsMask & (~CTU_FIFO_OVERRUN)) == 0u); /* CTU only allows FST.OR (CTU_FIFO_OVERRUN) to be cleared. */

    CTU_Type * const base = s_ctuBase[instance];

    if((statusFlagsMask & CTU_FIFO_OVERRUN) != 0u)
    {
        base->FST |= (uint32_t)CTU_FST_OR0_MASK << (CTU_FST_FULL1_SHIFT * fifoIdx);
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetFifoStatusFlags
* Description   : Returns bitmask with all the status flags for a selected result FIFO.
* Values for individual flags may be retrieved by masking the result with defines provided
* in the driver header file: CTU_FIFO_ - e.g. CTU_FIFO_OVERFLOW.
*
* Implements    : CTU_DRV_GetFifoStatusFlags_Activity
* END**************************************************************************/
uint32_t CTU_DRV_GetFifoStatusFlags(const uint32_t instance, const uint32_t fifoIdx)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);

    const CTU_Type * const base = s_ctuBase[instance];
    uint32_t result;
    const uint32_t shift = CTU_FST_FULL1_SHIFT * fifoIdx;

    result = (base->FST & (CTU_FIFO_ALL << shift)) >> shift;

    return result;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_EnableFifoInterrupts
* Description   : Enables the interrupts for the selected result FIFO, corresponding
* to the bits set in mask parameter.
* The mask parameter can be set using the defines provided in the driver header file:
* CTU_FIFO_ - e.g. CTU_FIFO_FULL.
*
* Implements    : CTU_DRV_EnableFifoInterrupts_Activity
* END**************************************************************************/
void CTU_DRV_EnableFifoInterrupts(const uint32_t instance, const uint32_t fifoIdx, const uint32_t intFlagsMask)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);

    CTU_Type * const base = s_ctuBase[instance];
    const uint32_t shift = CTU_FCR_FULL_EN1_SHIFT * fifoIdx;

    base->FCR |= ((intFlagsMask & CTU_FIFO_ALL) << shift);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_DisableFifoInterrupts
* Description   : Disables the interrupts for the selected result FIFO, corresponding
* to the bits set in mask parameter.
* The mask parameter can be set using the defines provided in the driver header file:
* CTU_FIFO_ - e.g. CTU_FIFO_FULL.
*
* Implements    : CTU_DRV_DisableFifoInterrupts_Activity
* END**************************************************************************/
void CTU_DRV_DisableFifoInterrupts(const uint32_t instance, const uint32_t fifoIdx, const uint32_t intFlagsMask)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < CTU_FR_COUNT);

    CTU_Type * const base = s_ctuBase[instance];
    const uint32_t shift = CTU_FCR_FULL_EN1_SHIFT * fifoIdx;

    base->FCR &= ~((intFlagsMask & CTU_FIFO_ALL) << shift);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ClearErrorFlags
* Description   : Clears the error flags corresponding to the bits set in mask parameter.
* The mask parameter can be set using the defines provided in the driver header file:
* CTU_ERROR_FLAG_ - e.g. CTU_ERROR_FLAG_INVALID_CMD.
*
* Implements    : CTU_DRV_ClearErrorFlags_Activity
* END**************************************************************************/
void CTU_DRV_ClearErrorFlags(const uint32_t instance, const uint32_t flagsMask)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->EFR |= (uint16_t)(flagsMask & CTU_ERROR_FLAG_ALL);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetErrorFlags
* Description   : Returns bit-mask with values for all the CTU error flags.
* Values for individual flags may be retrieved by masking the result with defines provided
* in the driver header file: CTU_ERROR_FLAG_ - e.g. CTU_ERROR_FLAG_MRS_RE.
*
* Implements    : CTU_DRV_GetErrorFlags_Activity
* END**************************************************************************/
uint32_t CTU_DRV_GetErrorFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    const CTU_Type * const base = s_ctuBase[instance];
    uint32_t result;

    result = base->EFR & CTU_ERROR_FLAG_ALL;

    return result;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ClearInterruptFlags
* Description   : Clears the interrupt flags, corresponding to the bits set in mask parameter.
* The mask parameter can be set using the defines provided in the driver header file:
* CTU_INTERRUPT_FLAG_ - e.g. CTU_INTERRUPT_FLAG_MRS.
*
* Implements    : CTU_DRV_ClearInterruptFlags_Activity
* END**************************************************************************/
void CTU_DRV_ClearInterruptFlags(const uint32_t instance, const uint32_t flagsMask)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->IFR |= (uint16_t)(flagsMask & CTU_INTERRUPT_FLAG_ALL);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetInterruptFlags
* Description   : Returns bitmask with values for all the CTU interrupt status flags.
* Values for individual flags may be retrieved by masking the result with defines provided
* in the driver header file: CTU_INTERRUPT_FLAG_ - e.g. CTU_INTERRUPT_FLAG_MRS.
*
* Implements    : CTU_DRV_GetInterruptFlags_Activity
* END**************************************************************************/
uint32_t CTU_DRV_GetInterruptFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    const CTU_Type * const base = s_ctuBase[instance];
    uint32_t result;

    result = base->IFR & CTU_INTERRUPT_FLAG_ALL;

    return result;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SwInternalTrigger
* Description   : Generates from software an event on the selected internal trigger,
* provided as input to the Scheduler subunit (SU).
* This bypasses any delays configured in Trigger Generator Subunit (TGS)
* and can be used when the TGS Counter is running or stopped.
*
* Implements    : CTU_DRV_SwInternalTrigger_Activity
* END**************************************************************************/
void CTU_DRV_SwInternalTrigger(const uint32_t instance, const uint32_t internalTrigIdx)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);
    DEV_ASSERT(internalTrigIdx < CTU_TCR_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_T0_SG_MASK << internalTrigIdx;
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SwMasterReloadSignal
* Description   : Software triggers the Master Reload Signal.
*
* Implements    : CTU_DRV_SwMasterReloadSignal_Activity
* END**************************************************************************/
void CTU_DRV_SwMasterReloadSignal(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_MRS_SG_MASK;
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_EnableOutput
* Description   : Enables CTU output
*
* Implements    : CTU_DRV_EnableOutput_Activity
* END**************************************************************************/
void CTU_DRV_EnableOutput(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR &= ~CTU_CR_CTU_ODIS_MASK;
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_DisableOutput
* Description   : Enables CTU output
*
* Implements    : CTU_DRV_DisableOutput_Activity
* END**************************************************************************/
void CTU_DRV_DisableOutput(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_CTU_ODIS_MASK;
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SetExternalTrigsDuration
* Description   : Sets the on-time + guard-time duration for external triggers.
*
* Implements    : CTU_DRV_SetExternalTrigsDuration_Activity
* END**************************************************************************/
void CTU_DRV_SetExternalTrigsDuration(const uint32_t instance, const uint8_t duration)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->COTR = CTU_COTR_COTGT(duration);
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SetDigitalFilterValue
* Description   : Sets the digital filter value. If the set value is >0, then the function
* implicitly enables the digital filter. Otherwise the function implicitly disables it.
*
* Implements    : CTU_DRV_SetDigitalFilterValue_Activity
* END**************************************************************************/
void CTU_DRV_SetDigitalFilterValue(const uint32_t instance, const uint8_t filterValue)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    if(filterValue == 0u)
    {
        base->CR &= ~CTU_CR_DFE_MASK;
        base->DFR = (uint16_t)(CTU_DFR_FILTER_N(0u));
    }
    else
    {
        base->DFR = CTU_DFR_FILTER_N(filterValue);
        base->CR |= CTU_CR_DFE_MASK;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SetExpectedConvDuration
* Description   : Sets the maximum expected duration for an ADC conversion on the
* selected ADC instance.
* Checking of the ADC conversion duration needs to be enabled via dedicated function.
*
* Implements    : CTU_DRV_SetExpectedConvDuration_Activity
* END**************************************************************************/
void CTU_DRV_SetExpectedConvDuration(const uint32_t instance, const ctu_adc_port_t adcIdx, const uint16_t duration)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    switch(adcIdx)
    {
    case CTU_ADC_PORT_A:
        base->EXPAR = CTU_EXPAR_EXPA(duration);
        break;
    case CTU_ADC_PORT_B:
        base->EXPBR = CTU_EXPBR_EXPB(duration);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_SetExpectedConvRange
* Description   : This function sets the value of counter range register. The value is used to mask out the
 * related bits for expected conversion duration. For the exact formula, please refer to reference manual.
*
* Implements    : CTU_DRV_SetExpectedConvRange_Activity
* END**************************************************************************/
void CTU_DRV_SetExpectedConvRange(const uint32_t instance, const uint16_t range)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CNTRNGR = CTU_CNTRNGR_CNTRNG(range);
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_EnableCheckDuration
* Description   : Enables checking of the ADC conversion duration, for the selected
* ADC instance. If a conversion takes a number of cycles out of the interval
* defined by CTU_DRV_SetExpectedConvRange() and CTU_DRV_SetExpectedConvDuration()
* an error flag is raised SERR_A/SERR_B (please refer to device reference manual
* for details).
*
* Implements    : CTU_DRV_EnableCheckDuration_Activity
* END**************************************************************************/
void CTU_DRV_EnableCheckDuration(const uint32_t instance, const ctu_adc_port_t adcIdx)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->IR |= (adcIdx == CTU_ADC_PORT_A) ? (uint16_t)CTU_IR_SAF_CNT_A_EN_MASK : (uint16_t)CTU_IR_SAF_CNT_B_EN_MASK;
}

/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_DisableCheckDuration
* Description   : Disables checking of the ADC conversion duration, for the selected
* ADC instance.
*
* Implements    : CTU_DRV_DisableCheckDuration_Activity
* END**************************************************************************/
void CTU_DRV_DisableCheckDuration(const uint32_t instance, const ctu_adc_port_t adcIdx)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->IR &= (adcIdx == CTU_ADC_PORT_A) ? ~((uint16_t) CTU_IR_SAF_CNT_A_EN_MASK) : ~((uint16_t) CTU_IR_SAF_CNT_B_EN_MASK);
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_EnableGeneralReload
* Description   : Enables General Reload (CR.GRE)
*
* Implements    : CTU_DRV_EnableGeneralReload_Activity
* END**************************************************************************/
void CTU_DRV_EnableGeneralReload(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_GRE_MASK;
}


/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_DisableGeneralReload
* Description   : Disables General Reload (CR.GRE) - for details please see reload
* mechanism described in device reference manual.
*
* Implements    : CTU_DRV_DisableGeneralReload_Activity
* END**************************************************************************/
void CTU_DRV_DisableGeneralReload(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_CGRE_MASK;

    uint16_t cr = base->CR;

    /* Wait while CGRE takes effect and clears GRE bit.
     * This is done to mitigate propagation delay in CTU module and guarantee that any code called after CTU_DRV_DisableGeneralReload
     * is executed after CGRE write takes effect.
     * The delay is not expected to take more than 2 CTU clock cycles. */
    while((cr & CTU_CR_GRE_MASK) != 0u)
    {
        cr = base->CR;
    }

}



/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_GetListStatus
* Description   : This function gets the list status information.
* Information for multiple lists is available when executing in parallel mode.
*
* Implements    : CTU_DRV_GetListStatus_Activity
* END**************************************************************************/
status_t CTU_DRV_GetListStatus(const uint32_t instance, const uint32_t listIdx, ctu_list_status_t * const listStatus)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    const CTU_Type * const base = s_ctuBase[instance];
    status_t status;

    if((CTU_DRV_GetErrorFlags(instance) & CTU_ERROR_FLAG_LIST_BUSY) == 0u)
    {
        status = STATUS_ERROR; /* Result is not meaningful, because EFR.LIST_BE is not set. */
    }
    else
    {
        switch(listIdx)
        {
        case 0u:
            listStatus->listAddr    = (uint8_t) ((base->LISTCSR & CTU_LISTCSR_LIST0_ADDR_MASK) >> CTU_LISTCSR_LIST0_ADDR_SHIFT);
            listStatus->listBlocked = ((base->LISTCSR & CTU_LISTCSR_LIST0_BLK_MASK) != 0u) ? true : false;
            break;
        case 1u:
            listStatus->listAddr    = (uint8_t) ((base->LISTCSR & CTU_LISTCSR_LIST1_ADDR_MASK) >> CTU_LISTCSR_LIST1_ADDR_SHIFT);
            listStatus->listBlocked = ((base->LISTCSR & CTU_LISTCSR_LIST1_BLK_MASK) != 0u) ? true : false;
            break;
        default:
            DEV_ASSERT(false);
            break;
        }

        status = STATUS_SUCCESS;
    }

    return status;
}



/*FUNCTION**********************************************************************
*
* Function Name : CTU_DRV_ResetAdcCmdListState
* Description   : This function resets CTU ADC command list control state machine.
* After a reset, the command list stops execution and waits for a new trigger event
* in order to initiate a command list execution.
*
* Implements    : CTU_DRV_ResetAdcCmdListState_Activity
* END***************************************************************************/
void CTU_DRV_ResetAdcCmdListState(const uint32_t instance)
{
    DEV_ASSERT(instance < CTU_INSTANCE_COUNT);

    CTU_Type * const base = s_ctuBase[instance];

    base->CR |= CTU_CR_CTU_ADC_R_MASK;
}



/*LOCAL FUNCTION**********************************************************************
*
* Function Name : CTU_SelectInput
* Description   : Returns bitmask with '1' on the position of the input signal edge(s)
* END*********************************************************************************/
static uint32_t CTU_SelectInput(const ctu_input_trig_t inputTrig, const ctu_input_edge_t inputEdge)
{
    uint32_t tgsisrMask = ~(CTU_INPUT_TRIG_BITMASK_ALL);

    switch(inputEdge)
    {
    case CTU_INPUT_EDGE_RISING:
        tgsisrMask = CTU_INPUT_TRIG_BITMASK_RISING(inputTrig);
        break;
    case CTU_INPUT_EDGE_FALLING:
        tgsisrMask = CTU_INPUT_TRIG_BITMASK_FALLING(inputTrig);
        break;
    case CTU_INPUT_EDGE_BOTH:
        tgsisrMask = CTU_INPUT_TRIG_BITMASK_BOTH(inputTrig);
        break;
    default:
        DEV_ASSERT(false);
        break;
    }

    return tgsisrMask;
}


/*LOCAL FUNCTION**********************************************************************
*
* Function Name : CTU_ConfigSingleCmd
* Description   : Configures a single ADC command at a selected position in the CTU command list.
* Note0: current lcFlag value refers to previous command in the list.
*  The last command (n) is defined by the LC bit of the next command (n+1) in the list.
* Note1: assumes 'pos' parameter is in correct range.
* END*********************************************************************************/
static void CTU_ConfigSingleCmd(const uint32_t instance, const uint32_t pos, const ctu_adc_cmd_t * const adcCmd, const uint32_t lcFlagValue)
{
    CTU_Type * const base = s_ctuBase[instance];
    uint16_t cmd = 0u;

    cmd |= CTU_CLR_A_LC(lcFlagValue); /* Common between all 3 formats */

    if(adcCmd->selfTestEn == false)   /* Not a Self Test Command (format A or B) */
    {
        cmd |= (adcCmd->intEn == true) ? CTU_CLR_A_CIR_MASK : 0u; /* Common between all 3 formats, but ignored in format C */
        cmd |= CTU_CLR_A_FIFO(adcCmd->fifoIdx);                   /* Common between formats A and B */

        switch(adcCmd->convMode)
        {
        case CTU_CONV_MODE_SINGLE: /* Command format A */
            cmd |= CTU_CLR_A_SU(adcCmd->adcPort);
            cmd |= CTU_CLR_A_CH(adcCmd->adcChanA);
            base->CHANNEL[pos].CLR_A = cmd;
            break;
        case CTU_CONV_MODE_DUAL: /* Command format B */
            cmd |= CTU_CLR_B_CMS_MASK;
            cmd |= CTU_CLR_B_CH_A(adcCmd->adcChanA);
            cmd |= CTU_CLR_B_CH_B(adcCmd->adcChanB);
            base->CHANNEL[pos].CLR_B = cmd;
            break;
        default:
            DEV_ASSERT(false);
            break;
        }
    }
    else /* Self Test Command (format C) */
    {
        /* CIR bit is ignored by CTU in self-test mode: the interrupt request is disabled and cannot be enabled using CIR bit. */
        cmd |= CTU_CLR_C_ST0_MASK;
        /* ST1: always set to 0 for self-test command */
        /* ST_CMS: unclear in RM Table 41-2 if it is used for self-test command */
        cmd |= (adcCmd->convMode == CTU_CONV_MODE_DUAL) ? CTU_CLR_C_ST_CMS_MASK : 0u;
        /* ST_SU: SU only set in single conversion mode */
        cmd |= (adcCmd->convMode == CTU_CONV_MODE_SINGLE) ? CTU_CLR_C_ST_SU(adcCmd->adcPort) : 0u;
        cmd |= CTU_CLR_C_ALG((uint32_t) adcCmd->selfTestAlg);
        cmd |= CTU_CLR_C_BSIZE(adcCmd->selfTestBurstSize);

        base->CHANNEL[pos].CLR_C = cmd;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

