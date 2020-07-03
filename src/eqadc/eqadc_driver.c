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
 * Functions are defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.1, Unpermitted operand to operator '>>'.
 * The right shift operation is executed on a signed value.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * The cast is required to perform a conversion between an unsigned integer and an enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
 * unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a byte value into a enum to get the address of internal on-chip ADC register.
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
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long, Cast from unsigned long to pointer.
 * The cast is required to perform a conversion between a pointer and an unsigned long define,
 * representing an address.
 *
 */

#include <stddef.h>
#include "device_registers.h"
#include "eqadc_driver.h"
#include "eqadc_hw_access.h"
#include "edma_driver.h"
#include "osif.h"
#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
/* Clock Manager is a dependency only when DEV_ASSERT is enabled, used in:
  - EQADC_ConfigAdc
*/
#include "clock_manager.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Defines for accessing bit-fields from EQADC commands */
#define EQADC_CMD_EOQ_MASK              (0x80000000u)
#define EQADC_CMD_PAUSE_MASK            (0x40000000u)
#define EQADC_CMD_REP_MASK              (0x20000000u)
#define EQADC_CMD_EB_MASK               (0x04000000u)
#define EQADC_CMD_BN_MASK               (0x02000000u)
/* Defines for accessing bit-fields from EQADC conv commands */
#define EQADC_CMD_CAL_MASK              (0x01000000u)
#define EQADC_CMD_LST_MASK              (0x000C0000u)
#define EQADC_CMD_TSR_MASK              (0x00020000u)
#define EQADC_CMD_FMT_MASK              (0x00010000u)
#define EQADC_CMD_CHAN_NUM_MASK         (0x0000FF00u)
#define EQADC_CMD_FFMT_MASK             (0x00010000u)
#define EQADC_CMD_ALT_CFG_SEL_MASK      (0x000000FFu)
#define EQADC_CMD_LST_SHIFT             (18u)
#define EQADC_CMD_CHAN_NUM_SHIFT        (8u)
#define EQADC_CMD_ALT_CFG_SEL_SHIFT     (0u)

/* Defines for accessing bit-fields from EQADC config commands */
#define EQADC_CMD_RW_MASK               (0x01000000u)
#define EQADC_CMD_REG_HIGH_BYTE_MASK    (0x00FF0000u)
#define EQADC_CMD_REG_LOW_BYTE_MASK     (0x0000FF00u)
#define EQADC_CMD_REG_ADDRESS_MASK      (0x000000FFu)
#define EQADC_CMD_MESSAGE_TAG_MASK      (0x00F00000u)
#define EQADC_CMD_REG_LOW_BYTE_SHIFT    (8u)
#define EQADC_CMD_MESSAGE_TAG_SHIFT     (20u)

#define EQADC_NUM_ALT_CONFIGS           (14u)
#define EQADC_NUM_EXT_ALT_CONFIGS       EQADC_NUM_ALT_CONFIGS
#define EQADC_NUM_CHAN_PULL_CONFIGS     (40u)
#define EQADC_NUM_STAC_BUS_CLIENTS      (2u)

#define EQADC_INVALID_DMA_VIRT_CHAN     (0xFFu)

/* The CFIFO index used for sending configuration commands required during Init() and Reset() */
#define EQADC_CFIFO_IDX_FOR_CONFIG_CMDS (0u)

/* Defines used by calibration procedure */
#define EQADC_CHAN_VREF_75_PROC         (43u)
#define EQADC_CHAN_VREF_25_PROC         (44u)
#define EQADC_CAL_VREF_75_PROC          (12288u)
#define EQADC_CAL_VREF_25_PROC          (4096u)
#define EQADC_GAIN_NUM_PRECISION_BITS   (14u)


/* Private functions definitions */
static void EQADC_ConfigAdc(const uint32_t instance,
                            const eqadc_adc_config_t * const adcConfig,
                            const uint8_t cfifoIdxForCmds);
static void EQADC_ConfigCfifo(const uint32_t instance,
                              const eqadc_cfifo_config_t * const cfifoConfig);
static void EQADC_ConfigCmdDma(const uint32_t instance,
                               const eqadc_cfifo_config_t * const cfifoConfig);
static void EQADC_ConfigResultDma(const uint32_t instance,
                                  const eqadc_result_dma_config_t * const resultDmaConfig);
static void EQADC_ConfigAltConfiguration(const uint32_t instance,
                                         const eqadc_alternate_config_t * const altConfig,
                                         const uint8_t cfifoIdxForCmds);
static void EQADC_ConfigExtAltConfiguration(const uint32_t instance,
                                            const eqadc_ext_alternate_config_t * const extAltConfig,
                                            const uint8_t cfifoIdxForCmds);
static void EQADC_ConfigChanPull(const uint32_t instance,
                                 const eqadc_chan_pull_config_t * const chanPullConfig,
                                 const uint8_t cfifoIdxForCmds);
static void EQADC_ConfigStacBusClient(const uint32_t instance,
                                      const eqadc_stac_bus_client_config_t * const stacBusClientConfig);

static void EQADC_GetDefaultCfifoConfig(eqadc_cfifo_config_t * const cfifoConfig);
static void EQADC_GetDefaultConfigCmd(eqadc_config_cmd_t * const configCmd);

static void EQADC_ResetAdc(const uint32_t instance,
                           const uint8_t adcIdx,
                           const uint8_t cfifoIdxForCmds);
static void EQADC_ResetCfifo(const uint32_t instance,
                             const uint8_t cfifoIdx);
static void EQADC_ResetAltConfiguration(const uint32_t instance,
                                        const eqadc_alt_config_sel_t altConfigSel,
                                        const uint8_t cfifoIdxForCmds);
static void EQADC_ResetExtAltConfiguration(const uint32_t instance,
                                           const eqadc_alt_config_sel_t extAltConfigSel,
                                           const uint8_t cfifoIdxForCmds);
static void EQADC_ResetChanPull(const uint32_t instance,
                                const uint8_t adcInputChanIdx,
                                const uint8_t cfifoIdxForCmds);
static void EQADC_ResetStacBusClient(const uint32_t instance,
                                     const uint32_t clientIdx);
static void EQADC_ResetTimeStampConfiguration(const uint32_t instance,
                                              const uint8_t cfifoIdxForCmds);

static uint32_t EQADC_GetCfifoConvCmd(const eqadc_conv_cmd_t * const convCmd);
static uint32_t EQADC_GetCfifoConfigCmd(const eqadc_config_cmd_t * const configCmd);

static inline uint8_t EQADC_GetAltConfigIdx(eqadc_alt_config_sel_t altConfigSel);
static inline eqadc_alt_config_sel_t EQADC_GetAltConfigSel(uint8_t index);
static inline eqadc_onchip_adc_reg_address_t EQADC_GetAltConfigRegAddr(eqadc_alt_config_sel_t altConfigSel);
static inline eqadc_onchip_adc_reg_address_t EQADC_GetExtAltConfigRegAddr(eqadc_alt_config_sel_t altConfigSel);

static void EQADC_WriteGainToConfigCmd(eqadc_config_cmd_t * const configCmd,
                                       const eqadc_calibration_target_t calibTarget,
                                       const uint16_t gain);
static void EQADC_WriteOffsetToConfigCmd(eqadc_config_cmd_t * const configCmd,
                                         const eqadc_calibration_target_t calibTarget,
                                         const int16_t offset);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for EQADC instances. */
static EQADC_Type * const s_eqadcBase[EQADC_INSTANCE_COUNT] = EQADC_BASE_PTRS;

static uint8_t dmaResultVirtualChans[EQADC_RFPR_COUNT] = {
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN, \
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN, \
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN
};
static uint8_t dmaCmdVirtualChans[EQADC_CFPR_COUNT] = {
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN, \
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN, \
        EQADC_INVALID_DMA_VIRT_CHAN, EQADC_INVALID_DMA_VIRT_CHAN
};

static const eqadc_onchip_adc_reg_address_t agrAddr[2u] = {
        EQADC_ONCHIP_ADC_REG_ADDRESS_AGR1,
        EQADC_ONCHIP_ADC_REG_ADDRESS_AGR2
};
static const eqadc_onchip_adc_reg_address_t aorAddr[2u] = {
        EQADC_ONCHIP_ADC_REG_ADDRESS_AOR1,
        EQADC_ONCHIP_ADC_REG_ADDRESS_AOR2
};

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetDefaultConfig
* Description   : Populates the config structure with default values.
* Config structure members which are pointer to arrays need to be initialized
* to point to memory allocated by the caller, before the function call.
* For each array, the function will populate with the number of elements set
* in the corresponding config struct member.
*
* Implements    : EQADC_DRV_GetDefaultConfig_Activity
* END**************************************************************************/
void EQADC_DRV_GetDefaultConfig(eqadc_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    uint8_t i;

    config->dbgReqMaskEn = false;
    config->digitalFilterLen = 0u;

    config->timebasePrescale = EQADC_TIMEBASE_PRESCALE_DISABLED;

    if(config->numAdcConfigs > 0u)
    {
        DEV_ASSERT(config->adcConfigArray != NULL);
        DEV_ASSERT(config->numAdcConfigs <= EQADC_NUM_ADC);
        eqadc_adc_config_t * adcConfig;

        for(i = 0u; i < config->numAdcConfigs; i++)
        {
            adcConfig = &(config->adcConfigArray[i]);
            adcConfig->adcIdx      = i;
            adcConfig->extMuxEn    = false;
            adcConfig->timebaseSel = EQADC_TIMEBASE_SEL_INTERNAL;
            adcConfig->clkPrescale = EQADC_PRESCALE_NOT_USED;
            adcConfig->clkOddPrescaleEn   = false;
            adcConfig->clkDutySel         = false;
            adcConfig->adcGain            = 0x4000u; /* = 1 in 1Q14. */
            adcConfig->adcOffset          = 0u;
            adcConfig->immediateConvCmdEn = false;
        }
    }
    else
    {
        config->adcConfigArray = NULL;
    }

    if(config->numCfifoConfigs > 0u)
    {
        DEV_ASSERT(config->cfifoConfigArray != NULL);
        DEV_ASSERT(config->numCfifoConfigs <= EQADC_CFPR_COUNT);
        eqadc_cfifo_config_t * cfifoConfig;

        for(i = 0u; i < config->numCfifoConfigs; i++)
        {
            cfifoConfig = &(config->cfifoConfigArray[i]);

            EQADC_GetDefaultCfifoConfig(cfifoConfig);
            cfifoConfig->cfifoIdx = i;
        }
    }
    else
    {
        config->cfifoConfigArray = NULL;
    }

    if(config->numResultDmaConfigs > 0u)
    {
        DEV_ASSERT(config->resultDmaConfigArray != NULL);
        DEV_ASSERT(config->numResultDmaConfigs <= EQADC_RFPR_COUNT);
        eqadc_result_dma_config_t * resultDmaConfig;

        for(i = 0u; i < config->numResultDmaConfigs; i++)
        {
            resultDmaConfig = &(config->resultDmaConfigArray[i]);

            /* These values cannot ensure a working configuration because:
             * - mapping of dmaVirtualChans to rfifo indexes is specific to EDMA and outside the scope of EQADC driver
             * - the destination buffer needs to be allocated externally by the caller */
            resultDmaConfig->rfifoIdx       = i;
            resultDmaConfig->dmaVirtualChan = 0u;
            resultDmaConfig->destPtr        = NULL;
            resultDmaConfig->destLength     = 0u;
            resultDmaConfig->callback       = NULL;
            resultDmaConfig->callbackParam  = NULL;
            resultDmaConfig->enHalfDestCallback = false;
        }
    }
    else
    {
        config->resultDmaConfigArray = NULL;
    }

    if(config->numAlternateConfigs > 0u)
    {
        DEV_ASSERT(config->alternateConfigArray != NULL);
        DEV_ASSERT(config->numAlternateConfigs <= EQADC_NUM_ALT_CONFIGS);
        eqadc_alternate_config_t * alternateConfig;

        for(i = 0u; i < config->numAlternateConfigs; i++)
        {
            alternateConfig = &(config->alternateConfigArray[i]);

            /* Default configuration to send result to RFIFOn selected via msgTag each conversion command */
            alternateConfig->altConfigSel  = EQADC_GetAltConfigSel(i);
            alternateConfig->resultInhibit = false; /* transfer to result queue is not inhibited */
            alternateConfig->dest          = 0u;    /* Do not send results to companion modules, unless extended alternate configurations are configured (=> DAM == 1).
                                                       First destination is RFIFOn selected via msgTag field from the conversion command.  */
            alternateConfig->signedResEn   = false; /* result is unsigned */
            alternateConfig->returnPSI     = false; /* indifferent, because results are not sent to companion modules */
            alternateConfig->adcResolution = EQADC_RESOLUTION_12_BITS;
            alternateConfig->timebaseSel   = EQADC_TIMEBASE_SEL_INTERNAL; /* Generation of timestamp results is controlled from each conversion command */
            alternateConfig->adcPregain    = EQADC_PREGAIN_X1;
            alternateConfig->adcGain[0u]   = 0x4000u; /* = 1 in Q15. Only used for alternate configuration index 1 and 2. Should be updated by user with values resulted from ADC calibration. */
            alternateConfig->adcGain[1u]   = 0x4000u; /* = 1 in Q15. Only used for alternate configuration index 1 and 2. Should be updated by user with values resulted from ADC calibration. */
            alternateConfig->adcOffset[0u] = 0u; /* Only used for alternate configuration index 1 and 2. Should be updated by user with values resulted from ADC calibration. */
            alternateConfig->adcOffset[1u] = 0u; /* Only used for alternate configuration index 1 and 2. Should be updated by user with values resulted from ADC calibration. */
        }
    }
    else
    {
        config->alternateConfigArray = NULL;
    }

    if(config->numExtAlternateConfigs > 0u)
    {
        DEV_ASSERT(config->extAlternateConfigArray != NULL);
        DEV_ASSERT(config->numExtAlternateConfigs <= EQADC_NUM_EXT_ALT_CONFIGS);
        eqadc_ext_alternate_config_t * extAlternateConfig;

        for(i = 0u; i < config->numExtAlternateConfigs; i++)
        {
            extAlternateConfig = &(config->extAlternateConfigArray[i]);

            /* This configuration assumes that corresponding alternate configuration has (ACR.DEST == 0) - is configured to select RFIFOn as first destination.
             * This is the default alternate configuration generated by GetDefaultConfiguration */
            extAlternateConfig->extAltConfigSel = EQADC_GetAltConfigSel(i);
            extAlternateConfig->resultInhibit2  = false; /* transfer to result queue is not inhibited */
            extAlternateConfig->dest2           = i;     /* second destination is selected to be companion module with index 'i' */
            extAlternateConfig->signedResEn2    = false; /* result sent to second destination is unsigned */
            extAlternateConfig->returnPSI2      = false; /* results from companion modules are sent to the same EQADC */
            extAlternateConfig->flushEn2        = false; /* no flush command is transmitted to altDest2*/
            extAlternateConfig->msgTagEn2       = false; /* do not use value in MsgTag2 */
            extAlternateConfig->msgTag2         = EQADC_MESSAGE_TAG_RFIFO0;
        }
    }
    else
    {
        config->extAlternateConfigArray = NULL;
    }

    if(config->numChanPullConfigs > 0u)
    {
        DEV_ASSERT(config->chanPullConfigArray != NULL);
        DEV_ASSERT(config->numChanPullConfigs <= EQADC_NUM_CHAN_PULL_CONFIGS);
        eqadc_chan_pull_config_t * chanPullConfig;

        for(i = 0u; i < config->numChanPullConfigs; i++)
        {
            chanPullConfig = &(config->chanPullConfigArray[i]);

            chanPullConfig->adcInputChanIdx  = i;
            chanPullConfig->chanPull         = EQADC_CHAN_PULL_UP_DOWN_EN;
            chanPullConfig->chanPullStrength = EQADC_PULL_STRENGTH_100KOHM;
        }
    }
    else
    {
        config->chanPullConfigArray = NULL;
    }

    if(config->numStacBusClientConfigs > 0u)
    {
        DEV_ASSERT(config->stacBusClientConfigArray != NULL);
        DEV_ASSERT(config->numStacBusClientConfigs <= EQADC_NUM_STAC_BUS_CLIENTS);
        eqadc_stac_bus_client_config_t * stacBusClientConfig;

        for(i = 0u; i < config->numStacBusClientConfigs; i++)
        {
            stacBusClientConfig = &(config->stacBusClientConfigArray[i]);

            stacBusClientConfig->clientIdx         = i;
            stacBusClientConfig->serverDataSlotSel = 0u;
            stacBusClientConfig->timebaseBitsSel   = EQADC_STAC_BITS_SEL_TBASE_0_15;
        }
    }
    else
    {
        config->stacBusClientConfigArray = NULL;
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_Init
* Description   : Initializes the selected EQADC instance general settings and
* on-chip ADC settings.
*
* Implements    : EQADC_DRV_Init_Activity
* END**************************************************************************/
void EQADC_DRV_Init(const uint32_t instance,
                    const eqadc_config_t * const config)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);

    EQADC_Type * const base = s_eqadcBase[instance];
    uint8_t i;

    EQADC_DRV_Reset(instance);

    /**** Configure global parameters from config struct ****/

    base->MCR   = (config->dbgReqMaskEn == true) ? (uint32_t)EQADC_MCR_DBG_MASK : (uint32_t)0u;
    base->ETDFR = EQADC_ETDFR_DFL(config->digitalFilterLen);

    eqadc_cfifo_config_t cfifoForCfgCmds; /* configuration for initializing the cfifo used for sending ADC configuration commands during Init() */
    EQADC_GetDefaultCfifoConfig(&cfifoForCfgCmds);
    cfifoForCfgCmds.cfifoIdx = EQADC_CFIFO_IDX_FOR_CONFIG_CMDS;
    cfifoForCfgCmds.opMode   = EQADC_CFIFO_MODE_SW_TRIG_SINGLE; /* each command will be sent via SW trigger */
    EQADC_ConfigCfifo(instance, &cfifoForCfgCmds);

    eqadc_config_cmd_t configCmd;
    EQADC_GetDefaultConfigCmd(&configCmd);

    /* Configure timebasePrescaler in TSCR on-chip register */
    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_TSCR;
    configCmd.adcRegValue   = EQADC_ADC_TSCR_TBC_CLK_PS(config->timebasePrescale);
    configCmd.cbufferNum    = 0u; /* TSCR can be configured by writing to any on-chip ADC module */

    /* send config cmd */
    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoForCfgCmds.cfifoIdx, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoForCfgCmds.cfifoIdx);  /* SW trigger to send the command */

    /**** Configure on-chip ADC properties ****/

    /* Must configure at least one on-chip ADC */
    DEV_ASSERT((config->numAdcConfigs > 0u) && (config->adcConfigArray != NULL));
    DEV_ASSERT(config->numAdcConfigs <= EQADC_NUM_ADC);
    const eqadc_adc_config_t * adcConfig;

#if (defined (DEV_ERROR_DETECT) || defined (CUSTOM_DEVASSERT))
    /* Check that the current input config doesn't have extMuxEn true for more than 1 on-chip ADC, without checking current register values.
     * Assumption: the extMuxEn bits of the on-chip ADCs have been already reset to 0u. */
    uint8_t numExtMuxEn = 0u;
    for(i = 0u; i < config->numAdcConfigs; i++)
    {
        numExtMuxEn += (config->adcConfigArray[i].extMuxEn == true) ? 1u : 0u;
    }
    /* extMuxEn cannot be enabled for more than 1 on-chip ADC at the same time. */
    DEV_ASSERT(numExtMuxEn <= 1u);
#endif

    uint8_t adcInitialized[EQADC_NUM_ADC] = {0u};
    for(i = 0; i < config->numAdcConfigs; i++)
    {
        adcConfig = &(config->adcConfigArray[i]);

        /* This function enables the selected ADC */
        EQADC_ConfigAdc(instance, adcConfig, cfifoForCfgCmds.cfifoIdx);

        adcInitialized[adcConfig->adcIdx] = 1u;
    }
    /* According to RM: due to legacy reasons, both ADC0 and ADC1 must be enabled for accurate conversion results.
     * So need to enable the on-chip ADC modules which were not initialized and thus are currently disabled. */
    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_CR; /* write to CR internal register */
    configCmd.adcRegValue   = EQADC_ADC_CR_EN_MASK; /* enable the on-chip ADC module */
    for(i = 0; i < EQADC_NUM_ADC; i++)
    {
        /* enable the rest of the ADC instances which were not configured */
        if(adcInitialized[i] != 1u)
        {
            configCmd.cbufferNum = i; /* command is sent to the selected on-chip ADC module */

            /* send config cmd */
            EQADC_DRV_PushCfifoConfigCmd(instance, cfifoForCfgCmds.cfifoIdx, &configCmd);
            EQADC_SetCfifoSSE(instance, cfifoForCfgCmds.cfifoIdx);  /* SW trigger to send the command */
        }
    }

    /**** Configure DMA for transferring results from rfifo to system memory ****/
    if(config->numResultDmaConfigs > 0u)
    {
        DEV_ASSERT(config->resultDmaConfigArray != NULL);
        DEV_ASSERT(config->numResultDmaConfigs <= EQADC_RFPR_COUNT);
        for(i = 0u; i < config->numResultDmaConfigs; i++)
        {
            EQADC_ConfigResultDma(instance, &(config->resultDmaConfigArray[i]));

            /* Configure EQADC to send DMA requests for RFIFO DRAIN events */
            EQADC_SetFifoIDCR(instance, config->resultDmaConfigArray[i].rfifoIdx, EQADC_IDCR0_RFDS1_MASK);
            EQADC_DRV_EnableDmaReq(instance, config->resultDmaConfigArray[i].rfifoIdx, EQADC_DMA_REQ_EN_RFIFO_DRAIN);
        }
    }

    /**** Configure alternate configurations ****/
    if(config->numAlternateConfigs > 0u)
    {
        DEV_ASSERT(config->alternateConfigArray != NULL);
        DEV_ASSERT(config->numAlternateConfigs <= EQADC_NUM_ALT_CONFIGS);
        for(i = 0; i < config->numAlternateConfigs; i++)
        {
            EQADC_ConfigAltConfiguration(instance, &(config->alternateConfigArray[i]), cfifoForCfgCmds.cfifoIdx);
        }
    }

    /**** Configure extended alternate configurations ****/
    if(config->numExtAlternateConfigs > 0u)
    {
        /* Alternate Configuration Destination Addressing Mode
         * Select the Extended Alternate Configuration Registers to send result to two destinations.
         * These destinations can be RFIFOs and companion modules connected to the PSI.
         * Second destination is selected via extended alternate configurations (ADC_EACR1-14). */
        base->MCR |= EQADC_MCR_DAM_MASK; /* enabled when extended alternate configs exist */

        DEV_ASSERT(config->extAlternateConfigArray != NULL);
        DEV_ASSERT(config->numExtAlternateConfigs <= EQADC_NUM_EXT_ALT_CONFIGS);
        for(i = 0; i < config->numExtAlternateConfigs; i++)
        {
            EQADC_ConfigExtAltConfiguration(instance, &(config->extAlternateConfigArray[i]), cfifoForCfgCmds.cfifoIdx);
        }
    }
    else
    {
        /* Alternate Configuration Destination Addressing Mode
         * Select Legacy mode. Uses a defined set of pairs of companion module destinations
         * in the PSI as defined byTable 30-121 (ADC_ACR1-14). The ADC_EACR1-14 registers are not considered.*/
        base->MCR &= ~EQADC_MCR_DAM_MASK;
    }

    /**** Configure channel pull resistors ****/
    if(config->numChanPullConfigs > 0u)
    {
        DEV_ASSERT(config->chanPullConfigArray != NULL);
        DEV_ASSERT(config->numChanPullConfigs <= EQADC_NUM_CHAN_PULL_CONFIGS);
        for(i = 0; i < config->numChanPullConfigs; i++)
        {
            EQADC_ConfigChanPull(instance, &(config->chanPullConfigArray[i]), cfifoForCfgCmds.cfifoIdx);
        }
    }

    /**** Configure STAC bus clients ****/
    if(config->numStacBusClientConfigs > 0u)
    {
        DEV_ASSERT(config->stacBusClientConfigArray != NULL);
        DEV_ASSERT(config->numStacBusClientConfigs <= EQADC_NUM_STAC_BUS_CLIENTS);
        for(i = 0; i < config->numStacBusClientConfigs; i++)
        {
            EQADC_ConfigStacBusClient(instance, &(config->stacBusClientConfigArray[i]));
        }
    }

    /**** Configure cfifo properties ****/
    /* Configure at the end of EQADC_DRV_Init() to make sure that
     * the CFIFO used for sending configuration commands gets reconfigured */
    /* Must configure at least one cfifo */
    DEV_ASSERT((config->numCfifoConfigs > 0u) && (config->cfifoConfigArray != NULL));
    DEV_ASSERT(config->numCfifoConfigs <= EQADC_CFPR_COUNT);
    for(i = 0; i < config->numCfifoConfigs; i++)
    {
        EQADC_ConfigCfifo(instance, &(config->cfifoConfigArray[i]));
    }

    /**** Clear all status flags excepting:
     * - EQADC_FIFO_STATUS_FLAG_CFIFO_FILL which has reset value '1'
     * - EQADC_FIFO_STATUS_FLAG_CFIFO_FILL for CFIFOs with DMA disabled
     * - EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN for RFIFOs with DMA enabled */
    uint32_t flagMask;
    for(i = 0u; i < EQADC_FISR_COUNT; i++)
    {
        flagMask = EQADC_FIFO_STATUS_FLAG_ALL & ~EQADC_FIFO_STATUS_FLAG_CFIFO_FILL;

        /* EQADC_FIFO_STATUS_SEL_CFIFO_FILL (FISR CFFFX) can only be cleared, if DMA transfer is not enabled for filling the cfifo */
        if((EQADC_GetFifoIDCR(instance, i) & EQADC_IDCR0_CFFS1_MASK) != 0u)
        {
            flagMask &= ~EQADC_FIFO_STATUS_FLAG_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
        /* EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN (FISR RFDFX) can only be cleared, if DMA transfer is not enabled for draining the rfifo */
        if((EQADC_GetFifoIDCR(instance, i) & EQADC_IDCR0_RFDS1_MASK) != 0u)
        {
            flagMask &= ~EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
        EQADC_DRV_ClearFifoStatus(instance, i, flagMask);
    }

    /**** Reset CFTCRx registers ****/
    for(i = 0u; i < EQADC_CFPR_COUNT; i++)
    {
        EQADC_SetCfifoTC_CF(instance, i, 0u);
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_Reset
* Description   : The function resets to default the general EQADC settings and on-chip ADC settings.
* The function shall disable the on-chip ADC instances.
*
* Implements    : EQADC_DRV_Reset_Activity
* END**************************************************************************/
void EQADC_DRV_Reset(const uint32_t instance)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);

    uint8_t i;
    const uint8_t cfifoIdxForCmds = EQADC_CFIFO_IDX_FOR_CONFIG_CMDS;

    /* Reset the cfifo used for sending the commands by the rest of the reset functions */
    EQADC_ResetCfifo(instance, cfifoIdxForCmds);

    eqadc_cfifo_config_t cfifoForCfgCmds;
    EQADC_GetDefaultCfifoConfig(&cfifoForCfgCmds);
    cfifoForCfgCmds.cfifoIdx = cfifoIdxForCmds;
    cfifoForCfgCmds.opMode   = EQADC_CFIFO_MODE_SW_TRIG_SINGLE; /* each command will be sent via SW trigger */
    EQADC_ConfigCfifo(instance, &cfifoForCfgCmds);

    for(i = 0u; i < EQADC_NUM_ADC; i++)
    {
        EQADC_ResetAdc(instance, i, cfifoIdxForCmds);
    }

    for(i = 0u; i < EQADC_RFPR_COUNT; i++)
    {
        status_t status;

        /* Stop DMA virtual channel and reset the EQADC state storing DMA virtual channel used for rfifo  */
        if(dmaResultVirtualChans[i] != EQADC_INVALID_DMA_VIRT_CHAN)
        {
            status = EDMA_DRV_StopChannel(dmaResultVirtualChans[i]);
            (void) status;
            dmaResultVirtualChans[i] = EQADC_INVALID_DMA_VIRT_CHAN;
        }

        /* reset EQADC bits and flags affecting rfifo interrupts and DMA */
        EQADC_ClearFifoIDCR(instance, i, EQADC_IDCR0_RFDS1_MASK);
        const uint16_t mask = (uint16_t)(EQADC_INT_EN_RFIFO_OVERFLOW | EQADC_INT_EN_RFIFO_DRAIN);
        EQADC_ClearFifoIDCR(instance, i, mask);

        /* flush RFIFO using default timeout of 2 ms */
        status = EQADC_DRV_FlushRfifo(instance, i, 2u /*ms*/);
        (void) status;
    }

    for(i = 0u; i < EQADC_NUM_ALT_CONFIGS; i++)
    {
        EQADC_ResetAltConfiguration(instance, EQADC_GetAltConfigSel(i), cfifoIdxForCmds);
    }

    for(i = 0u; i < EQADC_NUM_EXT_ALT_CONFIGS; i++)
    {
        EQADC_ResetExtAltConfiguration(instance, EQADC_GetAltConfigSel(i), cfifoIdxForCmds);
    }

    for(i = 0u; i < EQADC_NUM_CHAN_PULL_CONFIGS; i++)
    {
        EQADC_ResetChanPull(instance, i, cfifoIdxForCmds);
    }

    for(i = 0u; i < EQADC_NUM_STAC_BUS_CLIENTS; i++)
    {
        EQADC_ResetStacBusClient(instance, i);
    }
    /* Reset time stamp configuration */
    EQADC_ResetTimeStampConfiguration(instance, cfifoIdxForCmds);

    /* Reset all cfifos (including the one used by the rest of the reset functions) */
    for(i = 0u; i < EQADC_CFPR_COUNT; i++)
    {
        EQADC_ResetCfifo(instance, i);
    }

    /* Clear all status flags except EQADC_FIFO_STATUS_FLAG_CFIFO_FILL which has reset value '1' and needs to remain set
     * because the cfifos are empty */
    const uint32_t flagMask = EQADC_FIFO_STATUS_FLAG_ALL & ~EQADC_FIFO_STATUS_FLAG_CFIFO_FILL;
    for(i = 0u; i < EQADC_FISR_COUNT; i++)
    {
        EQADC_DRV_ClearFifoStatus(instance, i, flagMask);
    }

    /* Reset CFTCRx registers */
    for(i = 0u; i < EQADC_CFPR_COUNT; i++)
    {
        EQADC_SetCfifoTC_CF(instance, i, 0u);
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_DoCalibration
* Description   : Calibrates the selected ADC module for the selected EQADC instance.
*
* Implements    : EQADC_DRV_DoCalibration_Activity
* END**************************************************************************/
status_t EQADC_DRV_DoCalibration(const uint32_t instance,
                                 const uint8_t adcIdx,
                                 eqadc_calibration_params_t * const calibParam)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(adcIdx < EQADC_NUM_ADC);
    DEV_ASSERT(calibParam != NULL);

    eqadc_config_cmd_t configCmd;
    eqadc_conv_cmd_t convCmd;
    uint16_t result75proc, result25proc;
    uint16_t result;
    uint8_t resFifoNumEntries;
    const uint32_t rfifoIdx = calibParam->rfifoIdx;
    const uint32_t cfifoIdx = calibParam->cfifoIdx;
    const uint32_t timeout  = calibParam->timeout;
    uint32_t start, delta;
    status_t status = STATUS_SUCCESS;

    DEV_ASSERT(rfifoIdx < EQADC_RFPR_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    /* Make sure OSIF timer is initialized. */
    OSIF_TimeDelay(0u);

    start = OSIF_GetMilliseconds();
    delta = 0u;

    /* Save RFIFO Drain Event Enable - trigger DMA or interrupt, when a result is available in the rfifo */
    const uint16_t rfifoDrainEventMask = EQADC_GetFifoIDCR(instance, rfifoIdx) & EQADC_IDCR0_RFDE1_MASK;
    /* Disable RFIFO Drain Event Enable - calibration procedure polls the flag */
    EQADC_ClearFifoIDCR(instance, rfifoIdx, EQADC_IDCR0_RFDE1_MASK);

    /* Save RFIFO Drain Select - either DMA or interrupt selected for draining rfifo */
    const uint16_t rfifoDrainSelectMask = EQADC_GetFifoIDCR(instance, rfifoIdx) & EQADC_IDCR0_RFDS1_MASK;

    /* If DMA is selected for RFIFO, disable DMA transfers and configure for use with interrupt, to allow reseting of RFIFO DRAIN flag from software */
    if(rfifoDrainSelectMask != 0u)
    {
        /* Stop DMA virtual channel */
        if(dmaResultVirtualChans[rfifoIdx] != EQADC_INVALID_DMA_VIRT_CHAN)
        {
            status = EDMA_DRV_StopChannel(dmaResultVirtualChans[rfifoIdx]);
            (void) status;
        }

        EQADC_ClearFifoIDCR(instance, rfifoIdx, EQADC_IDCR0_RFDS1_MASK);
    }

    /* Save operating mode before calibration and make sure the cfifo is operating in SW triggered single scan mode */
    const uint32_t prevOpMode = EQADC_GetCfifoOpMode(instance, cfifoIdx);
    if(prevOpMode != (uint32_t)EQADC_CFIFO_MODE_SW_TRIG_SINGLE)
    {
        /* Cannot change directly the operating mode - must be disabled first */
        EQADC_SetCfifoOpMode(instance, cfifoIdx, EQADC_CFIFO_MODE_DISABLED);
        DEV_ASSERT(EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx) == CFIFO_STATUS_IDLE);
        /* Make sure the CFIFO has no command */
        EQADC_SetCfifoInv(instance, cfifoIdx);
        EQADC_SetCfifoOpMode(instance, cfifoIdx, EQADC_CFIFO_MODE_SW_TRIG_SINGLE);
    }

    /* For safe execution, make sure the RFIFO is empty, by reading all results (if any) and discarding them */
    resFifoNumEntries = EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR);
    while((resFifoNumEntries != 0u) && (delta < timeout))
    {
        result = EQADC_DRV_PopRfifoData(instance, rfifoIdx);
        (void) result; /* Discard results */
        resFifoNumEntries = EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR);

        delta = OSIF_GetMilliseconds() - start;
    }

    if(delta >= timeout)
    {
        status = STATUS_TIMEOUT;
    }
    else
    {
        /* Make sure the flag for reading results from RFIFO is not set */
        EQADC_DRV_ClearFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN);

        /* Init conversion command */
        convCmd.eoq            = true;
        convCmd.pause          = false;
        convCmd.repeatStart    = false;
        convCmd.cbufferNum     = adcIdx;
        convCmd.calibEn        = false;
        convCmd.msgTag         = (eqadc_message_tag_t) rfifoIdx;
        convCmd.samplingTime   = calibParam->samplingTime;
        convCmd.timeStampReq   = false;
        convCmd.channelNum     = EQADC_CHAN_VREF_25_PROC;
        convCmd.signEn         = false;
        convCmd.altConfigSel   = (eqadc_alt_config_sel_t) calibParam->calibTarget; /* Select which set of gain and offset to calibrate */
        convCmd.flushCompanion = false;

        /* Measure 25% VREF */
        EQADC_DRV_PushCfifoConvCmd(instance, cfifoIdx, &convCmd);
        EQADC_SetCfifoSSE(instance, cfifoIdx);  /* SW trigger to send the command */

        /* Wait for result to be ready in RFIFO or timeout to occur */
        delta = OSIF_GetMilliseconds() - start;
        while((EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN) == 0u) && (delta < timeout))
        {
            delta = OSIF_GetMilliseconds() - start;
        }

        if(delta >= timeout)
        {
            status = STATUS_TIMEOUT;
        }
        else
        {
            result25proc = EQADC_DRV_PopRfifoData(instance, rfifoIdx);
            EQADC_DRV_ClearFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN);
            EQADC_DRV_ClearFifoStatus(instance, cfifoIdx, EQADC_FIFO_STATUS_FLAG_END_OF_QUEUE);

            /* Measure 75% VREF */
            convCmd.channelNum = EQADC_CHAN_VREF_75_PROC;
            EQADC_DRV_PushCfifoConvCmd(instance, cfifoIdx, &convCmd);
            EQADC_SetCfifoSSE(instance, cfifoIdx);  /* SW trigger to send the command */

            /* Wait for result to be ready in RFIFO or timeout to occur */
            delta = OSIF_GetMilliseconds() - start;
            while((EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN) == 0u) && (delta < timeout))
            {
                delta = OSIF_GetMilliseconds() - start;
            }

            if(delta >= timeout)
            {
                status = STATUS_TIMEOUT;
            }
            else
            {
                result75proc = EQADC_DRV_PopRfifoData(instance, rfifoIdx);
                EQADC_DRV_ClearFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN);

                /* Calculate gain and offset */

                /* Q14 division of: (float)(EQADC_CAL_VREF_75_PROC - EQADC_CAL_VREF_25_PROC))/((float)(result75proc - result25proc) -> formula taken from reference manual */
                const uint64_t numeratorQ14 = (EQADC_CAL_VREF_75_PROC - EQADC_CAL_VREF_25_PROC) << EQADC_GAIN_NUM_PRECISION_BITS;
                const uint64_t denominatorQ14 = ((uint64_t)result75proc - (uint64_t)result25proc) << EQADC_GAIN_NUM_PRECISION_BITS;
                const uint64_t intermQ28 = numeratorQ14 << EQADC_GAIN_NUM_PRECISION_BITS;
                const uint64_t gainQ14 = (intermQ28 + (denominatorQ14 >> 1u)) / denominatorQ14; /* calculate with rounding */
                uint16_t gainQ14saturated = (uint16_t) gainQ14;
                if(gainQ14 > EQADC_ADC_GCCR_GCC_MASK)
                {
                    gainQ14saturated = EQADC_ADC_GCCR_GCC_MASK; /* saturate gain to maximum value allowed by register */
                    status = STATUS_ERROR;
                }

                /* Q14 multiplication of: (EQADC_CAL_VREF_75_PROC - gain*result75proc - 2u) -> formula taken from reference manual */
                const uint64_t aQ14 = (EQADC_CAL_VREF_75_PROC - 2u) << EQADC_GAIN_NUM_PRECISION_BITS;
                const int64_t offsetQ14 = aQ14 - (gainQ14 * ((uint64_t)result75proc));
                const int16_t offset = (int16_t)(offsetQ14 >> EQADC_GAIN_NUM_PRECISION_BITS); /* offset is a signed 14-bit integer value. Negative values should be expressed using two's complement */

                EQADC_GetDefaultConfigCmd(&configCmd);
                configCmd.type        = EQADC_CONFIG_CMD_TYPE_WRITE;
                configCmd.cbufferNum  = adcIdx; /* command is sent to the selected on-chip ADC module */

                /* Write ADC gain */
                EQADC_WriteGainToConfigCmd(&configCmd, calibParam->calibTarget, gainQ14saturated);

                EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdx, &configCmd);
                EQADC_SetCfifoSSE(instance, cfifoIdx);  /* SW trigger to send the command */

                /* Write ADC offset */
                EQADC_WriteOffsetToConfigCmd(&configCmd, calibParam->calibTarget, offset);

                EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdx, &configCmd);
                EQADC_SetCfifoSSE(instance, cfifoIdx);  /* SW trigger to send the command */

                /* Write results to output structure */
                calibParam->adcGain   = gainQ14saturated;
                calibParam->adcOffset = offset;

                /* Do not wait for configuration commands to get executed (by reading the corresponding internal register),
                 * because the function assumes that the ADC module is not used for other conversions during calibration.  */

                /* Clear EOQ flag for the corresponding CFIFO, after all commands have been pushed */
                EQADC_DRV_ClearFifoStatus(instance, cfifoIdx, EQADC_FIFO_STATUS_FLAG_END_OF_QUEUE);

                /* Clear CFTCRx */
                EQADC_SetCfifoTC_CF(instance, cfifoIdx, 0u);
            }
        }
    }

    /* Restore operating mode */
    if(prevOpMode != (uint32_t)EQADC_CFIFO_MODE_SW_TRIG_SINGLE)
    {
        /* Cannot change directly the operating mode - must be disabled first */
        EQADC_SetCfifoOpMode(instance, cfifoIdx, EQADC_CFIFO_MODE_DISABLED);
        DEV_ASSERT(EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx) == CFIFO_STATUS_IDLE);

        EQADC_SetCfifoOpMode(instance, cfifoIdx, prevOpMode);
    }

    /* Restore RFIFO drain select and re-enable DMA transfers */
    if(rfifoDrainSelectMask != 0u)
    {
        /* Start DMA virtual channel */
        status = EDMA_DRV_StartChannel(dmaResultVirtualChans[rfifoIdx]);
        (void) status;

        EQADC_SetFifoIDCR(instance, rfifoIdx, EQADC_IDCR0_RFDS1_MASK);
    }

    /* Restore RFIFO Drain Event Enable - trigger DMA or interrupt, when a result is available in the rfifo */
    if(rfifoDrainEventMask != 0u)
    {
        EQADC_SetFifoIDCR(instance, rfifoIdx, EQADC_IDCR0_RFDE1_MASK);
    }

    return status;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_SetSingleScanEnBit
* Description   : Sets the Single-Scan Enable (SSE) bit for the selected cfifo or cfifo pair (if inSync is set to true)
*
* Implements    : EQADC_DRV_SetSingleScanEnBit_Activity
* END**************************************************************************/
void EQADC_DRV_SetSingleScanEnBit(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const bool inSync)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    if(inSync == false)
    {
        /* Set SSE bit only for the selected cfifo */
        EQADC_SetCfifoSSE(instance, cfifoIdx);
    }
    else
    {
        /* Set SSE bit for the selected cfifo and its pair: cfifo(2N) and cfifo(2N+1) */
        EQADC_SetCfifoSSEPair(instance, cfifoIdx);
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_EnableImmediateConvCmd
* Description   : This function enables Immediate Conversion feature for CFIFO0.
* When enabled, the EQADC aborts any current conversions and immediately starts the conversion commands from CFIFO0.
* The commands which were already executing or pending, will (re)start execution after commands from CFIFO0 have completed.
* When disabled, the EQADC executes commands in normal priority scheme. When CFIFO0 is triggered, its conversion command can be put
* behind 2 pending conversion commands in the Cbuffer, commands which might be from lower priority cfifos.
*
* Implements    : EQADC_DRV_EnableImmediateConvCmd_Activity
* END**************************************************************************/
void EQADC_DRV_EnableImmediateConvCmd(const uint32_t instance,
                                      const uint32_t adcIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);

    EQADC_Type * const base = s_eqadcBase[instance];

    switch(adcIdx)
    {
        case 0u:
            base->MCR |= EQADC_MCR_ICEA0_MASK;
            break;
        case 1u:
            base->MCR |= EQADC_MCR_ICEA1_MASK;
            break;
        default:
            DEV_ASSERT(false); /* invalid value for adcIdx parameter */
            break;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_DisableImmediateConvCmd
* Description   : This function disables Immediate Conversion feature for CFIFO0.
* When disabled, the EQADC executes commands in normal priority scheme. When CFIFO0 is triggered, its conversion command can be put
* behind 2 pending conversion commands in the Cbuffer, commands which might be from lower priority cfifos.
* When enabled, the EQADC aborts any current conversions and immediately starts the conversion commands from CFIFO0.
* The commands which were already executing or pending, will (re)start execution after commands from CFIFO0 have completed.
*
* Implements    : EQADC_DRV_DisableImmediateConvCmd_Activity
* END**************************************************************************/
void EQADC_DRV_DisableImmediateConvCmd(const uint32_t instance,
                                       const uint32_t adcIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);

    EQADC_Type * const base = s_eqadcBase[instance];

    switch(adcIdx)
    {
        case 0u:
            base->MCR &= ~EQADC_MCR_ICEA0_MASK;
            break;
        case 1u:
            base->MCR &= ~EQADC_MCR_ICEA1_MASK;
            break;
        default:
            DEV_ASSERT(false); /* invalid value for adcIdx parameter */
            break;
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetFifoStatus
* Description   : The function returns the selected status information for a fifo
* (command or result fifos, depending on the status info selected).
* The function may return single or multiple bits, depending on the selected status information.
*
* Implements    : EQADC_DRV_GetFifoStatus_Activity
* END**************************************************************************/
uint8_t EQADC_DRV_GetFifoStatus(const uint32_t instance,
                                const uint32_t fifoIdx,
                                const eqadc_fifo_status_sel_t statusInfoSel)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_FISR_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t statusInfo = 0u;

    switch(statusInfoSel)
    {
        case EQADC_FIFO_STATUS_SEL_NON_COHERENCY:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_NCFX_MASK) >> EQADC_FISR_NCFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_TRIG_OVERRUN:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_TORFX_MASK) >> EQADC_FISR_TORFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_PAUSE:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_PFX_MASK) >> EQADC_FISR_PFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_END_OF_QUEUE:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_EOQFX_MASK) >> EQADC_FISR_EOQFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_CFIFO_UNDERFLOW:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_CFUFX_MASK) >> EQADC_FISR_CFUFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_CFIFO_SINGLE_SCAN:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_SSSX_MASK) >> EQADC_FISR_SSSX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_CFIFO_FILL:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_CFFFX_MASK) >> EQADC_FISR_CFFFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_RFIFO_OVERFLOW:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_RFOFX_MASK) >> EQADC_FISR_RFOFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_RFDFX_MASK) >> EQADC_FISR_RFDFX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_CFIFO_ENTRY_CNTR:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_CFCTRX_MASK) >> EQADC_FISR_CFCTRX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_CFIFO_TRANS_NEXT_PTR:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_TNXTPTRX_MASK) >> EQADC_FISR_TNXTPTRX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_RFCTRX_MASK) >> EQADC_FISR_RFCTRX_SHIFT;
            break;
        case EQADC_FIFO_STATUS_SEL_RFIFO_POP_NEXT_PTR:
            statusInfo = (base->FISR[fifoIdx] & EQADC_FISR_POPNXTPTRX_MASK) >> EQADC_FISR_POPNXTPTRX_SHIFT;
            break;
        default:
            /* value outside enum range is ignored */
            DEV_ASSERT(false);
            break;
    }

    return (uint8_t) (statusInfo & 0xFFu);
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_ClearFifoStatus
* Description   : The function clears the status flags corresponding to the bits enabled in the mask input parameter.
* The selected flags may correspond to both command or result fifos.
* The 'bitmask' input parameter can be set using EQADC_FIFO_STATUS_FLAG_ defines.
*
* NOTE: some flags can only be cleared in certain conditions
* e.g. CFIFO FILL and RFIFO DRAIN can only be cleared if DMA for selected fifo is disabled.
*
* Implements    : EQADC_DRV_ClearFifoStatus_Activity
* END**************************************************************************/
void EQADC_DRV_ClearFifoStatus(const uint32_t instance,
                               const uint32_t fifoIdx,
                               uint32_t bitmask)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_FISR_COUNT);
    DEV_ASSERT((bitmask & ~EQADC_FIFO_STATUS_FLAG_ALL) == 0u);

    EQADC_Type * const base = s_eqadcBase[instance];

    if((bitmask & EQADC_FIFO_STATUS_FLAG_CFIFO_FILL) != 0u)
    {
        /* EQADC_FIFO_STATUS_SEL_CFIFO_FILL (FISR CFFFX) can only be cleared, if DMA transfer is not enabled for filling the cfifo */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_CFFS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_FIFO_STATUS_FLAG_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }
    if((bitmask & EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN) != 0u)
    {
        /* EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN (FISR RFDFX) can only be cleared, if DMA transfer is not enabled for draining the rfifo */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_RFDS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }

    base->FISR[fifoIdx] = bitmask; /* w1c */
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_EnableIntReq
* Description   : Enables one or more interrupt request sources of a fifo
* (command or result fifo, depending on the interrupt source selected).
* The function accepts as parameter a bitmask selecting which of the interrupts to be enabled.
* The bitmask parameter should be set using API defines EQADC_INT_EN_.
*
* Implements    : EQADC_DRV_EnableIntReq_Activity
* END**************************************************************************/
void EQADC_DRV_EnableIntReq(const uint32_t instance,
                            const uint32_t fifoIdx,
                            uint16_t bitmask)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT((bitmask & ~EQADC_INT_EN_ALL) == 0u);

    if((bitmask & EQADC_INT_EN_RFIFO_DRAIN) != 0u)
    {
        /* INT_EN_RFIFO_DRAIN can only be written if DMA transfer is not enabled for draining rfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_RFDS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_INT_EN_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }
    if((bitmask & EQADC_INT_EN_CFIFO_FILL) != 0u)
    {
        /* INT_EN_CFIFO_FILL can only be written if DMA transfer is not enabled for filling cfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_CFFS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_INT_EN_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }

    EQADC_SetFifoIDCR(instance, fifoIdx, bitmask);
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_DisableIntReq
* Description   :  Disables one or more interrupt request sources of a fifo
* (command or result fifo, depending on the interrupt source selected).
* The function accepts as parameter a bitmask selecting which of the interrupts to be enabled.
* The bitmask parameter should be set using API defines EQADC_INT_EN_.
*
* Implements    : EQADC_DRV_DisableIntReq_Activity
* END**************************************************************************/
void EQADC_DRV_DisableIntReq(const uint32_t instance,
                             const uint32_t fifoIdx,
                             uint16_t bitmask)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT((bitmask & ~EQADC_INT_EN_ALL) == 0u);

    if((bitmask & EQADC_INT_EN_RFIFO_DRAIN) != 0u)
    {
        /* INT_EN_RFIFO_DRAIN can only be written if DMA transfer is not enabled for draining rfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_RFDS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_INT_EN_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }
    if((bitmask & EQADC_INT_EN_CFIFO_FILL) != 0u)
    {
        /* INT_EN_CFIFO_FILL can only be written if DMA transfer is not enabled for filling cfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_CFFS1_MASK) != 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_INT_EN_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }

    EQADC_ClearFifoIDCR(instance, fifoIdx, bitmask);
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_EnableDmaReq
* Description   : Enables one or more events for which a fifo issues DMA requests
* (command or result fifo, depending on the interrupt source selected).
* The function accepts as parameter a bitmask selecting which of the requests to be enabled.
* The bitmask parameter should be set using API defines EQADC_DMA_REQ_EN_.
*
* Implements    : EQADC_DRV_EnableDmaReq_Activity
* END**************************************************************************/
void EQADC_DRV_EnableDmaReq(const uint32_t instance,
                            const uint32_t fifoIdx,
                            uint16_t bitmask)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT((bitmask & ~EQADC_DMA_REQ_EN_ALL) == 0u);

    if((bitmask & EQADC_DMA_REQ_EN_RFIFO_DRAIN) != 0u)
    {
        /* DMA_REQ_EN_RFIFO_DRAIN can only be written if DMA transfer is enabled for draining rfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_RFDS1_MASK) == 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_DMA_REQ_EN_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }
    if((bitmask & EQADC_DMA_REQ_EN_CFIFO_FILL) != 0u)
    {
        /* DMA_REQ_EN_CFIFO_FILL can only be written if DMA transfer is enabled for filling cfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_CFFS1_MASK) == 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_DMA_REQ_EN_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }

    EQADC_SetFifoIDCR(instance, fifoIdx, bitmask);
}

/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_DisableDmaReq
* Description   : Disables one or more events for which a fifo issues DMA requests
* (command or result fifo, depending on the interrupt source selected).
* The function accepts as parameter a bitmask selecting which of the requests to be disabled.
* The bitmask parameter should be set using API defines EQADC_DMA_REQ_EN_.
*
* Implements    : EQADC_DRV_DisableDmaReq_Activity
* END**************************************************************************/
void EQADC_DRV_DisableDmaReq(const uint32_t instance,
                             const uint32_t fifoIdx,
                             uint16_t bitmask)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(fifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT((bitmask & ~EQADC_DMA_REQ_EN_ALL) == 0u);

    if((bitmask & EQADC_DMA_REQ_EN_RFIFO_DRAIN) != 0u)
    {
        /* DMA_REQ_EN_RFIFO_DRAIN can only be written if DMA transfer is enabled for draining rfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_RFDS1_MASK) == 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_DMA_REQ_EN_RFIFO_DRAIN; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }
    if((bitmask & EQADC_DMA_REQ_EN_CFIFO_FILL) != 0u)
    {
        /* DMA_REQ_EN_CFIFO_FILL can only be written if DMA transfer is enabled for filling cfifos */
        if((EQADC_GetFifoIDCR(instance, fifoIdx) & EQADC_IDCR0_CFFS1_MASK) == 0u)
        {
            DEV_ASSERT(false);
            bitmask &= ~EQADC_DMA_REQ_EN_CFIFO_FILL; /* If DMA is enabled, clear the flag from bitmask to avoid changing its state */
        }
    }

    EQADC_ClearFifoIDCR(instance, fifoIdx, bitmask);
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_PushCfifoConvCmd
* Description   : Pushes the conversion command in the selected command fifo (cfifo).
*
* Implements    : EQADC_DRV_PushCfifoConvCmd_Activity
* END**************************************************************************/
void EQADC_DRV_PushCfifoConvCmd(const uint32_t instance,
                                const uint32_t cfifoIdx,
                                const eqadc_conv_cmd_t * const convCmd)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT(convCmd != NULL);

    EQADC_Type * const base = s_eqadcBase[instance];

    base->CFPR[cfifoIdx] = EQADC_GetCfifoConvCmd(convCmd);
}


/* FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_WriteMemConvCmd
 * Description   : Writes the array of conversion commands from convCmdArray to
 * the memory location pointed by cQueueDest. The command struct members
 * are demapped to the command register bitfields, to be prepared for DMA transfer.
 *
 * Implements    : EQADC_DRV_WriteMemConvCmd_Activity
 * END**************************************************************************/
void EQADC_DRV_WriteMemConvCmd(const eqadc_conv_cmd_t * const convCmdArray,
                               uint32_t * const cQueueDest,
                               const uint8_t numCmds)
{
    DEV_ASSERT(convCmdArray != NULL);
    DEV_ASSERT(cQueueDest != NULL);

    uint8_t i;

    for(i = 0u; i < numCmds; i++)
    {
        cQueueDest[i] = EQADC_GetCfifoConvCmd(&(convCmdArray[i]));
    }
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_PushCfifoConfigCmd
* Description   : Pushes the configuration command in the selected command fifo (cfifo).
*
* Implements    : EQADC_DRV_PushCfifoConfigCmd_Activity
* END**************************************************************************/
void EQADC_DRV_PushCfifoConfigCmd(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const eqadc_config_cmd_t * const configCmd)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT(configCmd != NULL);

    EQADC_Type * const base = s_eqadcBase[instance];

    base->CFPR[cfifoIdx] = EQADC_GetCfifoConfigCmd(configCmd);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_WriteMemConfigCmd
 * Description   : Writes the array of configuration commands from configCmdArray
 * to the memory location pointed by cQueueDest.
 * The command struct members are demapped to the command register bitfields,
 * to be prepared for DMA transfer.
 *
 * Implements    : EQADC_DRV_WriteMemConfigCmd_Activity
 * END**************************************************************************/
void EQADC_DRV_WriteMemConfigCmd(const eqadc_config_cmd_t * const configCmdArray,
                                 uint32_t * const cQueueDest,
                                 const uint8_t numCmds)
{
    DEV_ASSERT(configCmdArray != NULL);
    DEV_ASSERT(cQueueDest != NULL);

    uint8_t i;

    for(i = 0u; i < numCmds; i++)
    {
        cQueueDest[i] = EQADC_GetCfifoConfigCmd(&(configCmdArray[i]));
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_PopRfifoData
 * Description   : Pops one result from the selected result fifo (rfifo).
 *
 * Implements    : EQADC_DRV_PopRfifoData_Activity
 * END**************************************************************************/
uint16_t EQADC_DRV_PopRfifoData(const uint32_t instance,
                                const uint32_t rfifoIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(rfifoIdx < EQADC_RFPR_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint16_t data;

    data = (uint16_t) (base->RFPR[rfifoIdx] & EQADC_RFPR_RF_POPX_MASK);

    return data;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetCfifoEntry
* Description   : The function returns the value of a selected command fifo entry (cfifo),
* from corresponding CFxRn and CF0Rn registers.
*
* Implements    : EQADC_DRV_GetCfifoEntry_Activity
* END**************************************************************************/
uint32_t EQADC_DRV_GetCfifoEntry(const uint32_t instance,
                                 const uint32_t cfifoIdx,
                                 const uint32_t entryIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t data;

    if((cfifoIdx == 0u) && (entryIdx >= EQADC_CF0R_COUNT))
    {
        DEV_ASSERT(entryIdx < (EQADC_CF0R_COUNT + EQADC_CF0ER_COUNT));
        data = base->CF0ER[entryIdx - EQADC_CF0R_COUNT];
    }
    else
    {
        DEV_ASSERT(entryIdx < EQADC_CF0R_COUNT);
        data = EQADC_GetCfifoCFxR(instance, cfifoIdx, entryIdx);
    }

    return data;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetRfifoEntry
* Description   : The function returns the value of a selected result fifo entry (rfifo),
* from corresponding RFxRn register.
* Calling this function does not pop the result from the RFIFO.
*
* Implements    : EQADC_DRV_GetRfifoEntry_Activity
* END**************************************************************************/
uint16_t EQADC_DRV_GetRfifoEntry(const uint32_t instance,
                                 const uint32_t rfifoIdx,
                                 const uint32_t entryIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(rfifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT(entryIdx < EQADC_RF0R_COUNT);

    uint16_t data;

    data = EQADC_GetRfifoRFxR(instance, rfifoIdx, entryIdx);

    return data;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetTransferCount
* Description   : The function returns the number of commands which have been
* completely transfered from the selected cfifo, from CFTCR TC_CF.
*
* Implements    : EQADC_DRV_GetTransferCount_Activity
* END**************************************************************************/
uint16_t EQADC_DRV_GetTransferCount(const uint32_t instance,
                                    const uint32_t cfifoIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    uint16_t numTC;

    numTC = EQADC_GetCfifoTC_CF(instance, cfifoIdx);

    return numTC;
}



/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_SetTransferCount
* Description   : The function sets the value for transfer counter register (CFTCR TC_CF)
*
* Implements    : EQADC_DRV_SetTransferCount_Activity
* END**************************************************************************/
void EQADC_DRV_SetTransferCount(const uint32_t instance,
                                const uint32_t cfifoIdx,
                                const uint16_t value)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    DEV_ASSERT(value < ((uint16_t)1u << (uint16_t)EQADC_CFTCR0_TC_CF0_WIDTH));

    EQADC_SetCfifoTC_CF(instance, cfifoIdx, value);
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_GetCfifoStatusCurrent
* Description   : Returns the current status of the selected command fifo (cfifo).
*
* Implements    : EQADC_DRV_GetCfifoStatusCurrent_Activity
* END**************************************************************************/
eqadc_cfifo_status_t EQADC_DRV_GetCfifoStatusCurrent(const uint32_t instance,
                                                     const uint32_t cfifoIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t cfs;
    eqadc_cfifo_status_t cfifoStatus = CFIFO_STATUS_IDLE;

    cfs = EQADC_GetCfifoCFS(base, cfifoIdx);

    switch(cfs)
    {
        case 0u:
            cfifoStatus = CFIFO_STATUS_IDLE;
            break;
        case 2u:
            cfifoStatus = CFIFO_STATUS_WAITING_FOR_TRIG;
            break;
        case 3u:
            cfifoStatus = CFIFO_STATUS_TRIGGERED;
            break;
        default:
            /* no operation - for normal HW operation this branch should not be reached */
            break;
    }

    return cfifoStatus;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_DRV_GetCfifoStatusSnapshot
* Description   : The function returns the cfifo status of previously completed
* command transfer from a cfifo to a CBuffer (on-chip ADC), from CFSSR TCB register.
* The register values are captured at the beginning of a command transfer to the
* corresponding CBuffer (each on-chip ADC has a single internal CBuffer).
* The status is captured at the time a current command transfer to CBuffer
* is initiated.
*
* Implements    : EQADC_DRV_GetCfifoStatusSnapshot_Activity
* END**************************************************************************/
eqadc_cfifo_status_t EQADC_DRV_GetCfifoStatusSnapshot(const uint32_t instance,
                                                      const uint32_t cfifoIdx,
                                                      const uint32_t adcIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint8_t tcb;
    eqadc_cfifo_status_t cfifoStatus = CFIFO_STATUS_IDLE;

    tcb = EQADC_GetCfifoTCB(base, cfifoIdx, adcIdx);

    switch(tcb)
    {
        case 0u:
            cfifoStatus = CFIFO_STATUS_IDLE;
            break;
        case 2u:
            cfifoStatus = CFIFO_STATUS_WAITING_FOR_TRIG;
            break;
        case 3u:
            cfifoStatus = CFIFO_STATUS_TRIGGERED;
            break;
        default:
            /* no operation - for normal HW operation this branch should not be reached */
            break;
    }
    return cfifoStatus;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_GetCfifoLastCmdSource
 * Description   : Returns the index of the cfifo from which the last command
 * was transfered to the selected CBuffer (on-chip ADC), from CFSSR LCFTCB.
 *
 * Implements    : EQADC_DRV_GetCfifoLastCmdSource_Activity
 * END**************************************************************************/
uint8_t EQADC_DRV_GetCfifoLastCmdSource(const uint32_t instance,
                                        const uint32_t adcIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t cfssr = 0u;
    uint8_t lcftcb;

    switch(adcIdx)
    {
        case 0u:
            cfssr = base->CFSSR0;
            break;
        case 1u:
            cfssr = base->CFSSR1;
            break;
        default:
            DEV_ASSERT(false); /* invalid value for adcIdx*/
            break;
    }

    lcftcb = (uint8_t) ((cfssr & EQADC_CFSSR0_LCFTCB0_MASK) >> EQADC_CFSSR0_LCFTCB0_SHIFT);

    return lcftcb;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_GetCfifoTransferCountLastCmd
 * Description   : Returns the value of Transfer Counter for the last cfifo transfer
 * to the selected CBuffer (on-chip ADC), from CFSSR TC_LCFTCB.
 *
 * Implements    : EQADC_DRV_GetCfifoTransferCountLastCmd_Activity
 * END**************************************************************************/
uint16_t EQADC_DRV_GetCfifoTransferCountLastCmd(const uint32_t instance,
                                                const uint32_t adcIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);

    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t cfssr = 0u;
    uint16_t tc_lcftcb;

    switch(adcIdx)
    {
        case 0u:
            cfssr = base->CFSSR0;
            break;
        case 1u:
            cfssr = base->CFSSR1;
            break;
        default:
            DEV_ASSERT(false); /* invalid value for adcIdx*/
            break;
    }

    tc_lcftcb = (uint16_t) ((cfssr & EQADC_CFSSR0_TC_LCFTCB0_MASK) >> EQADC_CFSSR0_TC_LCFTCB0_SHIFT);

    return tc_lcftcb;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_SetCfifoOpMode
 * Description   : Sets the operating mode for the selected CFIFO
 * If the selected CFIFO mode is different than DISABLED, and current CFIFO mode is not DISABLED
 * or CFIFO status is different than IDLE, the function returns STATUS_ERROR.
 * Otherwise the function updates the cfifo operating mode and returns STATUS_SUCCESS.
 *
 * Implements    : EQADC_DRV_SetCfifoOpMode_Activity
 * END**************************************************************************/
status_t EQADC_DRV_SetCfifoOpMode(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const eqadc_cfifo_mode_t cfifoOpMode)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    eqadc_cfifo_status_t cfifoState;
    uint32_t currentCfifoOpMode;
    status_t status = STATUS_SUCCESS;

    currentCfifoOpMode = EQADC_GetCfifoOpMode(instance, cfifoIdx);
    cfifoState = EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx);

    /* If MODEx is not disabled, it must not be changed to any other mode besides disabled. If MODEx
    is disabled and the CFIFO status is IDLE, MODEx can be changed to any other mode. */
    if(cfifoOpMode != EQADC_CFIFO_MODE_DISABLED)
    {
        if((currentCfifoOpMode != 0u) || (cfifoState != CFIFO_STATUS_IDLE))
        {
            status = STATUS_ERROR;
        }
    }

    if(status == STATUS_SUCCESS)
    {
        EQADC_SetCfifoOpMode(instance, cfifoIdx, cfifoOpMode);
    }

    return status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_GetCfifoOpMode
 * Description   : Get the operating mode for the selected CFIFO
 *
 * Implements    : EQADC_DRV_GetCfifoOpMode_Activity
 * END**************************************************************************/
eqadc_cfifo_mode_t EQADC_DRV_GetCfifoOpMode(const uint32_t instance,
                                            const uint32_t cfifoIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    uint32_t cfifoOpModeUnmapped;
    eqadc_cfifo_mode_t cfifoOpMode = EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_CONT;

    cfifoOpModeUnmapped = EQADC_GetCfifoOpMode(instance, cfifoIdx);

    switch(cfifoOpModeUnmapped)
    {
    case 0u:
        cfifoOpMode = EQADC_CFIFO_MODE_DISABLED;
        break;
    case 1u:
        cfifoOpMode = EQADC_CFIFO_MODE_SW_TRIG_SINGLE;
        break;
    case 2u:
        cfifoOpMode = EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_SINGLE;
        break;
    case 3u:
        cfifoOpMode = EQADC_CFIFO_MODE_HIGH_LVL_GATED_EXT_TRIG_SINGLE;
        break;
    case 4u:
        cfifoOpMode = EQADC_CFIFO_MODE_FALLING_EDGE_EXT_TRIG_SINGLE;
        break;
    case 5u:
        cfifoOpMode = EQADC_CFIFO_MODE_RISING_EDGE_EXT_TRIG_SINGLE;
        break;
    case 6u:
        cfifoOpMode = EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_SINGLE;
        break;
    case 9u:
        cfifoOpMode = EQADC_CFIFO_MODE_SW_TRIG_CONT;
        break;
    case 10u:
        cfifoOpMode = EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_CONT;
        break;
    case 11u:
        cfifoOpMode = EQADC_CFIFO_MODE_HIGH_LVL_GATED_EXT_TRIG_CONT;
        break;
    case 12u:
        cfifoOpMode = EQADC_CFIFO_MODE_FALLING_EDGE_EXT_TRIG_CONT;
        break;
    case 13u:
        cfifoOpMode = EQADC_CFIFO_MODE_RISING_EDGE_EXT_TRIG_CONT;
        break;
    case 14u:
        cfifoOpMode = EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_CONT;
        break;
    default:
        /* invalid case */
        break;
    }

    return cfifoOpMode;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_InvalidateCfifo
 * Description   : Invalidates all entries in the selected CFIFO, by setting CFINVx
 * If current CFIFO mode is not DISABLED or CFIFO status is different than IDLE,
 * the function returns STATUS_ERROR.
 * Otherwise the function invalidates the cfifo and returns STATUS_SUCCESS.
 * For all effects of setting CFINVx please refer to Reference Manual.
 *
 * Implements    : EQADC_DRV_InvalidateCfifo_Activity
 * END**************************************************************************/
status_t EQADC_DRV_InvalidateCfifo(const uint32_t instance,
                                   const uint32_t cfifoIdx)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(cfifoIdx < EQADC_CFPR_COUNT);
    eqadc_cfifo_status_t cfifoState;
    uint32_t currentCfifoOpMode;
    status_t status = STATUS_SUCCESS;

    currentCfifoOpMode = EQADC_GetCfifoOpMode(instance, cfifoIdx);
    cfifoState = EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx);

    if((currentCfifoOpMode != 0u) || (cfifoState != CFIFO_STATUS_IDLE))
    {
        status = STATUS_ERROR;
    }
    else
    {
        EQADC_SetCfifoInv(instance, cfifoIdx);
    }

    return status;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : EQADC_DRV_FlushRfifo
 * Description   : Flushes results from the selected RFIFO
 * Returns STATUS_ERROR if selected RFIFO is configured for draining results with DMA
 * or Drain Event (either DMA or Interrupt) is enabled for the selected RFIFO.
 * Returns STATUS_TIMEOUT if the selected RFIFO is not empty in the provided timeout interval
 *
 * Implements    : EQADC_DRV_FlushRfifo_Activity
 * END**************************************************************************/
status_t EQADC_DRV_FlushRfifo(const uint32_t instance,
                              const uint32_t rfifoIdx,
                              const uint32_t timeout)
{
    DEV_ASSERT(instance < EQADC_INSTANCE_COUNT);
    DEV_ASSERT(rfifoIdx < EQADC_RFPR_COUNT);
    status_t status = STATUS_SUCCESS;
    uint32_t startTime, deltaTime;
    uint8_t rfifoNumEntries;
    uint16_t result;

    OSIF_TimeDelay(0u); /* Make sure OSIF timer is initialized. */

    startTime = OSIF_GetMilliseconds();
    deltaTime = 0u;

    const uint16_t rfifoDrainSelect = EQADC_GetFifoIDCR(instance, rfifoIdx) & EQADC_IDCR0_RFDS1_MASK;
    const uint16_t rfifoDrainEnable = EQADC_GetFifoIDCR(instance, rfifoIdx) & EQADC_IDCR0_RFDE1_MASK;

    if((rfifoDrainSelect != 0u) || (rfifoDrainEnable != 0u))
    {
        status = STATUS_ERROR; /* Returns STATUS_ERROR if selected RFIFO is configured for draining results with DMA
                                  or Drain Event (either DMA or Interrupt) is enabled for the selected RFIFO. */
    }
    else
    {
        rfifoNumEntries = EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR);

        while ((rfifoNumEntries != 0u) && (deltaTime < timeout))
        {
            result = EQADC_DRV_PopRfifoData(instance, rfifoIdx);
            (void) result; /* discard result */

            rfifoNumEntries = EQADC_DRV_GetFifoStatus(instance, rfifoIdx, EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR);

            deltaTime = OSIF_GetMilliseconds() - startTime;
        }
        if(deltaTime >= timeout)
        {
            status = STATUS_TIMEOUT;
        }
        else
        {
            /* Reset RFIFO overflow and RFIFO drain flags */
            EQADC_DRV_ClearFifoStatus(instance, rfifoIdx, (EQADC_FIFO_STATUS_FLAG_RFIFO_OVERFLOW | EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN));
        }
    }

    return status;
}


/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void EQADC_ConfigAdc(const uint32_t instance,
                            const eqadc_adc_config_t * const adcConfig,
                            const uint8_t cfifoIdxForCmds)
{

#if defined(DEV_ERROR_DETECT) || defined(CUSTOM_DEVASSERT)
    static const clock_names_t eqadc_clocks[EQADC_INSTANCE_COUNT] = EQADC_CLOCKS;
    uint32_t eqadc_freq = 0u;
    uint32_t on_chip_adc_freq = 0u;
    uint8_t prescaler = 0u;
    status_t clk_status = CLOCK_SYS_GetFreq(eqadc_clocks[instance], &eqadc_freq);
    DEV_ASSERT(clk_status == STATUS_SUCCESS);
    (void) clk_status;
    if (adcConfig->clkPrescale == EQADC_PRESCALE_NOT_USED)
    {
        prescaler = 1u;
    }
    else
    {
        /* The prescaler value is calculated based on the mapping between enum values and prescaler value */
        prescaler = (adcConfig->clkOddPrescaleEn == false) ? (((uint8_t)(adcConfig->clkPrescale) + 1u) * 2u) : ((((uint8_t)(adcConfig->clkPrescale) + 1u) * 2u) + 1u);
    }
    on_chip_adc_freq = eqadc_freq / prescaler;
    DEV_ASSERT((on_chip_adc_freq >= EQADC_ON_CHIP_ADC_CLOCK_FREQ_MIN_RUNTIME) && (on_chip_adc_freq <= EQADC_ON_CHIP_ADC_CLOCK_FREQ_MAX_RUNTIME));
#endif

    uint16_t adcCR, adcGCCR, adcOCCR;
    eqadc_config_cmd_t configCmd;
    uint8_t adcIdx = adcConfig->adcIdx;

    EQADC_Type * const base = s_eqadcBase[instance];

    switch(adcIdx)
    {
        case 0u:
            base->MCR |= (adcConfig->immediateConvCmdEn == true) ? (EQADC_MCR_ICEA0_MASK) : (uint32_t)0u;
            break;
        case 1u:
            base->MCR |= (adcConfig->immediateConvCmdEn == true) ? (EQADC_MCR_ICEA1_MASK) : (uint32_t)0u;
            break;
        default:
            DEV_ASSERT(false);
            break;
    }

    adcCR = 0u;
    adcCR |= EQADC_ADC_CR_EN_MASK; /* Enable the on-chip ADC module */
    adcCR |= (adcConfig->extMuxEn == true) ? EQADC_ADC_CR_EMUX_MASK : 0u;

    switch(adcConfig->timebaseSel)
    {
        case EQADC_TIMEBASE_SEL_INTERNAL:
            /* CR TBSEL is 0 */
            break;
        case EQADC_TIMEBASE_SEL_IMPORTED_SRV1:
            adcCR |= EQADC_ADC_CR_TBSEL(1u);
            break;
        case EQADC_TIMEBASE_SEL_IMPORTED_SRV2:
            adcCR |= EQADC_ADC_CR_TBSEL(2u);
            break;
        default:
            /* invalid value for adcConfig->timebaseSel */
            DEV_ASSERT(false);
            break;
    }

    if(adcConfig->clkPrescale == EQADC_PRESCALE_NOT_USED)
    {
        /* The ADC0/1_CLK_SEL bits must only be written when the ADC0/1_EN bit is negated.
         * ADC0/1_CLK_SEL can be set during the same write cycle used to set ADC0/1_EN. */
        adcCR |= EQADC_ADC_CR_CLK_SEL_MASK; /* select system clock (prescaler not used) */
        /* CR CLK_PS value is indifferent */
    }
    else
    {
        /* CR CLK_SEL is 0u to select prescaler output */
        adcCR |= EQADC_ADC_CR_CLK_PS(adcConfig->clkPrescale);
    }

    if(adcConfig->clkOddPrescaleEn)
    {
        adcCR |= EQADC_ADC_CR_ODD_PS_MASK;
        adcCR |= (adcConfig->clkDutySel == true) ? EQADC_ADC_CR_CLK_DTY_MASK : (uint16_t)0u;
    }
    else
    {
        DEV_ASSERT(adcConfig->clkDutySel == false); /* Clock duty rate select is only for ODD divide factors */
    }

    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.type        = EQADC_CONFIG_CMD_TYPE_WRITE;
    configCmd.cbufferNum  = adcConfig->adcIdx; /* command is sent to the selected on-chip ADC module */

    /* Send config cmd for writing CR internal register */
    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_CR;
    configCmd.adcRegValue   = adcCR;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

    adcGCCR = EQADC_ADC_GCCR_GCC(adcConfig->adcGain);
    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_GCCR;
    configCmd.adcRegValue   = adcGCCR;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

    adcOCCR = EQADC_ADC_OCCR_OCC(adcConfig->adcOffset);
    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_OCCR;
    configCmd.adcRegValue   = adcOCCR;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}


static void EQADC_ConfigCfifo(const uint32_t instance,
                              const eqadc_cfifo_config_t * const cfifoConfig)
{
    EQADC_Type * const eqadcBase = s_eqadcBase[instance];
    uint8_t cfifoIdx = cfifoConfig->cfifoIdx;

    /* set CFIFO mode to disabled */
    EQADC_SetCfifoOpMode(instance, cfifoIdx, (uint32_t)EQADC_CFIFO_MODE_DISABLED);

    /* busy waiting for cfifo status to be idle */
    while(EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx) != CFIFO_STATUS_IDLE)
    {
    }

    /* cfifo operation mode can only be changed if cfifo status is IDLE and current operation mode is DISABLED */
    EQADC_SetCfifoOpMode(instance, cfifoIdx, (uint32_t)(cfifoConfig->opMode));

    /* Enable detection of HW trigger events, when trigger mode is not is single-scan level- or edge-trigger mode */
    switch(cfifoConfig->opMode)
    {
        case EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_SINGLE:
        case EQADC_CFIFO_MODE_HIGH_LVL_GATED_EXT_TRIG_SINGLE:
        case EQADC_CFIFO_MODE_FALLING_EDGE_EXT_TRIG_SINGLE:
        case EQADC_CFIFO_MODE_RISING_EDGE_EXT_TRIG_SINGLE:
        case EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_SINGLE:
            EQADC_SetCfifoSSE(instance, cfifoIdx);
            break;
        default:
            /* nothing required for rest of opModes */
            break;
    }

    if(cfifoConfig->fillReqSel == EQADC_REQ_TYPE_SEL_DMA)
    {
        EQADC_ConfigCmdDma(instance, cfifoConfig);
        EQADC_SetFifoIDCR(instance, cfifoIdx, EQADC_IDCR0_CFFS1_MASK);
        EQADC_DRV_EnableDmaReq(instance, cfifoIdx, EQADC_DMA_REQ_EN_CFIFO_FILL);
    }
    else
    {
        EQADC_ClearFifoIDCR(instance, cfifoIdx, EQADC_IDCR0_CFFS1_MASK);
        EQADC_DRV_DisableIntReq(instance, cfifoIdx, EQADC_INT_EN_CFIFO_FILL);
    }

    /* trigSel has effect only if opMode is not SW triggered */
    DEV_ASSERT((cfifoConfig->trigSel == 0u) || ((cfifoConfig->opMode != EQADC_CFIFO_MODE_SW_TRIG_SINGLE) && \
            (cfifoConfig->opMode != EQADC_CFIFO_MODE_SW_TRIG_CONT)));
    EQADC_SetCfifoTrig(instance, cfifoIdx, cfifoConfig->trigSel);

    if(cfifoConfig->tbgClksel == EQADC_TBG_CLKSEL_DISABLED)
    {
        SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_GEN_MASK;
        SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_CLKSEL_MASK;
        SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_TRIGPER_MASK;
    }
    else
    {
        /* Trigger Burst Generator is connected between mux controlled by ISEL4-7 (config via trigSel).
         * When enabled, a trigger must be selected via trigSel. */
        DEV_ASSERT(cfifoConfig->trigSel != 0u);

        SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_CLKSEL_MASK;
        SIU_TBG_CR[instance][cfifoIdx] |= SIU_TBG_CR_A_GEN(cfifoConfig->tbgClksel);

        DEV_ASSERT(cfifoConfig->tbgTriggerPeriod <= SIU_TBG_CR_A_TRIGPER_MASK);
        SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_TRIGPER_MASK;
        SIU_TBG_CR[instance][cfifoIdx] |= SIU_TBG_CR_A_TRIGPER(cfifoConfig->tbgTriggerPeriod);

        SIU_TBG_CR[instance][cfifoIdx] |= SIU_TBG_CR_A_GEN_MASK;
    }

    /* Configure special registers available only for CFIFO0 */
    if(cfifoIdx == 0u)
    {
        if(cfifoConfig->entryNumExtensionEn == true)
        {
            eqadcBase->CFCR0 |= EQADC_CFCR0_CFEEE0_MASK;
        }
        else
        {
            eqadcBase->CFCR0 &= ~EQADC_CFCR0_CFEEE0_MASK;
        }

        /* advance trigger operation mode (amode) can only be changed if cfifo status is IDLE and current amode is DISABLED */

        /* disable advance trigger operation mode; cfifo is assumed IDLE */
        eqadcBase->CFCR0 &= ~EQADC_CFCR0_AMODE0_MASK;

        EQADC_SetAdvanceTrig(instance, cfifoConfig->advanceTrigSel);

        if(cfifoConfig->advanceTrigMode != EQADC_ATRIG_MODE_DISABLED)
        {
            /* Reference Manual: For the streaming mode of operation when the ATRIG0 is used to enable the ETRIG0 or to
             * advance the command queue, the normal mode of operation is external trigger single scan. Other
             * settings are not fully tested. */
            DEV_ASSERT((cfifoConfig->opMode >= EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_SINGLE) && \
                    (cfifoConfig->opMode <= EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_SINGLE));

            /* The Advance Trigger only has effect if advance trigger mode is not SW trigger or a HW trigger is selected */
            DEV_ASSERT((cfifoConfig->advanceTrigMode != EQADC_ATRIG_MODE_SW_TRIG_SINGLE) || \
                    (cfifoConfig->advanceTrigSel == 0u));

            eqadcBase->CFCR0 |= EQADC_CFCR0_AMODE0(cfifoConfig->advanceTrigMode);

            /* enable streaming mode */
            eqadcBase->CFCR0 |= EQADC_CFCR0_STRME0_MASK;
        }
        else
        {
            /* streaming mode is disabled */
            eqadcBase->CFCR0 &= ~EQADC_CFCR0_STRME0_MASK;

            /* advance trigger doesn't have any effect */
        }
    }
}


static void EQADC_ConfigCmdDma(const uint32_t instance,
                               const eqadc_cfifo_config_t * const cfifoConfig)
{
    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t srcAddr, destAddr, srcSizeInBytes;
    uint8_t cfifoIdx = cfifoConfig->cfifoIdx;
    srcAddr = (uint32_t) cfifoConfig->sourcePtr;
    destAddr = (uint32_t)(&(base->CFPR[cfifoIdx]));
    srcSizeInBytes = sizeof(cfifoConfig->sourcePtr[0u]) * cfifoConfig->sourceLength;

    /* Update EQADC state with the DMA virtual channel number for current rfifo */
    dmaCmdVirtualChans[cfifoIdx] = cfifoConfig->dmaVirtualChan;

    /* Configure EDMA to transfer 1 command for each CFIFO FILL signal received.
     * After a number of transfers equal with the source buffer length, EDMA shall call the registered callback (if not NULL)
     * EDMA disables the channel after the src buffer has been copied and doesn't start the channel after Init().
     * The user shall call EDMA_DRV_StartChannel to enable the EDMA channel */
    edma_transfer_config_t tCfg;
    edma_loop_transfer_config_t tLoopCfg;
    tCfg.destAddr           = destAddr;
    tCfg.srcAddr            = srcAddr;
    tCfg.srcTransferSize    = EDMA_TRANSFER_SIZE_4B;
    tCfg.destTransferSize   = EDMA_TRANSFER_SIZE_4B;
    tCfg.srcOffset          = (int16_t)sizeof(cfifoConfig->sourcePtr[0u]); /* After each transfer, the src address is incremented with the size of buffer element */
    tCfg.destOffset         = 0;
    tCfg.srcLastAddrAdjust  = -(int32_t)srcSizeInBytes; /* Circular src buffer: after the src buffer is copied, src address is set at the beginning of the buffer */
    tCfg.destLastAddrAdjust = 0;
    tCfg.srcModulo          = EDMA_MODULO_OFF;
    tCfg.destModulo         = EDMA_MODULO_OFF;
    tCfg.minorByteTransferCount    = sizeof(cfifoConfig->sourcePtr[0u]);
    tCfg.scatterGatherEnable       = false;
    tCfg.scatterGatherNextDescAddr = 0u;
    tCfg.interruptEnable           = true;
    tCfg.loopTransferConfig        = &tLoopCfg;

    tLoopCfg.majorLoopIterationCount = cfifoConfig->sourceLength; /* Number of minor loops in a major loop */
    tLoopCfg.srcOffsetEnable         = false;
    tLoopCfg.dstOffsetEnable         = false;
    tLoopCfg.minorLoopOffset         = 0;
    tLoopCfg.minorLoopChnLinkEnable  = false;
    tLoopCfg.minorLoopChnLinkNumber  = 0u;
    tLoopCfg.majorLoopChnLinkEnable  = false;
    tLoopCfg.majorLoopChnLinkNumber  = 0u;

    status_t status;
    status = EDMA_DRV_ConfigLoopTransfer(cfifoConfig->dmaVirtualChan, &tCfg);
    (void) status;

    EDMA_DRV_DisableRequestsOnTransferComplete(cfifoConfig->dmaVirtualChan, true);
    /* Do not start the DMA channel -  it needs to be started manually outside the EQADC driver */

    if(cfifoConfig->callback != NULL)
    {
        status = EDMA_DRV_InstallCallback(cfifoConfig->dmaVirtualChan, cfifoConfig->callback, cfifoConfig->callbackParam);
        (void) status;

        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_ERR_INT, true);
        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_HALF_MAJOR_LOOP_INT, false);
        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_MAJOR_LOOP_INT, true);
    }
    else
    {
        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_ERR_INT, false);
        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_HALF_MAJOR_LOOP_INT, false);
        EDMA_DRV_ConfigureInterrupt(cfifoConfig->dmaVirtualChan, EDMA_CHN_MAJOR_LOOP_INT, false);
    }
}

static void EQADC_ConfigResultDma(const uint32_t instance,
                                  const eqadc_result_dma_config_t * const resultDmaConfig)
{
    const EQADC_Type * const base = s_eqadcBase[instance];
    uint32_t srcAddr, destAddr, destSizeInBytes;
    uint8_t rfifoIdx = resultDmaConfig->rfifoIdx;

    srcAddr = (uint32_t)(&(base->RFPR[rfifoIdx])) + 2u; /* +2u because read op needs to be done on RF_POPX, from the second 16bits in 32bits RFPR reg */
    destAddr = (uint32_t) resultDmaConfig->destPtr;
    destSizeInBytes = sizeof(resultDmaConfig->destPtr[0u]) * resultDmaConfig->destLength;

    /* Update EQADC state with the DMA virtual channel number for current rfifo */
    dmaResultVirtualChans[rfifoIdx] = resultDmaConfig->dmaVirtualChan;

    /* Configure EDMA to transfer 1 result for each EQADC result.
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


static void EQADC_ConfigAltConfiguration(const uint32_t instance,
                                         const eqadc_alternate_config_t * const altConfig,
                                         const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;
    uint16_t regVal = 0u;
    uint8_t i;

    /* Prepare uint32_t variable with value corresponding to ACR bit-fields */
    regVal |= (altConfig->resultInhibit == true) ? EQADC_ADC_ACR_RET_INH_MASK : (uint16_t)0u;
    regVal |= EQADC_ADC_ACR_DEST(altConfig->dest);
    regVal |= (altConfig->signedResEn == true) ? EQADC_ADC_ACR_FMTA_MASK : (uint16_t)0u;
    regVal |= (altConfig->returnPSI == true) ? EQADC_ADC_ACR_RPSI_MASK : (uint16_t)0u;
    regVal |= EQADC_ADC_ACR_RESSEL(altConfig->adcResolution);
    regVal |= EQADC_ADC_ACR_ATBSEL(altConfig->timebaseSel);
    regVal |= EQADC_ADC_ACR_PRE_GAIN(altConfig->adcPregain);

     /* Write configuration values in corresponding ACR register */
    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = EQADC_GetAltConfigRegAddr(altConfig->altConfigSel);
    configCmd.adcRegValue   = regVal;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

    /* AGR and AOR registers are only available for ALT_CONFIG 1 and ALT_CONFIG 2 */
    if((altConfig->altConfigSel == EQADC_ALT_CONFIG_SEL_CONFIG_1) || \
        (altConfig->altConfigSel == EQADC_ALT_CONFIG_SEL_CONFIG_2))
    {
        uint8_t altCfgIdx = EQADC_GetAltConfigIdx(altConfig->altConfigSel);
        for(i = 0; i < EQADC_NUM_ADC; i++)
        {
            /* Select the destination on-chip ADC instance */
            configCmd.cbufferNum = i;

            /* Write configuration values in corresponding AGR register to corresponding on-chip ADC instance */
            configCmd.adcRegAddress = agrAddr[altCfgIdx];
            configCmd.adcRegValue   = EQADC_ADC_AGR_ALTGCC(altConfig->adcGain[i]);
            EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
            EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

            /* Write configuration values in corresponding AOR register to corresponding on-chip ADC instance */
            configCmd.adcRegAddress = aorAddr[altCfgIdx];
            configCmd.adcRegValue   = EQADC_ADC_AOR_ALTOCC(altConfig->adcOffset[i]);
            EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
            EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
        }
    }
}


static void EQADC_ConfigExtAltConfiguration(const uint32_t instance,
                                            const eqadc_ext_alternate_config_t * const extAltConfig,
                                            const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;
    uint16_t regVal = 0u;

    /* Prepare uint32_t variable with value corresponding to ACR bit-fields */
    regVal |= (extAltConfig->resultInhibit2 == true) ? EQADC_ADC_EACR_RET_INH2_MASK : (uint16_t)0u;
    regVal |= EQADC_ADC_EACR_DEST2(extAltConfig->dest2);
    regVal |= (extAltConfig->signedResEn2 == true) ? EQADC_ADC_EACR_FMTA2_MASK : (uint16_t)0u;
    regVal |= (extAltConfig->returnPSI2 == true) ? EQADC_ADC_EACR_RPSI2_MASK : (uint16_t)0u;
    regVal |= (extAltConfig->flushEn2 == true) ? EQADC_ADC_EACR_FLEN2_MASK : (uint16_t)0u;
    regVal |= (extAltConfig->msgTagEn2 == true) ? EQADC_ADC_EACR_TEN2_MASK : (uint16_t)0u;
    regVal |= EQADC_ADC_EACR_MESSAGE_TAG2(extAltConfig->msgTag2);

    /* Write configuration values in corresponding ACR register */
    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = EQADC_GetExtAltConfigRegAddr(extAltConfig->extAltConfigSel);
    configCmd.adcRegValue   = regVal;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}


static void EQADC_ConfigChanPull(const uint32_t instance,
                                 const eqadc_chan_pull_config_t * const chanPullConfig,
                                 const uint8_t cfifoIdxForCmds)
{
    const uint8_t adcInputChanIdx = chanPullConfig->adcInputChanIdx;
    eqadc_config_cmd_t configCmd;
    uint16_t regVal = 0;
    const uint8_t adcRegAddress = (uint8_t)EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR0 + adcInputChanIdx;

    regVal |= EQADC_ADC_PUDCR_CH_PULL(chanPullConfig->chanPull);
    regVal |= EQADC_ADC_PUDCR_PULL_STR(chanPullConfig->chanPullStrength);

    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = (eqadc_onchip_adc_reg_address_t)adcRegAddress;
    configCmd.adcRegValue   = regVal;

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}


static void EQADC_ConfigStacBusClient(const uint32_t instance,
                                      const eqadc_stac_bus_client_config_t * const stacBusClientConfig)
{
    EQADC_Type * const eqadcBase = s_eqadcBase[instance];
    const uint32_t clientIdx = stacBusClientConfig->clientIdx;

    if(clientIdx == 0u)
    {
        eqadcBase->STACCCR &= ~(EQADC_STACCCR_SRV1_MASK | EQADC_STACCCR_STACBS1_MASK);
        eqadcBase->STACCCR |= EQADC_STACCCR_SRV1(stacBusClientConfig->serverDataSlotSel);
        eqadcBase->STACCCR |= EQADC_STACCCR_STACBS1(stacBusClientConfig->timebaseBitsSel);
    }
    else
    {
        eqadcBase->STACCCR &= ~(EQADC_STACCCR_SRV2_MASK | EQADC_STACCCR_STACBS2_MASK);
        eqadcBase->STACCCR |= EQADC_STACCCR_SRV2(stacBusClientConfig->serverDataSlotSel);
        eqadcBase->STACCCR |= EQADC_STACCCR_STACBS2(stacBusClientConfig->timebaseBitsSel);
    }
}


static void EQADC_GetDefaultCfifoConfig(eqadc_cfifo_config_t * const cfifoConfig)
{
    cfifoConfig->cfifoIdx = 0u;
    cfifoConfig->opMode   = EQADC_CFIFO_MODE_SW_TRIG_SINGLE;

    /* DMA default disabled */
    cfifoConfig->fillReqSel = EQADC_REQ_TYPE_SEL_INT;
    /* The rest of the DMA parameters are indifferent */
    cfifoConfig->dmaVirtualChan = 0u;
    cfifoConfig->sourcePtr      = NULL;
    cfifoConfig->sourceLength     = 0u;
    cfifoConfig->callback       = NULL;
    cfifoConfig->callbackParam  = NULL;

    cfifoConfig->trigSel = 0u; /* No trigger because SW Trigger mode is selected */

    /* Trigger Burst Generator default disabled */
    cfifoConfig->tbgClksel        = EQADC_TBG_CLKSEL_DISABLED;
    cfifoConfig->tbgTriggerPeriod = 0u;

    /* Members available only for CFIFO0 are configured as disabled
     * to allow the configuration to be used also for other cfifos. */
    /* Do not extend number of trigger entries */
    cfifoConfig->entryNumExtensionEn = false;

    cfifoConfig->advanceTrigMode = EQADC_ATRIG_MODE_DISABLED;
    cfifoConfig->advanceTrigSel  = 0u;
}


static void EQADC_GetDefaultConfigCmd(eqadc_config_cmd_t * const configCmd)
{
    configCmd->eoq           = true;
    configCmd->pause         = false;
    configCmd->repeatStart   = false;
    configCmd->cbufferNum    = 0u;
    configCmd->type          = EQADC_CONFIG_CMD_TYPE_WRITE;
    configCmd->adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_GCCR;
    configCmd->adcRegValue   = 0u;
    configCmd->msgTag        = EQADC_MESSAGE_TAG_RFIFO0; /* has no effect because cmd type is write */
}


static void EQADC_ResetAdc(const uint32_t instance,
                           const uint8_t adcIdx,
                           const uint8_t cfifoIdxForCmds)
{
   eqadc_config_cmd_t configCmd;

   EQADC_Type * const base = s_eqadcBase[instance];

   switch(adcIdx)
   {
       case 0u:
           base->MCR &= EQADC_MCR_ICEA0_MASK;
           break;
       case 1u:
           base->MCR &= EQADC_MCR_ICEA1_MASK;
           break;
       default:
           /* no op */
           break;
   }

   EQADC_GetDefaultConfigCmd(&configCmd);
   configCmd.type        = EQADC_CONFIG_CMD_TYPE_WRITE;
   configCmd.cbufferNum  = adcIdx; /* command is sent to the selected on-chip ADC module */

   /* Reset ADC CR */
   uint16_t adcCR = 0u;
   adcCR |= EQADC_ADC_CR_CLK_PS_MASK; /* only CLK_PS has reset value 1. The rest are 0 */
   /* Send config cmd for writing CR internal register */
   configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_CR;
   configCmd.adcRegValue   = adcCR;
   EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
   EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

   configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_GCCR;
   configCmd.adcRegValue   = 0x4000u; /* Reset value for GCCR */
   EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
   EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

   configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_OCCR;
   configCmd.adcRegValue   = 0u; /* Reset value for GCCR */
   EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
   EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}


static void EQADC_ResetCfifo(const uint32_t instance,
                             const uint8_t cfifoIdx)
{
    EQADC_Type * const eqadcBase = s_eqadcBase[instance];
    uint16_t mask = 0;

    /* CFINVx must not be written unless the MODEx is configured to disabled, and CFIFO status is IDLE. */

    /* set CFIFO mode to disabled */
    EQADC_SetCfifoOpMode(instance, cfifoIdx, (uint32_t)EQADC_CFIFO_MODE_DISABLED);

    /* busy waiting for cfifo status to be idle */
    while(EQADC_DRV_GetCfifoStatusCurrent(instance, cfifoIdx) != CFIFO_STATUS_IDLE)
    {
    }

    /* Reset all other registers corresponding to cfifo_config members */

    /* reset interrupts and dma enable flags, except the ones for rfifos */
    mask = (uint16_t)(EQADC_INT_EN_ALL & ~(EQADC_INT_EN_RFIFO_OVERFLOW | EQADC_INT_EN_RFIFO_DRAIN));
    EQADC_ClearFifoIDCR(instance, cfifoIdx, mask);
    /* reset register corresponding to fillReqSel */
    EQADC_ClearFifoIDCR(instance, cfifoIdx, EQADC_IDCR0_CFFS1_MASK);

    /* Stop DMA virtual channel and reset the EQADC state storing DMA virtual channel used for cfifo  */
    if(dmaCmdVirtualChans[cfifoIdx] != EQADC_INVALID_DMA_VIRT_CHAN)
    {
        status_t status;
        status = EDMA_DRV_StopChannel(dmaCmdVirtualChans[cfifoIdx]);
        (void) status;
        dmaCmdVirtualChans[cfifoIdx] = EQADC_INVALID_DMA_VIRT_CHAN;
    }

    /* invalidate CFIFO entries after interrupts have been disabled, otherwise it might generate interrupt/dma requests for filling the CFIFO */
    EQADC_SetCfifoInv(instance, cfifoIdx);

    /* reset register corresponding to trigSel */
    EQADC_SetCfifoTrig(instance, cfifoIdx, 0u);

    SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_GEN_MASK;
    SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_CLKSEL_MASK;
    SIU_TBG_CR[instance][cfifoIdx] &= ~SIU_TBG_CR_A_TRIGPER_MASK;

    /* reset special registers available only for CFIFO0 */
    if(cfifoIdx == 0u)
    {
        eqadcBase->CFCR0 &= ~EQADC_CFCR0_CFEEE0_MASK;

        /* disable advance trigger operation mode */
        eqadcBase->CFCR0 &= ~EQADC_CFCR0_AMODE0_MASK;

        eqadcBase->CFCR0 &= ~EQADC_CFCR0_STRME0_MASK;
        EQADC_SetAdvanceTrig(instance, 0u);
    }
}


static void EQADC_ResetAltConfiguration(const uint32_t instance,
                                        const eqadc_alt_config_sel_t altConfigSel,
                                        const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;
    uint8_t i;

     /* Write configuration values in corresponding ACR register */
    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = EQADC_GetAltConfigRegAddr(altConfigSel);
    configCmd.adcRegValue   = 0u; /* Reset value for ACR */
    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

    /* AGR and AOR registers are only available for ALT_CONFIG 1 and ALT_CONFIG 2 */
    if((altConfigSel == EQADC_ALT_CONFIG_SEL_CONFIG_1) || \
        (altConfigSel == EQADC_ALT_CONFIG_SEL_CONFIG_2))
    {
        uint8_t altCfgIdx = EQADC_GetAltConfigIdx(altConfigSel);
        for(i = 0; i < EQADC_NUM_ADC; i++)
        {
            /* Select the destination on-chip ADC instance */
            configCmd.cbufferNum = i;

            /* Write configuration values in corresponding AGR register to corresponding on-chip ADC instance */
            configCmd.adcRegAddress = agrAddr[altCfgIdx];
            configCmd.adcRegValue   = 0x4000u; /* Reset value for AGR */
            EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
            EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

            /* Write configuration values in corresponding AOR register to corresponding on-chip ADC instance */
            configCmd.adcRegAddress = aorAddr[altCfgIdx];
            configCmd.adcRegValue   = 0u; /* Reset value for AOR */
            EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
            EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
        }
    }
}

static void EQADC_ResetExtAltConfiguration(const uint32_t instance,
                                           const eqadc_alt_config_sel_t extAltConfigSel,
                                           const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;

    /* Write configuration values in corresponding ACR register */
    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = EQADC_GetExtAltConfigRegAddr(extAltConfigSel);
    configCmd.adcRegValue   = 0u; /* reset value for EACR */
    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}


static void EQADC_ResetChanPull(const uint32_t instance,
                                const uint8_t adcInputChanIdx,
                                const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;
    const uint8_t adcRegAddress = (uint8_t)EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR0 + adcInputChanIdx;

    EQADC_GetDefaultConfigCmd(&configCmd);
    configCmd.adcRegAddress = (eqadc_onchip_adc_reg_address_t) adcRegAddress;
    configCmd.adcRegValue   = 0u; /* Reset value for PUDCR */

    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}

static void EQADC_ResetTimeStampConfiguration(const uint32_t instance,
                                              const uint8_t cfifoIdxForCmds)
{
    eqadc_config_cmd_t configCmd;

    EQADC_GetDefaultConfigCmd(&configCmd);

    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_TSCR;
    configCmd.adcRegValue   = 0u; /* Reset value for TSCR */
    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */

    configCmd.adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_TBCR;
    configCmd.adcRegValue   = 0u; /* Reset value for TBCR */
    EQADC_DRV_PushCfifoConfigCmd(instance, cfifoIdxForCmds, &configCmd);
    EQADC_SetCfifoSSE(instance, cfifoIdxForCmds);  /* SW trigger to send the command */
}

static void EQADC_ResetStacBusClient(const uint32_t instance,
                                     const uint32_t clientIdx)
{
    EQADC_Type * const eqadcBase = s_eqadcBase[instance];

    if(clientIdx == 0u)
    {
        eqadcBase->STACCCR &= ~(EQADC_STACCCR_SRV1_MASK | EQADC_STACCCR_STACBS1_MASK);
    }
    else
    {
        eqadcBase->STACCCR &= ~(EQADC_STACCCR_SRV2_MASK | EQADC_STACCCR_STACBS2_MASK);
    }
}


static uint32_t EQADC_GetCfifoConvCmd(const eqadc_conv_cmd_t * const convCmd)
{
    uint32_t cmd = 0u;

    cmd |= (convCmd->eoq == true) ? EQADC_CMD_EOQ_MASK : 0u;
    cmd |= (convCmd->pause == true) ? EQADC_CMD_PAUSE_MASK : 0u;
    cmd |= (convCmd->repeatStart == true) ? EQADC_CMD_REP_MASK : 0u;
    /* EB bit must be 0 */
    cmd &= ~EQADC_CMD_EB_MASK;

    DEV_ASSERT(convCmd->cbufferNum < EQADC_NUM_ADC);
    /* For cbufferNum == 0u, BN bit must be 0 */
    cmd |= (convCmd->cbufferNum == 1u) ? EQADC_CMD_BN_MASK : 0u;

    cmd |= (convCmd->calibEn == true) ? EQADC_CMD_CAL_MASK : 0u;
    cmd |= (((uint32_t) convCmd->msgTag) << EQADC_CMD_MESSAGE_TAG_SHIFT) & EQADC_CMD_MESSAGE_TAG_MASK;
    cmd |= (((uint32_t) convCmd->samplingTime) << EQADC_CMD_LST_SHIFT) & EQADC_CMD_LST_MASK;
    cmd |= (convCmd->timeStampReq == true) ? EQADC_CMD_TSR_MASK : 0u;
    cmd |= (((uint32_t) convCmd->channelNum) << EQADC_CMD_CHAN_NUM_SHIFT) & EQADC_CMD_CHAN_NUM_MASK;

    if(convCmd->altConfigSel != EQADC_ALT_CONFIG_SEL_DISABLED)
    {
        /* Alternate configuration format */
        cmd |= (((uint32_t) convCmd->altConfigSel) << EQADC_CMD_ALT_CFG_SEL_SHIFT) & EQADC_CMD_ALT_CFG_SEL_MASK;
        cmd |= (convCmd->flushCompanion == true) ? EQADC_CMD_FFMT_MASK : 0u;
    }
    else
    {
        /* Standard format */
        cmd |= (convCmd->signEn == true) ? EQADC_CMD_FMT_MASK : 0u;
    }

    return cmd;
}


static uint32_t EQADC_GetCfifoConfigCmd(const eqadc_config_cmd_t * const configCmd)
{
    uint32_t cmd = 0u;

    cmd |= (configCmd->eoq == true) ? EQADC_CMD_EOQ_MASK : 0u;
    cmd |= (configCmd->pause == true) ? EQADC_CMD_PAUSE_MASK : 0u;
    cmd |= (configCmd->repeatStart == true) ? EQADC_CMD_REP_MASK : 0u;
    /* EB bit must be 0 */

    DEV_ASSERT(configCmd->cbufferNum < EQADC_NUM_ADC);
    /* For cbufferNum == 0u, BN bit must be 0 */
    cmd |= (configCmd->cbufferNum == 1u) ? EQADC_CMD_BN_MASK : 0u;

    cmd |= (((uint32_t) configCmd->adcRegAddress) & EQADC_CMD_REG_ADDRESS_MASK);

    switch(configCmd->type)
    {
        case EQADC_CONFIG_CMD_TYPE_READ:
            cmd |= EQADC_CMD_RW_MASK;
            cmd |= (((uint32_t) configCmd->msgTag) << EQADC_CMD_MESSAGE_TAG_SHIFT) & EQADC_CMD_MESSAGE_TAG_MASK;
            break;
        case EQADC_CONFIG_CMD_TYPE_WRITE:
            cmd &= ~EQADC_CMD_RW_MASK;
            cmd |= ((((uint32_t) configCmd->adcRegValue) << EQADC_CMD_REG_LOW_BYTE_SHIFT) \
                    & (EQADC_CMD_REG_HIGH_BYTE_MASK | EQADC_CMD_REG_LOW_BYTE_MASK));
            break;
        default:
            DEV_ASSERT(false);
            break;
    }

    return cmd;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetAltConfigIdx
 * Description   : Returns the index in range [0,EQADC_NUM_ALT_CONFIGS), corresponding
 * to a value from eqadc_alt_config_sel_t.
 * IMPORTANT NOTE: the function assumes that values of eqadc_alt_config_sel_t enum are consecutive
 *
 *END*************************************************************************/
static inline uint8_t EQADC_GetAltConfigIdx(eqadc_alt_config_sel_t altConfigSel)
{
    uint8_t index = 0u;

    if(altConfigSel != EQADC_ALT_CONFIG_SEL_DISABLED)
    {
        index = (uint8_t)(altConfigSel) - (uint8_t)(EQADC_ALT_CONFIG_SEL_CONFIG_1);
        DEV_ASSERT(index < EQADC_NUM_ALT_CONFIGS);
    }

    return index;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetAltConfigSel
 * Description   : Returns the eqadc_alt_config_sel_t corresponding to the input index in range [0,EQADC_NUM_ALT_CONFIGS)
 * IMPORTANT NOTE: the function assumes that values of eqadc_alt_config_sel_t enum are consecutive
 *
 *END*************************************************************************/
static inline eqadc_alt_config_sel_t EQADC_GetAltConfigSel(uint8_t index)
{
    const uint8_t altConfigSel = (uint8_t)EQADC_ALT_CONFIG_SEL_CONFIG_1 + index;
    return (eqadc_alt_config_sel_t)altConfigSel;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetAltConfigRegAddr
 * Description   : Returns the ACR register address, corresponding to a value from eqadc_alt_config_sel_t.
 *
 *END*************************************************************************/
static inline eqadc_onchip_adc_reg_address_t EQADC_GetAltConfigRegAddr(eqadc_alt_config_sel_t altConfigSel)
{
    static const eqadc_onchip_adc_reg_address_t acrIdxMap[EQADC_NUM_ALT_CONFIGS] = {
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR1,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR2,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR3,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR4,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR5,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR6,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR7,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR8,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR9,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR10,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR11,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR12,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR13,
            EQADC_ONCHIP_ADC_REG_ADDRESS_ACR14
    };

    return acrIdxMap[EQADC_GetAltConfigIdx(altConfigSel)];
}

/*FUNCTION*********************************************************************
 *
 * Function Name : EQADC_GetExtAltConfigRegAddr
 * Description   : Returns the ACR register address, corresponding to a value from eqadc_alt_config_sel_t.
 *
 *END*************************************************************************/
static inline eqadc_onchip_adc_reg_address_t EQADC_GetExtAltConfigRegAddr(eqadc_alt_config_sel_t altConfigSel)
{
    static const eqadc_onchip_adc_reg_address_t eacrIdxMap[EQADC_NUM_EXT_ALT_CONFIGS] = {
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR1,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR2,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR3,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR4,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR5,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR6,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR7,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR8,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR9,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR10,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR11,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR12,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR13,
            EQADC_ONCHIP_ADC_REG_ADDRESS_EACR14
    };

    return eacrIdxMap[EQADC_GetAltConfigIdx(altConfigSel)];
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_WriteGainToConfigCmd
* Description   : Write gain in the configuration command received as pointer according to calibTarget
* END**************************************************************************/
static void EQADC_WriteGainToConfigCmd(eqadc_config_cmd_t * const configCmd,
                                       const eqadc_calibration_target_t calibTarget,
                                       const uint16_t gain)
{
    uint16_t adcRegValue = 0u;
    eqadc_onchip_adc_reg_address_t adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AGR1;

    switch(calibTarget)
    {
    case EQADC_CALIBRATION_TARGET_MAIN:
        adcRegValue   = EQADC_ADC_GCCR_GCC(gain);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_GCCR;
        break;
    case EQADC_CALIBRATION_TARGET_ALT_CONFIG_1:
        adcRegValue   = EQADC_ADC_AGR_ALTGCC(gain);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AGR1;
        break;
    case EQADC_CALIBRATION_TARGET_ALT_CONFIG_2:
        adcRegValue   = EQADC_ADC_AGR_ALTGCC(gain);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AGR2;
        break;
    default:
        /* invalid case */
        DEV_ASSERT(false);
        break;
    }

    configCmd->adcRegAddress = adcRegAddress;
    configCmd->adcRegValue   = adcRegValue;
}


/*FUNCTION**********************************************************************
*
* Function Name : EQADC_WriteOffsetToConfigCmd
* Description   : Write offset in the configuration command received as pointer according to calibTarget
* END**************************************************************************/
static void EQADC_WriteOffsetToConfigCmd(eqadc_config_cmd_t * const configCmd,
                                         const eqadc_calibration_target_t calibTarget,
                                         const int16_t offset)
{
    uint16_t adcRegValue = 0u;
    eqadc_onchip_adc_reg_address_t adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AOR1;

    switch(calibTarget)
    {
    case EQADC_CALIBRATION_TARGET_MAIN:
        adcRegValue   = EQADC_ADC_OCCR_OCC(offset);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_OCCR;
        break;
    case EQADC_CALIBRATION_TARGET_ALT_CONFIG_1:
        adcRegValue   = EQADC_ADC_AOR_ALTOCC(offset);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AOR1;
        break;
    case EQADC_CALIBRATION_TARGET_ALT_CONFIG_2:
        adcRegValue   = EQADC_ADC_AOR_ALTOCC(offset);
        adcRegAddress = EQADC_ONCHIP_ADC_REG_ADDRESS_AOR2;
        break;
    default:
        /* invalid case */
        DEV_ASSERT(false);
        break;
    }
    configCmd->adcRegAddress = adcRegAddress;
    configCmd->adcRegValue   = adcRegValue;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
