/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
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
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope.
 * The variables are ROM-able and are shared between different functions in the module.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Could be declared as pointing to const.
 * Function prototype dependent on external function pointer typedef.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize base pointers with unsigned integer values.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type.
 * The cast is required due to the way parameters are passed to the callback function.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const/volatile from a pointer or reference.
 * The cast is required to initialize the user provided configuration structure.
 */

#include <stddef.h>
#include "device_registers.h"
#include "srx_driver.h"
#include "status.h"
#include "clock_manager.h"
#include "srx_hw_access.h"
#include "edma_driver.h"
#include "interrupt_manager.h"
#include "srx_irq.h"

/*! @cond DRIVER_INTERNAL_USE_ONLY */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/**
 * Defines 1s/x = SRX_ONE_MICROSECOND_CYCLES
 */
#define SRX_ONE_MICROSECOND_CYCLES (1000000u)

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * Private data
 ******************************************************************************/

/**
 * Default channel configuration
 */
static const srx_channel_config_t s_srxDefaultChannelConfig[] =
{
    {
        .channelId = 0u,
        .tickDuration = 3u,
        .inputFilter = SRX_INPUT_FILTER_NONE,
        .diagConfig =
        {
            .diagEvents = (
                            SRX_EV_CAL_RESYNC |
                            SRX_EV_CAL_20_25 |
                            SRX_EV_SMSG_OFLW |
                            SRX_EV_FMSG_OFLW |
                            SRX_EV_PP_DIAG_ERR |
                            SRX_EV_CAL_LEN_ERR |
                            SRX_EV_CAL_DIAG_ERR |
                            SRX_EV_NIB_VAL_ERR |
                            SRX_EV_SMSG_CRC_ERR |
                            SRX_EV_FMSG_CRC_ERR |
                            SRX_EV_NUM_EDGES_ERR |
                            SRX_EV_NONE
                          ),
            .idleCount = SRX_BUS_IDLE_DISABLED,
            .calibVar = SRX_CALIB_VAR_25_PERCENT,
            .diagPulse = SRX_PULSE_CHECK_BOTH,
            .pausePulse = SRX_PAUSE_PULSE_DISABLED,
            .succesiveCal = SRX_SUCC_CAL_CHK_PREFFERED
        },
        .fastMsgConfig =
        {
            .numberOfNibbles = 6,
            .dmaEnable = false,
            .crcIncludeStatus = true,
            .disableCrcCheck = false,
            .crcType = SRX_CRC_LEGACY
        },
        .slowMsgConfig =
        {
            .dmaEnable = false,
            .crcType = SRX_CRC_RECOMMENDED
        }
    }
};


/**
 * Default user config
 */
static const srx_driver_user_config_t s_srxDefaultUserConfig =
{
    .fastMsgDmaPtr = NULL,
    .slowMsgDmaPtr = NULL,
    .fastDmaChannel = 0U,
    .slowDmaChannel = 0U,
    .callbackFunc =
    {
        .function = NULL,
        .param = NULL
    },
    .fastDmaFIFOEnable = false,
    .fastDmaFIFOSize = 1,
    .channelConfig = (const srx_channel_config_t *)s_srxDefaultChannelConfig,
    .numOfConfigs = 1
};

/**
 * Array with clock instances to SRX peripherals
 */
static const clock_names_t s_srxClockNames[SRX_INSTANCE_COUNT] = FEATURE_SRX_CLOCK_NAMES;

/**
 * DMA multiplexer for the Fast channel.
 */
static const dma_request_source_t s_srxFastDMASrc[SRX_INSTANCE_COUNT] = FEATURE_SRX_FAST_DMA_REQS;

/**
 * DMA multiplexer for the Slow channel.
 */
static const dma_request_source_t s_srxSlowDMASrc[SRX_INSTANCE_COUNT] = FEATURE_SRX_SLOW_DMA_REQS;

/**
 * Pointers to the configuration structures
 */
static srx_state_t * s_srxStatePtr[SRX_INSTANCE_COUNT];

/**
 * Base image of the configuration structure used for DMA transfers.
 */
static const edma_transfer_config_t srxDmaBaseConfig =
{
    .srcAddr = 0u, /* Modified by function */
    .destAddr = 0u, /* Modified by function */
    .srcTransferSize = EDMA_TRANSFER_SIZE_4B,
    .destTransferSize = EDMA_TRANSFER_SIZE_4B,
    .srcOffset = 0, /* Modified by function */
    .destOffset = (int16_t)sizeof(uint32_t),
    .srcLastAddrAdjust = 0, /* Modified by function */
    .destLastAddrAdjust = 0, /* Modified by function */
    .srcModulo = EDMA_MODULO_OFF,
    .destModulo = EDMA_MODULO_OFF,
    .minorByteTransferCount = 0, /* Modified by function */
    .scatterGatherEnable = false,
    .scatterGatherNextDescAddr = 0u,
    .interruptEnable = true,
    .loopTransferConfig = &(edma_loop_transfer_config_t)
    {
        .majorLoopIterationCount = 1u,
        .srcOffsetEnable = true,
        .dstOffsetEnable = true,
        .minorLoopOffset = 0,
        .minorLoopChnLinkEnable = false,
        .minorLoopChnLinkNumber = 0u,
        .majorLoopChnLinkEnable = false,
        .majorLoopChnLinkNumber = 0u
    }
};

/*******************************************************************************
 * Private functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_InitDiag
 * Description   : Initializes the diagnostics side of the driver.
 *
 *END**************************************************************************/
static void SRX_DRV_InitDiag(const uint32_t instance, const uint8_t channel, const srx_diag_config_t * config)
{
    /* Set bus IDLE count */
    SRX_DRV_HW_SetBusIdleCnt(instance, channel, config->idleCount);

    /* Set calibration range */
    SRX_DRV_HW_SetCalRng(instance, channel, config->calibVar);

    /* Set pause pulse diag check */
    SRX_DRV_HW_SetPpChkSel(instance, channel, config->diagPulse);

    /* Set pause pulse enable status */
    SRX_DRV_HW_SetPausePulseEnable(instance, channel, config->pausePulse);

    /* Successive calibration check */
    SRX_DRV_HW_SetSuccCalChk(instance, channel, config->succesiveCal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_InitSlowMes
 * Description   : Initializes the Slow message reception side of the driver.
 *
 *END**************************************************************************/
static void SRX_DRV_InitSlowMsg(const uint32_t instance, const uint8_t channel, const srx_slow_msg_config_t * config)
{
    /* CRC related */
    SRX_DRV_HW_SetSlowCrcType(instance, channel, config->crcType);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_InitFastMes
 * Description   : Initializes the Fast message reception side of the driver.
 *
 *END**************************************************************************/
static void SRX_DRV_InitFastMsg(const uint32_t instance, const uint8_t channel, const srx_fast_msg_config_t * config)
{
    /* Number of nibbles */
    SRX_DRV_HW_SetFastNumNibbles(instance, channel, config->numberOfNibbles);

    /* CRC related */
    SRX_DRV_HW_SetFastCrcType(instance, channel, config->crcType);
    SRX_DRV_HW_SetFastCrcIncStatus(instance, channel, config->crcIncludeStatus);
    SRX_DRV_HW_SetFastDisableCrc(instance, channel, config->disableCrcCheck);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_ComputeTimestampPrescaler
 * Description   : Computes the message timestamp base prescaler.
 *
 *END**************************************************************************/
static uint8_t SRX_DRV_ComputeTimestampPrescaler(const uint32_t clock)
{
    uint32_t tsPre;

    /* 0 in worst case */
    tsPre = (uint32_t)(clock / SRX_ONE_MICROSECOND_CYCLES) - 1u;
    /* Saturate at upper interval */
    tsPre = (tsPre > 255u) ? 255u : tsPre;

    return (uint8_t)tsPre;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_ComputeChannelPrescaler
 * Description   : Computes the channel reception clock prescaler.
 *
 *END**************************************************************************/
static uint32_t SRX_DRV_ComputeChannelPrescaler(const uint8_t reqTick, const uint32_t clock)
{
    uint32_t chPre;

    /* Compute and set the channel prescaler */
    chPre = clock / 1000u; /* Keep info on fractional frequencies */

    chPre = (reqTick * chPre) / 1000u;

    return chPre;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_FastRxCompleteDma
 * Description   : Reception complete DMA notification
 * for the Fast channel.
 *
 *END**************************************************************************/
static void SRX_DRV_FastRxCompleteDma(void * parameter, edma_chn_status_t status)
{
    uint8_t instance = *(const uint8_t *)parameter;

    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);

    if (status == EDMA_CHN_NORMAL)
    {
        DEV_ASSERT(s_srxStatePtr[instance] != NULL);

        /* Call the notification */
        if(s_srxStatePtr[instance]->callbackFunc.function != NULL)
        {
            s_srxStatePtr[instance]->callbackFunc.function(instance, 0u, /* Channel always passed as 0 in DMA mode */
                    SRX_CALLBACK_FAST_DMA_RX_COMPLETE, s_srxStatePtr[instance]->callbackFunc.param);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_SlowRxCompleteDma
 * Description   : Reception complete DMA notification
 * for the Slow channel.
 *
 *END**************************************************************************/
static void SRX_DRV_SlowRxCompleteDma(void * parameter, edma_chn_status_t status)
{
    uint8_t instance = *(const uint8_t *)parameter;

    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);

    if (status == EDMA_CHN_NORMAL)
    {
        DEV_ASSERT(s_srxStatePtr[instance] != NULL);

        /* Call the notification */
        if(s_srxStatePtr[instance]->callbackFunc.function != NULL)
        {
            s_srxStatePtr[instance]->callbackFunc.function(instance, 0u, /* Channel always passed as 0 in DMA mode */
                    SRX_CALLBACK_SLOW_DMA_RX_COMPLETE, s_srxStatePtr[instance]->callbackFunc.param);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_ConfigureDmaTransfer
 * Description   : Configures a loop DMA transfer for the given channel.
 *
 *END**************************************************************************/
static void SRX_DRV_ConfigureDmaTransfer(const uint8_t channel, const uint32_t srcAddr,
                                         const uint32_t destAddr, const bool hasFifo,
                                         const uint8_t lenFifo)
{
    /* Generate the structure. Passing it on the stack is OK
     * since the values are copied to peripheral registers */
    edma_transfer_config_t transferConfig = srxDmaBaseConfig;

    /* Modify source and destination */
    transferConfig.srcAddr = srcAddr;
    transferConfig.destAddr = destAddr;

#ifdef FEATURE_SRX_DMA_HAS_FIFO
    /* Depending on case, configure the parameters */
    if(hasFifo)
    {
        /* Valid values are between 1 and FEATURE_SRX_MAX_FIFO_SIZE */
        uint8_t locFifoLen = (lenFifo < 1u) ? 1u : ((lenFifo > FEATURE_SRX_MAX_FIFO_SIZE) ? FEATURE_SRX_MAX_FIFO_SIZE : lenFifo);

        transferConfig.minorByteTransferCount = locFifoLen * sizeof(srx_raw_msg_t);
        transferConfig.destLastAddrAdjust = -((int32_t)locFifoLen * (int32_t)sizeof(srx_raw_msg_t));
        transferConfig.srcOffset = 0;
        transferConfig.srcLastAddrAdjust = 0;
    }
    else
    {
        transferConfig.minorByteTransferCount = sizeof(srx_raw_msg_t);
        transferConfig.destLastAddrAdjust = -((int32_t)sizeof(srx_raw_msg_t));
        transferConfig.srcOffset = (int16_t)(sizeof(uint32_t));
        transferConfig.srcLastAddrAdjust = -((int32_t)sizeof(srx_raw_msg_t));
    }
#else /* Parts with no FIFO feature */
    (void)hasFifo;
    (void)lenFifo;

    transferConfig.minorByteTransferCount = sizeof(srx_raw_msg_t);
    transferConfig.destLastAddrAdjust = -((int32_t)sizeof(srx_raw_msg_t));
    transferConfig.srcOffset = (int16_t)(sizeof(uint32_t));
    transferConfig.srcLastAddrAdjust = -((int32_t)sizeof(srx_raw_msg_t));
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

    /* Configure a loop transfer */
    (void)EDMA_DRV_ConfigLoopTransfer(channel, &transferConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_InstallCallbacks
 * Description   : Installs callbacks for the driver events.
 *
 *END**************************************************************************/
static void SRX_DRV_InstallCallbacks(const uint32_t instance)
{
    uint32_t chInd;
    bool fastDmaSet = false;
    bool slowDmaSet = false;
    srx_state_t * lState;

    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    lState = s_srxStatePtr[instance];

    /* Check for valid function first */
    if(lState->callbackFunc.function != NULL)
    {
        for(chInd = 0u; chInd < SRX_CHANNEL_COUNT; chInd++)
        {
            /* Only on active channel */
            if(lState->activeChannels[chInd])
            {
                /* Rx events only if no bits are set in mask */
                if((lState->channelEvents[chInd] & SRX_EV_ALL) != 0u)
                {
                    SRX_DRV_HW_SetEventConfig(lState->instanceId, (uint8_t)chInd, lState->channelEvents[chInd]);
                    SRX_DRV_IRQ_EnableIRQ(lState->instanceId, chInd, SRX_IRQ_ERROR);
                }
                else
                {
                    SRX_DRV_HW_SetEventConfig(lState->instanceId, (uint8_t)chInd, SRX_EV_NONE);
                    SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_ERROR);
                }

                /* Fast events */
                if(lState->fastDmaEnabled[chInd])
                {
                    /* Mark to be set later */
                    fastDmaSet = true;

                    /* This means that there will be no more fast Channel Rx interrupts since DMA is enabled. */
                    SRX_DRV_HW_SetFastRxInterrupt(lState->instanceId, (uint8_t)chInd, false);
                    SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_FAST);
                }
                else
                {
                    /* Enable channel interrupt, since DMA is not enabled */
                    SRX_DRV_HW_SetFastRxInterrupt(lState->instanceId, (uint8_t)chInd, true);
                    SRX_DRV_IRQ_EnableIRQ(lState->instanceId, chInd, SRX_IRQ_FAST);
                }

                /* Slow events */
                if(lState->slowDmaEnabled[chInd])
                {
                    /* Mark to be set later */
                    slowDmaSet = true;

                    /* This means that there will be no more fast Channel Rx interrupts since DMA is enabled. */
                    SRX_DRV_HW_SetSlowRxInterruptStatus(lState->instanceId, (uint8_t)chInd, false);
                    SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_SLOW);
                }
                else
                {
                    /* Enable channel interrupt, since DMA is not enabled */
                    SRX_DRV_HW_SetSlowRxInterruptStatus(lState->instanceId, (uint8_t)chInd, true);
                    SRX_DRV_IRQ_EnableIRQ(lState->instanceId, chInd, SRX_IRQ_SLOW);
                }
            }
        }

        /* Fast DMA callbacks (global for all channels) */
        if(fastDmaSet)
        {
            (void)EDMA_DRV_InstallCallback(lState->fastDmaChannel,
                                           &SRX_DRV_FastRxCompleteDma,
                                           &lState->instanceId);
        }

        /* Slow DMA callbacks (global for all channels) */
        if(slowDmaSet)
        {
            (void)EDMA_DRV_InstallCallback(lState->slowDmaChannel,
                                           &SRX_DRV_SlowRxCompleteDma,
                                           &lState->instanceId);
        }
    }
    else /* if(lState->callbackFunc.function != NULL) Disable all callbacks in case the function was removed */
    {
        for(chInd = 0u; chInd < SRX_CHANNEL_COUNT; chInd++)
        {
            /* Only on active channel */
            if(lState->activeChannels[chInd])
            {
                /* Fast events */
                if(lState->fastDmaEnabled[chInd])
                {
                    /* Mark to be set later */
                    fastDmaSet = true;
                }

                /* Slow events */
                if(lState->slowDmaEnabled[chInd])
                {
                    /* Mark to be set later */
                    slowDmaSet = true;
                }

                /* Disable Diagnostics interrupts */
                SRX_DRV_HW_SetEventConfig(lState->instanceId, (uint8_t)chInd, SRX_EV_NONE);
                SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_ERROR);

                /* Disable Fast interrupts */
                SRX_DRV_HW_SetFastRxInterrupt(lState->instanceId, (uint8_t)chInd, false);
                SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_FAST);

                /* Disable slow interrupts */
                SRX_DRV_HW_SetSlowRxInterruptStatus(lState->instanceId, (uint8_t)chInd, false);
                SRX_DRV_IRQ_DisableIRQ(lState->instanceId, chInd, SRX_IRQ_SLOW);
            }
        }

        /* Only if active */
        if(fastDmaSet)
        {
            (void)EDMA_DRV_InstallCallback(lState->fastDmaChannel,
                                           ((void *)0u),
                                           &lState->instanceId);
        }

        /* Only if active */
        if(slowDmaSet)
        {
            (void)EDMA_DRV_InstallCallback(lState->slowDmaChannel,
                                           ((void *)0u),
                                           &lState->instanceId);
        }
    }
}

/*******************************************************************************
 * Public functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_FastHandler
 * Description   : Gets called from the low level handler
 * with instance and channel as parameter.
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_FastHandler(const uint32_t instance, const uint32_t channel)
{
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);

    if(s_srxStatePtr[instance] != NULL)
    {
        /* Check if channel is valid first */
        if(s_srxStatePtr[instance]->activeChannels[channel])
        {

#if defined(ERRATA_E7425) /* Enable only when we have this active */
            if (SRX_DRV_HW_ErrataE7425Workaroud(instance, channel))
            {
#endif /* defined(ERRATA_E7425) */

            if(s_srxStatePtr[instance]->callbackFunc.function != NULL)
            {
                s_srxStatePtr[instance]->callbackFunc.function(instance, channel,
                        SRX_CALLBACK_FAST_RX_COMPLETE, s_srxStatePtr[instance]->callbackFunc.param);
            }

#if defined(ERRATA_E7425)
            }
#endif /* defined(ERRATA_E7425) */
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_SlowHandler
 * Description   : Gets called from the low level handler
 * with instance and channel as parameter.
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_SlowHandler(const uint32_t instance, const uint32_t channel)
{
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);

    if(s_srxStatePtr[instance] != NULL)
    {
        /* Check if channel is valid first */
        if(s_srxStatePtr[instance]->activeChannels[channel])
        {
            if(s_srxStatePtr[instance]->callbackFunc.function != NULL)
            {
                s_srxStatePtr[instance]->callbackFunc.function(instance, channel,
                        SRX_CALLBACK_SLOW_RX_COMPLETE, s_srxStatePtr[instance]->callbackFunc.param);
            }
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_IRQ_RxErrHandler
 * Description   : Gets called from the low level handler
 * with instance and channel as parameter.
 *
 *END**************************************************************************/
void SRX_DRV_IRQ_RxErrHandler(const uint32_t instance, const uint32_t channel)
{
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);

    if(s_srxStatePtr[instance] != NULL)
    {
        /* Check if channel is valid first */
        if(s_srxStatePtr[instance]->activeChannels[channel])
        {

#if defined(ERRATA_E7425) /* Enable only when we have this active */
            if (SRX_DRV_HW_ErrataE7425Workaroud(instance, channel))
            {
#endif /* defined(ERRATA_E7425) */

            if(s_srxStatePtr[instance]->callbackFunc.function != NULL)
            {
                s_srxStatePtr[instance]->callbackFunc.function(instance, channel,
                        SRX_CALLBACK_RX_ERROR, s_srxStatePtr[instance]->callbackFunc.param);
            }

#if defined(ERRATA_E7425)
            }
#endif /* defined(ERRATA_E7425) */
        }
    }
}

/*! @endcond */

/*******************************************************************************
 * API implementation
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_Init
 * Description   : Initializes the driver for a given peripheral
 * according to the given configuration structure.
 *
 * Implements    : SRX_DRV_Init_Activity
 *END**************************************************************************/
status_t SRX_DRV_Init(const uint32_t instance, const srx_driver_user_config_t * configPtr, srx_state_t * state)
{
    uint32_t inputClock;
    uint8_t chInd;
    bool fDmaEn = false;
    bool sDmaEn = false;
    status_t initErr;
    const srx_channel_config_t * chPtr;
    status_t retVal;

    /* Check for correct calling parameters */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(configPtr != NULL);
    DEV_ASSERT(configPtr->numOfConfigs <= SRX_CHANNEL_COUNT);

    /* Only if not already initialized */
    if(s_srxStatePtr[instance] != NULL)
    {
        retVal = STATUS_ERROR;
    }
    else
    {
        /* Copy the references */
        s_srxStatePtr[instance] = state;

        /* Populate the configuration structure */
        state->fastMsgDmaPtr = configPtr->fastMsgDmaPtr;
        state->slowMsgDmaPtr = configPtr->slowMsgDmaPtr;
        state->fastDmaChannel = configPtr->fastDmaChannel;
        state->slowDmaChannel = configPtr->slowDmaChannel;
        state->callbackFunc = configPtr->callbackFunc;
        state->instanceId = (uint8_t)instance;

        /* Get the protocol clock frequency */
        initErr = CLOCK_SYS_GetFreq(s_srxClockNames[instance], &inputClock);
        DEV_ASSERT(initErr == STATUS_SUCCESS);
        DEV_ASSERT(inputClock >= SRX_ONE_MICROSECOND_CYCLES);
        (void) initErr;

        /* Set register time base */
        SRX_DRV_HW_SetTimestampPrescaler(instance, SRX_DRV_ComputeTimestampPrescaler(inputClock));

        /* Setup the channels */
        for(chInd = 0u; chInd < configPtr->numOfConfigs; chInd++)
        {
            /* Only if we have a valid channel */
            DEV_ASSERT(configPtr->channelConfig[chInd].channelId < SRX_CHANNEL_COUNT);

            /* Obtain a pointer to channel config */
            chPtr = &configPtr->channelConfig[chInd];

            /* Mark channel as active */
            state->activeChannels[chPtr->channelId] = true;

            /* Copy DMA status */
            state->fastDmaEnabled[chPtr->channelId] = chPtr->fastMsgConfig.dmaEnable;
            state->slowDmaEnabled[chPtr->channelId] = chPtr->slowMsgConfig.dmaEnable;

            /* Store active events */
            state->channelEvents[chPtr->channelId] = chPtr->diagConfig.diagEvents;

            /* pre-scaler and compensation */
            SRX_DRV_HW_SetChannelPrescaler(instance, chPtr->channelId,
                    (uint16_t)SRX_DRV_ComputeChannelPrescaler(chPtr->tickDuration, inputClock));

            SRX_DRV_HW_SetChannelCompensation(instance, chPtr->channelId, true);

            /* DMA enablers */
            if(chPtr->fastMsgConfig.dmaEnable)
            {
                SRX_DRV_HW_SetFastDma(instance, chPtr->channelId, true);
                fDmaEn = true;
            }

            if(chPtr->slowMsgConfig.dmaEnable)
            {
                SRX_DRV_HW_SetSlowDma(instance, chPtr->channelId, true);
                sDmaEn = true;
            }

            /* Setup diagnostics */
            SRX_DRV_InitDiag(instance, chPtr->channelId, &chPtr->diagConfig);

            /* Setup fast messages */
            SRX_DRV_InitFastMsg(instance, chPtr->channelId, &chPtr->fastMsgConfig);

            /* Setup slow messages */
            SRX_DRV_InitSlowMsg(instance, chPtr->channelId, &chPtr->slowMsgConfig);
        }

        /* Enable DMA transfers */
        if(fDmaEn)
        {
            /* Assures correct designation, rather than configuring it from DMA */
            (void)EDMA_DRV_SetChannelRequestAndTrigger(state->fastDmaChannel, (uint8_t)s_srxFastDMASrc[instance], false);

#ifdef FEATURE_SRX_DMA_HAS_FIFO
            /* Configure the FIFO */
            if(configPtr->fastDmaFIFOEnable)
            {

                SRX_DRV_HW_SetFifoState(instance, true);
                SRX_DRV_HW_SetFifoWm(instance, configPtr->fastDmaFIFOSize);
            }
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

            /* Configure DMA */
            SRX_DRV_ConfigureDmaTransfer(state->fastDmaChannel,
                                         (uint32_t)SRX_DRV_HW_GetFastDmaRegStartAddr(instance),
                                         (uint32_t)state->fastMsgDmaPtr,
                                         configPtr->fastDmaFIFOEnable,
                                         configPtr->fastDmaFIFOSize);

            /* Start channel */
            (void)EDMA_DRV_StartChannel(state->fastDmaChannel);
        }

        if(sDmaEn)
        {
            /* Configure DMA */
            (void)EDMA_DRV_SetChannelRequestAndTrigger(state->slowDmaChannel, (uint8_t)s_srxSlowDMASrc[instance], false);

            /* Configure DMA */
            SRX_DRV_ConfigureDmaTransfer(state->slowDmaChannel,
                                         (uint32_t)SRX_DRV_HW_GetSlowDmaRegStartAddr(instance),
                                         (uint32_t)state->slowMsgDmaPtr,
                                         false, 0u);

            /* Start channel */
            (void)EDMA_DRV_StartChannel(state->slowDmaChannel);
        }

        /* Setup notifications */
        SRX_DRV_InstallCallbacks(instance);

        /* Enable channels */
        for(chInd = 0u; chInd < SRX_CHANNEL_COUNT; chInd++)
        {
            if(state->activeChannels[chInd])
            {
                /* Enable channel */
                SRX_DRV_HW_SetChannelStatus(instance, chInd, true);
            }
        }

        /* Enable peripheral */
        SRX_DRV_HW_SetPeripheralStatus(instance, true);

        /* Success */
        retVal = STATUS_SUCCESS;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetEvents
 * Description   : Returns a list containing masks for the current active events.
 * Also clears the active events in the process.
 *
 * Implements    : SRX_DRV_GetEvents_Activity
 *END**************************************************************************/
status_t SRX_DRV_GetEvents(const uint32_t instance, const uint32_t channel, srx_event_t * events)
{
    status_t retVal;
    srx_event_t locEv;

    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Only if channel is enabled */
    if(s_srxStatePtr[instance]->activeChannels[channel] != false)
    {
        /* Get, clear and return */
        locEv = SRX_DRV_HW_GetActiveEvents(instance, (uint8_t)channel);
        SRX_DRV_HW_ClearActiveEvents(instance, (uint8_t)channel, locEv);
        *events = locEv;
        retVal = STATUS_SUCCESS;
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetFastMsg
 * Description   : Returns last received fast message and clears the
 * Rx complete flag.
 *
 * Implements    : SRX_DRV_GetFastMsg_Activity
 *END**************************************************************************/
status_t SRX_DRV_GetFastMsg(const uint32_t instance, const uint32_t channel, srx_fast_msg_t * message)
{
    srx_raw_msg_t locMsg;
    status_t retVal;

    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Only if channel is enabled */
    if(s_srxStatePtr[instance]->activeChannels[channel] != false)
    {
        SRX_DRV_HW_GetFastRawMsg(instance, (uint8_t)channel, &locMsg);
        SRX_DRV_HW_ConvertFastRaw(message, &locMsg);
        retVal = STATUS_SUCCESS;
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetSlowMsg
 * Description   : Returns last received slow message and clears the
 * Rx complete flag.
 *
 * Implements    : SRX_DRV_GetSlowMsg_Activity
 *END**************************************************************************/
status_t SRX_DRV_GetSlowMsg(const uint32_t instance, const uint32_t channel, srx_slow_msg_t * message)
{
    srx_raw_msg_t locMsg;
    status_t retVal;

    /* Invalid instance or channel */
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);


    /* Only if channel is enabled */
    if(s_srxStatePtr[instance]->activeChannels[channel] != false)
    {
        SRX_DRV_HW_GetSlowRawMsg(instance, (uint8_t)channel, &locMsg);
        SRX_DRV_HW_ConvertSlowRaw(message, &locMsg);
        retVal = STATUS_SUCCESS;
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetFastMsgFromRaw
 * Description   : Transforms a RAW fast message into a normal fast message.
 *
 * Implements    : SRX_DRV_GetFastMsgFromRaw_Activity
 *END**************************************************************************/
void SRX_DRV_GetFastMsgFromRaw(srx_fast_msg_t * msg, const srx_raw_msg_t * rawMsg)
{
    /* Just call the conversion function */
    SRX_DRV_HW_ConvertFastRaw(msg, rawMsg);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetSlowMsgFromRaw
 * Description   : Transforms a RAW slow message into a normal slow message.
 *
 * Implements    : SRX_DRV_GetSlowMsgFromRaw_Activity
 *END**************************************************************************/
void SRX_DRV_GetSlowMsgFromRaw(srx_slow_msg_t * msg, const srx_raw_msg_t * rawMsg)
{
    /* Just call the conversion function */
    SRX_DRV_HW_ConvertSlowRaw(msg, rawMsg);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetSlowRxStatus
 * Description   : Returns the buffer status for any incoming SLOW message.
 *
 * Implements    : SRX_DRV_GetSlowRxStatus_Activity
 *END**************************************************************************/
bool SRX_DRV_GetSlowRxStatus(const uint32_t instance, const uint32_t channel)
{
    bool retVal;

    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Only if channel is enabled */
    if(s_srxStatePtr[instance]->activeChannels[channel] != false)
    {
        retVal = SRX_DRV_HW_GetSlowRxStatus(instance, (uint8_t)channel);
    }
    else
    {
        retVal = false;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetFastRxStatus
 * Description   : Returns the buffer status for any incoming FAST message.
 *
 * Implements    : SRX_DRV_GetFastRxStatus_Activity
 *END**************************************************************************/
bool SRX_DRV_GetFastRxStatus(const uint32_t instance, const uint32_t channel)
{
    bool retVal;

    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(channel < SRX_CHANNEL_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Only if channel is enabled */
    if(s_srxStatePtr[instance]->activeChannels[channel] != false)
    {
        retVal = SRX_DRV_HW_GetFastRxStatus(instance, (uint8_t)channel);
    }
    else
    {
        retVal = false;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_SetFastMsgDmaBuffer
 * Description   : Sets (modifies) the buffer in which the DMA driven
 * reception for Fast messages is made. Length of the
 * buffer must be (fastDmaFIFOSize)
 * bytes in case fastDmaFIFOEnable is TRUE.
 *
 * Implements    : SRX_DRV_SetFastMsgDmaBuffer_Activity
 *END**************************************************************************/
status_t SRX_DRV_SetFastMsgDmaBuffer(const uint32_t instance, srx_raw_msg_t * buffer)
{
    /* Invalid instance */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Store and update */
    s_srxStatePtr[instance]->fastMsgDmaPtr = buffer;
    EDMA_DRV_SetDestAddr(s_srxStatePtr[instance]->fastDmaChannel, (uint32_t)buffer);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_SetRxCallbackFunction
 * Description   : Sets (modifies) the callback function assigned to the
 * peripheral instance
 *
 * Implements    : SRX_DRV_SetRxCallbackFunction_Activity
 *END**************************************************************************/
status_t SRX_DRV_SetRxCallbackFunction(const uint32_t instance, srx_callback_func_t function, void * param)
{
    /* Invalid instance */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Copy to state structure */
    s_srxStatePtr[instance]->callbackFunc.function = function;
    s_srxStatePtr[instance]->callbackFunc.param = param;

    /* Notification setup */
    SRX_DRV_InstallCallbacks(instance);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_SetSlowMsgDmaBuffer
 * Description   : Sets (modifies) the buffer in which the DMA driven
 * reception for slow messages is made.
 *
 * Implements    : SRX_DRV_SetSlowMsgDmaBuffer_Activity
 *END**************************************************************************/
status_t SRX_DRV_SetSlowMsgDmaBuffer(const uint32_t instance, srx_raw_msg_t * buffer)
{
    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);
    DEV_ASSERT(s_srxStatePtr[instance] != NULL);

    /* Store and update */
    s_srxStatePtr[instance]->slowMsgDmaPtr = buffer;
    EDMA_DRV_SetDestAddr(s_srxStatePtr[instance]->slowDmaChannel, (uint32_t)buffer);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_DeInit
 * Description   : De-Initializes the peripheral and brings it's registers into a reset state.
 *
 * Implements    : SRX_DRV_DeInit_Activity
 *END**************************************************************************/
status_t SRX_DRV_Deinit(const uint32_t instance)
{
    uint8_t indChan;
    status_t retVal;

    /* Invalid instance or channel */
    DEV_ASSERT(instance < SRX_INSTANCE_COUNT);

    if (s_srxStatePtr[instance] != NULL)
    {
        for(indChan = 0u; indChan < SRX_CHANNEL_COUNT; indChan++)
        {
            /* Mask interrupts */
            SRX_DRV_IRQ_DisableIRQ(instance, indChan, SRX_IRQ_FAST);
            SRX_DRV_IRQ_DisableIRQ(instance, indChan, SRX_IRQ_SLOW);
            SRX_DRV_IRQ_DisableIRQ(instance, indChan, SRX_IRQ_ERROR);
        }

        /* Reset peripheral*/
        SRX_DRV_HW_ResetPeripheral(instance);

        /* Clear internal data */
        s_srxStatePtr[instance] = NULL;

        retVal = STATUS_SUCCESS;
    }
    else
    {
        retVal = STATUS_ERROR;
    }

    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_GetDefaultConfig
 * Description   : Returns a default configuration for the TLE4998 sensor.
 *
 * Implements    : SRX_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void SRX_DRV_GetDefaultConfig(srx_driver_user_config_t * config)
{
    DEV_ASSERT(config->channelConfig != NULL);
    srx_channel_config_t * lconfig = (srx_channel_config_t *)config->channelConfig;

    /* Copy default user config */
    *config = s_srxDefaultUserConfig;

    /* Copy channel config */
    *lconfig = *s_srxDefaultUserConfig.channelConfig;

    /* Restore */
    config->channelConfig = lconfig;
}

/*******************************************************************************
* EOF
*******************************************************************************/
