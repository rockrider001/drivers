/*
 * Copyright 2017-2018 NXP
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
 * @file dspi_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * The code is not dynamically linked. An absolute stack address is obtained when
 * taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, Could be made static.
 * Some functions are part of API and are called by user's application and can't be made static
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different essential type.
 * This assignment is required for selecting the IRQn_Type without using complex switch statement or
 * for avoiding compatibility issues in interrupt handling.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'
 * This is required by the conversion of a bool into a register value.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 *(different essential type categories).
 * This is required by the conversion of unsigned integer into instance number or of interrupt request name.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type and a
 * pointer to a different object type
 * This is needed for receive/transmit buffers, which can be uint8_t, uint16_t or uint32_t depending on
 * frame size.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type
 * This is needed for receive/transmit buffers, which can be uint8_t, uint16_t or uint32_t depending on
 * frame size.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long
 * The cast is required to initialize the DMA transfer the the unsigned long variable represents the source
 * or the destination of the transfer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 *
 */

#include "dspi_driver.h"
#include "dspi_hw_access.h"
#include "edma_driver.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/* Define DSPI modes which are used by the driver*/
#define DSPI_SPI_MODE 0U

/* Define the number of microseconds/seconds */
#define MICROSECONDS 1000000U

/* Define the two modes of the SPI */
#define MASTER_MODE 1U
#define SLAVE_MODE  0U

/* Define all DSPI base address */
static DSPI_Type * DSPI_Instances[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT]             = FEATURE_DSPI_INSTANCES;

#ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
/* Define all interrupt vectors */
static IRQn_Type   dspiSendInterruptVector[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT]    = FEATURES_DSPI_SEND_INTERUPT_VECTOR;
static IRQn_Type   dspiRecieveInterruptVector[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT] = FEATURES_DSPI_RECEIVE_INTERUPT_VECTOR;
static IRQn_Type   dspiFaultInterruptVector[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT]   = FEATURES_DSPI_FAULT_INTERUPT_VECTOR;
#endif

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
/* Define the PCS allocation for each DSPI/SPI module */
static uint8_t PCS_Mapping[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT] = FEATURE_DSPI_PCS_MAPPING;
#endif /* defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

/* Define all internal functions */
static void DSPI_RECEIVE_isr(dspi_instance_t instance);

static void DSPI_SEND_isr(dspi_instance_t instance);

/* Define baud rate prescalers */
static const uint32_t prescalerValues[] = {2U, 3U, 5U, 7U};

/* Define all internal IRQ handlers */
void DSPI0_Send_IRQHandler(void);

void DSPI1_Send_IRQHandler(void);

void DSPI2_Send_IRQHandler(void);

void DSPI3_Send_IRQHandler(void);

void DSPI4_Send_IRQHandler(void);

void DSPI5_Send_IRQHandler(void);

void DSPI6_Send_IRQHandler(void);

void DSPI7_Send_IRQHandler(void);

void DSPI8_Send_IRQHandler(void);

void DSPI9_Send_IRQHandler(void);

void DSPI0_Receive_IRQHandler(void);

void DSPI1_Receive_IRQHandler(void);

void DSPI2_Receive_IRQHandler(void);

void DSPI3_Receive_IRQHandler(void);

void DSPI4_Receive_IRQHandler(void);

void DSPI5_Receive_IRQHandler(void);

void DSPI6_Receive_IRQHandler(void);

void DSPI7_Receive_IRQHandler(void);

void DSPI8_Receive_IRQHandler(void);

void DSPI9_Receive_IRQHandler(void);

void DSPI0_FIFO_Error_IRQHandler(void);

void DSPI1_FIFO_Error_IRQHandler(void);

void DSPI2_FIFO_Error_IRQHandler(void);

void DSPI3_FIFO_Error_IRQHandler(void);

void DSPI4_FIFO_Error_IRQHandler(void);

void DSPI5_FIFO_Error_IRQHandler(void);

void DSPI6_FIFO_Error_IRQHandler(void);

void DSPI7_FIFO_Error_IRQHandler(void);

void DSPI8_FIFO_Error_IRQHandler(void);

void DSPI9_FIFO_Error_IRQHandler(void);

static uint32_t abs_dif(uint32_t a,
                        uint32_t b)
{
    if (a > b)
    {
        return a - b;
    }
    else
    {
        return b - a;
    }
}

static dspi_state_t * DSPI_state[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT] = FEATURE_DSPI_INITIAL_STATE;

#if (!(defined FEATURE_DSPI_HAS_EXTENDED_MODE))
static void DSPI_DRV_MasterCompleteTxDMATransfer(void * parameter,
                                               edma_chn_status_t status);
#endif
static void DSPI_DRV_MasterCompleteDMATransfer(void * parameter,
                                               edma_chn_status_t status);

static void DSPI_GetBestFreq(dspi_instance_t instance,
                             uint32_t requestedFreq,
                             uint8_t * bestPrescaler,
                             uint8_t * bestScaler)
{
    uint32_t bestFreq = 0xFFFFFFFFU;
    uint32_t freq1 = 0U;
    uint32_t freq2 = 0U;
    uint8_t scaler = 0U;
    uint8_t prescaler = 0U;

    uint32_t tempBestFreq   = 0;
    uint8_t  tempScaler     = 0;

    uint32_t sourceFreq = 0U;
    uint32_t low, high;
    /* Define the clock mapping for each DSPI/SPI module */
    static const clock_names_t clock_Mapping[SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT] = FEATURE_DSPI_CLOCK_MAPPING;
    /* Define baud rate scalers */
    static const uint32_t baudRateScaler[] = {
        2U, 4U, 6U, 8U, 16U, 32U, 64U, 128U, 256U, 512U, 1024U, 2048U, 4096U, 8192U, 16384U, 32768U
    };
    clock_names_t clock_name = clock_Mapping[(uint32_t)instance];

    (void)CLOCK_SYS_GetFreq(clock_name, &sourceFreq);

    *bestPrescaler = 0U;
    *bestScaler = 0U;
    scaler = 0U;

    for (prescaler = 0; prescaler < DSPI_PRESCALER_NUM; prescaler++)
    {
        low = 0U;
        high = DSPI_SCALER_NUM;

        /* Implement golden section search algorithm */
        do
        {
            scaler = (low + high) / 2U;
            freq1 = sourceFreq / prescalerValues[prescaler] / baudRateScaler[scaler];
            if (abs_dif(requestedFreq, bestFreq) > abs_dif(requestedFreq, freq1))
            {
                bestFreq = freq1;
            }

            if (freq1 < requestedFreq)
            {
                high = scaler;
            }
            else
            {
                low = scaler;
            }
        }
        while ((high - low) > 1U);

        /* Evaluate last 2 scaler values */
        freq1 = sourceFreq / prescalerValues[prescaler] / baudRateScaler[low];
        freq2 = sourceFreq / prescalerValues[prescaler] / baudRateScaler[high];
        if (abs_dif(requestedFreq, freq1) > abs_dif(requestedFreq, freq2))
        {
            tempBestFreq   = freq2;
            tempScaler     = high;
        }
        else
        {
            tempBestFreq   = freq1;
            tempScaler     = low;
        }

        if (abs_dif(requestedFreq, bestFreq) >= abs_dif(requestedFreq, tempBestFreq))
        {
            bestFreq       = tempBestFreq;
            *bestPrescaler = prescaler;
            *bestScaler    = tempScaler;
        }

        /* If current frequency is equal to target frequency  stop the search */
        if (bestFreq == requestedFreq)
        {
            break;
        }
    }
}

static void DSPI_GetBestDelay(uint32_t requestedDelay,
                              uint8_t * bestPrescaler,
                              uint8_t * bestScaler)
{
    uint32_t bestDelay = 0U;
    uint32_t currentDelay;
    uint8_t scaler, prescaler;
    uint32_t sourceFreq = 80000000U;
    uint32_t low, high;

    *bestPrescaler = 0U;
    *bestScaler = 0U;
    scaler = 0U;

    for (prescaler = 0; prescaler < DSPI_PRESCALER_NUM; prescaler++)
    {
        low = 0U;
        high = DSPI_SCALER_NUM;
        while ((high - low) > 1U)
        {
            scaler = (low + high) / 2U;
            if (scaler < 12U)
            {
                currentDelay = (((uint32_t)(prescalerValues[prescaler]) << ((uint32_t)scaler + 1U)) * MICROSECONDS) / sourceFreq;
            }
            else
            {
                currentDelay = (((uint32_t)(prescalerValues[prescaler]) << ((uint32_t)scaler + 1U)) / sourceFreq) * MICROSECONDS;
            }

            if (abs_dif(requestedDelay, bestDelay) > abs_dif(requestedDelay, currentDelay))
            {
                bestDelay = currentDelay;
                *bestPrescaler = prescaler;
                *bestScaler = scaler;
            }

            if (currentDelay < requestedDelay)
            {
                low = scaler;
            }
            else
            {
                high = scaler;
            }
        }
    }
}

/*!
 * @brief Configures the DSPI master mode bus timing delay options.
 *
 * This function involves the DSPI module's delay options to
 * "fine tune" some of the signal timings and match the timing needs of a slower peripheral device.
 * This is an optional function that can be called after the DSPI module has been initialized for
 * master mode. The timings are adjusted in terms of cycles of the baud rate clock.
 * The bus timing delays that can be adjusted are listed below:
 *
 * SCK to PCS Delay: Adjustable delay option between the last edge of SCK to the de-assertion
 *                   of the PCS signal.
 *
 * PCS to SCK Delay: Adjustable delay option between the assertion of the PCS signal to the
 *                   first SCK edge.
 *
 * Delay between Transfers: Adjustable delay option between the de-assertion of the PCS signal for
 *                          a frame to the assertion of the PCS signal for the next frame.
 *
 * Implements : DSPI_MasterSetDelay_Activity
 */
status_t DSPI_MasterSetDelay(dspi_instance_t instance,
                             uint32_t delayBetweenTransfers,
                             uint32_t delaySCKtoPCS,
                             uint32_t delayPCStoSCK)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));

    DSPI_Type * base = DSPI_Instances[instance];
    uint8_t scaler, prescaler;

    /* Set the device in HALT mode. */
    DSPI_Set_MCR_HALT(base, 1U);
    /* Configure the delay between transfers. */
    DSPI_GetBestDelay(delayBetweenTransfers, &prescaler, &scaler);
    DSPI_Set_CTAR_PDT(base, 0U, prescaler);
    DSPI_Set_CTAR_DT(base, 0u, scaler);
    /* Configure the delay between SCK and PCS. */
    DSPI_GetBestDelay(delaySCKtoPCS, &prescaler, &scaler);
    DSPI_Set_CTAR_PASC(base, 0U, prescaler);
    DSPI_Set_CTAR_ASC(base, 0U, scaler);
    /* Configure the delay between PCS and SCK. */
    DSPI_GetBestDelay(delayPCStoSCK, &prescaler, &scaler);
    DSPI_Set_CTAR_PCSSCK(base, 0U, prescaler);
    DSPI_Set_CTAR_CSSCK(base, 0U, scaler);
    /* Set the device in RUN mode */
    DSPI_Set_MCR_HALT(base, 0UL);

    return STATUS_SUCCESS;
}

/*!
 * @brief This function returns a default configuration for DSPI module in SPI master mode
 *
 * After this call config will contain the following DSPI - SPI master setup:
 *          -Frame size 8 bits
 *          -Clock frequency 100kHz
 *          -Clock active high
 *          -Sampling on the first edge
 *          -MSB first
 *          -Transfer is perform interrupt base
 *          -PCS is active low
 * Implements : DSPI_GetDefaultMasterCfg_Activity
 */
status_t DSPI_GetDefaultMasterCfg(dspi_master_config_t * config)
{
    DEV_ASSERT(config != NULL);

    config->bitcount = 8U;
    config->bitsPerSec = 100000U;
    config->callback = NULL;
    config->callbackParam = NULL;
    config->clkPolarity = DSPI_ACTIVE_HIGH;
    config->pcsPolarity = DSPI_ACTIVE_LOW;
    config->clkPhase = DSPI_CLOCK_PHASE_1ST_EDGE;
    config->isClkContinuous = false;
    config->lsbFirst = false;
    config->transferType = DSPI_USING_INTERRUPTS;
    config->continuousPCS = false;
    config->whichPCS = 0U;

    return STATUS_SUCCESS;
}

/*!
 * @brief Initializes a DSPI instance for  master mode operation.
 *
 * This function uses an interrupt-driven or DMA method for transferring data.
 * In this function, the term "config" is used to indicate the SPI device for which the DSPI
 * master is communicating.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the DSPI module, resets the DSPI module,
 * configures the IRQ state structure, enables the module-level interrupt to the core, and
 * enables the DSPI module.
 *
 * Implements : DSPI_MasterInit_Activity
 */
status_t DSPI_MasterInit(dspi_instance_t instance,
                         dspi_state_t * state,
                         const dspi_master_config_t * config)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(config->whichPCS < PCS_Mapping[instance]);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(DSPI_state[instance] == NULL);
    /* If the applications needs frames bigger than 32 bits/frame or 16 bits/frame depending on platform please use continuous mode. */
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    DEV_ASSERT(config->bitcount <= 32U);
#else
    DEV_ASSERT(config->bitcount <= 16U);
#endif
    /* If extended mode is not supported DMA transfers are not accepted */
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    DEV_ASSERT(config != NULL);
#endif
    DSPI_Type * base = DSPI_Instances[instance];
    uint8_t scaler, prescaler;
    status_t status;
    uint32_t polarity = 0;
    uint8_t i;

    /* Disable module. */
    DSPI_Set_MCR_HALT(base, 1U);
    /* Configure module mode as master. */
    DSPI_Set_MCR_MSTR(base, MASTER_MODE);
    /* Configure if CLK continuous mode. */
    DSPI_Set_MCR_CONT_SCKE(base, (uint32_t)(config->isClkContinuous));
    /* Configure as SPI. */
    DSPI_Set_MCR_DCONF(base, DSPI_SPI_MODE);
    /* Configure PCS polarity. */
    for (i = 0; i < 8U; i++)
    {
        polarity |= (uint32_t)config->pcsPolarity << i;
    }

    DSPI_Set_MCR_PCSIS(base, polarity);
    /* Clock enable. */
    DSPI_Set_MCR_MDIS(base, 0U);
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* Enable DSPI extended mode every time. */
    DSPI_Set_MCR_XSPI(base, 1U);
#endif
    /* Configure DSPI frame size for extended mode. */
    DSPI_Set_CTAR_FMSZ(base, 0U, ((uint32_t)config->bitcount - 1U) & (~DSPI_EXTENDED_FRAME_SIZE_MASK));
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    DSPI_Set_CTARE_FMSZE(base, 0U, ((uint32_t)config->bitcount - 1U) >> DSPI_EXTENDED_FRAME_SIZE_SHIFT);
#endif
    /* Configure CLK signal properties. */
    DSPI_Set_CTAR_CPHA(base, 0U, (uint32_t)config->clkPhase);
    DSPI_Set_CTAR_CPOL(base, 0U, (uint32_t)config->clkPolarity);
    /* Configure if the transfer use LSB or MSB. */
    DSPI_Set_CTAR_LSBFE(base, 0U, (uint32_t)config->lsbFirst);
    /* Configure CLK frequency. */
    DSPI_GetBestFreq(instance, config->bitsPerSec, &prescaler, &scaler);
    DSPI_Set_CTAR_BR(base, 0U, scaler);
    DSPI_Set_CTAR_PBR(base, 0U, prescaler);
    /* Configure default delays for SPI signals */
    /* Configure the delay between transfers. */
    DSPI_Set_CTAR_PDT(base, 0U, prescaler);
    DSPI_Set_CTAR_DT(base, 0u, (uint32_t)scaler >> 2U);
    /* Configure the delay between SCK and PCS. */
    DSPI_Set_CTAR_PASC(base, 0U, prescaler);
    DSPI_Set_CTAR_ASC(base, 0U, (uint32_t)scaler >> 2U);
    /* Configure the delay between PCS and SCK. */
    DSPI_Set_CTAR_PCSSCK(base, 0U, prescaler);
    DSPI_Set_CTAR_CSSCK(base, 0U, (uint32_t)scaler >> 2U);

    /* Clear RFDF flag */
    DSPI_Clear_SR_RFDF(base);

    /* Configure state structure. */
    if (config->bitcount <= (uint16_t)8U)
    {
        state->bytesPerFrame = 1U;
    }
    else if (config->bitcount <= (uint16_t)16U)
    {
        state->bytesPerFrame = 2U;
    }
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    else if (config->bitcount <=  (uint16_t)32U)
    {
        state->bytesPerFrame = 4U;
    }
#endif
    else
    {
        /* Nothing to do */
    }

    state->toReceiveIndex = 0U;
    state->toSendIndex = 0U;
    state->lsbFirst = config->lsbFirst;
    state->transferType = config->transferType;
    state->rxDMAChannel = config->rxDMAChannel;
    state->txDMAChannel = config->txDMAChannel;
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    state->txAdditionalDMAChannel = config->txAdditionalDMAChannel;
#endif
    state->callback = config->callback;
    state->callbackParam = config->callbackParam;
    state->status = DSPI_TRANSFER_OK;
    state->mode = DSPI_MASTER;
    state->continuousPCS = config->continuousPCS;
    state->whichPCS = config->whichPCS;
    /* Enable IRQ if interrupt mode is selected. */
    if (config->transferType == DSPI_USING_INTERRUPTS)
    {
        /* Configure IRQs. */
        DSPI_Set_RSER_TFFF_RE(base, 0U);
        DSPI_Set_RSER_RFDF_RE(base, 0U);
        #ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
        INT_SYS_EnableIRQ(dspiSendInterruptVector[instance]);
        INT_SYS_EnableIRQ(dspiRecieveInterruptVector[instance]);
        #endif
    }
    else
    {
        /* Connect RX/TX flags to DMA requests */
        DSPI_Set_RSER_RFDF_DIRS(base, 1U);
        DSPI_Set_RSER_TFFF_DIRS(base, 1U);
    }

    /* Enable FIFO errors interrupt to handle RX FIFO overflow or TX FIFO underflow */
    DSPI_Set_RSER_RFOF_RE(base, 0U);
    DSPI_Set_RSER_TFUF_RE(base, 0U);
    #ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
    INT_SYS_EnableIRQ(dspiFaultInterruptVector[instance]);
    #endif
    #ifdef FEATIRES_DSPI_HAS_SHARED_INTERRUPTS
    INT_SYS_EnableIRQ((IRQn_Type)(FEATURES_DSPI_INTERUPT_BASE + (uint32_t)instance));
    #endif
    /* Initialize the semaphore for blocking transfer */
    status = OSIF_SemaCreate(&(state->dspiSemaphore), 0);

    if (status != STATUS_SUCCESS)
    {

        /* OSIF can't create the semaphore */
        return STATUS_ERROR;
    }

    /* Store default mode in state structure */
    state->isBlocking = false;
    /* Store the state structure in internal memory */
    DSPI_state[instance] = state;

    return STATUS_SUCCESS;
}


/*!
 * @brief Performs an non-blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function returns immediately after initiating the transfer. The user
 * needs to check whether the transfer is complete using the DSPI_GetTransferStatus
 * function.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 * Implements : DSPI_MasterTransfer_Activity
 */
status_t DSPI_MasterTransfer(dspi_instance_t instance,
                             const void * sendBuffer,
                             void * receiveBuffer,
                             uint16_t frames)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(DSPI_state[instance] != NULL);
    DEV_ASSERT(receiveBuffer != NULL);
    DEV_ASSERT(sendBuffer != NULL);

    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];
    edma_transfer_size_t dmaTxTransferSize = EDMA_TRANSFER_SIZE_1B;
    edma_transfer_size_t dmaRxTransferSize = EDMA_TRANSFER_SIZE_1B;
    uint8_t offsetRX = 0;
#if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
    uint8_t offsetTX = 0;
#endif

#if (!(defined FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* On devices which doesn't have extended mode and SPI is in DMA mode the number of frames must be more than 1 */
    if ((state->transferType == DSPI_USING_DMA) && (frames < 2U))
    {
        DEV_ASSERT(0);
    }

#endif

    if (state->status == DSPI_IN_PROGRESS)
    {
        return STATUS_BUSY;
    }

#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* Save in state structure details about the next transfer. */
    if (state->bytesPerFrame == 4U)
    {
        state->framesToSend = frames * 2U;
    }
    else
    {
#endif
        state->framesToSend = frames;
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    }
#endif

    state->framesToReceive = frames;
    state->lastFrameSent = false;
    state->txBuffer = (const void *)sendBuffer;
    state->rxBuffer = receiveBuffer;
    state->toSendIndex = 0U;
    state->toReceiveIndex = 0U;
    state->configWord = 0U;
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    state->lastConfigWord = 0U;
#endif
    /* Calculate the command word which will be write in PUSHR register every time. */
    state->configWord =
        (((uint32_t)(state->continuousPCS) << (uint16_t)DSPI_PUSHR_CONT_SHIFT) | ((1UL << state->whichPCS) << (uint16_t)DSPI_PUSHR_PCS_SHIFT) |
         (1UL << (uint16_t)DSPI_PUSHR_CTCNT_SHIFT));

    state->status = DSPI_IN_PROGRESS;
    /* Clear FIFOs */
    DSPI_Set_MCR_CLR_TXF(base, 1UL);
    DSPI_Set_MCR_CLR_RXF(base, 1UL);
    (void)DSPI_Get_POPR_RXDATA(base);

    /* Disable module. */
    DSPI_Set_MCR_HALT(base, 1U);
    /* Clear EOQ flag */
    DSPI_Clear_SR_EOQF(base);
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* Configure commands */
    /* Extend configuration for for both configurations */
    base->CTARE[1] = base->CTARE[0];
    base->MODE.CTAR[1] = base->MODE.CTAR[0];

    if (state->continuousPCS == false)
    {
        DSPI_Set_CTARE_DTCP(base, 0U, (uint32_t)frames - 1U);
        DSPI_Set_PUSHR_CMD(base, (uint16_t)((state->configWord) >> 16UL));
        /* Push end of queue command */
        DSPI_Set_CTARE_DTCP(base, 1U, 1U);
        DSPI_Set_PUSHR_CMD(base, (uint16_t)((state->configWord | (1UL << DSPI_PUSHR_EOQ_SHIFT) | (DSPI_PUSHR_CTAS(1U))) >> 16UL));
    }
    else
    {
        DSPI_Set_CTARE_DTCP(base, 0U, (uint32_t)frames - 1U);
        DSPI_Set_PUSHR_CMD(base, (uint16_t)(state->configWord >> 16U));
        DSPI_Set_CTARE_DTCP(base, 1U, 1U);
        /* Push end of queue command and clear continuous. */
        state->configWord &=  ~DSPI_PUSHR_CONT_MASK;
        state->configWord = (state->configWord & ~DSPI_PUSHR_CTCNT_MASK) | DSPI_PUSHR_CTAS(1U) | (1UL << (uint16_t)DSPI_PUSHR_EOQ_SHIFT);
        DSPI_Set_PUSHR_CMD(base, (uint16_t)(state->configWord >> 16U));
    }

#endif /* if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE)) */

    if (state->transferType == DSPI_USING_INTERRUPTS)
    {
        /* Nothing to do */
    }
    else
    {
        /* Due to the PUSHR and POPR resolution and the endianess is necessary to
         * determine the offset for POPR register and the DMA transfer size
         */
        switch (state->bytesPerFrame)
        {
            case 1:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_1B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_1B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT               
                offsetRX = 3U;
                #if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
                offsetTX = 1U;
                #endif
            #endif
                break;
            case 2:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_2B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_2B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT
                offsetRX = 2U;
                #if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
                offsetTX = 0U;
                #endif
            #endif
                break;
#if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
            case 4:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_2B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_4B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT
                offsetRX = 0U;
                offsetTX = 0U;
            #endif
                break;
#endif
            default: /* Nothing to do */
                break;
        }
#if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
        /* Configure TX DMA channel */
        (void)EDMA_DRV_ConfigMultiBlockTransfer(state->txDMAChannel,
                                                EDMA_TRANSFER_MEM2PERIPH,
                                                (uint32_t)sendBuffer,
                                                (uint32_t)((uint32_t)(&(base->PUSHR.FIFO.TX)) + (uint32_t)offsetTX),
                                                dmaTxTransferSize,
                                                (uint32_t)1U << (uint8_t)(dmaTxTransferSize),
                                                ((uint32_t)frames * (uint32_t)state->bytesPerFrame) /
                                                (uint32_t)((uint32_t)1U << (uint32_t)(dmaTxTransferSize)),
        true);

        (void)EDMA_DRV_ConfigMultiBlockTransfer(state->rxDMAChannel,
                                                EDMA_TRANSFER_PERIPH2MEM,
                                                (uint32_t)((uint32_t)(&(base->POPR)) + offsetRX),
                                                (uint32_t)receiveBuffer,
                                                dmaRxTransferSize,
                                                (uint32_t)1U << (uint8_t)(dmaRxTransferSize),
                                                ((uint32_t)frames * (uint32_t)state->bytesPerFrame) /
                                                (uint32_t)((uint32_t)1U << (uint8_t)(dmaRxTransferSize)),
            true);
        /* Start RX channel */
        (void)EDMA_DRV_InstallCallback(state->rxDMAChannel, (DSPI_DRV_MasterCompleteDMATransfer), (void *)(instance));
        (void)EDMA_DRV_StartChannel(state->rxDMAChannel);

        /* Start TX channel */
        (void)EDMA_DRV_StartChannel(state->txDMAChannel);
#else  /* if (defined FEATURE_DSPI_HAS_EXTENDED_MODE) */
        state->lastConfigWord = state->configWord | (1UL << DSPI_PUSHR_EOQ_SHIFT);
        if (state->continuousPCS == true)
        {
            state->lastConfigWord &= ~DSPI_PUSHR_CTAS_MASK;
        }

        /*Configure transfer descriptors */
        edma_transfer_config_t transferDescriptors[2];
        edma_loop_transfer_config_t loopConfigurations[2];

        /*Configure the transfer from buffer to configWord*/
        loopConfigurations[0].majorLoopIterationCount = state->framesToSend;
        loopConfigurations[0].srcOffsetEnable = false;
        loopConfigurations[0].dstOffsetEnable = false;
        loopConfigurations[0].majorLoopChnLinkEnable = false;
        loopConfigurations[0].minorLoopChnLinkEnable = true;
        loopConfigurations[0].minorLoopChnLinkNumber = state->txAdditionalDMAChannel;

        transferDescriptors[0].srcAddr = (uint32_t)(state->txBuffer);
        transferDescriptors[0].destAddr = (uint32_t)(&(state->configWord)) + offsetRX;
        transferDescriptors[0].srcTransferSize = dmaTxTransferSize;
        transferDescriptors[0].destTransferSize = dmaTxTransferSize;
        transferDescriptors[0].srcOffset = (int16_t)(1U << dmaTxTransferSize);
        transferDescriptors[0].destOffset = 0;
        transferDescriptors[0].srcModulo = EDMA_MODULO_OFF;
        transferDescriptors[0].destModulo = EDMA_MODULO_OFF;
        transferDescriptors[0].minorByteTransferCount = (uint32_t)(1U << dmaTxTransferSize);
        transferDescriptors[0].loopTransferConfig = &loopConfigurations[0];
        transferDescriptors[0].interruptEnable = true;
        transferDescriptors[0].srcLastAddrAdjust = 0;
        transferDescriptors[0].destLastAddrAdjust = 0;
        transferDescriptors[0].scatterGatherEnable = false;

        /*Configure the transfer from ConfigWord to PUSHR*/
        loopConfigurations[1].majorLoopIterationCount = state->framesToSend;
        loopConfigurations[1].srcOffsetEnable = false;
        loopConfigurations[1].dstOffsetEnable = false;
        loopConfigurations[1].majorLoopChnLinkEnable = false;
        loopConfigurations[1].minorLoopChnLinkNumber = 0U;
        loopConfigurations[1].minorLoopChnLinkEnable = false;

        transferDescriptors[1].srcAddr = (uint32_t)(&(state->configWord));
        transferDescriptors[1].destAddr = (uint32_t)(&(base->PUSHR));
        transferDescriptors[1].srcTransferSize = EDMA_TRANSFER_SIZE_4B;
        transferDescriptors[1].destTransferSize = EDMA_TRANSFER_SIZE_4B;
        transferDescriptors[1].srcOffset = 0;
        transferDescriptors[1].destOffset = 0;
        transferDescriptors[1].srcModulo = EDMA_MODULO_OFF;
        transferDescriptors[1].destModulo = EDMA_MODULO_OFF;
        transferDescriptors[1].minorByteTransferCount = 4U;
        transferDescriptors[1].scatterGatherEnable = false;
        transferDescriptors[1].srcLastAddrAdjust = 0;
        transferDescriptors[1].destLastAddrAdjust = 0;
        transferDescriptors[1].interruptEnable = false;
        transferDescriptors[1].loopTransferConfig = &loopConfigurations[1];

        (void)EDMA_DRV_ConfigLoopTransfer(state->txDMAChannel, &transferDescriptors[0]);
        (void)EDMA_DRV_ConfigLoopTransfer(state->txAdditionalDMAChannel, &transferDescriptors[1]);

        /* Configure RX DMA channel */
        (void)EDMA_DRV_ConfigMultiBlockTransfer(state->rxDMAChannel,
                                                EDMA_TRANSFER_PERIPH2MEM,
                                                (uint32_t)((uint32_t)(&(base->POPR)) + offsetRX),
                                                (uint32_t)receiveBuffer,
                                                dmaRxTransferSize,
                                                (uint32_t)1U << (uint8_t)(dmaRxTransferSize),
                                                ((uint32_t)frames * (uint32_t)state->bytesPerFrame) /
                                                (uint32_t)((uint32_t)1U << (uint8_t)(dmaRxTransferSize)),
            true);
        /* Start RX channel */
        (void)EDMA_DRV_InstallCallback(state->rxDMAChannel, (DSPI_DRV_MasterCompleteDMATransfer), (void *)(instance));
        (void)EDMA_DRV_InstallCallback(state->txDMAChannel, (DSPI_DRV_MasterCompleteTxDMATransfer), (void *)(instance));
        (void)EDMA_DRV_StartChannel(state->rxDMAChannel);

        (void)EDMA_DRV_StartChannel(state->txDMAChannel);
        (void)EDMA_DRV_StartChannel(state->txAdditionalDMAChannel);

        (void)EDMA_DRV_DisableRequestsOnTransferComplete(state->txDMAChannel, true);
        (void)EDMA_DRV_DisableRequestsOnTransferComplete(state->txAdditionalDMAChannel, true);

#endif /* if (defined FEATURE_DSPI_HAS_EXTENDED_MODE) */
    }

    /* Clear all flags used by driver */
    DSPI_Clear_SR_TFFF(base);
    DSPI_Clear_SR_RFDF(base);
    DSPI_Clear_SR_RFOF(base);
    DSPI_Clear_SR_TFUF(base);

    /* Enable RX/TX interrupt request. */
    DSPI_Set_RSER_RFDF_RE(base, 1U);
    DSPI_Set_RSER_TFFF_RE(base, 1U);
    DSPI_Set_RSER_RFOF_RE(base, 1U);
    DSPI_Set_RSER_TFUF_RE(base, 1U);

    /* Enable module */
    DSPI_Set_MCR_HALT(base, 0U);

    return STATUS_SUCCESS;
}

/*!
 * @brief Performs an blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 * Implements : DSPI_MasterTransferBlocking_Activity
 */
status_t DSPI_MasterTransferBlocking(dspi_instance_t instance,
                                     const void * sendBuffer,
                                     void * receiveBuffer,
                                     uint16_t frames,
                                     uint32_t timeout)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));

    status_t status;
    dspi_state_t * state = DSPI_state[instance];

    if (state->status == DSPI_IN_PROGRESS)
    {
        return STATUS_BUSY;
    }

    state->isBlocking = true;
    (void)OSIF_SemaWait(&(state->dspiSemaphore), 0);
    status = DSPI_MasterTransfer(instance, sendBuffer, receiveBuffer, frames);
    if (status == STATUS_SUCCESS)
    {
        status = OSIF_SemaWait(&(state->dspiSemaphore), timeout);
        if (status != STATUS_SUCCESS)
        {
            (void)DSPI_AbortTransfer(instance);
            state->status = DSPI_TRANSFER_FAIL;

            return status;
        }
    }

    return STATUS_SUCCESS;
}

/*!
 * @brief Terminates an asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 *
 * Implements : DSPI_AbortTransfer_Activity
 */
status_t DSPI_AbortTransfer(dspi_instance_t instance)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(DSPI_state[instance] != NULL);

    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];

    /* Halt DSPI */
    DSPI_Set_MCR_HALT(base, 1U);

    /* Disable and clear all interrupts. */
    DSPI_Set_RSER_TFFF_RE(base, 0U);
    DSPI_Set_RSER_RFDF_RE(base, 0U);
    DSPI_Set_RSER_RFOF_RE(base, 0U);
    DSPI_Set_RSER_TFUF_RE(base, 0U);
    DSPI_Clear_SR_RFOF(base);
    DSPI_Clear_SR_TFUF(base);
    DSPI_Clear_SR_RFDF(base);
    DSPI_Clear_SR_TFFF(base);

    /* Restore transfer mode to default value */
    state->isBlocking = false;
    state->status = DSPI_TRANSFER_OK;

    return STATUS_SUCCESS;
}

/*!
 * @brief Change the chip select used by DSPI driver.
 *
 * This function can be used to change the chip select configured when the driver was initialized.
 * It is useful for applications where more slave devices are controlled by only one dspi instance.
 * On MPC47xx platform this function should be used only for master mode. In Slave mode only one chip
 * select is available.
 *
 * Implements : DSPI_UpdateCS_Activity
 */
status_t DSPI_UpdateCS(dspi_instance_t instance,
                       uint8_t whichPCS)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(whichPCS < PCS_Mapping[instance]);

    dspi_state_t * state = DSPI_state[instance];

    state->whichPCS = whichPCS;

    return STATUS_SUCCESS;
}

/*!
 * @brief Change the frame size of the spi transfer
 *
 * Implements : DSPI_SetFrameSize_Activity
 */
status_t DSPI_SetFrameSize(dspi_instance_t instance, uint8_t frameSize)
{

        DSPI_Type * base = DSPI_Instances[instance];
        dspi_state_t * state = DSPI_state[instance];

        /* Disable module. */
        DSPI_Set_MCR_HALT(base, 1U);

        if(state->mode == DSPI_MASTER)
        {
            /* Configure DSPI frame size for master mode. */
            DSPI_Set_CTAR_FMSZ(base, 0U, ((uint32_t)frameSize - 1U) & (~DSPI_EXTENDED_FRAME_SIZE_MASK));
        #if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
            DSPI_Set_CTARE_FMSZE(base, 0U, ((uint32_t)frameSize - 1U) >> DSPI_EXTENDED_FRAME_SIZE_SHIFT);
        #endif
        }
        else
        {
            /* Configure DSPI frame size for slave mode. */
            DSPI_Set_CTAR_SLAVE_FMSZ(base, ((uint32_t)frameSize - 1U));
        }

        /* Configure state structure. */
        if (frameSize <= (uint16_t)8U)
        {
            state->bytesPerFrame = 1U;
        }
        else if (frameSize <= (uint16_t)16U)
        {
            state->bytesPerFrame = 2U;
        }
    #if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))  
        else if (frameSize <= (uint16_t)32U)
        {
            state->bytesPerFrame = 4U;
        }
    #endif
        else
        {
            /* Nothing to do */
        }

        /* Enable module. */
        DSPI_Set_MCR_HALT(base, 0U);

        return STATUS_SUCCESS;
}

/*!
 * @brief Enable or disable continuous mode 
 *
 * Implements : DSPI_SetContinuousMode_Activity
 */
status_t DSPI_SetContinuousMode(dspi_instance_t instance, bool enable)
{
    dspi_state_t * state = DSPI_state[instance];
    state->continuousPCS = enable;
    return STATUS_SUCCESS;
}
/*!
 * @brief This function returns a default configuration for DSPI module in SPI slave mode
 *
 * Implements : DSPI_GetDefaultSlaveCfg_Activity
 */
status_t DSPI_GetDefaultSlaveCfg(dspi_slave_config_t * config)
{
    DEV_ASSERT(config != NULL);

    config->bitcount = 8U;
    config->callback = NULL;
    config->callbackParam = NULL;
    config->clkPolarity = DSPI_ACTIVE_HIGH;
    config->clkPhase = DSPI_CLOCK_PHASE_1ST_EDGE;
    config->transferType = DSPI_USING_INTERRUPTS;

    return STATUS_SUCCESS;
}

/*!
 * @brief Initializes a DSPI instance for slave mode operation.
 *
 * This function uses an interrupt-driven or DMA method for transferring data.
 * In this function, the term "config" is used to indicate the SPI device for which the DSPI
 * slave is communicating.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the DSPI module, resets the DSPI module,
 * configures the IRQ state structure, enables the module-level interrupt to the core, and
 * enables the DSPI module.
 *
 * Implements : DSPI_SlaveInit_Activity
 */
status_t DSPI_SlaveInit(dspi_instance_t instance,
                        dspi_state_t * state,
                        const dspi_slave_config_t * config)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(DSPI_state[instance] == NULL);
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    DEV_ASSERT(config->bitcount <= 32U);
#else
    DEV_ASSERT(config->bitcount <= 16U);
#endif

    DSPI_Type * base = DSPI_Instances[instance];
    status_t status;

    /* Disable module. */
    DSPI_Set_MCR_HALT(base, 1U);
    /* Configure module mode as master. */
    DSPI_Set_MCR_MSTR(base, SLAVE_MODE);
    /* Configure as SPI. */
    DSPI_Set_MCR_DCONF(base, DSPI_SPI_MODE);
    /* Clock enable. */
    DSPI_Set_MCR_MDIS(base, 0U);
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* Enable DSPI extended mode every time. */
    DSPI_Set_MCR_XSPI(base, 1U);
#endif
    /* Configure DSPI frame size for extended mode. */
    DSPI_Set_CTAR_SLAVE_FMSZ(base, ((uint32_t)config->bitcount - 1U));
    /* Configure CLK signal properties. */
    DSPI_Set_CTAR_SLAVE_CPHA(base, (uint32_t)config->clkPhase);
    DSPI_Set_CTAR_CPOL(base, 0U, (uint32_t)config->clkPolarity);
    /* Clear RFDF flag */
    DSPI_Clear_SR_RFDF(base);

    /* Configure state structure. */
    if (config->bitcount <= 8U)
    {
        state->bytesPerFrame = 1U;
    }
    else
    {
        if (config->bitcount <= 16U)
        {
            state->bytesPerFrame = 2U;
        }
        else
        {
            state->bytesPerFrame = 4U;
        }
    }

    state->toReceiveIndex = 0U;
    state->toSendIndex = 0U;
    state->lsbFirst = false;
    state->transferType = config->transferType;
    state->rxDMAChannel = config->rxDMAChannel;
    state->txDMAChannel = config->txDMAChannel;
    state->callback = config->callback;
    state->callbackParam = config->callbackParam;
    state->status = DSPI_TRANSFER_OK;
    state->mode = DSPI_SLAVE;
    state->lastFrameSent = false;
    state->configWord = 0U;

    if (config->transferType == DSPI_USING_INTERRUPTS)
    {
        /* Configure IRQs. */
        DSPI_Set_RSER_TFFF_RE(base, 0U);
        DSPI_Set_RSER_RFDF_RE(base, 0U);
        #ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
        INT_SYS_EnableIRQ(dspiSendInterruptVector[instance]);
        INT_SYS_EnableIRQ(dspiRecieveInterruptVector[instance]);
        #endif
    }
    else
    {
        /* Connect RX/TX flags to DMA requests */
        DSPI_Set_RSER_RFDF_DIRS(base, 1U);
        DSPI_Set_RSER_TFFF_DIRS(base, 1U);
    }

    /* Enable FIFO errors interrupt to handle RX FIFO overflow or TX FIFO underflow */
    DSPI_Set_RSER_RFOF_RE(base, 0U);
    DSPI_Set_RSER_TFUF_RE(base, 0U);
    #ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
    INT_SYS_EnableIRQ(dspiFaultInterruptVector[instance]);
    #endif
    #ifdef FEATIRES_DSPI_HAS_SHARED_INTERRUPTS
    INT_SYS_EnableIRQ((IRQn_Type)(FEATURES_DSPI_INTERUPT_BASE + (uint32_t)instance));
    #endif

    /* Initialize the semaphore for blocking transfer */
    status = OSIF_SemaCreate(&(state->dspiSemaphore), 0);

    if (status != STATUS_SUCCESS)
    {

        /* OSIF can't create the semaphore */
        return STATUS_ERROR;
    }

    /* Store default mode in state structure */
    state->isBlocking = false;
    /* Store the state strucure in internal memory */
    DSPI_state[instance] = state;

    return STATUS_SUCCESS;
}

/*!
 * @brief Performs an non-blocking SPI slave mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function returns immediately after initiating the transfer. The user
 * needs to check whether the transfer is complete using the DSPI_GetTransferStatus
 * function.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 * Implements : DSPI_SlaveTransfer_Activity
 */
status_t DSPI_SlaveTransfer(dspi_instance_t instance,
                            const void * sendBuffer,
                            void * receiveBuffer,
                            uint16_t frames)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(DSPI_state[instance] != NULL);
    DEV_ASSERT(receiveBuffer != NULL);
    DEV_ASSERT(sendBuffer != NULL);

    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];
    edma_transfer_size_t dmaTxTransferSize = EDMA_TRANSFER_SIZE_1B;
    edma_transfer_size_t dmaRxTransferSize = EDMA_TRANSFER_SIZE_1B;
    uint8_t offsetRX = 0;
    uint8_t offsetTX = 0;

    if (state->status == DSPI_IN_PROGRESS)
    {
        return STATUS_BUSY;
    }

    /* Clear FIFOs */
    DSPI_Set_MCR_CLR_TXF(base, 1);
    DSPI_Set_MCR_CLR_RXF(base, 1);
    (void)DSPI_Get_POPR_RXDATA(base);

    /* Disable module. */
    DSPI_Set_MCR_HALT(base, 1U);

#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    /* Save in state structure details about the next transfer. */
    if (state->bytesPerFrame == 4U)
    {
        state->framesToSend = frames * 2U;
    }
    else
    {
#endif
        state->framesToSend = frames;
    #if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    }
    #endif

    state->framesToReceive = frames;
    state->lastFrameSent = false;
    state->txBuffer = (const void *)sendBuffer;
    state->rxBuffer = receiveBuffer;
    state->toSendIndex = 0U;
    state->toReceiveIndex = 0U;
#if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    DSPI_Set_CTARE_DTCP(base, 0U, frames);
#endif
    state->status = DSPI_IN_PROGRESS;
    if (state->transferType == DSPI_USING_INTERRUPTS)
    {
        /* Nothing to do */
    }
    else
    {
        /* Due to the PUSHR and POPR resolution and the endianess is necessary to
         * determine the offset for POPR register and the DMA transfer size
         */
        switch (state->bytesPerFrame)
        {
            case 1:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_1B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_1B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT
                offsetRX = 3U;
                offsetTX = 1U;
            #endif
                break;
            case 2:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_2B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_2B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT
                offsetRX = 2U;
                offsetTX = 0U;
            #endif
                break;
#if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
            case 4:
                dmaTxTransferSize = EDMA_TRANSFER_SIZE_2B;
                dmaRxTransferSize = EDMA_TRANSFER_SIZE_4B;
            #ifndef FEATIRES_DSPI_LITTLE_ENDIAN_FORMAT
                offsetRX = 0U;
                offsetTX = 0U;
            #endif
                break;
#endif
            default: /* Nothing to do */
                break;
        }
        /* Configure TX DMA channel */
        (void)EDMA_DRV_ConfigMultiBlockTransfer(state->txDMAChannel,
                                                EDMA_TRANSFER_MEM2PERIPH,
                                                (uint32_t)sendBuffer,
                                                (uint32_t)((uint32_t)(&(base->PUSHR.FIFO.TX)) + offsetTX),
                                                dmaTxTransferSize,
                                                (uint32_t)1U << (uint8_t)(dmaTxTransferSize),
                                                ((uint32_t)frames * (uint32_t)state->bytesPerFrame) /
                                                (uint32_t)((uint32_t)1U << (uint8_t)(dmaTxTransferSize)),
        true);
        /* Start TX channel */
        (void)EDMA_DRV_StartChannel(state->txDMAChannel);
        (void)EDMA_DRV_ConfigMultiBlockTransfer(state->rxDMAChannel,
                                                EDMA_TRANSFER_PERIPH2MEM,
                                                (uint32_t)((uint32_t)(&(base->POPR)) + offsetRX),
                                                (uint32_t)receiveBuffer,
                                                dmaRxTransferSize,
                                                (uint32_t)1U << (uint8_t)(dmaRxTransferSize),
                                                ((uint32_t)frames * (uint32_t)state->bytesPerFrame) /
                                                (uint32_t)((uint32_t)1U << (uint8_t)(dmaRxTransferSize)),
            true);
        /* Start RX channel */
        (void)EDMA_DRV_StartChannel(state->rxDMAChannel);
        (void)EDMA_DRV_InstallCallback(state->rxDMAChannel, (DSPI_DRV_MasterCompleteDMATransfer), (void *)(instance));
    }

    /* Clear all flags used by driver */
    DSPI_Clear_SR_TFFF(base);
    DSPI_Clear_SR_RFDF(base);
    DSPI_Clear_SR_RFOF(base);
    DSPI_Clear_SR_TFUF(base);

    /* Enable RX/TX interrupt request. */
    DSPI_Set_RSER_TFFF_RE(base, 1U);
    DSPI_Set_RSER_RFDF_RE(base, 1U);
    DSPI_Set_RSER_RFOF_RE(base, 1U);
    DSPI_Set_RSER_TFUF_RE(base, 1U);
    DSPI_Set_MCR_HALT(base, 0U);

    return STATUS_SUCCESS;
}

/*!
 * @brief Performs an blocking SPI slave mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus.
 * This function allows the user to optionally pass in a SPI configuration structure which
 * allows the user to change the SPI bus attributes in conjunction with initiating a SPI transfer.
 * The difference between passing in the SPI configuration structure here as opposed to the
 * configure bus function is that the configure bus function returns the calculated baud rate where
 * this function does not. The user can also call the configure bus function prior to the transfer
 * in which case the user would simply pass in a NULL to the transfer function's device structure
 * parameter.
 *
 * Implements : DSPI_SlaveTransferBlocking_Activity
 */
status_t DSPI_SlaveTransferBlocking(dspi_instance_t instance,
                                    const void * sendBuffer,
                                    void * receiveBuffer,
                                    uint16_t frames,
                                    uint32_t timeout)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));

    status_t status;
    dspi_state_t * state = DSPI_state[instance];

    if (state->status == DSPI_IN_PROGRESS)
    {
        return STATUS_BUSY;
    }

    state->isBlocking = true;
    (void)OSIF_SemaWait(&(state->dspiSemaphore), 0);
    status = DSPI_SlaveTransfer(instance, sendBuffer, receiveBuffer, frames);
    if (status == STATUS_SUCCESS)
    {
        status = OSIF_SemaWait(&(state->dspiSemaphore), timeout);
        if (status != STATUS_SUCCESS)
        {
            (void)DSPI_AbortTransfer(instance);
            state->status = DSPI_TRANSFER_FAIL;

            return status;
        }
    }

    return STATUS_SUCCESS;
}

/*!
 * @brief Shuts down a DSPI instance.
 *
 * This function resets the DSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 *
 * Implements : DSPI_Deinit_Activity
 */
status_t DSPI_Deinit(dspi_instance_t instance)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(DSPI_state[instance] != NULL);

    const dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];
    status_t status = STATUS_SUCCESS;
    /* Reset registers */
    DSPI_Reset(base);
    /* Reset interrupts */
    #ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
    INT_SYS_DisableIRQ(dspiSendInterruptVector[instance]);
    INT_SYS_DisableIRQ(dspiRecieveInterruptVector[instance]);
    INT_SYS_DisableIRQ(dspiFaultInterruptVector[instance]);
    #endif
    #ifdef FEATIRES_DSPI_HAS_SHARED_INTERRUPTS
    INT_SYS_DisableIRQ((IRQn_Type)(FEATURES_DSPI_INTERUPT_BASE + (uint32_t)instance));
    #endif
    status = OSIF_SemaDestroy(&(state->dspiSemaphore));
    DEV_ASSERT(status == STATUS_SUCCESS);
    DSPI_state[instance] = NULL;

    return status;
}

/*!
 * @brief Get the current transfer status.
 *
 * This function return the status of the last transfer.
 *
 * Implements : DSPI_GetTransferStatus_Activity
 */
status_t DSPI_GetTransferStatus(dspi_instance_t instance,
                                dspi_transfer_status_t * status)
{
    DEV_ASSERT((uint32_t)instance < (SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT));
    DEV_ASSERT(DSPI_state[instance] != NULL);

    const dspi_state_t * state = DSPI_state[instance];
    *status = state->status;

    return STATUS_SUCCESS;
}

static void DSPI_UpdateTransferDoneState(dspi_instance_t instance)
{
    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];

    if (state->isBlocking)
    {
        (void)OSIF_SemaPost(&(state->dspiSemaphore));
        /* Restore transfer mode to default value */
        state->isBlocking = false;
    }

    state->status = DSPI_TRANSFER_OK;
    if (state->callback != NULL)
    {
        state->callback(state, SPI_EVENT_END_TRANSFER, state->callbackParam);
    }

    /* Halt DSPI */
    DSPI_Set_MCR_HALT(base, 1U);
}

static void DSPI_RECEIVE_isr(dspi_instance_t instance)
{
    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];
    uint32_t data;
    uint32_t framesToReceiveNow; /* This value store how many words(16 bits) can be read from RX FIFO */
    uint32_t index;

    framesToReceiveNow = DSPI_Get_SR_RXCTR(base);
    while ((state->framesToReceive != 0U) && (framesToReceiveNow != 0U))
    {
        data = DSPI_Get_POPR_RXDATA(base);
        index = state->toReceiveIndex;
        if (state->bytesPerFrame == (uint16_t)1U)
        {
            ((uint8_t *)(state->rxBuffer))[index] = (uint8_t)data;
        }
        else if (state->bytesPerFrame == (uint16_t)2U)
        {
            ((uint16_t *)(state->rxBuffer))[index] = (uint16_t)data;
        }
    #if (defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
        else if (state->bytesPerFrame == (uint16_t)4U)
        {
            ((uint32_t *)(state->rxBuffer))[index] = (uint32_t)data;
        }
    #endif
        else
        {
            /* Nothing to do */
        }

        state->toReceiveIndex++;
        state->framesToReceive--;
        framesToReceiveNow = DSPI_Get_SR_RXCTR(base);
    }

    DSPI_Clear_SR_RFDF(base);
    /*If all frames were received the RX interrupt is disabled. */
    if (state->framesToReceive == 0U)
    {
        DSPI_Set_RSER_RFDF_RE(base, 0U);
        if (state->framesToSend == 0U)
        {
            /* Disable DSPI module */
            DSPI_UpdateTransferDoneState(instance);

            /* Disable FIFOs interrupt errors */
            DSPI_Set_RSER_RFOF_RE(base, 0U);
            DSPI_Set_RSER_TFUF_RE(base, 0U);
        }
    }
}

static void DSPI_SEND_isr(dspi_instance_t instance)
{
    dspi_state_t * state = DSPI_state[instance];
    DSPI_Type * base = DSPI_Instances[instance];
    uint16_t data = 0;
    uint32_t framesToSendNow; /* This value will store how many words(16 bits) can be added to TX FIFO */
    uint32_t index;

    /* Fill TX buffer */
    /* Get the number of available words in FIFO */
    framesToSendNow = DSPI_FIFO_SIZE - DSPI_Get_SR_TXCTR(base);
    while ((state->framesToSend != 0U) && (framesToSendNow != 0U))
    {
        index = state->toSendIndex;
        /* Get current data to be send. */
        if (state->bytesPerFrame == 1U)
        {
            data = ((const uint8_t *)(state->txBuffer))[index];
        }
        else if (state->bytesPerFrame == 2U)
        {
            data = ((const uint16_t *)(state->txBuffer))[index];
        }
        else
        {
        #if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
            /* This is the case for state->bytesPerFrame == 4 and state->lsbFirst == false */
            if ((state->toSendIndex % (uint16_t)2U) == (uint16_t)0U)
            {
                data = ((const uint16_t *)(state->txBuffer))[index + 1U];
            }
            else
            {
                data = ((const uint16_t *)(state->txBuffer))[index - 1U];
            }
        #endif
        }

        /* Update all indexes used in transfer */
        state->toSendIndex++;
        state->framesToSend--;
        framesToSendNow--;
#if (defined FEATURE_DSPI_HAS_EXTENDED_MODE)
        DSPI_Set_PUSHR_TX(base, data);
#else
        /* Mark last frame as end of queue */
        if (state->framesToSend == 0U)
        {
            state->configWord |= (uint32_t)DSPI_PUSHR_EOQ_MASK;
            /* If current transfer is continuous clear CONT */
            if (state->continuousPCS)
            {
                state->configWord &= ~DSPI_PUSHR_CONT_MASK;
            }
        }

        DSPI_Set_PUSHR(base, data | state->configWord);
#endif /* if (defined FEATURE_DSPI_HAS_EXTENDED_MODE) */
    }

    /*If all frames were send the TX interrupt is disabled. */
    if (state->framesToSend == 0U)
    {
        DSPI_Set_RSER_TFFF_RE(base, 0U);
        if (state->framesToReceive == 0U)
        {
            /* Disable DSPI module */
            DSPI_UpdateTransferDoneState(instance);

            /* Disable FIFOs interrupt errors */
            DSPI_Set_RSER_RFOF_RE(base, 0U);
            DSPI_Set_RSER_TFUF_RE(base, 0U);
        }
    }

    DSPI_Clear_SR_TFFF(base);
}

#if (!(defined FEATURE_DSPI_HAS_EXTENDED_MODE))
static void DSPI_DRV_MasterCompleteTxDMATransfer(void * parameter,
                                               edma_chn_status_t status)
{
    uint32_t     instance = (uint32_t)parameter;
    dspi_state_t * state  = DSPI_state[instance];
    (void)status;

    /* If current transfer is continuous clear CONT bit */
    if (state->continuousPCS)
    {
        state->configWord &= ~DSPI_PUSHR_CONT_MASK;
    }

    /* Triggers a sw request for txAdditionalDMA channel */
    EDMA_DRV_TriggerSwRequest(state->txAdditionalDMAChannel);
}
#endif

static void DSPI_DRV_MasterCompleteDMATransfer(void * parameter,
                                               edma_chn_status_t status)
{
    uint32_t     instance = (uint32_t)parameter;
    DSPI_Type    * base   = DSPI_Instances[instance];
    (void)status;

    DSPI_UpdateTransferDoneState((dspi_instance_t)instance);

    /* Disable FIFOs interrupt errors */
    DSPI_Set_RSER_TFFF_RE(base, 0U);
    DSPI_Set_RSER_RFDF_RE(base, 0U);
    DSPI_Set_RSER_RFOF_RE(base, 0U);
    DSPI_Set_RSER_TFUF_RE(base, 0U);
    DSPI_Clear_SR_RFOF(base);
    DSPI_Clear_SR_TFUF(base);
    DSPI_Clear_SR_RFDF(base);
    DSPI_Clear_SR_TFFF(base);
}

static void DSPI_DRV_TrannsferFail(dspi_instance_t instance)
{
    dspi_state_t * state = DSPI_state[instance];

    (void)DSPI_AbortTransfer(instance);
    state->status = DSPI_TRANSFER_FAIL;
}

/*Define interrupt handlers*/
/*Depending on CPU the interrupt controller has one or more interrupts for the same peripheral device. */
#ifdef FEATURES_DSPI_HAS_INDEPENDENT_INTERRUPTS
    void DSPI0_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)0U);
    }

    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 2U)
    void DSPI1_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)1U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 3U)
    void DSPI2_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)2U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 4U)
    void DSPI3_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)3U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 5U)
    void DSPI4_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)4U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 6U)
    void DSPI5_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)5U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 7U)
    void DSPI6_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)6U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 8U)
    void DSPI7_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)7U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 9U)
    void DSPI8_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)8U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 10U)
    void DSPI9_Send_IRQHandler(void)
    {
        DSPI_SEND_isr((dspi_instance_t)9U);
    }
    #endif
    
    void DSPI0_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)0U);
    }
    
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 2U)
    void DSPI1_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)1U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 3U)
    void DSPI2_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)2U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 4U)
    void DSPI3_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)3U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 5U)
    void DSPI4_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)4U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 6U)
    void DSPI5_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)5U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 7U)
    void DSPI6_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)6U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 8U)
    void DSPI7_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)7U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 9U)
    void DSPI8_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)8U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 10U)
    void DSPI9_Receive_IRQHandler(void)
    {
        DSPI_RECEIVE_isr((dspi_instance_t)9U);
    }
    #endif
    
    
    void DSPI0_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)0U);
    }
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 2U)
    void DSPI1_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)1U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 3U)
    void DSPI2_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)2U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 4U)
    void DSPI3_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)3U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 5U)
    void DSPI4_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)4U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 6U)
    void DSPI5_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)5U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 7U)
    void DSPI6_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)6U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 8U)
    void DSPI7_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)7U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 9U)
    void DSPI8_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)8U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 10U)
    void DSPI9_FIFO_Error_IRQHandler(void)
    {
        DSPI_DRV_TrannsferFail((dspi_instance_t)9U);
    }
    #endif
#endif

#ifdef FEATIRES_DSPI_HAS_SHARED_INTERRUPTS
    void SPI_Handler(dspi_instance_t instance);
    
    void SPI_Handler(dspi_instance_t instance)
    {
        const DSPI_Type* base = DSPI_Instances[instance];
        
        /* Check if receive buffer contains data. */
        if (DSPI_Get_SR_RFDF(base) == 1U)
        {
            if (DSPI_Get_RSER_RFDF_RE(base))
            {
                DSPI_RECEIVE_isr(instance);
            }
        }
        
        /* Check if transmit buffer can be filled. */
        if (DSPI_Get_SR_TFFF(base) == 1U)
        {
            if (DSPI_Get_RSER_TFFF_RE(base))
            {
                DSPI_SEND_isr(instance);
            }
        }
        
        /* Check error flags. */
        if (DSPI_Get_SR_TFUF(base) == 1U)
        {
            if (DSPI_Get_RSER_TFUF_RE(base))
            {
                DSPI_DRV_TrannsferFail(instance);
            }
        }
        else
        {
            if (DSPI_Get_SR_RFOF(base) == 1U)
            {
                if (DSPI_Get_RSER_RFOF_RE(base))
                {
                    DSPI_DRV_TrannsferFail(instance);
                }
            }
        }
    }
    
    void SPI0_IRQHandler(void);
    
    void SPI0_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)0U);
    }
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 2U)
    void SPI1_IRQHandler(void);    

    void SPI1_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)1U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 3U)
    void SPI2_IRQHandler(void);  
    
    void SPI2_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)2U);
    }
    #endif
    
    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 4U)
    void SPI3_IRQHandler(void);     
    
    void SPI3_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)3U);
    }
    #endif

    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 5U)
    void SPI4_IRQHandler(void);     
    
    void SPI4_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)4U);
    }
    #endif

    #if ((SPI_INSTANCE_COUNT + DSPI_INSTANCE_COUNT) >= 6U)
    void SPI5_IRQHandler(void);     

    void SPI5_IRQHandler(void)
    {
        SPI_Handler((dspi_instance_t)5U);
    }
    #endif

#endif
