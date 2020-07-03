/*
 * Copyright 2019 NXP
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

#include "lfast_hw_access.h"
#include "sipi_irq.h"
#include "interrupt_manager.h"
#include "edma_driver.h"
#include "clock.h"

/*!
 * @file zipwire_driver.c
 *
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
 * Violates MISRA 2012 Mandatory Rule 9.1, Symbol not initialized.
 * Reported structure is declared and initialized in the same function where it is used.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (wider essential type for the destination).
 * The value needs to be assigned to a 32-bit register.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object type
 * and a pointer to a different object type.
 * This is needed for callbacks from other modules, which cannot know the actual argument type (dma)
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.5, Conversion from pointer to void to pointer to other type
 * This is needed for callbacks from other modules, which cannot know the actual argument type (dma).
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const/volatile from a pointer or reference.
 * The driver uses a common function for read and write, so the same pointer is used either for the constant
 * tx buffer, or for the non-constant rx buffer. The information about transfer direction is kept in another variable
 * and the driver will not write in the tx buffer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 *
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
SIPI_Type * s_sipi_instances[SIPI_INSTANCE_COUNT] = SIPI_BASE_PTRS;
static LFAST_Type * s_lfast_instances[LFAST_INSTANCE_COUNT] = LFAST_BASE_PTRS;
static zipwire_state_t * s_zipwire_state[SIPI_INSTANCE_COUNT];
static zipwire_chn_state_t * s_zipwire_chn_state[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT];

/*******************************************************************************
 * Private functions
 ******************************************************************************/
static status_t ZIPWIRE_LaunchRWCommand(uint8_t instance,
                                        uint8_t channel,
                                        zipwire_transfer_descriptor_t *data,
                                        uint32_t dataArrayLength,
                                        bool write,
                                        bool isBlocking);
static status_t ZIPWIRE_LaunchRWCommandWithDma(uint8_t instance,
                                               uint8_t channel,
                                               uint32_t *dataArray,
                                               const uint32_t *addressArray,
                                               uint32_t arrayLength,
                                               bool write,
                                               bool isBlocking);
static void ZIPWIRE_WriteData(uint8_t instance, uint8_t channel, const zipwire_transfer_descriptor_t *data);
static void ZIPWIRE_StopTransfer(uint8_t instance, uint8_t channel);
static void ZIPWIRE_DmaCallback(void *state,  edma_chn_status_t status);

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_GetDefaultConfig
 * Description   : This function gets the default configuration structure.
 *
 * Implements    : ZIPWIRE_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void ZIPWIRE_DRV_GetDefaultConfig(zipwire_config_t *zipwireConfig, lfast_config_t *lfastConfig)
{
    /* LFAST default config */
    lfastConfig->role = LFAST_MASTER;
    lfastConfig->speedMode = LFAST_HIGH_SPEED;
    lfastConfig->syncTimeout = 1000U;
    lfastConfig->syncAttempts = 0U;
    lfastConfig->preDiv = LFAST_PLL_REF_DIV_1;
    lfastConfig->fbDiv = 15U;
    lfastConfig->lsClkDiv = LFAST_LOW_SPEED_CLK_DIV_2;
    /* Zipwire default config */
    zipwireConfig->lfastConfig = lfastConfig;
    zipwireConfig->addrOffset = ZIPWIRE_ADDR_NO_CHANGE;
    zipwireConfig->timeoutClkDiv = ZIPWIRE_DIV_64;
    zipwireConfig->maxCountReachedInt = false;
    zipwireConfig->callback = NULL;
    zipwireConfig->callbackParam = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_Init
 * Description   : This function initializes the zipwire driver.
 *
 * Implements    : ZIPWIRE_DRV_Init_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_Init(uint8_t instance,
                          zipwire_state_t *state,
                          const zipwire_config_t *config)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] == NULL);
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(config->lfastConfig != NULL);

    LFAST_Type *lfastBase = s_lfast_instances[instance];
    SIPI_Type *sipiBase = s_sipi_instances[instance];
    uint8_t i;
    status_t stat;

    /* Check that the peripherals are clock gated on */
#if defined(CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    clock_names_t sipiClkName;
    uint32_t sipiClkFreq;
    clock_names_t sipiClkNames[SIPI_INSTANCE_COUNT] = SIPI_CLOCK_NAMES;

    sipiClkName = sipiClkNames[instance];

    (void)CLOCK_DRV_GetFreq(sipiClkName, &sipiClkFreq);
    DEV_ASSERT(sipiClkFreq != 0U);
#endif

    /* Perform a soft reset of the SIPI state machine */
    stat = SIPI_SoftReset(sipiBase);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    /* Enable SIPI module */
    SIPI_Enable(sipiBase, true);
    /* Enable target functionality for the SIPI module */
    SIPI_TargetEnable(sipiBase, true);
    /* Move SIPI to INIT mode */
    SIPI_EnterInitMode(sipiBase);
    /* Configure address offset and clock prescaler with user options */
    SIPI_SetAddrOffset(sipiBase, (uint8_t)config->addrOffset);
    SIPI_SetTimeoutClockPrescaler(sipiBase, ((uint16_t)config->timeoutClkDiv));
    /* Move back to NORMAL mode */
    SIPI_ExitInitMode(sipiBase);
    /* Enable CRC error interrupt */
    SIPI_SetCrcErrInt(sipiBase, true);
    /* Configure maximum address interrupt with user option */
    SIPI_SetMaxCountReachedInt(sipiBase, config->maxCountReachedInt);

    /* Save the reference to the state structure */
    s_zipwire_state[instance] = state;
    /* Configure user callbacks */
    s_zipwire_state[instance]->callback = config->callback;
    s_zipwire_state[instance]->callbackParam = config->callbackParam;

    /* Initialize appropriate LFAST interface */
    if (config->lfastConfig->role == LFAST_MASTER)
    {
        stat = LFAST_MasterInit(lfastBase,
                                config->lfastConfig->preDiv,
                                config->lfastConfig->fbDiv,
                                config->lfastConfig->speedMode,
                                config->lfastConfig->lsClkDiv,
                                config->lfastConfig->syncTimeout,
                                config->lfastConfig->syncAttempts);
    }
    else
    {
        stat = LFAST_SlaveInit(lfastBase,
                               config->lfastConfig->preDiv,
                               config->lfastConfig->fbDiv,
                               config->lfastConfig->speedMode,
                               config->lfastConfig->lsClkDiv,
                               config->lfastConfig->syncTimeout);
    }
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Enable SIPI IRQs at interrupt controller level */
    for (i = 0U; i < SIPI_IRQ_COUNT; i++)
    {
        INT_SYS_EnableIRQ(sipi_irqs[i]);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_Deinit
 * Description   : This function de-initializes the zipwire driver.
 *
 * Implements    : ZIPWIRE_DRV_Deinit_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_Deinit(uint8_t instance)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);

    LFAST_Type *lfastBase = s_lfast_instances[instance];
    SIPI_Type *sipiBase = s_sipi_instances[instance];
    uint8_t i;
    status_t stat;

    /* Perform a soft reset of the SIPI state machine */
    stat = SIPI_SoftReset(sipiBase);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Disable SIPI global interrupts */
    SIPI_SetCrcErrInt(sipiBase, false);
    SIPI_SetMaxCountReachedInt(sipiBase, false);

    /* Reset user callbacks */
    s_zipwire_state[instance]->callback = NULL;
    s_zipwire_state[instance]->callbackParam = NULL;

    /* Disable SIPI interrupts at interrupt controller level */
    for (i = 0U; i < SIPI_IRQ_COUNT; i++)
    {
        INT_SYS_DisableIRQ(sipi_irqs[i]);
    }

    /* Deinitialize all channels */
    for (i = 0U; i < SIPI_CHANNEL_COUNT; i++)
    {
        (void)ZIPWIRE_DRV_DeinitChannel(instance, i);
    }

    /* Disable SIPI module */
    SIPI_Enable(sipiBase, false);
    /* LFAST interface */
    LFAST_Enable(lfastBase, false);

    /* Clear the state structure reference */
    s_zipwire_state[instance] = NULL;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_InitChannel
 * Description   : This function initializes a ZIPWIRE channel
 *
 * Implements    : ZIPWIRE_DRV_InitChannel_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_InitChannel(uint8_t instance, const zipwire_chn_config_t *config, zipwire_chn_state_t *state)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(config->chNo < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][config->chNo] == NULL);

    SIPI_Type *sipiBase = s_sipi_instances[instance];
    status_t stat;

    /* Create the synchronization semaphore */
    stat = OSIF_SemaCreate(&state->syncSem, 0U);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Save instance & channel info */
    state->instance = instance;
    state->channel = config->chNo;

    /* Save references user callbacks */
    state->callback = config->callback;
    state->callbackParam = config->callbackParam;

    /* Initialize channel status */
    state->chStatus = STATUS_SUCCESS;
    /* Initialize transfer buffer reference to NULL */
    state->transferBuffer = NULL;
    /* Initialize id request and target ID to zero */
    state->targetId = NULL;
    state->idRequest = false;
    state->disableNotification = false;

    /* Initialize DMA */
    state->dma = config->dma;
    state->dmaDataChn = config->dmaDataChn;
    state->dmaAddrChn = config->dmaAddrChn;
    state->dmaWriteTransfer = false;

    /* Save the reference of the channel state structure */
    s_zipwire_chn_state[instance][config->chNo] = state;

    /* Enable channel interrupts */
    SIPI_SetChannelInterrupt(instance, config->chNo, SIPI_ACK_ERR_IRQ, config->ackErrIrq);
    SIPI_SetChannelInterrupt(instance, config->chNo, SIPI_TIMEOUT_IRQ, config->timeoutErrIrq);
    SIPI_SetChannelInterrupt(instance, config->chNo, SIPI_TRANS_ID_ERR_IRQ, config->transIdErrIrq);
    SIPI_SetChannelInterrupt(instance, config->chNo, SIPI_TRIGGER_IRQ, true);
    /* Move SIPI to INIT mode */
    SIPI_EnterInitMode(sipiBase);
    /* Configure channel timeout */
    SIPI_SetChannelTimeout(instance, config->chNo, config->timeout);
    /* Enable the channel */
    SIPI_EnableChannel(instance, config->chNo, true);
    /* Move back to NORMAL mode*/
    SIPI_ExitInitMode(sipiBase);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_DeinitChannel
 * Description   : This function de-initializes a ZIPWIRE channel
 *
 * Implements    : ZIPWIRE_DRV_DeinitChannel_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_DeinitChannel(uint8_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);

    SIPI_Type *sipiBase = s_sipi_instances[instance];
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    status_t stat;

    /* Check if the channel is initialized */
    if (s_zipwire_chn_state[instance][channel] == NULL)
    {
        return STATUS_SUCCESS;
    }

    /* Destroy the synchronization semaphore */
    stat = OSIF_SemaDestroy(&chState->syncSem);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Reset user callbacks */
    chState->callback = NULL;
    chState->callbackParam = NULL;

    /* Disable channel interrupts */
    SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_ERR_IRQ, false);
    SIPI_SetChannelInterrupt(instance, channel, SIPI_TIMEOUT_IRQ, false);
    SIPI_SetChannelInterrupt(instance, channel, SIPI_TRANS_ID_ERR_IRQ, false);
    SIPI_SetChannelInterrupt(instance, channel, SIPI_TRIGGER_IRQ, false);

    /* Clear the state reference */
    s_zipwire_chn_state[instance][channel] = NULL;

    /* Move SIPI to INIT mode */
    SIPI_EnterInitMode(sipiBase);
    /* Disable the channel */
    SIPI_EnableChannel(instance, channel, false);
    /* Move back to NORMAL mode*/
    SIPI_ExitInitMode(sipiBase);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_Read
 * Description   : This function performs multiple read transfers, asynchronously.
 *
 * Implements    : ZIPWIRE_DRV_Read_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_Read(uint8_t instance,
                          uint8_t channel,
                          zipwire_transfer_descriptor_t *dataArray,
                          uint32_t dataArrayLength)
{
    status_t stat;

    /* Launch the read command and return */
    stat = ZIPWIRE_LaunchRWCommand(instance, channel, dataArray, dataArrayLength, false, false);

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_ReadBlocking
 * Description   : This function performs multiple read transfers, synchronously.
 *
 * Implements    : ZIPWIRE_DRV_ReadBlocking_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_ReadBlocking(uint8_t instance,
                                  uint8_t channel,
                                  zipwire_transfer_descriptor_t *dataArray,
                                  uint32_t dataArrayLength,
                                  uint32_t timeout)
{
    DEV_ASSERT(timeout > 0U);
    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Launch the read command */
    stat = ZIPWIRE_LaunchRWCommand(instance, channel, dataArray, dataArrayLength, false, true);

    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    else
    {
        /* Wait for all the read requests to be serviced */
        stat = OSIF_SemaWait(&chState->syncSem, timeout);

        if (stat == STATUS_TIMEOUT)
        {
            chState->isBlocking = false;
            /* In case of timeout, disable channel logic, so no answers are considered beyond this point */
            SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, false);
            SIPI_SetChannelReadRequest(instance, channel, false);
            /* Update channel status */
            chState->chStatus = STATUS_TIMEOUT;
        }
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_ReadDma
 * Description   : This function performs multiple read transfers using DMA.
 *                 The function returns right after launching the first command.
 *
 * Implements    : ZIPWIRE_DRV_ReadDma_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_ReadDma(uint8_t instance,
                             uint8_t channel,
                             uint32_t *dataArray,
                             const uint32_t *addressArray,
                             uint32_t arrayLength)
{
    status_t stat;

    /* Launch the read command and return */
    stat = ZIPWIRE_LaunchRWCommandWithDma(instance, channel, dataArray, addressArray,
                                          arrayLength, false, false);

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_ReadDmaBlocking
 * Description   : This function performs multiple read transfers using DMA and
 *                 waits for either transfer completion or timeout event.
 *
 * Implements    : ZIPWIRE_DRV_ReadDmaBlocking_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_ReadDmaBlocking(uint8_t instance,
                                     uint8_t channel,
                                     uint32_t *dataArray,
                                     const uint32_t *addressArray,
                                     uint32_t arrayLength,
                                     uint32_t timeout)
{
    DEV_ASSERT(timeout > 0U);
    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    SIPI_Type *sipiBase = s_sipi_instances[instance];

    /* Launch the read command */
    stat = ZIPWIRE_LaunchRWCommandWithDma(instance,channel, dataArray, addressArray,
                                          arrayLength, false, true);

    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    else
    {
        /* Wait for all the read requests to be serviced */
        stat = OSIF_SemaWait(&chState->syncSem, timeout);

        if (stat == STATUS_TIMEOUT)
        {
            chState->isBlocking = false;
            /* In case of timeout, disable channel logic and DMA requests */
            (void)EDMA_DRV_StopChannel(chState->dmaAddrChn);
            (void)EDMA_DRV_StopChannel(chState->dmaDataChn);
            /* Move SIPI to INIT mode */
            SIPI_EnterInitMode(sipiBase);
            /* Disable channel DMA */
            SIPI_EnableChannelDma(instance, channel, false);
            /* Move SIPI back to normal mode */
            SIPI_ExitInitMode(sipiBase);
            /* Disable channel read requests */
            SIPI_SetChannelReadRequest(instance, channel, false);
            /* Update channel status */
            chState->chStatus = STATUS_TIMEOUT;
        }
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_Write
 * Description   : This function performs multiple write transfers, asynchronously.
 *
 * Implements    : ZIPWIRE_DRV_Write_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_Write(uint8_t instance,
                           uint8_t channel,
                           const zipwire_transfer_descriptor_t *dataArray,
                           uint32_t dataArrayLength)
{
    status_t stat;

    /* Launch the write command and return */
    stat = ZIPWIRE_LaunchRWCommand(instance, channel, (zipwire_transfer_descriptor_t *)dataArray,
                                   dataArrayLength, true, false);

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_WriteBlocking
 * Description   : This function performs multiple write transfers, synchronously.
 *
 * Implements    : ZIPWIRE_DRV_WriteBlocking_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_WriteBlocking(uint8_t instance,
                                   uint8_t channel,
                                   const zipwire_transfer_descriptor_t *dataArray,
                                   uint32_t dataArrayLength,
                                   uint32_t timeout)
{
    DEV_ASSERT(timeout > 0U);
    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Launch the write command */
    stat = ZIPWIRE_LaunchRWCommand(instance, channel, (zipwire_transfer_descriptor_t *)dataArray,
                                   dataArrayLength, true, true);

    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    else
    {
        /* Wait for all the write requests to be serviced */
        stat = OSIF_SemaWait(&chState->syncSem, timeout);

        if (stat == STATUS_TIMEOUT)
        {
            chState->isBlocking = false;
            /* In case of timeout, disable channel logic, so no ACK is considered beyond this point */
            SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, false);
            SIPI_SetChannelWriteRequest(instance, channel, false);
            /* Update channel status */
            chState->chStatus = STATUS_TIMEOUT;
        }
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_WriteDma
 * Description   : This function performs multiple write transfers using DMA.
 *                 The function returns right after launching the first command.
 *
 * Implements    : ZIPWIRE_DRV_WriteDma_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_WriteDma(uint8_t instance,
                              uint8_t channel,
                              const uint32_t *dataArray,
                              const uint32_t *addressArray,
                              uint32_t arrayLength)
{
    status_t stat;

    /* Launch the write command and return */
    stat = ZIPWIRE_LaunchRWCommandWithDma(instance, channel, (uint32_t *)dataArray,
                                          addressArray, arrayLength, true, false);

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_WriteDmaBlocking
 * Description   : This function performs multiple write transfers using DMA and
 *                 waits for either transfer completion or timeout event.
 *
 * Implements    : ZIPWIRE_DRV_WriteDmaBlocking_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_WriteDmaBlocking(uint8_t instance,
                                      uint8_t channel,
                                      const uint32_t *dataArray,
                                      const uint32_t *addressArray,
                                      uint32_t arrayLength,
                                      uint32_t timeout)
{
    DEV_ASSERT(timeout > 0U);
    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    SIPI_Type *sipiBase = s_sipi_instances[instance];

    /* Launch the write command */
    stat = ZIPWIRE_LaunchRWCommandWithDma(instance, channel, (uint32_t *)dataArray,
                                          addressArray, arrayLength, true, true);

    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    else
    {
        /* Wait for all the write requests to be serviced */
        stat = OSIF_SemaWait(&chState->syncSem, timeout);

        if (stat == STATUS_TIMEOUT)
        {
            chState->isBlocking = false;
            /* In case of timeout, disable channel logic and DMA requests */
            (void)EDMA_DRV_StopChannel(chState->dmaAddrChn);
            (void)EDMA_DRV_StopChannel(chState->dmaDataChn);
            /* Move SIPI to INIT mode */
            SIPI_EnterInitMode(sipiBase);
            /* Disable channel DMA */
            SIPI_EnableChannelDma(instance, channel, false);
            /* Move SIPI back to normal mode */
            SIPI_ExitInitMode(sipiBase);
            /* Disable channel write requests */
            SIPI_SetChannelWriteRequest(instance, channel, false);
            /* Update channel status */
            chState->chStatus = STATUS_TIMEOUT;
        }
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_StreamWrite
 * Description   : This function performs a streaming write operation synchronously.
 *
 * Implements    : ZIPWIRE_DRV_StreamWrite_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_StreamWrite(uint8_t instance,
                                 uint8_t channel,
                                 uint32_t dataAddress,
                                 uint32_t targetAcrRegAddress,
                                 const uint32_t *data,
                                 uint32_t timeout)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel == SIPI_STREAMING_CH);
    DEV_ASSERT(data != NULL);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);
    DEV_ASSERT(timeout > 0U);

    uint8_t i;
    zipwire_transfer_descriptor_t targetConfig;
    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Create a transfer descriptor for configuring target address in the ACR register */
    targetConfig.size = ZIPWIRE_32_BITS;
    targetConfig.address = targetAcrRegAddress;
    targetConfig.data = dataAddress;
    /* Temporary disable notifications */
    chState->disableNotification = true;
    /* Perform a single write command to set the address count register at the target node */
    stat = ZIPWIRE_DRV_WriteBlocking(instance, channel, &targetConfig, 1U, timeout);
    /* Enable back success notification */
    chState->disableNotification = false;
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Channel setup */
    chState->isBlocking = true;
    chState->chStatus = STATUS_BUSY;
    chState->remainingTransfers = 0U;

    /* Fill in the data for the streaming transfer */
    for (i = 0U; i < SIPI_STREAM_TRANSFER_LEN; i++)
    {
        SIPI_STREAM_DATA(i) = *data;
        data++;
    }

    /* Enable write ACK interrupt */
    SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, true);
    /* Enable write and streaming requests */
    SIPI_SetChannelWriteRequest(instance, channel, true);
    SIPI_SetChannelStreamingWriteRequest(instance, channel, true);
    /* Write address register with dummy value to trigger the transfer */
    SIPI_SetChannelAddr(instance, channel, 0U);

    /* Wait for the write request to be serviced */
    stat = OSIF_SemaWait(&chState->syncSem, timeout);

    if (stat == STATUS_TIMEOUT)
    {
        chState->isBlocking = false;
        /* In case of timeout, disable channel logic, so no ACK is considered beyond this point */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, false);
        SIPI_SetChannelStreamingWriteRequest(instance, channel, false);
        SIPI_SetChannelWriteRequest(instance, channel, false);
        /* Update channel status */
        chState->chStatus = STATUS_TIMEOUT;
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_RequestId
 * Description   : This function performs an ID request transfer.
 *
 * Implements    : ZIPWIRE_DRV_RequestId_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_RequestId(uint8_t instance, uint8_t channel, uint32_t *id, uint32_t timeout)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(id != NULL);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);
    DEV_ASSERT(timeout > 0U);

    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Check if channel is busy with another transfer */
    if (chState->chStatus == STATUS_BUSY)
    {
        return STATUS_BUSY;
    }

    /* Transfer setup */
    chState->idRequest = true;
    chState->remainingTransfers = 0U;
    chState->targetId = id;
    /* Channel becomes busy now */
    chState->chStatus = STATUS_BUSY;

    /* Enable ID request */
    SIPI_SetChannelIdTransferRequest(instance, channel, true);
    /* Enable read answer interrupt */
    SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, true);
    /* Set the target address - dummy value to trigger the transfer */
    SIPI_SetChannelAddr(instance, channel, 0U);

    /* Wait for the ID request to be serviced */
    stat = OSIF_SemaWait(&chState->syncSem, timeout);

    if (stat == STATUS_TIMEOUT)
    {
        chState->isBlocking = false;
        /* In case of timeout, disable channel logic, so no ACK is considered beyond this point */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, false);
        SIPI_SetChannelIdTransferRequest(instance, channel, false);
        /* Update channel status */
        chState->chStatus = STATUS_TIMEOUT;
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_Trigger
 * Description   : This function sends a trigger command to the target.
 *
 * Implements    : ZIPWIRE_DRV_Trigger_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_Trigger(uint8_t instance, uint8_t channel, uint32_t timeout)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);
    DEV_ASSERT(timeout > 0U);

    status_t stat;
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Check if channel is busy with another transfer */
    if (chState->chStatus == STATUS_BUSY)
    {
        return STATUS_BUSY;
    }

    /* Transfer setup */
    chState->isBlocking = true;
    chState->remainingTransfers = 0U;
    chState->idRequest = false;
    /* Channel becomes busy now */
    chState->chStatus = STATUS_BUSY;

    /* Enable read answer interrupt */
    SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, true);
    /* Send ID request */
    SIPI_SetChannelTriggerCommand(instance, channel, true);

    /* Wait for the ID request to be serviced */
    stat = OSIF_SemaWait(&chState->syncSem, timeout);

    if (stat == STATUS_TIMEOUT)
    {
        chState->isBlocking = false;
        /* In case of timeout, disable channel logic, so no ACK is considered beyond this point */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, false);
        SIPI_SetChannelTriggerCommand(instance, channel, false);
        /* Update channel status */
        chState->chStatus = STATUS_TIMEOUT;
    }

    return chState->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DRV_GetChannelStatus
 * Description   : This function returns the current channel status.
 *
 * Implements    : ZIPWIRE_DRV_GetChannelStatus_Activity
 *END**************************************************************************/
status_t ZIPWIRE_DRV_GetChannelStatus(uint8_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);

    return s_zipwire_chn_state[instance][channel]->chStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_LaunchRWCommand
 * Description   : This internal function launches a read/write command at the
 *                 initiator.
 *
 *END**************************************************************************/
static status_t ZIPWIRE_LaunchRWCommand(uint8_t instance,
                                        uint8_t channel,
                                        zipwire_transfer_descriptor_t *data,
                                        uint32_t dataArrayLength,
                                        bool write,
                                        bool isBlocking)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);
    DEV_ASSERT(data != NULL);
    DEV_ASSERT(dataArrayLength > 0U);

    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* Check if channel is busy with another transfer */
    if (chState->chStatus == STATUS_BUSY)
    {
        return STATUS_BUSY;
    }

    /* Transfer setup */
    chState->isBlocking = isBlocking;
    chState->transferBuffer = data;
    chState->remainingTransfers = dataArrayLength - 1U;
    chState->idRequest = false;
    /* Channel becomes busy now */
    chState->chStatus = STATUS_BUSY;

    if (write)
    {
        /* Write transfer */

        /* Enable write request */
        SIPI_SetChannelWriteRequest(instance, channel, true);
        /* Enable write ACK interrupt */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, true);
        /* Write the data and trigger the transfer */
        ZIPWIRE_WriteData(instance, channel, data);
    }
    else
    {
        /* Read transfer */

        /* Configure the word length */
        SIPI_SetChannelWordLength(instance, channel, data->size);
        /* Enable read request */
        SIPI_SetChannelReadRequest(instance, channel, true);
        /* Enable read answer interrupt */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, true);
        /* Set the target address - this will trigger the transfer */
        SIPI_SetChannelAddr(instance, channel, data->address);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_LaunchRWCommandWithDma
 * Description   : This internal function launches a read/write command at the
 *                 initiator using DMA requests.
 *
 *END**************************************************************************/
static status_t ZIPWIRE_LaunchRWCommandWithDma(uint8_t instance,
                                               uint8_t channel,
                                               uint32_t *dataArray,
                                               const uint32_t *addressArray,
                                               uint32_t arrayLength,
                                               bool write,
                                               bool isBlocking)
{
    DEV_ASSERT(instance < SIPI_INSTANCE_COUNT);
    DEV_ASSERT(channel < SIPI_CHANNEL_COUNT);
    DEV_ASSERT(s_zipwire_state[instance] != NULL);
    DEV_ASSERT(s_zipwire_chn_state[instance][channel] != NULL);
    DEV_ASSERT(addressArray != NULL);
    DEV_ASSERT(dataArray != NULL);
    DEV_ASSERT(arrayLength > 0U);

    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    SIPI_Type *sipiBase = s_sipi_instances[instance];
    DEV_ASSERT(chState->dma);

    /* Check if channel is busy with another transfer */
    if (chState->chStatus == STATUS_BUSY)
    {
        return STATUS_BUSY;
    }

    /* DMA loop transfer configuration for address channel */
    edma_loop_transfer_config_t addrLoopConfig = {
        .dstOffsetEnable = false,
        .srcOffsetEnable = false,
        .minorLoopOffset = 0,
        .minorLoopChnLinkEnable = false,
        .minorLoopChnLinkNumber = 0U,
        .majorLoopChnLinkEnable = false,
        .majorLoopChnLinkNumber = 0U,
        .majorLoopIterationCount = (write ? arrayLength : (arrayLength - 1U))
    };
    /* DMA loop transfer configuration for data channel */
    edma_loop_transfer_config_t dataLoopConfig = {
        .dstOffsetEnable = false,
        .srcOffsetEnable = false,
        .minorLoopOffset = 0,
        .minorLoopChnLinkEnable = true,
        .minorLoopChnLinkNumber = chState->dmaAddrChn,
        .majorLoopChnLinkEnable = write,
        .majorLoopChnLinkNumber = chState->dmaAddrChn,
        .majorLoopIterationCount = arrayLength
    };
    /* DMA transfer descriptor for address channel */
    edma_transfer_config_t addrConfig = {
        .srcAddr = ((uint32_t)addressArray),
        .destAddr = (uint32_t)CAR[instance][channel],
        .srcTransferSize = EDMA_TRANSFER_SIZE_4B,
        .destTransferSize = EDMA_TRANSFER_SIZE_4B,
        .srcOffset = 4,
        .destOffset = 0,
        .srcLastAddrAdjust = 0,
        .destLastAddrAdjust = 0,
        .srcModulo = EDMA_MODULO_OFF,
        .destModulo = EDMA_MODULO_OFF,
        .minorByteTransferCount = 4U,
        .scatterGatherEnable = false,
        .scatterGatherNextDescAddr = 0U,
        .interruptEnable = write,
        .loopTransferConfig = &addrLoopConfig
    };
    /* DMA transfer descriptor for data channel */
    edma_transfer_config_t dataConfig = {
        .srcAddr = (write ? ((uint32_t)dataArray) : ((uint32_t)CDR[instance][channel])),
        .destAddr = (write ? ((uint32_t)CDR[instance][channel]) : ((uint32_t)dataArray)),
        .srcTransferSize = EDMA_TRANSFER_SIZE_4B,
        .destTransferSize = EDMA_TRANSFER_SIZE_4B,
        .srcOffset = (write ? (int16_t)4 : (int16_t)0),
        .destOffset = (write ? (int16_t)0 : (int16_t)4),
        .srcLastAddrAdjust = 0,
        .destLastAddrAdjust = 0,
        .srcModulo = EDMA_MODULO_OFF,
        .destModulo = EDMA_MODULO_OFF,
        .minorByteTransferCount = 4U,
        .scatterGatherEnable = false,
        .scatterGatherNextDescAddr = 0U,
        .interruptEnable = (!write),
        .loopTransferConfig = &dataLoopConfig
    };

    /* Transfer setup */
    chState->isBlocking = isBlocking;
    chState->remainingTransfers = 0U;
    chState->idRequest = false;
    chState->dmaWriteTransfer = write;
    /* Channel becomes busy now */
    chState->chStatus = STATUS_BUSY;

    /* Enable DMA functionality for the channel */
    SIPI_EnterInitMode(sipiBase);
    SIPI_EnableChannelDma(instance, channel, true);
    SIPI_ExitInitMode(sipiBase);

    /* Only 32 bits transfers supported in DMA mode */
    SIPI_SetChannelWordLength(instance, channel, ZIPWIRE_32_BITS);

    if (write)
    {
        /* Enable write request */
        SIPI_SetChannelWriteRequest(instance, channel, true);
        /* Install DMA transfer complete callback to finish the zipwire transfer */
        (void)EDMA_DRV_InstallCallback(chState->dmaAddrChn, ZIPWIRE_DmaCallback, chState);
    }
    else
    {
        /* Enable write request */
        SIPI_SetChannelReadRequest(instance, channel, true);
        /* First address is written by sw, DMA starts transferring from the next */
        addrConfig.srcAddr += 4U;
        /* Install DMA transfer complete callback to finish the zipwire transfer */
        (void)EDMA_DRV_InstallCallback(chState->dmaDataChn, ZIPWIRE_DmaCallback, chState);
    }

    /* Set the DMA configurations in the two TCDs and start the channels */
    if (addrConfig.loopTransferConfig->majorLoopIterationCount > 0U)
    {
        EDMA_DRV_PushConfigToReg(chState->dmaAddrChn, &addrConfig);
        EDMA_DRV_DisableRequestsOnTransferComplete(chState->dmaAddrChn, true);
        (void)EDMA_DRV_StartChannel(chState->dmaAddrChn);
    }
    EDMA_DRV_PushConfigToReg(chState->dmaDataChn, &dataConfig);
    EDMA_DRV_DisableRequestsOnTransferComplete(chState->dmaDataChn, true);
    (void)EDMA_DRV_StartChannel(chState->dmaDataChn);

    /* Write the first address to trigger the first read transfer */
    if (!write)
    {
        SIPI_SetChannelAddr(instance, channel, *addressArray);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_DmaCallback
 * Description   : This internal function represents the DMA callback used
 *                 internally by the driver to finish DMA transfers.
 *
 *END**************************************************************************/
static void ZIPWIRE_DmaCallback(void *state,  edma_chn_status_t status)
{
    zipwire_chn_state_t *chState = (zipwire_chn_state_t *)state;
    zipwire_event_t event;

    /* Disable read/write requests */
    if (chState->dmaWriteTransfer)
    {
        SIPI_SetChannelWriteRequest(chState->instance, chState->channel, false);
        SIPI_ClearChannelFlag(chState->instance, chState->channel, SIPI_ACK_FLAG);
        event = ZIPWIRE_EVENT_WRITE_COMPLETE;
    }
    else
    {
        SIPI_SetChannelReadRequest(chState->instance, chState->channel, false);
        SIPI_ClearChannelFlag(chState->instance, chState->channel, SIPI_READ_ANSWER_FLAG);
        event = ZIPWIRE_EVENT_READ_COMPLETE;
    }

    /* Update internal status */
    if (status == EDMA_CHN_NORMAL)
    {
        chState->chStatus = STATUS_SUCCESS;
    }
    else
    {
        chState->chStatus = STATUS_ERROR;
    }

    /* If the transfer was blocking, notify the waiting thread */
    if (chState->isBlocking)
    {
        (void)OSIF_SemaPost(&chState->syncSem);
        chState->isBlocking = false;
    }
    /* Call the user callback, if any */
    if (chState->callback != NULL)
    {
        if (status == EDMA_CHN_NORMAL)
        {
            chState->callback(chState->instance, chState->channel, event, chState->callbackParam);
        }
        else
        {
            chState->callback(chState->instance, chState->channel, ZIPWIRE_EVENT_DMA_ERR, chState->callbackParam);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_WriteData
 * Description   : This internal function computes the correct values for data
 *                 and address registers for the desired data to reach the right
 *                 location at the target node.
 *
 *END**************************************************************************/
static void ZIPWIRE_WriteData(uint8_t instance, uint8_t channel, const zipwire_transfer_descriptor_t *data)
{
    DEV_ASSERT((data->size != ZIPWIRE_16_BITS) || ((data->address & 1U) == 0U));
    DEV_ASSERT((data->size != ZIPWIRE_32_BITS) || ((data->address & 3U) == 0U));

    uint32_t address = data->address;

    /* Configure transfer size */
    SIPI_SetChannelWordLength(instance, channel, data->size);

    switch (data->size)
    {
        case ZIPWIRE_32_BITS:
            /* No processing needed for 32-bit words */
            SIPI_SetChannelData(instance, channel, data->data);
            break;
        case ZIPWIRE_16_BITS:
            /* For 16-bit words with multiple-of-4 addresses (ending in 0b00), data needs to be shifted with 16 bits */
            SIPI_SetChannelData(instance, channel, (data->data << ((data->address & 2U) << 3U)));
            /* LSB of address determines which half word will be transferred from the data register */
            address |= (((data->address & 2U) >> 1U) ^ 1U);
            break;
        case ZIPWIRE_8_BITS:
            /* For 8-bit transfers, data needs to be shifted on the appropriate byte within the DATA register */
            SIPI_SetChannelData(instance, channel, (data->data << (((address & 3U) ^ 3U) << 3U)));
            break;
        default:
            /* This branch should never be reached */
            DEV_ASSERT(false);
            break;
    }

    /* Set the target address - this will trigger the transfer */
    SIPI_SetChannelAddr(instance, channel, address);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ZIPWIRE_StopTransfer
 * Description   : This internal function stops the ongoing transfer.
 *
 *END**************************************************************************/
static void ZIPWIRE_StopTransfer(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    SIPI_Type *sipiBase = s_sipi_instances[instance];
    /* Stop DMA */
    if (chState->dma)
    {
        (void)EDMA_DRV_StopChannel(s_zipwire_chn_state[instance][channel]->dmaDataChn);
        (void)EDMA_DRV_StopChannel(s_zipwire_chn_state[instance][channel]->dmaAddrChn);
        SIPI_EnterInitMode(sipiBase);
        SIPI_EnableChannelDma(instance, channel, false);
        SIPI_ExitInitMode(sipiBase);
    }
    /* Signal the end of the blocking operation */
    if (chState->isBlocking)
    {
        (void)OSIF_SemaPost(&chState->syncSem);
    }
    /* Disable all requests */
    *CCR[instance][channel] &= ~(SIPI_CCR0_WRT_MASK | SIPI_CCR0_RRT_MASK | SIPI_CCR0_TC_MASK | SIPI_CCR0_ST_MASK);
    /* Disable read answer/write ack interrupts */
    *CIR[instance][channel] &= ~(SIPI_CIR0_WAIE_MASK | SIPI_CIR0_RAIE_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnReadAnswerHandler
 * Description   : IRQ handler for read answer received interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnReadAnswerHandler(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* ID request serviced */
    if (chState->idRequest)
    {
        /* Get the received ID */
        *chState->targetId = SIPI_DATA(SIPI_GetChannelData(instance, channel), ZIPWIRE_32_BITS);

        /* Update channel state */
        chState->targetId = NULL;
        chState->idRequest = false;

        /* Disable the interrupt and the id requests for this channel */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, false);
        SIPI_SetChannelIdTransferRequest(instance, channel, false);

        /* Update internal status */
        chState->chStatus = STATUS_SUCCESS;

        /* Notify the waiting thread */
        (void)OSIF_SemaPost(&chState->syncSem);

        /* Call the user callback, if any */
        if (chState->callback != NULL)
        {
            chState->callback(instance, channel, ZIPWIRE_EVENT_TARGET_ID_RECEIVED, chState->callbackParam);
        }
    }
    /* Read request serviced */
    else
    {
        /* Get the received data in the user buffer */
        chState->transferBuffer->data = SIPI_DATA(SIPI_GetChannelData(instance, channel), chState->transferBuffer->size);

        /* If this was the last transfer, disable this channel from further listening for answers */
        if (chState->remainingTransfers == 0U)
        {
            /* Disable the interrupt and the read requests for this channel */
            SIPI_SetChannelInterrupt(instance, channel, SIPI_READ_ANSWER_IRQ, false);
            SIPI_SetChannelReadRequest(instance, channel, false);

            /* Update internal status */
            chState->chStatus = STATUS_SUCCESS;

            /* If the transfer was blocking, notify the waiting thread */
            if (chState->isBlocking)
            {
                (void)OSIF_SemaPost(&chState->syncSem);
                chState->isBlocking = false;
            }

            /* Call the user callback, if any */
            if (chState->callback != NULL)
            {
                chState->callback(instance, channel, ZIPWIRE_EVENT_READ_COMPLETE, chState->callbackParam);
            }
        }
        /* More transfers to be done */
        else
        {
            /* Update buffer reference */
            chState->transferBuffer++;
            chState->remainingTransfers--;

            /* Configure and launch the next transfer */
            SIPI_SetChannelWordLength(instance, channel, chState->transferBuffer->size);
            SIPI_SetChannelAddr(instance, channel, chState->transferBuffer->address);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnAckHandler
 * Description   : IRQ handler for ACK received interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnAckHandler(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];

    /* If this was the last transfer, disable this channel from further writing data */
    if (chState->remainingTransfers == 0U)
    {
        /* Disable the interrupt and the write transfers for this channel */
        SIPI_SetChannelInterrupt(instance, channel, SIPI_ACK_IRQ, false);
        *CCR[instance][channel] &= ~(SIPI_CCR0_WRT_MASK | SIPI_CCR0_ST_MASK | SIPI_CCR0_TC_MASK);

        /* Update internal status */
        chState->chStatus = STATUS_SUCCESS;

        /* If the transfer was blocking, notify the waiting thread */
        if (chState->isBlocking)
        {
            (void)OSIF_SemaPost(&chState->syncSem);
            chState->isBlocking = false;
        }
        /* Call the user callback, if any */
        if ((chState->callback != NULL) && (!(chState->disableNotification)))
        {
            chState->callback(instance, channel, ZIPWIRE_EVENT_WRITE_COMPLETE, chState->callbackParam);
        }
    }
    /* More transfers to be done */
    else
    {
        /* Update buffer reference */
        chState->transferBuffer++;
        chState->remainingTransfers--;

        /* Launch the next transfer */
        ZIPWIRE_WriteData(instance, channel, chState->transferBuffer);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnTriggerHandler
 * Description   : IRQ handler for trigger command received interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnTriggerHandler(uint8_t instance, uint8_t channel)
{
    const zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    if (chState->callback != NULL)
    {
        chState->callback(instance, channel, ZIPWIRE_EVENT_TRIGGER_COMMAND, chState->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_CrcErrHandler
 * Description   : IRQ handler for global CRC error interrupt.
 *
 *END**************************************************************************/
void SIPI_CrcErrHandler(uint8_t instance)
{
    const zipwire_state_t *state = s_zipwire_state[instance];
    if (state->callback != NULL)
    {
        state->callback(instance, ZIPWIRE_EVENT_GLOBAL_CRC_ERR, state->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_MaxCountReachedHandler
 * Description   : IRQ handler for max count reached interrupt.
 *
 *END**************************************************************************/
void SIPI_MaxCountReachedHandler(uint8_t instance)
{
    const zipwire_state_t *state = s_zipwire_state[instance];
    if (state->callback != NULL)
    {
        state->callback(instance, ZIPWIRE_EVENT_MAX_COUNT_REACHED, state->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnTimeoutErrHandler
 * Description   : IRQ handler for channel timeout error interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnTimeoutErrHandler(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    chState->chStatus = STATUS_ZIPWIRE_TIMEOUT_ERR;
    ZIPWIRE_StopTransfer(instance, channel);
    if (chState->callback != NULL)
    {
        chState->callback(instance, channel, ZIPWIRE_EVENT_TIMEOUT_ERR, chState->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnTransIdErrHandler
 * Description   : IRQ handler for channel transaction ID error interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnTransIdErrHandler(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    chState->chStatus = STATUS_ZIPWIRE_TRANSACTION_ID_ERR;
    ZIPWIRE_StopTransfer(instance, channel);
    if (chState->callback != NULL)
    {
        chState->callback(instance, channel, ZIPWIRE_EVENT_TRANSACTION_ID_ERR, chState->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SIPI_ChnAckErrHandler
 * Description   : IRQ handler for channel ACK error interrupt.
 *
 *END**************************************************************************/
void SIPI_ChnAckErrHandler(uint8_t instance, uint8_t channel)
{
    zipwire_chn_state_t *chState = s_zipwire_chn_state[instance][channel];
    chState->chStatus = STATUS_ZIPWIRE_ACK_ERR;
    ZIPWIRE_StopTransfer(instance, channel);
    if (chState->callback != NULL)
    {
        chState->callback(instance, channel, ZIPWIRE_EVENT_ACK_ERR, chState->callbackParam);
    }
}

#if defined(__cplusplus)
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
