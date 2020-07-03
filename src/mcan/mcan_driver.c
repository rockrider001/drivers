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
 
/*!
* @file mcan_driver.c
*
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
* Function is defined for usage by application code.
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
* The return statement before end of function is used for simpler code structure
* and better readability.
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be
* performed between a pointer to object and an integer type.
* The cast is required as CAN instance base addresses are defined as unsigned
* integers in the header file, but the registers are accessed via pointers to
* structures.
*
* @section [global]
* Violates MISRA 2012 Required Rule 11.6, A conversion should not be
* performed between a pointer to object and an integer type.
* The cast is required as CAN instance base addresses are defined as unsigned
* integers in the header file, but the registers are accessed via pointers to
* structures.
*
* @section [global]
* Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
* The code is not dynamically linked. An absolute stack address is obtained when
* taking the address of the near auto variable. A source of error in writing
* dynamic code is that the stack segment may be different from the data segment.
*
* @section [global]
* Violates MISRA 2012 Required Rule 11.3, cast performed between a pointer to
* object type and a pointer to a different object type
* The cast is used for casting a bytes buffer into an words buffer in order to
* optimize copying data to/from the message buffer.
*
*/
#include "mcan_irq.h"
#include "mcan_hw_access.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* CAN bit timing values */
#define MCAN_NUM_TQ_MIN     4U
#define MCAN_NUM_TQ_MAX   385U
#define MCAN_PRESDIV_MAX  511U
#define MCAN_PSEG1_MAX    255U
#define MCAN_PSEG2_MAX    127U
#define MCAN_TSEG1_MIN      0U
#define MCAN_TSEG1_MAX    255U
#define MCAN_TSEG2_MIN      0U
#define MCAN_TSEG2_MAX    127U
#define MCAN_RJW_MAX        1U

static status_t MCAN_StartSendData(uint8_t instance,
                                   uint8_t mb_idx,
                                   const mcan_data_info_t *tx_info,
                                   uint32_t msg_id,
                                   const uint8_t *mb_data,
                                   bool isBlocking);

static void MCAN_CompleteTransfer(uint8_t instance,
                                  mode_type_t type,
                                  uint32_t mb_idx);

static status_t MCAN_StartRxMessageBufferData(uint8_t instance,
                                              uint8_t mb_idx,
                                              mcan_msgbuff_t *data,
                                              bool isBlocking);

static void MCAN_CompleteRxMessageFifoData(uint8_t instance,
                                           mode_type_t type);

static void MCAN_CompleteRxFIFO(uint8_t instance,
                                mode_type_t type);

static status_t MCAN_SetRxFilter(uint8_t instance,
                                 mcan_id_table_t * const * idFilterTable,
                                 uint8_t FiltersNo);

static status_t MCAN_StartRxMessageFifoData(
                    uint8_t instance,
                    mode_type_t type,
                    mcan_msgbuff_t *data,
                    bool isBlocking
                    );

static inline status_t MCAN_NecessarySpace(const mcan_user_config_t *data,
                                           uint8_t stdIdFilters,
                                           uint8_t extIdFilters);

static inline void MCAN_InitFifo(M_CAN_Type * base,
                                 const mcan_user_config_t *data,
                                 uint32_t *address);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to runtime state structure.*/
static mcan_state_t * g_mcanStatePtr[M_CAN_INSTANCE_COUNT] = { NULL };
/* Table of base addresses for CAN instances. */
static M_CAN_Type * const g_mcanBase[] = M_CAN_BASE_PTRS;
 
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_NecessarySpace
 * Description   : Will calculate the required space for configuration and check if
 * exceed available space.
 * This is not a public API as it is called from other driver function.
 *
 *END**************************************************************************/
static inline status_t MCAN_NecessarySpace(const mcan_user_config_t *data,
                                           uint8_t stdIdFilters,
                                           uint8_t extIdFilters)
{
    uint8_t noOfMBs;
    /* Calculate total number of message buffers needed */
    if (data->rx_fifo_needed == MCAN_RXFIFO_DISABLED)
    {

        noOfMBs = (data->rx_num_mb + data->tx_num_mb);
    }
    else if (data->rx_fifo_needed == MCAN_RXFIFO_0_1_ENABLE)
    {

        noOfMBs = (data->fifoConfigs[0U].fifo_elements + data->fifoConfigs[1U].fifo_elements +
                   data->rx_num_mb + data->tx_num_mb);
    }
    else
    {
        noOfMBs = (data->fifoConfigs[0U].fifo_elements + data->rx_num_mb + data->tx_num_mb);
    }
    /* Check if the necessary space will exceed the addressable one by each instance */
    uint32_t usedSpace = McanCalcSpaceWords((uint32_t)stdIdFilters, (uint32_t)extIdFilters, ((uint32_t)MCAN_ComputeTxPayloadSize((uint8_t)data->payload)>>2U), (uint32_t)noOfMBs);
    if (usedSpace >= FEATURE_MCAN_RAM_MESSAGES_WRD)
    {
        return STATUS_CAN_BUFF_OUT_OF_RANGE;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_InitFifo
 * Description   : Will configure the FIFO settings
 * This is not a public API as it is called from other driver function.
 *
 *END**************************************************************************/
static inline void MCAN_InitFifo(M_CAN_Type * base,
                                 const mcan_user_config_t *data,
                                 uint32_t *address)
{
    /* Disable Fifo */
    MCAN_FIFOSetSize(base, 0U, 0U);
    MCAN_FIFOSetStartAddress(base, 0U, 0U);
    MCAN_FIFOSetSize(base, 1U, 0U);
    MCAN_FIFOSetStartAddress(base, 1U, 0U);
    if ((data->rx_fifo_needed == MCAN_RXFIFO_0_ENABLE) || (data->rx_fifo_needed == MCAN_RXFIFO_0_1_ENABLE))
    {
        DEV_ASSERT(data->fifoConfigs != NULL);
        MCAN_FIFOSetOpMode(base, 0U, data->fifoConfigs[0U].modeFIFO);
        MCAN_FIFOSetWaterMark(base, 0U, data->fifoConfigs[0U].fifo_watermark);
        MCAN_FIFOSetSize(base, 0U, data->fifoConfigs[0U].fifo_elements);
        MCAN_FIFOSetPayloadSize(base, 0U, data->payload);
        MCAN_FIFOSetStartAddress(base, 0U, *address);
        *address = MCAN_GetRxFIFOSpace(base, 0U);

    }
    if (data->rx_fifo_needed == MCAN_RXFIFO_1_ENABLE)
    {

        DEV_ASSERT(data->fifoConfigs != NULL);
        MCAN_FIFOSetOpMode(base, 1U, data->fifoConfigs[0U].modeFIFO);
        MCAN_FIFOSetWaterMark(base, 1U, data->fifoConfigs[0U].fifo_watermark);
        MCAN_FIFOSetSize(base, 1U, data->fifoConfigs[0U].fifo_elements);
        MCAN_FIFOSetPayloadSize(base, 1U, data->payload);
        MCAN_FIFOSetStartAddress(base, 1U, *address);
        *address = MCAN_GetRxFIFOSpace(base, 1U);
    }
    if (data->rx_fifo_needed == MCAN_RXFIFO_0_1_ENABLE)
    {
        DEV_ASSERT(data->fifoConfigs != NULL);
        MCAN_FIFOSetOpMode(base, 1U, data->fifoConfigs[1U].modeFIFO);
        MCAN_FIFOSetWaterMark(base, 1U, data->fifoConfigs[1U].fifo_watermark);
        MCAN_FIFOSetSize(base, 1U, data->fifoConfigs[1U].fifo_elements);
        MCAN_FIFOSetPayloadSize(base, 1U, data->payload);
        MCAN_FIFOSetStartAddress(base, 1U, *address);
        *address = MCAN_GetRxFIFOSpace(base, 1U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_Init
 * Description   : Initialize MCAN driver.
 * This function will reset MCAN module, set maximum
 * number of message buffers, initialize all message buffers as inactive, enable
 * RX FIFO if needed, mask all mask bits, disable all MB interrupts, enable
 * MCAN normal mode, and enable all the error interrupts if needed.
 *
 * Implements    : MCAN_DRV_Init_Activity
 *END**************************************************************************/
status_t MCAN_DRV_Init(uint8_t instance,
                       mcan_state_t *state,
                       const mcan_user_config_t *data)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(state != NULL);
    DEV_ASSERT(g_mcanStatePtr[instance] == NULL);
    DEV_ASSERT(data->tx_num_mb <= FEATURE_MCAN_TX_MB_NUM);
    DEV_ASSERT(data->rx_num_mb <= FEATURE_MCAN_RX_MB_NUM);
    
    status_t status;
    M_CAN_Type * base = g_mcanBase[instance];
    mcan_time_segment_t bitrate;
    uint8_t i,j;
    uint32_t address;
    uint32_t idx;
    uint8_t stdIdFilters, extIdFilters;
    stdIdFilters = 0U;
    extIdFilters = 0U;

    if (instance > 0U)
    {
        address = FEATURE_MCAN_MESSAGE_RAM_START_ADR + (FEATURE_MCAN_RAM_MESSAGES_WRD<<2U);
        for (idx = FEATURE_MCAN_RAM_MESSAGES_WRD; idx <= ((FEATURE_MCAN_RAM_MESSAGES_WRD<<1U) - 1U); idx++)
        {
            MCAN_RAM->RAM[idx] = 0U;
        }
    }
    else
    {
        address = FEATURE_MCAN_MESSAGE_RAM_START_ADR;
        for (idx=0; idx <= FEATURE_MCAN_RAM_MESSAGES_WRD; idx++)
        {
            MCAN_RAM->RAM[idx] = 0U;
        }
    }
    MCAN_ClearAllReceivedFlags(base);
    status = MCAN_AllowConfiguration(base);
    if (STATUS_SUCCESS == status)
    {
        bitrate = data->bitrate;
        MCAN_SetNominalTimeSegments(base, &bitrate);
        if (data->fd_enable == true)
        {
            MCAN_SetNonIsoOperation(base, false);
            bitrate = data->bitrate_cbt;
            MCAN_SetDataTimeSegments(base, &bitrate);
            MCAN_SetFlexDataEnable(base, true);
            MCAN_SetBitRateSwitch(base, true);
        }
        else
        {
            MCAN_SetDataTimeSegments(base, &bitrate);
            MCAN_SetFlexDataEnable(base, false);
            MCAN_SetBitRateSwitch(base, false);
        }
    }
    /* Configure Filters to don't store in FIFO non match frames and to filter remote ones */
    MCAN_SetAcceptNonMatchStdFrame(base, NON_MATCH_ACCEPT_REJECT);
    MCAN_SetAcceptNonMatchExtFrame(base, NON_MATCH_ACCEPT_REJECT);
    MCAN_SetRejectRemoteStdFrame(base, false);
    MCAN_SetRejectRemoteExtFrame(base, false);

    if (data->num_id_filters != 0U)
    {
        for (i=0;i<data->num_id_filters;i++)
        {
            DEV_ASSERT(data->filterConfigs[i] != NULL);
            if (data->filterConfigs[i]->isExtendedFrame == true)
            {
                extIdFilters++;
            }
            else
            {
                stdIdFilters++;
            }
        }
    }
    state->extFIFO_filterIDs = extIdFilters;
    state->stdFIFO_filterIDs = stdIdFilters;
    /* Increase the no of filters associated for Individual Rx MBs */
    extIdFilters += data->rx_num_mb;
    stdIdFilters += data->rx_num_mb;

    if (STATUS_SUCCESS == status)
    {
        status = MCAN_NecessarySpace(data, stdIdFilters, extIdFilters);
    }

    if ((extIdFilters <= FEATURE_MCAN_EXT_ID_NUM) && (stdIdFilters <= FEATURE_MCAN_STD_ID_NUM) && (STATUS_SUCCESS == status))
    {
        MCAN_SetStdIDFilterAddress(base, address);
        MCAN_SetStdIDFilterSize(base, stdIdFilters);
        address = MCAN_GetFilterSTDSpace(base);
        MCAN_SetExtIDFilterAddress(base, address);
        MCAN_SetExtIDFilterSize(base, extIdFilters);
        address = MCAN_GetFilterEXTSpace(base);
    }
    else
    {
        status = STATUS_CAN_BUFF_OUT_OF_RANGE;
    }

    if ((data->num_id_filters != 0U) && (STATUS_SUCCESS == status))
    {
        status = MCAN_SetRxFilter(instance, data->filterConfigs, data->num_id_filters);
    }
    if (STATUS_SUCCESS == status)
    {
        status = MCAN_SetOperationMode(base, data->mcanMode);
        /* Config FIFO */
        MCAN_InitFifo(base ,data, &address);
    }

    if ((data->payload != MCAN_PAYLOAD_SIZE_8) && (data->fd_enable == false))
    {
        status = STATUS_ERROR;
    }
    else
    {
        MCAN_SetTxElementdSize(base, data->payload);
        MCAN_SetRxElementdSize(base, data->payload);
    }

    if ((data->tx_num_mb != 0U) && (STATUS_SUCCESS == status))
    {
        MCAN_SetTxBuffSize(base, data->tx_num_mb);
        MCAN_SetTxBuffAddress(base, address);
        address = MCAN_GetTxMBSpace(base);
        /* Enable interrupts for Transmission, Cancellation complete */
        MCAN_SetInterruptLineSelect(base, M_CAN_IE_TCE_SHIFT, instance);
        MCAN_SetInterruptLineSelect(base, M_CAN_IE_TCFE_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IE_TCE_SHIFT, true);
        MCAN_SetInterruptEnable(base, M_CAN_IE_TCFE_SHIFT, true);
    }
    else
    {
        MCAN_SetTxBuffSize(base, 0U);
        MCAN_SetTxBuffAddress(base, 0U);
    }

    if (status == STATUS_SUCCESS)
    {
        if (data->rx_num_mb != 0U)
        {
            MCAN_SetRxBuffAddress(base, address);
        }

    }
    if (status == STATUS_SUCCESS)
    {
        /* Add 2 MBs for RxFIFO0&1 */
        for (i=0U; i < (data->rx_num_mb + data->tx_num_mb+FEATURE_MCAN_NO_RXFIFO); i++)
        {
            status = OSIF_SemaCreate(&state->mbs[i].mbSema, 0U);
            if (status != STATUS_SUCCESS)
            {
                for (j = 0U; j < i; j++)
                {
                    (void)OSIF_SemaDestroy(&state->mbs[j].mbSema);
                }
                status = STATUS_ERROR;
                break;
            }
            state->mbs[i].isBlocking = false;
            state->mbs[i].mb_message = NULL;
            state->mbs[i].state = MCAN_MB_IDLE;
        }
    }

    if (STATUS_SUCCESS == status)
    {
        INT_SYS_EnableIRQ(MCAN_IRQn);
        MCAN_SetInterruptLine(base, instance, true);
        state->noOfMB = data->rx_num_mb + data->tx_num_mb;
        state->callback = NULL;
        state->callbackParam = NULL;
        state->error_callback = NULL;
        state->errorCallbackParam = NULL;
        /* Save runtime structure pointers so irq handler can point to the correct state structure */
        g_mcanStatePtr[instance] = state;
        /* Finish to configure State Machine of MCAN Start CAN operations*/
        MCAN_SetInitialization(base, false);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_ConfigRxMb
 * Description   : Configure a Rx message buffer.
 * Then this function will set up the filter ID according to mb index fields according
 * to reception data, configure the the offset of MB to store the received message
 * as individual MB
 *
 * Implements    : MCAN_DRV_ConfigRxMb_Activity
 *END**************************************************************************/
status_t MCAN_DRV_ConfigRxMb(uint8_t instance,
                             uint8_t mb_idx,
                             const mcan_data_info_t *rx_info,
                             uint32_t msg_id)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    DEV_ASSERT(rx_info != NULL);
    mcan_state_t * state = g_mcanStatePtr[instance];
    status_t status = STATUS_SUCCESS;
    M_CAN_Type const * base = g_mcanBase[instance];

    if (mb_idx >= (g_mcanStatePtr[instance]->noOfMB - MCAN_GetTxBuffSize(base)))
    {
        status = STATUS_CAN_BUFF_OUT_OF_RANGE;
    }

    uint8_t filterIdx = 0U;

    if ((rx_info->msg_id_type == MCAN_MSG_ID_STD) && (status == STATUS_SUCCESS))
    {
        filterIdx = state->stdFIFO_filterIDs + mb_idx;
        mcan_id_table_t filterID;
        filterID.filterConfig = FILTER_CONF_RX_BUFF;
        filterID.filterType = FILTER_TYPE_RANGE_ID;
        filterID.id1 = msg_id;
        /* Stored in the MB offset from MB 0 */
        filterID.id2 = mb_idx;
        filterID.isExtendedFrame = false;
        status = MCAN_CheckIdFilter(base, filterID.isExtendedFrame, msg_id, filterIdx);
        if (status == STATUS_SUCCESS)
        {
            MCAN_SetSTD_IdFilter(base, &filterID, filterIdx);
        }
    }

    if ((rx_info->msg_id_type == MCAN_MSG_ID_EXT) && (status == STATUS_SUCCESS))
    {
        filterIdx = state->extFIFO_filterIDs + mb_idx;
        mcan_id_table_t filterID;
        filterID.filterConfig = FILTER_CONF_RX_BUFF;
        filterID.filterType = FILTER_TYPE_RANGE_ID;
        filterID.id1 = msg_id;
        /* Stored in the MB offset from MB 0 */
        filterID.id2 = mb_idx;
        filterID.isExtendedFrame = true;
        status = MCAN_CheckIdFilter(base, filterID.isExtendedFrame, msg_id, filterIdx);
        if (status == STATUS_SUCCESS)
        {
            MCAN_SetEXT_IdFilter(base, &filterID, filterIdx);
        }
    }
    if (status == STATUS_SUCCESS)
    {
        /* Get Virtual MB Index */
        uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_BUFF, mb_idx);
        state->mbs[vmb_idx].filterIdx = filterIdx;
        if (MCAN_MSG_ID_STD == rx_info->msg_id_type)
        {
            state->mbs[vmb_idx].isExtended = false;
        }
        else
        {
            state->mbs[vmb_idx].isExtended = true;
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_StartRxMessageBufferData
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t MCAN_StartRxMessageBufferData(uint8_t instance,
                                              uint8_t mb_idx,
                                              mcan_msgbuff_t *data,
                                              bool isBlocking)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];

    if (mb_idx >= (state->noOfMB - MCAN_GetTxBuffSize(base)))
    {
        return STATUS_CAN_BUFF_OUT_OF_RANGE;
    }

    /* Get Virtual MB Index */
    uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_BUFF, mb_idx);
    /* Start receiving mailbox */
    if (state->mbs[vmb_idx].state != MCAN_MB_IDLE)
    {
        return STATUS_BUSY;
    }

    state->mbs[vmb_idx].state = MCAN_MB_RX_BUSY;
    state->mbs[vmb_idx].mb_message = data;
    state->mbs[vmb_idx].isBlocking = isBlocking;
    /* Check if the message is already received */
    if (MCAN_GetMsgBuffIntCmd(base, MCAN_RX_BUFF, mb_idx) == 1U)
    {
        MCAN_GetMsgBuff(base, mb_idx, MCAN_RX_BUFF,state->mbs[vmb_idx].mb_message);
        MCAN_CompleteTransfer(instance, MCAN_RX_BUFF, mb_idx);
        if (state->callback != NULL)
        {
            state->callback(instance, MCAN_EVENT_RX_COMPLETE, mb_idx, state);
        }
    }
    /* Message stored to Dedicated Rx Buffer Interrupt Enable */
    MCAN_SetInterruptLineSelect(base, M_CAN_IE_DRXE_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IE_DRXE_SHIFT, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_Receive
 * Description   : This function receives a CAN frame into a configured message
 * buffer. The function returns immediately. If a callback is installed, it will
 * be invoked after the frame was received and read into the specified buffer.
 *
 * Implements    : MCAN_DRV_Receive_Activity
 *END**************************************************************************/
status_t MCAN_DRV_Receive( uint8_t instance,
                           uint8_t mb_idx,
                           mcan_msgbuff_t *data)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    status_t result = STATUS_ERROR;

    result = MCAN_StartRxMessageBufferData(instance, mb_idx, data, false);

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_ReceiveBlocking
 * Description   : This function receives a CAN frame into a configured message
 * buffer. The function blocks until either a frame was received, or the
 * specified timeout expired.
 *
 * Implements    : MCAN_DRV_ReceiveBlocking_Activity
 *END**************************************************************************/
status_t MCAN_DRV_ReceiveBlocking(uint8_t instance,
                                     uint8_t mb_idx,
                                     mcan_msgbuff_t *data,
                                     uint32_t timeout_ms)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    status_t result = STATUS_ERROR;
    M_CAN_Type const * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];

    result = MCAN_StartRxMessageBufferData(instance, mb_idx, data, true);

    if (result == STATUS_SUCCESS)
    {
        status_t status;
        /* Get Virtual MB Index */
        uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_BUFF, mb_idx);
        status = OSIF_SemaWait(&state->mbs[vmb_idx].mbSema, timeout_ms);
        if (status == STATUS_TIMEOUT)
        {
            /* If the flag is set Successful reception else report TimeOut */
            if (MCAN_GetMsgBuffIntCmd(base, MCAN_RX_BUFF, mb_idx) == 0U)
            {
                result = STATUS_TIMEOUT;
            }
            else
            {
                if (state->mbs[vmb_idx].state == MCAN_MB_RX_BUSY)
                {
                   MCAN_GetMsgBuff(base, mb_idx, MCAN_RX_BUFF,state->mbs[vmb_idx].mb_message);
                   MCAN_CompleteTransfer(instance, MCAN_RX_BUFF, mb_idx);
                   result = STATUS_SUCCESS;
                }
                if (state->callback != NULL)
                {
                    state->callback(instance, MCAN_EVENT_RX_COMPLETE, mb_idx, state);
                }
            }
        }

        if (result != STATUS_SUCCESS)
        {
            state->mbs[vmb_idx].state = MCAN_MB_IDLE;
        }
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_ConfigTxMb
 * Description   : Configure a Tx message buffer.
 * This function will first check if RX FIFO is enabled. If RX FIFO is enabled,
 * the function will make sure if the MB requested is not occupied by RX FIFO
 * and ID filter table. Then this function will set up the message buffer fields,
 * configure the message buffer code for Tx buffer as INACTIVE, and enable the
 * Message Buffer interrupt.
 *
 * Implements    : MCAN_DRV_ConfigTxMb_Activity
 *END**************************************************************************/
status_t MCAN_DRV_ConfigTxMb(
    uint8_t instance,
    uint8_t mb_idx,
    const mcan_data_info_t *tx_info,
    uint32_t msg_id)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    DEV_ASSERT(tx_info != NULL);
    DEV_ASSERT(mb_idx <= (g_mcanStatePtr[instance]->noOfMB));

    status_t status = STATUS_SUCCESS;
    M_CAN_Type const * base = g_mcanBase[instance];

    if (mb_idx >= MCAN_GetTxBuffSize(base))
    {
        status = STATUS_CAN_BUFF_OUT_OF_RANGE;
    }
    if (status == STATUS_SUCCESS)
    {
        volatile uint32_t *mcan_mb = MCAN_GetMsgBuffRegion(base, mb_idx, MCAN_TX_BUFF);
        volatile uint32_t *mcan_mb_id   = &mcan_mb[0U];
        volatile uint32_t *mcan_mb_code = &mcan_mb[1U];

        /* Clean up the arbitration field area */
        *mcan_mb_id = 0U;
        *mcan_mb_code = 0U;

        if (tx_info->msg_id_type == MCAN_MSG_ID_STD)
        {
            /* make sure IDE is not set */
            *mcan_mb_id &= ~(CAN_ID_XTD_MASK);

            /* ID[28-18] */
            *mcan_mb_id &= ~CAN_ID_STD_MASK;
            *mcan_mb_id |= (msg_id << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

        }

        if (tx_info->msg_id_type == MCAN_MSG_ID_EXT)
        {
            /* make sure IDE is set */
            *mcan_mb_id |= CAN_ID_XTD_MASK;
            /* ID [28-0] */
            *mcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *mcan_mb_id |= (msg_id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
        }

        if (tx_info->is_remote == true)
        {
            *mcan_mb_id |= CAN_ID_RTR_MASK;
        }

        if (tx_info->fd_enable == true)
        {
            if (MCAN_IsFDEnabled(base) == false)
            {
                return STATUS_ERROR;
            }
            *mcan_mb_code |= CAN_CS_FDF_MASK;
        }

        if (tx_info->enable_brs == true)
        {
            if (MCAN_IsBRSEnabled(base) == false)
            {
                return STATUS_ERROR;
            }
            *mcan_mb_code |= CAN_CS_BRS_MASK;
        }
        /* Set the length of data in bytes */
        uint32_t dlc_code = MCAN_ComputeDLCValue(tx_info->data_length);
        *mcan_mb_code &= ~CAN_CS_DLC_MASK;
        *mcan_mb_code |= ((dlc_code << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetRxFilter
 * Description   : Will set the ID filters based on configuration list provided as
 * a pointer to array of configurations
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t MCAN_SetRxFilter(uint8_t instance,
                                mcan_id_table_t * const * idFilterTable,
                                uint8_t FiltersNo)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(idFilterTable != NULL);
    M_CAN_Type const * base = g_mcanBase[instance];
    mcan_state_t const * state = g_mcanStatePtr[instance];

    uint8_t idx;

    uint8_t ext_idx, std_idx;

    if (FiltersNo > (state->extFIFO_filterIDs + state->stdFIFO_filterIDs))
    {
        return STATUS_ERROR;
    }
    ext_idx = 0U;
    std_idx = 0U;
    for (idx = 0U; idx < FiltersNo; idx++)
    {
        if (idFilterTable[idx]->isExtendedFrame == true)
        {
            if (ext_idx >= state->extFIFO_filterIDs)
            {
                return STATUS_CAN_BUFF_OUT_OF_RANGE;
            }
            else
            {
                MCAN_SetEXT_IdFilter(base, idFilterTable[idx], ext_idx);
                ext_idx++;
            }
        }
        else
        {
            if (std_idx >= state->stdFIFO_filterIDs)
            {
                return STATUS_CAN_BUFF_OUT_OF_RANGE;
            }
            else
            {
                MCAN_SetSTD_IdFilter(base, idFilterTable[idx], std_idx);
                std_idx++;
            }
        }
    }
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_StartSendData
 * Description   : Initiate (start) a transmit by beginning the process of
 * sending data, and configure all Tx MB fields.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t MCAN_StartSendData(uint8_t instance,
                                   uint8_t mb_idx,
                                   const mcan_data_info_t *tx_info,
                                   uint32_t msg_id,
                                   const uint8_t *mb_data,
                                   bool isBlocking)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(tx_info != NULL);

    status_t result=STATUS_SUCCESS;
    uint32_t databyte;
    uint8_t dlc_value;
    mcan_state_t * state = g_mcanStatePtr[instance];
    M_CAN_Type const * base = g_mcanBase[instance];

    if ((tx_info->data_length != 0U) && (mb_data == NULL))
    {
        return STATUS_ERROR;
    }

    if (mb_idx >= MCAN_GetTxBuffSize(base))
    {
         result = STATUS_CAN_BUFF_OUT_OF_RANGE;
    }
    /* Get Virtual MB Index */
    uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_TX_BUFF, mb_idx);
    if (result == STATUS_SUCCESS)
    {
        if (state->mbs[vmb_idx].state != MCAN_MB_IDLE)
        {
            result = STATUS_BUSY;
        }
    }

    if (result == STATUS_SUCCESS)
    {
        state->mbs[vmb_idx].state = MCAN_MB_TX_BUSY;
        state->mbs[vmb_idx].isBlocking = isBlocking;
        state->mbs[vmb_idx].isRemote = tx_info->is_remote;


        volatile uint32_t *mcan_mb = MCAN_GetMsgBuffRegion(base, mb_idx, MCAN_TX_BUFF);
        volatile uint32_t *mcan_mb_id   = &mcan_mb[0];
        volatile uint32_t *mcan_mb_code = &mcan_mb[1];
        volatile uint32_t *mcan_mb_data_32 = &mcan_mb[2];
        const uint32_t *msgData_32 = (const uint32_t *)mb_data;

        if (mb_data != NULL)
        {
            dlc_value = MCAN_GetTxPayloadSize(base);
            uint8_t payload_size = MCAN_ComputeTxPayloadSize(dlc_value);

            for (databyte = 0; databyte < (tx_info->data_length & ~3U); databyte += 4U)
            {
                 REV_BYTES_32(msgData_32[databyte >> 2U], mcan_mb_data_32[databyte >> 2U]);
            }
            if (databyte < tx_info->data_length)
            {
                /* Clear memory to reconstruct 4 byte access */
                mcan_mb_data_32[databyte >> 2U] = 0U;
            }
            for ( ; databyte < tx_info->data_length; databyte++)
            {
                /* Write access is allowed only as 32 bits to message ram */
                uint32_t aux = mb_data[databyte];
                mcan_mb_data_32[databyte >> 2U] = mcan_mb_data_32[databyte >> 2U] | (aux << ((databyte & 3U) << 3U));
            }
            /* Add padding, if needed */
            for (databyte = tx_info->data_length; databyte < payload_size; databyte++)
            {
                if ((databyte & 3U) == 0U)
                {
                    /* Clear memory to reconstruct 4 byte access */
                    mcan_mb_data_32[databyte >> 2U] = 0U;
                }
                /* Write access is allowed only as 32 bits to message ram */
                mcan_mb_data_32[databyte >> 2U] = mcan_mb_data_32[databyte >> 2U] | ((uint32_t)tx_info->fd_padding << ((databyte & 3U) << 3U));
            }
        }

        /* Clean up the arbitration field area */
        *mcan_mb_id = 0U;
        *mcan_mb_code = 0U;

        if (tx_info->msg_id_type == MCAN_MSG_ID_STD)
        {
            /* make sure IDE is not set */
            *mcan_mb_id &= ~(CAN_ID_XTD_MASK);

            /* ID[28-18] */
            *mcan_mb_id &= ~CAN_ID_STD_MASK;
            *mcan_mb_id |= (msg_id << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

        }

        if (tx_info->msg_id_type == MCAN_MSG_ID_EXT)
        {
            /* make sure IDE is set */
            *mcan_mb_id |= CAN_ID_XTD_MASK;
            /* ID [28-0] */
            *mcan_mb_id &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *mcan_mb_id |= (msg_id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));
        }

        if (tx_info->is_remote == true)
        {
            *mcan_mb_id |= CAN_ID_RTR_MASK;
        }

        if (tx_info->fd_enable == true)
        {
            if (MCAN_IsFDEnabled(base) == false)
            {
                result = STATUS_ERROR;
            }
            else
            {
                *mcan_mb_code |= CAN_CS_FDF_MASK;
            }
        }

        if (tx_info->enable_brs == true)
        {
            if (MCAN_IsBRSEnabled(base) == false)
            {
                result = STATUS_ERROR;
            }
            else
            {
                *mcan_mb_code |= CAN_CS_BRS_MASK;
            }
        }

        dlc_value = MCAN_ComputeDLCValue(tx_info->data_length);
        if (dlc_value != 0xFFU)
        {
            /* Set the length of data in bytes */
            *mcan_mb_code &= ~CAN_CS_DLC_MASK;
            *mcan_mb_code |= ((uint32_t)dlc_value << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;
        }
        else
        {
            result = STATUS_ERROR;
        }
    }
    if (result == STATUS_ERROR)
    {
        state->mbs[vmb_idx].state = MCAN_MB_IDLE;
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_Send
 * Description   : This function sends a CAN frame using a configured message
 * buffer. The function returns immediately. If a callback is installed, it will
 * be invoked after the frame was sent.
 *
 * Implements    : MCAN_DRV_Send_Activity
 *END**************************************************************************/
status_t MCAN_DRV_Send(uint8_t instance,
                       uint8_t mb_idx,
                       const mcan_data_info_t *tx_info,
                       uint32_t msg_id,
                       const uint8_t *mb_data)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(tx_info != NULL);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    status_t result;
    M_CAN_Type * base = g_mcanBase[instance];

    result = MCAN_StartSendData(instance, mb_idx, tx_info, msg_id, mb_data, false);
    if (result == STATUS_SUCCESS)
    {
        /* Enable message buffer interrupt*/
        MCAN_SetMsgBuffIntCmd(base, MCAN_TX_BUFF, mb_idx, true);
        MCAN_SetTxBuffAddReq(base, mb_idx);
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SendBlocking
 * Description   : This function sends a CAN frame using a configured message
 * buffer. The function blocks until either the frame was sent, or the specified
 * timeout expired.
 *
 * Implements    : MCAN_DRV_SendBlocking_Activity
 *END**************************************************************************/
status_t MCAN_DRV_SendBlocking( uint8_t instance,
                                uint8_t mb_idx,
                                const mcan_data_info_t *tx_info,
                                uint32_t msg_id,
                                const uint8_t *mb_data,
                                uint32_t timeout_ms)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(tx_info != NULL);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    status_t result = STATUS_ERROR;
    mcan_state_t * state = g_mcanStatePtr[instance];
    M_CAN_Type * base  = g_mcanBase[instance];

    result = MCAN_StartSendData(instance, mb_idx, tx_info, msg_id, mb_data, true);

    if (result == STATUS_SUCCESS)
    {
        /* Enable message buffer interrupt*/
        uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_TX_BUFF, mb_idx);
        /* Always will be STATUS_SUCCESS */
        MCAN_SetMsgBuffIntCmd(base, MCAN_TX_BUFF, mb_idx, true);
        MCAN_SetTxBuffAddReq(base, mb_idx);
        result = OSIF_SemaWait(&state->mbs[vmb_idx].mbSema, timeout_ms);

        if (result == STATUS_TIMEOUT)
        {
            if (state->mbs[vmb_idx].state != MCAN_MB_IDLE)
            {
                MCAN_SetCancelInterrupt(base, mb_idx, true);
                MCAN_SetCancelRequest(base, mb_idx);
                /* Check if this remains set if an ongoing transmission is in progress */
                while (MCAN_GetBuffRequestPendig(base, mb_idx) == true)
                {
                    /* Do Nothing */
                }
                if (MCAN_GetTransmissionOccurred(base, mb_idx) == true)
                {
                    result = STATUS_SUCCESS;
                }
            }
            else
            {
                result = STATUS_SUCCESS;
            }
        }
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_CompleteRxMessageFifoData
 * Description   : Finish up a receive by completing the process of receiving
 * data and transfer in the buffer location.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void MCAN_CompleteRxMessageFifoData(uint8_t instance,mode_type_t type)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    mcan_state_t const * state = g_mcanStatePtr[instance];
    M_CAN_Type * base  = g_mcanBase[instance];
    uint8_t get_idx = 0U;
    uint8_t mb_idx = MCAN_GetVirtualMBIndex(base,type,0U);

    if (MCAN_RX_FIFO0 == type)
    {
        get_idx = MCAN_FIFOGetIndex(base, 0U);
        MCAN_GetMsgBuff(base, get_idx, MCAN_RX_FIFO0, state->mbs[mb_idx].mb_message);
        MCAN_FIFOSetAcknowledge(base, 0U, get_idx);
    }
    else if (MCAN_RX_FIFO1 == type)
    {
        get_idx = MCAN_FIFOGetIndex(base, 1U);
        MCAN_GetMsgBuff(base, get_idx, MCAN_RX_FIFO1, state->mbs[mb_idx].mb_message);
        MCAN_FIFOSetAcknowledge(base, 1U, get_idx);
    }
    else
    {
        /* Misra Require Rule 15.7 */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_CompleteRxFIFO
 * Description   : Finish up a RxFIFO complete reception and signals the event
 * in case of callbacks are installed.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void MCAN_CompleteRxFIFO(uint8_t instance,mode_type_t type)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    mcan_state_t * state = g_mcanStatePtr[instance];
    M_CAN_Type const * base  = g_mcanBase[instance];

    /* Get Virtual MB Index */
    uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, type, 0U);
    if (state->mbs[vmb_idx].state == MCAN_MB_RX_BUSY)
    {
        MCAN_CompleteRxMessageFifoData(instance, type);
        MCAN_CompleteTransfer(instance, type, 0U);
        /* Invoke callback */
        if (state->callback != NULL)
        {
            if (type == MCAN_RX_FIFO0)
            {
                state->callback(instance, MCAN_EVENT_RX0FIFO_COMPLETE, 0U, state);
            }
            else if (type == MCAN_RX_FIFO1)
            {
                state->callback(instance, MCAN_EVENT_RX1FIFO_COMPLETE, 0U, state);
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_CompleteTransfer
 * Description   : Finish up a transmit by completing the process of sending
 * data and disabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static void MCAN_CompleteTransfer(uint8_t instance,
                                  mode_type_t type,
                                  uint32_t mb_idx)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];
    /* Get Virtual MB Index */
    uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, type, mb_idx);

    switch (type)
    {
        case MCAN_TX_BUFF :
            /* Disable the transmitter data register empty interrupt */
            MCAN_SetMsgBuffIntCmd(base, type, mb_idx, false);
            break;
        case MCAN_RX_BUFF :
            /* Clear the receiver data register empty interrupt */
            MCAN_SetMsgBuffIntCmd(base, type, mb_idx, true);
            break;
        case MCAN_RX_FIFO0 :
            MCAN_SetInterruptEnable(base, M_CAN_IR_RF0N_SHIFT, false);
            break;
        case MCAN_RX_FIFO1 :
            MCAN_SetInterruptEnable(base, M_CAN_IR_RF0N_SHIFT, false);
            break;
        default :
            /* Do Nothing */
            break;
    }
    /* Update the information of the module driver state */
    if (state->mbs[vmb_idx].isBlocking)
    {
        (void)OSIF_SemaPost(&state->mbs[vmb_idx].mbSema);
    }
    state->mbs[vmb_idx].state = MCAN_MB_IDLE;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : MCAN_DRV_GetTransferStatus
 * Description   : This function returns whether the previous MCAN receive is
 *                 completed.
 * When performing a non-blocking receive, the user can call this function to
 * ascertain the state of the current receive progress: in progress (or busy)
 * or complete (success).
 *
 * Implements    : MCAN_DRV_GetTransferStatus_Activity
 *END**************************************************************************/
status_t MCAN_DRV_GetTransferStatus(uint8_t instance,mode_type_t type ,uint8_t mb_idx)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    M_CAN_Type const * base = g_mcanBase[instance];
    mcan_state_t const * state = g_mcanStatePtr[instance];
    status_t status = STATUS_BUSY;

    /* Get Virtual MB Index */
    uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, type, mb_idx);

    if (state->mbs[vmb_idx].state == MCAN_MB_IDLE)
    {
        status = STATUS_SUCCESS;
    }
    else
    {
        status = STATUS_BUSY;
    }
    return status;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_TxHandlerProcess
 * Description   : This function will process the Transmission frame handler part
 * form MCAN interruption.
 * This function is not public API.
 *
 *END**************************************************************************/
static inline void MCAN_TxHandlerProcess(uint8_t instance)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    M_CAN_Type const * const base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];
    /* Get the interrupts that are enabled and ready */
    uint32_t mb_idx = 0U;
    while (mb_idx < MCAN_GetTxBuffSize(base))
    {
        uint8_t flag_reg = MCAN_GetMsgBuffIntCmd(base,MCAN_TX_BUFF, mb_idx);

        if (flag_reg != 0U)
        {
            /* Get Virtual MB Index */
            uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_TX_BUFF, mb_idx);
            if (state->mbs[vmb_idx].state == MCAN_MB_TX_BUSY)
            {
                MCAN_CompleteTransfer(instance, MCAN_TX_BUFF, mb_idx);
                /* Invoke callback */
                if (state->callback != NULL)
                {
                    state->callback(instance, MCAN_EVENT_TX_COMPLETE, mb_idx, state);
                }
            }
        }
        mb_idx++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_TxCancelHandlerProcess
 * Description   : This function will process the Transmission Cancel frame part
 * form MCAN interruption.
 * This function is not public API.
 *
 *END**************************************************************************/
static inline void MCAN_TxCancelHandlerProcess(uint8_t instance)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];
    /* Get the interrupts that are enabled and ready */
    uint32_t mb_idx = 0;
    while (mb_idx < MCAN_GetTxBuffSize(base))
    {
        uint8_t flag_reg = MCAN_GetCancelReqStatus(base, mb_idx);

        if (flag_reg != 0U)
        {
            /* Get Virtual MB Index */
            uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_TX_BUFF, mb_idx);
            if (state->mbs[vmb_idx].state == MCAN_MB_TX_BUSY)
            {
                MCAN_CompleteTransfer(instance, MCAN_TX_BUFF, mb_idx);
            }
            /* Invoke callback */
            if (state->callback != NULL)
            {
                state->callback(instance, MCAN_EVENT_CANCEL, mb_idx, state);
            }
            /* Disable corresponding Interrupt */
            MCAN_SetCancelInterrupt(base, mb_idx, false);
        }
        mb_idx++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_RxHandlerProcess
 * Description   : This function will process the Receive as individual MB part
 * form MCAN interruption.
 * This function is not public API.
 *
 *END**************************************************************************/
static inline void MCAN_RxHandlerProcess(uint8_t instance)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    M_CAN_Type const * const base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];

    /* Get the interrupts that are enabled and ready */
    uint32_t mb_idx = 0U;

    while (mb_idx < ((uint32_t)state->noOfMB-MCAN_GetTxBuffSize(base)))
    {
        uint8_t flag_reg = MCAN_GetMsgBuffIntCmd(base,MCAN_RX_BUFF, mb_idx);

        if (flag_reg != 0U)
        {
            /* Get Virtual MB Index */
            uint8_t vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_BUFF, mb_idx);
            if (state->mbs[vmb_idx].state == MCAN_MB_RX_BUSY)
            {
                MCAN_GetMsgBuff(base, mb_idx, MCAN_RX_BUFF,state->mbs[vmb_idx].mb_message);
                MCAN_CompleteTransfer(instance, MCAN_RX_BUFF, mb_idx);
                /* Invoke callback */
                if (state->callback != NULL)
                {
                    state->callback(instance, MCAN_EVENT_RX_COMPLETE, mb_idx, state);
                }
            }
        }
        mb_idx++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_InterrupHandler
 * Description   : Interrupt handler for MCAN.
 * This handler read data from MB or FIFO, and then clear the interrupt flags.
 * This is not a public API as it is called whenever an interrupt occurs.
 *
 *END**************************************************************************/
void MCAN_InterrupHandler(uint8_t instance)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];

    uint32_t irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was transmission complete */
    if ((MCAN_GetInterruptStatus(base) & M_CAN_IR_TC_MASK) == M_CAN_IR_TC_MASK)
    {
        MCAN_TxHandlerProcess(instance);
        /* Clear Interrupt Flag */
        base->IR = M_CAN_IR_TC_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was transmission cancel */
    if ((irqStatus & M_CAN_IR_TCF_MASK) == M_CAN_IR_TCF_MASK)
    {
        MCAN_TxCancelHandlerProcess(instance);
        /* Clear Interrupt Flag */
        base->IR = M_CAN_IR_TCF_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    if ((irqStatus & M_CAN_IR_DRX_MASK) == M_CAN_IR_DRX_MASK)
    {
        MCAN_RxHandlerProcess(instance);
        /* Clear Interrupt Flag */
        base->IR = M_CAN_IR_DRX_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was Receive FIFO_0 New Element */
    if ((irqStatus & M_CAN_IR_RF0N_MASK) == M_CAN_IR_RF0N_MASK)
    {
        MCAN_CompleteRxFIFO(instance, MCAN_RX_FIFO0);
        /* Clear Interrupt Flag */
        base->IR = M_CAN_IR_RF0N_MASK;
    }
    /* Check If the interrupt was Receive FIFO_1 New Element */
    if ((irqStatus & M_CAN_IR_RF1N_MASK) == M_CAN_IR_RF1N_MASK)
    {
        MCAN_CompleteRxFIFO(instance, MCAN_RX_FIFO1);
        /* Clear Interrupt Flag */
        base->IR = M_CAN_IR_RF1N_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was Watermark level FIFO_0  */
    if ((irqStatus & M_CAN_IR_RF0W_MASK) == M_CAN_IR_RF0W_MASK)
    {
         /* Invoke callback */
         if (state->callback != NULL)
         {
             state->callback(instance, MCAN_EVENT_RX0FIFO_WARNING, 0U, state);
         }
         /* Clear Interrupt Flag */
         base->IR = M_CAN_IR_RF0W_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was Watermark level FIFO_1  */
    if ((irqStatus & M_CAN_IR_RF1W_MASK) == M_CAN_IR_RF1W_MASK)
    {
         /* Invoke callback */
         if (state->callback != NULL)
         {
             state->callback(instance, MCAN_EVENT_RX1FIFO_WARNING, 0U, state);
         }
         /* Clear Interrupt Flag */
         base->IR = M_CAN_IR_RF1W_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was FIFO_0 Lost Message (Overflow)  */
    if ((irqStatus & M_CAN_IR_RF0L_MASK) == M_CAN_IR_RF0L_MASK)
    {
         /* Invoke callback */
         if (state->callback != NULL)
         {
             state->callback(instance, MCAN_EVENT_RX0FIFO_OVERFLOW, 0U, state);
         }
         /* Clear Interrupt Flag */
         base->IR = M_CAN_IR_RF0L_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    /* Check If the interrupt was FIFO_0 Lost Message (Overflow)  */
    if ((irqStatus & M_CAN_IR_RF1L_MASK) == M_CAN_IR_RF1L_MASK)
    {
         /* Invoke callback */
         if (state->callback != NULL)
         {
             state->callback(instance, MCAN_EVENT_RX1FIFO_OVERFLOW, 0U, state);
         }
         /* Clear Interrupt Flag */
         base->IR = M_CAN_IR_RF1L_MASK;
    }
    irqStatus = MCAN_GetInterruptStatus(base);
    if ((irqStatus & ERROR_DETECTION_MASK) != 0U)
    {
        /* Invoke callback */
         if (state->error_callback != NULL)
         {
             state->error_callback(instance, MCAN_EVENT_ERROR, state);
         }
         /* Clear All error Interrupts Flags */
         base->IR = ERROR_DETECTION_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_Deinit
 * Description   : Shutdown a MCAN module.
 * This function will disable all MCAN interrupts, and disable the MCAN.
 *
 * Implements    : MCAN_DRV_Deinit_Activity
 *END**************************************************************************/
status_t MCAN_DRV_Deinit(uint8_t instance)
{

    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];
    status_t result = STATUS_SUCCESS;
    status_t osifStat;
    uint32_t i;

    /* Disable MCAN interrupts.*/
    if (state != NULL)
    {
        if (state->error_callback != NULL)
        {
            state->error_callback = NULL;
            state->errorCallbackParam = NULL;
        }
        if (state->callback != NULL)
        {
            state->callback = NULL;
            state->callbackParam = NULL;
        }
    }
    /* Start Initialization to allow configuration */
    MCAN_SetInitialization(base, true);
    (void)MCAN_AllowConfiguration(base);

    MCAN_SetInterruptEnable(base, M_CAN_IE_TCE_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IE_TCFE_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IE_DRXE_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF0W_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF0N_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF0F_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF0L_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF1W_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF1F_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF1L_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_PED_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_RF1F_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_EW_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BO_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_EP_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_ELO_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BEU_SHIFT, false);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BEC_SHIFT, false);

    if (state != NULL)
    {
        for (i = 0U; i < state->noOfMB; i++)
        {
            osifStat = OSIF_SemaDestroy(&state->mbs[i].mbSema);
            if (osifStat != STATUS_SUCCESS)
            {
                result = STATUS_ERROR;
            }
        }
    }

    if (result == STATUS_SUCCESS)
    {
        /* Clear state pointer that is checked by MCAN_DRV_Init */
        g_mcanStatePtr[instance] = NULL;
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_InstallEventCallback
 * Description   : Installs a callback function for the IRQ handler.
 *
 * Implements    : MCAN_DRV_InstallEventCallback_Activity
 *END**************************************************************************/
void MCAN_DRV_InstallEventCallback(uint8_t instance,
                                   mcan_callback_t callback,
                                   void *callbackParam)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    M_CAN_Type * base = g_mcanBase[instance];
    mcan_state_t * state = g_mcanStatePtr[instance];

    state->callback = callback;
    state->callbackParam = callbackParam;

    /* Check if FIFO 0 is initialized */
    if (MCAN_FIFOGetSize(base, 0U) != 0U)
    {
        /* Enable Interrupt for Watermark level in FIFO 0 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF0W_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF0W_SHIFT, true);
        /* Enable Interrupt for Overflow level in FIFO 0 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF0L_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF0L_SHIFT, true);
    }

    /* Check if FIFO 1 is initialized */
    if (MCAN_FIFOGetSize(base, 1U) != 0U)
    {
        /* Enable Interrupt for Watermark level in FIFO 1 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF1W_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF1W_SHIFT, true);
        /* Enable Interrupt for Overflow level in FIFO 1 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF1L_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF1L_SHIFT, true);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_InstallErrorCallback
 * Description   : Installs an error callback function for the IRQ handler and enables/disables
 *                 error interrupts.
 *
 * Implements    : MCAN_DRV_InstallErrorCallback_Activity
 *END**************************************************************************/
void MCAN_DRV_InstallErrorCallback(uint8_t instance,
                                   mcan_error_callback_t callback,
                                   void *callbackParam)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    mcan_state_t * state = g_mcanStatePtr[instance];
    M_CAN_Type * base = g_mcanBase[instance];

    state->error_callback = callback;
    state->errorCallbackParam = callbackParam;

    /* Enable Interrupt for Protocol Error in Data Phase */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_PED_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_PED_SHIFT, true);

    /* Enable Interrupt for Protocol Error in Arbitration Phase */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_PEA_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_PEA_SHIFT, true);

    /* Enable Interrupt for Bus_Off Status */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_BO_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BO_SHIFT, true);

    /* Enable Interrupt for Warning Status */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_EW_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_EW_SHIFT, true);

    /* Enable Interrupt for Error Passive */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_EP_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_EP_SHIFT, true);

    /* Enable Interrupt for Error Logging Overflow */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_ELO_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_ELO_SHIFT, true);

    /* Enable Interrupt for Bit Error Uncorrected */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_BEU_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BEU_SHIFT, true);

    /* Enable Interrupt for Bit Error Corrected */
    MCAN_SetInterruptLineSelect(base, M_CAN_IR_BEC_SHIFT, instance);
    MCAN_SetInterruptEnable(base, M_CAN_IR_BEC_SHIFT, true);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_ConfigRxFifo
 * Description   : Confgure RX FIFO ID filter table elements.
 * This function will confgure RX FIFO ID filter table elements.
 *
 * Implements    : MCAN_DRV_ConfigRxFifo_Activity
 *END**************************************************************************/
status_t MCAN_DRV_ConfigRxFifo(uint8_t instance,
                               mcan_id_table_t * const * id_filter_table,
                               uint8_t noOfFilters)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    M_CAN_Type * base = g_mcanBase[instance];

    /* Check if the MCAN is already in init mode */
    status_t result = MCAN_AllowConfiguration(base);
    if (STATUS_ERROR == result)
    {
        /* Start Initialization to allow configuration */
        MCAN_SetInitialization(base, true);
        (void)MCAN_AllowConfiguration(base);
    }

    status_t status = MCAN_SetRxFilter(instance, id_filter_table, noOfFilters);

    if (STATUS_ERROR == result)
    {
        /* Finish to configurate State Machine of MCAN Start CAN operations*/
        MCAN_SetInitialization(base, false);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_StartRxMessageFifoData
 * Description   : Initiate (start) a receive by beginning the process of
 * receiving data and enabling the interrupt.
 * This is not a public API as it is called from other driver functions.
 *
 *END**************************************************************************/
static status_t MCAN_StartRxMessageFifoData(uint8_t instance,
                                            mode_type_t type,
                                            mcan_msgbuff_t *data,
                                            bool isBlocking)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    mcan_state_t * state = g_mcanStatePtr[instance];
    M_CAN_Type * base = g_mcanBase[instance];
    if (type != MCAN_RX_FIFO1)
    {
        if (type != MCAN_RX_FIFO0)
        {
            return STATUS_ERROR;
        }
    }
    /* Check if enabled FIFO */
    if (type == MCAN_RX_FIFO0)
    {
        if(MCAN_FIFOGetSize(base, 0U) == 0U)
        {
            return STATUS_ERROR;
        }
    }

    if  (type == MCAN_RX_FIFO1)
     {
        if (MCAN_FIFOGetSize(base, 1U) == 0U)
        {
            return STATUS_ERROR;
        }
     }

    uint8_t vmb_idx =  MCAN_GetVirtualMBIndex(base, type, 0U);
    /* Start receiving fifo */
    if (state->mbs[vmb_idx].state != MCAN_MB_IDLE)
    {
        return STATUS_BUSY;
    }
    state->mbs[vmb_idx].state = MCAN_MB_RX_BUSY;
    state->mbs[vmb_idx].isBlocking = isBlocking;
    /* This will get filled by the interrupt handler */
    state->mbs[vmb_idx].mb_message = data;

    if (MCAN_RX_FIFO0 == type)
    {
        /* Enable Interrupt for a new Frame received in FIFO 0 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF0N_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF0N_SHIFT, true);
        /* If messages already in FIFO */
        if (MCAN_FIFOGetLevel(base,0U) > 0U)
        {
            MCAN_CompleteRxFIFO(instance, type);
        }
    }

    if (MCAN_RX_FIFO1 == type)
    {
        /* Enable Interrupt for a new Frame received in FIFO 1 */
        MCAN_SetInterruptLineSelect(base, M_CAN_IR_RF1N_SHIFT, instance);
        MCAN_SetInterruptEnable(base, M_CAN_IR_RF1N_SHIFT, true);
        /* If messages already in FIFO */
        if (MCAN_FIFOGetLevel(base,1U) > 0U)
        {
            MCAN_CompleteRxFIFO(instance, type);
        }
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_RxFifo
 * Description   : This function receives a CAN frame using the Rx FIFO. The
 * function returns immediately. If a callback is installed, it will be invoked
 * after the frame was received and read into the specified buffer.
 *
 * Implements    : MCAN_DRV_RxFifo_Activity
 *END**************************************************************************/
status_t MCAN_DRV_RxFifo(uint8_t instance, 
                         uint8_t fifo,
                         mcan_msgbuff_t *data)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    status_t result = STATUS_ERROR;

    if (fifo == 0U)
    {
        result = MCAN_StartRxMessageFifoData(instance, MCAN_RX_FIFO0, data, false);
    }
    else if (fifo == 1U)
    {
        result = MCAN_StartRxMessageFifoData(instance, MCAN_RX_FIFO1, data, false);
    }
    else
    {
        /* Misra Require Rule 15.7 */
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_RxFifoBlocking
 * Description   : This function receives a CAN frame using the Rx FIFO. The
 * function blocks until either a frame was received, or the specified timeout
 * expired.
 *
 * Implements    : MCAN_DRV_RxFifoBlocking_Activity
 *END**************************************************************************/
status_t MCAN_DRV_RxFifoBlocking(uint8_t instance, uint8_t fifo,
                                 mcan_msgbuff_t *data,
                                 uint32_t timeout_ms)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type *base = g_mcanBase[instance];
    mcan_state_t *state = g_mcanStatePtr[instance];
    status_t result = STATUS_ERROR;
    uint8_t vmb_idx = 0U;

    if (fifo == 0U)
    {
        result = MCAN_StartRxMessageFifoData(instance, MCAN_RX_FIFO0, data, true);
        /* Get Virtual MB Index */
        vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_FIFO0, 0U);
    }
    else if (fifo == 1U)
    {
        result = MCAN_StartRxMessageFifoData(instance, MCAN_RX_FIFO1, data, true);
        /* Get Virtual MB Index */
        vmb_idx = MCAN_GetVirtualMBIndex(base, MCAN_RX_FIFO1, 0U);
    }
    else
    {
        /* Misra Require Rule 15.7 */
    }

    if (result == STATUS_SUCCESS)
    {
        result = OSIF_SemaWait(&state->mbs[vmb_idx].mbSema, timeout_ms);

        if (result == STATUS_TIMEOUT)
        {
            if (fifo == 0U)
            {
                MCAN_SetInterruptEnable(base, M_CAN_IR_RF0N_SHIFT, false);
            }
            else if (fifo == 1U)
            {
                MCAN_SetInterruptEnable(base, M_CAN_IR_RF1N_SHIFT, false);
            }
            else
            {
                /* Misra Require Rule 15.7 */
            }

            if (state->mbs[vmb_idx].state == MCAN_MB_IDLE)
            {
                result = STATUS_SUCCESS;
            }
            state->mbs[vmb_idx].state = MCAN_MB_IDLE;
        }
    }
    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SetNominalBitrate
 * Description   : Set MCAN baudrate.
 * This function will set up all the time segment values for classical frames.
 * Those time segment values are passed in by the user and are based on the required baudrate.
 *
 * Implements    : MCAN_DRV_SetNominalBitrate_Activity
 *END**************************************************************************/
status_t MCAN_DRV_SetNominalBitrate(uint8_t instance, const mcan_time_segment_t *bitrate)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type *base = g_mcanBase[instance];
    status_t status = STATUS_ERROR;

    /* Start Initialization to allow configuration */
    MCAN_SetInitialization(base, true);
    status = MCAN_AllowConfiguration(base);

    DEV_ASSERT(status == STATUS_SUCCESS);

    MCAN_SetNominalTimeSegments(base, bitrate);

    MCAN_SetInitialization(base, false);
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SetDataBitrate
 * Description   : Set MCAN baudrate.
 * This function will set up all the time segment values for classical frames and extended FD frames.
 * Those time segment values are passed in by the user and are based on the required baudrate.
 *
 * Implements    : MCAN_DRV_SetDataBitrate_Activity
 *END**************************************************************************/
status_t MCAN_DRV_SetDataBitrate(uint8_t instance, const mcan_time_segment_t *bitrate)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type *base = g_mcanBase[instance];
    status_t status = STATUS_ERROR;
    /* Start Initialization to allow configuration */
    MCAN_SetInitialization(base, true);
    status = MCAN_AllowConfiguration(base);

    DEV_ASSERT(status == STATUS_SUCCESS);

    MCAN_SetDataTimeSegments(base, bitrate);

    MCAN_SetInitialization(base, false);
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_AbortTransfer
 * Description   : This function shuts down the MCAN by disabling interrupts and
 *                 the transmitter/receiver.
 *
 * Implements    : MCAN_DRV_AbortTransfer_Activity
 *END**************************************************************************/
status_t MCAN_DRV_AbortTransfer(uint8_t instance, mode_type_t type, uint8_t mb_idx)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    mcan_state_t const * state = g_mcanStatePtr[instance];
    M_CAN_Type *base = g_mcanBase[instance];
    status_t status = STATUS_ERROR;
    uint8_t vmb_idx = 0U;

    switch (type)
    {
        case MCAN_RX_BUFF :
            if (mb_idx >= (state->noOfMB - MCAN_GetTxBuffSize(base)))
            {
                /* Should Be STATUS_ERROR */
                return status;
            }
            vmb_idx = MCAN_GetVirtualMBIndex(base, type, mb_idx);
        break;
        case MCAN_TX_BUFF:
            if (mb_idx >= MCAN_GetTxBuffSize(base))
            {
                /* Should Be STATUS_ERROR */
                return status;
            }
            /* Get Virtual MB Index */
            vmb_idx = MCAN_GetVirtualMBIndex(base, type, mb_idx);
        break;
        case MCAN_RX_FIFO0:
            /* Allowed fall through */
        case MCAN_RX_FIFO1:
            /* Get Virtual MB Index */
            vmb_idx = MCAN_GetVirtualMBIndex(base, type, 0U);
        break;
        default :
            /* Should not reach here all cases are covered */
        break;
    }

    if (state->mbs[vmb_idx].state == MCAN_MB_IDLE)
    {
        return STATUS_CAN_NO_TRANSFER_IN_PROGRESS;
    }

    switch (type)
    {
    case MCAN_RX_BUFF :
    /* Disable Filter to not receive any more messages in the MBs */
    MCAN_AbortRX(base, state, vmb_idx);
    MCAN_CompleteTransfer(instance, type, mb_idx);
    status = STATUS_SUCCESS;
    /* Remains status success from MCAN_AbortRX */
    break;
    case MCAN_TX_BUFF:
    MCAN_SetCancelInterrupt(base, mb_idx, true);
    MCAN_SetCancelRequest(base, mb_idx);
    /* Check if this remains set if ongoing transmission is in progress */
    while (MCAN_GetBuffRequestPendig(base, mb_idx) == true)
    {
        /* Do Nothing */
    }
    if (MCAN_GetTransmissionOccurred(base, mb_idx) == true)
    {
        status = STATUS_CAN_NO_TRANSFER_IN_PROGRESS;
    }
    else
    {
        status = STATUS_SUCCESS;
    }
    MCAN_CompleteTransfer(instance, type, mb_idx);
    break;
    case MCAN_RX_FIFO0:
    /* Allow Fall through */
    case MCAN_RX_FIFO1:
    {
        MCAN_CompleteTransfer(instance, type, 0U);
        status = STATUS_SUCCESS;
    }
    break;
    default :
        /* Should not reach here all cases are covered */
    break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SetRxFifoFilterMask
 * Description   : This function sets a filter Mask for a specific filter that is
 * not configured for a specific Rx Message Buffer and is a classic filter type.
 *
 * Implements    : MCAN_DRV_SetRxFifoFilterMask_Activity
 *END**************************************************************************/
status_t MCAN_DRV_SetRxFifoFilterMask(uint8_t instance,
                                      mcan_msgbuff_id_type_t type,
                                      uint8_t fl_idx,
                                      uint32_t mask)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);

    M_CAN_Type const *base = g_mcanBase[instance];

    status_t status = STATUS_ERROR;
    mcan_id_table_t filter;

    if (MCAN_MSG_ID_STD == type)
    {
        if (fl_idx < MCAN_GetStdIDFilterSize(base))
        {
            MCAN_GetSTD_IDFilter(base, &filter, fl_idx);
            if ((filter.filterConfig != FILTER_CONF_RX_BUFF) && (filter.filterType == FILTER_TYPE_CLASIC))
            {
                filter.id2 = mask;
                MCAN_SetSTD_IdFilter(base, &filter, fl_idx);
                status = STATUS_SUCCESS;
            }
        }
        else
        {
            status = STATUS_CAN_BUFF_OUT_OF_RANGE;
        }
    }
    else
    {
        if (fl_idx < MCAN_GetExtIDFilterSize(base))
        {
            MCAN_GetEXT_IDFilter(base, &filter, fl_idx);
            if ((filter.filterConfig != FILTER_CONF_RX_BUFF) && (filter.filterType == FILTER_TYPE_CLASIC))
            {
                filter.id2 = mask;
                MCAN_SetEXT_IdFilter(base, &filter, fl_idx);
                status = STATUS_SUCCESS;
            }
        }
        else
        {
            status = STATUS_CAN_BUFF_OUT_OF_RANGE;
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SetGlobalFilterConfig
 * Description   : This function sets Global filter Configuration behavior,
 * Store in RxFIFO or Reject non match frames, and pass to filter or automatic rejection 
 * of remote frames.
 *
 * Implements    : MCAN_DRV_SetGlobalFilterConfig_Activity
 *END**************************************************************************/
void MCAN_DRV_SetGlobalFilterConfig(uint8_t instance,
                                    nonMatchAccept_t nonAcceptStd,
                                    nonMatchAccept_t nonAcceptExt,
                                    bool remoteStd,
                                    bool remoteExt)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(g_mcanStatePtr[instance] != NULL);
    M_CAN_Type * const base = g_mcanBase[instance];
    status_t status = STATUS_SUCCESS;

    status = MCAN_AllowConfiguration(base);
    if (status == STATUS_ERROR)
    {
        MCAN_SetInitialization(base, true);
        (void)MCAN_AllowConfiguration(base);
    }
    MCAN_SetAcceptNonMatchStdFrame(base, nonAcceptStd);
    MCAN_SetAcceptNonMatchExtFrame(base, nonAcceptExt);
    MCAN_SetRejectRemoteStdFrame(base, remoteStd);
    MCAN_SetRejectRemoteExtFrame(base, remoteExt);
    if (status == STATUS_ERROR)
    {
        MCAN_SetInitialization(base, false);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_CheckDsample
 * Description   : Check the Sample value
 *
 *END**************************************************************************/
static inline uint32_t MCAN_CheckDsample(uint32_t tmpSample, uint32_t samplePoint)
{
    if (tmpSample > samplePoint)
    {
        return (tmpSample - samplePoint);
    }
    return (samplePoint - tmpSample);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_CheckdBitrate
 * Description   : Check the Bitrate value
 *
 *END**************************************************************************/
static inline uint32_t MCAN_CheckdBitrate(uint32_t tmpBitrate, uint32_t bitrate)
{
    if (tmpBitrate > bitrate)
    {
        return (tmpBitrate - bitrate);
    }
    return (bitrate - tmpBitrate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_CheckJumpwidth
 * Description   : Check the JumpWidth value
 *
 *END**************************************************************************/
static inline uint32_t MCAN_CheckJumpwidth(uint32_t pseg1)
{
    if (pseg1 < MCAN_RJW_MAX)
    {
        return pseg1;
    }
    return MCAN_RJW_MAX;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_ProccessTSeg
 * Description   : Calculate Tseg value
 *
 *END**************************************************************************/
static inline void MCAN_ProccessTSeg(uint32_t * tSeg1, uint32_t * tSeg2)
{
    /* Adjust time segment 1 and time segment 2 */
    while ((*tSeg1 >= MCAN_TSEG1_MAX) || (*tSeg2 < 1U))
    {
        *tSeg2 = *tSeg2 + 1U;
        if (*tSeg1 > 0U)
        {
            *tSeg1 = *tSeg1 - 1U;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_ProccessPSeg
 * Description   : Calculate Pseg value
 *
 *END**************************************************************************/
static inline void MCAN_ProccessPSeg(uint32_t * tmpPropseg, uint32_t * tmpPseg1)
{
    while ((*tmpPropseg + *tmpPseg1 + 1U) > MCAN_PSEG1_MAX)
    {
        if (*tmpPropseg > 0U)
        {
            *tmpPropseg = *tmpPropseg - 1U;
        }
        *tmpPseg1 = *tmpPseg1 + 1U;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_BitrateToTimeSeg
 * Description   : Converts a bitrate (kbit/s) in time segment values for
 *                 standard CAN frame.
 *
 *END**************************************************************************/
static void MCAN_BitrateToTimeSeg(uint32_t bitrate,
                                  uint32_t clkFreq,
                                  mcan_time_segment_t *timeSeg)
{
    uint32_t dBitrate, tmpBitrate, dBitrateMin, tmpPresdiv, tmpSample,
             dSampleMin, dSample, samplePoint, numTq, pseg1, pseg2, propseg,
             presdiv, tSeg1, tSeg2, tmpPseg1, tmpPseg2, tmpPropseg;

    presdiv = 0U;
    propseg = 0U;
    pseg1 = 0U;
    pseg2 = 0U;

    dSampleMin = 100U;
    dBitrateMin = 1000000U;
    samplePoint = 88U;

    for (tmpPresdiv = 0U; tmpPresdiv < MCAN_PRESDIV_MAX; tmpPresdiv++) {

        /* Compute the number of time quanta in 1 bit time */
        numTq = clkFreq / ((tmpPresdiv + 1U) * bitrate);
        /* Compute the real bitrate resulted */
        tmpBitrate = clkFreq / ((tmpPresdiv + 1U) * numTq);

        /* The number of time quanta in 1 bit time must be lower than the one supported */
        if ((numTq >= MCAN_NUM_TQ_MIN) && (numTq <= MCAN_NUM_TQ_MAX))
        {
            /* Compute time segments based on the value of the sampling point */
            tSeg1 = (numTq * samplePoint / 100U) - 1U;
            /* Round ((numTq * samplePoint / 100U)) */
            if (((numTq * samplePoint) % 100U) >= 50U)
            {
                tSeg1 +=1U;
            }

            tSeg2 = numTq - 1U - tSeg1;

            /* Adjust time segment 1 and time segment 2 */
            MCAN_ProccessTSeg(&tSeg1, &tSeg2);

            tmpPseg2 = tSeg2 - 1U;

            /* Start from pseg1 = pseg2 and adjust until propseg is valid */
            tmpPseg1 = tmpPseg2;
            tmpPropseg = tSeg1 - tmpPseg1 - 2U;

            MCAN_ProccessPSeg(&tmpPropseg, &tmpPseg1);

            if (((tSeg1 > MCAN_TSEG1_MAX) || (tSeg2 > MCAN_TSEG2_MAX) || (tSeg2 <= MCAN_TSEG2_MIN) || (tSeg1 <= MCAN_TSEG1_MIN)) ||
               (((tmpPseg1 + tmpPropseg+1U) > MCAN_PSEG1_MAX) || (tmpPseg2 > MCAN_PSEG2_MAX)))
            {
                continue;
            }

            tmpSample = ((tSeg1 + 1U) * 100U) / numTq;
            dSample = MCAN_CheckDsample(tmpSample , samplePoint);
            dBitrate = MCAN_CheckdBitrate(tmpBitrate , bitrate);

            if ((dBitrate < dBitrateMin) ||
                ((dBitrate == dBitrateMin) && (dSample < dSampleMin)))
            {
                dSampleMin = dSample;
                dBitrateMin = dBitrate;
                pseg1 = tmpPseg1;
                pseg2 = tmpPseg2;
                presdiv = tmpPresdiv;
                propseg = tmpPropseg+1U;

                if ((dBitrate == 0U) && (dSample <= 1U))
                {
                    break;
                }
            }
        }
    }

    timeSeg->phaseSeg1 = pseg1;
    timeSeg->phaseSeg2 = pseg2;
    timeSeg->preDivider = presdiv;
    timeSeg->propSeg = propseg;
    timeSeg->rJumpwidth = MCAN_CheckJumpwidth(pseg1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_GetErrorStatus
 * Description   : Gets the error status
 *
 * Implements    : MCAN_DRV_GetErrorStatus_Activity
 *END**************************************************************************/
void MCAN_DRV_GetErrorStatus(uint8_t instance, mcan_error_status_t * errors)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);
    DEV_ASSERT(errors != NULL);
    M_CAN_Type const * base  = g_mcanBase[instance];
    
    errors->errorCounter = MCAN_GetErrorCounter(base);
    errors->protocolStatus = MCAN_GetProtocolStatus(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_SetTDCOffset
 * Description   : Enables/Disables the Transceiver Delay Compensation feature, sets
 * the Transceiver Delay Compensation Offset and Transmitter Delay Compensation Filter 
 * Window Length
 *
 * Implements    : MCAN_DRV_SetTDCOffset_Activity
 *END**************************************************************************/
status_t MCAN_DRV_SetTDCOffset(uint8_t instance, bool enable, uint8_t offset, uint8_t filterWindowLength)
{
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type * base = g_mcanBase[instance];
    status_t status = STATUS_ERROR;

    if (offset >= filterWindowLength)
    {
        /* Not allowed the offset to be greater than TDCF */
        return status;
    }
    /* Start Initialization to allow configuration */
    MCAN_SetInitialization(base, true);
    status = MCAN_AllowConfiguration(base);
    
    DEV_ASSERT(STATUS_SUCCESS == status);

    MCAN_EnableTDC(base, enable);
    MCAN_SetTDCOffset(base, offset);
    MCAN_SetTDCFilterWindow(base, filterWindowLength);
    
    MCAN_SetInitialization(base, false);
    
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_GetDefaultConfig
 * Description   : Gets the default configuration structure
 *
 * Implements    : MCAN_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void MCAN_DRV_GetDefaultConfig(mcan_user_config_t *config)
{
    /* Checks input parameter. */
    DEV_ASSERT(config != NULL);

    uint32_t clkFreq;
    mcan_time_segment_t timeSeg;

    /* Get the PE clock frequency */
    (void) CLOCK_SYS_GetFreq(MCAN0_CLK, &clkFreq);
    /* Time segments computed for PE bitrate = 500 Kbit/s, sample point = 87.5 */
    MCAN_BitrateToTimeSeg(500000U, clkFreq, &timeSeg);

    /* Maximum number of message buffers */
    config->rx_num_mb = 8U;
    config->tx_num_mb = 8U;
    /* Rx FIFO is disabled */
    config->rx_fifo_needed = MCAN_RXFIFO_DISABLED;
    /* Number of Rx FIFO ID filters */
    config->num_id_filters = 0U;
    /* Normal operation mode */
    config->mcanMode = MCAN_NORMAL_MODE;
    /* Time segments for the arbitration phase */
    config->bitrate = timeSeg;
    /* Payload size */
    config->payload = MCAN_PAYLOAD_SIZE_8;
    /* Flexible data rate is disabled */
    config->fd_enable = false;
    /* Time segments for the data phase of FD frames */
    config->bitrate_cbt = timeSeg;
    /* Rx FIFO No config */
    config->fifoConfigs = NULL;
    config->filterConfigs = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_GetBitrate
 * Description   : Get MCAN Nominal baudrate.
 * This function will be return the current bit rate settings for classical frames
 * or the arbitration phase of FD frames.
 *
 * Implements    : MCAN_DRV_GetBitrate_Activity
 *END**************************************************************************/
void  MCAN_DRV_GetBitrate(uint8_t instance, mcan_time_segment_t *bitrate)
{
    DEV_ASSERT(bitrate != NULL);
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type const * base = g_mcanBase[instance];
    
    MCAN_GetNominalTimeSegments(base, bitrate);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_DRV_GetBitrateFD
 * Description   : Get MCAN baudrate for data phase.
 * This function will be return the current bit rate settings for the data phase
 * of FD frames.
 *
 * Implements    : MCAN_DRV_GetBitrateFD_Activity
 *END**************************************************************************/
void  MCAN_DRV_GetBitrateFD(uint8_t instance, mcan_time_segment_t *bitrate)
{
    DEV_ASSERT(bitrate != NULL);
    DEV_ASSERT(instance < M_CAN_INSTANCE_COUNT);

    M_CAN_Type const * base = g_mcanBase[instance];
    MCAN_GetDataTimeSegments(base, bitrate);
}    
/*******************************************************************************
 * EOF
 ******************************************************************************/
