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
 * @file mcan_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * There are some global macros used for accessing different fields of CAN frames
 * which might also be useful to the user.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
 * Function-like macros are used instead of inline functions in order to ensure
 * that the performance will not be decreased if the functions will not be
 * inlined by the compiler.
 */
 
#ifndef MCAN_DRIVER_H
#define MCAN_DRIVER_H

#include "device_registers.h"
#include "osif.h"

/*!
 * @defgroup mcan_driver MCAN Driver
 * @ingroup mcan
 * @addtogroup mcan_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief MCAN transfer type related structures
 * Implements : mode_type_t_Class
 */
typedef enum{
    MCAN_TX_BUFF,    /*!< Tx Buffer Transmission Payload*/
    MCAN_RX_BUFF,    /*!< Rx Buffer Reception Payload*/
    MCAN_RX_FIFO0,   /*!< Rx FIFO 0 Payload */
    MCAN_RX_FIFO1,   /*!< Rx FIFO 1 Payload */
} mode_type_t;

 /*! @brief MCAN bitrate related structures
 * Implements : mcan_time_segment_t_Class
 */
typedef struct {
    uint32_t propSeg;         /*!< Propagation segment*/
    uint32_t phaseSeg1;       /*!< Phase segment 1*/
    uint32_t phaseSeg2;       /*!< Phase segment 2*/
    uint32_t preDivider;      /*!< Clock prescaler division factor*/
    uint32_t rJumpwidth;      /*!< Resync jump width*/
} mcan_time_segment_t;

/*! @brief MCAN payload sizes
 * Implements : mcan_fd_payload_size_t_Class
 */
typedef enum {
    MCAN_PAYLOAD_SIZE_8  ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_12 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_16 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_20 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_24 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_32 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_48 ,    /*!< MCAN message buffer payload size in bytes*/
    MCAN_PAYLOAD_SIZE_64      /*!< MCAN message buffer payload size in bytes*/
} mcan_fd_payload_size_t;

/*! @brief MCAN Message Buffer ID type
 * Implements : mcan_msgbuff_id_type_t_Class
 */
typedef enum {
    MCAN_MSG_ID_STD,         /*!< Standard ID*/
    MCAN_MSG_ID_EXT          /*!< Extended ID*/
} mcan_msgbuff_id_type_t;

/*! @brief MCAN Events Signals for Callback 
 * Implements : mcan_event_type_t_Class
 */
typedef enum {
    MCAN_EVENT_RX_COMPLETE,
    MCAN_EVENT_TX_COMPLETE,
    MCAN_EVENT_RX0FIFO_COMPLETE,
    MCAN_EVENT_RX1FIFO_COMPLETE,
    MCAN_EVENT_RX0FIFO_WARNING,
    MCAN_EVENT_RX1FIFO_WARNING,
    MCAN_EVENT_RX0FIFO_OVERFLOW,
    MCAN_EVENT_RX1FIFO_OVERFLOW,
    MCAN_EVENT_ERROR,
    MCAN_EVENT_CANCEL
} mcan_event_type_t;    

/*! @brief The state of a given MB (idle/Rx busy/Tx busy).
 * Implements : mcan_mb_state_t_Class
 */
typedef enum {
    MCAN_MB_IDLE,      /*!< The MB is not used by any transfer. */
    MCAN_MB_RX_BUSY,   /*!< The MB is used for a reception. */
    MCAN_MB_TX_BUSY,   /*!< The MB is used for a transmission. */
} mcan_mb_state_t;

/*! @brief Fifo Mode of MCAN interface.
 * Implements : mcan_rxfifo_mode_t_Class
 */
typedef enum {
    MCAN_RXFIFO_DISABLED,
    MCAN_RXFIFO_0_ENABLE,
    MCAN_RXFIFO_1_ENABLE,
    MCAN_RXFIFO_0_1_ENABLE
} mcan_rxfifo_mode_t;

/*! @brief MCAN operation modes
 * Implements : mcan_operation_modes_t_Class
 */
typedef enum {
    MCAN_NORMAL_MODE,        /*!< Normal mode or user mode */
    MCAN_LISTEN_ONLY_MODE,   /*!< Listen-only mode */
    MCAN_LOOPBACK_MODE,      /*!< Loop-back mode */
    MCAN_TEST_MODE,          /*!< Test Mode */
    MCAN_DISABLE_MODE        /*!< Module disable mode */
} mcan_operation_modes_t;

/*! @brief MCAN FIFO operation modes
 * Implements : fifo_op_mode_t_Class
 */
typedef enum {
    FIFO_MODE_BLOCKING,     /*!< FIFO blocking mode */
    FIFO_MODE_OVERWRITE     /*!< FIFO overwrite mode */    
} fifo_op_mode_t;

/*! @brief MCAN RxFIFO Configuration
 * Implements : rxfifo_conf_t_Class
 */
typedef struct{
    fifo_op_mode_t modeFIFO;    /*!< RxFIFO operation mode */
    uint8_t fifo_elements;      /*!< No of RxFIFO Elements */
    uint8_t fifo_watermark;     /*!< No of RxFIFO Watermark */
} rxfifo_conf_t;

/*! @brief MCAN Id Filter NonMatch acceptance
 * Implements : nonMatchAccept_t_Class
 */
typedef enum{
     NON_MATCH_ACCEPT_RXFIFO_0, /*!< Non Matched Frames are Accepted in Rx FIFO 0 */
     NON_MATCH_ACCEPT_RXFIFO_1, /*!< Non Matched Frames are Accepted in Rx FIFO 1 */
     NON_MATCH_ACCEPT_REJECT    /*!< Non Matched Frames are Rejected */
} nonMatchAccept_t;

/*! @brief MCAN Filter Type
 * Implements : filter_type_Class
 */
typedef enum{
    FILTER_TYPE_RANGE_ID,               /*!< Range filter from ID1 to ID2 */
    FILTER_TYPE_DUAL_ID,                /*!< Dual ID filter for ID1 or ID2 */
    FILTER_TYPE_CLASIC,                 /*!< Classic filter: ID1 = filter, ID2 = mask */
    FILTER_TYPE_DISABLE                 /*!< Filter element disabled in STD ID, Range filter from ID1 to ID2 (ID2 >= ID1),
                                             XIDAM mask not applied in case of EXT ID */
} filter_type;

/*! @brief MCAN Filter Configuration Type
 * Implements : filter_element_config_t_Class
 */
typedef enum{
    FILTER_CONF_DISABLE,                /*!< Disable filter element */
    FILTER_CONF_RX_FIFO0,               /*!< Store in Rx FIFO 0 if filter matches */
    FILTER_CONF_RX_FIFO1,               /*!< Store in Rx FIFO 1 if filter matches */
    FILTER_CONF_REJECT_ID,              /*!< Reject ID if filter matches */
    FILTER_CONF_SET_PRIO,               /*!< Set priority if filter matches */
    FILTER_CONF_SET_PRIO_RX_FIFO0,      /*!< Set priority and store in FIFO 0 if filter matches */
    FILTER_CONF_SET_PRIO_RX_FIFO1,      /*!< Set priority and store in FIFO 1 if filter matches */
    FILTER_CONF_RX_BUFF                 /*!< Store into Rx Buffer or as debug message, configuration of filter type ignored */
} filter_element_config_t;

/*! @brief MCAN ID filter table structure
 * Implements : mcan_id_table_t_Class
 */
typedef struct {
    filter_type              filterType;         /*!< Filter Type element */
    filter_element_config_t  filterConfig;       /*!< Filter Configuration element*/
    bool                     isExtendedFrame;    /*!< Extended frame*/
    uint32_t                 id1;                /*!< ID1 filter element */
    uint32_t                 id2;                /*!< ID2 filter element */
} mcan_id_table_t;

/*! @brief MCAN configuration
 * Implements : mcan_user_config_t_Class
 */
typedef struct {
    uint8_t rx_num_mb;                              /*!< The Rx number of Message Buffers */
    uint8_t tx_num_mb;                                /*!< The Tx number of Message Buffers */
    uint8_t num_id_filters;                         /*!< The number of ID filters needed */
    mcan_rxfifo_mode_t rx_fifo_needed;              /*!< This controls whether the Rx FIFO feature is enabled or not.*/
    mcan_operation_modes_t mcanMode;                /*!< User configurable FlexCAN operation modes. */
    mcan_fd_payload_size_t payload;                 /*!< The payload size of the mailboxes specified in bytes. */
    bool fd_enable;                                 /*!< Enable/Disable the Flexible Data Rate feature. */
    mcan_time_segment_t bitrate;                    /*!< The bitrate used for standard frames or for the arbitration phase of FD frames. */
    mcan_time_segment_t bitrate_cbt;                /*!< The bitrate used for the data phase of FD frames. */
    rxfifo_conf_t * fifoConfigs;                    /*!< The configurations of FIFOs */
    mcan_id_table_t ** filterConfigs;               /*!< The configurations of ID Filters */
} mcan_user_config_t;

 
 /*! @brief MCAN message buffer structure
 * Implements : mcan_msgbuff_t_Class
 */
typedef struct {
    uint32_t  cs;                       /*!< Code and Status*/
    uint32_t msgId;                     /*!< Message Buffer ID*/
    uint8_t data[64];                   /*!< Data bytes of the FlexCAN message*/
    uint8_t dataLen;                    /*!< Length of data in bytes */
} mcan_msgbuff_t;
 
/*! @brief Information needed for internal handling of a given MB.
 * Implements : mcan_mb_handle_t_Class
 */
typedef struct {
    mcan_msgbuff_t *mb_message;            /*!< The FlexCAN MB structure */
    semaphore_t mbSema;                    /*!< Semaphore used for signaling completion of a blocking transfer */
    volatile mcan_mb_state_t state;        /*!< The state of the current MB (idle/Rx busy/Tx busy) */
    bool isBlocking;                       /*!< True if the transfer is blocking */
    bool isRemote;                         /*!< True if the frame is a remote frame */
    bool isExtended;                       /*!< True if the frame is a extended ID, used in case of RX MBs */
    uint8_t filterIdx;                     /*!< The filter Index, used in case of RX MBs */
} mcan_mb_handle_t;

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 * Implements : mcan_state_t_Class
 */
typedef struct MCANState {
    mcan_mb_handle_t mbs[FEATURE_MCAN_RX_MB_NUM + FEATURE_MCAN_TX_MB_NUM + FEATURE_MCAN_NO_RXFIFO];    /*!< Array containing information
                                                                                                       related to each MB and 1 MB for Each RxFIFO*/
    void (*callback)(uint8_t instance,
                     mcan_event_type_t eventType,
                     uint32_t buffIdx,
                     struct MCANState *driverState);        /*!< IRQ handler callback function. */
    void *callbackParam;                                    /*!< Parameter used to pass user data
                                                                    when invoking the callback
                                                                    function. */
    void (*error_callback)(uint8_t instance,
                           mcan_event_type_t eventType,
                           struct MCANState *driverState);  /*!< Error IRQ handler callback
                                                                    function. */
    void *errorCallbackParam;                               /*!< Parameter used to pass user data
                                                                    when invoking the error callback
                                                                    function. */
    uint8_t noOfMB;                                         /*!< No of MB configured to be used by the driver */
    uint8_t stdFIFO_filterIDs;                              /*!< No of standard FilterIDs used by FIFO1 and FIFO2 */
    uint8_t extFIFO_filterIDs;                              /*!< No of extened FilterIDs used by FIFO1 and FIFO2 */
} mcan_state_t;

/*! @brief MCAN Driver callback function type
 * Implements : mcan_callback_t_Class
 */
typedef void (*mcan_callback_t)(uint8_t instance, mcan_event_type_t eventType,
                                uint32_t buffIdx, mcan_state_t *mcanState);

/*! @brief MCAN Driver error callback function type
 * Implements : mcan_error_callback_t_Class
 */
typedef void (*mcan_error_callback_t)(uint8_t instance, mcan_event_type_t eventType,
                                      mcan_state_t *mcanState);

/*! @brief MCAN data info from user
 * Implements : mcan_data_info_t_Class
 */
typedef struct {
    mcan_msgbuff_id_type_t msg_id_type;     /*!< Type of message ID (standard or extended)*/
    uint32_t data_length;                   /*!< Length of Data in Bytes*/
    bool fd_enable;                         /*!< Enable or disable FD*/
    uint8_t fd_padding;                     /*!< Set a value for padding. It will be used when the data length code (DLC)
                                                 specifies a bigger payload size than data_length to fill the MB */
    bool enable_brs;                        /*!< Enable bit rate switch inside a CAN FD format frame*/
    bool is_remote;                         /*!< Specifies if the frame is standard or remote */
} mcan_data_info_t;

/*! @brief MCAN Errors Status
 * Implements : mcan_error_status_t_Class
 */
typedef struct {
    uint32_t errorCounter;          /* Error Counter Register */
    uint32_t protocolStatus;        /* Protocol Status Register */
} mcan_error_status_t;

 /*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the MCAN peripheral.
 *
 * This function initializes
 * @param   instance                   The MCAN instance number
 * @param   state                      Pointer to the FlexCAN driver state structure.
 * @param   data                       The MCAN platform data
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_ERROR if other error occurred
 */
status_t MCAN_DRV_Init(
       uint8_t instance,
       mcan_state_t *state,
       const mcan_user_config_t *data);

/*!
 * @brief MCAN transmit message buffer field configuration.
 *
 * @param   instance                   The MCAN instance number
 * @param   mb_idx                     Index of the message buffer
 * @param   tx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of the message buffer is invalid;
 *          STATUS_ERROR if other error occurred
 */
status_t MCAN_DRV_ConfigTxMb(
    uint8_t instance,
    uint8_t mb_idx,
    const mcan_data_info_t *tx_info,
    uint32_t msg_id);

/*!
 * @brief Sends a CAN frame using the specified message buffer.
 *
 * This function sends a CAN frame using a configured message buffer. The function
 * returns immediately. If a callback is installed, it will be invoked after
 * the frame was sent.
 *
 * @param   instance   The MCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   tx_info    Data info
 * @param   msg_id     ID of the message to transmit
 * @param   mb_data    Bytes of the CAN message.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy
 */
status_t MCAN_DRV_Send(
    uint8_t instance,
    uint8_t mb_idx,
    const mcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data);

/*!
 * @brief Sends a CAN frame using the specified message buffer, in a blocking manner.
 *
 * This function sends a CAN frame using a configured message buffer. The function
 * blocks until either the frame was sent, or the specified timeout expired.
 *
 * @param   instance   The MCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   tx_info    Data info
 * @param   msg_id     ID of the message to transmit
 * @param   mb_data    Bytes of the CAN message
 * @param   timeout_ms A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached
 */
status_t MCAN_DRV_SendBlocking(
    uint8_t instance,
    uint8_t mb_idx,
    const mcan_data_info_t *tx_info,
    uint32_t msg_id,
    const uint8_t *mb_data,
    uint32_t timeout_ms);

/*!
 * @brief MCAN receive message buffer field configuration
 *
 * @param   instance   The MCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   rx_info    Data info
 * @param   msg_id     ID of the message to transmit
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_ERROR if other error occurred
 */
status_t MCAN_DRV_ConfigRxMb(
    uint8_t instance,
    uint8_t mb_idx,
    const mcan_data_info_t *rx_info,
    uint32_t msg_id);

/*!
 * @brief Receives a CAN frame using the specified message buffer.
 *
 * This function receives a CAN frame using a configured message buffer. The function
 * returns immediately. If a callback is installed, it will be invoked after
 * the frame was received and read into the specified buffer.
 *
 * @param   instance   The MCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   data       The MCAN receive message buffer data.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy
 */
status_t MCAN_DRV_Receive(
    uint8_t instance,
    uint8_t mb_idx,
    mcan_msgbuff_t *data);

/*!
 * @brief Receives a CAN frame using the specified message buffer, in a blocking manner.
 *
 * This function receives a CAN frame using a configured message buffer. The function
 * blocks until either a frame was received, or the specified timeout expired.
 *
 * @param   instance   The MCAN instance number
 * @param   mb_idx     Index of the message buffer
 * @param   data       The CAN receive message buffer data.
 * @param   timeout_ms A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a message buffer is invalid;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached
 */
status_t MCAN_DRV_ReceiveBlocking(uint8_t instance,
                                  uint8_t mb_idx,
                                  mcan_msgbuff_t *data,
                                  uint32_t timeout_ms);

/*!
 * @brief Installs a callback function for the IRQ handler.
 *
 * @param instance The MCAN instance number.
 * @param callback The callback function.
 * @param callbackParam User parameter passed to the callback function through the state parameter.
 */
void MCAN_DRV_InstallEventCallback(uint8_t instance,
                                   mcan_callback_t callback,
                                   void *callbackParam);


/*!
 * @brief Installs a Error callback function for the IRQ handler.
 *
 * @param instance The MCAN instance number.
 * @param callback The callback function.
 * @param callbackParam User parameter passed to the callback function through the state parameter.
 */
void MCAN_DRV_InstallErrorCallback(uint8_t instance,
                                      mcan_error_callback_t callback,
                                      void *callbackParam);

/*!
 * @brief Shuts down a MCAN instance.
 *
 * @param   instance        A MCAN instance number
 * @return  STATUS_SUCCESS  if successful;
 *          STATUS_ERROR    if failed
 */
status_t MCAN_DRV_Deinit( uint8_t instance );

/*!
 * @brief Configure the FiltersID for the RxFIFOs
 *
 * @param instance          The MCAN instance number.
 * @param id_filter_table   Structure of pointers that contains the configurations of FilterIDs
 * @param noOfFilters       Number of filters in the structure
 * @return  STATUS_SUCCESS  if successful;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the space for strandard\extended filters exceed the space
 *                                       from initialization;
 *          STATUS_ERROR if the total number of filters exceed all reserved space for filters.
 *
 */
status_t MCAN_DRV_ConfigRxFifo(uint8_t instance,
                               mcan_id_table_t * const * id_filter_table,
                               uint8_t noOfFilters);

/*!
 * @brief Returns whether the previous MCAN transfer has finished.
 *
 * When performing an async transfer, call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success).
 *
 * @param instance  The MCAN instance number.
 * @param mb_idx    The index of the message buffer.
 * @param type      Transfer Type to get status (Rx\Tx\Fifo)
 * @return STATUS_SUCCESS if successful;
 *         STATUS_BUSY if a resource is busy;
 */
status_t MCAN_DRV_GetTransferStatus(uint8_t instance, mode_type_t type, uint8_t mb_idx);

/*!
 * @brief Receives a CAN frame using the message FIFO.
 *
 * This function receives a CAN frame using the Rx FIFO. The function returns
 * immediately. If a callback is installed, it will be invoked after the frame
 * was received and read into the specified buffer.
 *
 * @param   instance    The MCAN instance number.
 * @param   fifo        The FIFO0/FIFO1 selected
 * @param   data        The CAN receive message buffer data.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_ERROR if other error occurred
 */
status_t MCAN_DRV_RxFifo(uint8_t instance, 
                         uint8_t fifo,
                         mcan_msgbuff_t *data);

/*!
 * @brief Receives a CAN frame using the message FIFO, in a blocking manner.
 *
 * This function receives a CAN frame using the Rx FIFO. The function blocks until
 * either a frame was received, or the specified timeout expired.
 *
 * @param   instance    The MCAN instance number.
 * @param   fifo        The FIFO0/FIFO1 selected
 * @param   data        The CAN receive message buffer data.
 * @param   timeout_ms  A timeout for the transfer in milliseconds.
 * @return  STATUS_SUCCESS if successful;
 *          STATUS_BUSY if a resource is busy;
 *          STATUS_TIMEOUT if the timeout is reached;
 *          STATUS_ERROR if other error occurred
 */
status_t MCAN_DRV_RxFifoBlocking(uint8_t instance, 
                                 uint8_t fifo,
                                 mcan_msgbuff_t *data,
                                 uint32_t timeout_ms);

/*!
 * @brief Sets the MCAN nominal bit rate for standard frames.
 *
 * @param   instance    A MCAN instance number
 * @param   bitrate     A pointer to the CAN bit rate settings.
 * @return  STATUS_SUCCESS if successful.
 */
status_t MCAN_DRV_SetNominalBitrate(uint8_t instance, const mcan_time_segment_t *bitrate);

/*!
 * @brief Sets the MCAN data bit rate for extended FD frames.
 *
 * @param   instance    A MCAN instance number
 * @param   bitrate     A pointer to the CAN bit rate settings.
 * @return  STATUS_SUCCESS if successful.
 */
status_t MCAN_DRV_SetDataBitrate(uint8_t instance, const mcan_time_segment_t *bitrate);

/*!
 * @brief Abort a transfer.
 *
 * @param   instance    A MCAN instance number
 * @param   type        Transfer Type to get status (Rx\Tx\Fifo)
 * @param   mb_idx      The index of the message buffer.
 * @return  STATUS_SUCCESS if successful abort;
 *          STATUS_CAN_NO_TRANSFER_IN_PROGRESS if the transfer occurred.
 *          STATUS_ERROR if other error occurred.
 */
status_t MCAN_DRV_AbortTransfer(uint8_t instance, mode_type_t type, uint8_t mb_idx);

/*!
 * @brief This function sets a filter Mask for a specific filter.
 * Note ! This filter mask can be applied only for classical type filters and
 * for filters that are not part of an Rx Individual Buffer process.
 * @param   instance    A MCAN instance number
 * @param   type        Filter Type STD or EXT ID
 * @param   fl_idx      Filter index in the allocated space STD/EXT
 * @param   mask        Filter Mask
 * @return  STATUS_SUCCESS if successful set mask;
 *          STATUS_CAN_BUFF_OUT_OF_RANGE if the index of a filter is outside the reserved space.
 *          STATUS_ERROR if other error occurred.
 */
status_t MCAN_DRV_SetRxFifoFilterMask(uint8_t instance,
                                     mcan_msgbuff_id_type_t type,
                                     uint8_t fl_idx,
                                     uint32_t mask);
                                     
/*!
 * @brief  This function sets Global filter Configuration behavior
 * Store in RxFIFO or Reject nonAccepted frames, and automatic rejection of remote frames.
 * Note ! By Default the driver will set to reject all non match frames and to pass on filters
 * all remote frames.
 *
 * @param   instance        A MCAN instance number
 * @param   nonAcceptStd    Store in RxFIFO0\1 or reject Standard Frames
 * @param   nonAcceptExt    Store in RxFIFO0\1 or reject Extended Frames
 * @param   remoteStd       false -> Filter remote STD Frames
 *                          true  -> Reject all remote STD Frames
 * @param   remoteExt       false -> Filter remote EXT Frames
 *                          true  -> Reject all remote EXT Frames
 */
void MCAN_DRV_SetGlobalFilterConfig(uint8_t instance,
                                    nonMatchAccept_t nonAcceptStd,
                                    nonMatchAccept_t nonAcceptExt,
                                    bool remoteStd,
                                    bool remoteExt);

/*!
 * @brief Gets the default configuration structure
 *
 * This function gets the default configuration structure, with the following settings:
 * - 16 message buffers 8 Rx and 8 Tx
 * - flexible data rate disabled
 * - Rx FIFO disabled
 * - normal operation mode
 * - 8 byte payload size
 * - bitrate of 500 Kbit/s (computed for sample point = 87.5)
 *
 * @param[out] config The configuration structure
 */
void MCAN_DRV_GetDefaultConfig(mcan_user_config_t *config);

/*!
 * @brief Returns reported error conditions.
 *
 * Reports various error conditions detected in the reception and transmission of a CAN frame
 * and some general status of the device.
 *
 * @param      instance The MCAN instance number.
 * @param[out] errors   Pointer to the structure where errors will be returned.
 */
void MCAN_DRV_GetErrorStatus(uint8_t instance, mcan_error_status_t * errors);

/*!
 * @brief Enables/Disables the Transceiver Delay Compensation feature and sets
 * the Transceiver Delay Compensation Offset (offset value to be added to the
 * measured transceiver's loop delay in order to define the position of the
 * delayed comparison point when bit rate switching is active).
 *
 * @param   instance            A MCAN instance number
 * @param   enable              Enable/Disable Transceiver Delay Compensation
 * @param   offset              Transceiver Delay Compensation Offset (TDCO)
 * @param   filterWindowLength  The value for SSP position, the value must be greater than offset(TDCO)
 * @return  STATUS_SUCCESS if successful set TDCO feature;
 *          STATUS_ERROR if offset is greater or equal filterWindowLength.
 */
status_t MCAN_DRV_SetTDCOffset(uint8_t instance, bool enable, uint8_t offset, uint8_t filterWindowLength);

/*!
 * @brief Gets the MCAN bit rate for standard frames or the arbitration phase of FD frames.
 * @note The propSeg will always be 0, and the phaseSeg1 is actually the sum of phaseSeg1 and propSeg as 
 *       bitrate initialization values.
 *
 * @param       instance    A MCAN instance number
 * @param[out]  bitrate     A pointer to a variable for returning the MCAN bit rate settings
 */
void MCAN_DRV_GetBitrate(uint8_t instance, mcan_time_segment_t *bitrate);

/*!
 * @brief Gets the MCAN bit rate for the data phase of FD frames (BRS enabled).
 * @note The propSeg will always be 0, and the phaseSeg1 is actually the sum of phaseSeg1 and propSeg as 
 *       bitrate initialization values.
 *  
 * @param       instance    A MCAN instance number
 * @param[out]  bitrate     A pointer to a variable for returning the MCAN bit rate settings
 */
void MCAN_DRV_GetBitrateFD(uint8_t instance, mcan_time_segment_t *bitrate);

#if defined(__cplusplus)
}
#endif

#endif /* MCAN_DRIVER_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
 
