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
 * @file mcan_hw_access.h
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
 
#ifndef MCAN_HW_ACCESS_H
#define MCAN_HW_ACCESS_H

#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "mcan_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MB_RAM ((uint32_t *) FEATURE_MCAN_MESSAGE_RAM_START_ADR);
/*! @brief MCAN endianness handling */
#ifdef CORE_BIG_ENDIAN
    #define McanSwapBytesInWordIndex(index) (index)
    #define McanSwapBytesInWord(a, b) (b = a)
#endif
        /* a = stdIds, b = extIds, c = mb_size(words), d = TotalMbs(Rx+Tx+RxFifo),    */
#define McanCalcSpaceWords(a, b, c, d) (a + (2u*b) + ((2u+c)*d))
typedef enum{
    MCAN_LOOPBACK_INTERNAL,    /*!< Internal LoopBack Mode */
    MCAN_LOOPBACK_EXTERNAL     /*!< External LoopBack Mode */        
} loopback_mode;

typedef struct{
    uint32_t RAM[FEATURE_MCAN_RAM_MESSAGES_WRD<<1U];
} RAM_MB;

#define MCAN_RAM    ((RAM_MB *)FEATURE_MCAN_MESSAGE_RAM_START_ADR)

#define CAN_ID_EXT_MASK                          0x1FFFFFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT_WIDTH                         28

#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD_WIDTH                         11

#define CAN_ID_XTD_MASK                          0x40000000u
#define CAN_ID_XTD_SHIFT                         30
#define CAN_ID_XTD_WIDTH                         1

#define CAN_ID_RTR_MASK                          0x20000000u
#define CAN_ID_RTR_SHIFT                         29
#define CAN_ID_RTR_WIDTH                         1

#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC_WIDTH                         4

#define CAN_CS_FDF_MASK                          0x200000u
#define CAN_CS_FDF_SHIFT                         21
#define CAN_CS_FDF_WIDTH                         1

#define CAN_CS_BRS_MASK                          0x100000u
#define CAN_CS_BRS_SHIFT                         20
#define CAN_CS_BRS_WIDTH                         1

#define MCAN_STD_FILTER_ID_SFT_MASK              0xC0000000u
#define MCAN_STD_FILTER_ID_SFT_SHIFT             30
#define MCAN_STD_FILTER_ID_SFT_WIDTH             2

#define MCAN_STD_FILTER_ID_SFEC_MASK             0x38000000u
#define MCAN_STD_FILTER_ID_SFEC_SHIFT            27
#define MCAN_STD_FILTER_ID_SFEC_WIDTH            3

#define MCAN_STD_FILTER_ID_SFID1_MASK            0x7FF0000u
#define MCAN_STD_FILTER_ID_SFID1_SHIFT           16
#define MCAN_STD_FILTER_ID_SFID1_WIDTH           11

#define MCAN_STD_FILTER_ID_SFID2_MASK            0x7FFu
#define MCAN_STD_FILTER_ID_SFID2_SHIFT           0
#define MCAN_STD_FILTER_ID_SFID2_WIDTH           11

#define MCAN_EXT_FILTER_ID_EFEC_MASK             0xE0000000u
#define MCAN_EXT_FILTER_ID_EFEC_SHIFT            29
#define MCAN_EXT_FILTER_ID_EFEC_WIDTH            3

#define MCAN_EXT_FILTER_ID_EFID_MASK             0x1FFFFFFFu
#define MCAN_EXT_FILTER_ID_EFID_SHIFT            0
#define MCAN_EXT_FILTER_ID_EFID_WIDTH            28

#define MCAN_EXT_FILTER_ID_EFT_MASK              0xC0000000u
#define MCAN_EXT_FILTER_ID_EFT_SHIFT             30
#define MCAN_EXT_FILTER_ID_EFT_WIDTH             2

#define ERROR_DETECTION_MASK    (M_CAN_IR_PED_MASK | M_CAN_IR_PEA_MASK | M_CAN_IR_BO_MASK | M_CAN_IR_EW_MASK | \
                                 M_CAN_IR_EP_MASK | M_CAN_IR_ELO_MASK | M_CAN_IR_BEU_MASK | M_CAN_IR_BEC_MASK)
/*******************************************************************************
 * API
 ******************************************************************************/
 
#if defined(__cplusplus)
extern "C" {
#endif

/**
 *  @brief Disable the Rx Filter corresponding to Rx Message Buffer
 *
 *  @param base The MCAN base address
 *  @param state Pointer to the driver state structure
 *  @param vmb_idx Virtual message box index
 */
void MCAN_AbortRX(M_CAN_Type const * base,
                      mcan_state_t const * state,
                      uint8_t vmb_idx);

/**
 *  @brief Set Mode of operation of the MCAN
 *  
 *  @param base The MCAN base address
 *  @param mode Mode of operation to be set
 *  @return  STATUS_SUCCESS if successful;
 *           STATUS_ERROR if failed;
 */
status_t MCAN_SetOperationMode( M_CAN_Type * base,
                                mcan_operation_modes_t mode);

/**
 *  @brief Allow Configuration of the MCAN
 *  
 *  @param base The MCAN base address
 *  @return  STATUS_SUCCESS if successful;
 *           STATUS_ERROR if failed;
 */
status_t MCAN_AllowConfiguration(M_CAN_Type * base);

/**
 *  @brief Return the Payload Value set for Transmission\Reception
 *  
 *  @param base The MCAN base address
 *  @param type Rx Payload\ Tx Payload
 *  @return Value of payload 8/12/16/20/24/32/48/64
 */
uint8_t MCAN_GetPayloadSize(const M_CAN_Type * base, mode_type_t type);

/**
 *  @brief Return the Address of the last occupied
 *  standard Individual Filters
 *
 *  @param base The MCAN base address
 *  @return Address of the last standard ID filter.
 */
uint32_t MCAN_GetFilterSTDSpace(M_CAN_Type const * base);

/**
 *  @brief Return the Address of the last occupied
 *  extended Individual Filters
 *
 *  @param base The MCAN base address
 *  @return Address of the last extended ID filter.
 */
uint32_t MCAN_GetFilterEXTSpace(M_CAN_Type const * base);

/**
 *  @brief Return the Address of the last occupied
 *  MB element of the RxFIFO
 *
 *  @param base The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @return Address of the last MB ocupied by RxFIFO
 */
uint32_t MCAN_GetRxFIFOSpace(M_CAN_Type const * base, uint8_t fifoIdx);

/**
 *  @brief Return the Address of the last occupied
 *  Transmission Message Buffer
 *
 *  @param base The MCAN base address
 *  @return Address of the last Tx MB
 */
uint32_t MCAN_GetTxMBSpace(M_CAN_Type const * base);

/**
 *  @brief Return Payload Size based on the DLC code
 *
 *  @param dlcValue DLC Code
 *  @return Payload Value in Bytes
 */
uint8_t MCAN_ComputePayloadSize(uint8_t dlcValue);

/**
 *  @brief Return Tx Payload Size based on the Tx TBDS code
 *
 *  @param  Tx Buffer Data Field Size
 *  @return Payload Value in Bytes
 */
uint8_t MCAN_ComputeTxPayloadSize(uint8_t dlcValue);

/**
 *  @brief Return DLC Code based on the Payload Value
 *
 *  @param payloadSize Payload Value in Bytes
 *  @return DLC Code
 */
uint8_t MCAN_ComputeDLCValue(uint32_t payloadSize);

/**
 *  @brief Return if cancellation has occurred or not
 *
 *  @param base The MCAN base address
 *  @param msgBuffIdx No of MB in space type
 *  @return Cancellation status
 */
uint8_t MCAN_GetCancelReqStatus(M_CAN_Type const * base, uint32_t msgBuffIdx);

/**
 *  @brief Return the Virtual MailBox index
 *
 *  @param base The MCAN base address
 *  @param mode_type_t MB type
 *  @param msgBuffIdx No of MB in space type
 *  @return Virtual Mailbox Index
 */
uint8_t MCAN_GetVirtualMBIndex( M_CAN_Type const * base,
                                mode_type_t type,
                                uint32_t msgBuffIdx);

/**
 *  @brief Set Msg Buff Interrupt Enable/Disable
 *
 *  @param base The MCAN base address
 *  @param mode_type_t MB type
 *  @param msgBuffIdx No of MB in space type
 *  @param enable Enable/Disable Interrupt
 */
void MCAN_SetMsgBuffIntCmd( M_CAN_Type * base,
                                mode_type_t type,
                                uint32_t msgBuffIdx,
                                bool enable);

/**
 *  @brief Get Msg Buff Start Address
 *
 *  @param base The MCAN base address
 *  @param msgBuffIdx No of MB in space type
 *  @param mode_type_t MB type
 *  @return Returns the start of a MB area, based on its index.
 */                                
volatile uint32_t* MCAN_GetMsgBuffRegion( M_CAN_Type const * base,
                                          uint32_t msgBuffIdx,
                                          mode_type_t type);

/**
 *  @brief Set standard Id Filter
 *
 *  @param base The MCAN base address
 *  @param idFilter Pointer to id Filter configuration
 *  @param fl_idx No of Standard Id filter
 */
void MCAN_SetSTD_IdFilter(M_CAN_Type const * base, mcan_id_table_t const * idFilter, uint8_t fl_idx);

/**
 *  @brief Set extended Id Filter
 *
 *  @param base The MCAN base address
 *  @param idFilter Pointer to id Filter configuration
 *  @param fl_idx No of Standard Id filter
 */
void MCAN_SetEXT_IdFilter(M_CAN_Type const * base, mcan_id_table_t const * idFilter, uint8_t fl_idx);

/**
 *  @brief Check if a filter is set on the filter table in place
 *
 *  @param base The MCAN base address
 *  @param isExtended If the filter ID is Extended or Standard
 *  @param filterID Filter ID to check
 *  @param fl_idx No of Id filter entry in filter table to check
 *  @return STATUS_ERROR if a filter is in place for ID Filter
 *          STATUS_SUCCESS if no filter affects the ID checked
 */
status_t MCAN_CheckIdFilter(M_CAN_Type const * base, bool isExtended, uint32_t filterID,uint8_t filterIdx);

/**
 *  @brief Get extended Id Filter
 *
 *  @param base The MCAN base address
 *  @param idFilter Pointer to id Filter configuration to store
 *  @param fl_idx No of Extended Id filter entry in filter table
 */
void MCAN_GetEXT_IDFilter(M_CAN_Type const * base, mcan_id_table_t * idFilter, uint8_t fl_idx);

/**
 *  @brief Get Standard Id Filter
 *
 *  @param base The MCAN base address
 *  @param idFilter Pointer to id Filter configuration to store
 *  @param fl_idx No of Standard Id filter entry in filter table
 */
void MCAN_GetSTD_IDFilter(M_CAN_Type const * base, mcan_id_table_t * idFilter, uint8_t fl_idx);

/**
 *  @brief Get Instance No of MCAN
 *
 *  @param base The MCAN base address
 *  @return The instance No Of MCAN
 */
uint32_t MCAN_GetInstance(const M_CAN_Type * base);

/**
 *  @brief Set Test Mode for LoopBack
 *
 *  @param base The MCAN base address
 *  @param type LoopBack Type Internal\External
 *  @return STATUS_ERROR Setting failed
 *          STATUS_SUCCESS Configuration successfully set
 */
status_t MCAN_EnableTestMode(M_CAN_Type * base, loopback_mode type);

/**
 *  @brief Get Msg Buff Interrupt 
 *
 *  @param base The MCAN base address
 *  @param mode_type_t MB type
 *  @param msgBuffIdx No of MB in space type
 *  @return interrupt active flag 
 */ 
uint8_t MCAN_GetMsgBuffIntCmd(M_CAN_Type const * base,
                              mode_type_t type,
                              uint32_t msgBuffIdx);

/*!
 * @brief Gets the MCAN message buffer fields.
 *
 * @param   base               The MCAN base address
 * @param   msgBuffIdx       Index of the message buffer
 * @param   msgBuff          The fields of the message buffer
 */
void MCAN_GetMsgBuff( M_CAN_Type const * base,
                          uint32_t msgBuffIdx,
                          mode_type_t type,
                          mcan_msgbuff_t *msgBuff);

/**
 *  @brief Get interrupt line active for tested interrupt no
 *
 *  @param base The MCAN base address
 *  @param inter Interrupt bit position
 *  @return Line active
 */
uint8_t MCAN_GetInterruptLine(M_CAN_Type const * base, uint8_t inter);

/**
 *  @brief Enable LoopBack Test Mode
 *  
 *  @param base   The MCAN base address
 *  @param enable Enable\Disable LoopBack Mode
 */
static inline void MCAN_SetLoopBackMode(M_CAN_Type * base, bool enable)
{
    base->TEST = (base->TEST & ~M_CAN_TEST_LBCK_MASK) | M_CAN_TEST_LBCK(enable ? 1UL : 0UL);
}

/**
 * @brief Sets the MCAN time segments for setting up nominal bit rate.
 *
 * @param   base       The MCAN base address
 * @param   timeSeg    MCAN time segments, which need to be set for the nominal bit rate.
 */
static inline void MCAN_SetNominalTimeSegments(M_CAN_Type * base, const mcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    (base->NBTP) = ((base->NBTP) & ~(M_CAN_NBTP_NTSEG2_MASK | M_CAN_NBTP_NTSEG1_MASK |
                                        M_CAN_NBTP_NBRP_MASK | M_CAN_NBTP_NSJW_MASK));
    (base->NBTP) = ((base->NBTP) | (M_CAN_NBTP_NTSEG1((timeSeg->phaseSeg1 + timeSeg->propSeg)) |
                                    M_CAN_NBTP_NTSEG2(timeSeg->phaseSeg2) |
                                    M_CAN_NBTP_NBRP(timeSeg->preDivider) |
                                    M_CAN_NBTP_NSJW(timeSeg->rJumpwidth)));
}

/**
 * @brief Gets the MCAN time segments for nominal bit rate or arbitration for FD CAN frames.
 *
 * @param   base       The MCAN base address
 * @param   timeSeg    Pointer to MCAN time segments, where the values will be stored for the nominal bit rate.
 */
static inline void MCAN_GetNominalTimeSegments(M_CAN_Type const * base, mcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    timeSeg->propSeg = 0U;
    timeSeg->phaseSeg1 = (base->NBTP & M_CAN_NBTP_NTSEG1_MASK) >> M_CAN_NBTP_NTSEG1_SHIFT;
    timeSeg->phaseSeg2 = (base->NBTP & M_CAN_NBTP_NTSEG2_MASK) >> M_CAN_NBTP_NTSEG2_SHIFT;
    timeSeg->preDivider = (base->NBTP & M_CAN_NBTP_NBRP_MASK) >> M_CAN_NBTP_NBRP_SHIFT;
    timeSeg->rJumpwidth = (base->NBTP & M_CAN_NBTP_NSJW_MASK) >> M_CAN_NBTP_NSJW_SHIFT;
}

/**
 * @brief Sets the MCAN time segments for setting up data bit rate.
 *
 * @param   base       The MCAN base address
 * @param   timeSeg    MCAN time segments, which need to be set for the data bit rate.
 */
static inline void MCAN_SetDataTimeSegments(M_CAN_Type * base, const mcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);

    (base->DBTP) = ((base->DBTP) & ~(M_CAN_DBTP_DTSEG2_MASK | M_CAN_DBTP_DTSEG1_MASK |
                                     M_CAN_DBTP_DBRP_MASK | M_CAN_DBTP_DSJW_MASK));

    (base->DBTP) = ((base->DBTP) | (M_CAN_DBTP_DTSEG1((timeSeg->phaseSeg1 + timeSeg->propSeg)) |
                                    M_CAN_DBTP_DTSEG2(timeSeg->phaseSeg2) |
                                    M_CAN_DBTP_DBRP(timeSeg->preDivider) |
                                    M_CAN_DBTP_DSJW(timeSeg->rJumpwidth)));
}

/**
 * @brief Gets the MCAN time segments for data bit rate.
 *
 * @param   base       The MCAN base address
 * @param   timeSeg    Pointer to MCAN time segments, where the values will be stored for the data bit rate.
 */
static inline void MCAN_GetDataTimeSegments(M_CAN_Type const * base, mcan_time_segment_t *timeSeg)
{
    DEV_ASSERT(timeSeg != NULL);
    
    timeSeg->propSeg = 0U;
    timeSeg->phaseSeg1 = (base->DBTP & M_CAN_DBTP_DTSEG1_MASK) >> M_CAN_DBTP_DTSEG1_SHIFT;
    timeSeg->phaseSeg2 = (base->DBTP & M_CAN_DBTP_DTSEG2_MASK) >> M_CAN_DBTP_DTSEG2_SHIFT;
    timeSeg->preDivider = (base->DBTP & M_CAN_DBTP_DBRP_MASK) >> M_CAN_DBTP_DBRP_SHIFT;
    timeSeg->rJumpwidth = (base->DBTP & M_CAN_DBTP_DSJW_MASK) >> M_CAN_DBTP_DSJW_SHIFT;

}

/**
 * @brief Enable the MCAN Transceiver Delay Compensation feature.
 *
 * @param   base       The MCAN base address
 * @param   enable     True -> enable TDC, false -> disable TDC
 */
static inline void MCAN_EnableTDC(M_CAN_Type * base, bool enable)
{
    base->DBTP = ((base->DBTP & ~M_CAN_DBTP_TDC_MASK) | M_CAN_DBTP_TDC(enable ? 1UL : 0UL));
}

/**
 * @brief Enable the MCAN Transceiver Delay Compensation Value.
 *
 * @param   base   The MCAN base address
 * @param   value  Value for Transceiver Delay Compensation in time quanta
 */
static inline void MCAN_SetTDCOffset(M_CAN_Type * base, uint8_t value)
{
    base->TDCR = (base->TDCR & ~M_CAN_TDCR_TDCO_MASK) | M_CAN_TDCR_TDCO(value);
}

/**
 * @brief Enable the MCAN Transceiver Delay Compensation Filter Window Length
 *
 * @param   base   The MCAN base address
 * @param   value  Value for Transceiver Delay Compensation Filter Window in time quanta
 */
static inline void MCAN_SetTDCFilterWindow(M_CAN_Type * base, uint8_t value)
{
    base->TDCR = (base->TDCR & ~M_CAN_TDCR_TDCF_MASK) | M_CAN_TDCR_TDCF(value);
}

/**
 *  @brief Sets the MCAN behavior of FD frame format 
 *  
 *  @param base   The MCAN base address
 *  @param format true -> format Bosch CAN FD Specification V1.0
 *                false -> format ISO11898-1
 */
static inline void MCAN_SetNonIsoOperation(M_CAN_Type * base, bool format)
{   
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_NISO_MASK) | M_CAN_CCCR_NISO(format ? 1UL : 0UL);
}

/**
 *  @brief Enable Bit Rate Switch 
 *  
 *  @param base   The MCAN base address
 *  @param enable Disable\Enable BitRateSwitch
 */
static inline void MCAN_SetBitRateSwitch(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_BRSE_MASK) | M_CAN_CCCR_BRSE(enable ? 1UL : 0UL);
}

/**
 *  @brief Enable MCAN FD Operation 
 *  
 *  @param base   The MCAN base address
 *  @param enable Disable\Enable FD mode
 */
static inline void MCAN_SetFlexDataEnable(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_FDOE_MASK) | M_CAN_CCCR_FDOE(enable ? 1UL : 0UL);
}

/**
 *  @brief Get MCAN FD Operation is enabled
 *  
 *  @param base   The MCAN base address
 *  @return true -> Enabled FD Mode
 *          false -> Disabled FD Mode
 */
static inline bool MCAN_IsFDEnabled(const M_CAN_Type * base)
{
    return (((base->CCCR & M_CAN_CCCR_FDOE_MASK) ==  M_CAN_CCCR_FDOE_MASK) ? true : false);
}

/**
 *  @brief Get MCAN Bit Rate Switch Operation is enabled
 *
 *  @param base   The MCAN base address
 *  @return true -> Enabled BRS Mode
 *          false -> Disabled BRS Mode
 */
static inline bool MCAN_IsBRSEnabled(const M_CAN_Type * base)
{
    return (((base->CCCR & M_CAN_CCCR_BRSE_MASK) ==  M_CAN_CCCR_BRSE_MASK) ? true : false);
}

/**
 *  @brief Enable MCAN Test Mode
 *  
 *  @param base   The MCAN base address
 *  @param enable Disable\Enable Test mode
 */
static inline void MCAN_SetTestMode(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_TEST_MASK) | M_CAN_CCCR_TEST(enable ? 1UL : 0UL);
}

/**
 *  @brief Enable Bus Monitor
 *  
 *  @param base   The MCAN base address
 *  @param enable Disable\Enable BUS Monitor Mode
 */
static inline void MCAN_EnableBusMonitorMode(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_MON_MASK) | M_CAN_CCCR_MON(enable ? 1UL : 0UL);
}

/**
 *  @brief Set Configuration Change Enable 
 *  Provide access to the protected registers 
 *  
 *  @param base   The MCAN base address
 *  @param enable Disable\Enable Protected Mode
 */
static inline void MCAN_SetConfigChangeEnable(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_CCE_MASK) | M_CAN_CCCR_CCE(enable ? 1UL : 0UL);
}

/**
 *  @brief Set Initialization of MCAN module
 *  
 *  @param base   The MCAN base address
 *  @param enable false is MCAN in normal operation
 *                true is MCAN in Initialization Procedure 
 */
static inline void MCAN_SetInitialization(M_CAN_Type * base, bool enable)
{
    base->CCCR = (base->CCCR & ~M_CAN_CCCR_INIT_MASK) | M_CAN_CCCR_INIT(enable ? 1UL : 0UL);

    while ((base->CCCR & M_CAN_CCCR_INIT_MASK) != M_CAN_CCCR_INIT(enable ? 1UL : 0UL))
    {
        /* Wait the time domain to sync and check the value */
    }
}

/**
 *  @brief Check Initialization of MCAN module
 *
 *  @param base   The MCAN base address
 *  @return false is MCAN in normal operation
 *          true is MCAN in Initialization Procedure
 */
static inline bool MCAN_isInitilized(M_CAN_Type const * base)
{
    return (((base->CCCR & M_CAN_CCCR_INIT_MASK) >> M_CAN_CCCR_INIT_SHIFT) != 0U);
}

/**
 *  @brief Set Tx Individual Buffers Payload Size
 *
 *  @param base   The MCAN base address
 *  @param size   No of payload bytes supported by Tx Individual Buffers
 */
static inline void MCAN_SetTxElementdSize(M_CAN_Type * base, mcan_fd_payload_size_t size)
{
    base->TXESC = (base->TXESC & ~M_CAN_TXESC_TBDS_MASK) | M_CAN_TXESC_TBDS((uint32_t)size);
}

/**
 *  @brief Get Tx Individual Buffers Payload Size
 *
 *  @param base   The MCAN base address
 *  @return  code of No of payload bytes supported by Tx Individual Buffers
 */
static inline uint8_t MCAN_GetTxPayloadSize(M_CAN_Type const * base)
{
    return ((uint8_t)(base->TXESC & M_CAN_TXESC_TBDS_MASK) >> M_CAN_TXESC_TBDS_SHIFT);
}

/**
 *  @brief Set Rx Individual Buffers Payload Size
 *
 *  @param base   The MCAN base address
 *  @param size   No of payload bytes supported by Rx Individual Buffers
 */
static inline void MCAN_SetRxElementdSize(M_CAN_Type * base, mcan_fd_payload_size_t size)
{
    base->RXESC = (base->RXESC & ~M_CAN_RXESC_RBDS_MASK) | M_CAN_RXESC_RBDS((uint32_t)size);
}

/**
 *  @brief Set Acceptance for Non Matching Standard Frames
 *
 *  @param base   The MCAN base address
 *  @param value   Accepted in RXFIFO_0/RXFIFO_1 or Rejected
 */
static inline void MCAN_SetAcceptNonMatchStdFrame(M_CAN_Type * base, nonMatchAccept_t value)
{
    base->GFC = (base->GFC & ~M_CAN_GFC_ANFS_MASK) | M_CAN_GFC_ANFS(value);
}

/**
 *  @brief Set Acceptance for Non Matching Extended Frames
 *
 *  @param base   The MCAN base address
 *  @param value   Accepted in RXFIFO_0/RXFIFO_1 or Rejected
 */
static inline void MCAN_SetAcceptNonMatchExtFrame(M_CAN_Type * base, nonMatchAccept_t value)
{
    base->GFC = (base->GFC & ~M_CAN_GFC_ANFE_MASK) | M_CAN_GFC_ANFE(value);
}

/**
 *  @brief Set Rejection of Extended Remote Frames
 *
 *  @param base   The MCAN base address
 *  @param enable True will reject ext remote frames/ false will filter them
 */
static inline void MCAN_SetRejectRemoteExtFrame(M_CAN_Type * base, bool enable)
{
    base->GFC = (base->GFC & ~M_CAN_GFC_RRFE_MASK) | M_CAN_GFC_RRFE(enable ? 1UL : 0UL);
}

/**
 *  @brief Set Rejection of Standard Remote Frames
 *
 *  @param base   The MCAN base address
 *  @param enable True will reject std remote frames/ false will filter them
 */
static inline void MCAN_SetRejectRemoteStdFrame(M_CAN_Type * base, bool enable)
{
    base->GFC = (base->GFC & ~M_CAN_GFC_RRFS_MASK) | M_CAN_GFC_RRFS(enable ? 1UL : 0UL);
}

/**
 *  @brief Set FIFO Operation Mode Blocking or OverWrite
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param mode        Mode Blocking or OverWrite of FIFO operation
 */
static inline void MCAN_FIFOSetOpMode(M_CAN_Type * base, uint8_t fifoIdx, fifo_op_mode_t mode)
{

    if (fifoIdx == 0U)
    {
        base->RXF0C = (base->RXF0C & ~M_CAN_RXF0C_F0OM_MASK) | M_CAN_RXF0C_F0OM(mode);
    }
    else
    {
        base->RXF1C = (base->RXF1C & ~M_CAN_RXF1C_F1OM_MASK) | M_CAN_RXF1C_F1OM(mode);
    }
}

/**
 *  @brief Set FIFO WaterMark Level
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param level    WaterMark Level
 */
static inline void MCAN_FIFOSetWaterMark(M_CAN_Type * base, uint8_t fifoIdx, uint8_t level)
{
    if (fifoIdx == 0U)
    {
        base->RXF0C = (base->RXF0C & ~M_CAN_RXF0C_F0WM_MASK) | M_CAN_RXF0C_F0WM(level);
    }
    else
    {
        base->RXF1C = (base->RXF1C & ~M_CAN_RXF1C_F1WM_MASK) | M_CAN_RXF1C_F1WM(level);
    }
}

/**
 *  @brief Set FIFO Acknowledge Level
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param level    Acknowledge Index
 */
static inline void MCAN_FIFOSetAcknowledge(M_CAN_Type * base, uint8_t fifoIdx, uint8_t level)
{
    if (fifoIdx == 0U)
    {
        base->RXF0A = M_CAN_RXF0A_F0AI(level);
    }
    else
    {
        base->RXF1A = M_CAN_RXF1A_F1AI(level);
    }
}

/**
 *  @brief Get FIFO Index
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @return         No of FIFO elements pulled from fifo
 */
static inline uint8_t MCAN_FIFOGetIndex(M_CAN_Type const * base, uint8_t fifoIdx)
{
    uint8_t value = 0U;
    if (fifoIdx == 0U)
    {
        value = ((uint8_t)((base->RXF0S & M_CAN_RXF0S_F0GI_MASK) >> M_CAN_RXF0S_F0GI_SHIFT));
    }
    else
    {
        value = ((uint8_t)((base->RXF1S & M_CAN_RXF1S_F1GI_MASK) >> M_CAN_RXF1S_F1GI_SHIFT));
    }
    return value;
}

/**
 *  @brief Get FIFO Level
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @return         No of elements in FIFO
 */
static inline uint8_t MCAN_FIFOGetLevel(M_CAN_Type const * base, uint8_t fifoIdx)
{
    uint8_t value = 0U;
    if (fifoIdx == 0U)
    {
        value = ((uint8_t)((base->RXF0S & M_CAN_RXF0S_F0FL_MASK) >> M_CAN_RXF0S_F0FL_SHIFT));
    }
    else
    {
        value = ((uint8_t)((base->RXF1S & M_CAN_RXF1S_F1FL_MASK) >> M_CAN_RXF1S_F1FL_SHIFT));
    }

    return value;
}

/**
 *  @brief Set FIFO Depth Level
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param size        No of FIFO elements
 */
static inline void MCAN_FIFOSetSize(M_CAN_Type * base, uint8_t fifoIdx, uint8_t size)
{
    if (fifoIdx == 0U)
    {
        base->RXF0C = (base->RXF0C & ~M_CAN_RXF0C_F0S_MASK) | M_CAN_RXF0C_F0S(size);
    }
    else
    {
        base->RXF1C = (base->RXF1C & ~M_CAN_RXF1C_F1S_MASK) | M_CAN_RXF1C_F1S(size);
    }
}

/**
 *  @brief Get FIFO Depth Level
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @return         No of FIFO elements
 */
static inline uint8_t MCAN_FIFOGetSize(M_CAN_Type const * base, uint8_t fifoIdx)
{
    uint8_t value = 0U;
    if (fifoIdx == 0U)
    {
        value = ((uint8_t)((base->RXF0C & M_CAN_RXF0C_F0S_MASK) >> M_CAN_RXF0C_F0S_SHIFT));
    }
    else
    {
        value = ((uint8_t)((base->RXF1C & M_CAN_RXF1C_F1S_MASK) >> M_CAN_RXF1C_F1S_SHIFT));
    }

    return value;
}

/**
 *  @brief Set FIFO Start Address
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param address  Fifo Start Address
 */
static inline void MCAN_FIFOSetStartAddress(M_CAN_Type * base, uint8_t fifoIdx, uint32_t address)
{
    if (fifoIdx == 0U)
    {
        base->RXF0C = (base->RXF0C & ~M_CAN_RXF0C_F0SA_MASK) | (address&M_CAN_RXF0C_F0SA_MASK);
    }
    else
    {
        base->RXF1C = (base->RXF1C & ~M_CAN_RXF1C_F1SA_MASK) | (address&M_CAN_RXF0C_F0SA_MASK);
    }
}

/**
 *  @brief Set FIFO Payload Size
 *
 *  @param base     The MCAN base address
 *  @param fifoIdx  Fifo Idx to apply operation mode 0 or 1
 *  @param address  Fifo Payload Size
 */
static inline void MCAN_FIFOSetPayloadSize(M_CAN_Type * base, uint8_t fifoIdx, mcan_fd_payload_size_t size)
{
    if (fifoIdx == 0U)
    {
        base->RXESC = (base->RXESC & ~M_CAN_RXESC_F0DS_MASK) | M_CAN_RXESC_F0DS(size);
    }
    else
    {
        base->RXESC = (base->RXESC & ~M_CAN_RXESC_F1DS_MASK) | M_CAN_RXESC_F1DS(size);
    }
}

/**
 *  @brief Set Standard Filter Size
 *
 *  @param base   The MCAN base address
 *  @param size   No of individual standard filters
 */
static inline void MCAN_SetStdIDFilterSize(M_CAN_Type * base, uint8_t size)
{
    base->SIDFC = (base->SIDFC & ~M_CAN_SIDFC_LSS_MASK) | M_CAN_SIDFC_LSS(size);
}

/**
 *  @brief Get Standard Filter Size
 *
 *  @param base   The MCAN base address
 *  @return   No of individual standard filters configured
 */
static inline uint8_t MCAN_GetStdIDFilterSize(M_CAN_Type const * base)
{
    return (uint8_t)((base->SIDFC&M_CAN_SIDFC_LSS_MASK)>>M_CAN_SIDFC_LSS_SHIFT);
}

/**
 *  @brief Set Standard Filter Start Address
 *
 *  @param base     The MCAN base address
 *  @param address  Start Address
 */
static inline void MCAN_SetStdIDFilterAddress(M_CAN_Type * base, uint32_t address)
{
    base->SIDFC = (base->SIDFC & ~M_CAN_SIDFC_FLSSA_MASK) | (address&M_CAN_SIDFC_FLSSA_MASK);
}

/**
 *  @brief Set Extended Filter Size
 *
 *  @param base   The MCAN base address
 *  @param size   No of individual extended filters
 */
static inline void MCAN_SetExtIDFilterSize(M_CAN_Type * base, uint8_t size)
{
    base->XIDFC = (base->XIDFC & ~M_CAN_XIDFC_LSE_MASK) | M_CAN_XIDFC_LSE(size);
}

/**
 *  @brief Get Extended Filter Size
 *
 *  @param base   The MCAN base address
 *  @return   No of individual extended filters configured
 */
static inline uint8_t MCAN_GetExtIDFilterSize(M_CAN_Type const * base)
{
    return (uint8_t)((base->XIDFC&M_CAN_XIDFC_LSE_MASK)>>M_CAN_XIDFC_LSE_SHIFT);
}

/**
 *  @brief Set Extended Filter Start Address
 *
 *  @param base     The MCAN base address
 *  @param address  Start Address
 */
static inline void MCAN_SetExtIDFilterAddress(M_CAN_Type * base, uint32_t address)
{
    base->XIDFC = (base->XIDFC & ~M_CAN_XIDFC_FLESA_MASK) | (address&M_CAN_XIDFC_FLESA_MASK);
}

/**
 *  @brief Set Tx Individual Buffers Start Address
 *
 *  @param base     The MCAN base address
 *  @param address  Start Address
 */
static inline void MCAN_SetTxBuffAddress(M_CAN_Type * base, uint32_t address)
{
    base->TXBC = (base->TXBC & ~M_CAN_TXBC_TBSA_MASK) | (address&M_CAN_TXBC_TBSA_MASK);
}

/**
 *  @brief Set number Tx Individual Buffers Start Address
 *
 *  @param base   The MCAN base address
 *  @param size   No of individual TX buffers
 */
static inline void MCAN_SetTxBuffSize(M_CAN_Type * base, uint8_t size)
{
    base->TXBC = (base->TXBC & ~M_CAN_TXBC_NDTB_MASK) | M_CAN_TXBC_NDTB(size);
}

/**
 *  @brief Set Tx Individual Buffers Start Request
 *
 *  @param base      The MCAN base address
 *  @param buff_idx  Buffer Index to Start
 */
static inline void MCAN_SetTxBuffAddReq(M_CAN_Type * base, uint8_t buff_idx)
{
    base->TXBAR = ((base->TXBAR) | (1UL << (buff_idx%32U)));
}

/**
 *  @brief Get number of Tx Individual Buffers
 *
 *  @param base   The MCAN base address
 *  @return  No of individual TX buffers
 */
static inline uint8_t MCAN_GetTxBuffSize(M_CAN_Type const * base)
{
    return (uint8_t)((base->TXBC & M_CAN_TXBC_NDTB_MASK) >> M_CAN_TXBC_NDTB_SHIFT);
}

/**
 *  @brief Set number Rx Individual Buffers Start Address
 *
 *  @param base     The MCAN base address
 *  @param address  Start Address
 */
static inline void MCAN_SetRxBuffAddress(M_CAN_Type * base, uint32_t address)
{
    base->RXBC = ((base->RXBC & ~M_CAN_RXBC_RBSA_MASK) | (address&M_CAN_RXBC_RBSA_MASK));
}

/**
 *  @brief Get Status of Interrupts
 *
 *  @param base   The MCAN base address
 *  @return  Interrupts Status
 */
static inline uint32_t MCAN_GetInterruptStatus(M_CAN_Type const * base)
{
    uint32_t aux = base->IE;
    return (base->IR & aux);
}

/**
 *  @brief Set Interrupt Enable\Disable Interrupt
 *
 *  @param base   The MCAN base address
 *  @param inter  Bit position assigned for interrupt
 *  @param line   Line 0 or Line 1
 */
static inline void MCAN_SetInterruptLineSelect(M_CAN_Type * base, uint32_t inter, uint8_t line)
{
    /* Enable the corresponding Interrupt */
    DEV_ASSERT(line <= 1U);
    uint32_t temp = (uint32_t)line << (inter % 32U);
    base->ILS = (base->ILS & ~(1UL << (inter % 32U))) | temp;
}

/**
 *  @brief Set Interrupt Enable\Disable Interrupt
 *
 *  @param base   The MCAN base address
 *  @param inter  Bit position assigned for interrupt
 *  @param enable Enable\Disable
 */
static inline void MCAN_SetInterruptEnable(M_CAN_Type * base, uint8_t inter, bool enable)
{
    /* Enable the corresponding Interrupt */
    uint32_t temp = 1UL << (inter % 32U);
    if (enable)
    {
        (base->IE) = ((base ->IE) | (temp));
    }
    else
    {
        (base->IE) = ((base->IE) & ~(temp));
    }
}

/**
 *  @brief Set Interrupt Line
 *
 *  @param base      The MCAN base address
 *  @param interrupt No assigned
 *  @param enable    Enable\Disable
 */
static inline void MCAN_SetInterruptLine(M_CAN_Type * base, uint8_t line, bool enable)
{
    /* Enable the corresponding Interrupt */
    uint32_t temp = 1UL << (line % 2U);
    if (line < 2U)
    {
        if (enable)
        {
            (base->ILE) = ((base ->ILE) | (temp));
        }
        else
        {
            (base->ILE) = ((base->ILE) & ~(temp));
        }
    }
}

/**
 *  @brief Set Cancellation Request
 *
 *  @param base   The MCAN base address
 *  @param mb_idx No of Tx MB to assign cancel request
 */  
static inline void MCAN_SetCancelRequest(M_CAN_Type * base, uint32_t mb_idx)
{
    base->TXBCR |= ((uint32_t)1U << (mb_idx % 32U));
}

/**
 *  @brief Get Request Pending Operation
 *
 *  @param base   The MCAN base address
 *  @param mb_idx No of Tx MB to check request
 *  @return  true if operations still in progress
 *           false operation finished
 */
static inline bool MCAN_GetBuffRequestPendig(M_CAN_Type const * base, uint32_t mb_idx)
{
    return (((base->TXBRP & ((uint32_t)1U << (mb_idx % 32U))) >> (mb_idx % 32U)) != 0U);
}

/**
 *  @brief Get Transmission Occurred Status
 *
 *  @param base   The MCAN base address
 *  @param mb_idx No of Tx MB to check request
 *  @return  true if transmission was successful
 *           false transmission not occurred
 */
static inline bool MCAN_GetTransmissionOccurred(M_CAN_Type const * base, uint32_t mb_idx)
{
    uint32_t mask = (1UL << (mb_idx%32U));
    return (((base->TXBTO & mask) >> (mb_idx%32U)) != 0U);
}

/**
 *  @brief Set Cancellation Interrupt
 *
 *  @param base   The MCAN base address
 *  @param inter  No of MB cancel interrupt assigned
 *  @param enable Enable\Disable
 */
static inline void MCAN_SetCancelInterrupt(M_CAN_Type * base, uint32_t inter, bool enable)
{
    /* Enable the corresponding Interrupt */
    uint32_t temp = 1UL << (inter % 32U);
    if (enable)
    {
        (base->TXBCIE) = ((base ->TXBCIE) | (temp));
    }
    else
    {
        (base->TXBCIE) = ((base->TXBCIE) & ~(temp));
    }
}

/**
 *  @brief Clear All pending Flags for received Data
 *
 *  @param base   The MCAN base address
 */
static inline void MCAN_ClearAllReceivedFlags(M_CAN_Type * base)
{
    base->NDAT1 = 0xFFFFFFFFU;
    base->NDAT2 = 0xFFFFFFFFU;
}

/**
 *  @brief Get Error Counter Status
 *
 *  @param base   The MCAN base address
 *  @return  Value of register
 */
static inline uint32_t MCAN_GetErrorCounter(M_CAN_Type const * base)
{
    return base->ECR;
}

/**
 *  @brief Get Protocol Error Status
 *
 *  @param base   The MCAN base address
 *  @return  Value of register
 */
static inline uint32_t MCAN_GetProtocolStatus(M_CAN_Type const * base)
{
    return base->PSR;
}
#if defined(__cplusplus)
}
#endif

#endif /* MCAN_HW_ACCESS_H */ 
/*******************************************************************************
 * EOF
 ******************************************************************************/
 
