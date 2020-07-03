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
 * @file mcan_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, cast performed between a pointer to
 * object type and a pointer to a different object type
 * The cast is used for casting a bytes buffer into an words buffer in order to
 * optimize copying data to/from the message buffer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, conversion between a pointer and integer
 * type.
 * The cast is needed to obtain an address for the message buffer.
 * The cast is required as CAN instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from pointer to unsigned long
 * The cast is needed to obtain an address for the message buffer
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, Variable not defined with external linkage
 * The variables are defined in the driver header file to make transition to other
 * platforms easier.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * The code is not dynamically linked. An absolute stack address is obtained when
 * taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 */
 
#include "mcan_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* CAN FD extended data length DLC encoding */
#define CAN_DLC_VALUE_12_BYTES                   9U
#define CAN_DLC_VALUE_16_BYTES                   10U
#define CAN_DLC_VALUE_20_BYTES                   11U
#define CAN_DLC_VALUE_24_BYTES                   12U
#define CAN_DLC_VALUE_32_BYTES                   13U
#define CAN_DLC_VALUE_48_BYTES                   14U
#define CAN_DLC_VALUE_64_BYTES                   15U

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static filter_element_config_t MCAN_GetFilterConfig(mcan_msgbuff_id_type_t type,
                                                    uint32_t value);

static filter_type MCAN_GetFilterType(mcan_msgbuff_id_type_t type,
                                      uint32_t value);

/*******************************************************************************
 * Code
 ******************************************************************************/
 

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_ComputePayloadSize
 * Description   : Computes the maximum payload size (in bytes), given a DLC
 * field value.
 *
 *END**************************************************************************/
uint8_t MCAN_ComputePayloadSize(uint8_t dlcValue)
{
    uint8_t ret=0U;

    if (dlcValue <= 8U)
    {
        ret = dlcValue;
    }
    else
    {
        switch (dlcValue) {
            case CAN_DLC_VALUE_12_BYTES:
                ret = 12U;
                break;
            case CAN_DLC_VALUE_16_BYTES:
                ret = 16U;
                break;
            case CAN_DLC_VALUE_20_BYTES:
                ret = 20U;
                break;
            case CAN_DLC_VALUE_24_BYTES:
                ret = 24U;
                break;
            case CAN_DLC_VALUE_32_BYTES:
                ret = 32U;
                break;
            case CAN_DLC_VALUE_48_BYTES:
                ret = 48U;
                break;
            case CAN_DLC_VALUE_64_BYTES:
                ret = 64U;
                break;
            default:
                /* The argument is not a valid DLC size */
                break;
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_ComputeTxPayloadSize
 * Description   : Computes the maximum payload size (in bytes), given a Tx Buffer
 * Data Field Size field value.
 *
 *END**************************************************************************/
uint8_t MCAN_ComputeTxPayloadSize(uint8_t dlcValue)
{
    uint8_t ret=0xFFU;

    switch (dlcValue)
    {
        case (uint8_t)MCAN_PAYLOAD_SIZE_8:
            ret = 8U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_12:
            ret = 12U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_16:
            ret = 16U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_20:
            ret = 20U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_24:
            ret = 24U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_32:
            ret = 32U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_48:
            ret = 48U;
            break;
        case (uint8_t)MCAN_PAYLOAD_SIZE_64:
            ret = 64U;
            break;
        default:
            /* The argument is not a valid DLC size */
            break;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: MCAN_ComputeDLCValue
 * Description  : Computes the DLC field value, given a payload size (in bytes).
 *
 *END**************************************************************************/
uint8_t MCAN_ComputeDLCValue(uint32_t payloadSize)
{
    uint32_t ret = 0xFFU;                     /* 0,  1,  2,  3,  4,  5,  6,  7,  8, */
    static const uint8_t payload_code[65] = { 0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
                                    /* 9 to 12 payload have DLC Code 12 Bytes */
                                CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES,
                                    /* 13 to 16 payload have DLC Code 16 Bytes */
                                CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES,
                                    /* 17 to 20 payload have DLC Code 20 Bytes */
                                CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES,
                                    /* 21 to 24 payload have DLC Code 24 Bytes */
                                CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES,
                                    /* 25 to 32 payload have DLC Code 32 Bytes */
                                CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES,
                                CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES,
                                    /* 33 to 48 payload have DLC Code 48 Bytes */
                                CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
                                CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
                                CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
                                CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
                                    /* 49 to 64 payload have DLC Code 64 Bytes */
                                CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
                                CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
                                CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
                                CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES };

    if (payloadSize <= 64U)
    {
        ret = payload_code[payloadSize];
    }
    else
    {
        /* The argument is not a valid payload size will return 0xFF*/
    }

    return (uint8_t)ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: MCAN_GetFilterSTDSpace
 * Description  : Return the Address of the last occupied standard Individual Filter
 *
 *END**************************************************************************/
uint32_t MCAN_GetFilterSTDSpace(M_CAN_Type const *base)
{
    uint32_t value=0U;
    uint32_t size;
    size = MCAN_GetStdIDFilterSize(base);
    /* Address of the last ocupied of the STD ID Filters */
    value = (base->SIDFC & M_CAN_SIDFC_FLSSA_MASK)+(size << 2U);
    return value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: MCAN_GetFilterEXTSpace
 * Description  : Return the Address of the last occupied extended Individual Filter
 *
 *END**************************************************************************/
uint32_t MCAN_GetFilterEXTSpace(M_CAN_Type const *base)
{
    uint32_t value=0U;
    uint32_t size;
    size = MCAN_GetExtIDFilterSize(base);
    value = (base->XIDFC & M_CAN_XIDFC_FLESA_MASK)+(size << 3U);
    return value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: MCAN_GetRxFIFOSpace
 * Description  : Return the Address of the last occupied MB element of the RxFIFO
 *
 *END**************************************************************************/
uint32_t MCAN_GetRxFIFOSpace(M_CAN_Type const *base, uint8_t fifoIdx)
{
    uint32_t value=0U;
    uint32_t arbitration_field_size = 8U;
    uint32_t size =  MCAN_FIFOGetSize(base, fifoIdx);

    if (fifoIdx == 0U)
    {
        uint32_t paysize = MCAN_GetPayloadSize(base, MCAN_RX_FIFO0);
        value = (base->RXF0C & M_CAN_RXF0C_F0SA_MASK) + (size * (paysize+arbitration_field_size));
    }
    else if (fifoIdx == 1U)
    {
        uint32_t paysize = MCAN_GetPayloadSize(base, MCAN_RX_FIFO1);
        value = (base->RXF1C & M_CAN_RXF1C_F1SA_MASK) + (size * (paysize+arbitration_field_size));
    }
    else
    { /* Do Nothing */}
    return value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name: MCAN_GetTxMBSpace
 * Description  : Return the Address of the last occupied Transmission Message Buffer
 *
 *END**************************************************************************/
uint32_t MCAN_GetTxMBSpace(M_CAN_Type const *base)
{
    uint32_t value = 0U;
    uint32_t size=MCAN_GetTxBuffSize(base);
    uint32_t arbitration_field_size = 8U;
    uint32_t paysize = MCAN_GetPayloadSize(base, MCAN_TX_BUFF);
    value = (base->TXBC & M_CAN_TXBC_TBSA_MASK) + (size*(paysize+arbitration_field_size));
    return value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetMsgBuffRegion
 * Description   : Returns the start of a MB area, based on its index.
 *
 *END**************************************************************************/
volatile uint32_t* MCAN_GetMsgBuffRegion(M_CAN_Type const *base,
                                         uint32_t msgBuffIdx,
                                         mode_type_t type)
{
    uint32_t *RAM = NULL;
    uint32_t address = 0U;
    uint8_t payload_size = MCAN_GetPayloadSize(base, type);

    uint8_t arbitration_field_size = 8U;

    uint8_t mb_size = (uint8_t)(payload_size + arbitration_field_size);

    if (type == MCAN_TX_BUFF)
    {
        address = (FEATURE_MCAN_MESSAGE_RAM_START_ADR | (base->TXBC&M_CAN_TXBC_TBSA_MASK));
        RAM = (uint32_t *)address;
    }
    else if (type == MCAN_RX_BUFF)
    {
        address = (FEATURE_MCAN_MESSAGE_RAM_START_ADR | (base->RXBC&M_CAN_RXBC_RBSA_MASK));
        RAM = (uint32_t *)address;
    }
    else if (type == MCAN_RX_FIFO0)
    {
        address = (FEATURE_MCAN_MESSAGE_RAM_START_ADR | (base->RXF0C&M_CAN_RXF0C_F0SA_MASK));
        RAM = (uint32_t *)address;
    }
    else if (type == MCAN_RX_FIFO1)
    {
        address = (FEATURE_MCAN_MESSAGE_RAM_START_ADR | (base->RXF1C&M_CAN_RXF1C_F1SA_MASK));
        RAM = (uint32_t *)address;
    }
    else
    {
        /* Misra Require Rule 15.7 */
    }
    /* Multiply the MB index by the MB size (in words) */
    uint32_t mb_index = (msgBuffIdx* ((uint32_t)mb_size >> 2U));

    return (&RAM[mb_index]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetPayloadSize
 * Description   : Returns the payload size of the MBs (in bytes).
 *
 *END**************************************************************************/
uint8_t MCAN_GetPayloadSize(const M_CAN_Type *base, mode_type_t type)
{
    uint32_t payloadSize = 8U;
    uint32_t payloadCode = 0U;
    /* The standard payload size is 8 bytes */
    if (!MCAN_IsFDEnabled(base))
    {
        payloadSize = 8U;
    }
    else
    {
        if (type == MCAN_TX_BUFF)
        {
            payloadCode = ((base->TXESC & M_CAN_TXESC_TBDS_MASK) >>  M_CAN_TXESC_TBDS_SHIFT);
        }
        if (type == MCAN_RX_BUFF)
        {
            payloadCode = ((base->RXESC & M_CAN_RXESC_RBDS_MASK) >>  M_CAN_RXESC_RBDS_SHIFT);
        }
        if (type == MCAN_RX_FIFO0)
        {
            payloadCode = ((base->RXESC & M_CAN_RXESC_F0DS_MASK) >>  M_CAN_RXESC_F0DS_SHIFT);
        }
        if (type == MCAN_RX_FIFO1)
        {
            payloadCode = ((base->RXESC & M_CAN_RXESC_F1DS_MASK) >>  M_CAN_RXESC_F1DS_SHIFT);
        }
        switch (payloadCode)
        {
           case    (uint32_t)MCAN_PAYLOAD_SIZE_64:
               payloadSize = 64U;
               break;
           case    (uint32_t)MCAN_PAYLOAD_SIZE_48:
               payloadSize = 48U;
               break;
           case    (uint32_t)MCAN_PAYLOAD_SIZE_32:
               payloadSize = 32U;
               break;
           default:
               payloadSize = ((payloadCode<<2U) + 8U);
               break;
        }
    }

    return (uint8_t)payloadSize;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_AllowConfiguration
 * Description   : Returns the status of protection write of configuration registers\
 *
 *END***************************************************************************/
status_t MCAN_AllowConfiguration(M_CAN_Type *base)
{
   if (MCAN_isInitilized(base))
   {
       MCAN_SetConfigChangeEnable(base, true);
       /* Wait until configuration is allowed */
       while ((base->CCCR & (M_CAN_CCCR_INIT_MASK | M_CAN_CCCR_CCE_MASK)) != (M_CAN_CCCR_INIT_MASK | M_CAN_CCCR_CCE_MASK))
       {
           /* Do Nothing */
       }
       return STATUS_SUCCESS;
   }
   else
   {
       return STATUS_ERROR;
   }
}

/*FUNCTION**********************************************************************
*
* Function Name : MCAN_EnableTestMode
* Description   : Returns the status of initialization.
*
*END***************************************************************************/
status_t MCAN_EnableTestMode(M_CAN_Type *base, loopback_mode type)
{
   status_t status;
   status = MCAN_AllowConfiguration(base);
   if(status == STATUS_SUCCESS)
   {    
        if ((base->CCCR&M_CAN_CCCR_TEST_MASK) != M_CAN_CCCR_TEST_MASK)
        {  
            MCAN_SetTestMode(base, true);      
        }
        if (type == MCAN_LOOPBACK_INTERNAL)
        {
            MCAN_SetLoopBackMode(base, true);
            MCAN_EnableBusMonitorMode(base, true);
        }
        if (type == MCAN_LOOPBACK_EXTERNAL)
        {
            MCAN_SetLoopBackMode(base, true);
            MCAN_EnableBusMonitorMode(base, false);
        }
   }
   return status;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetOperationMode
 * Description   : Enable a MCAN operation mode.
 * This function will enable one of the modes listed in mcan_operation_modes_t.
 *
 *END**************************************************************************/
status_t MCAN_SetOperationMode( M_CAN_Type *base,
                                mcan_operation_modes_t mode)
{
    status_t status = MCAN_AllowConfiguration(base);
    if (STATUS_SUCCESS == status)
    {    
        if (mode == MCAN_LOOPBACK_MODE)
        {
            status = MCAN_EnableTestMode(base, MCAN_LOOPBACK_INTERNAL);
        }
        else if (mode == MCAN_NORMAL_MODE)
        {
            MCAN_SetLoopBackMode(base, false);
            MCAN_EnableBusMonitorMode(base, false);
        }
        else if (mode == MCAN_LISTEN_ONLY_MODE)
        {
            MCAN_SetLoopBackMode(base, false);
            MCAN_EnableBusMonitorMode(base, true);
        }
        else
        {
            /* Misra Require Rule 15.7 Else If */
        }
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetMsgBuffIntCmd
 * Description   : Enable/Disable the corresponding Message Buffer interrupt.
 *
 *END**************************************************************************/
void MCAN_SetMsgBuffIntCmd( M_CAN_Type *base,
                                mode_type_t type,
                                uint32_t msgBuffIdx,
                                bool enable)
{
    uint32_t temp;

    switch(type)
    {
    case MCAN_TX_BUFF :
        /* Enable the corresponding message buffer Interrupt */
        temp = 1UL << (msgBuffIdx % 32U);
        if (msgBuffIdx  < 32U)
        {
            if (enable)
            {
                (base->TXBTIE) = ((base ->TXBTIE) | (temp));
            }
            else
            {
                (base->TXBTIE) = ((base->TXBTIE) & ~(temp));
            }
        }
        break;
    case MCAN_RX_BUFF :
        /* Enable the corresponding message buffer Interrupt */
        temp = 1UL << (msgBuffIdx % 32U);
        if (msgBuffIdx  < 32U)
        {
            if (enable)
            {
                base->NDAT1 = temp;
            }
            else
            {
                (base->NDAT1) = ((base->NDAT1) & ~(temp));
            }
        }
        if ((msgBuffIdx >= 32U) && (msgBuffIdx < 64U))
        {
            if (enable)
            {
                base->NDAT2 = temp;
            }
            else
            {
                (base->NDAT2) = ((base->NDAT2) & ~(temp));
            }
        }
        break;
    default:
           /* Do Nothing */
           break;
    };
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetInterruptLine
 * Description   : Get interrupt line active for tested interrupt no
 *
 *END**************************************************************************/
uint8_t MCAN_GetInterruptLine(M_CAN_Type const *base, uint8_t inter )
{
    uint8_t flag = 0xFF;
    uint32_t mask;

    mask = ((base->IE) & (1UL << (inter % 32U)));
    mask = base->IR & mask;
    if (mask != 0U)
    {
        flag = (uint8_t)(((base->ILS & mask) >> (inter % 32U)) & 1UL);
    }
    return flag;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetVirtualMBIndex
 * Description   : Return the Virtual MailBox index corresponding to real message
 * buffer index, based on operation issued on the message buffer
 *
 *END**************************************************************************/
uint8_t MCAN_GetVirtualMBIndex( M_CAN_Type const *base, mode_type_t type, uint32_t msgBuffIdx)
{
    uint32_t value = msgBuffIdx;
    if (type == MCAN_RX_FIFO0)
    {
        return 0U;
    }
    value += 1U;
    if (type == MCAN_RX_FIFO1)
    {
        return (uint8_t)value;
    }
    value += 1U;
    if (type == MCAN_TX_BUFF)
    {
        return (uint8_t)value;
    }
    value += MCAN_GetTxBuffSize(base);
    if (type == MCAN_RX_BUFF)
    {
        return (uint8_t)value;
    }
    /* Should Never reach here */
    return 0U;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetMsgBuffIntCmd
 * Description   : Enable/Disable the corresponding Message Buffer interrupt.
 *
 *END**************************************************************************/
uint8_t MCAN_GetMsgBuffIntCmd(M_CAN_Type const *base, mode_type_t type, uint32_t msgBuffIdx)
{
    uint8_t flag = 0;
    uint32_t mask;

    switch(type)
    {
    case MCAN_TX_BUFF :
        mask = base->TXBTIE;
        flag = (uint8_t)(((base->TXBTO & mask) >> (msgBuffIdx % 32U)) & 1U);
        break;
    case MCAN_RX_BUFF :
        if (msgBuffIdx < 32U)
        {
            flag = (uint8_t)((base->NDAT1 >> (msgBuffIdx % 32U)) & 1U);
        }
#if FEATURE_MCAN_RX_MB_NUM > 32U
        if ((msgBuffIdx >= 32U) && (msgBuffIdx < 64U))
        {
            flag = (uint8_t)((base->NDAT2 >> (msgBuffIdx % 32U)) & 1U);
        }
#endif
        break;
    default :
        flag = 0u;
        break;
    };
    return flag;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetCancelReqStatus
 * Description   : Return if cancellation has occurred or not
 *
 *END**************************************************************************/
uint8_t MCAN_GetCancelReqStatus(M_CAN_Type const *base, uint32_t msgBuffIdx)
{
    uint8_t flag = 0;
    uint32_t mask;

    mask = base->TXBCIE;
    flag = (uint8_t)(((base->TXBCF & mask) >> (msgBuffIdx % 32U)) & 1U);

    return flag;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_CheckIdFilter
 * Description   : Check if a filter is set on the filter table in place
 *
 *END**************************************************************************/
status_t MCAN_CheckIdFilter(M_CAN_Type const *base, bool isExtended, uint32_t filterID,uint8_t filterIdx)
{
    uint8_t fl_idx;
    mcan_id_table_t idFilter;
    if (true == isExtended)
    {
        DEV_ASSERT(filterIdx <= MCAN_GetExtIDFilterSize(base));
    }
    else
    {
        DEV_ASSERT(filterIdx <= MCAN_GetStdIDFilterSize(base));
    }
    for (fl_idx = 0U; fl_idx <= filterIdx; fl_idx++)
    {
        if (true == isExtended)
        {
            MCAN_GetEXT_IDFilter(base, &idFilter, fl_idx);
        }
        else
        {
            MCAN_GetSTD_IDFilter(base, &idFilter, fl_idx);
        }
        if ((idFilter.filterConfig != FILTER_CONF_DISABLE) && (idFilter.filterConfig != FILTER_CONF_RX_BUFF))
        {
            switch (idFilter.filterType)
            {
            case FILTER_TYPE_RANGE_ID:
                if ((idFilter.id1 <= filterID) && (idFilter.id2 >= filterID))
                {
                    return STATUS_ERROR;
                }
                break;
            case FILTER_TYPE_DUAL_ID:
                if ((idFilter.id1 == filterID) || (idFilter.id2 == filterID))
                {
                    return STATUS_ERROR;
                }
                break;
            case FILTER_TYPE_CLASIC:
                if ((idFilter.id1 & idFilter.id2) == (filterID & idFilter.id2))
                {
                    return STATUS_ERROR;
                }
                break;
            case FILTER_TYPE_DISABLE:
                if ((idFilter.isExtendedFrame == true) && (idFilter.id1 <= filterID) && (idFilter.id2 >= filterID))
                {
                    return STATUS_ERROR;
                }
                break;
            default :
                /* Do Nothing */
                break;
            };
        }
        else if ((idFilter.filterConfig == FILTER_CONF_RX_BUFF))
        {
            if ((idFilter.id1 == filterID))
            {
                return STATUS_ERROR;
            }
        }
        else
        {
            /* Misra Require Else If Rule 15.7*/
        }
    }
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetFilterConfig
 * Description   : Returns the Filter configuration, based on filter configuration field
 * value and filter type ID std/ext
 *
 *END**************************************************************************/
static filter_element_config_t MCAN_GetFilterConfig(mcan_msgbuff_id_type_t type, uint32_t value)
{
    filter_element_config_t ret = FILTER_CONF_DISABLE;
    uint32_t aux = 0U;
    if (type == MCAN_MSG_ID_STD)
    {
        aux = ((value & MCAN_STD_FILTER_ID_SFEC_MASK) >> MCAN_STD_FILTER_ID_SFEC_SHIFT);
    }
    else
    {
        aux = ((value & MCAN_EXT_FILTER_ID_EFEC_MASK) >> MCAN_EXT_FILTER_ID_EFEC_SHIFT);
    }

    switch    (aux)
    {
        case 0U :
            ret =  FILTER_CONF_DISABLE;
            break;
        case 1U :
            ret =  FILTER_CONF_RX_FIFO0;
            break;
        case 2U :
            ret =  FILTER_CONF_RX_FIFO1;
            break;
        case 3U :
            ret =  FILTER_CONF_REJECT_ID;
            break;
        case 4U :
            ret =  FILTER_CONF_SET_PRIO;
            break;
        case 5U :
            ret =  FILTER_CONF_SET_PRIO_RX_FIFO0;
            break;
        case 6U :
            ret =  FILTER_CONF_SET_PRIO_RX_FIFO1;
            break;
        case 7U :
            ret =  FILTER_CONF_RX_BUFF;
            break;
        default :
            /* Do Nothing */
            break;
    };
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetFilterType
 * Description   : Returns the Filter type, based on filter type field
 * value and filter type ID std/ext
 *
 *END**************************************************************************/
static filter_type MCAN_GetFilterType(mcan_msgbuff_id_type_t type, uint32_t value)
{
    filter_type ret = FILTER_TYPE_DISABLE;
    uint32_t aux;
    if (type == MCAN_MSG_ID_STD)
    {
        aux = ((value & MCAN_STD_FILTER_ID_SFT_MASK) >> MCAN_STD_FILTER_ID_SFT_SHIFT);
    }
    else
    {
        aux = ((value & MCAN_EXT_FILTER_ID_EFT_MASK) >> MCAN_EXT_FILTER_ID_EFT_SHIFT);
    }

    switch (aux)
    {
        case 0U :
            ret = FILTER_TYPE_RANGE_ID;
            break;
        case 1U :
            ret = FILTER_TYPE_DUAL_ID;
            break;
        case 2U :
            ret = FILTER_TYPE_CLASIC;
            break;
        case 3U :
            ret = FILTER_TYPE_DISABLE;
            break;
        default :
            /* Do Nothing */
            break;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetSTD_IdFilter
 * Description   : Get standard Id Filter
 *
 *END**************************************************************************/
void MCAN_GetSTD_IDFilter(M_CAN_Type const *base, mcan_id_table_t *idFilter, uint8_t fl_idx)
{
    uint32_t filterAddress;
    DEV_ASSERT(idFilter != NULL);
    DEV_ASSERT(fl_idx <= MCAN_GetStdIDFilterSize(base));
    filterAddress = (base->SIDFC&M_CAN_SIDFC_FLSSA_MASK) + ((uint32_t)fl_idx << 2U);
    filterAddress = FEATURE_MCAN_MESSAGE_RAM_START_ADR | filterAddress;

    uint32_t const *stdFlElem = (uint32_t *)filterAddress;
    idFilter->filterConfig    = MCAN_GetFilterConfig(MCAN_MSG_ID_STD, *stdFlElem);
    idFilter->filterType      = MCAN_GetFilterType(MCAN_MSG_ID_STD, *stdFlElem);
    idFilter->id1             = (*stdFlElem & MCAN_STD_FILTER_ID_SFID1_MASK) >> MCAN_STD_FILTER_ID_SFID1_SHIFT;
    idFilter->id2             = (*stdFlElem & MCAN_STD_FILTER_ID_SFID2_MASK) >> MCAN_STD_FILTER_ID_SFID2_SHIFT;
    idFilter->isExtendedFrame = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetSTD_IdFilter
 * Description   : Set standard Id Filter
 *
 *END**************************************************************************/
void MCAN_SetSTD_IdFilter(M_CAN_Type const *base, mcan_id_table_t const *idFilter, uint8_t fl_idx)
{
    DEV_ASSERT(idFilter != NULL);
    DEV_ASSERT(fl_idx <= MCAN_GetStdIDFilterSize(base));
    uint32_t filterAddress;
    filterAddress = (base->SIDFC&M_CAN_SIDFC_FLSSA_MASK) + ((uint32_t)fl_idx << 2U);
    filterAddress = FEATURE_MCAN_MESSAGE_RAM_START_ADR | filterAddress;

    uint32_t *stdFlElem =  (uint32_t *)filterAddress;

    *stdFlElem = 0U;
    *stdFlElem =  MCAN_STD_FILTER_ID_SFT_MASK & ((uint32_t)idFilter->filterType << MCAN_STD_FILTER_ID_SFT_SHIFT);
    *stdFlElem |= MCAN_STD_FILTER_ID_SFEC_MASK & ((uint32_t)idFilter->filterConfig << MCAN_STD_FILTER_ID_SFEC_SHIFT);
    *stdFlElem |= MCAN_STD_FILTER_ID_SFID1_MASK & (idFilter->id1 << MCAN_STD_FILTER_ID_SFID1_SHIFT);
    *stdFlElem |= MCAN_STD_FILTER_ID_SFID2_MASK & (idFilter->id2 << MCAN_STD_FILTER_ID_SFID2_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetEXT_IdFilter
 * Description   : Get Extended Id Filter
 *
 *END**************************************************************************/
void MCAN_GetEXT_IDFilter(M_CAN_Type const *base, mcan_id_table_t *idFilter, uint8_t fl_idx)
{
    DEV_ASSERT(idFilter != NULL);
    DEV_ASSERT(fl_idx <= MCAN_GetExtIDFilterSize(base));
    uint32_t filterAddress;
    filterAddress = (base->XIDFC&M_CAN_XIDFC_FLESA_MASK) + ((uint32_t)fl_idx << 3U);
    filterAddress = FEATURE_MCAN_MESSAGE_RAM_START_ADR | filterAddress;

    uint32_t const *extFlElem0 = (uint32_t *)filterAddress;
    filterAddress = filterAddress + 4U;
    uint32_t const *extFlElem1 = (uint32_t *)filterAddress;

    idFilter->filterConfig = MCAN_GetFilterConfig(MCAN_MSG_ID_EXT, *extFlElem0);
    idFilter->filterType = MCAN_GetFilterType(MCAN_MSG_ID_EXT, *extFlElem1);
    idFilter->id1 =  (*extFlElem0&MCAN_EXT_FILTER_ID_EFID_MASK);
    idFilter->id2 =  (*extFlElem1&MCAN_EXT_FILTER_ID_EFID_MASK);
    idFilter->isExtendedFrame=true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_SetEXT_IdFilter
 * Description   : Set Extended Id Filter
 *
 *END**************************************************************************/
void MCAN_SetEXT_IdFilter(M_CAN_Type const *base, mcan_id_table_t const * const idFilter, uint8_t fl_idx)
{
    DEV_ASSERT(idFilter != NULL);
    DEV_ASSERT(fl_idx <= MCAN_GetExtIDFilterSize(base));
    uint32_t filterAddress;
    filterAddress = (base->XIDFC&M_CAN_XIDFC_FLESA_MASK) + ((uint32_t)fl_idx << 3U);
    filterAddress = FEATURE_MCAN_MESSAGE_RAM_START_ADR | filterAddress;

    uint32_t *extFlElem0 = (uint32_t *)filterAddress;
    filterAddress = filterAddress + 4U;
    uint32_t *extFlElem1 = (uint32_t *)filterAddress;

    *extFlElem0 = 0U;
    *extFlElem1 = 0U;

    *extFlElem0 = MCAN_EXT_FILTER_ID_EFEC_MASK & ((uint32_t)idFilter->filterConfig << MCAN_EXT_FILTER_ID_EFEC_SHIFT);
    *extFlElem0 |= MCAN_EXT_FILTER_ID_EFID_MASK & idFilter->id1;
    *extFlElem1 = MCAN_EXT_FILTER_ID_EFT_MASK & ((uint32_t)idFilter->filterType << MCAN_EXT_FILTER_ID_EFT_SHIFT);
    *extFlElem1 |=  MCAN_EXT_FILTER_ID_EFID_MASK & idFilter->id2;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_GetMsgBuff
 * Description   : Gets the MCAN message buffer fields.
 *
 *END**************************************************************************/
void  MCAN_GetMsgBuff( M_CAN_Type const *base,
                        uint32_t msgBuffIdx,
                        mode_type_t type,
                        mcan_msgbuff_t *msgBuff)
{
    DEV_ASSERT(msgBuff != NULL);

    uint8_t i=0U;

    volatile uint32_t const *mcan_mb = MCAN_GetMsgBuffRegion(base, msgBuffIdx, type);

    volatile uint32_t const *mcan_mb_id   = &mcan_mb[0U];
    volatile uint32_t const *mcan_mb_code = &mcan_mb[1U];

    volatile const uint8_t  *mcan_mb_data = (volatile const uint8_t *)(&mcan_mb[2U]);
    volatile const uint32_t *mcan_mb_data_32 = &mcan_mb[2U];
    uint32_t *msgBuff_data_32 = (uint32_t *)(msgBuff->data);

    uint8_t can_mb_dlc_value = (uint8_t)(((*mcan_mb_code) & CAN_CS_DLC_MASK) >> 16U);
    uint8_t payload_size = MCAN_ComputePayloadSize(can_mb_dlc_value);

    msgBuff->dataLen = payload_size;
    msgBuff->cs = *mcan_mb_code;
    if ((*mcan_mb_id & CAN_ID_XTD_MASK) != 0U)
    {
        msgBuff->msgId = ((*mcan_mb_id) & CAN_ID_EXT_MASK);
    }
    else
    {
        msgBuff->msgId = ((*mcan_mb_id) & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT;
    }

    for (i = 0U; i < (payload_size & ~3U); i += 4U)
    {
        uint32_t aux = mcan_mb_data_32[i >> 2U];
        REV_BYTES_32(aux, msgBuff_data_32[i >> 2U]);
    }
    for (; i < payload_size; i++)
    {    /* Max allowed value for index is 63 */
        msgBuff->data[i] = mcan_mb_data[(i & 0xFCU) + (3U - (i & 3U))];
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAN_AbortRX
 * Description   : Abort receive by disable the corresponding filter.
 *
 *END**************************************************************************/
void MCAN_AbortRX(M_CAN_Type const * base, mcan_state_t const * state, uint8_t vmb_idx)
{
    mcan_id_table_t filterID;
    filterID.filterConfig = FILTER_CONF_DISABLE;
    if (state->mbs[vmb_idx].isExtended == true)
    {
        MCAN_SetEXT_IdFilter(base, &filterID, state->mbs[vmb_idx].filterIdx);
    }
    else
    {
        MCAN_SetSTD_IdFilter(base, &filterID, state->mbs[vmb_idx].filterIdx);
    }
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
