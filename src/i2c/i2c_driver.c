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

#include "osif.h"
#include "i2c_hw_access.h"
#include "i2c_driver.h"
#include "interrupt_manager.h"

/*!
 * @i2c_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, Could define variable at block scope
 * The variables are defined in the common source file and this rule can't be
 * applied.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 * This conversion is required because the converted values are the addresses used in DMA transfer.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long, cast from unsigned long to pointer
 * and cast from unsigned int to pointer. The cast is required to perform a conversion between a pointer
 * and an unsigned long define, representing an address or vice versa.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const/volatile from a pointer or reference.
 * The cast is required to initialize a DMA transfer. The converted value is the address of a register.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code
 * structure and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 18.4, Pointer arithmetic other than array indexing used.
 * This operation it's necessary to handle the buffers after the DMA transfer is completed.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define I2C_DRV_DUMMY_BYTE    (0xFFU)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for I2C instances. */
static I2C_Type * const g_i2cBase[I2C_INSTANCE_COUNT] = I2C_BASE_PTRS;

/* Table for i2c IRQ numbers */
static const IRQn_Type g_i2cIrqId[I2C_INSTANCE_COUNT] = I2C_IRQS;

/* Pointer to runtime state structure.*/
static i2c_master_state_t* g_i2cMasterStatePtr[I2C_INSTANCE_COUNT] = { NULL };
static i2c_slave_state_t* g_i2cSlaveStatePtr[I2C_INSTANCE_COUNT] = { NULL };

/* Master Start DMA transfer */
static void I2C_DRV_MasterStartDmaTransfer(uint32_t instance);

/* I2C DMA request sources */
static const uint8_t g_i2cDMASrc[I2C_INSTANCE_COUNT][2] = I2C_DMA_REQ;

/* Clock sources, for getting the input clock frequency */
static const clock_names_t g_i2cClock[I2C_INSTANCE_COUNT] = I2C_CLOCKS;

/* @brief SCL divider values */
static const uint32_t g_i2cDividers[NO_I2C_SCL_DIVIDERS][2] = I2C_SCL_DIVIDER;

static status_t I2C_DRV_MasterWaitTransferEnd(uint32_t instance,
        uint32_t timeout);

/*! @brief Direction of a I2C transfer - transmit or receive.
 * */
typedef enum {
    I2C_TX_REQ = 0, /*!< The driver will perform an I2C transmit transfer */
    I2C_RX_REQ = 1, /*!< The driver will perform an I2C receive transfer */
} i2c_transfer_direction_t;

/*!
 * @brief DMA internal parameters structure
 *
 * This structure is used in DMA transfers. It contains different
 * variables required for setting and maintaining a DMA transfer.
 */
typedef struct {
    /*! @cond DRIVER_INTERNAL_USE_ONLY */
    uint8_t dmaChannel;                          /* Number of DMA channel */
    edma_transfer_type_t dmaTransferType;        /* DMA type transfer */
    uint32_t i2cDataRegAddr;                     /* I2C data register address */
    uint8_t *bufferTransfer;                     /* Buffer used for transfer */
    uint32_t transferSize;                       /* Buffer data size */
    i2c_transfer_direction_t transferDirection;  /* DMA transfer direction: receive or transmit */
/*! @endcond */
} i2c_dma_transfer_params_t;


/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterInit
 * Description   : initialize the I2C master mode driver
 * Implements    : I2C_DRV_MasterInit_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterInit(uint32_t instance,
        const i2c_master_user_config_t * userConfigPtr,
        i2c_master_state_t * master)
{

    I2C_Type *baseAddr;
    status_t retVal;

    DEV_ASSERT(master != NULL);
    DEV_ASSERT(userConfigPtr != NULL);
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    /*  Check to see if the I2C master instance is already initialized */
    DEV_ASSERT(g_i2cMasterStatePtr[instance] == NULL);

    baseAddr = g_i2cBase[instance];
    g_i2cMasterStatePtr[instance] = master;

    /*  Initialize driver status structure */
    master->rxBuff = NULL;
    master->rxSize = 0U;
    master->txBuff = NULL;
    master->txSize = 0U;
    master->status = STATUS_SUCCESS;
    master->i2cIdle = true;
    master->slaveAddress = userConfigPtr->slaveAddress;
    master->transferType = userConfigPtr->transferType;
    master->stopGenerated = true;

    /* Initialize callback and callback parameter*/
    master->masterCallback = userConfigPtr->masterCallback;
    master->callbackParam = userConfigPtr->callbackParam;

    /*  Store DMA channel number used in transfer */
    master->dmaChannel = userConfigPtr->dmaChannel;
    master->blocking = false;

    /* Initial state */
    I2C_ResetModuleState(baseAddr);

    /* Initialize the semaphore */
    retVal = OSIF_SemaCreate(&(master->idleSemaphore), 0);
    DEV_ASSERT(retVal == STATUS_SUCCESS);

    /* Set baud rate */
    (void)I2C_DRV_MasterSetBaudRate(instance, userConfigPtr->baudRate);

    /* Enable I2C Module*/
    I2C_Set_ModuleEnable(baseAddr, true);

    /* Interrupt enable*/
    INT_SYS_EnableIRQ(g_i2cIrqId[instance]);

    (void) retVal;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveInit
 * Description   : initialize the I2C slave mode driver
 * Implements    : I2C_DRV_SlaveInit_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveInit(uint32_t instance,
        const i2c_slave_user_config_t * userConfigPtr,
        i2c_slave_state_t * slave)
{

    I2C_Type *baseAddr;
    status_t retVal;

    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(userConfigPtr != NULL);
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    DEV_ASSERT(g_i2cSlaveStatePtr[instance] == NULL);

    baseAddr = g_i2cBase[instance];
    g_i2cSlaveStatePtr[instance] = slave;

    /*  Initialize driver status structure */
    slave->status = STATUS_SUCCESS;
    slave->slaveListening = userConfigPtr->slaveListening;
    slave->slaveCallback = userConfigPtr->slaveCallback;
    slave->callbackParam = userConfigPtr->callbackParam;
    slave->txBuff = NULL;
    slave->rxBuff = NULL;
    slave->txSize = 0U;
    slave->rxSize = 0U;
    slave->slaveAddress = userConfigPtr->slaveAddress;
    slave->dataAck = true;
    slave->isTransferInProgress = false;
    slave->blocking = false;

    /* Initial state */
    I2C_ResetModuleState(baseAddr);

    /* Initialize the semaphore */
    retVal = OSIF_SemaCreate(&(slave->idleSemaphore), 0);
    DEV_ASSERT(retVal == STATUS_SUCCESS);

    /* Set Slave Address*/
    I2C_Set_SlaveAddress(baseAddr, slave->slaveAddress);

    /* Enable i2c interrupt*/
    INT_SYS_EnableIRQ(g_i2cIrqId[instance]);

    if (slave->slaveListening)
    {
        /* Interrupts enabled*/
        I2C_Set_BusIRQEnable(baseAddr, true);

        /* Enable I2C Module*/
        I2C_Set_ModuleEnable(baseAddr, true);

        /* Slave Mode */
        I2C_Set_ModuleModSelect(baseAddr, I2C_SLAVE);

        /* Enable Bus idle interrupt */
        I2C_Set_BusIdleIRQ(baseAddr, true);

        /* Enable/Disable Acknowledge */
        I2C_Set_DataAcknowledge(baseAddr, true);

        /* Clear bus interrupt flag*/
        I2C_Clear_BusInterruptFlag(baseAddr, true);
    }

    (void) retVal;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSendAddress
 * Description   : Sets the 7-bit address of the slave which will be used for
 * any future transfers initiated by the I2C master.
 * Implements    : I2C_DRV_MasterSetSlaveAddress_Activity
 *
 *END**************************************************************************/
void I2C_DRV_MasterSetSlaveAddress(uint32_t instance, uint8_t address)
{

    i2c_master_state_t * master;
    master = g_i2cMasterStatePtr[instance];

    DEV_ASSERT(master != NULL);

    master->slaveAddress = address;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterTransmitAddress
 * Description   : master module sends on I-BUS the slave address and checks for
 * acknowledge
 *
 *END**************************************************************************/
static status_t I2C_DRV_MasterTransmitAddress(uint32_t instance)
{
    uint8_t address;
    I2C_Type *baseAddr;
    const i2c_master_state_t * master;

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];

    DEV_ASSERT(master != NULL);
    DEV_ASSERT(master != NULL);

    /* Select master mode */
    I2C_Set_ModuleModSelect(baseAddr, I2C_MASTER);

    /* Select transmit mode for master */
    I2C_Set_ModuleTransmitMode(baseAddr, true);

    /* Transmit address */
    address = (uint8_t) ((master->slaveAddress << 1U) + (uint8_t) (!master->sendData));
    I2C_Set_DataIBDR(baseAddr, address);

	/* In case master want to receive one byte set NACK */
    if((!master->sendData) && (master->rxSize == 1U))
    {
        /* Send NACK */
        I2C_Set_DataAcknowledge(baseAddr, false);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_GenerateStopSignal
 * Description   : generates stop signal for mastre I2C module
 *
 *END**************************************************************************/
static void I2C_DRV_GenerateStopSignal(uint8_t instance)
{
    I2C_Type *baseAddr;
	i2c_master_state_t * master;

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];

    /* Check if the module is in master mode */
    if (I2C_Get_ModuleModSelect(baseAddr))
    {
        /* Generate stop */
        I2C_Set_ModuleModSelect(baseAddr, I2C_SLAVE);
        master->stopGenerated = true;
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSetRxBuffer
 * Description   : sets the RX buffer for slave module
 * Implements    : I2C_DRV_SlaveSetRxBuffer_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveSetRxBuffer(uint8_t instance, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_slave_state_t *slave;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    slave->rxBuff = rxBuff;
    slave->rxSize = rxSize;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSetTxBuffer
 * Description   : sets the TX buffer for slave module
 * Implements    : I2C_DRV_SlaveSetTxBuffer_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveSetTxBuffer(uint8_t instance, const uint8_t *txBuff,
        uint32_t txSize)
{
    i2c_slave_state_t *slave;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    slave->txBuff = txBuff;
    slave->txSize = txSize;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSendData
 * Description   : perform a non-blocking send transaction on the I2C bus
 * Implements    : I2C_DRV_MasterSendData_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterSendData(uint32_t instance, const uint8_t *txBuff,
        uint32_t txSize, bool sendStop)
{
    I2C_Type *baseAddr;
    i2c_master_state_t *master;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Check if driver is busy */
    if(!master->i2cIdle)
    {
    	return STATUS_BUSY;
    }

    if(master->stopGenerated)
    {
        /* Check if bus is busy */
        if(I2C_Get_BusBusyEvent(baseAddr))
        {
    	    return STATUS_I2C_BUS_BUSY;
        }
    }

    /* Copy parameters to drive state structure */
    master->txBuff = txBuff;
    master->txSize = txSize;
    master->i2cIdle = false;
    master->status = STATUS_BUSY;
    master->sendData = true;
    master->sendStop = sendStop;

    /* Activate interrupts for DMA transfer*/
    I2C_Set_BusIRQEnable(baseAddr, true);

    /* Activate interrupts for bus idle */
    I2C_Set_BusIdleIRQ(baseAddr, true);

    if (master->transferType == I2C_USING_DMA)
    {
        /* Start DMA transfer */
        I2C_DRV_MasterStartDmaTransfer(instance);
    }
    else
    {
        (void)I2C_DRV_MasterTransmitAddress(instance);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveData
 * Description   : perform a blocking send transaction on the I2C bus
 * Implements    : I2C_DRV_MasterReceiveData_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterReceiveData(uint32_t instance, uint8_t *rxBuff,
        uint32_t rxSize, bool sendStop)
{
    I2C_Type *baseAddr;
    i2c_master_state_t *master;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Check if driver is busy */
    if(!master->i2cIdle)
    {
    	return STATUS_BUSY;
    }

    if(master->stopGenerated)
    {
        /* Check if bus is busy */
        if(I2C_Get_BusBusyEvent(baseAddr))
        {
    	    return STATUS_I2C_BUS_BUSY;
        }
    }

    /* Copy parameters to driver state structure */
    master->rxSize = rxSize;
    master->i2cIdle = false;
    master->rxBuff = rxBuff;
    master->status = STATUS_BUSY;
    master->sendData = false;
    master->sendStop = sendStop;

    /* Acknowledge enabled */
    I2C_Set_DataAcknowledge(baseAddr, true);

    /* Enable interrupts */
    I2C_Set_BusIRQEnable(baseAddr, true);

    /* Activate interrupts for bus idle*/
    I2C_Set_BusIdleIRQ(baseAddr, true);

    if (master->transferType == I2C_USING_DMA)
    {
        /* Start DMA transfer */
        I2C_DRV_MasterStartDmaTransfer(instance);
    }
    else
    {
        (void)I2C_DRV_MasterTransmitAddress(instance);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 * Implements    : I2C_DRV_MasterSendDataBlocking_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterSendDataBlocking(uint32_t instance,
        const uint8_t *txBuff, uint32_t txSize, bool sendStop, uint32_t timeout)
{
    status_t retVal = STATUS_SUCCESS;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    i2c_master_state_t *master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Check if driver is busy */
    if(!master->i2cIdle)
    {
    	return STATUS_BUSY;
    }

    /* mark transfer as blocking */
    master->blocking = true;
	
    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)OSIF_SemaWait(&(master->idleSemaphore), 0);

    retVal = I2C_DRV_MasterSendData(instance, txBuff, txSize, sendStop);

    if (retVal != STATUS_SUCCESS)
    {
        master->blocking = false;
        return retVal;
    }

    /* Wait for transfer to end */
    return I2C_DRV_MasterWaitTransferEnd(instance, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 * Implements    : I2C_DRV_MasterReceiveDataBlocking_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterReceiveDataBlocking(uint32_t instance, uint8_t *rxBuff,
        uint32_t rxSize, bool sendStop, uint32_t timeout)
{
    status_t retVal = STATUS_SUCCESS;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    i2c_master_state_t *master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Check if driver is busy */
    if(!master->i2cIdle)
    {
    	return STATUS_BUSY;
    }

    /* mark transfer as blocking */
    master->blocking = true;
	
    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)OSIF_SemaWait(&(master->idleSemaphore), 0);

    retVal = I2C_DRV_MasterReceiveData(instance, rxBuff, rxSize, sendStop);
    if (retVal != STATUS_SUCCESS)
    {
        master->blocking = false;
        return retVal;
    }

    /* Wait for transfer to end */
    return I2C_DRV_MasterWaitTransferEnd(instance, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSendDataEvent
 * Description   : handle a send data event for master
 *
 *END**************************************************************************/
static void I2C_DRV_MasterSendDataEvent(uint32_t instance, I2C_Type *baseAddr,
        i2c_master_state_t *master)
{
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(baseAddr != NULL);
    DEV_ASSERT(master != NULL);

    if (master->txSize > 0U)
    {
        I2C_Set_DataIBDR(baseAddr, master->txBuff[0U]);

        master->txBuff++;
        master->txSize--;
    }
    else
    {
        master->status = STATUS_SUCCESS;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterEndTransfer
 * Description   : ends current transmission or reception
 *
 *END**************************************************************************/
static void I2C_DRV_MasterEndTransfer(I2C_Type *baseAddr,
        i2c_master_state_t *master)
{
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(baseAddr != NULL);

    /* Disable all events */
    I2C_Set_BusIRQEnable(baseAddr, false);
    I2C_Set_BusIdleIRQ(baseAddr, false);

    master->txBuff = NULL;
    master->txSize = 0;
    master->rxBuff = NULL;
    master->rxSize = 0;
    master->i2cIdle = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static status_t I2C_DRV_MasterWaitTransferEnd(uint32_t instance,
        uint32_t timeout)
{
    status_t osifError = STATUS_SUCCESS;
    I2C_Type *baseAddr;
    i2c_master_state_t *master;

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];

    /* Wait for transfer to be completed by the IRQ */
    osifError = OSIF_SemaWait(&(master->idleSemaphore), timeout);

    if (osifError == STATUS_TIMEOUT)
    {
    	if(master->transferType == I2C_USING_DMA)
    	{
    		(void)EDMA_DRV_StopChannel(master->dmaChannel);
    		I2C_Set_DMAEnable(baseAddr, false);
    	}
    	master->sendStop = true;

    	/* In case of timeout, abort will be done at byte boundary */
    	if(I2C_Get_ModuleTransmitMode(baseAddr))
    	{
    	    master->txSize = 0U;
    	}
    	else
    	{
    		master->rxSize = 1U;
    	}

    	/* Wait for interrupt to occur */
       (void) OSIF_SemaWait(&(master->idleSemaphore), timeout);

        master->status = STATUS_TIMEOUT;

    }

    /* Blocking transfer is over */
    master->blocking = false;
    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveDataEvent
 * Description   : handle a receive data event for master
 *
 *END**************************************************************************/
static void I2C_DRV_MasterReceiveDataEvent(uint32_t instance, const I2C_Type *baseAddr,
        i2c_master_state_t *master)
{
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(baseAddr != NULL);
    DEV_ASSERT(master != NULL);

    if (master->rxSize > 0U)
    {
        /* Receive data from slave and store it */
        master->rxBuff[0U] = I2C_Get_DataIBDR(baseAddr);

        master->rxBuff++;
        master->rxSize--;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveEndTransfer
 * Description   : ends current transmission or reception
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveEndTransfer(I2C_Type *baseAddr,
        i2c_slave_state_t *slave)
{
    DEV_ASSERT(slave != NULL);
    DEV_ASSERT(baseAddr != NULL);

    /* Deactivate events */
    I2C_Set_BusIRQEnable(baseAddr, false);

    /* Disable I2C slave */
    I2C_Set_ModuleEnable(baseAddr, false);

    slave->isTransferInProgress = false;
    slave->rxBuff = NULL;
    slave->rxSize = 0U;
    slave->txBuff = NULL;
    slave->rxSize = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveWaitTransferEnd
 * Description   : waits for the end of a blocking transfer
 *
 *END**************************************************************************/
static status_t I2C_DRV_SlaveWaitTransferEnd(uint32_t instance,
        uint32_t timeout)
{
    status_t osifError = STATUS_SUCCESS;
    I2C_Type *baseAddr;
    i2c_slave_state_t *slave;

    baseAddr = g_i2cBase[instance];
    slave = g_i2cSlaveStatePtr[instance];

    /* Wait for transfer to be completed by the IRQ */
    osifError = OSIF_SemaWait(&(slave->idleSemaphore), timeout);

    if (osifError == STATUS_TIMEOUT)
    {
        I2C_DRV_SlaveEndTransfer(baseAddr, slave);
        slave->status = STATUS_TIMEOUT;
    }

    /* Blocking transfer is over */
    slave->blocking = false;
    return slave->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveReceiveData
 * Description   : perform a non-blocking receive transaction on the I2C bus
 * Implements    : I2C_DRV_SlaveReceiveData_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveReceiveData(uint32_t instance, uint8_t *rxBuff,
        uint32_t rxSize)
{
    I2C_Type *baseAddr;
    i2c_slave_state_t *slave;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    baseAddr = g_i2cBase[instance];
    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* Use this function only if the slave is not listening */
    DEV_ASSERT(slave->slaveListening == false);

    /* Check if driver is busy */
    if(slave->isTransferInProgress)
    {
    	return STATUS_BUSY;
    }

    slave->rxBuff = rxBuff;
    slave->rxSize = rxSize;
    slave->status = STATUS_BUSY;

    slave->isTransferInProgress = true;

    /* Enable I2C Module*/
    I2C_Set_ModuleEnable(baseAddr, true);

    /* Slave Mode */
    I2C_Set_ModuleModSelect(baseAddr, I2C_SLAVE);

    /*  Enable/Disable Acknowledge */
    I2C_Set_DataAcknowledge(baseAddr, slave->dataAck);

    /* Clear bus interrupt flag*/
    I2C_Clear_BusInterruptFlag(baseAddr, true);

    /* Interrupt enabled*/
    I2C_Set_BusIRQEnable(baseAddr, true);

    /* Enable bus idle interrupts */
    I2C_Set_BusIdleIRQ(baseAddr, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveReceiveDataBlocking
 * Description   : perform a blocking receive transaction on the I2C bus
 * Implements : I2C_DRV_SlaveReceiveDataBlocking_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveReceiveDataBlocking(uint32_t instance, uint8_t *rxBuff,
        uint32_t rxSize, uint32_t timeout)
{
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    i2c_slave_state_t *slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* Check if driver is busy */
    if(slave->isTransferInProgress)
    {
    	return STATUS_BUSY;
    }

    /* mark transfer as blocking */
    slave->blocking = true;
	
    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)OSIF_SemaWait(&(slave->idleSemaphore), 0);

    (void)I2C_DRV_SlaveReceiveData(instance, rxBuff, rxSize);

    /* Wait for transfer to end */
    return I2C_DRV_SlaveWaitTransferEnd(instance, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSendData
 * Description   : performs a non-blocking send transaction on the I2C bus
 * Implements    : I2C_DRV_SlaveSendData_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveSendData(uint32_t instance, const uint8_t * txBuff,
        uint32_t txSize)
{
    I2C_Type *baseAddr;
    i2c_slave_state_t * slave;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    baseAddr = g_i2cBase[instance];
    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* If the slave is in listening mode the user should not use this function or the blocking counterpart. */
    DEV_ASSERT(slave->slaveListening == false);

    /* Check if slave is busy */
    if(slave->isTransferInProgress)
    {
    	return STATUS_BUSY;
    }

    slave->txBuff = txBuff;
    slave->txSize = txSize;
    slave->status = STATUS_BUSY;

    /* Interrupts enabled*/
    I2C_Set_BusIRQEnable(baseAddr, true);

    /* Enable I2C Module*/
    I2C_Set_ModuleEnable(baseAddr, true);

    /* Enable bus idle interrupts */
    I2C_Set_BusIdleIRQ(baseAddr, true);

    /* Clear bus interrupt flag*/
    I2C_Clear_BusInterruptFlag(baseAddr, true);

    slave->isTransferInProgress = true;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSendDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 * Implements : I2C_DRV_SlaveSendDataBlocking_Activity
 *END**************************************************************************/
status_t I2C_DRV_SlaveSendDataBlocking(uint32_t    instance,
                                           const uint8_t *  txBuff,
                                           uint32_t txSize,
                                           uint32_t timeout)
{
    status_t retVal = STATUS_SUCCESS;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    i2c_slave_state_t *slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* Check if driver is busy */
    if(slave->isTransferInProgress)
    {
    	return STATUS_BUSY;
    }

    /* mark transfer as blocking */
    slave->blocking = true;
	
    /* Dummy wait to ensure the semaphore is 0, no need to check result */
    (void)OSIF_SemaWait(&(slave->idleSemaphore), 0);

    retVal = I2C_DRV_SlaveSendData(instance, txBuff, txSize);
    if (retVal != STATUS_SUCCESS)
    {
        return retVal;
    }

    /* Wait for transfer to end */
    return I2C_DRV_SlaveWaitTransferEnd(instance, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveStopEvent
 * Description   : performs stop for slave module
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveStopEvent(I2C_Type *baseAddr, i2c_slave_state_t *slave, status_t moduleStatus)
{
     DEV_ASSERT(baseAddr != NULL);
     DEV_ASSERT(slave != NULL);

    if (!slave->slaveListening)
    {
        I2C_DRV_SlaveEndTransfer(baseAddr, slave);

        /* Signal transfer end for blocking transfers */
        if (slave->blocking == true)
        {
            (void) OSIF_SemaPost(&(slave->idleSemaphore));
        }
    }

    slave->status = moduleStatus;

    if (slave->slaveCallback != NULL) {
        slave->slaveCallback(I2C_SLAVE_EVENT_STOP, slave->callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveSendDataEvent
 * Description   : handle a slave send data event for slave module
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveSendDataEvent(I2C_Type *baseAddr, i2c_slave_state_t *slave)
{
    DEV_ASSERT(baseAddr != NULL);
    DEV_ASSERT(slave != NULL);

    slave->status = STATUS_BUSY;

    if (slave->txSize == 0U)
    {
        /* Out of data, call callback to allow user to provide a new buffer */
        if (slave->slaveCallback != NULL)
        {
            slave->slaveCallback(I2C_SLAVE_EVENT_TX_EMPTY, slave->callbackParam);
        }
    }

    if (slave->txSize > 0U)
    {
        /* Send data to master*/
        I2C_Set_DataIBDR(baseAddr, slave->txBuff[0U]);

        slave->txBuff++;
        slave->txSize--;
    }
    else
    {
        /* Slave end transfer */
        I2C_DRV_SlaveStopEvent(baseAddr, slave, STATUS_I2C_TX_UNDERRUN);

        /* Transmit dummy byte */
        I2C_Set_DataIBDR(baseAddr, I2C_DRV_DUMMY_BYTE);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveReceiveDataEvent
 * Description   : handle a receive data event
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveReceiveDataEvent(uint8_t instance, I2C_Type *baseAddr,
        i2c_slave_state_t *slave)
{
    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);
    DEV_ASSERT(baseAddr != NULL);
    DEV_ASSERT(slave != NULL);

    slave->status = STATUS_BUSY;

    if (slave->rxSize == 0U)
    {
        if (slave->slaveCallback != NULL)
        {
            slave->slaveCallback(I2C_SLAVE_EVENT_RX_FULL, slave->callbackParam);
        }
    }

    if (slave->rxSize > 0U)
    {
        /* Read data from IBDR and store it */
        slave->rxBuff[0U] = I2C_Get_DataIBDR(baseAddr);

        slave->rxBuff++;
        slave->rxSize--;
    }
    else
    {
        /* Slave end transfer */
        I2C_DRV_SlaveStopEvent(baseAddr, slave, STATUS_I2C_RX_OVERRUN);

        /* Dummy read from IBDR */
        (void) I2C_Get_DataIBDR(baseAddr);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_AddressAsSlaveEvent
 * Description   : handle a address as slave event for slave I2C module
 *
 *END**************************************************************************/
static void I2C_DRV_AddressAsSlaveEvent(I2C_Type *baseAddr,
        i2c_slave_state_t *slave)
{

    DEV_ASSERT(baseAddr != NULL);

    /* Check if slave is transmitting or receiving - check SRW bit */
    if (I2C_Get_SlaveTransmitMode(baseAddr))
    {
        /* Set TX Mode */
        I2C_Set_ModuleTransmitMode(baseAddr, true);

        /* Request from master to transmit data */
        if ((slave->slaveCallback != NULL) && slave->slaveListening)
        {
            slave->slaveCallback(I2C_SLAVE_EVENT_TX_REQ, slave->callbackParam);
        }
        /* Write data to IBDR to clear the IAAS bit! */
        I2C_DRV_SlaveSendDataEvent(baseAddr, slave);
    }
    else
    {
        /* Request from master to receive data */
        if ((slave->slaveCallback != NULL) && slave->slaveListening)
        {
            slave->slaveCallback(I2C_SLAVE_EVENT_RX_REQ, slave->callbackParam);
        }

        /* Set RX Mode */
        I2C_Set_ModuleTransmitMode(baseAddr, false);

        /* Dummy reading */
        (void) I2C_Get_DataIBDR(baseAddr);
    }
    slave->status = STATUS_BUSY;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterCompleteDMATransfer
 * Description   : Finish up a transfer DMA for master. The main purpose of
 *                 this function is to create a function compatible with DMA
 *                 callback type
 *
 *END**************************************************************************/
static void I2C_DRV_MasterCompleteDMATransfer(void *parameter,
        edma_chn_status_t status)
{
    I2C_Type *baseAddr;
    i2c_master_state_t *master;

    uint32_t instance = (uint32_t) parameter;
    baseAddr = g_i2cBase[instance];

    master = g_i2cMasterStatePtr[instance];

    if(status == EDMA_CHN_ERROR)
    {
    	/* Generate stop signal */
    	I2C_DRV_GenerateStopSignal((uint8_t)instance);

        I2C_DRV_MasterEndTransfer(baseAddr, master);

        /* Signal transfer end for blocking transfers */
        if (master->blocking == true)
        {
            (void) OSIF_SemaPost(&(master->idleSemaphore));
        }

        if (master->masterCallback != NULL)
        {
            master->masterCallback(I2C_MASTER_EVENT_END_TRANSFER, master->callbackParam);
        }
    }
    else
    {
        if (master->sendData)
        {
            master->txBuff += master->txSize - 1U;
            master->txSize = 1;
        }
        else
        {
            master->rxBuff += master->rxSize - 2U;
            master->rxSize = 2;
        }
    }

    if(status == EDMA_CHN_ERROR)
    {
        master->status = STATUS_ERROR;
    }
    else
    {
        master->status = STATUS_SUCCESS;
    }

    I2C_Set_DMAEnable(baseAddr, false);

    /* Send the last byte of data in case CPU interrupt is not triggered */
    if ((I2C_Get_TCF(baseAddr)) && (status != EDMA_CHN_ERROR) && (master->sendData))
    {
    	 /* Send the next byte of data */
    	 I2C_DRV_MasterSendDataEvent(instance, baseAddr, master);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_ConfigureDmaTransfer
 * Description   : configures the DMA transfer
 *
 *END**************************************************************************/
static void I2C_DRV_ConfigureDmaTransfer(uint32_t instance, const i2c_dma_transfer_params_t *dmaTransParams)
{
    if (dmaTransParams->transferDirection == I2C_TX_REQ)
    {
        (void)EDMA_DRV_SetChannelRequestAndTrigger(dmaTransParams->dmaChannel,  g_i2cDMASrc[instance][I2C_TX_REQ], false);
        (void)EDMA_DRV_ConfigMultiBlockTransfer(dmaTransParams->dmaChannel, dmaTransParams->dmaTransferType, (uint32_t)dmaTransParams->bufferTransfer,
                                                (uint32_t)dmaTransParams->i2cDataRegAddr, EDMA_TRANSFER_SIZE_1B, (uint32_t)1U,
                                                (uint32_t)dmaTransParams->transferSize, false);
    }
    else
    {
        (void)EDMA_DRV_SetChannelRequestAndTrigger(dmaTransParams->dmaChannel,  g_i2cDMASrc[instance][I2C_RX_REQ], false);
        (void)EDMA_DRV_ConfigMultiBlockTransfer(dmaTransParams->dmaChannel, dmaTransParams->dmaTransferType, (uint32_t)dmaTransParams->i2cDataRegAddr,
                                                (uint32_t)dmaTransParams->bufferTransfer, EDMA_TRANSFER_SIZE_1B, (uint32_t)1U,
                                                (uint32_t)dmaTransParams->transferSize, false);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterStartDmaTransfer
 * Description   : starts the DMA transfer (only for master)
 *
 *END**************************************************************************/
static void I2C_DRV_MasterStartDmaTransfer(uint32_t instance)
{
    I2C_Type *baseAddr = g_i2cBase[instance];
    const i2c_master_state_t *master = g_i2cMasterStatePtr[instance];
    i2c_dma_transfer_params_t dmaTransParams;

    dmaTransParams.dmaChannel = master->dmaChannel;

    if (master->sendData)
    {
        dmaTransParams.dmaTransferType = EDMA_TRANSFER_MEM2PERIPH;
        dmaTransParams.i2cDataRegAddr = (uint32_t) (&(baseAddr->IBDR));
        dmaTransParams.bufferTransfer = (uint8_t *) (master->txBuff);
        dmaTransParams.transferDirection = I2C_TX_REQ;
        dmaTransParams.transferSize = master->txSize - 1U; /* In transmitting mode the DMA is not transferring the last byte of data */
    }
    else
    {
        dmaTransParams.dmaTransferType = EDMA_TRANSFER_PERIPH2MEM;
        dmaTransParams.i2cDataRegAddr = (uint32_t) (&(baseAddr->IBDR));
        dmaTransParams.bufferTransfer = master->rxBuff;
        dmaTransParams.transferDirection = I2C_RX_REQ;
        dmaTransParams.transferSize = master->rxSize - 2U; /* In receiving mode the DMA is not transferring the last two bytes */
    }

    (void) I2C_DRV_ConfigureDmaTransfer(instance, &dmaTransParams);

    /* Disable DMA requests for channel when transfer is done */
    EDMA_DRV_DisableRequestsOnTransferComplete(dmaTransParams.dmaChannel, true);

    /* Call callback function when all the bytes were transfered. */
    (void) EDMA_DRV_InstallCallback(dmaTransParams.dmaChannel,
            (I2C_DRV_MasterCompleteDMATransfer), (void*) (instance));

    /* Start channel */
    (void) EDMA_DRV_StartChannel(dmaTransParams.dmaChannel);

    if (master->sendData)
    {
        /* Enable transmit/receive DMA request*/
        I2C_Set_DMAEnable(baseAddr, true);
    }

    (void)I2C_DRV_MasterTransmitAddress(instance);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterGetTransferStatus
 * Description   : return the current status of the I2C master transfer
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function
 * to ascertain the state of the current transfer. In addition, if the transfer is still
 * in progress, the user can get the number of words that should be receive.
 *
 * Implements    : I2C_DRV_MasterGetTransferStatus_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterGetTransferStatus(uint32_t instance)
{
    const i2c_master_state_t *master;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveGetTransferStatus
 * Description   : return the current status of the I2C master transfer
 *
 * When performing an a-sync (non-blocking) transfer, the user can call this function
 * to ascertain the state of the current transfer. In addition, if the transfer is still
 * in progress, the user can get the number of words that should be receive.
 *
 * Implements : I2C_DRV_SlaveGetTransferStatus_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveGetTransferStatus(uint32_t instance)
{
    const i2c_slave_state_t *slave;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    return slave->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterDeinit
 * Description   : de-initialize the I2C master mode driver
 * Implements : I2C_DRV_MasterDeinit_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterDeinit(uint32_t instance)
{
    I2C_Type *baseAddr;
    const i2c_master_state_t *master;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_i2cIrqId[instance]);

    /* Destroy the semaphore */
    (void) OSIF_SemaDestroy(&(master->idleSemaphore));

    g_i2cMasterStatePtr[instance] = NULL;

    /* Disable master */
    I2C_Set_ModuleEnable(baseAddr, false);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveDeinit
 * Description   : de-initialize the I2C slave mode driver
 * Implements    : I2C_DRV_SlaveDeinit_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveDeinit(uint32_t instance)
{
    I2C_Type *baseAddr;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    baseAddr = g_i2cBase[instance];
    const i2c_slave_state_t *slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* Disable i2c interrupt */
    INT_SYS_DisableIRQ(g_i2cIrqId[instance]);

    /* Destroy the semaphore */
    (void) OSIF_SemaDestroy(&(slave->idleSemaphore));

    g_i2cSlaveStatePtr[instance] = NULL;

    /* Disable I2C slave */
    I2C_Set_ModuleEnable(baseAddr, false);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 * Implements    : I2C_DRV_MasterAbortTransferData_Activity
 *END**************************************************************************/
status_t I2C_DRV_MasterAbortTransferData(uint8_t instance)
{
    I2C_Type *baseAddr;
    i2c_master_state_t *master;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    baseAddr = g_i2cBase[instance];
    master = g_i2cMasterStatePtr[instance];
    DEV_ASSERT(master != NULL);

    /* Check if driver is busy */
    if(master->i2cIdle)
    {
    	return STATUS_SUCCESS;
    }

    /* End transfer: force stop generation */
    master->status = STATUS_I2C_ABORTED;

    I2C_DRV_GenerateStopSignal(instance);

    I2C_DRV_MasterEndTransfer(baseAddr, master);

    /* Software reset of the I2C module for stop generation */
    I2C_Set_ModuleEnable(baseAddr, false);
    I2C_Set_ModuleEnable(baseAddr, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveAbortTransferData
 * Description   : abort a non-blocking I2C Master transmission or reception
 * Implements    : I2C_DRV_SlaveAbortTransferData_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_SlaveAbortTransferData(uint32_t instance)
{
    i2c_slave_state_t *slave;
    I2C_Type *baseAddr;

    DEV_ASSERT(instance < I2C_INSTANCE_COUNT);

    baseAddr = g_i2cBase[instance];
    slave = g_i2cSlaveStatePtr[instance];
    DEV_ASSERT(slave != NULL);

    /* Check if slave is busy */
    if(!slave->isTransferInProgress)
    {
    	return STATUS_SUCCESS;
    }

    if (!slave->slaveListening)
    {
        I2C_DRV_SlaveEndTransfer(baseAddr, slave);

        /* Deactivate events */
        I2C_Set_BusIRQEnable(baseAddr, false);
        I2C_Set_BusIdleIRQ(baseAddr, false);

        slave->status = STATUS_I2C_ABORTED;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_BaudRateFindDivider
 * Description   : find divider for baud rate
 *
 *END**************************************************************************/
static void I2C_DRV_BaudRateFindDivider(uint32_t divider, uint32_t * valReg)
{
	uint32_t difMin;
	uint32_t difMax;
	uint32_t i;

	/* Find in table the value which match the divider */
	for (i = 0; i < (NO_I2C_SCL_DIVIDERS - 2U); i++)
	{
		if ((divider >= g_i2cDividers[i][1U]) && (divider < g_i2cDividers[i + 1U][1U]))
		{
			/* Check which value is closer to measured divider */
			difMin = (uint32_t) (divider - g_i2cDividers[i][1U]);
			difMax = (uint32_t) (g_i2cDividers[i + 1U][1U] - divider);

			if (difMin > difMax)
			{
				/* Divider is closer to the right value */
				*valReg = g_i2cDividers[i + 1U][0U];
			}
			else
			{
				/* Divider is closer to the left value */
				*valReg = g_i2cDividers[i][0U];
			}

			break;
		}
	}

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterSetBaudRate
 * Description   : set the master baudRate
 * Implements    : I2C_DRV_MasterSetBaudRate_Activity
 *
 *END**************************************************************************/
status_t I2C_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate)
{
	I2C_Type *baseAddr;
	uint32_t divider;
	uint32_t valReg;
	uint32_t inputClock;
	status_t clkStatus;

	baseAddr = g_i2cBase[instance];

	/* Get the protocol clock frequency */
	clkStatus = CLOCK_SYS_GetFreq(g_i2cClock[instance], &inputClock);
	DEV_ASSERT(clkStatus == STATUS_SUCCESS);
	DEV_ASSERT(inputClock > 0U);
	(void) clkStatus;

	/* Calculate divider needed for the requested baudRate */
	divider = (uint32_t)((inputClock + (baudRate >> 1))/baudRate);
    valReg = 1U;

	/* Divider takes the smallest value from the table if the calculated value is smaller than all dividers found in table */
	if (divider < g_i2cDividers[0U][1U])
	{
		valReg = g_i2cDividers[0U][0U];
	}
	else if (divider >= g_i2cDividers[NO_I2C_SCL_DIVIDERS - 1U][1U])
	{
		/* Divider takes biggest value from the table if the calculated value is bigger than all the dividers found in table */
		valReg = g_i2cDividers[NO_I2C_SCL_DIVIDERS - 1U][0U];
	}
	else
	{
		I2C_DRV_BaudRateFindDivider(divider, &valReg);
	}

    baseAddr->IBFD = (uint8_t)valReg;

	return STATUS_SUCCESS;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterGetBaudRate
 * Description   : get the master baudRate
 * Implements    : I2C_DRV_MasterGetBaudRate_Activity
 *
 *END**************************************************************************/
uint32_t I2C_DRV_MasterGetBaudRate(uint32_t instance)
{
	const I2C_Type *baseAddr;
	status_t clkStatus;
	uint8_t regDivider;
	uint32_t inputClock;
	uint32_t divider;
	uint32_t baudRate;
	uint32_t i;

	baseAddr = g_i2cBase[instance];

	/* Get the protocol clock frequency */
	clkStatus = CLOCK_SYS_GetFreq(g_i2cClock[instance], &inputClock);
	DEV_ASSERT(clkStatus == STATUS_SUCCESS);
	DEV_ASSERT(inputClock > 0U);
	(void) clkStatus;

	regDivider = baseAddr->IBFD;
	divider = 1U;

	/* Check in the I2C dividers table the divider associated with the value of IBFD register */
	for(i=0; i<NO_I2C_SCL_DIVIDERS; i++)
	{
		if(g_i2cDividers[i][0U] == (uint32_t)regDivider)
		{
			divider = g_i2cDividers[i][1U];

			break;
		}
	}

	baudRate = (uint32_t)(inputClock/divider);

	return baudRate;
}

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterArbitrationLostHandler
 * Description   : handle master arbitration lost event
 *
 *END**************************************************************************/
static void I2C_DRV_MasterArbitrationLostHandler(uint32_t instance)
{
	i2c_master_state_t * master;

    I2C_Type *baseAddr;
    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    master = g_i2cMasterStatePtr[instance];

    /* Clear the master arbitration lost flag */
    I2C_Clear_MasterArbitrationLostFlag(baseAddr);

    master->status = STATUS_I2C_ARBITRATION_LOST;

    /* End transfer */
    I2C_DRV_MasterEndTransfer(baseAddr, master);

    master->stopGenerated = true;

    if (master->masterCallback != NULL)
    {
        master->masterCallback(I2C_MASTER_EVENT_END_TRANSFER, master->callbackParam);
    }

    /* Signal transfer end for blocking transfers */
    if (master->blocking == true) {
        (void) OSIF_SemaPost(&(master->idleSemaphore));
    }

    if(master->transferType == I2C_USING_DMA)
    {
    	 I2C_Set_DMAEnable(baseAddr, false);
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_GetBusBusyEvent
 * Description   : handle bus busy event
 *
 *END**************************************************************************/
static void I2C_DRV_BusBusyEventHandler(uint32_t instance)
{
	i2c_master_state_t * master;
    I2C_Type *baseAddr;

    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    master = g_i2cMasterStatePtr[instance];

    /* Report success if no error was recorded */
    if (master->status == STATUS_BUSY)
    {
        master->status = STATUS_SUCCESS;
    }

    I2C_DRV_MasterEndTransfer(baseAddr, master);

    if (master->masterCallback != NULL)
    {
        master->masterCallback(I2C_MASTER_EVENT_END_TRANSFER, master->callbackParam);
    }

    /* Signal transfer end for blocking transfers */
    if (master->blocking == true) {
        (void) OSIF_SemaPost(&(master->idleSemaphore));
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveTransmitDataEvent
 * Description   : handle slave transmit data event
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveTransmitDataEvent(uint32_t instance)
{
    i2c_slave_state_t * slave;
    I2C_Type *baseAddr;

    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    slave = g_i2cSlaveStatePtr[instance];

    /* Check the acknowledge from master */
    if (I2C_Get_ReceivedACK(baseAddr))
    {
        /* Transmit next byte to master */
        I2C_DRV_SlaveSendDataEvent(baseAddr, slave);
    }
    else
    {
        /* Switch to RX mode and dummy read */
        I2C_Set_ModuleTransmitMode(baseAddr, false);
        (void) I2C_Get_DataIBDR(baseAddr);
    }

}


/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterToSlaveHandler
 * Description   : master to slave handler
 *
 *END**************************************************************************/
static void I2C_DRV_MasterToSlaveHandler(uint8_t instance)
{

    const I2C_Type *baseAddr;
    baseAddr = g_i2cBase[instance];

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
   	if(I2C_Get_MasterArbitrationLostEvent(baseAddr))
   	{
   		I2C_DRV_MasterArbitrationLostHandler(instance);
   	}
#endif
   	if (!I2C_Get_BusBusyEvent(baseAddr))
    {
   		I2C_DRV_BusBusyEventHandler(instance);
    }

}

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveArbitrationLostHandler
 * Description   : handle slave arbitration lost handler
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveArbitrationLostHandler(uint8_t instance)
{
    i2c_slave_state_t * slave;
    I2C_Type *baseAddr;

    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    slave = g_i2cSlaveStatePtr[instance];

    /* Clear the arbitration lost flag */
    I2C_Clear_MasterArbitrationLostFlag(baseAddr);

    /* Check address as slave flag */
    if (I2C_Get_AddressAsSlave(baseAddr))
    {
        /* It was an address transfer */
        I2C_DRV_AddressAsSlaveEvent(baseAddr, slave);
    }
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveTransferHandler
 * Description   : handle slave receive/transmit event
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveTransferHandler(uint8_t instance)
{
    i2c_slave_state_t * slave;
    I2C_Type *baseAddr;

    baseAddr = g_i2cBase[instance];
    slave = g_i2cSlaveStatePtr[instance];

    /* Module is transmitting or receiving */
    if (I2C_Get_ModuleTransmitMode(baseAddr))
    {

    	I2C_DRV_SlaveTransmitDataEvent(instance);
    }
    else
    {
    	if(I2C_Get_TCF(baseAddr))
    	{
            /* Read data from IBDR and store it */
            I2C_DRV_SlaveReceiveDataEvent(instance, baseAddr, slave);
    	}
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveHandleTransferEvents
 * Description   : handle slave transfer events
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveHandleTransferEvents(uint8_t instance)
{
    i2c_slave_state_t * slave;
    I2C_Type *baseAddr;

    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    slave = g_i2cSlaveStatePtr[instance];

    /* Check if slave is addressed as slave */
    if (I2C_Get_AddressAsSlave(baseAddr))
    {
        /* It was an address transfer */
        I2C_DRV_AddressAsSlaveEvent(baseAddr, slave);
    }
    else if(!I2C_Get_BusBusyEvent(baseAddr))
    {
        /* Report success if no error was recorded */
        if(slave->status == STATUS_BUSY)
        {
            I2C_DRV_SlaveStopEvent(baseAddr, slave, STATUS_SUCCESS);
        }
    }
    else
    {
    	I2C_DRV_SlaveTransferHandler(instance);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_SlaveIRQHandler
 * Description   : handle slave I2C module interrupts
 *
 *END**************************************************************************/
static void I2C_DRV_SlaveIRQHandler(uint8_t instance)
{
    const i2c_master_state_t * master;

    const I2C_Type *baseAddr;
    baseAddr = g_i2cBase[instance];

    /* If the module is slave */
    master = g_i2cMasterStatePtr[instance];

    if (master != NULL)
    {
    	/* Master arbitration lost event */
    	I2C_DRV_MasterToSlaveHandler(instance);
    }
    else
    {
#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
        /* Check arbitration lost event */
        if (I2C_Get_MasterArbitrationLostEvent(baseAddr))
        {
        	I2C_DRV_SlaveArbitrationLostHandler(instance);
        }
        else
        {
#endif
            /* No arbitration lost event */
            I2C_DRV_SlaveHandleTransferEvents(instance);

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
        }
#endif
    }

    /* Cast to void to avoid compiler warnings */
    (void) baseAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterLastByteTransmitted
 * Description   : Master last byte transmitted handler
 *
 *END**************************************************************************/
static void I2C_DRV_MasterLastByteTransmitted(uint8_t instance)
{
	i2c_master_state_t * master;

	I2C_Type *baseAddr;
    baseAddr = g_i2cBase[instance];

    master = g_i2cMasterStatePtr[instance];

    if (master->sendStop)
    {
        /* Generate stop signal */
        I2C_DRV_GenerateStopSignal(instance);
    }
    else
    {
        /* Generate repeated start */
        I2C_Set_RepeatStart(baseAddr, true);
        master->stopGenerated = false;
        master->i2cIdle = true;
    }

    /* No more data in buffer, end transmission */
    I2C_DRV_MasterEndTransfer(baseAddr, master);

    if (master->masterCallback != NULL) {
        master->masterCallback(I2C_MASTER_EVENT_END_TRANSFER, master->callbackParam);
    }

    master->status = STATUS_SUCCESS;

    /* Signal transfer end for blocking transfers */
    if (master->blocking == true)
    {
        (void) OSIF_SemaPost(&(master->idleSemaphore));
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveAckEvent
 * Description   : Master receive acknowledge event
 *
 *END**************************************************************************/
static void I2C_DRV_MasterReceiveAckEvent(uint32_t instance)
{
	i2c_master_state_t * master;

	I2C_Type *baseAddr;
	baseAddr = g_i2cBase[instance];

    master = g_i2cMasterStatePtr[instance];

    /* End of address cycle? */
    if (master->sendData)
    {
        /* Send the next byte of data */
        I2C_DRV_MasterSendDataEvent(instance, baseAddr, master);
    }
    else
    {
        /* If is the end of address cycle and master is in RX mode */
        I2C_Set_ModuleTransmitMode(baseAddr, master->sendData);

        /* Dummy read from IBDR */
        (void) I2C_Get_DataIBDR(baseAddr);

        if ((master->transferType == I2C_USING_DMA)
                && (!master->sendData))
        {
            I2C_Set_DMAEnable(baseAddr, true);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterGenerateStopEvent
 * Description   : Master generate stop signal event
 *
 *END**************************************************************************/
static void I2C_DRV_MasterGenerateStopEvent(uint8_t instance)
{
	i2c_master_state_t * master;

	I2C_Type *baseAddr;
	baseAddr = g_i2cBase[instance];

    master = g_i2cMasterStatePtr[instance];

    /* Generate Stop Signal */
    I2C_DRV_GenerateStopSignal(instance);

    master->status = STATUS_I2C_RECEIVED_NACK;

     /* Master End Transfer */
     I2C_DRV_MasterEndTransfer(baseAddr, master);

     if (master->masterCallback != NULL)
     {
         master->masterCallback(I2C_MASTER_EVENT_END_TRANSFER, master->callbackParam);
     }

     /* Signal transfer end for blocking transfers */
     if (master->blocking == true)
     {
         (void) OSIF_SemaPost(&(master->idleSemaphore));
     }

     if(master->transferType == I2C_USING_DMA)
     {
     	I2C_Set_DMAEnable(baseAddr, false);
     }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterTransmitHandler
 * Description   : Master transmit mode handler
 *
 *END**************************************************************************/
static void I2C_DRV_MasterTransmitHandler(uint8_t instance)
{
	const i2c_master_state_t * master;
	const I2C_Type *baseAddr;

	baseAddr = g_i2cBase[instance];

    /* Master Mode */

    master = g_i2cMasterStatePtr[instance];

    if(I2C_Get_TCF(baseAddr))
	{
	   /* Last byte transmitted ? */
	   if ((master->txSize == 0U)&&(master->sendData))
	   {
		   I2C_DRV_MasterLastByteTransmitted(instance);
	   }
	   else
	   {
	       /* Receive acknowledge */
	       if (I2C_Get_ReceivedACK(baseAddr))
	       {
	    	   I2C_DRV_MasterReceiveAckEvent(instance);
	       }
	       else
	       {
	    	   I2C_DRV_MasterGenerateStopEvent(instance);
	       }
	   }
	}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterReceiveHandler
 * Description   : Master receive mode handler
 *
 *END**************************************************************************/
static void I2C_DRV_MasterReceiveHandler(uint8_t instance)
{
    /* Master is in receive mode */

	i2c_master_state_t * master;
	I2C_Type *baseAddr;

	baseAddr = g_i2cBase[instance];

    master = g_i2cMasterStatePtr[instance];

    /* If last byte to be read */
    if (master->rxSize == 1U)
    {
        if (master->sendStop)
        {
            /* Generate Stop Signal */
            I2C_DRV_GenerateStopSignal(instance);
        }
        else
        {
            /* Generate repeated start */
            I2C_Set_RepeatStart(baseAddr, true);
            master->stopGenerated = false;
            master->i2cIdle = true;

            /* Signal transfer end for blocking transfers */
            if (master->blocking == true)
            {
                (void) OSIF_SemaPost(&(master->idleSemaphore));
            }

            master->status = STATUS_SUCCESS;
        }
    }
    else
    {
        /* If second last byte to be read */
        if (master->rxSize == 2U)
        {
            /* Set noAck = 1 */
            I2C_Set_DataAcknowledge(baseAddr, false);
        }
    }
    /* Read data from IBDR and store */
    I2C_DRV_MasterReceiveDataEvent(instance, baseAddr, master);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_MasterIRQHandler
 * Description   : handle master I2C module interrupts
 *
 *END**************************************************************************/
static void I2C_DRV_MasterIRQHandler(uint8_t instance)
{
	const I2C_Type *baseAddr;
	baseAddr = g_i2cBase[instance];

    /* Check the mode - receive or transmit */
    if (I2C_Get_ModuleTransmitMode(baseAddr))
    {
    	I2C_DRV_MasterTransmitHandler(instance);
    }
    else
    {
    	I2C_DRV_MasterReceiveHandler(instance);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C_DRV_ModuleIRQHandler
 * Description   : handle master/slave I2C module interrupts
 *
 *END**************************************************************************/
void I2C_DRV_ModuleIRQHandler(uint8_t instance)
{
    I2C_Type *baseAddr;
    baseAddr = g_i2cBase[instance];

    /* Clear I-Bus Interrupt */
    I2C_Clear_BusInterruptFlag(baseAddr, true);

    /* Check if is master or slave module */
    if (I2C_Get_ModuleModSelect((const I2C_Type *)baseAddr) == false)
    {
    	I2C_DRV_SlaveIRQHandler(instance);
    }
    else
    {
        I2C_DRV_MasterIRQHandler(instance);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
