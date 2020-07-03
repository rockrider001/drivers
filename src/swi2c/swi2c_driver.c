/*
 * Copyright 2017 NXP
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
 * @swi2c_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 * Impermissible cast; cannot cast from 'essentially enum<i>' to 'essentially Boolean'. This is required by the
 * conversion of a enum into a bool.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially Boolean'. This is required by the
 * conversion of a uint_16 into a bool.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (different essential type categories).
 * This is required by the conversion of a bit-field into enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used in case a TIMEOUT happened. In case
 * of a TIMEOUT the program should return from the function and return STATUS_TIMEOUT.
 */


#include "swi2c_driver.h"
#include "pins_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Values of the swi2c pins */
typedef enum
{
    LOW = 0U,                                /* The pin is low */
    HIGH = 1U                                /* The pin is high */
}swi2c_pinValue_t;

/*! @brief Acknowledge from swi2c module */
typedef enum
{
   NACK = 0,                                 /* NACK is send */
   ACK = 1                                   /* ACK is send */
}swi2c_ack_type_t;

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterInit
 * Description   : initialize the SWI2C master mode driver
 * Implements    : SWI2C_DRV_MasterInit_Activity
 *
 *END**************************************************************************/
status_t SWI2C_DRV_MasterInit(swi2c_master_state_t *master,
        const swi2c_master_user_config_t *userConfigPtr)
{
    /* Check input parameters */
    DEV_ASSERT(master != NULL);
    DEV_ASSERT(userConfigPtr != NULL);

    /* Initialize pins */
    master->sclPin = userConfigPtr->sclPin;
    master->sdaPin = userConfigPtr->sdaPin;
    master->sdaReadPin = userConfigPtr->sdaReadPin;
    master->sclReadPin = userConfigPtr->sclReadPin;

    /* Set SDA and SCL HIGH */
    PINS_DRV_SetPins(master->sdaPin->port, (uint16_t)(1U << master->sdaPin->pinNumber));
    PINS_DRV_SetPins(master->sclPin->port, (uint16_t)(1U << master->sclPin->pinNumber));

    /* Initialize driver status structure */
    master->status = STATUS_SUCCESS;
    master->slaveAddress = userConfigPtr->slaveAddress;

    /* Initialize the baudRate */
    master->baudRate = userConfigPtr -> baudRate;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_GetPinValue
 * Description   : Gets the pin value if the pin is configured as input
 *
 *END**************************************************************************/
static bool SWI2C_DRV_GetPinValue(const GPIO_Type *base, uint16_t pinNum)
{
    uint16_t value;
    value = PINS_DRV_ReadPins(base);
    value = (((value) & ((uint16_t)1U << pinNum))) >> pinNum;

    return (bool)value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterSendAddress
 * Description   : Sets the 7-bit address of the slave which will be used for
 *                 any future transfers initiated by the SWI2C master.
 * Implements    : SWI2C_DRV_MasterSetSlaveAddress_Activity
 *
 *END**************************************************************************/
void SWI2C_DRV_MasterSetSlaveAddress(swi2c_master_state_t *master, uint8_t address)
{
    /* Check if master state structure is null */
    DEV_ASSERT(master != NULL);

    master->slaveAddress = address;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_CheckSignalValue
 * Description   : Check if SDA or SCL signals have the desire value
 *
 *END**************************************************************************/
static uint32_t SWI2C_DRV_CheckSignalValue(const swi2c_pin_t *pin, swi2c_pinValue_t value)
{
    uint32_t timeout = WAITING_CYCLES;

    /* Wait until the pin is LOW */
    while((SWI2C_DRV_GetPinValue(pin->port, pin->pinNumber) == !((bool)value)) && (timeout > 0U))
    {
            timeout--;
    }

    return timeout;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_WaitSclPeriod
 * Description   : Use to generate the SCL signal. The SCL signal is kept HIGH or LOW
 *                 during this function.
 *
 *END**************************************************************************/
static void SWI2C_DRV_WaitSclPeriod(const swi2c_master_state_t *master)
{
    volatile uint32_t i;
    /* Cycles in which SCL is kept HIGH or LOW*/
    for(i=0;i<(master->baudRate);i++){}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_GenerateStartSignal
 * Description   : Generates start signal
 *
 *END**************************************************************************/
static status_t SWI2C_DRV_GenerateStartSignal(const swi2c_master_state_t *master)
{
    uint32_t timeout;

    /* Check if SDA and SCL lines are HIGH */
    timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    timeout = SWI2C_DRV_CheckSignalValue(master->sdaReadPin, HIGH);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    /* SDA and SCL transition to LOW */
    PINS_DRV_ClearPins(master->sdaPin->port, (uint16_t)(1U << master->sdaPin->pinNumber));

    /* Check if SDA line is LOW for a proper start condition */
    timeout = SWI2C_DRV_CheckSignalValue(master->sdaReadPin, LOW);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    /* Wait a SCL period before clearing SCL pin*/
    SWI2C_DRV_WaitSclPeriod(master);

    PINS_DRV_ClearPins(master->sclPin->port, (uint16_t)(1U << master->sclPin->pinNumber));

    /* Check if SCL is LOW */
    timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, LOW);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_GenerateStopSignal
 * Description   : Generates stop signal for swi2c master module
 *
 *END**************************************************************************/
static status_t SWI2C_DRV_GenerateStopSignal(const swi2c_master_state_t *master)
{
    uint32_t timeout;

    DEV_ASSERT(master != NULL);

    /* SDA transition to LOW */
    PINS_DRV_ClearPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));

    /* SCL transition to HIGH */
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

    /* Wait the SCL line to get HIGH */
    timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);

    if (timeout > 0U)
    {
    	/* Wait a SCL period before setting SDA to HIGH */
    	SWI2C_DRV_WaitSclPeriod(master);

        /* SDA transition to HIGH */
        PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));
    }

    /* Idle state for SDA and SCL line */
    PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_SendByte
 * Description   : Sends one byte of data to a slave module. This function is implemented
 *                 only for the SWI2C master module.
 *
 *END**************************************************************************/
static status_t SWI2C_DRV_MasterSendByte(swi2c_master_state_t *master, uint8_t data)
{
    uint8_t i;
    uint8_t shift;
    uint16_t ack;
    uint32_t timeout;

    DEV_ASSERT(master != NULL);

    shift = 0x80;

    /* Write 8 bits of data */
    for (i = 0; i < 8U; i++)
    {
        /* Check if SCL signal is LOW */
        timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, LOW);

        if (timeout > 0U)
        {
            if((data & shift) > 0U)
            {
                /* Set SDA signal HIGH */
                PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << (master ->sdaPin->pinNumber)));
            }
            else
            {
                /* Set SDA signal to LOW */
                PINS_DRV_ClearPins(master->sdaPin->port, (pins_channel_type_t)(1U << (master->sdaPin->pinNumber))); /* SDA LOW */
            }
        }
        else
        {
            /* Master get status timeout */
            master->status = STATUS_TIMEOUT;

            return master->status;
        }

        shift = (uint8_t)(shift >> 1);

        SWI2C_DRV_WaitSclPeriod(master);

        /* Generate SCL HIGH signal */
        PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << (master->sclPin->pinNumber)));

        /* Wait until the signal is HIGH in case slave is clock stretching the SCL signal */
       timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);
       if(timeout == 0U)
       {
           /* Master get status timeout */
           master->status = STATUS_TIMEOUT;

           return master->status;
       }

        SWI2C_DRV_WaitSclPeriod(master);

        /* Generate SCL LOW signal */
        PINS_DRV_ClearPins(master->sclPin->port, (pins_channel_type_t)(1U << (master->sclPin->pinNumber))); /* SCL LOW */
    }

    /* SDA should remain HIGH */
    PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << (master->sdaPin->pinNumber)));

    SWI2C_DRV_WaitSclPeriod(master);

    /* Generate last front of clock */
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << (master->sclPin->pinNumber)));

    /* Wait in case of clock stretching from slave */
    (void)SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);

    /* Read the acknowledge pin */
    ack = (uint16_t)SWI2C_DRV_GetPinValue(master->sdaReadPin->port, (uint16_t)master->sdaReadPin->pinNumber);

    /* If acknowledge pin is HIGH than slave sent NACK */
    if (ack > 0U)
    {
        master->status = STATUS_I2C_RECEIVED_NACK;
    }

    SWI2C_DRV_WaitSclPeriod(master);

    /* SCL transition to LOW */
    PINS_DRV_ClearPins(master->sclPin->port, (pins_channel_type_t)(1U << (master->sclPin->pinNumber)));

    return master->status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterTransmitAddress
 * Description   : master module sends on I2C BUS the slave address and checks for
 *                 acknowledge
 *END**************************************************************************/
static status_t SWI2C_DRV_MasterTransmitAddress(swi2c_master_state_t *master)
{
    uint8_t address;
    status_t retCode;

    DEV_ASSERT(master != NULL);

    /* Generate start signal */
    retCode = SWI2C_DRV_GenerateStartSignal(master);
    if(retCode == STATUS_TIMEOUT)
    {
    	master->status = STATUS_TIMEOUT;
        return retCode;
    }

    /* Transmit address */
    address = (uint8_t) ((master->slaveAddress << 1U) + (uint8_t) (!master->sendData));
    retCode = SWI2C_DRV_MasterSendByte(master, address);

    return retCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterSendDataBlocking
 * Description   : Perform a blocking send transaction on the I2C buffer
 * Implements    : SWI2C_DRV_MasterSendDataBlocking_Activity
 *
 *END**************************************************************************/
status_t SWI2C_DRV_MasterSendDataBlocking(swi2c_master_state_t *master, const uint8_t *txBuff, uint32_t txSize, bool sendStop)
{
    status_t retCode;
    uint32_t i;

    DEV_ASSERT(txBuff != NULL);
    DEV_ASSERT(txSize > 0U);

    DEV_ASSERT(master != NULL);

    /* Copy parameters to drive state structure */
    master->status = STATUS_BUSY;
    master->sendData = true;

    /* Transmit address */
    retCode = SWI2C_DRV_MasterTransmitAddress(master);
    if(retCode != STATUS_BUSY)
    {
    	/* Master end transfer */
    	(void)SWI2C_DRV_GenerateStopSignal(master);

        return master->status;
    }

    /* Send to slave one byte at a time */
    for(i=0U; i<txSize; i++)
    {
       retCode = SWI2C_DRV_MasterSendByte(master, txBuff[0]);

       if(retCode != STATUS_BUSY)
       {
          (void)SWI2C_DRV_GenerateStopSignal(master);
       }

        txBuff++;
    }

    SWI2C_DRV_WaitSclPeriod(master);

    /* Generate Stop */
    if(sendStop)
    {
        (void)SWI2C_DRV_GenerateStopSignal(master);
    }

    /* Free the BUS */
    PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t) (1U << master->sdaPin->pinNumber));
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t) (1U << master->sclPin->pinNumber));

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterSendACK
 * Description   : master module sends ACK to slave module
 *
 *END**************************************************************************/
static status_t SWI2C_DRV_MasterSendACK(const swi2c_master_state_t *master, swi2c_ack_type_t ack)
{
    uint32_t timeout;

    if((uint16_t)ack > 0U)
    {
        /* Send acknowledge to slave module */
        PINS_DRV_ClearPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));

        /* Verify if SDA is low */
        timeout = SWI2C_DRV_CheckSignalValue(master->sdaReadPin, LOW);
        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }
    else
    {
        /* Send acknowledge to slave module */
        PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));

        /* Verify if SDA is low */
        timeout = SWI2C_DRV_CheckSignalValue(master->sdaReadPin, HIGH);
        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    SWI2C_DRV_WaitSclPeriod(master);

    /* Set SCL to HIGH */
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

    /* Verify if SCL is set  */
    timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    SWI2C_DRV_WaitSclPeriod(master);

    /* Clear SCL pin */
    PINS_DRV_ClearPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

    /* Verify if SCL is cleared  */
    timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, LOW);
    if(timeout == 0U)
    {
        return STATUS_TIMEOUT;
    }

    /* Set SDA to HIGH */
    PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterReadByte
 * Description   : Perform a blocking receive transaction on the I2C bus.
 *
 *END**************************************************************************/
static uint8_t SWI2C_DRV_MasterReadByte(swi2c_master_state_t *master)
{
    uint8_t shift;
    uint8_t i;
    uint32_t timeout;

    DEV_ASSERT(master != NULL);
    shift = 0U;

    /* Read 8 bits of data sent from I2C slave module  */
    for(i = 0; i<8U; i++)
    {
        SWI2C_DRV_WaitSclPeriod(master);

        /* SCL transition to HIGH */
        PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << (master->sclPin->pinNumber)));

        /* Check if SCL is HIGH */
        timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, HIGH);
        if(timeout == 0U)
        {
            master->status = STATUS_TIMEOUT;
        }

        shift = (uint8_t)(shift << 1U);

        /* Read the SDA line */
        if ((uint8_t)SWI2C_DRV_GetPinValue(master->sdaReadPin->port, master->sdaReadPin->pinNumber) > 0U)
        {
            shift++;
        }

        SWI2C_DRV_WaitSclPeriod(master);

        /* Clear SCL pin */
        PINS_DRV_ClearPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

        /* Check if SCL is LOW */
        timeout = SWI2C_DRV_CheckSignalValue(master->sclReadPin, LOW);
        if(timeout == 0U)
        {
        	master->status = STATUS_TIMEOUT;
        }
    }

    return shift;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_MasterReceiveDataBlocking
 * Description   : perform a blocking send transaction on the I2C bus
 * Implements    : SWI2C_DRV_MasterReceiveDataBlocking_Activity
 *
 *END**************************************************************************/
status_t SWI2C_DRV_MasterReceiveDataBlocking(swi2c_master_state_t *master, uint8_t *rxBuff, uint32_t rxSize, bool sendStop)
{
    uint32_t i;
    status_t retCode;

    /* Check parameters*/
    DEV_ASSERT(rxBuff != NULL);
    DEV_ASSERT(rxSize > 0U);

    DEV_ASSERT(master != NULL);

    /* Copy parameters to driver state structure */
    master->status = STATUS_BUSY;
    master->sendData = false;

    /* Transmit address */
    retCode = SWI2C_DRV_MasterTransmitAddress(master);
    if(retCode != STATUS_BUSY)
    {
    	/* Master end transfer */
    	(void)SWI2C_DRV_GenerateStopSignal(master);

        return retCode;
    }

    /* Receive data buffer */
    for(i=0; i<rxSize; i++)
    {
        rxBuff[0U] = SWI2C_DRV_MasterReadByte(master);

        if(master->status != STATUS_BUSY)
        {
        	/* Generate stop */
        	(void)SWI2C_DRV_GenerateStopSignal(master);

        	return master->status;

        }

        if(i == (rxSize - 1U))
        {
           (void)SWI2C_DRV_MasterSendACK(master, NACK);
        }
        else
        {
           (void)SWI2C_DRV_MasterSendACK(master, ACK);
        }
        rxBuff++;
    }

    if (sendStop)
    {
        /* Generate stop */
        (void)SWI2C_DRV_GenerateStopSignal(master);
    }

    /* Free the BUS */
    PINS_DRV_SetPins(master->sdaPin->port, (pins_channel_type_t)(1U << master->sdaPin->pinNumber));
    PINS_DRV_SetPins(master->sclPin->port, (pins_channel_type_t)(1U << master->sclPin->pinNumber));

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWI2C_DRV_SetWaitTimeForSCLTransition
 * Description   : sets the wait time in cycles for the SCL transition
 * Implements    : SWI2C_DRV_SetWaitTimeForSCLTransition_Activity
 *
 *END**************************************************************************/
void SWI2C_DRV_SetWaitTimeForSCLTransition(swi2c_master_state_t *master, uint32_t cycles)
{
    /* Check if master state structure is not null */
    DEV_ASSERT(master != NULL);

    master->baudRate = cycles;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
