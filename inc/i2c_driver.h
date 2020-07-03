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

#if !defined(I2C_DRIVER_H)
#define I2C_DRIVER_H

#include <stddef.h>
#include <stdbool.h>
#include "status.h"
#include "edma_driver.h"
#include "osif.h"
#include "device_registers.h"
#include "clock_manager.h"
#include "callbacks.h"

/* */
/* */
/* */

/*!
 * @addtogroup i2c_drv I2C Driver
 * @ingroup i2c
 * @brief Inter-Integrated Circuit Driver
 * @{
 */

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*! @brief Type of I2C transfer (based on interrupts or DMA).
 * Implements : i2c_transfer_type_t_Class
 */
typedef enum
{
   I2C_USING_DMA         = 0,    /*!< The driver will use DMA to perform I2C transfer */
   I2C_USING_INTERRUPTS  = 1,    /*!< The driver will use interrupts to perform I2C transfer */
}  i2c_transfer_type_t;

/*!
 * @brief Slave configuration structure
 *
 * This structure is used to provide configuration parameters for the I2C slave at initialization time.
 * Implements : i2c_slave_user_config_t_Class
 */
typedef struct
{
    uint8_t slaveAddress;                     /*!< 7 bit slave address */
    bool slaveListening;                      /*!< Slave mode (always listening or on demand only) */
    i2c_slave_callback_t slaveCallback;       /*!< Slave callback function */
    void *callbackParam;                      /*!< Parameter for the slave callback function */
} i2c_slave_user_config_t;

/*!
 * @brief Slave internal context structure
 *
 * This structure is used by the slave-mode driver for its internal logic. It must
 * be provided by the application through the I2C_DRV_SlaveInit() function.
 * The application should make no assumptions about the content of this structure.
 *
 * Implements: i2c_slave_state_t_Class
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    status_t status;                        /* The I2C slave status */
    volatile bool isTransferInProgress;     /* Slave is busy because of an ongoing transfer */
    uint32_t txSize;                        /* Size of the TX buffer */
    uint32_t rxSize;                        /* Size of the RX buffer */
    const uint8_t * txBuff;                 /* Pointer to Tx Buffer */
    uint8_t * rxBuff;                       /* Pointer to Rx Buffer */
    uint8_t slaveAddress;                   /* Slave address */
    bool slaveListening;                    /* Slave mode (always listening or on demand only) */
    semaphore_t idleSemaphore;              /* Semaphore used by blocking functions */
    bool txUnderrunWarning;                 /* Possible slave tx underrun */
    bool dataAck;                           /* Data acknowledge enable or disable*/
    bool blocking;                          /* Specifies if the current transfer is blocking */
    i2c_slave_callback_t slaveCallback;     /* Slave callback function */
    void *callbackParam;                    /* Parameter for the slave callback function */
/*! @endcond */
} i2c_slave_state_t;

/*!
 * @brief Master configuration structure
 *
 * This structure is used to provide configuration parameters for the I2C master at initialization time.
 * Implements : i2c_master_user_config_t_Class
 */
typedef struct
{
    uint8_t slaveAddress;                       /*!< Slave address, 7-bit */
    uint32_t baudRate;                           /*!< The baud rate in hertz to use with current slave device */
    i2c_transfer_type_t transferType;           /*!< Type of I2C transfer */
    uint8_t dmaChannel;                         /*!< Channel number for DMA channel. If DMA mode isn't used this field will be ignored */
    i2c_master_callback_t masterCallback;       /*!< Master callback function. Note that this function will be
                                                     called from the interrupt service routine at the end of a transfer,
                                                     so its execution time should be as small as possible. It can be
                                                     NULL if you want to check manually the status of the transfer. */
    void *callbackParam;                        /*!< Parameter for the master callback function */
} i2c_master_user_config_t;

/*!
 * @brief Master internal context structure
 *
 * This structure is used by the master-mode driver for its internal logic. It must
 * be provided by the application through the I2C_DRV_MasterInit() function.
 * The application should make no assumptions about the content of this structure.
 * Implements: i2c_master_state_t_Class
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */

    uint8_t * rxBuff;                       /* Pointer to receive data buffer */
    uint32_t rxSize;                        /* Size of receive data buffer */
    const uint8_t * txBuff;                 /* Pointer to transmit data buffer */
    uint32_t txSize;                        /* Size of transmit data buffer */
    volatile status_t status;               /* Status of last driver operation */
    uint8_t slaveAddress;                   /* Slave address */
    volatile bool i2cIdle;                  /* Idle/busy state of the driver */
    bool sendStop;                          /* Specifies if STOP condition must be generated after current transfer */
    bool blocking;                          /* Specifies if the current transfer is blocking */
    i2c_transfer_type_t transferType;       /* Type of I2C transfer */
    uint8_t dmaChannel;                     /* Channel number for DMA rx channel */
    bool sendData;                          /* Master is in transmitting or receive mode*/
    bool stopGenerated;                     /* Specifies if stop was generated for the previous transfer */
    semaphore_t idleSemaphore;              /* Semaphore used by blocking functions */
    i2c_master_callback_t masterCallback;   /* Master callback function */
    void *callbackParam;                    /* Parameter for the master callback function */
/*! @endcond */
} i2c_master_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the I2C master mode driver
 *
 * This function initializes the I2C driver in master mode.
 *
 * @param instance  I2C peripheral instance number
 * @param userConfigPtr    Pointer to the I2C master user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param master    Pointer to the I2C master driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using I2C_DRV_MasterDeinit().
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterInit(uint32_t instance,
                                    const i2c_master_user_config_t * userConfigPtr,
                                    i2c_master_state_t * master);

/*!
 * @brief Perform a non-blocking send transaction on the I2C bus
 *
 * This function starts the transmission of a block of data to the currently
 * configured slave address and returns immediately.
 * The rest of the transmission is handled by the interrupt service routine.
 *
 * @param instance  I2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterSendData(uint32_t instance, const uint8_t *txBuff, uint32_t txSize, bool sendStop);

/*!
 * @brief Perform a blocking send transaction on the I2C bus
 *
 * This function sends a block of data to the currently configured slave address, and
 * only returns when the transmission is complete.
 *
 * @param instance  I2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the transmission
 * @param timeout   timeout for the transfer in milliseconds
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterSendDataBlocking(uint32_t instance,
        const uint8_t *txBuff, uint32_t txSize, bool sendStop, uint32_t timeout);

/*!
 * @brief Abort a non-blocking I2C Master transmission or reception
 *
 * @param instance  I2C peripheral instance number
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterAbortTransferData(uint8_t instance);

/*!
 * @brief Perform a non-blocking receive transaction on the I2C bus
 *
 * This function starts the reception of a block of data from the currently
 * configured slave address and returns immediately.
 * The rest of the reception is handled by the interrupt service routine.
 *
 * @param instance  I2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterReceiveData(uint32_t instance, uint8_t *rxBuff, uint32_t rxSize, bool sendStop);

/*!
 * @brief Perform a blocking receive transaction on the I2C bus
 *
 * This function receives a block of data from the currently configured slave address,
 * and only returns when the transmission is complete.
 *
 * @param instance  I2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @param timeout   timeout for the transfer in milliseconds
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterReceiveDataBlocking(uint32_t instance, uint8_t *rxBuff, uint32_t rxSize, bool sendStop, uint32_t timeout);

/*!
 * @brief De-initialize the I2C master mode driver
 *
 * This function de-initializes the I2C driver in master mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance  I2C peripheral instance number
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterDeinit(uint32_t instance);

/*!
 * @brief Return the current status of the I2C master transfer
 *
 * This function can be called during a non-blocking transmission to check the
 * status of the transfer.
 *
 * @param instance  I2C peripheral instance number
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_MasterGetTransferStatus(uint32_t instance);


/*!
 * @brief Set the slave address for the I2C communication
 *
 * This function sets the slave address which will be used for any future
 * transfer initiated by the I2C master.
 *
 * @param instance  I2C peripheral instance number
 * @param address   slave 7-bit address
 */
void I2C_DRV_MasterSetSlaveAddress(uint32_t instance, uint8_t address);


/*!
 * @brief Set the master baud rate for the I2C communication
 *
 * This function sets the master baud rate of the I2C master module.
 *
 * @param instance  I2C peripheral instance number
 * @param baudRate  the desired baud rate in Hz
 */
status_t I2C_DRV_MasterSetBaudRate(uint32_t instance, uint32_t baudRate);

/*!
 * @brief Get the master baud rate for the I2C communication
 *
 * This function returns the master baud rate of the I2C master module.
 *
 * @param instance  I2C peripheral instance number
 * @return    the baud rate in Hz
 */
uint32_t I2C_DRV_MasterGetBaudRate(uint32_t instance);
/*!
 * @brief Initialize the I2C slave mode driver
 *
 * @param instance  I2C peripheral instance number
 * @param userConfigPtr    Pointer to the I2C slave user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param slave     Pointer to the I2C slave driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using I2C_DRV_SlaveDeinit().
 * @return    Error or success status returned by API
 */

status_t I2C_DRV_SlaveInit(uint32_t instance,
                                   const i2c_slave_user_config_t * userConfigPtr,
                                   i2c_slave_state_t * slave);

/*!
 * @brief Perform a non-blocking send transaction on the I2C bus
 *
 * Performs a non-blocking send transaction on the I2C bus when the slave is
 * not in listening mode (initialized with slaveListening = false). It starts
 * the transmission and returns immediately. The rest of the transmission is
 * handled by the interrupt service routine.
 *
 * @param instance  I2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveSendData(uint32_t instance, const uint8_t *txBuff, uint32_t txSize);

/*!
 * @brief Perform a blocking send transaction on the I2C bus
 *
 * Performs a blocking send transaction on the I2C bus when the slave is
 * not in listening mode (initialized with slaveListening = false). It sets
 * up the transmission and then waits for the transfer to complete before
 * returning.
 *
 * @param instance  I2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param timeout   timeout for the transfer in milliseconds
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveSendDataBlocking(uint32_t instance, const uint8_t *  txBuff, uint32_t txSize, uint32_t timeout);

/*!
 * @brief Perform a non-blocking receive transaction on the I2C bus
 *
 * Performs a non-blocking receive transaction on the I2C bus when the slave is
 * not in listening mode (initialized with slaveListening = false). It starts
 * the reception and returns immediately. The rest of the reception is
 * handled by the interrupt service routine.
 *
 * @param instance  I2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveReceiveData(uint32_t instance, uint8_t *rxBuff, uint32_t rxSize);

/*!
 * @brief Perform a blocking receive transaction on the I2C bus
 *
 * Performs a blocking receive transaction on the I2C bus when the slave is
 * not in listening mode (initialized with slaveListening = false). It sets
 * up the reception and then waits for the transfer to complete before
 * returning.
 *
 * @param instance  I2C peripheral instance number
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param timeout   timeout for the transfer in milliseconds
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveReceiveDataBlocking(uint32_t instance, uint8_t *rxBuff, uint32_t rxSize, uint32_t timeout);

/*!
 * @brief Abort a non-blocking I2C Master transmission or reception
 *
 * @param instance  I2C peripheral instance number
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveAbortTransferData(uint32_t instance);

/*!
 * @brief Provide a buffer for receiving data.
 *
 * This function provides a buffer in which the I2C slave-mode driver can
 * store received data. It can be called for example from the user callback provided at
 * initialization time, when the driver reports events I2C_SLAVE_EVENT_RX_REQ or
 * I2C_SLAVE_EVENT_RX_FULL.
 *
 * @param instance  I2C peripheral instance number
 * @param rxBuff    pointer to the data to be transferred
 * @param rxSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveSetRxBuffer(uint8_t instance, uint8_t *rxBuff, uint32_t rxSize);

/*!
 * @brief Provide a buffer for transmitting data
 *
 * This function provides a buffer from which the I2C slave-mode driver can
 * transmit data. It can be called for example from the user callback provided at
 * initialization time, when the driver reports events I2C_SLAVE_EVENT_TX_REQ or
 * I2C_SLAVE_EVENT_TX_EMPTY.
 *
 * @param instance  I2C peripheral instance number
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveSetTxBuffer(uint8_t instance, const uint8_t *txBuff,uint32_t txSize);

/*!
 * @brief Return the current status of the I2C slave transfer
 *
 * This function can be called during a non-blocking transmission to check the
 * status of the transfer.
 *
 * @param instance  I2C peripheral instance number
 * @param bytesRemaining   the number of remaining bytes in the active I2C transfer
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveGetTransferStatus(uint32_t instance);

/*!
 *@brief Slave module de-initialization function
 * This function de-initializes the I2C driver in slave mode. The driver can't be used
 * again until reinitialized. The context structure is no longer needed by the driver and
 * can be freed after calling this function.
 *
 * @param instance I2C peripheral instance number
 * @return    Error or success status returned by API
 */
status_t I2C_DRV_SlaveDeinit(uint32_t instance);

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* I2C_DRIVER_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
