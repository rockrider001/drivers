/*
 * Copyright (c) 2017, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2019 NXP
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
#ifndef DSPI_HEADER_H
#define DSPI_HEADER_H
#include "status.h"
#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"
#include "osif.h"
#include "callbacks.h"
#include <stdio.h>
#include "edma_driver.h"

/* */
/* */
/* */

/*!
 * @addtogroup dspi_driver Deserial Serial Peripheral Interface Driver
 * @ingroup dspi
 * @brief  Deserial Serial Peripheral Interface Driver
 * @{
 */
/*!
 * @brief Defines the instance mapping for DSPI/SPI modules
 *
 * This enum is used to define available instances
 *
 * Implements : dspi_instance_t_Class
 */
#if (defined (CPU_MPC5747C) || defined (CPU_MPC5748C) || defined (CPU_MPC5746G) || defined (CPU_MPC5747G) || defined (CPU_MPC5748G))
typedef enum
{
    DSPI0_INSTANCE = 0, DSPI1_INSTANCE = 1, DSPI2_INSTANCE = 2, DSPI3_INSTANCE = 3, SPI0_INSTANCE = 4, SPI1_INSTANCE = 5, SPI2_INSTANCE = 6,
    SPI3_INSTANCE = 7, SPI4_INSTANCE = 8, SPI5_INSTANCE = 9
} dspi_instance_t;
#endif
#if (defined (CPU_MPC5744B) || defined (CPU_MPC5745B) || defined (CPU_MPC5746B) || defined (CPU_MPC5744C) || defined (CPU_MPC5745C) || \
    defined (CPU_MPC5746C))
typedef enum
{
    DSPI0_INSTANCE = 0, DSPI1_INSTANCE = 1, DSPI2_INSTANCE = 2, DSPI3_INSTANCE = 3, SPI0_INSTANCE = 4, SPI1_INSTANCE = 5, SPI2_INSTANCE = 6,
    SPI3_INSTANCE = 7
} dspi_instance_t;
#endif
#if (defined (CPU_MPC5744P) || defined (CPU_MPC5743P) || defined (CPU_MPC5742P) || defined (CPU_MPC5741P))
typedef enum
{
    SPI0_INSTANCE = 0, SPI1_INSTANCE = 1, SPI2_INSTANCE = 2, SPI3_INSTANCE = 3,
} dspi_instance_t;
#endif
#if (defined(CPU_S32R274) || defined(CPU_S32R372))
typedef enum
{
    SPI1_INSTANCE = 0,
    SPI2_INSTANCE = 1,
} dspi_instance_t;    
#endif  
 #if defined(S32V23x_SERIES)
typedef enum
{
    SPI0_INSTANCE = 0,
    SPI1_INSTANCE = 1,
    SPI2_INSTANCE = 2,
    SPI3_INSTANCE = 3,
} dspi_instance_t;    
#endif

 #if defined(S32S247TV_SERIES)
typedef enum
{
    SPI0_INSTANCE = 0,
    SPI1_INSTANCE = 1,
    SPI2_INSTANCE = 2,
    SPI3_INSTANCE = 3,
    SPI4_INSTANCE = 4,
    SPI5_INSTANCE = 5
} dspi_instance_t;    
#endif

 #if defined(CPU_MPC5777C) || defined(CPU_MPC5775B) || defined(CPU_MPC5775E)
typedef enum
{
    DSPI0_INSTANCE = 0,
    DSPI1_INSTANCE = 1,
    DSPI2_INSTANCE = 2,
    DSPI3_INSTANCE = 3,
    DSPI4_INSTANCE = 4,
} dspi_instance_t;    
#endif

 #if defined(CPU_MPC5746R) || defined(CPU_MPC5745R) || defined(CPU_MPC5743R)
typedef enum
{
    DSPI0_INSTANCE  = 0,
    DSPI1_INSTANCE  = 1,
    DSPI2_INSTANCE  = 2,
    DSPI3_INSTANCE  = 3,
    DSPI4_INSTANCE  = 4,
    DSPIM0_INSTANCE = 5,
    DSPIM1_INSTANCE = 6,
} dspi_instance_t;    
#endif

/*!
 * @brief Defines the mode of the DSPI (master or slave)
 *
 * Select if the device is used as master or slave.
 *
 */
typedef enum
{
    DSPI_MASTER = 0, DSPI_SLAVE  = 1
} dspi_functional_mode_t;


/*!
 * @brief Defines the polarity for data signal of DSPI
 *
 * This enum is used to define data polarity
 *
 * Implements : dspi_polarity_t_Class
 */
typedef enum
{
    DSPI_ACTIVE_HIGH = 0, DSPI_ACTIVE_LOW = 1
} dspi_polarity_t;

/*!
 * @brief Defines shift and capture edges for SPI transfer
 *
 * This enum is used to define the edges used for shifting and captureing
 *
 * Implements : dspi_clock_phase_t_Class
 */
typedef enum
{
    DSPI_CLOCK_PHASE_1ST_EDGE = 0U, /*!< Data captured on SCK 1st edge, changed on 2nd. */
    DSPI_CLOCK_PHASE_2ND_EDGE = 1U  /*!< Data changed on SCK 1st edge, captured on 2nd. */
} dspi_clock_phase_t;

/*!
 * @brief Defines how are handle the RX/TX buffers
 *
 * This typedef defines the available methods to handle RX and TX buffers
 *
 * Implements : dspi_transfer_type_t_Class
 */
typedef enum
{
    DSPI_USING_DMA = 0U, DSPI_USING_INTERRUPTS = 1U
} dspi_transfer_type_t;

/*!
 * @brief Defines the internal status of the last transfer
 *
 * This typedef defines the status of the last transfer and is used for internal checks and by the
 * application for monitoring the transfers.
 *
 * Implements : dspi_transfer_status_t_Class
 */
typedef enum
{
    DSPI_TRANSFER_OK = 0, DSPI_TRANSFER_FAIL = 1, DSPI_IN_PROGRESS = 2
} dspi_transfer_status_t;

/*!
 * @brief Defines the configuration for SPI master mode.
 *
 * This structure is used to configure the DSPI module in SPI master mode. It stores information about
 * transfer type, callback, bus configuration.
 *
 * Implements : dspi_master_config_t_Class
 */
typedef struct
{
    uint32_t bitsPerSec;                 /*!< Baud rate in bits per second*/
    dspi_polarity_t pcsPolarity;         /*!< PCS polarity for all available PCS*/
    uint16_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    dspi_clock_phase_t clkPhase;         /*!< Selects which phase of clock to capture data */
    dspi_polarity_t clkPolarity;         /*!< Selects clock polarity */
    bool isClkContinuous;                /*!< Enable/disable continuous clock */
    bool lsbFirst;                       /*!< Option to transmit LSB first */
    dspi_transfer_type_t transferType;   /*!< Type of DSPI transfer */
    uint8_t rxDMAChannel;                /*!< Channel number for DMA rx channel. If DMA mode isn't used this field will be ignored. */
    uint8_t txDMAChannel;                /*!< Channel number for DMA tx channel. If DMA mode isn't used this field will be ignored. */
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    uint8_t txAdditionalDMAChannel;      /*!< If extended mode is not supported this channel is used to push commands in the coresponding FIFO */
#endif
    spi_callback_t callback;             /*!< User callback function. Note that this function will be
                                          *  called from the interrupt service routine, so its
                                          *  execution time should be as small as possible. It can be
                                          *  NULL if it is not needed */
    void * callbackParam;                 /*!< Parameter for the callback function */
    bool continuousPCS;                  /*!< Enable/disable continuous CS */
    uint8_t whichPCS;                    /*!< Select which CS is used */
} dspi_master_config_t;

/*!
 * @brief Defines the configuration for SPI slave mode.
 *
 * This structure is used to configure the DSPI module in SPI slave mode. It stores information about
 * transfer type, callback, bus configuration.
 *
 * Implements : dspi_slave_config_t_Class
 */
typedef struct
{
    uint16_t bitcount;                   /*!< Number of bits/frame, minimum is 8-bits */
    dspi_clock_phase_t clkPhase;         /*!< Selects which phase of clock to capture data */
    dspi_polarity_t clkPolarity;         /*!< Selects clock polarity */
    dspi_transfer_type_t transferType;   /*!< Type of DSPI transfer */
    uint8_t rxDMAChannel;                /*!< Channel number for DMA rx channel. If DMA mode isn't used this field will be ignored. */
    uint8_t txDMAChannel;                /*!< Channel number for DMA tx channel. If DMA mode isn't used this field will be ignored. */
    spi_callback_t callback;             /*!< User callback function. Note that this function will be
                                          *  called from the interrupt service routine, so its
                                          *  execution time should be as small as possible. It can be
                                          *  NULL if it is not needed */
    void * callbackParam;                 /*!< Parameter for the callback function */
} dspi_slave_config_t;

/*!
 * @brief Defines the state structure for DSPI in SPI mode.
 *
 * This structure is used by DSPI driver in SPI mode to store internal details about current transfer/configuration.
 *
 * Implements : dspi_state_t_Class
 */
typedef struct
{
    uint8_t bytesPerFrame;                   /*!< How many bytes will contain a frame */
    volatile uint16_t framesToSend;          /*!< Frames which will be transmitted */
    volatile uint16_t toSendIndex;           /*!< Index in transmit buffer */
    volatile uint16_t framesToReceive;       /*!< Frames which will be received */
    volatile uint16_t toReceiveIndex;        /*!< Index in receive buffer */
    bool lsbFirst;                           /*!< Option to transmit LSB first */
    const void * volatile txBuffer;          /*!< Channel number for DMA tx channel. If DMA mode isn't used this field will be ignored. */
    void * volatile rxBuffer;                /*!< Channel number for DMA rx channel. If DMA mode isn't used this field will be ignored. */
    volatile dspi_transfer_status_t status;  /*!< The status for the current transfer */
    dspi_transfer_type_t transferType;       /*!< Type of DSPI transfer (DMA or interrupt base) */
    uint8_t rxDMAChannel;                    /*!< Channel number for receive */
    uint8_t txDMAChannel;                    /*!< Channel number for send */
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    uint8_t txAdditionalDMAChannel;       /*!< If extended mode is not supported this channel is used to push commands in the coresponding FIFO */
#endif
    volatile dspi_functional_mode_t mode;    /*!< Slave or master mode for SPI mode*/
    uint32_t configWord;                     /*!< Store the config word which will be pushed in CMD FIFO */
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    uint32_t lastConfigWord;                 /*!< If extended mode is not supported 2 configuration words are neccessary to push end of queue */
#endif
    volatile bool lastFrameSent;             /*!< Flag used for sending the last frame in continuous mode */
    spi_callback_t callback;                 /*!< User callback function. Note that this function will be
                                              *    called from the interrupt service routine, so its
                                              *    execution time should be as small as possible. It can be
                                              *    NULL if it is not needed */
    void * callbackParam;                     /*!< Parameter for the callback function */
    bool continuousPCS;                      /*!< Enable/disable continuous CS */
    uint8_t whichPCS;                        /*!< Select which CS is used */
    semaphore_t dspiSemaphore;               /*!< The semaphore used for blocking transfers */
    bool isBlocking;                         /*!< Save the transfer type */
#if (!defined (FEATURE_DSPI_HAS_EXTENDED_MODE))
    uint8_t stcd[STCD_SIZE(3U)];             /* Buffer for DMA scatter-gather operations */
#endif
} dspi_state_t;

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
 * @param instance The instance number of the DSPI peripheral.
 * @param delayBetweenTransfers Minimum delay between 2 transfers in microseconds
 * @param delaySCKtoPCS Minimum delay between SCK and PCS
 * @param delayPCStoSCK Minimum delay between PCS and SCK
 * @return STATUS_SUCCESS The operation has completed successfully
 *
 */
status_t DSPI_MasterSetDelay(dspi_instance_t instance,
                             uint32_t delayBetweenTransfers,
                             uint32_t delaySCKtoPCS,
                             uint32_t delayPCStoSCK);

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
 * @param config Pointer to the structure which store the basic config transfer
 * @return An error code or STATUS_SUCCESS.
 *
 */
status_t DSPI_GetDefaultMasterCfg(dspi_master_config_t * config);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param state The pointer to the DSPI master driver state structure. The user
 *  passes the memory for this run-time state structure. The DSPI master driver
 *  populates the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @param config The data structure containing information about a device on the SPI bus
 * @return An error code or STATUS_SUCCESS.
 *
 */
status_t DSPI_MasterInit(dspi_instance_t instance,
                         dspi_state_t * state,
                         const dspi_master_config_t * config);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param frames The number of frames to send and receive.
 * @param transferConfig Pointer to transfer configuration (Select PCS, configure parity and configure
 *  PCS continuous mode).
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress
 *
 */
status_t DSPI_MasterTransfer(dspi_instance_t instance,
                             const void * sendBuffer,
                             void * receiveBuffer,
                             uint16_t frames);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param frames The number of frames to send and receive.
 * @param transferConfig Pointer to transfer configuration (Select PCS, configure parity and configure
 *  PCS continuous mode).
 * @param timeout A timeout for the transfer in milliseconds. If the transfer takes longer than
 *  this amount of time, the transfer is aborted and a STATUS_TIMEOUT error
 *  returned.
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress
 *         STATUS_TIMEOUT The transfer timed out and was aborted.
 *
 */
status_t DSPI_MasterTransferBlocking(dspi_instance_t instance,
                                     const void * sendBuffer,
                                     void * receiveBuffer,
                                     uint16_t frames,
                                     uint32_t timeout);

/*!
 * @brief Terminates an asynchronous transfer early.
 *
 * During an a-sync (non-blocking) transfer, the user has the option to terminate the transfer early
 * if the transfer is still in progress.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @return STATUS_SUCCESS The function was successfully aborted
 *
 */
status_t DSPI_AbortTransfer(dspi_instance_t instance);

/*!
 * @brief Change the chip select used by DSPI driver.
 *
 * This function can be used to change the chip select configured when the driver was initialized.
 * It is useful for applications where more slave devices are controlled by only one dspi instance.
 * On MPC47xx platform this function should be used only for master mode. In Slave mode only one chip
 * select is available.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param whichPCS Select the chip select number.
 * @return STATUS_SUCCESS The function was successfully aborted
 *
 */
status_t DSPI_UpdateCS(dspi_instance_t instance,
                       uint8_t whichPCS);

/*!
 * @brief Change the frame size of the spi transfer
 *
 * This function should be used only when the previous transfer is complete.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param frameSize The new frame size.
 * @return STATUS_SUCCESS The function was successfully aborted
 *
 */                    
status_t DSPI_SetFrameSize(dspi_instance_t instance,
                           uint8_t frameSize);      

/*!
 * @brief Enable or disable continuous mode 
 *
 * This function should be used only when the previous transfer is complete.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param enable New state of the continuous mode.
 * @return STATUS_SUCCESS The function was successfully aborted
 *
 */                        
status_t DSPI_SetContinuousMode(dspi_instance_t instance, 
                                bool enable);                          
/*!
 * @brief This function returns a default configuration for DSPI module in SPI slave mode
 *
 * After this call config will contain the following DSPI - SPI master setup:
 *          -Frame size 8 bits
 *          -Clock active high
 *          -Sampling on the first edge
 *          -Transfer is perform interrupt base
 *          -Parity check disabled
 * @param config Pointer to the structure which store the basic config transfer
 * @return An error code or STATUS_SUCCESS.
 *
 */
status_t DSPI_GetDefaultSlaveCfg(dspi_slave_config_t * config);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param state The pointer to the DSPI slave driver state structure. The user
 *  passes the memory for this run-time state structure. The DSPI slave driver
 *  populates the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @param config The data structure containing information about a device on the SPI bus
 * @return An error code or STATUS_SUCCESS.
 *
 */
status_t DSPI_SlaveInit(dspi_instance_t instance,
                        dspi_state_t * state,
                        const dspi_slave_config_t * config);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param frames The number of frames to send and receive.
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress
 *
 */
status_t DSPI_SlaveTransfer(dspi_instance_t instance,
                            const void * sendBuffer,
                            void * receiveBuffer,
                            uint16_t frames);

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
 * @param instance The instance number of the DSPI peripheral.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param frames The number of frames to send and receive.
 * @param timeout A timeout for the transfer in milliseconds. If the transfer takes longer than
 *  this amount of time, the transfer is aborted and a STATUS_TIMEOUT error
 *  returned.
 * @return STATUS_SUCCESS The transfer was successful, or
 *         STATUS_BUSY Cannot perform transfer because a transfer is already in progress
 *         STATUS_TIMEOUT The transfer timed out and was aborted.
 *
 */
status_t DSPI_SlaveTransferBlocking(dspi_instance_t instance,
                                    const void * sendBuffer,
                                    void * receiveBuffer,
                                    uint16_t frames,
                                    uint32_t timeout);

/*!
 * @brief Shuts down a DSPI instance.
 *
 * This function resets the DSPI peripheral, gates its clock, and disables the interrupt to
 * the core.  It first checks to see if a transfer is in progress and if so returns an error
 * status.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @return STATUS_SUCCESS The transfer has completed successfully, or
 *         STATUS_BUSY The transfer is still in progress.
 *         STATUS_ERROR if driver is error and needs to clean error.
 *
 */
status_t DSPI_Deinit(dspi_instance_t instance);

/*!
 * @brief Get the current transfer status.
 *
 * This function return the status of the last transfer.
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param status The status of the last transfer
 *
 * @return STATUS_SUCCESS The transfer has completed successfully
 */
status_t DSPI_GetTransferStatus(dspi_instance_t instance,
                                dspi_transfer_status_t * status);

/*! @}*/
#endif /* DSPI_HEADER_H_ */
