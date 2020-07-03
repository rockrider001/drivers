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

#ifndef ZIPWIRE_DRIVER_H_
#define ZIPWIRE_DRIVER_H_

#include "device_registers.h"
#include "status.h"
#include "callbacks.h"
#include "osif.h"
#include <stdlib.h>

/*!
 * @file zipwire_driver.h
 */

/*!
 * @defgroup zipwire_driver ZIPWIRE Driver
 * @ingroup zipwire
 * @addtogroup zipwire_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief LFAST interface role (master/slave)
 * Implements : lfast_role_t_Class
 */
typedef enum {
    LFAST_SLAVE = 0U,
    LFAST_MASTER = 1U
} lfast_role_t;

/*!
 * @brief LFAST speed mode
 * Implements : lfast_speed_mode_t_Class
 */
typedef enum {
    LFAST_LOW_SPEED = 0U,
    LFAST_HIGH_SPEED = 1U
} lfast_speed_mode_t;

/*!
 * @brief LFAST PLL reference clock divider.
 * Implements : lfast_pll_refclk_div_t_Class
 */
typedef enum
{
    LFAST_PLL_REF_DIV_1 = 0U,  /*!< Reference clock directly passed to the LFAST PLL */
    LFAST_PLL_REF_DIV_2 = 1U,  /*!< Reference clock divided by 2 */
    LFAST_PLL_REF_DIV_3 = 2U,  /*!< Reference clock divided by 3 */
    LFAST_PLL_REF_DIV_4 = 3U   /*!< Reference clock divided by 4 */
} lfast_pll_refclk_div_t;

/*!
 * @brief LFAST clock division factor in Low Speed Select mode.
 * Implements : lfast_low_speed_clk_t_Class
 */
typedef enum {
    LFAST_LOW_SPEED_CLK_DIV_2 = 0U,
    LFAST_LOW_SPEED_CLK_DIV_4 = 1U
} lfast_low_speed_clk_t;

/*!
 * @brief LFAST configuration.
 * Implements : lfast_config_t_Class
 */
typedef struct
{
    lfast_role_t role;              /*!< LFAST role: MASTER/SLAVE */
    lfast_speed_mode_t speedMode;   /*!< LFAST speed mode: high-speed/low-speed */
    uint32_t syncTimeout;           /*!< Timeout used for the LFAST master-slave synchronization;
                                         @Note: A value of zero for this parameter is equivalent to timeout
                                                being disregarded by the driver; the LFAST initialization
                                                will wait forever for commands/responses from the other node. */
    uint32_t syncAttempts;          /*!< Number of attempts for the master to synchronize with the slave;
                                         this field is only used by the master node.
                                         @Note: A value of zero for this parameter is equivalent to an infinite
                                                number of attempts; the LFAST master will try forever to synchronize
                                                with the slave */
    lfast_pll_refclk_div_t preDiv;  /*!< Division factor for LFAST PLL Reference Clock input. */
    uint8_t fbDiv;                  /*!< Feedback Division factor for LFAST PLL VCO output clock. */
    lfast_low_speed_clk_t lsClkDiv; /*!< LFAST clock division factor in Low Speed Select mode. */
} lfast_config_t;

/*!
 * @brief Zipwire address offset.
 * Implements : zipwire_address_offset_t_Class
 */
typedef enum
{
    ZIPWIRE_ADDR_NO_CHANGE = 0U,  /*!< No change. Address stays the same after the transfer is done */
    ZIPWIRE_ADDR_INC_4     = 1U,  /*!< Increment address by 4 */
    ZIPWIRE_ADDR_DEC_4     = 2U   /*!< Decrement address by 4 */
} zipwire_address_offset_t;

/*!
 * @brief Zipwire transfer size.
 * Implements : zipwire_transfer_size_t_Class
 */
typedef enum
{
    ZIPWIRE_8_BITS  = 0U,  /*!< 8-bit transfer */
    ZIPWIRE_16_BITS = 1U,  /*!< 16-bit transfer */
    ZIPWIRE_32_BITS = 2U   /*!< 32-bit transfer */
} zipwire_transfer_size_t;

/*!
 * @brief Zipwire timeout counter prescaler.
 * Implements : zipwire_timeout_prescaler_t_Class
 */
typedef enum
{
    ZIPWIRE_DIV_64   = 0x040U,  /*!< Timeout counter clock = system clock/64 */
    ZIPWIRE_DIV_128  = 0x080U,  /*!< Timeout counter clock = system clock/128 */
    ZIPWIRE_DIV_256  = 0x100U,  /*!< Timeout counter clock = system clock/256 */
    ZIPWIRE_DIV_512  = 0x200U,  /*!< Timeout counter clock = system clock/512 */
    ZIPWIRE_DIV_1024 = 0x400U   /*!< Timeout counter clock = system clock/1024 */
} zipwire_timeout_prescaler_t;

/*!
 * @brief Zipwire transfer descriptor.
 * Implements : zipwire_transfer_descriptor_t_Class
 */
typedef struct
{
    uint32_t address;
    uint32_t data;
    zipwire_transfer_size_t size;
} zipwire_transfer_descriptor_t;

/*!
 * @brief Zipwire user configuration.
 * Implements : zipwire_config_t_Class
 */
typedef struct
{
    lfast_config_t *lfastConfig;               /*!< LFAST configuration */
    zipwire_address_offset_t addrOffset;       /*!< Address increment/decrement for stream transfers */
    zipwire_timeout_prescaler_t timeoutClkDiv; /*!< SIPI timeout clock prescaler */
    bool maxCountReachedInt;                   /*!< Maximum address reached interrupt enable */
    zipwire_callback_t callback;               /*!< Global callback (max count reached/global CRC error) */
    void * callbackParam;                      /*!< Global callback parameter */
} zipwire_config_t;

/*!
 * @brief Zipwire driver state structure.
 * Implements : zipwire_state_t_Class
 */
typedef struct
{
    zipwire_callback_t callback;  /*!< Global callback (max count reached/global CRC error) */
    void * callbackParam;         /*!< Global callback parameter */
} zipwire_state_t;

/*!
 * @brief Zipwire channel state structure.
 * Implements : zipwire_chn_state_t_Class
 */
typedef struct
{
    uint8_t instance;                              /*!< SIPI instance number */
    uint8_t channel;                               /*!< SIPI channel number */
    semaphore_t syncSem;                           /*!< Synchronization semaphore */
    zipwire_chn_callback_t callback;               /*!< Channel callback */
    void * callbackParam;                          /*!< Channel callback parameter */
    volatile status_t chStatus;                    /*!< Channel status */
    volatile bool isBlocking;                      /*!< Flag used for channel blocking operation */
    zipwire_transfer_descriptor_t *transferBuffer; /*!< Internal array holding the buffer for continuous transfers */
    volatile uint32_t remainingTransfers;          /*!< Number of continuous transfers still to be served */
    bool dma;                                      /*!< Enable DMA functionality for this channel */
    bool dmaWriteTransfer;                         /*!< Flag used to differentiate between read and write DMA transfers */
    uint8_t dmaDataChn;                            /*!< DMA channel number used to transfer data */
    uint8_t dmaAddrChn;                            /*!< DMA channel number used to transfer addresses */
    volatile bool idRequest;                       /*!< Flag used to mark ID request transfers */
    bool disableNotification;                      /*!< Flag used when the application must not be notified of an event */
    uint32_t * targetId;                           /*!< Internal variable storing the requested target ID */
} zipwire_chn_state_t;

/*!
 * @brief Zipwire channel configuration.
 * Implements : zipwire_chn_config_t_Class
 */
typedef struct
{
    uint8_t chNo;                         /*!< Channel number (0-3) */
    uint8_t timeout;                      /*!< Timeout value for requests. */
    bool dma;                             /*!< Enable channel DMA functionality */
    uint8_t dmaDataChn;                   /*!< DMA channel number used to transfer data */
    uint8_t dmaAddrChn;                   /*!< DMA channel number used to transfer addresses */
    bool timeoutErrIrq;                   /*!< Enables/disables timeout error handling for the channel */
    bool ackErrIrq;                       /*!< Enables/disables ACK error handling for the channel */
    bool transIdErrIrq;                   /*!< Enables/disables transaction ID error handling for the channel */
    zipwire_chn_callback_t callback;      /*!< Channel error callback */
    void * callbackParam;                 /*!< Channel error callback parameter */
} zipwire_chn_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Gets the default configuration structure.
 *
 * This function gets the default configuration structure, with the following settings:
 *   - no address increment/decrement
 *   - timeout clock prescaler: 64
 *   - maximum address reached interrupt disabled
 *   - no callback
 *   - LFAST parameters:
 *      * MASTER role
 *      * high speed
 *      * timeout for initialization - 1000
 *      * infinite number of synchronization attempts
 *      * PLL reference clock input not divided
 *      * feedback division factor for PLL VCO output: 16
 *      * LFAST clock in low speed mode divided by 2
 *
 * @param[out] zipwireConfig - The zipwire configuration structure
 * @param[out] lfastConfig - LFAST configuration structure, referenced in the zipwire config
 */
void ZIPWIRE_DRV_GetDefaultConfig(zipwire_config_t *zipwireConfig, lfast_config_t *lfastConfig);

/*!
 * @brief Initializes the driver.
 *
 * This function initializes the appropriate SIPI and LFAST interfaces, according to the
 * configuration passed by the user.
 *
 * @param[in] instance - Instance number
 * @param[in] state - Pointer to the state structure which will be used for holding
 *                    the internal state of the driver.
 * @param[in] config - The configuration structure
 * @return    An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_Init(uint8_t instance,
                          zipwire_state_t *state,
                          const zipwire_config_t *config);

/*!
 * @brief De-initializes the ZIPWIRE driver.
 *
 * This function shuts down the communication interfaces and brings the driver state machine
 * back to the uninitialized state.
 *
 * @param[in] instance - Instance number
 * @return    An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_Deinit(uint8_t instance);

/*!
 * @brief Initializes a ZIPWIRE channel.
 *
 * This function initializes a channel with the settings provided by the user.
 *
 * @param[in] instance - Instance number
 * @param[in] config - The channel configuration structure
 * @param[in] state - The channel state structure
 * @return    An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_InitChannel(uint8_t instance, const zipwire_chn_config_t *config, zipwire_chn_state_t *state);

/*!
 * @brief De-initializes a ZIPWIRE channel.
 *
 * This function de-initializes a channel.
 *
 * @param[in] instance - Instance number
 * @param[in] channel - Channel number
 * @return    An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_DeinitChannel(uint8_t instance, uint8_t channel);

/*!
 * @brief Performs multiple read transfers.
 *
 * This function performs multiple reads from the addresses supplied by the user within the array
 * parameter. It returns once the first transfer is launched. If a callback is installed, the user will be
 * notified when the last read transfer is done; otherwise, transfer status can be checked by
 * calling 'ZIPWIRE_DRV_GetChannelStatus'.
 *
 * @param[in]     instance - Instance number
 * @param[in]     channel - The channel number
 * @param[in/out] dataArray - Array of transfer descriptors (address, size, data)
 * @param[in]     dataArrayLength - Length of the data array
 * @return        An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_Read(uint8_t instance,
                          uint8_t channel,
                          zipwire_transfer_descriptor_t *dataArray,
                          uint32_t dataArrayLength);

/*!
 * @brief Performs multiple read transfers synchronously.
 *
 * This function performs multiple reads from the addresses supplied by the user within the array
 * parameter. It does not return until all the read requests are served or an error occurs.
 * Read data is stored in the array elements.
 *
 * @param[in]     instance - Instance number
 * @param[in]     channel - The channel number
 * @param[in/out] dataArray - Array of transfer descriptors (address, size, data)
 * @param[in]     dataArrayLength - Length of the data array
 * @param[in]     timeout - Time allowed for all the read operations.
 * @return        An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_ReadBlocking(uint8_t instance,
                                  uint8_t channel,
                                  zipwire_transfer_descriptor_t *dataArray,
                                  uint32_t dataArrayLength,
                                  uint32_t timeout);

/*!
 * @brief Performs multiple read transfers using DMA.
 *
 * This function performs multiple read transfers from the address supplied by the user, using
 * DMA requests. The DMA engine automatically copies data in the data buffer.
 * The function returns once the first transfer is launched. If a callback is installed, the user will be
 * notified when the last read transfer is done; otherwise, transfer status can be checked by
 * calling 'ZIPWIRE_DRV_GetChannelStatus'.
 * @Note: Only 32-bits transfers are supported in DMA mode.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[out] dataArray - Data buffer (target data copied here)
 * @param[in]  addressArray - Array containing target addresses where data will be read from
 * @param[in]  arrayLength - Length of the data & address buffers
 * @return     An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_ReadDma(uint8_t instance,
                             uint8_t channel,
                             uint32_t *dataArray,
                             const uint32_t *addressArray,
                             uint32_t arrayLength);

/*!
 * @brief Performs multiple read transfers using DMA, synchronously.
 *
 * This function performs multiple read transfers from the address supplied by the user, using
 * DMA requests. The DMA engine automatically copies data in the data buffer.
 * The function does not return until all the read requests are served or an error occurs.
 * @Note: Only 32-bits transfers are supported in DMA mode.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[out] dataArray - Data buffer (target data copied here)
 * @param[in]  addressArray - Array containing target addresses where data will be read from
 * @param[in]  arrayLength - Length of the data & address buffers
 * @param[in]  timeout - Time allowed for all the read operations.
 * @return     An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_ReadDmaBlocking(uint8_t instance,
                                     uint8_t channel,
                                     uint32_t *dataArray,
                                     const uint32_t *addressArray,
                                     uint32_t arrayLength,
                                     uint32_t timeout);

/*!
 * @brief Performs multiple write transfers.
 *
 * This function performs multiple write operations at the addresses supplied by the user within the array
 * parameter. It returns once the first transfer is launched. If a callback is installed, the user will be
 * notified when the last write transfer is done; otherwise, transfer status can be checked with by
 * calling 'ZIPWIRE_DRV_GetChannelStatus'.
 *
 * @param[in] instance - Instance number
 * @param[in] channel - The channel number
 * @param[in] dataArray - Array of transfer descriptors (address, size, data)
 * @param[in] dataArrayLength - Length of the data array
 * @return    An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_Write(uint8_t instance,
                           uint8_t channel,
                           const zipwire_transfer_descriptor_t *dataArray,
                           uint32_t dataArrayLength);

/*!
 * @brief Performs multiple write transfers synchronously.
 *
 * This function performs multiple write operations at the addresses supplied by the user within the array
 * parameter. It  does not return until the last write operation is completed or an error occurred.
 *
 * @param[in] instance - Instance number
 * @param[in] channel - The channel number
 * @param[in] dataArray - Array of transfer descriptors (address, size, data)
 * @param[in] dataArrayLength - Length of the data array
 * @param[in] timeout - Time allowed for all the write operations.
 * @return    An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_WriteBlocking(uint8_t instance,
                                   uint8_t channel,
                                   const zipwire_transfer_descriptor_t *dataArray,
                                   uint32_t dataArrayLength,
                                   uint32_t timeout);

/*!
 * @brief Performs multiple write transfers using DMA.
 *
 * This function performs multiple write transfers to the address supplied by the user, using
 * DMA requests. The DMA engine automatically copies data from the data buffer.
 * The function returns once the first transfer is launched. If a callback is installed, the user will be
 * notified when the last read transfer is done; otherwise, transfer status can be checked by
 * calling 'ZIPWIRE_DRV_GetChannelStatus'.
 * @Note: Only 32-bits transfers are supported in DMA mode.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[in]  dataArray - Data buffer
 * @param[in]  addressArray - Array containing target addresses where data will be written
 * @param[in]  arrayLength - Length of the data & address buffers
 * @return     An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_WriteDma(uint8_t instance,
                              uint8_t channel,
                              const uint32_t *dataArray,
                              const uint32_t *addressArray,
                              uint32_t arrayLength);

/*!
 * @brief Performs multiple write transfers using DMA, synchronously.
 *
 * This function performs multiple write transfers from the address supplied by the user, using
 * DMA requests. The DMA engine automatically copies data from the data buffer.
 * The function does not return until all the write requests are served or an error occurs.
 * @Note: Only 32-bits transfers are supported in DMA mode.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[in]  dataArray - Data buffer
 * @param[in]  addressArray - Array containing target addresses where data will be written
 * @param[in]  arrayLength - Length of the data & address buffers
 * @param[in]  timeout - Time allowed for all the read operations.
 * @return     An error - code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_WriteDmaBlocking(uint8_t instance,
                                      uint8_t channel,
                                      const uint32_t *dataArray,
                                      const uint32_t *addressArray,
                                      uint32_t arrayLength,
                                      uint32_t timeout);
/*!
 * @brief Performs a synchronous stream write.
 *
 * This function performs a streaming write operation. It does not return until
 * all the bytes are transferred.
 *
 * @param[in] instance - Instance number
 * @param[in] channel - The channel number
 * @param[in] dataAddress - Target address where the data will be written
 * @param[in] targetAcrRegAddress - Address of the SIPI_ACR register on the target node
 * @param[in] data - Array of data bytes to be streamed; it should point to an array of
 *                   minimum 8 bytes (SIPI stream transfer size). It is application responsibility
 *                   to correctly allocate memory before passing this reference, driver is
 *                   unaware of memory allocation at application level.
 * @param[in] timeout - Time allowed for the stream transfer to complete.
 * @return An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_StreamWrite(uint8_t instance,
                                 uint8_t channel,
                                 uint32_t dataAddress,
                                 uint32_t targetAcrRegAddress,
                                 const uint32_t *data,
                                 uint32_t timeout);

/*!
 * @brief Performs an ID request transfer.
 *
 * This requests the device ID from the target node. The target ID will be saved in
 * the output parameter provided by application.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[out] id - Reference to user variable where the target ID is stored
 * @param[in]  timeout - Time allowed for the ID request to complete.
 * @return     An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_RequestId(uint8_t instance, uint8_t channel, uint32_t *id, uint32_t timeout);

/*!
 * @brief Sends a trigger command to the target.
 *
 * This function sends a trigger transfer command to the target.
 *
 * @param[in]  instance - Instance number
 * @param[in]  channel - The channel number
 * @param[in]  timeout - Time allowed for the trigger command to be acknowledged.
 * @return     An error code or STATUS_SUCCESS
 */
status_t ZIPWIRE_DRV_Trigger(uint8_t instance, uint8_t channel, uint32_t timeout);

/*!
 * @brief Returns the channel status.
 *
 * This function returns the status of the last transfer executed by the channel.
 *
 * @param[in] instance - Instance number
 * @param[in] channel - The channel number
 * @return The current channel status, or the status of the latest command;
 *         STATUS_BUSY is returned if a non-blocking command is in progress;
 *         STATUS_SUCCESS is returned if the last command completed successfully;
 *         If an error occurred in the last command, an appropriate error code is
 *         returned; please check the zipwire error codes descriptions in "status.h".
 */
status_t ZIPWIRE_DRV_GetChannelStatus(uint8_t instance, uint8_t channel);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif  /* ZIPWIRE_DRIVER_H_ */


/*******************************************************************************
 * EOF
 ******************************************************************************/

