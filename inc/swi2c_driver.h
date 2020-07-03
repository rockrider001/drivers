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

#if !defined(SWI2C_DRIVER_H)
#define SWI2C_DRIVER_H

#include <stddef.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @file swi2c_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * The macro is used in swi2c driver in a function, but in i2c_pal the function is not called.
 */

/*!
 * @addtogroup swi2c_drv I2C Driver
 * @ingroup swi2c
 * @brief Software-Inter-Integrated Circuit Driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Defines the maximum number of cycles to wait for a signal transition */
#define WAITING_CYCLES           0xFFFFu

/*!
 * @brief SWI2C pin structure
 *
 * This structure is used to provide configuration parameters for pins used for swi2c driver.
 * Implements : swi2c_pin_t_Class
 */
typedef struct
{
    uint16_t pinNumber;
    GPIO_Type *port;
}swi2c_pin_t;

/*!
 * @brief Master configuration structure
 *
 * This structure is used to provide configuration parameters for the I2C master at initialization time.
 * Implements : swi2c_master_user_config_t_Class
 */
typedef struct
{
    uint8_t slaveAddress;                       /*!< Slave address, 7-bit */
    uint32_t baudRate;                          /*!< The number of waiting cycles for a transition of the SCL signal */
    swi2c_pin_t *sdaPin;                        /*!< SDA pin */
    swi2c_pin_t *sclPin;                        /*!< SCL pin */
    swi2c_pin_t *sdaReadPin;                    /*!< Pin used to read the SDA line */
    swi2c_pin_t *sclReadPin;                    /*!< Pin used to read the SCL line */
} swi2c_master_user_config_t;


/*!
 * @brief Master internal context structure
 *
 * This structure is used by the master-mode driver for its internal logic. It must
 * be provided by the application through the I2C_DRV_MasterInit() function.
 * The application should make no assumptions about the content of this structure.
 * Implements : swi2c_master_state_t_Class
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    swi2c_pin_t *sdaPin;                        /* SDA pin configured as output - must be open-drain*/
    swi2c_pin_t *sclPin;                        /* SCL pin configured as output - must be open-drain */
    swi2c_pin_t *sdaReadPin;                    /* Pin to read SDA line, configured as input */
    swi2c_pin_t *sclReadPin;                    /* Pin to read SCL line, configured as input */
    volatile status_t status;                   /* Status of last driver operation */
    uint8_t slaveAddress;                       /* Slave address */
    uint32_t baudRate;                          /* Cycles  */
    bool sendData;                              /* Master is in transmitting or receive mode */
/*! @endcond */
}swi2c_master_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the I2C master mode driver
 *
 * This function initializes the SWI2C driver in master mode.
 *
 * @param master    SWI2C master state structure
 * @param userConfigPtr    Pointer to the I2C master user configuration structure. The function
 *                         reads configuration data from this structure and initializes the
 *                         driver accordingly. The application may free this structure after
 *                         the function returns.
 * @param master    Pointer to the I2C master driver context structure. The driver uses
 *                  this memory area for its internal logic. The application must make no
 *                  assumptions about the content of this structure, and must not free this
 *                  memory until the driver is de-initialized using SWI2C_DRV_MasterDeinit().
 * @return    Error or success status returned by API
 */
status_t SWI2C_DRV_MasterInit(swi2c_master_state_t * master, const swi2c_master_user_config_t * userConfigPtr);

/*!
 * @brief Set the slave address for the I2C communication
 *
 * This function sets the slave address which will be used for any future
 * transfer initiated by the SWI2C master.
 *
 * @param master    SWI2C master state structure
 * @param address   slave 7-bit address
 */
void SWI2C_DRV_MasterSetSlaveAddress(swi2c_master_state_t *master, uint8_t address);

/*!
 * @brief Performs a blocking send transaction on the I2C bus
 *
 * This function starts the transmission of a block of data to the currently
 * configured slave address.
 *
 * @param master    SWI2C master state structure
 * @param txBuff    pointer to the data to be transferred
 * @param txSize    length in bytes of the data to be transferred
 * @param sendStop  specifies whether or not to generate stop condition after the transmission
 * @return    Error or success status returned by API
 */
status_t SWI2C_DRV_MasterSendDataBlocking(swi2c_master_state_t *master, const uint8_t *txBuff, uint32_t txSize, bool sendStop);

/*!
 * @brief Performs a blocking receive transaction on the I2C bus
 *
 * This function starts the reception of a block of data from the currently
 * configured slave address.
 *
 * @param master    SWI2C master state structure
 * @param rxBuff    pointer to the buffer where to store received data
 * @param rxSize    length in bytes of the data to be transferred
 * @param sendStop    specifies whether or not to generate stop condition after the reception
 * @return    Error or success status returned by API
 */
status_t SWI2C_DRV_MasterReceiveDataBlocking(swi2c_master_state_t *master, uint8_t *rxBuff, uint32_t rxSize, bool sendStop);

/*!
 * @brief Sets the number of cycles to wait for a SCL transition.
 *
 * @param master    SWI2C master state structure
 * @param cycles    cycles to wait before a SCL transition.
 */
void SWI2C_DRV_SetWaitTimeForSCLTransition(swi2c_master_state_t *master, uint32_t cycles);

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* SWI2C_DRIVER_H */
/*******************************************************************************
 * EOF
 *******************************************************************************/
