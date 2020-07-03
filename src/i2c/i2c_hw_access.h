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
 * @i2c_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
 * unsigned' to 'essentially Boolean'. This is required by the conversion of a bit into a bool.
 * Impermissible cast; cannot cast from 'essentially Boolean' to 'essentially unsigned'.
 * This is required by the conversion of a bool into a bit.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * This is required by the conversion of a bitfield of a register into a enum.
 *
 */


#ifndef I2C_HW_ACCESS_H_
#define I2C_HW_ACCESS_H_

#endif /* I2C_HAL_H_ */

#include <stdbool.h>
#include "i2c_driver.h"

/** I2C - Register Layout Typedef */
/*! @brief I2C module types
 * Implements : i2c_module_type_Class
 */
typedef enum
{
    I2C_SLAVE =  0,             /*!< The I2C module is in slave mode >*/
    I2C_MASTER = 1,             /*!< The I2C module is in master mode >*/
}i2c_module_type;


/*******************************************************************************
 * Interrupt handler signature for I2C module
 ******************************************************************************/
extern void I2C_DRV_ModuleIRQHandler(uint8_t instance);

/*!
 * @brief Enable or disable the I2C module.
 *
 * This function enables or disables the I2C module.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  specifies whether to enable or disable the I2C Bus module
 */
static inline void I2C_Set_ModuleEnable(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_MDIS_MASK));
    regValue |= I2C_IBCR_MDIS(!enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Enable or disable the I2C Bus module interrupts.
 *
 * This function enables or disables the I2C Bus Interrupts.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  specifies whether to enable or disable the I2C Bus interrupts
 */
static inline void I2C_Set_BusIRQEnable(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_IBIE_MASK));
    regValue |= I2C_IBCR_IBIE(enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Master/Slave module select.
 *
 * When the MSSL bit is changed from from 0 to 1 master mode
 * is selected.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable Master/Slave select
 */
static inline void I2C_Set_ModuleModSelect(I2C_Type *baseAddr, i2c_module_type moduleType) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_MSSL_MASK));
    regValue |= I2C_IBCR_MSSL(moduleType);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Master/Slave module select.
 *
 * When the MSSL bit is changed from from 0 to 1 master mode
 * is selected.
 *
 * @param baseAddr  base address of the I2C module
 * @return module mode of operation (master or slave)
 */
static inline bool I2C_Get_ModuleModSelect(const I2C_Type *baseAddr)
{
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue = (regValue & I2C_IBCR_MSSL_MASK) >> I2C_IBCR_MSSL_SHIFT;
    return (bool) regValue;
}

/*!
 * @brief Transmit/Receive mode select.
 *
 * This function enable the transmitting or receiving mode.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  0 for receive and 1 for transmit
 */
static inline void I2C_Set_ModuleTransmitMode(I2C_Type *baseAddr,
        bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_TXRX_MASK));
    regValue |= I2C_IBCR_TXRX(enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Get transmit/receive mode select.
 *
 * This function enable the transmitting or receiving mode.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  0 for receive and 1 for transmit
 */

static inline bool I2C_Get_ModuleTransmitMode(const I2C_Type *baseAddr)
{
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue = (regValue & I2C_IBCR_TXRX_MASK) >> I2C_IBCR_TXRX_SHIFT;
    return (bool) regValue;
}

/*!
 * @brief Data acknowledge mode select.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  specifies whether to enable or disable the I2C acknowledge signal
 * for both master and slave
 */
static inline void I2C_Set_DataAcknowledge(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_NOACK_MASK));
    regValue |= I2C_IBCR_NOACK(!enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief DMA TX/RX request signals enable.
 *
 *This function will enable or disable the DMA TX/RX request signals.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable  specifies whether to enable or disable the DMA Tx/RX request signals
 * for both data and slave
 */
static inline void I2C_Set_DMAEnable(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_DMAEN_MASK));
    regValue |= I2C_IBCR_DMAEN(enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Generate repeat start cycle.
 *
 * This function will generate a repeat start condition on the bus.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable generate a repeated START condition on the bus if the
 * module is in master mode
 */
static inline void I2C_Set_RepeatStart(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBCR;
    regValue &= (uint8_t) (~(I2C_IBCR_RSTA_MASK));
    regValue |= I2C_IBCR_RSTA(enable);
    baseAddr->IBCR = (uint8_t) regValue;
}

/*!
 * @brief Check if the module is addressed as slave.
 *
 * @param baseAddr  base address of the I2C module
 */
static inline bool I2C_Get_AddressAsSlave(const I2C_Type *baseAddr) {
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_IAAS_MASK) >> I2C_IBSR_IAAS_SHIFT;
    return (bool) regValue;
}

/*!
 * @brief Return the idle/busy state of the I2C bus
 *
 * This function returns true if the I2C bus is busy and false if the bus is idle.
 *
 * @param baseAddr  base address of the I2C module
 * @return  the idle/busy state of the I2C bus
 */
static inline bool I2C_Get_BusBusyEvent(const I2C_Type *baseAddr) {
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_IBB_MASK) >> I2C_IBSR_IBB_SHIFT;
    return (bool) regValue;
}

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
/*!
 * @brief Check the occurrence of an arbitration lost event
 *
 * This function returns true if the I2C master detects an arbitration lost
 * condition, as defined by the I2C standard.
 *
 * @param baseAddr  base address of the I2C module
 * @return  indication of an arbitration lost event
 */
static inline bool I2C_Get_MasterArbitrationLostEvent(
        const I2C_Type *baseAddr) {
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_IBAL_MASK) >> I2C_IBSR_IBAL_SHIFT;
    return (bool) regValue;
}
#endif

#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
/*!
 * @brief Clear the occurrence of an arbitration lost event
 *
 * This function clears the arbitration lost flag.
 *
 * @param baseAddr  base address of the I2C module
 */
static inline void I2C_Clear_MasterArbitrationLostFlag(I2C_Type *baseAddr)
{
    bool enable = true;
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue &= (uint8_t)((~(I2C_IBSR_IBAL_MASK)) & (~(I2C_IBSR_IBIF_MASK)));
    regValue |= I2C_IBSR_IBAL(enable);
    baseAddr->IBSR = (uint8_t) regValue;
}
#endif

/*!
 * @brief Check if the slave is transmit or receive mode.
 *
 * This function returns true if the I2C is in transmitting mode
 * and false if the slave is in receive mode.
 *
 * @param baseAddr  base address of the I2C module
 * @return  indication that slave is in transmitting mode
 */
static inline bool I2C_Get_SlaveTransmitMode(const I2C_Type *baseAddr) {
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_SRW_MASK) >> I2C_IBSR_SRW_SHIFT;
    return (bool) regValue;
}

/*!
 * @brief Check if acknowledge received.
 *
 * This function verify if a acknowledge bit is received.
 *
 * @param baseAddr  base address of the I2C module
 * @return  true if acknowledge signal is received.
 */
static inline bool I2C_Get_ReceivedACK(const I2C_Type *baseAddr) {
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_RXAK_MASK) >> I2C_IBSR_RXAK_SHIFT;
    return !((bool) regValue);
}

/*!
 * @brief Clear the I-Bus interrupt flag.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable a true value will clear the I-Bus Interruption flag
 */
static inline void I2C_Clear_BusInterruptFlag(I2C_Type *baseAddr, bool enable)
{
    uint8_t regValue = (uint8_t) baseAddr->IBSR;
#if (!defined(NO_ARBITRATION_LOST_DETECTION_FEATURE))
    regValue &= (uint8_t)((~(I2C_IBSR_IBIF_MASK)) & (~(I2C_IBSR_IBAL_MASK)));
#else
    regValue &= (uint8_t)(~(I2C_IBSR_IBIF_MASK));
#endif
    regValue |= I2C_IBSR_IBIF(enable);
    baseAddr->IBSR = (uint8_t) regValue;
}

/*!
 * @brief Return the received data
 *
 * This function returns data received by the I2C module after
 * an address match occurred.
 *
 * @param baseAddr  base address of the I2C module
 * @return  data received by the I2C module
 */
static inline uint8_t I2C_Get_DataIBDR(const I2C_Type *baseAddr) {
    uint8_t regValue = baseAddr->IBDR;
    regValue = (regValue & I2C_IBDR_DATA_MASK) >> I2C_IBDR_DATA_SHIFT;
    return (uint8_t) regValue;
}


/*!
 * @brief Return the value of the transfer complete flag
 *
 * This function returns the value of the transfer complete flag
 *
 * @param baseAddr  base address of the I2C module
 * @return true if transfer is completed, false if transfer is in progress
 */
static inline bool I2C_Get_TCF(const I2C_Type *baseAddr) {
    uint8_t regValue = baseAddr->IBSR;
    regValue = (regValue & I2C_IBSR_TCF_MASK) >> I2C_IBSR_TCF_SHIFT;
    return (bool) regValue;
}

/*!
 * @brief Write data which will be transmitted in master or slave mode.
 *
 * This function transmits data in both master and slave mode.
 *
 * @param baseAddr  base address of the I2C module
 * @param  data to be send
 */
static inline void I2C_Set_DataIBDR(I2C_Type *baseAddr, uint8_t data) {
    baseAddr->IBDR = (uint8_t) data;
}

/*IBIC Interrupt config register*/

/*!
 * @brief Enable/Disable Bus Idle Interrupt.
 *
 * This function can be used to enable generation of an interrupt once the
 * I2C Bus becomes idle.
 *
 * @param baseAddr  base address of the I2C module
 * @param enable a true value will enable the Bus Idle Interrupt
 */
static inline void I2C_Set_BusIdleIRQ(I2C_Type *baseAddr, bool enable) {
    uint8_t regValue = (uint8_t) baseAddr->IBIC;
    regValue &= (uint8_t) (~(I2C_IBIC_BIIE_MASK));
    regValue |= I2C_IBIC_BIIE(enable);
    baseAddr->IBIC = (uint8_t) regValue;
}

/*!
 * @brief Set slave address for I2C module.
 *
 * @param baseAddr  base address of the I2C module
 * @param data  address for slave
 */
static inline void I2C_Set_SlaveAddress(I2C_Type *baseAddr, uint8_t data) {
    uint8_t regValue = (uint8_t) baseAddr->IBAD;
    regValue &= (uint8_t) (~(I2C_IBAD_ADR_MASK));
    regValue |= I2C_IBAD_ADR(data);
    baseAddr->IBAD = (uint8_t) regValue;
}

/*!
 * @brief Reset Module State
 *
 * This function reset the value of I2C Bus Control register and
 * the value of I2C Bus Interrupt Configuration Register.
 *
 * @param baseAddr  base address of the I2C module
 */
static inline void I2C_ResetModuleState(I2C_Type *baseAddr)
{
    /* Reset the I2C Bus Control Register values */
    baseAddr->IBCR = 0x80;

    /* Reset the I2C Bus Interrupt Configuration Register */
    baseAddr->IBIC = 0x00;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

