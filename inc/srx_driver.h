/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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

#ifndef SRX_DRIVER_H
#define SRX_DRIVER_H

/* Required headers */
#include <stddef.h>
#include "device_registers.h"
#include "status.h"

/*!
 * @addtogroup srx
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * Masks for various events
 * These can be combined such as: events |= (EV_1 | EV_2);
 * Also used inside the configuration structure for enabling/disabling certain events
 */
#define SRX_EV_BUS_IDLE          ((uint32_t)1u << 0) /* Channel has been idle for more than the allowed value */
#define SRX_EV_CAL_RESYNC        ((uint32_t)1u << 1) /* Successive Calibration Check has failed three times */
#define SRX_EV_CAL_20_25         ((uint32_t)1u << 2) /* Calibration pulse received on this channel has variation in between ±20% to ±25% from 56 ticks */
#define SRX_EV_SMSG_OFLW         ((uint32_t)1u << 3) /* Slow message overflow */
#define SRX_EV_FMSG_OFLW         ((uint32_t)1u << 4) /* Fast message overflow */
#define SRX_EV_PP_DIAG_ERR       ((uint32_t)1u << 5) /* The ratio of calibration pulse length to overall message length (with pause pulse) is more than ±1.5625% between two messages */
#define SRX_EV_CAL_LEN_ERR       ((uint32_t)1u << 6) /* Calibration pulse is more than 56 ticks ±25% */
#define SRX_EV_CAL_DIAG_ERR      ((uint32_t)1u << 7) /* Successive Calibration pulses differ by ±1.56% */
#define SRX_EV_NIB_VAL_ERR       ((uint32_t)1u << 8) /* Any nibble data value <0 or >15 */
#define SRX_EV_SMSG_CRC_ERR      ((uint32_t)1u << 9) /* Checksum error in Slow Serial Message */
#define SRX_EV_FMSG_CRC_ERR      ((uint32_t)1u << 10) /* Checksum error in Fast Serial Message */
#define SRX_EV_NUM_EDGES_ERR     ((uint32_t)1u << 11) /*  Not the expected number of negative edges between calibration pulse */
#define SRX_EV_FIFO_OVERFLOW     ((uint32_t)1u << 12) /* Overflow in FIFO queue */
#define SRX_EV_FDMA_UNDERFLOW    ((uint32_t)1u << 13) /* Underflow in Fast message DMA registers */
#define SRX_EV_SDMA_UNDERFLOW    ((uint32_t)1u << 14) /* Underflow in Slow message DMA registers */
#define SRX_EV_ALL ((uint32_t)(SRX_EV_BUS_IDLE | SRX_EV_CAL_RESYNC | SRX_EV_CAL_20_25 | SRX_EV_SMSG_OFLW \
                    | SRX_EV_FMSG_OFLW | SRX_EV_PP_DIAG_ERR | SRX_EV_CAL_LEN_ERR | SRX_EV_CAL_DIAG_ERR \
                    | SRX_EV_NIB_VAL_ERR  | SRX_EV_SMSG_CRC_ERR | SRX_EV_FMSG_CRC_ERR | SRX_EV_NUM_EDGES_ERR \
                    | SRX_EV_FIFO_OVERFLOW | SRX_EV_FDMA_UNDERFLOW | SRX_EV_SDMA_UNDERFLOW)) /* All flags */
#define SRX_EV_NONE ((uint32_t)(~(SRX_EV_ALL))) /* No events */
/**
 * @brief Fast/Slow CRC type
 *
 * Contains the possible values for the configuration
 * of the CRC type.
 *
 * Implements : srx_msg_crc_t_Class
 */
typedef enum
{
    SRX_CRC_RECOMMENDED, /* Additional 0 data nibble XORed with the rest of the nibbles */
    SRX_CRC_LEGACY /* No additional 0 data nibble is XORed */
} srx_msg_crc_t;

/**
 * @brief Idle check configuration
 *
 * Contains the possible values use in setting
 * up the IDLE pulse diagnostics.
 *
 * Implements : srx_diag_idle_cnt_cfg_t_Class
 */
typedef enum
{
    SRX_BUS_IDLE_DISABLED,
    SRX_BUS_IDLE_245_CLK_TICKS,
    SRX_BUS_IDLE_508_CLK_TICKS,
    SRX_BUS_IDLE_1016_CLK_TICKS,
    SRX_BUS_IDLE_2032_CLK_TICKS
} srx_diag_idle_cnt_cfg_t;

/**
 * @brief Calibration configuration
 *
 * Contains the allowed values for configuring
 * the variance in the calibration pulse.
 *
 * Implements : srx_diag_calib_pulse_var_cfg_t_Class
 */
typedef enum
{
    SRX_CALIB_VAR_20_PERCENT,
    SRX_CALIB_VAR_25_PERCENT
} srx_diag_calib_pulse_var_cfg_t;

/**
 * @brief Diagnostics pulse configuration
 *
 * Contains the possible value for configuring
 * the diagnostics pulse checks.
 *
 * Implements : srx_diag_pulse_cfg_t_Class
 */
typedef enum
{
    SRX_PULSE_CHECK_BOTH,
    SRX_PULSE_CHECK_PAUSE
} srx_diag_pulse_cfg_t;

/**
 * @brief Pause pulse configuration
 *
 * Contains the possible value of the PAUSE pulse
 * configuration.
 *
 * Implements : srx_diag_pause_pulse_cfg_t_Class
 */
typedef enum
{
    SRX_PAUSE_PULSE_DISABLED,
    SRX_PAUSE_PULSE_ENABLED
} srx_diag_pause_pulse_cfg_t;

/**
 * @brief Successive calibration check method
 *
 * This contains the values use in the configuration
 * for the successive calibration pulse check field.
 *
 * Implements : srx_diag_succ_cal_check_cfg_t_Class
 */
typedef enum
{
    SRX_SUCC_CAL_CHK_LOW_LATENCY,
    SRX_SUCC_CAL_CHK_PREFFERED
} srx_diag_succ_cal_check_cfg_t;

/**
 * @brief Input filter settings
 *
 * Contains all the possible values for configuring
 * the channel hardware input filter (glitch filter).
 * It is expressed in a number of protocol clocks
 * during which the signal should maintain it's value
 * to be considered stable.
 *
 * Implements : srx_channel_input_filter_t_Class
 */
typedef enum
{
    SRX_INPUT_FILTER_NONE, /*!< No filtering */
    SRX_INPUT_FILTER_1,
    SRX_INPUT_FILTER_2,
    SRX_INPUT_FILTER_4,
    SRX_INPUT_FILTER_8,
    SRX_INPUT_FILTER_16,
    SRX_INPUT_FILTER_32,
    SRX_INPUT_FILTER_64,
    SRX_INPUT_FILTER_128
} srx_channel_input_filter_t;

/**
 * @brief Slow message type
 *
 * This is determined by the peripheral, at reception.
 *
 * Implements : srx_slow_msg_type_t_Class
 */
typedef enum
{
    SRX_SLOW_TYPE_SHORT, /*!< Standard 4 bit ID, 8 bit Data */
    SRX_SLOW_TYPE_ENHANCED_4_BIT, /*!< Enhanced 4 bit ID, 16 bit Data */
    SRX_SLOW_TYPE_ENHANCED_8_BIT /*!< Enhanced 8 bit ID, 12 bit Data */
} srx_slow_msg_type_t;

/**
 * @brief Type of a SRX event
 *
 * Implements : srx_event_t_Class
 */
typedef uint32_t srx_event_t;

/**
 * @brief Type of issued callback
 *
 * Implements : srx_callback_type_t_Class
 */
typedef enum
{
    SRX_CALLBACK_SLOW_DMA_RX_COMPLETE,
    SRX_CALLBACK_FAST_DMA_RX_COMPLETE,
    SRX_CALLBACK_SLOW_RX_COMPLETE,
    SRX_CALLBACK_FAST_RX_COMPLETE,
    SRX_CALLBACK_RX_ERROR
} srx_callback_type_t;

/**
 * @brief Type of callback function
 *
 * Implements : srx_callback_func_t_Class
 */
typedef void(* srx_callback_func_t)(uint32_t instance, uint32_t channel, srx_callback_type_t type, void * param);

/**
 * @brief Type of callback structure
 *
 * Implements : srx_callback_t_Class
 */
typedef struct
{
    srx_callback_func_t function; /*!< Callback function */
    void * param; /*!< User parameter */
} srx_callback_t;

/**
 * @brief Fast messages configuration
 *
 * Contains all the necessary fields used in the
 * configuration of the fast message channel.
 *
 * Implements : srx_fast_msg_config_t_Class
 */
typedef struct
{
    /* Message length */
    uint8_t numberOfNibbles; /*!< Number of nibbles for the message. Valid values 1 >= x <= 6 */

    /* DMA enabler for the specific channel */
    bool dmaEnable; /*!< Enable DMA transfers */

    /* CRC related */
    bool crcIncludeStatus; /*!< Include the STATUS nibble in the CRC calculation */
    bool disableCrcCheck; /*!< Disable CRC checks for the channel */
    srx_msg_crc_t crcType; /*!< CRC type */
} srx_fast_msg_config_t;

/**
 * @brief Slow messages configuration
 *
 * Contains all the necessary fields used in the
 * configuration of the slow message channel.
 *
 * Implements : srx_slow_msg_config_t_Class
 */
typedef struct
{
    /* DMA enabler for the specific channel */
    bool dmaEnable; /*!< Enable DMA transfers */

    /* CRC related */
    srx_msg_crc_t crcType; /*!< CRC type */
} srx_slow_msg_config_t;

/**
 * @brief Diagnostics configuration
 *
 * Contains all the fields used in the configuration of the
 * diagnostics side of the driver. For a better understanding
 * of these settings, the SAE J2716 specification and
 * the user manual for the current part should be read.
 *
 * Implements : srx_diag_config_t_Class
 */
typedef struct
{
    srx_event_t diagEvents; /*!< Flags for active diagnostics events */
    srx_diag_idle_cnt_cfg_t idleCount; /*!< Maximum allowed IDLE time */
    srx_diag_calib_pulse_var_cfg_t calibVar; /*!< Valid calibration pulse range */
    srx_diag_pulse_cfg_t diagPulse; /*!< Selection for which diagnostics to run for pulses*/
    srx_diag_pause_pulse_cfg_t pausePulse; /*!< Selection of PAUSE pulse*/
    srx_diag_succ_cal_check_cfg_t succesiveCal; /*!< Successive calibration check method */
} srx_diag_config_t;

/**
 * @brief Channel configuration structure
 *
 * Contains all the required fields for the configuration
 * of a single channel inside the peripheral.
 *
 * Implements : srx_channel_config_t_Class
 */
typedef struct
{
    uint8_t channelId; /*!< Designated channel */
    uint8_t tickDuration; /*!< Tick duration in microseconds */
    srx_channel_input_filter_t inputFilter; /*!< Channel input filter configuration */
    srx_diag_config_t diagConfig; /*!< Diagnostics configuration structure*/
    srx_fast_msg_config_t fastMsgConfig; /*!< Fast messages configuration structure */
    srx_slow_msg_config_t slowMsgConfig; /*!< Slow messages configuration structure */
} srx_channel_config_t;

/**
 * @brief Raw message data type
 *
 * Raw message type. It is the actual format transferred by the
 * DMA peripheral and contained inside the Rx channel registers.
 *
 * Implements : srx_raw_msg_t_Class
 */
typedef struct
{
    uint32_t dataField0; /*!< Raw data field [0] */
    uint32_t dataField1; /*!< Raw data field [1] */
    uint32_t dataField2; /*!< Raw data field [2] */
} srx_raw_msg_t;

/**
 * @brief Actual format for a Fast message
 *
 * Contains the actual fields of a complete FAST message.
 *
 * Implements : srx_fast_msg_t_Class
 */
typedef struct
{
    uint32_t data; /*!< Data field formed from all possible nibbles */
    uint32_t timeStamp; /*!< Message timestamp */
    uint8_t channelNumber; /*!< Channel number, 4 bits */
    uint8_t statusField; /*!< Status field for the message, 2 bits */
    uint8_t crc; /*!< CRC for the fast message, 4 bits */
} srx_fast_msg_t;

/**
 * @brief Actual format for a Slow message
 *
 * Contains the actual fields of a complete SLOW message
 * Lengths for the actual fields depend on the actual message type
 * Please see the SAE J2716 specification.
 *
 * Implements : srx_slow_msg_t_Class
 */
typedef struct
{
    uint32_t timeStamp; /*!< Message timestamp */
    uint16_t data; /*!< Data field */
    srx_slow_msg_type_t type; /*!< Message type */
    uint8_t id; /*!< Message ID */
    uint8_t channelNumber; /*!< Channel number, 4 bits */
    uint8_t crc; /*!< CRC for the slow message, max 6 bits */
} srx_slow_msg_t;

/**
 * @brief Configuration structure
 *
 * Contains all the fields necessary for a complete
 * configuration of the peripheral
 *
 * Implements : srx_driver_user_config_t_Class
 */
typedef struct
{
    /* Dma related transfers are common to all channels */
    srx_raw_msg_t * fastMsgDmaPtr; /*!< Initial DMA target buffer */
    srx_raw_msg_t * slowMsgDmaPtr; /*!< Initial DMA target buffer */
    srx_callback_t callbackFunc; /*!< Callback function */
    uint8_t slowDmaChannel; /*!< Assigned DMA channel for slow messages */
    uint8_t fastDmaChannel; /*!< Assigned DMA channel for fast messages */
    bool fastDmaFIFOEnable; /*!< Enable FIFO for Fast DMA transfers */
    uint8_t fastDmaFIFOSize; /*!< Minimum FIFO size to trigger a Fast DMA transfer */

    /* Unique instances for each channel */
    const srx_channel_config_t * channelConfig; /*!< Pointer to a channel configuration list */
    uint8_t numOfConfigs; /*!< Number of configurations in the channel configuration list */
} srx_driver_user_config_t;

/**
 * @brief State structure
 *
 * Contains internal driver state data.
 *
 * Implements : srx_state_t_Class
 */
typedef struct
{
/*! @cond DRIVER_INTERNAL_USE_ONLY */
    /* Configuration data */
    srx_raw_msg_t * fastMsgDmaPtr; /*!< DMA transfer target pointer for fast messages */
    srx_raw_msg_t * slowMsgDmaPtr; /*!< DMA transfer target pointer for slow messages */
    uint8_t fastDmaChannel; /*!< DMA channel on which fast message transfers are performed */
    uint8_t slowDmaChannel; /*!< DMA channel on which slow message transfers are performed */
    srx_callback_t callbackFunc; /*!< Callback function */
    srx_event_t channelEvents[SRX_CHANNEL_COUNT]; /*!< Event configuration for each channel */
    bool activeChannels[SRX_CHANNEL_COUNT]; /*!< Activation state for the specific channel */
    bool fastDmaEnabled[SRX_CHANNEL_COUNT]; /*!< Holds the state for the Fast DMA channel */
    bool slowDmaEnabled[SRX_CHANNEL_COUNT]; /*!< Holds the state for the Slow DMA channel */
    uint8_t instanceId; /*!< Instance Id number */
/*! @endcond */
} srx_state_t;

/**
 * @brief Main initializer for the driver
 *
 * Initializes the driver for a given peripheral
 * according to the given configuration structure.
 *
 * @param[in] instance Instance of the SRX peripheral
 * @param[in] configPtr Pointer to the configuration structure
 * @param[out] state Pointer to the state structure to populate
 * @return The status of the operation
 */
status_t SRX_DRV_Init(const uint32_t instance, const srx_driver_user_config_t * configPtr, srx_state_t * state);

/**
 * @brief Event list getter
 *
 * Returns a list containing masks for the current active events.
 * Also clears the active events in the process.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] channel Channel for which the mask shall be read
 * @param[out] events Pointer to a srx_event_t type in which to put the mask
 * @return The status of the operation
 */
status_t SRX_DRV_GetEvents(const uint32_t instance, const uint32_t channel, srx_event_t * events);

/**
 * @brief Gets RX status for the Fast channel
 *
 * Returns the buffer status for any incoming FAST message.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] channel Channel for which the request is made
 * @return TRUE if there is a message in the RX buffer
 */
bool SRX_DRV_GetFastRxStatus(const uint32_t instance, const uint32_t channel);

/**
 * @brief Returns last received Fast message
 *
 * Returns last received fast message and clears the
 * Rx complete flag.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] channel Target channel
 * @param[out] message Pointer to the target structure
 * @return The status of the operation
 */
status_t SRX_DRV_GetFastMsg(const uint32_t instance, const uint32_t channel, srx_fast_msg_t * message);

/**
 * @brief Sets the target buffer for reception
 *
 * Sets (modifies) the buffer in which the DMA driven
 * reception for Fast messages is made. Length of the
 * buffer must be (fastDmaFIFOSize)
 * bytes in case fastDmaFIFOEnable is TRUE.
 *
 * @param[in] instance Peripheral instance number
 * @param[out] buffer Pointer to the target raw buffer
 * @return The status of the operation
 */
status_t SRX_DRV_SetFastMsgDmaBuffer(const uint32_t instance, srx_raw_msg_t * buffer);

/**
 * @brief Raw to normal fast message transformation
 *
 * Transforms a RAW fast message into a normal fast message.
 *
 * @param[in] msg Pointer to a structure which will contain the fast message
 * @param[in] rawMsg pointer to a raw message
 */
void SRX_DRV_GetFastMsgFromRaw(srx_fast_msg_t * msg, const srx_raw_msg_t * rawMsg);

/**
 * @brief Gets RX status for the Slow channel
 *
 * Returns the buffer status for any incoming SLOW message.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] channel Channel for which the request is made
 * @return TRUE if there is a message in the RX buffer
 */
bool SRX_DRV_GetSlowRxStatus(const uint32_t instance, const uint32_t channel);

/**
 * @brief Returns last received slow message
 *
 * Returns last received slow message and clears the
 * Rx complete flag.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] channel Target channel
 * @param[out] message Pointer to the target structure
 * @return The status of the operation
 */
status_t SRX_DRV_GetSlowMsg(const uint32_t instance, const uint32_t channel, srx_slow_msg_t * message);

/**
 * @brief Sets the target buffer for reception
 *
 * Sets (modifies) the buffer in which the DMA driven
 * reception for slow messages is made.
 *
 * @param[in] instance Peripheral instance number
 * @param[out] buffer Pointer to the target raw buffer
 * @return The status of the operation
 */
status_t SRX_DRV_SetSlowMsgDmaBuffer(const uint32_t instance, srx_raw_msg_t * buffer);

/**
 * @brief Raw to normal slow message transformation
 *
 * Transforms a RAW slow message into a normal slow message.
 *
 * @param[in] msg Pointer to a structure which will contain the slow message
 * @param[in] rawMsg pointer to a raw message
 */
void SRX_DRV_GetSlowMsgFromRaw(srx_slow_msg_t * msg, const srx_raw_msg_t * rawMsg);

/**
 * @brief Set peripheral Rx notification
 *
 * Sets a notification function.
 * If the given function is NULL, it disables interrupt mode.
 *
 * @param[in] instance Peripheral instance number
 * @param[in] function Callback function
 * @param[in] param Callback parameter
 * @return The status of the operation
 */
status_t SRX_DRV_SetRxCallbackFunction(const uint32_t instance, srx_callback_func_t function, void * param);

/**
 * @brief Reset the peripheral.
 *
 * De-Initializes the peripheral and brings it's registers into a reset state.
 *
 * @param[in] instance Peripheral instance number
 * @return The status of the operation
 */
status_t SRX_DRV_Deinit(const uint32_t instance);

/**
 * @brief Default configuration structure.
 *
 * Returns a default configuration for the TLE4998 sensor
 *
 * @param[out] config Default configuration structure
 */
void SRX_DRV_GetDefaultConfig(srx_driver_user_config_t * config);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup srx */

#endif /* SRX_DRIVER_H */
