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
/*!
* @file ic_emios_driver.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13,  Pointer parameter 'icParam' could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * This macro is used by user.
 *
 */

#ifndef IC_EMIOS_DRIVER_H
#define IC_EMIOS_DRIVER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "emios_common.h"

/*!
 * @defgroup ic_emios_driver Input capture (eMIOS)
 * @ingroup emios
 * @brief Input capture mode (measure time events, period measurement).
 * @{
 */

/*******************************************************************************
* Variables
******************************************************************************/
#define   EMIOS_MODE_PULSE_WIDTH              0x04U                 /*!< Input Pulse-Width Measurement */
#define   EMIOS_MODE_PERIOD                   0x05U                 /*!< Input Period Measurement */

#define   EMIOS_FALLING_EDGE                  0x00U                 /*!< Falling edge capturing */
#define   EMIOS_RISING_EDGE                   0x01U                 /*!< Rising edge capturing */

#define   EMIOS_MODE_SAIC                     0x02U                 /*!< Single-Action Input Capture */

/*!
* @brief Input capture mode
* Implements : emios_ic_mode_config_t_Class
*/
typedef enum
{
    EMIOS_MODE_IC                           = 0x05U                 /*!< Input Capture mode */
} emios_ic_mode_config_t;

/*!
 * @brief Input capture measurement type for input capture mode
 * Implements : emios_input_capture_mode_t_Class
 */
typedef enum
{
    EMIOS_CAPTURE_TRIGGER_EDGE_FALLING      = 0x00U,                /*!< Falling edge trigger */
    EMIOS_CAPTURE_TRIGGER_EDGE_RISING       = 0x01U,                /*!< Rising edge trigger */
    EMIOS_CAPTURE_TRIGGER_EDGE_ANY          = 0x02U,                /*!< Rising and falling edge trigger */
    EMIOS_RISING_EDGE_PERIOD_MEASUREMENT    = 0x03U,                /*!< Period measurement between two consecutive rising edges */
    EMIOS_FALLING_EDGE_PERIOD_MEASUREMENT   = 0x04U,                /*!< Period measurement between two consecutive falling edges */
    EMIOS_PERIOD_ON_MEASUREMENT             = 0x05U,                /*!< The time measurement taken for the pulse to remain ON or HIGH state */
    EMIOS_PERIOD_OFF_MEASUREMENT            = 0x06U                 /*!< The time measurement taken for the pulse to remain OFF or LOW state */
} emios_input_capture_mode_t;

/*!
 * @brief Input Period Measurement(IPM) configuration parameters structure
 * Implements : emios_input_capture_param_t_Class
 */
typedef struct
{
    emios_ic_mode_config_t                    mode;                 /*!< Sub-mode selected */
    emios_bus_select_t                        timebase;             /*!< Counter bus selected */
    emios_input_filter_t                      filterInput;          /*!< Filter Value */
    bool                                      filterEn;             /*!< Input capture filter state */
    emios_input_capture_mode_t                inputCaptureMode;     /*!< Input capture mode */
} emios_input_capture_param_t;

/*******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS DRIVER API
 * @{
 */

/*!
 * @brief Initialize Input Capture Mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] icParam A pointer to the IPWM configuration structure
 * @return operation status
 *        - STATUS_SUCCESS   :  Operation was successful.
 *        - STATUS_ERROR     :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_IC_InitInputCaptureMode(uint8_t emiosGroup,
                                           uint8_t channel,
                                           const emios_input_capture_param_t *icParam);

/*!
 * @brief Get last measurement value in input capture mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[out] retVal A pointer to return value
 * @return operation status
 *        - STATUS_SUCCESS                 :  Operation was successful.
 *        - STATUS_ERROR                   :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_IC_GetLastMeasurement(uint8_t emiosGroup,
                                         uint8_t channel,
                                         uint32_t * const retValue);
/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* IC_EMIOS_DRIVER_H */
/*******************************************************************************
* EOF
******************************************************************************/


