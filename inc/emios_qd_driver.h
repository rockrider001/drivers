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
/*!
* @file qd_emios_driver.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13,  Pointer parameter 'qdParam' could be declared as pointing to const
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

#ifndef QD_EMIOS_DRIVER_H
#define QD_EMIOS_DRIVER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "emios_common.h"

/*!
 * @defgroup qd_emios_driver Quadrature decode (eMIOS)
 * @ingroup emios
 * @brief Quadrature decode mode (measure time events, period measurement follow phase encode or count and direction mode).
 * @{
 */

/*******************************************************************************
* Variables
******************************************************************************/

/*!
* @brief EMIOS quadrature decode modes, phase encode or count and direction mode
* Implements : emios_qd_mode_t_Class
*/
typedef enum
{
    EMIOS_MODE_COUNT_AND_DIR_ENCODE   = 0x0CU,    /*!< Quadrature Decode (for count and direction encoders type) */

    EMIOS_MODE_PHASE_ENCODE           = 0x0DU     /*!< Quadrature Decode (for phase_A and phase_B encoders type) */
} emios_qd_mode_t;

/*!
 * @brief EMIOS quadrature configure structure for input programmer filter
 *
 * Implements : emios_qd_ipf_config_t_Class
 */
typedef struct
{
    emios_pulse_polarity_mode_t     inputActiveMode;        /*!< Input active value, Choose active low or high level                           */
    emios_clock_internal_ps_t       internalPrescaler;      /*!< Internal prescaler,  prescaler channel clock by internalPrescaler + 1         */
    bool                            internalPrescalerEn;    /*!< Internal prescaler Enable                                                     */
    emios_input_filter_t            filterInput;            /*!< Filter Value                                                                  */
    bool                            filterEn;               /*!< Input capture filter state                                                    */
    emios_edge_trigger_mode_t       triggerMode;            /*!< Input signal trigger mode,                                                    */
} emios_qd_ipf_config_t;

/*!
 * @brief EMIOS quadrature configure structure
 *
 * Implements : emios_qd_config_t_Class
 */
typedef struct
{
    emios_qd_ipf_config_t           ipfConfig;              /*!< Configuration for Input programmer filter                                     */
    emios_qd_mode_t                 mode;                   /*!< EMIOS_QUAD_PHASE_ENCODE or EMIOS_QUAD_COUNT_AND_DIR                           */
    uint32_t                        initialCounterVal;      /*!< Initial counter value                                                         */
    uint32_t                        matchesValue;           /*!< Initial A1 matches value                                                      */
    emios_pulse_polarity_mode_t     inputActiveMode;        /*!< Input active value, Choose active low or high level                           */
    emios_clock_internal_ps_t       internalPrescaler;      /*!< Internal prescaler, prescaler channel clock by internalPrescaler + 1          */
    bool                            internalPrescalerEn;    /*!< Internal prescaler Enable                                                     */
    emios_input_filter_t            filterInput;            /*!< Filter Value                                                                  */
    bool                            filterEn;               /*!< Input capture filter state                                                    */
    emios_edge_trigger_mode_t       triggerMode;            /*!< Input signal trigger mode,                                                    */
    bool                            enableFlagIsr;          /*!< True then FLAG generates an interrupt request, else disable interrupt request */
} emios_qd_config_t;

/*!
 * @brief EMIOS quadrature state(counter value and flags)
 *
 * Implements : emios_qd_state_t_Class
 */
typedef struct
{
    uint32_t counterValue;      /*!< Counter value                                                           */
    bool     overRun;           /*!< True if overrun has occurred, False if overrun has not occurred         */
    bool     overflowCounter;   /*!< True if overflow has occurred, False if overflow has not occurred       */
    bool     overFlag;          /*!< True if a match event in the comparators occurred else then no occurred */
} emios_qd_state_t;

/*******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS DRIVER API
 * @{
 */

/*!
 * @brief Initialization the quadrature decode mode and started measurement.
 *
 * @param[in] emiosGroup The eMIOS group id.
 * @param[in] channel The channel in this eMIOS group.
 * @param[in] qdParam Configuration structure: Quadrature decode mode, polarity for both phases,
 *                      initial value for the counter, filter configuration, interrupt request and disable prescaler(no clock).
 * @return
 *        - STATUS_SUCCESS: Completed successfully.
 *        - STATUS_ERROR  : If configuration mode do not EMIOS_MODE_COUNT_AND_DIR_ENCODE or EMIOS_MODE_PHASE_ENCODE.
 */
status_t EMIOS_DRV_QuadDecodeStart(uint8_t emiosGroup,
                                   uint8_t channel,
                                   const emios_qd_config_t *qdParam);

/*!
 * @brief De-activates the quadrature decode mode.
 *
 * @param[in] emiosGroup The eMIOS group id.
 * @param[in] channel The channel in this eMIOS group.
 * @return
 *        - STATUS_SUCCESS: Completed successfully.
 *        - STATUS_ERROR  : If configured mode did not EMIOS_MODE_COUNT_AND_DIR_ENCODE or EMIOS_MODE_PHASE_ENCODE.
 */
status_t EMIOS_DRV_QuadDecodeStop(uint8_t emiosGroup,
                                  uint8_t channel);

/*!
 * @brief Return the current quadrature decoder state (counter value, overflow flag and
 * overflow direction)
 *
 * @param[in] emiosGroup The eMIOS group id.
 * @param[in] channel The channel in this eMIOS group.
 * @param[out] qdState The current state of quadrature decoder
 * @return
 *        - STATUS_SUCCESS: Completed successfully.
 *        - STATUS_ERROR  : If configuration mode do not EMIOS_MODE_COUNT_AND_DIR_ENCODE or EMIOS_MODE_PHASE_ENCODE.
 */
status_t EMIOS_DRV_QuadDecodeGetState(uint8_t emiosGroup,
                                      uint8_t channel,
                                      emios_qd_state_t * const qdState);

/*!
 * @brief This function will get the default configuration values
 *        in the structure which is used as a common use-case.
 * @param[out] config Pointer to the structure in which the
 *                    configuration will be saved.
 * @return None
 */
void EMIOS_DRV_QuadDecodeGetDefaultConfig(emios_qd_config_t * const config);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* QD_EMIOS_DRIVER_H */
/*******************************************************************************
* EOF
******************************************************************************/



