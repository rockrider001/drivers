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
* @file oc_emios_driver.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 */

#ifndef OC_EMIOS_DRIVER_H
#define OC_EMIOS_DRIVER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "emios_common.h"

/*!
 * @defgroup oc_emios_driver Output compare (eMIOS)
 * @ingroup emios
 * @brief Output compare mode (Single action output compare mode, Double action output compare mode).
 * @{
 */

/*******************************************************************************
* Variables
******************************************************************************/
/*!
* @brief Output compare mode
* Implements : emios_oc_mode_config_t_Class
*/
typedef enum
{
    EMIOS_MODE_SAOC                      = 0x03U,                /*!< Single-Action Output Compare */
    EMIOS_MODE_DAOC_FSET_TRAILING_MATCH  = 0x06U,                /*!< Double-Action Output Compare, FLAG set at Trailing edge match */
    EMIOS_MODE_DAOC_FSET_BOTH_MATCH      = 0x07U                 /*!< Double-Action Output Compare, FLAG set at both Leading & Trailing edge match */
} emios_oc_mode_config_t;

/*!
 * @brief Output active mode : Hight, low or toggle
 * Implements : emios_output_active_mode_t_Class
 */
typedef enum
{
    EMIOS_OUTPUT_ACTIVE_LOW              = 0x00U,                /*!< Active low */
    EMIOS_OUTPUT_ACTIVE_HIGH             = 0x01U,                /*!< Active hight */
    EMIOS_OUTPUT_ACTIVE_TOGGLE           = 0x02U,                /*!< Active toggle */
    EMIOS_OUTPUT_ACTIVE_DISABLE          = 0x03U                 /*!< Disable output */
} emios_output_active_mode_t;

/*!
 * @brief Output compare configuration parameters structure
 * Implements : emios_oc_param_t_Class
 */
typedef struct
{
    emios_oc_mode_config_t          mode;                        /*!< Sub-mode selected */
    emios_bus_select_t              timebase;                    /*!< Counter bus selected */
    uint32_t                        matchLeadingEdgeValue;       /*!< Match value of leading edge in DAOC mode
                                                                      Match value of compare mode in SAOC mode */
    uint32_t                        matchTrailingEdgeValue;      /*!< Match value of leading edge
                                                                      Ignore if SAOC mode selected */
    emios_output_active_mode_t      outputActiveMode;            /*!< Output active positive / negative pulse */
} emios_oc_param_t;

/*******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS DRIVER API
 * @{
 */

/*!
 * @brief Initialize Output Compare mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] ocParam A pointer to the DAOC configuration structure
 * @return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_OC_InitOutputCompareMode(uint8_t emiosGroup,
                                            uint8_t channel,
                                            const emios_oc_param_t *ocParam);

/*!
 * @brief Update new SAOC Match value
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newMatchValue New match value to update
 * @return void
 */
void EMIOS_DRV_OC_SetSingleActOutputCmpMatch(uint8_t emiosGroup,
                                             uint8_t channel,
                                             uint32_t newMatchValue);

/*!
 * @brief Get Match value in SAOC mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of Output compare match value
 */
uint32_t EMIOS_DRV_OC_GetSingleActOutputCmpMatch(uint8_t emiosGroup,
                                                 uint8_t channel);

/*!
 * @brief An output compare match can be simulated in software
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_OC_ForceSingleActOutputCmpMatch(uint8_t emiosGroup,
                                               uint8_t channel);

/*!
 * @brief Update Leading edge and Trailing edge position in DAOC mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newLeadingEdgeVal New Leading edge position value
 * @param[in] newTrailingEdgeVal New Trailing edge position value
 * @return void
 */
void EMIOS_DRV_OC_SetDoubleActOutputCmpMatch(uint8_t emiosGroup,
                                             uint8_t channel,
                                             uint32_t newLeadingEdgeVal,
                                             uint32_t newTrailingEdgeVal);

/*!
 * @brief Get DAOC Match value
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[out] retLeadingEdgeVal A pointer to return Leading edge value
 * @param[out] retTrailingEdgeVal A pointer to return Trailing edge value
 * @return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_OC_GetDoubleActOutputCmpMatch(uint8_t emiosGroup,
                                                 uint8_t channel,
                                                 uint32_t * const retLeadingEdgeVal,
                                                 uint32_t * const retTrailingEdgeVal);

/*!
 * @brief Allow the software to force the output flipflop to the level corresponding
 * to a comparison event in leading comparator.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_OC_ForceDoubleActOutputCmpLeadingMatch(uint8_t emiosGroup,
                                                      uint8_t channel);

/*!
 * @brief Allow the software to force the output flipflop to the level corresponding
 * to a comparison event in trailing comparator.
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_OC_ForceDoubleActOutputCmpTrailingMatch(uint8_t emiosGroup,
                                                       uint8_t channel);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* OC_EMIOS_DRIVER_H */
/*******************************************************************************
* EOF
******************************************************************************/


