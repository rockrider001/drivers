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
* @file mc_emios_driver.h
*/

/**
 * @page misra_violations MISRA-C:2012 violations
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
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter 'mcbParam' could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 *
 */

#ifndef MC_EMIOS_DRIVER_H
#define MC_EMIOS_DRIVER_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "emios_common.h"

/*!
 * @defgroup mc_emios_driver Counter (eMIOS)
 * @ingroup emios
 * @brief Modulus counter mode.
 * @{
 */

/*******************************************************************************
* Variables
******************************************************************************/
/*!
* @brief Modulus counter mode
* Implements : emios_mc_mode_config_t_Class
*/
typedef enum
{
    EMIOS_MODE_MC_UP_CNT_CLR_START_INT_CLK       = 0x10U,     /*!< Modulus Counter (Up counter with clear on match start), internal clock */
    EMIOS_MODE_MC_UP_CNT_CLR_START_EXT_CLK       = 0x11U,     /*!< Modulus Counter (Up counter with clear on match start), external clock */
    EMIOS_MODE_MC_UP_CNT_CLR_END_INT_CLK         = 0x12U,     /*!< Modulus Counter (Up counter with clear on match end), internal clock */
    EMIOS_MODE_MC_UP_CNT_CLR_END_EXT_CLK         = 0x13U,     /*!< Modulus Counter (Up counter with clear on match end), external clock */

    EMIOS_MODE_MC_UPDOWN_CNT_FLAGX1_INT_CLK      = 0x14U,     /*!< Modulus Counter (Up/Down counter), internal clock */
    EMIOS_MODE_MC_UPDOWN_CNT_FLAGX1_EXT_CLK      = 0x15U,     /*!< Modulus Counter (Up/Down counter), external clock */
    EMIOS_MODE_MC_UPDOWN_CNT_FLAGX2_INT_CLK      = 0x16U,     /*!< Modulus Counter (Up/Down counter), internal clock */
    EMIOS_MODE_MC_UPDOWN_CNT_FLAGX2_EXT_CLK      = 0x17U,     /*!< Modulus Counter (Up/Down counter), external clock */

    EMIOS_MODE_MCB_UP_COUNTER_INT_CLK            = 0x50U,     /*!< Modulus Counter Buffered (Up counter), using internal clock */
    EMIOS_MODE_MCB_UP_COUNTER_EXT_CLK            = 0x51U,     /*!< Modulus Counter Buffered (Up counter), using external clock */

    EMIOS_MODE_MCB_UPDOWN_CNT_FLAGX1_INT_CLK     = 0x54U,     /*!< Modulus Counter Buffered (Up/Down counter),
                                                                   Flags are generated only at A1 match start, Using internal clock */
    EMIOS_MODE_MCB_UPDOWN_CNT_FLAGX1_EXT_CLK     = 0x55U,     /*!< Modulus Counter Buffered (Up/Down counter),
                                                                   Flags are generated only at A1 match start, Using external clock */
    EMIOS_MODE_MCB_UPDOWN_CNT_FLAGX2_INT_CLK     = 0x56U,     /*!< Modulus Counter Buffered (Up/Down counter),
                                                                   Flags are generated at A1 match start and cycle boundary, Using internal clock */
    EMIOS_MODE_MCB_UPDOWN_CNT_FLAGX2_EXT_CLK     = 0x57U      /*!< Modulus Counter Buffered (Up/Down counter),
                                                                   Flags are generated at A1 match start and cycle boundary, Using external clock */
} emios_mc_mode_config_t;

/*!
 * @brief MC configuration parameters structure
 * Implements : emios_mc_mode_param_t_Class
 */
typedef struct
{
    emios_mc_mode_config_t          mode;                     /*!< Sub-mode selected */
    uint32_t                        period;                   /*!< If up mode period = A1, period = 2(A1) with MC up/down mode,
                                                                   period = 2(A1) -2 with MCB up/down mode */
    emios_clock_internal_ps_t       internalPrescaler;        /*!< Internal prescaler,  pre-scale channel clock by internalPrescaler +1 */
    bool                            internalPrescalerEn;      /*!< Internal prescaler Enable */
    emios_input_filter_t            filterInput;              /*!< Filter Value, ignore if not select external clock mode */
    bool                            filterEn;                 /*!< Input capture filter state, ignore if not select external clock mode */
    emios_edge_trigger_mode_t       triggerMode;              /*!< Input signal trigger mode, ignore if not select external clock mode */
} emios_mc_mode_param_t;

/*******************************************************************************
* API
******************************************************************************/
/*!
 * @name eMIOS DRIVER API
 * @{
 */

/*!
 * @brief Initialize eMIOS Counter mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] mcParam A pointer to the counter configuration structure
 * @return operation status
 *        - STATUS_SUCCESS  :  Operation was successful.
 *        - STATUS_ERROR    :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_MC_InitCounterMode(uint8_t emiosGroup,
                                      uint8_t channel,
                                      const emios_mc_mode_param_t *mcParam);

/*!
 * @brief Update new period value for counter mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @param[in] newPeriod The new period value
 * @return operation status
 *        - STATUS_SUCCESS            :  Operation was successful.
 *        - STATUS_EMIOS_WRONG_MODE   :  Can not set period to your mode.
 *        - STATUS_ERROR              :  Operation failed, invalid input value.
 */
status_t EMIOS_DRV_MC_SetCounterPeriod(uint8_t emiosGroup,
                                       uint8_t channel,
                                       uint32_t newPeriod);

/*!
 * @brief Read Counter period value in counter mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of counter period
          - 0        : If have error
          - non zero : If success
 */
uint32_t EMIOS_DRV_MC_GetCounterPeriod(uint8_t emiosGroup,
                                       uint8_t channel);

/*!
 * @brief Get current counter value in counter mode
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return uint32_t Value of current counter value
 */
uint32_t EMIOS_DRV_MC_CounterRead(uint8_t emiosGroup,
                                  uint8_t channel);

/*!
 * @brief Start counter, note that counter start automaticaly after initialization
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_MC_CounterStart(uint8_t emiosGroup,
                               uint8_t channel);

/*!
 * @brief Stop counter
 *
 * @param[in] emiosGroup The eMIOS group id
 * @param[in] channel The channel in this eMIOS group
 * @return void
 */
void EMIOS_DRV_MC_CounterStop(uint8_t emiosGroup,
                              uint8_t channel);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* MC_EMIOS_DRIVER_H */
/*******************************************************************************
* EOF
******************************************************************************/

