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

#ifndef WKPU_DRIVER_H
#define WKPU_DRIVER_H

/*! @file wkpu_driver.h */

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/* */
/* */
/* */

/*!
 * @defgroup wkpu_driver WKPU Driver
 * @ingroup wkpu
 * @brief This section describes the programming interface of the WKPU driver.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief NMI destination source.
 * Implements : wkpu_nmi_destination_src_t_Class
 */
typedef enum
{
#ifdef FEATURE_WKPU_SUPPORT_NONE_REQUEST
    WKPU_NMI_NONE               = 3U,    /*!< No NMI, critical interrupt, or machine check request generated */
#endif
#ifdef FEATURE_WKPU_SUPPORT_MACHINE_CHK_REQ
    WKPU_NMI_MACHINE_CHK_REQ    = 2U,   /*!< Machine check request interrupt */
#endif
#ifdef FEATURE_WKPU_SUPPORT_CRITICAL_INT
    WKPU_NMI_CRITICAL_INT       = 1U,   /*!< Critical interrupt */
#endif
#ifdef FEATURE_WKPU_SUPPORT_NON_MASK_INT
    WKPU_NMI_NON_MASK_INT       = 0U   /*!< Non-maskable interrupt */
#endif
} wkpu_nmi_destination_src_t;

/*!
 * @brief Edge event.
 * Implements : wkpu_edge_event_t_Class
 */
typedef enum
{
    WKPU_EDGE_NONE      = 0U,   /*!< None event */
    WKPU_EDGE_RISING    = 1U,   /*!< Rising edge event */
    WKPU_EDGE_FALLING   = 2U,   /*!< Falling edge event */
    WKPU_EDGE_BOTH      = 3U    /*!< Both rising and falling edge event */
} wkpu_edge_event_t;

/*!
 * @brief Status Flag.
 * Implements : wkpu_status_flag_t_Class
 */
typedef enum
{
    WKPU_FLAG_STATUS    = 0U,   /*!< Status flag */
    WKPU_FLAG_OVERRUN   = 1U    /*!< Overrun status flag */
} wkpu_status_flag_t;

/*!
 * @brief NMI configuration structure.
 * Implements : wkpu_nmi_cfg_t_Class
 */
typedef struct
{
    wkpu_core_t core;                               /*!< WKPU core source */
    wkpu_nmi_destination_src_t destinationSrc;      /*!< NMI destination source */
    bool wkpReqEn;                                  /*!< NMI request enable */
    bool filterEn;                                  /*!< NMI filter enable */
    wkpu_edge_event_t edgeEvent;                    /*!< NMI edge events */
    bool lockEn;                                    /*!< NMI configuration lock register */
} wkpu_nmi_cfg_t;


#ifdef FEATURE_WKPU_SUPPORT_INTERRUPT_REQUEST
/*!
 * @brief WKPU interrupt configuration structure.
 * Implements : wkpu_interrupt_cfg_t_Class
 */
typedef struct
{
    uint8_t hwChannel;              /*!< The WKPU hardware channel */
    wkpu_edge_event_t edgeEvent;    /*!< WKPU/interrupt edge events */
    bool filterEn;                  /*!< WKPU/interrupt filter enable */
    bool pullEn;                    /*!< WKPU/interrupt pull enable */
} wkpu_interrupt_cfg_t;
#endif


#ifdef FEATURE_WKPU_SUPPORT_RESET_REQUEST
/*!
 * @brief Reset destination source.
 * Implements : wkpu_reset_destination_src_t_Class
 */
typedef enum
{
    WKPU_RESET_REQ_RGM         = 0U,    /*!< Reset request to RGM */
    WKPU_RESET_NO_REACTION     = 1U     /*!< No reaction */
} wkpu_reset_destination_src_t;

/*!
 * @brief Reset configuration structure.
 * Implements : wkpu_reset_cfg_t_Class
 */
typedef struct
{
    wkpu_reset_destination_src_t destinationSrc;    /*!< Reset destination source */
    bool wkpReqEn;                                  /*!< Reset wakeup request enable */
    wkpu_edge_event_t edgeEvent;                    /*!< Reset edge events  */
    bool lockEn;                                    /*!< Reset configuration lock register */
} wkpu_reset_cfg_t;
#endif


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name WKPU DRIVER API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes NMI of the WKPU module
 *
 * This function sets the NMI configuration.
 * Note that: The user must make sure that the NMI configuration must have configure for core 0,
 * the filter only configured at this core, other cores will be ignored
 *
 * @param[in] instance The WKPU instance number
 * @param[in] coreCnt The number of cores
 * @param[in] pNmiConfig Pointer to structure of NMI configuration
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_InitNMI(uint32_t instance,
                          uint8_t coreCnt,
                          const wkpu_nmi_cfg_t * pNmiConfig);

/*!
 * @brief De-initializes NMI of the WKPU module
 *
 * This function de-initializes NMI of the WKPU module.
 * Reset NMI configuration, disable Wake-up, clear filter enable, edge events enable
 *
 * @param[in] instance The WKPU instance number
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_DeinitNMI(uint32_t instance);

/*!
 * @brief Gets NMI default configuration
 *
 * This function gets NMI default configuration.
 * Note that: The user need provides an array have 3 elements of NMI configuration
 *
 * @param[out] pNmiConfig Pointer to structure of NMI configuration
 */
void WKPU_DRV_GetNMIDefaultConfig(wkpu_nmi_cfg_t * const pNmiConfig);

/*!
 * @brief Clears NMI flag
 *
 * This function clears the NMI (status or overrun) flag for each core
 *
 * @param[in] instance The WKPU instance number
 * @param[in] core The core number
 * @param[in] flag The NMI flag (status/overrun)
 */
void WKPU_DRV_ClearNMIFlag(uint32_t instance,
                           wkpu_core_t core,
                           wkpu_status_flag_t flag);


#ifdef FEATURE_WKPU_SUPPORT_INTERRUPT_REQUEST
/*!
 * @brief Initializes the interrupt WKPU module
 *
 * This function initializes interrupt WKPU driver based on user configuration input.
 * The channelCnt takes values between 1 and the maximum channel count supported by the hardware
 *
 * @param[in] instance The WKPU instance number
 * @param[in] channelCnt The number of channels
 * @param[in] pInterruptConfig Pointer to structure of interrupt configuration for each the channel
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_InitInterrupt(uint32_t instance,
                                uint8_t channelCnt,
                                const wkpu_interrupt_cfg_t * pInterruptConfig);

/*!
 * @brief De-initializes the interrupt WKPU module
 *
 * This function de-initializes the interrupt WKPU module.
 * Reset interrupt configuration, disable IRQ and Wake-up, clear filter enable,
 * pull-up enable, edge events enable.
 *
 * @param[in] instance The WKPU instance number
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_DeinitInterrupt(uint32_t instance);

/*!
 * @brief Gets interrupt default configuration
 *
 * This function gets interrupt default configuration.
 * Note that: The user need provides an array have 32 elements of interrupt configuration
 *
 * @param[out] pInterruptConfig Pointer to structure of interrupt configuration for each the channel
 */
void WKPU_DRV_GetInterruptDefaultConfig(wkpu_interrupt_cfg_t * const pInterruptConfig);

/*!
 * @brief Configures for interrupt
 *
 * This function sets the interrupt configuration based on interrupt configuration input
 *
 * @param[in] instance The WKPU instance number
 * @param[in] pInterruptConfig Pointer to structure of interrupt configuration
 */
void WKPU_DRV_SetInterruptConfig(uint32_t instance,
                                 const wkpu_interrupt_cfg_t * pInterruptConfig);

/*!
 * @brief Clears interrupt Configuration
 *
 * This function clears interrupt configuration, disable IRQ and Wake-up,
 * clear filter enable, pull-up enable, edge events enable
 *
 * @param[in] instance The WKPU instance number
 * @param[in] hwChannel The WKPU hardware channel
 */
void WKPU_DRV_ClearInterruptConfig(uint32_t instance,
                                   uint8_t hwChannel);

/*!
 * @brief Sets interrupt normal mode
 *
 * This function sets interrupt normal mode(enable) for the WKPU,
 * enable interrupt and wake-up
 *
 * @param[in] instance The WKPU instance number
 * @param[in] hwChannel The WKPU hardware channel
 */
void WKPU_DRV_SetInterruptNormalMode(uint32_t instance,
                                     uint8_t hwChannel);

/*!
 * @brief Sets interrupt sleep mode
 *
 * This function sets sleep mode for the WKPU or disable interrupt
 * and wake-up
 *
 * @param[in] instance The WKPU instance number
 * @param[in] hwChannel The WKPU hardware channel
 */
void WKPU_DRV_SetInterruptSleepMode(uint32_t instance,
                                    uint8_t hwChannel);

/*!
 * @brief Sets edge events
 *
 * This function sets edge events for each channel of the WKPU
 *
 * @param[in] instance The WKPU instance number
 * @param[in] hwChannel The WKPU hardware channel
 * @param[in] edge The edge event for interrupt
 */
void WKPU_DRV_SetInterruptEdgeEvent(uint32_t instance,
                                    uint8_t hwChannel,
                                    wkpu_edge_event_t edge);

/*!
 * @brief Clears interrupt flag
 *
 * This function clears interrupt flag for channel mask
 *
 * @param[in] instance The WKPU instance number
 * @param[in] hwChannel The WKPU hardware channel
 */
void WKPU_DRV_ClearInterruptFlag(uint32_t instance,
                                 uint8_t hwChannel);
#endif

#ifdef FEATURE_WKPU_SUPPORT_RESET_REQUEST
/*!
 * @brief Initializes the reset WKPU module
 *
 * This function sets reset configuration of the WKPU based on reset configuration input
 *
 * @param[in] instance The WKPU instance number
 * @param[in] pResetConfig Pointer to structure of reset configuration
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_InitReset(uint32_t instance,
                            const wkpu_reset_cfg_t * pResetConfig);

/*!
 * @brief De-initializes the Reset WKPU module
 *
 * This function de-initializes the Reset WKPU module. Reset configuration,
 * disable Wake-up, edge events enable and lock enable.
 *
 * @param[in] instance The WKPU instance number
 * @return Execution status (success/error)
 */
status_t WKPU_DRV_DeinitReset(uint32_t instance);

/*!
 * @brief Gets Reset default configuration
 *
 * This function gets Reset default configuration.
 *
 * @param[out] pNmiConfig Pointer to structure of NMI configuration
 */
void WKPU_DRV_GetResetDefaultConfig(wkpu_reset_cfg_t * const pResetConfig);

/*!
 * @brief Clears Reset flag
 *
 * This function clears the Reset (status or overrun) flag
 *
 * @param[in] instance The WKPU instance number
 * @param[in] flag The NMI flag (status/overrun)
 */
void WKPU_DRV_ClearResetFlag(uint32_t instance,
                             wkpu_status_flag_t flag);
#endif

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* WKPU_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
