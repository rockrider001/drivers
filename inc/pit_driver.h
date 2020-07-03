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
 * @file pit_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * This is required to use in DEV_ASSERT.
 */

#ifndef PIT_DRIVER_H
#define PIT_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "device_registers.h"
#include "status.h"

/* */
/* */
/* */

/*!
 * @addtogroup pit_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The total PIT timer channel includes standard and RTI timer channels */
#if FEATURE_PIT_HAS_RTI_CHANNEL
#define PIT_CHANNEL_COUNT (PIT_TIMER_COUNT + 1U)
#else
#define PIT_CHANNEL_COUNT PIT_TIMER_COUNT
#endif
/*! @brief The RTI timer channel index */
#define PIT_RTICHANNEL_INDEX (PIT_TIMER_COUNT)

/*!
 * @brief Unit options for PIT period.
 *
 * This is used to determine unit of timer period
 * Implements : pit_period_units_t_Class
 */
typedef enum
{
    PIT_PERIOD_UNITS_COUNTS        = 0x00U, /*!< Period value unit is count */
    PIT_PERIOD_UNITS_MICROSECONDS  = 0x01U  /*!< Period value unit is microsecond */
} pit_period_units_t;

/*! @brief Structure to configure the PIT
 *
 * This structure holds the configuration settings for the PIT
 * Implements : pit_config_t_Class
 */
typedef struct
{
    bool enableStandardTimers;  /*!< Enable standard timer */
#if FEATURE_PIT_HAS_RTI_CHANNEL
    bool enableRTITimer;        /*!< Enable real time interrupt timer */
#endif
    bool stopRunInDebug;        /*!< Stop timer running in debug mode */
} pit_config_t;

/*! @brief Structure to configure the PIT timer channel
 *
 * This structure holds the configuration settings for the PIT timer channel
 * Implements : pit_channel_config_t_Class
 */
typedef struct
{
    uint8_t                 hwChannel;           /*!< Timer channel number */
    pit_period_units_t      periodUnit;          /*!< Period value unit */
    uint32_t                period;              /*!< Timer channel interrupt generation enable  */
    bool                    enableChain;         /*!< Enable standard timer channel chaining     */
    bool                    enableInterrupt;     /*!< Enable interrupt generation                */
} pit_channel_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Gets the default PIT configuration
 *
 * This function gets default PIT module configuration structure.
 *
 * @param[in] config The configuration structure
 */
void PIT_DRV_GetDefaultConfig(pit_config_t * const config);

/*!
 * @brief Gets the default timer channel configuration
 *
 * This function gets default timer channel configuration structure.
 *
 * @param[in] config The channel configuration structure
 */
void PIT_DRV_GetDefaultChanConfig(pit_channel_config_t * const config);

/*!
 * @brief Initializes the PIT module.
 *
 * This function enables the standard timer and real timer interrupt timer,
 * configures PIT module operation in Debug mode. The PIT configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other PIT driver function.
 *
 * This is an example demonstrating how to define a PIT configuration structure:
   @code
   pit_config_t pitInit =
   {
        .enableStandardTimers = true,
        .enableRTITimer = true,
        .stopRunInDebug = true
   };
   @endcode
 *
 * @param[in] instance PIT module instance number
 * @param[in] config Pointer to PIT configuration structure
 */
void PIT_DRV_Init(const uint32_t instance,
                  const pit_config_t * const config);

/*!
 * @brief De-Initializes the PIT module.
 *
 * This function disables PIT timer and set all PIT register to default value.
 * In order to use the PIT module again, PIT_DRV_Init must be called.
 *
 * @param[in] instance PIT module instance number
 */
void PIT_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Initializes the PIT channel.
 *
 * This function initializes the PIT timers by using a channel. Pass in the channel
 * configuration structure. Timers do not start counting by default after calling this
 * function. The function PIT_DRV_StartChannel must be called to start the timer counting.
 * Call the PIT_DRV_SetTimerPeriodByUs to re-set the period.
 *
 * This is an example demonstrating how to define a PIT channel configuration structure:
   @code
   pit_channel_config_t pitTestInit =
   {
        .hwChannel = 0U,
        .periodUnits = PIT_PERIOD_UNITS_MICROSECONDS,
        .period = 1000000U,
        .enableChain = false,
        .enableInterrupt = true
   };
   @endcode
 *
 * @param[in] instance PIT module instance number
 * @param[in] chnlConfig Pointer to PIT channel configuration structure
 * @return Operation status
 *         - STATUS_SUCCESS: Operation was successful
 *         - STATUS_ERROR: The input period is invalid
 */
status_t PIT_DRV_InitChannel(const uint32_t instance,
                             const pit_channel_config_t * const chnlConfig);

/*!
 * @brief Configure timer channel period.
 *
 * This function sets the timer channel period in microseconds or count base on
 * period unit argument.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @param[in] period Timer channel period
 * @param[in] periodUnit Period unit
 * @return Operation status
 *         - STATUS_SUCCESS: Input period of timer channel is valid
 *         - STATUS_ERROR: Input period of timer channel is invalid
 */
status_t PIT_DRV_ConfigChannel(const uint32_t instance,
                               const uint8_t channel,
                               const uint32_t period,
                               const pit_period_units_t periodUnit);

/* @} */

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function starts every timer channel counting.
 * After calling this function, timer channel loads period value, count down to 0 and
 * then load the respective start value again. Each time a timer channel reaches 0,
 * it generates a trigger pulse and sets the timeout interrupt flag.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 */
void PIT_DRV_StartChannel(const uint32_t instance,
                          const uint8_t channel);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function stops every timer channel counting. Timer channels reload their periods
 * respectively after the next time they call the PIT_DRV_StartChannel.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 */
void PIT_DRV_StopChannel(const uint32_t instance,
                         const uint8_t channel);

/* @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * This function sets the timer channel period in microseconds.
 * The period range depends on the frequency of the PIT source clock. If the required period
 * is out of range, use the lifetime timer if applicable.
 * This function is only valid for one single channel. If channels are chained together,
 * the period here makes no sense.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @param[in] periodUs Timer channel period in microseconds
 * @return Operation status
 *         - STATUS_SUCCESS: Input period of timer channel is valid
 *         - STATUS_ERROR: Input period of timer channel is invalid
 */
status_t PIT_DRV_SetTimerPeriodByUs(const uint32_t instance,
                                    const uint8_t channel,
                                    const uint32_t periodUs);

/*!
 * @brief Gets the current timer channel counting value in microseconds.
 *
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time. Make sure the
 * running time does not exceed the timer channel period. The time stamp returned is
 * down-counting.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @return Current timer channel counting value in microseconds
 */
uint64_t PIT_DRV_GetCurrentTimerUs(const uint32_t instance,
                                   const uint8_t channel);

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit.
 * Timer channel begin counting from the value set by this function.
 * The counter period of a running timer channel can be modified by first stopping
 * the timer channel, setting a new load value, and starting the timer channel again. If
 * channel are not restarted, the new value is loaded after the next trigger
 * event. Note that The RTI channel must not be set to a value lower than 32 cycles,
 * otherwise interrupts may be lost, as it takes several cycles to clear the RTI interrupt
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @param[in] count Timer channel period in count unit
 */
void PIT_DRV_SetTimerPeriodByCount(const uint32_t instance,
                                   const uint8_t channel,
                                   const uint32_t count);

/*!
 * @brief Gets current counter value.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @return Current timer channel counting value in count
 */
uint32_t PIT_DRV_GetCurrentTimerCount(const uint32_t instance,
                                      const uint8_t channel);


/*!
 * @brief Build the 64-bit lifetimer.
 *
 * The lifetime timer is a 64-bit timer which chains timer channel 0 and timer channel 1 together
 * with the start value of both channels is set to the maximum value(0xFFFFFFFF).
 * The period of lifetime timer is equal to the "period of
 * timer 0 * period of timer 1".
 *
 * @param[in] instance PIT module instance number.
 */
void PIT_DRV_SetLifetimeTimerCount(const uint32_t instance);

/*!
 * @brief Reads the current lifetime counter value.
 *
 * The Lifetime timer is 64-bit timer which chains timer 0 and timer 1 together.
 * The period of lifetime timer equals to "period of timer 0 * period of timer 1".
 * This feature returns an absolute time stamp in count. The time stamp
 * value does not exceed the timer period. The timer is up-counting.
 * Calling PIT_DRV_SetLifetimeTimerCount to use this timer.
 *
 * @param[in] instance PIT module instance number.
 * @return Current lifetime timer value
 */
uint64_t PIT_DRV_GetLifetimeTimerCount(const uint32_t instance);

/*!
 * @brief Reads the current lifetime value in microseconds.
 *
 * This feature returns an absolute time stamp in microseconds. The time stamp
 * value does not exceed the timer period. The timer is up-counting.
 *
 * @param[in] instance PIT module instance number
 * @return Current lifetime timer value in microseconds
 */
uint64_t PIT_DRV_GetLifetimeTimerUs(const uint32_t instance);

/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Enables the interrupt generation of timer channel.
 *
 * This function allows enabling interrupt generation of timer channel
 * when timeout occurs.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 */
void PIT_DRV_EnableChannelInterrupt(const uint32_t instance,
                                    const uint8_t channel);

/*!
 * @brief Disables the interrupt generation of timer channel.
 *
 * This function allows disabling interrupt generation of timer channel
 * when timeout occurs.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 */
void PIT_DRV_DisableChannelInterrupt(const uint32_t instance,
                                     const uint8_t channel);

/*!
 * @brief Gets the current interrupt flag of timer channels.
 *
 * This function gets the current interrupt flag of timer channels.
 * Every time the timer channel counts to 0, this flag is set.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 * @return Current status of the timeout flag
 */
uint32_t PIT_DRV_GetStatusFlags(const uint32_t instance,
                                const uint8_t channel);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the timer interrupt flag after a timeout event
 * occurs.
 *
 * @param[in] instance PIT module instance number
 * @param[in] channel Timer channel number.
 */
void PIT_DRV_ClearStatusFlags(const uint32_t instance,
                              const uint8_t channel);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* PIT_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
