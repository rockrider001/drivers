/*
 * Copyright 2016-2019 NXP
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

#ifndef POWER_MC_ME_HW_ACCESS_H
#define POWER_MC_ME_HW_ACCESS_H

#include "power_manager_MPC57xx.h"

/*!
 * @file power_mc_me_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * User configuration structure is defined in hal and is referenced from driver.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * There are some global macros that can be used by user code for range checking.
 */

/*!
 * power_mc_me_hw_access
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name Mode Entry Module APIs*/
/*@{*/

/*!
 * @brief Get the current mode status of the MC_ME module
 *
 * @param[in]  baseAddr  Base address of the MC_ME module
 * @param[out] status    The current mode status
 */
void MC_ME_GetStatus(const MC_ME_Type * const baseAddr,
                     power_manager_user_config_t * const status);

/*!
 * @brief Configures the power mode.
 *
 * This function triggers software-controlled mode changes. Depending on the
 * modes as enabled by ME register bits, configurations corresponding to unavailable modes
 * are reserved and access to <mode>_MC registers must respect this for successful mode
 * requests.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param mode      Target power mode
 */
void MC_ME_SetPowerMode(MC_ME_Type * const baseAddr,
                        power_manager_modes_t mode);

/*!
 * @brief Returns if a mode switch transition is currently in progress
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @return          True if a mode switch transitions is in progress, false otherwise
 */
bool MC_ME_ModeTransitionInProgress(const MC_ME_Type * const baseAddr);

/*!
 * @brief Enables the power mode.
 *
 * This function enables user modes. RESET, DRUN, and RUN0 modes are always enabled.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param mode      Power mode
 */
void MC_ME_EnablePowerMode(MC_ME_Type * const baseAddr,
                           power_manager_modes_t mode);

/*!
 * @brief Disables the power mode.
 *
 * This function allows disabling the chip modes which are not required for a given
 * chip. RESET, SAFE, DRUN, and RUN0 modes are always enabled
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param mode      Power mode
 */
void MC_ME_DisablePowerMode(MC_ME_Type * const baseAddr,
                            power_manager_modes_t mode);

/*!
 * @brief Applies the provided run mode configuration.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param config    Run mode configuration structure
 */
void MC_ME_SetRunModeConfig(MC_ME_Type * const baseAddr,
                            const power_manager_user_config_t * const config);

/*!
 * @brief Function checks clock switching in progress
 * This function checks the clock switching complete.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @return status of clock switching in progress.
 */
bool MC_ME_CheckSystemClkInProgress(const MC_ME_Type * const baseAddr);

/*!
 * @brief Function gets previous mode of chip.
 * This function gets mode which the chip was jump from lastest mode to the current mode.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @return power mode of chip.
 */
void MC_ME_GetPreviousMode(const MC_ME_Type * const baseAddr,
                           power_manager_modes_t * mode);

/*!
 * @brief Function gets status of SDPLL bit in another mode.
 * This function checks SDPLL was enabled or disabled
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param mode      the power mode which user want to check.
 *
 * @return status of SDPLL was true or false.
 */
bool MC_ME_GetSdpllEnable(const MC_ME_Type * const baseAddr,
                          power_manager_modes_t mode);
/*!
 * @brief Function sets or resets SDPLL bit in Runx mode.
 *
 * @param baseAddr  Base address for current MC_ME instance.
 * @param mode      the power mode which user want to check.
 * @param enable    set or clear SDPLL in RUNx register.
 *                  true: set SDPLL
 *                  false: reset SDPLL
 *
 */
void MC_ME_EnableSdpllRun(MC_ME_Type * const baseAddr,
                          power_manager_modes_t mode,
                          bool enable);
/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* POWER_MC_ME_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
