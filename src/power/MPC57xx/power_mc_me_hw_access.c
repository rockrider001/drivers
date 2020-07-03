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
/*!
 * @file power_mc_me_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Likely use of null pointer 'unknown-name' in
 * left argument to operator '->'
 * This is caused by the offsetof implementation.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower
 * or different essential type
 * This is required by the conversion of a unsigned value of a bitfield/bit into a enum value.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
 * unsigned' to 'essentially enum<i>' / 'essentially Boolean' or vice-versa
 * This is required by the conversions of bitfields of registers to/from an enum, bool
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 *(different essential type categories).
 * This is required by the conversion of a bit/bitfield of a register into boolean or a enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.8, Attempt to cast away const from a pointer
 * Writing MISRA-compliant code in this case causes 'type qualifier is meaningless on cast type'
 * warnings on some compilers.
 */

#include <stddef.h>
#include <stdbool.h>
#include "power_mc_me_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_GetStatus
 * Description   : Get the current mode status of the MC_ME module
 * This function get the current mode status of the MC_ME module
 *
 *END**************************************************************************/
void MC_ME_GetStatus(const MC_ME_Type * const baseAddr,
                     power_manager_user_config_t * const status)
{
    uint32_t regValue = baseAddr->GS;

    status->powerMode = (power_manager_modes_t)((regValue & MC_ME_GS_S_CURRENT_MODE_MASK) >> MC_ME_GS_S_CURRENT_MODE_SHIFT);
    switch (status->powerMode)
    {
        /* Test mode */
#if FEATURE_MC_ME_HAS_TEST_MODE
        case POWER_MANAGER_TEST:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->TEST_MC & MC_ME_TEST_MC_PWRLVL_MASK) >> MC_ME_TEST_MC_PWRLVL_SHIFT);
            break;
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
        /* SAFE */
        case POWER_MANAGER_SAFE:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->SAFE_MC & MC_ME_SAFE_MC_PWRLVL_MASK) >> MC_ME_SAFE_MC_PWRLVL_SHIFT);
            break;
        /* DRUN */
        case POWER_MANAGER_DRUN:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->DRUN_MC & MC_ME_DRUN_MC_PWRLVL_MASK) >> MC_ME_DRUN_MC_PWRLVL_SHIFT);
            break;
        /* RUN0 */
        case POWER_MANAGER_RUN0:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->RUN0_MC & MC_ME_RUN0_MC_PWRLVL_MASK) >> MC_ME_RUN0_MC_PWRLVL_SHIFT);
            break;
        /* RUN1 */
        case POWER_MANAGER_RUN1:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->RUN1_MC & MC_ME_RUN1_MC_PWRLVL_MASK) >> MC_ME_RUN1_MC_PWRLVL_SHIFT);
            break;
        /* RUN2 */
        case POWER_MANAGER_RUN2:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->RUN2_MC & MC_ME_RUN2_MC_PWRLVL_MASK) >> MC_ME_RUN2_MC_PWRLVL_SHIFT);
            break;
        /* RUN3 */
        case POWER_MANAGER_RUN3:
            status->powerLevel = (mc_me_power_level_t)((baseAddr->RUN3_MC & MC_ME_RUN3_MC_PWRLVL_MASK) >> MC_ME_RUN3_MC_PWRLVL_SHIFT);
            break;
        /* HALT */
#if FEATURE_MC_ME_HAS_HALT_MODE
        case POWER_MANAGER_HALT0:
            status->powerLevel = MC_ME_PWRLVL_0;
            break;
#endif /* #if FEATURE_MC_ME_HAS_HALT_MODE */
        /* STOP */
        case POWER_MANAGER_STOP0:
            status->powerLevel = MC_ME_PWRLVL_0;
            break;
        /* STANDBY */
#if FEATURE_MC_ME_HAS_STANDBY_MODE
        case POWER_MANAGER_STANDBY0:
            status->powerLevel = MC_ME_PWRLVL_0;
            break;
#endif /* #if FEATURE_MC_ME_HAS_STANDBY_MODE */
        default :
            DEV_ASSERT(0);
            break;
    }
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
    status->flashMode = (mc_me_flash_mode_t)((regValue & MC_ME_GS_S_FLA_MASK) >> MC_ME_GS_S_FLA_SHIFT);
#endif /* FEATURE_MC_ME_HAS_FLAON_CONFIG */
    status->outputPowerdown = (bool)((regValue & MC_ME_GS_S_PDO_MASK) >> MC_ME_GS_S_PDO_SHIFT);
    status->mainVoltage = (bool)((regValue & MC_ME_GS_S_MVR_MASK) >> MC_ME_GS_S_MVR_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_SetPowerMode
 * Description   : This function triggers software-controlled mode changes. Depending on the
 * modes as enabled by ME register bits, configurations corresponding to unavailable modes
 * are reserved and access to <mode>_MC registers must respect this for successful mode
 * requests.
 *
 *END**************************************************************************/
void MC_ME_SetPowerMode(MC_ME_Type * const baseAddr,
                        power_manager_modes_t mode)
{
    /* The mechanism to change mode requires the write operation twice: first time with key, and second time with inverted key */
    baseAddr->MCTL = MC_ME_MCTL_TARGET_MODE(mode) | MC_ME_MCTL_KEY(FEATURE_MC_ME_KEY);
    baseAddr->MCTL = MC_ME_MCTL_TARGET_MODE(mode) | MC_ME_MCTL_KEY(FEATURE_MC_ME_KEY_INV);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_ModeTransitionInProgress
 * Description   : Returns if a mode switch transition is currently in progress.
 *
 *END**************************************************************************/
bool MC_ME_ModeTransitionInProgress(const MC_ME_Type * const baseAddr)
{
    uint32_t regValue = baseAddr->GS;

    return (bool)((((regValue & MC_ME_GS_S_MTRANS_MASK) >> MC_ME_GS_S_MTRANS_SHIFT) != 0U) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_DisablePowerMode
 * Description   : This function allows disabling the chip modes which are not required for a given
 * chip. RESET, SAFE, DRUN, and RUN0 modes are always enabled.
 *
 *END**************************************************************************/
void MC_ME_DisablePowerMode(MC_ME_Type * const baseAddr,
                            power_manager_modes_t mode)
{
    baseAddr->ME &= ~(uint32_t)(1U << mode);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_EnablePowerMode
 * Description   : This function enables user modes. RESET, SAFE, DRUN, and RUN0 modes are always enabled.
 *
 *END**************************************************************************/
void MC_ME_EnablePowerMode(MC_ME_Type * const baseAddr,
                           power_manager_modes_t mode)
{
    baseAddr->ME |= (uint32_t)(1U << mode);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_SetRunModeConfig
 * Description   : Applies the provided run mode configuration.
 * This function applies the provided run mode configuration.
 *
 *END**************************************************************************/
void MC_ME_SetRunModeConfig(MC_ME_Type * const baseAddr,
                            const power_manager_user_config_t * config)
{
    uint32_t regValue = 0U;
    switch (config->powerMode)
    {
        case POWER_MANAGER_RESET:
            /* Do nothing */
            break;
        /* Test mode */
#if FEATURE_MC_ME_HAS_TEST_MODE
        case POWER_MANAGER_TEST:
            regValue = baseAddr->TEST_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_TEST_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_TEST_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_TEST_MC_PDO(config->outputPowerdown) | MC_ME_TEST_MC_PWRLVL(config->powerLevel);
            baseAddr->TEST_MC = regValue;
            break;
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
        /* Safe mode */
        case POWER_MANAGER_SAFE:
            regValue = baseAddr->SAFE_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_SAFE_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_SAFE_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_SAFE_MC_PDO(config->outputPowerdown) | MC_ME_SAFE_MC_PWRLVL(config->powerLevel);
            baseAddr->SAFE_MC = regValue;
            break;
        /* Drun mode */
        case POWER_MANAGER_DRUN:
            regValue = baseAddr->DRUN_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_DRUN_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_DRUN_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_DRUN_MC_PWRLVL(config->powerLevel);
            baseAddr->DRUN_MC = regValue;
            break;
        /* Run 0 mode */
        case POWER_MANAGER_RUN0:
            regValue = baseAddr->RUN0_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_RUN0_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_RUN0_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_RUN0_MC_PWRLVL(config->powerLevel);
            baseAddr->RUN0_MC = regValue;
            break;
        /* Run 1 mode */
        case POWER_MANAGER_RUN1:
            regValue = baseAddr->RUN1_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_RUN1_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_RUN1_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_RUN1_MC_PWRLVL(config->powerLevel);
            baseAddr->RUN1_MC = regValue;
            break;
        /* Run 2 mode */
        case POWER_MANAGER_RUN2:
            regValue = baseAddr->RUN2_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_RUN2_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_RUN2_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_RUN2_MC_PWRLVL(config->powerLevel);
            baseAddr->RUN2_MC = regValue;
            break;
        /* Run 3 mode */
        case POWER_MANAGER_RUN3:
            regValue = baseAddr->RUN3_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK | MC_ME_RUN3_MC_PWRLVL_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_RUN3_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_RUN3_MC_PWRLVL(config->powerLevel);
            baseAddr->RUN3_MC = regValue;
            break;
        /* Halt mode */
#if FEATURE_MC_ME_HAS_HALT_MODE
        case POWER_MANAGER_HALT0:
            regValue = baseAddr->HALT0_MC;
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_HALT0_MC_FLAON(config->flashMode);
#endif
            baseAddr->HALT0_MC = regValue;
            break;
#endif /* FEATURE_MC_ME_HAS_HALT_MODE */
        /* Stop mode */
        case POWER_MANAGER_STOP0:
            /* control register STOP0 with CPU has STOP0 mode not STOP mode */
#if FEATURE_MC_ME_HAS_STOP0_MODE
            regValue = baseAddr->STOP0_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_STOP0_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_STOP0_MC_MVRON(config->mainVoltage) | MC_ME_STOP0_MC_PDO(config->outputPowerdown);
            baseAddr->STOP0_MC = regValue;
#else
            regValue = baseAddr->STOP_MC;
            regValue &= ~(MC_ME_GS_S_PDO_MASK | MC_ME_GS_S_MVR_MASK);
#if FEATURE_MC_ME_HAS_FLAON_CONFIG
            regValue &= ~MC_ME_GS_S_FLA_MASK;
            regValue |= MC_ME_STOP_MC_FLAON(config->flashMode);
#endif
            regValue |= MC_ME_STOP_MC_MVRON(config->mainVoltage) | MC_ME_STOP_MC_PDO(config->outputPowerdown);
            baseAddr->STOP_MC = regValue;
#endif /* #if FEATURE_MC_ME_HAS_STOP0_MODE */
            break;
        /* Standby mode */
#if FEATURE_MC_ME_HAS_STANDBY_MODE
        case POWER_MANAGER_STANDBY0:
            /* Do nothing */
            break;
#endif /* #if FEATURE_MC_ME_HAS_STANDBY_MODE */
        default:
            DEV_ASSERT(0);
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_CheckSystemClkInProgress
 * Description   : This function read the PCS_PROG which is used check 
 * the clock switching.
 *
 *END**************************************************************************/
bool MC_ME_CheckSystemClkInProgress(const MC_ME_Type * const baseAddr)
{
    uint32_t regValue = baseAddr->DMTS;
    regValue = (regValue & MC_ME_DMTS_PCS_PROG_MASK) >> MC_ME_DMTS_PCS_PROG_SHIFT;

    return (regValue != 0U) ? true : false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_GetPreviousMode
 * Description   : This function read mode which the chip was jump
 * from lastest mode to the current mode.
 *
 *END**************************************************************************/
void MC_ME_GetPreviousMode(const MC_ME_Type * const baseAddr,
                           power_manager_modes_t * mode)
{
    uint32_t regValue = baseAddr->DMTS;
    regValue = (regValue & MC_ME_DMTS_PREVIOUS_MODE_MASK) >> MC_ME_DMTS_PREVIOUS_MODE_SHIFT;
    switch (regValue)
    {
        case 0:
            *mode = POWER_MANAGER_RESET;
            break;
#if FEATURE_MC_ME_HAS_TEST_MODE
        case 1:
            *mode = POWER_MANAGER_TEST;
            break;
#endif
        case 2:
            *mode = POWER_MANAGER_SAFE;
            break;
        case 3:
            *mode = POWER_MANAGER_DRUN;
            break;
        case 4:
            *mode = POWER_MANAGER_RUN0;
            break;
        case 5:
            *mode = POWER_MANAGER_RUN1;
            break;
        case 6:
            *mode = POWER_MANAGER_RUN2;
            break;
        case 7:
            *mode = POWER_MANAGER_RUN3;
            break;
#if FEATURE_MC_ME_HAS_HALT_MODE
        case 8:
            *mode = POWER_MANAGER_HALT0;
            break;
#endif
        case 10:
            *mode = POWER_MANAGER_STOP0;
            break;
#if FEATURE_MC_ME_HAS_STANDBY_MODE
        case 13:
            *mode = POWER_MANAGER_STANDBY0;
            break;
#endif
        default:
            DEV_ASSERT(0);
            break;
    }
}

#if FEATURE_HAS_SDPLL_CLK_CONFIG
/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_GetSdpllEnable
 * Description   : This function read bit SDPLL was enabled or disabled in
 * another mode
 *
 *END**************************************************************************/
bool MC_ME_GetSdpllEnable(const MC_ME_Type * const baseAddr,
                          power_manager_modes_t mode)
{
    uint32_t regValue = 0U;

    switch (mode)
    {
        case POWER_MANAGER_DRUN:
            regValue = baseAddr->DRUN_MC & MC_ME_DRUN_MC_SDPLLON_MASK;
            break;
        case POWER_MANAGER_RUN0:
            regValue =  baseAddr->RUN0_MC & MC_ME_RUN0_MC_SDPLLON_MASK;
            break;
        case POWER_MANAGER_RUN1:
            regValue =  baseAddr->RUN1_MC & MC_ME_RUN1_MC_SDPLLON_MASK;
            break;
        case POWER_MANAGER_RUN2:
            regValue =  baseAddr->RUN2_MC & MC_ME_RUN2_MC_SDPLLON_MASK;
            break;
        case POWER_MANAGER_RUN3:
            regValue =  baseAddr->RUN3_MC & MC_ME_RUN3_MC_SDPLLON_MASK;
            break;
#if FEATURE_MC_ME_HAS_TEST_MODE
        case POWER_MANAGER_TEST:
            regValue =  baseAddr->TEST_MC & MC_ME_TEST_MC_SDPLLON_MASK;
            break;
#endif
        default:
            regValue =  0U;
            break;
    }

    return (regValue != 0U) ? true : false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MC_ME_EnableSdpllRun
 * Description   : This function set or clear bit SDPLL in Runx mode.
 *
 *END**************************************************************************/
void MC_ME_EnableSdpllRun(MC_ME_Type * const baseAddr,
                          power_manager_modes_t mode,
                          bool enable)
{
    uint32_t regValue = 0u;
    switch(mode)
    {
        case POWER_MANAGER_RUN0:
            regValue = baseAddr->RUN0_MC;
            regValue &= ~MC_ME_RUN0_MC_SDPLLON_MASK;
            regValue |= MC_ME_RUN0_MC_SDPLLON((enable == true) ? 1UL : 0UL);
            baseAddr->RUN0_MC = regValue;
            break;
        case POWER_MANAGER_RUN1:
            regValue = baseAddr->RUN1_MC;
            regValue &= ~MC_ME_RUN1_MC_SDPLLON_MASK;
            regValue |= MC_ME_RUN1_MC_SDPLLON((enable == true) ? 1UL : 0UL);
            baseAddr->RUN1_MC = regValue;
            break;
        case POWER_MANAGER_RUN2:
            regValue = baseAddr->RUN2_MC;
            regValue &= ~MC_ME_RUN2_MC_SDPLLON_MASK;
            regValue |= MC_ME_RUN2_MC_SDPLLON((enable == true) ? 1UL : 0UL);
            baseAddr->RUN2_MC = regValue;
            break;
        case POWER_MANAGER_RUN3:
            regValue = baseAddr->RUN3_MC;
            regValue &= ~MC_ME_RUN3_MC_SDPLLON_MASK;
            regValue |= MC_ME_RUN3_MC_SDPLLON((enable == true) ? 1UL : 0UL);
            baseAddr->RUN3_MC = regValue;
            break;
        default:
            /* Do nothing */
            (void) regValue;
            break;
    }
}
#endif /* #if FEATURE_HAS_SDPLL_CLK_CONFIG */

/*******************************************************************************
 * EOF
 ******************************************************************************/
