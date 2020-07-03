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
 * @file power_manager_MPC57xx.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, There shall be no occurrence of
 * undefined or critical unspecified behaviour.
 * The addresses of the stack variables are only used at local scope.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5,
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>'.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3,
 * Expression assigned to a narrower or different essential type.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A cast shall not be performed
 * between pointer to void and an arithmetic type.
 * The base address parameter from HAL functions is provided as integer so
 * it needs to be cast to pointer.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be performed
 * between a pointer to object and an integer type.
 * The base address parameter from HAL functions is provided as integer so
 * a conversion between a pointer and an integer has to be performed
 */

#include "power_manager.h"
#include "power_mc_me_hw_access.h"
#include <stdbool.h>

/*! Timeout used for waiting to set new mode */
#define POWER_SET_MODE_TIMEOUT 200000U
/*! Timeout used for waiting to set new mode */
#define POWER_SET_PREVIOUS_MODE_TIMEOUT 250000U

/*! @brief Power manager internal structure. */
power_manager_state_t gPowerManagerState;

/*******************************************************************************
 * INTERNAL FUNCTIONS
 ******************************************************************************/
static status_t POWER_SYS_WaitForModeStatus(power_manager_modes_t mode);
static status_t POWER_SYS_WaitForPreModeStatus(power_manager_modes_t mode);
static status_t POWER_SYS_EnterStopMode(const power_manager_modes_t powerMode);

#if FEATURE_HAS_SDPLL_CLK_CONFIG
static void POWER_DRV_SwitchSDPLL(power_manager_modes_t nextMode, power_manager_modes_t curMode);
static void POWER_DRV_DisableSdPll(void);
static void POWER_DRV_EnableSdPll(void);
#endif
/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_DoInit
 * Description   : Implementation-specific configuration of power modes.
 *
 * This function performs the actual implementation-specific initialization based on the provided power mode configurations.
 *END**************************************************************************/
status_t POWER_SYS_DoInit(void)
{
    uint32_t i = 0U;

    /* Disable user modes first; they will be enabled as needed */
#if FEATURE_MC_ME_HAS_TEST_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_TEST);
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN1);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN2);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN3);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_STOP0);
#if FEATURE_MC_ME_HAS_HALT_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_HALT0);
#endif /* #if FEATURE_MC_ME_HAS_HALT_MODE */
#if FEATURE_MC_ME_HAS_STANDBY_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_STANDBY0);
#endif /* #if FEATURE_MC_ME_HAS_STANDBY_MODE */
    /* Configure run modes */
    for (i = 0; i < gPowerManagerState.configsNumber; i++)
    {
        const power_manager_user_config_t * const config = (*gPowerManagerState.configs)[i];
        MC_ME_SetRunModeConfig(MC_ME, config);
        MC_ME_EnablePowerMode(MC_ME, config->powerMode);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_DoDeinit
 * Description   : Implementation-specific de-initialization of power manager.
 *
 * This function performs the actual implementation-specific de-initialization.
 *END**************************************************************************/
status_t POWER_SYS_DoDeinit(void)
{
#if FEATURE_MC_ME_HAS_TEST_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_TEST);
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN1);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN2);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_RUN3);
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_STOP0);
#if FEATURE_MC_ME_HAS_HALT_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_HALT0);
#endif /* #if FEATURE_MC_ME_HAS_HALT_MODE */
#if FEATURE_MC_ME_HAS_STANDBY_MODE
    MC_ME_DisablePowerMode(MC_ME, POWER_MANAGER_STANDBY0);
#endif /* #if FEATURE_MC_ME_HAS_STANDBY_MODE */

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_EnterStopMode
 * Description   : Enter STOP mode.
 *
 * This function will switch Spll clock when CPU enters stop mode.
 *END**************************************************************************/
static status_t POWER_SYS_EnterStopMode(const power_manager_modes_t powerMode)
{
    status_t returnCode = STATUS_SUCCESS;
#ifdef ERRATA_E10365
    status_t tmpReturnCode = STATUS_SUCCESS;
    power_manager_modes_t currentMode = POWER_SYS_GetCurrentMode();
    bool sdpllMode = MC_ME_GetSdpllEnable(MC_ME,currentMode);

    if (sdpllMode == true)
    {
        MC_ME_EnableSdpllRun(MC_ME,currentMode,false);
        MC_ME_SetPowerMode(MC_ME, currentMode);
        POWER_DRV_DisableSdPll();
        returnCode = POWER_SYS_WaitForModeStatus(currentMode);
        if (STATUS_SUCCESS == returnCode)
        {
            /* Switch the mode
            * After switch this mode the core stop and wait interrupt or wakeup */
            MC_ME_SetPowerMode(MC_ME, powerMode);
            /* Check previous which was STOP0 or HALT mode */
            returnCode = POWER_SYS_WaitForPreModeStatus(powerMode);
            /* Enable SDPLL in the RUNx mode */
            MC_ME_EnableSdpllRun(MC_ME,currentMode,true);
            MC_ME_SetPowerMode(MC_ME, currentMode);
            /* Enabling SDPLL on system */
            POWER_DRV_EnableSdPll();
            tmpReturnCode = POWER_SYS_WaitForModeStatus(currentMode);
            if (returnCode == STATUS_SUCCESS)
            {
                returnCode = tmpReturnCode;
            }
        }
    }
    else
    {
        /* Switch the mode
        * After switch this mode the core stop and wait interrupt or wakeup */
        MC_ME_SetPowerMode(MC_ME, powerMode);
        returnCode = POWER_SYS_WaitForPreModeStatus(powerMode);
    }

#else
    /* Switch the mode
    * After switch this mode the core stop and wait interrupt or wakeup */
    MC_ME_SetPowerMode(MC_ME, powerMode);
    returnCode = POWER_SYS_WaitForPreModeStatus(powerMode);
#endif /* #ifdef ERRATA_E10365 */

    return returnCode;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_DoSetMode
 * Description   : Configures the power mode.
 *
 * This function performs the actual implementation-specific logic to switch to one of the defined power modes.
 *END**************************************************************************/
status_t POWER_SYS_DoSetMode(const power_manager_user_config_t * const configPtr)
{
    status_t returnCode = STATUS_SUCCESS;
    power_manager_modes_t currentMode = POWER_SYS_GetCurrentMode();

    /* Configure the power mode */
    switch (configPtr->powerMode)
    {
        /* Reset mode */
        case POWER_MANAGER_RESET:
            MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_RESET);
            break;
        /* Test mode */
#if FEATURE_MC_ME_HAS_TEST_MODE
        case POWER_MANAGER_TEST:
            /* Test mode can be entered from DRUN mode */
            if ((currentMode != POWER_MANAGER_DRUN) && (currentMode != POWER_MANAGER_TEST))
            {
                /* Switch the mode */
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_DRUN);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                POWER_DRV_SwitchSDPLL(POWER_MANAGER_DRUN, currentMode);
#endif
                returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_DRUN);
            }
            if (returnCode == STATUS_SUCCESS)
            {
                /* Switch the mode */
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_TEST);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                POWER_DRV_SwitchSDPLL(POWER_MANAGER_TEST, POWER_SYS_GetCurrentMode());
#endif
                returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_TEST);
            }
            break;
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
        /* Safe mode */
        case POWER_MANAGER_SAFE:
        /* Safe mode can be entered from DRUN and RUNx mode */
            MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_SAFE);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
            POWER_DRV_SwitchSDPLL(POWER_MANAGER_SAFE, currentMode);
#endif
            returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_SAFE);
            break;
        /* DRUN mode */
        case POWER_MANAGER_DRUN:
            /* Switch the mode */
            MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_DRUN);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
            POWER_DRV_SwitchSDPLL(POWER_MANAGER_DRUN, currentMode);
#endif
            returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_DRUN);
            break;
        /* RUN mode */
        /* Fall-through */
        case POWER_MANAGER_RUN0:
        /* Fall-through */
        case POWER_MANAGER_RUN1:
        /* Fall-through */
        case POWER_MANAGER_RUN2:
        /* Fall-through */
        case POWER_MANAGER_RUN3:
            /* RUNx modes can be entered from DRUN,SAFE and other RUNx modes */
            if (currentMode <= POWER_MANAGER_SAFE)
            {
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_DRUN);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                POWER_DRV_SwitchSDPLL(POWER_MANAGER_DRUN, currentMode);
#endif
                returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_DRUN);
            }
            if (STATUS_SUCCESS == returnCode)
            {
                /* Switch the run mode */
                MC_ME_SetPowerMode(MC_ME, configPtr->powerMode);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                POWER_DRV_SwitchSDPLL(configPtr->powerMode, POWER_SYS_GetCurrentMode());
#endif /* #if FEATURE_HAS_SDPLL_CLK_CONFIG */
                returnCode = POWER_SYS_WaitForModeStatus(configPtr->powerMode);
            }
            break;
#if FEATURE_MC_ME_HAS_HALT_MODE
        case POWER_MANAGER_HALT0:
        /* Fall-through */
#endif
        case POWER_MANAGER_STOP0:
            /* HALT0 and STOP0 mode can be entered only from RUN mode */
            /* Must switch to drun mode if current mode is test or safe mode */
            if (currentMode <= POWER_MANAGER_SAFE)
            {
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_DRUN);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                POWER_DRV_SwitchSDPLL(POWER_MANAGER_DRUN, currentMode);
#endif
                returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_DRUN);
            }
            if (returnCode == STATUS_SUCCESS)
            {
                if ((currentMode < POWER_MANAGER_RUN0) || (currentMode > POWER_MANAGER_RUN3))
                {
                    MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_RUN0);
#if FEATURE_HAS_SDPLL_CLK_CONFIG
                    POWER_DRV_SwitchSDPLL(POWER_MANAGER_RUN0, POWER_SYS_GetCurrentMode());
#endif
                    returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_RUN0);
                }
                if (STATUS_SUCCESS == returnCode)
                {
                    returnCode = POWER_SYS_EnterStopMode(configPtr->powerMode);
                }
            }
            break;
#if FEATURE_MC_ME_HAS_STANDBY_MODE
        case POWER_MANAGER_STANDBY0:
            /* STANDBY0 mode can be entered from RUN and DRUN mode */
            if ((currentMode < POWER_MANAGER_DRUN) || (currentMode > POWER_MANAGER_RUN3))
            {
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_DRUN);
                returnCode = POWER_SYS_WaitForModeStatus(POWER_MANAGER_DRUN);
            }
            if (STATUS_SUCCESS == returnCode)
            {
                /* Switch the STANDBY mode.
                 * After switch this mode the core stop and just reset or wake up it */
                MC_ME_SetPowerMode(MC_ME, POWER_MANAGER_STANDBY0);
            }
            break;
#endif /* #if FEATURE_MC_ME_HAS_STANDBY_MODE */
        default:
            /* invalid power mode */
            returnCode = STATUS_UNSUPPORTED;
            break;
    }

    return returnCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetCurrentMode
 * Description   : Returns currently running power mode.
 *
 * Implements POWER_SYS_GetCurrentMode_Activity
 *END**************************************************************************/
power_manager_modes_t POWER_SYS_GetCurrentMode(void)
{
    power_manager_user_config_t status;
    MC_ME_GetStatus(MC_ME, &status);

    return status.powerMode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_WaitForModeStatus
 * Description   : Internal function used for waiting for mode change to finish
 * mode            The expected running mode
 *
 *END**************************************************************************/
static status_t POWER_SYS_WaitForModeStatus(power_manager_modes_t mode)
{
    status_t retCode = STATUS_MCU_TRANSITION_FAILED;
    power_manager_modes_t currentMode = POWER_MANAGER_DRUN;
    uint32_t i = 0U;
    bool pcsWait = false;

    switch (mode)
    {
#if FEATURE_MC_ME_HAS_TEST_MODE
        case POWER_MANAGER_TEST:
        /* Fall-through */
#endif /* #if FEATURE_MC_ME_HAS_TEST_MODE */
        case POWER_MANAGER_SAFE:
        /* Fall-through */
        case POWER_MANAGER_DRUN:
        /* Fall-through */
        case POWER_MANAGER_RUN0:
        /* Fall-through */
        case POWER_MANAGER_RUN1:
        /* Fall-through */
        case POWER_MANAGER_RUN2:
        /* Fall-through */
        case POWER_MANAGER_RUN3:
            retCode = STATUS_SUCCESS;
            break;
        default:
            retCode = STATUS_UNSUPPORTED;
            break;
    }

    if (STATUS_SUCCESS == retCode)
    {
        for (i = 0U; i < POWER_SET_MODE_TIMEOUT; i++)
        {
            if (!MC_ME_ModeTransitionInProgress(MC_ME))
            {
                pcsWait = MC_ME_CheckSystemClkInProgress(MC_ME);
                currentMode = POWER_SYS_GetCurrentMode();
                if ((currentMode == mode) && (pcsWait == false))
                {
                    break;
                }
            }
        }
    }

    if (i >= POWER_SET_MODE_TIMEOUT)
    {
        retCode = STATUS_MCU_TRANSITION_FAILED;
    }

    return retCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_SYS_GetPreviousMode
 * Description   : This function will return previous mode where the chip jump
 * from lastest mode to current mode. The function can not return STANDBY mode
 * because register control is disabled.
 *
 * Implements POWER_SYS_GetPreviousMode_Activity
 *END**************************************************************************/
power_manager_modes_t POWER_SYS_GetPreviousMode(void)
{
    power_manager_modes_t returnMode;

    MC_ME_GetPreviousMode(MC_ME,&returnMode);

    return returnMode;
}

/*FUNCTION******************************************************************************
 *
 * Function Name : POWER_SYS_WaitForPreModeStatus
 * Description   : Internal function used for waiting for previous mode update to finish
 *                 This function only uses to check previous mode when cpu exited
                   stop or halt mode.
 * mode            The expected checking mode
 *
 *END**********************************************************************************/
static status_t POWER_SYS_WaitForPreModeStatus(power_manager_modes_t mode)
{
    status_t retCode = STATUS_MCU_TRANSITION_FAILED;
    uint32_t i = 0U;
    power_manager_modes_t preMode = POWER_MANAGER_DRUN;
    bool pcsWait = false;

    switch (mode)
    {
        /* Fall-through */
#if FEATURE_MC_ME_HAS_HALT_MODE
        case POWER_MANAGER_HALT0:
        /* Fall-through */
#endif
        case POWER_MANAGER_STOP0:
            retCode = STATUS_SUCCESS;
            break;
        default:
            retCode = STATUS_UNSUPPORTED;
            break;
    }

    if (STATUS_SUCCESS == retCode)
    {
        for (i = 0U; i < POWER_SET_PREVIOUS_MODE_TIMEOUT; i++)
        {
            MC_ME_GetPreviousMode(MC_ME,&preMode);
            pcsWait = MC_ME_CheckSystemClkInProgress(MC_ME);
            if ((preMode == mode) && (pcsWait == false))
            {
                break;
            }
        }
    }

    if (i >= POWER_SET_PREVIOUS_MODE_TIMEOUT)
    {
        retCode = STATUS_MCU_TRANSITION_FAILED;
    }

    return retCode;
}

#if FEATURE_HAS_SDPLL_CLK_CONFIG
/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_DRV_SwitchSDPLL
 * Description   : This function will enable or disable SDPLL when switches another
 * mode. This one will be called while entering the one mode.
 *
 *END**************************************************************************/
static void POWER_DRV_SwitchSDPLL(power_manager_modes_t nextMode, power_manager_modes_t curMode)
{
    bool fistMode = MC_ME_GetSdpllEnable(MC_ME,curMode);
    bool secondMode = MC_ME_GetSdpllEnable(MC_ME, nextMode);

    if ((secondMode == false) && (fistMode == true))
    {
        POWER_DRV_DisableSdPll();
    }
    else if ((secondMode == true) && (fistMode == false))
    {
        POWER_DRV_EnableSdPll();
    }
    else
    {
        /* another case switch normal */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_DRV_DisableSdPll
 * Description   : This function will disable SDPLL to reset SDPLL lock
 *
 *END**************************************************************************/
static void POWER_DRV_DisableSdPll(void)
{
    SDPLL_PUT_IN_RESET();
    SDPLL_STOP_CALIBRATION();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POWER_DRV_EnableSdPll
 * Description   : This function will enable SDPLL to normal operation.
 *
 *END**************************************************************************/
static void POWER_DRV_EnableSdPll(void)
{
    SDPLL_GET_OUT_OF_RESET();
    SDPLL_START_CALIBRATION();
}
#endif /* #if FEATURE_HAS_SDPLL_CLK_CONFIG */

/*******************************************************************************
 * EOF
 ******************************************************************************/
