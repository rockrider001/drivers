/*
 * Copyright 2017-2018 NXP.
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
 * @file swt_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
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
 */

#include <stddef.h>
#include "swt_hw_access.h"
#include "interrupt_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for SWT instances. */
static SWT_Type * const s_swtBase[SWT_INSTANCE_COUNT] = SWT_BASE_PTRS;

#if !defined(FEATURE_SWT_IRQ_TIE_NMI_IRQ)
/*! @brief Table to save SWT IRQ enum numbers defined in CMSIS header file. */
static const IRQn_Type s_swtIrqId[SWT_INSTANCE_COUNT] = SWT_IRQS;
#endif

/*******************************************************************************
 * Code
 *******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_LockConfig
 * Description   : Lock SWT registers.
 * When locked, the SWT_CR, SWT_TO, SWT_WN and SWT_SK registers are read-only.
 *
 * Implements    : SWT_DRV_LockConfig_Activity
 *END**************************************************************************/
status_t SWT_DRV_LockConfig(uint32_t instance,
                            swt_lock_t lockConfig)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        /* Configures lock bits */
        SWT_Lock(base, lockConfig);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_Init
 * Description   : Initializes the SWT driver by user configuration and start timer.
 * Ensure that the SIRC clock gate is enabled.
 *
 * Implements    : SWT_DRV_Init_Activity
 *END**************************************************************************/
status_t SWT_DRV_Init(uint32_t instance,
                      const swt_user_config_t * const userConfigPtr)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);

    SWT_Type * const base = s_swtBase[instance];
    status_t ret = STATUS_SUCCESS;

    if (SWT_IsEnable(base))
    {
        ret = STATUS_ERROR;
    }
    else
    {
        /* Configures the SWT instance */
        ret = SWT_Config(base, userConfigPtr);

        if ((ret == STATUS_SUCCESS) && userConfigPtr->intEnable)
        {
#if !defined(FEATURE_SWT_IRQ_TIE_NMI_IRQ)
            /* Enables SWT interrupt vector */
            INT_SYS_EnableIRQ(s_swtIrqId[instance]);
#endif
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_GetDefaultConfig
 * Description   : Gets the default configuration of the SWT.
 *
 * Implements    : SWT_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void SWT_DRV_GetDefaultConfig(swt_user_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    /* All masters can access */
    config->mapConfig = 0xFFU;
    /* Do not create reset request on invalid access */
    config->invalidReset = false;
    /* Disable window mode */
    config->winEnable = false;
    /* Disable interrupt */
    config->intEnable = false;
#if FEATURE_SWT_HAS_CLOCK_SELECT
    /* The SWT is clocked by internal oscillator */
    config->clockSelect = true;
#endif
#if FEATURE_SWT_HAS_STOP_MODE
    /* The SWT counter continues to run in stop mode */
    config->stop = false;
#endif
    /* The SWT counter continues to run in debug mode */
    config->debug = false;
    /* Fixed service sequence mode */
    config->serviceMode = SWT_FS_SEQ_MODE;
    /* Unlock */
    config->lockConfig = SWT_UNLOCK;

    /* Gets default timeout value */
    config->timeoutValue = FEATURE_SWT_TO_RESET_VALUE;

    /* Gets default window value */
    config->windowValue = 0U;

    /* Gets default initial service key value */
    config->initKey = 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_Deinit
 * Description   : Resets all configuration to default and disable the SWT instance.
 *
 * Implements    : SWT_DRV_Deinit_Activity
 *END**************************************************************************/
status_t SWT_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        /* Disable SWT timer */
        SWT_Disable(base);

        /* Reset SWT configuration to default */
        SWT_Reset(base);
#if !defined(FEATURE_SWT_IRQ_TIE_NMI_IRQ)
        /* Disables SWT interrupt vector */
        INT_SYS_DisableIRQ(s_swtIrqId[instance]);
#endif
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_SetServiceConfig
 * Description   : Sets service mode and sets initial service key if in Keyed Service Mode.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_SetServiceConfig_Activity
 *END**************************************************************************/
status_t SWT_DRV_SetServiceConfig(uint32_t instance,
                                  swt_service_mode_t mode,
                                  uint16_t serviceKey)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        /* Sets service mode */
        SWT_SetServiceMode(base, mode);

        /* Sets initial service key */
        if (mode == SWT_KS_SEQ_MODE)
        {
            SWT_SetServiceKey(base, serviceKey);
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_Service
 * Description   : Resets the SWT counter in fixed service sequence and keyed service
 * sequence modes.
 * - Fixed Service Sequence Mode: write fixed keys.
 * - Keyed Service Sequence Mode: write pseudo random keys (17*SK+3) mod 2^16.
 *
 * Implements    : SWT_DRV_Service_Activity
 *END**************************************************************************/
void SWT_DRV_Service(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];

    switch (SWT_GetServiceMode(base))
    {
    /* Fixed Service Sequence Mode */
    case SWT_FS_SEQ_MODE:
        SWT_SetServiceCmd(base, FEATURE_SWT_FIXED_SERVICE_VALUE1);
        SWT_SetServiceCmd(base, FEATURE_SWT_FIXED_SERVICE_VALUE2);
        break;
    /* Keyed Service Sequence Mode */
    case SWT_KS_SEQ_MODE:
        SWT_SetServiceCmd(base, SWT_ServiceKeyGen(base));
        SWT_SetServiceCmd(base, SWT_ServiceKeyGen(base));
        break;
    /* Fixed Address Execution Mode / Incremental Address Execution Mode */
    default:
        /* Executes code at the address loaded into the designated IAC register */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_SetIntConfig
 * Description   : Enables/Disables the SWT timeout interrupt.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_SetIntConfig_Activity
 *END**************************************************************************/
status_t SWT_DRV_SetIntConfig(uint32_t instance,
                              bool enable)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
#if !defined(FEATURE_SWT_IRQ_TIE_NMI_IRQ)
        /* Enables/Disables SWT interrupt vector */
        if (enable)
        {
            INT_SYS_EnableIRQ(s_swtIrqId[instance]);
        }
        else
        {
            INT_SYS_DisableIRQ(s_swtIrqId[instance]);
        }
#endif
        /* Enables/Disables SWT interrupt */
        SWT_EnableInt(base, enable);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_ClearIntFlag
 * Description   : Clears the Timeout Interrupt Flag.
 *
 * Implements    : SWT_DRV_ClearIntFlag_Activity
 *END**************************************************************************/
void SWT_DRV_ClearIntFlag(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];

    /* Clears interrupt flag */
    (void)SWT_WriteReadIntReg(base, SWT_IR_TIF_MASK, SWT_IR_TIF_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_GetIntStatus
 * Description   : Gets the Timeout Interrupt Status.
 *
 * Implements    : SWT_DRV_GetIntStatus_Activity
 *END**************************************************************************/
bool SWT_DRV_GetIntStatus(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];

    /* Gets interrupt status */
    return ((SWT_WriteReadIntReg(base, SWT_IR_TIF_MASK, 0U) & SWT_IR_TIF_MASK) != 0U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_SetTimeoutValue
 * Description   : Sets timeout value.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_SetTimeoutValue_Activity
 *END**************************************************************************/
status_t SWT_DRV_SetTimeoutValue(uint32_t instance,
                                 uint32_t value)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    status_t ret = STATUS_ERROR;

    /* Checks timeout value */
    if (value >= FEATURE_SWT_TO_MINIMUM_VALUE)
    {
        /* Unlocks SWT instance */
        ret = SWT_UnLock(base);

        if (ret == STATUS_SUCCESS)
        {
            /* Sets timeout value */
            SWT_SetTimeoutValue(base, value);
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_SetWindowConfig
 * Description   : Enables/Disables window mode and sets window value if enabled.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_SetWindowConfig_Activity
 *END**************************************************************************/
status_t SWT_DRV_SetWindowConfig(uint32_t instance,
                                 bool enable,
                                 uint32_t value)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        /* Sets window value */
        if (enable)
        {
            SWT_SetWindowValue(base, value);
        }

        /* Enables/Disables window mode */
        SWT_EnableWindow(base, enable);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_GetCounterValue
 * Description   : Gets current counter value.
 * When the watchdog is disabled (SWT_CR[WEN] is 0), this field shows the value of the internal down
 * counter. When the watchdog is enabled (SWT_CR[WEN] is 1), this field is cleared (the value is
 * 0x0000_0000). Values in this field can lag behind the internal counter value for up to 6 system clock
 * cycles plus 8 counter clock cycles. Therefore, the value read from this field immediately after
 * disabling the watchdog may be higher than the actual value of the internal counter.
 * The SWT timer should be disabled before calling this function.
 *
 * Implements    : SWT_DRV_GetCounterValue_Activity
 *END**************************************************************************/
status_t SWT_DRV_GetCounterValue(uint32_t instance,
                                 uint32_t * value)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    const SWT_Type * const base = s_swtBase[instance];
    status_t ret = STATUS_SUCCESS;

    if (SWT_IsEnable(base))
    {
        ret = STATUS_ERROR;
    }

    *value = SWT_GetCounter(base);

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_StartTimer
 * Description   : Enables the SWT timer.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_StartTimer_Activity
 *END**************************************************************************/
status_t SWT_DRV_StartTimer(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        SWT_Enable(base);
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_DRV_StopTimer
 * Description   : Disables the SWT timer.
 * This function will unlock the SWT configuration.
 *
 * Implements    : SWT_DRV_StopTimer_Activity
 *END**************************************************************************/
status_t SWT_DRV_StopTimer(uint32_t instance)
{
    DEV_ASSERT(instance < SWT_INSTANCE_COUNT);

    SWT_Type * const base = s_swtBase[instance];
    /* Unlocks SWT instance */
    status_t ret = SWT_UnLock(base);

    if (ret == STATUS_SUCCESS)
    {
        SWT_Disable(base);
    }

    return ret;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
