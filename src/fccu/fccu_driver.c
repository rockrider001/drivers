/*
 * Copyright 2017-2019 NXP.
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
 * @file fccu_driver.c
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
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * Variable faults it is used only in the scope of that function in which is declared.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 18.5, More than two pointer indirection levels.
 * It is required for storing multiple arrays of configurations arrays of pointers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.3, Cast performed between a pointer to object
 * type and a pointer to a different object type.
 * The cast is required to check if the object passed as parameter is defined and different
 * from a null pointer.
 */

#include <stddef.h>
#include "fccu_hw_access.h"
#include "fccu_driver.h"
#include "interrupt_manager.h"
#include "osif.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for fccu instances. */
static FCCU_Type * const s_fccuBase[FCCU_INSTANCE_COUNT] = FCCU_BASE_PTRS;
/* Configured Instance State Structure for Global FCCU IRQ */
static fccu_isr_state_t s_IrqStatus[FCCU_INSTANCE_COUNT];
/*  Table of address for instance NCF configuration structures */
static fccu_config_ncf_t const ** s_ncfConfigurations[FCCU_INSTANCE_COUNT];

/*******************************************************************************
* Code
*******************************************************************************/

/*!
 * @brief Set Semaphore Unlock State
 *
 * @param[in] instance      number of FCCU instance
 */
static inline void FCCU_DRV_SemaphoreUnlock(const uint32_t  instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    INT_SYS_DisableIRQGlobal();
    s_IrqStatus[instance].semaphore = FCCU_SEMAPHORE_UNLOCK;
    INT_SYS_EnableIRQGlobal();
}

/*!
 * @brief Set Semaphore Lock State
 *
 * @param[in] instance      number of FCCU instance
 */
static inline void FCCU_DRV_SemaphoreLock(const uint32_t  instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    INT_SYS_DisableIRQGlobal();
    s_IrqStatus[instance].semaphore = FCCU_SEMAPHORE_LOCK;
    INT_SYS_EnableIRQGlobal();
}

/*!
 * @brief Check Ncf Reaction Types
 *
 * @param[in] x     reaction storage.
 * @param[in] y     reaction type to compare.
 * @return      Return true if reaction match with reaction storage or
 *              false if not match with reaction storage.
 */
static inline bool FCCU_DRV_CheckNcfReaction(const uint8_t x,
                                             const fccu_int_status_t y)
{
    return ((x & (uint8_t)(1U << y)) == (uint8_t)(1U << y)) ? true : false;
}
/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_ClearFaults
 * Description   : Clear Fault Status Register.
 * Need to clear the fault register NCF_S by setting bit position corresponding with
 * fault faultIndex. Any value higher than last faultNo will clear all faults
 *
 * Implements    : FCCU_DRV_ClearFaults_Activity
 * END**************************************************************************/
status_t FCCU_DRV_ClearFaults(const uint32_t instance,
                              uint8_t faultIdx)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    uint8_t index = 0U;
    uint32_t msElapsed;
    status_t status = STATUS_SUCCESS;
    if (faultIdx >= FEATURE_FCCU_CLEAR_ALL_FAULTS)
    {
        /* Clear all faults if faultIndex is higher than the (Max No of faults + 1) */
        while (index < FCCU_NCF_S_COUNT)
        {
            FCCU_SetNCFKey(base, FEATURE_FCCU_NCF_KEY);
            /* Clear All Faults from all registers */
            base->NCF_S[index] = 0xFFFFFFFFU;
            /* Wait the issued automatically OP12 to finish successfully */
            msElapsed = OSIF_GetMilliseconds();
            do
            {
                if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
                {
                    status = STATUS_TIMEOUT;
                    break;
                }
            }
            while ((FCCU_GetOperationStatus(base) == FCCU_OP_INPROGRESS));
            index++;
        }
    }
    else
    {
        FCCU_SetNCFKey(base, FEATURE_FCCU_NCF_KEY);
        FCCU_ClearNCFsStatus(base, faultIdx);
        /* Wait the issued automatically OP12 to finish successfully */
        msElapsed = OSIF_GetMilliseconds();
        do
        {
            if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
            {
                status = STATUS_TIMEOUT;
                break;
            }
        }
        while (FCCU_GetOperationStatus(base) == FCCU_OP_INPROGRESS);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_RunOperation
 * Description   : Run Operation issued by user.
 *
 *
 * Implements    : FCCU_DRV_RunOperation_Activity
 * END**************************************************************************/
void FCCU_DRV_RunOperation(const uint32_t instance,
                           const fccu_run_op_t operation)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    register volatile uint32_t temp;
    uint32_t msElapsed = OSIF_GetMilliseconds();

    /* Wait to finish previous operation */
    while (FCCU_OP_INPROGRESS == FCCU_GetOperationStatus(base))
    {
        if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
        {
            DEV_ASSERT(false);
            break;
        }
    }
    temp = base->CTRL;
    switch (operation)
    {
        /* This functions are operation sensitive after unlock next instruction need to be write FCCU CTRL register */
        case FCCU_RUN_OP1:
            temp |= FCCU_CTRL_OPR(FCCU_RUN_OP1);
            FCCU_SetControlKey(base, FEATURE_FCCU_UNLOCK_OP1);
            base->CTRL = temp;
            break;
        case FCCU_RUN_OP2:
            temp |= FCCU_CTRL_OPR(FCCU_RUN_OP2);
            FCCU_SetControlKey(base, FEATURE_FCCU_UNLOCK_OP2);
            base->CTRL = temp;
            break;
        case FCCU_RUN_OP31:
            temp |= FCCU_CTRL_OPR(FCCU_RUN_OP31);
            FCCU_SetControlKey(base, FEATURE_FCCU_UNLOCK_OP31);
            base->CTRL = temp;
            break;
        default:
            FCCU_SetOperationRun(base, operation);
            break;
    } /* switch end */
    msElapsed = OSIF_GetMilliseconds();
    while (FCCU_OP_INPROGRESS == FCCU_GetOperationStatus(base))
    {
        if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
        {
            DEV_ASSERT(false);
            break;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_ReadFaults
 * Description   : Update and Read NonCritical Faults Triggered.
 *
 * Implements    : FCCU_DRV_ReadFaults_Activity
 * END**************************************************************************/
void FCCU_DRV_ReadFaults(const uint32_t instance,
                         fccu_faults_flags_t *  const param)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    const FCCU_Type * const base = s_fccuBase[instance];
    uint8_t index = 0;
    /* Update Status Fault Register with OP10 command */
    FCCU_DRV_RunOperation(instance, FCCU_RUN_OP10);
    while (index < FCCU_NCF_S_COUNT)
    {
        param->faultsFlags[index] = base->NCF_S[index];
        index++;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_GetConfigState
 * Description   : Update and Read FCCU Status.
 *
 * Implements    : FCCU_DRV_GetConfigState_Activity
 * END**************************************************************************/
fccu_status_t FCCU_DRV_GetConfigState(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    const FCCU_Type * const base = s_fccuBase[instance];
    /* This Operation will update the last state of FCCU in Status register */
    FCCU_DRV_RunOperation(instance, FCCU_RUN_OP3);
    return FCCU_GetStatus(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_SetConfigState
 * Description   : Set FCCU in Config mode and if errors clear all them.
 *
 * Implements    : FCCU_DRV_SetConfigState_Activity
 * END**************************************************************************/
status_t FCCU_DRV_SetConfigState(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_OTHER;
    status_t clearStatus = STATUS_FCCU_ERROR_OTHER;
    fccu_status_t fccuState;
    fccu_faults_flags_t faults;
    uint32_t faultsCheck = 0U;
    uint32_t msElapsed;
    uint8_t idx;
    /* Unlock FCCU to permit enter in ConfigState*/
    FCCU_SetTransientConfigLock(base, false);
    fccuState = FCCU_DRV_GetConfigState(instance);

    switch (fccuState)
    {
        case FCCU_STATUS_CONFIGURATION:
            /* Update status */
            status = STATUS_SUCCESS;
            break;
        case FCCU_STATUS_NORMAL:
            /* Clear all pending faults */
            clearStatus = FCCU_DRV_ClearFaults(instance, FEATURE_FCCU_CLEAR_ALL_FAULTS);
            if (clearStatus == STATUS_SUCCESS)
            {
                /* Get FCCU operation status */
                if (FCCU_OP_SUCCESSFUL != FCCU_GetOperationStatus(base))
                {
                    /* Update FCCU_STAT Register */
                    FCCU_DRV_RunOperation(instance, FCCU_RUN_OP3);
                }
                /* Run OP1 to put FCCU enter configuration mode */
                FCCU_DRV_RunOperation(instance, FCCU_RUN_OP1);
            }
            /* Read FCCU configure state again */
            if (FCCU_STATUS_CONFIGURATION == FCCU_DRV_GetConfigState(instance))
            {
                status = STATUS_SUCCESS;
            }
            else
            {
                status = STATUS_FCCU_ERROR_SET_CONFIG;
            }
            break;
        default : /* Case Alarm or Fault */
            /* Clear Errors in case if the FCCU before put FCCU enter configuration mode */
            msElapsed = OSIF_GetMilliseconds();
            do
            {
                clearStatus = FCCU_DRV_ClearFaults(instance, FEATURE_FCCU_CLEAR_ALL_FAULTS);
                if (STATUS_SUCCESS == clearStatus)
                {
                    FCCU_DRV_ReadFaults(instance, &faults);
                    if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
                    {
                        status = STATUS_TIMEOUT;
                        break;
                    }
                    for (idx = 0U; idx < FCCU_NCF_S_COUNT; idx++)
                    {
                        faultsCheck += faults.faultsFlags[idx];
                    }
                }
                else
                {
                    status = STATUS_TIMEOUT;
                }
            }
            while (faultsCheck != 0UL);

            /* Get FCCU operation status */
            if (FCCU_OP_SUCCESSFUL != FCCU_GetOperationStatus(base))
            {
                /* Update FCCU_STAT Register */
                FCCU_DRV_RunOperation(instance, FCCU_RUN_OP3);
            }
            /* Force FCCU in Normal Configuration State */
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP2);
            if (FCCU_STATUS_NORMAL != FCCU_DRV_GetConfigState(instance))
            {
                if (status != STATUS_TIMEOUT)
                {
                    status = STATUS_FCCU_ERROR_SET_CONFIG;
                }
            }
            else
            {
                FCCU_DRV_RunOperation(instance, FCCU_RUN_OP1);
                /* Read FCCU configure state again */
                if (FCCU_STATUS_CONFIGURATION == FCCU_DRV_GetConfigState(instance))
                {
                    status = STATUS_SUCCESS;
                }
                else
                {
                    status = STATUS_FCCU_ERROR_SET_CONFIG;
                }
            }
            break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_SetNormalState
 * Description   : Set FCCU to normal State
 * If errors occurred then clear them and try again to put in normal state.
 *
 * Implements    : FCCU_DRV_SetNormalState_Activity
 * END**************************************************************************/
status_t FCCU_DRV_SetNormalState(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);

    fccu_status_t fccuState;
    fccu_faults_flags_t faults;
    status_t status = STATUS_FCCU_ERROR_OTHER;
    uint32_t msElapsed;
    uint32_t faultsCheck = 0U;
    uint8_t idx;
    fccuState = FCCU_DRV_GetConfigState(instance);

    switch (fccuState)
    {
        case FCCU_STATUS_NORMAL:
            /* Update FCCU status */
            status = STATUS_SUCCESS;
            break;
        case FCCU_STATUS_CONFIGURATION:
            /* Put FCCU in Normal state */
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP2);
            /* Read FCCU configures state again */
            if (FCCU_STATUS_NORMAL == FCCU_DRV_GetConfigState(instance))
            {
                status = STATUS_SUCCESS;
            }
            else
            {
                status = STATUS_FCCU_ERROR_SET_NORMAL;
            }
            break;
        case FCCU_STATUS_ALARM:
        case FCCU_STATUS_FAULT:
        default:
            /* fall-through */
            /* Clear Errors in case if the FCCU is in Fault or Alarm State */
            msElapsed = OSIF_GetMilliseconds();
            do
            {
                if (STATUS_SUCCESS == FCCU_DRV_ClearFaults(instance, FEATURE_FCCU_CLEAR_ALL_FAULTS))
                {
                    FCCU_DRV_ReadFaults(instance, &faults);
                    if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
                    {
                        status = STATUS_TIMEOUT;
                        break;
                    }
                    for (idx = 0U; idx < FCCU_NCF_S_COUNT; idx++)
                    {
                        faultsCheck += faults.faultsFlags[idx];
                    }
                }
                else
                {
                    status = STATUS_TIMEOUT;
                }
            }
            while (faultsCheck != 0UL);

            if (status != STATUS_TIMEOUT)
            {
                /* Put FCCU in Normal state */
                FCCU_DRV_RunOperation(instance, FCCU_RUN_OP2);
                /* Read FCCU configures state again */
                if (FCCU_STATUS_NORMAL == FCCU_DRV_GetConfigState(instance))
                {
                    status = STATUS_SUCCESS;
                }
                else
                {
                    status = STATUS_FCCU_ERROR_SET_NORMAL;
                }
            }
            break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_SetStateTimer
 * Description   : Set the value for TimeOut timer to set Config State
 * Not accessible from FCCU Config State
 *
 * Implements    : FCCU_DRV_SetStateTimer_Activity
 * END**************************************************************************/
status_t FCCU_DRV_SetStateTimer(const uint32_t instance,
                                const uint8_t value)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_OTHER;
    /* To be write FCCU need to be in Normal, Alarm, Fault Config, not applicable form Config State */
    if (FCCU_STATUS_CONFIGURATION == FCCU_DRV_GetConfigState(instance))
    {
        status = FCCU_DRV_SetNormalState(instance);
        if (STATUS_SUCCESS != status)
        {
            status = STATUS_FCCU_ERROR_CONFIG_TIMEOUT;
        }
        else
        {
            FCCU_SetTimerInterval(base, value);
        }
    }
    else
    {
        FCCU_SetTimerInterval(base, value);
        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_LockConfig
 * Description   : Locks the configuration
 * When locked, the fccu_CR, fccu_TO, fccu_WN and fccu_SK registers are read-only.
 *
 * Implements    : FCCU_DRV_LockConfig_Activity
 * END**************************************************************************/
void FCCU_DRV_LockConfig(const uint32_t instance,
                         const fccu_lock_type_t lock)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    switch (lock)
    {
        case FCCU_LOCK_TYPE_TEMPORARY:
            FCCU_SetTransientConfigLock(base, true);
            break;
        case FCCU_LOCK_TYPE_PERMANENT:
            FCCU_SetPermanentConfigLock(base);
            break;
        case FCCU_LOCK_TYPE_NO_LOCK:
            FCCU_SetTransientConfigLock(base, false);
            break;
        default:
            /* Do nothing */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_IrqStatusSetEvent
 * Description   : Update the status of the Interrupt Event
 * This can be done only if the Semaphore that locks the event is unlock.
 *
 * Implements    : FCCU_DRV_IrqStatusSetEvent_Activity
 * END**************************************************************************/
void FCCU_DRV_IrqStatusSetEvent(const uint32_t  instance,
                                const fccu_irq_status_t state)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_DRV_SemaphoreLock(instance);
    s_IrqStatus[instance].eventType = state;
    FCCU_DRV_SemaphoreUnlock(instance);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_IrqStatusGetEvent
 * Description   : Return the status of the Interrupt Event
 * This can be done only if the Semaphore that locks the event is unlock.
 *
 * Implements    : FCCU_DRV_IrqStatusGetEvent_Activity
 * END**************************************************************************/
void FCCU_DRV_IrqStatusGetEvent(const uint32_t  instance,
                                fccu_irq_status_t * const state)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(state != NULL);
    uint32_t msElapsed = OSIF_GetMilliseconds();
    while (FCCU_SEMAPHORE_UNLOCK != s_IrqStatus[instance].semaphore)
    {
        if (FCCU_TIME_DELAY <= (OSIF_GetMilliseconds() - msElapsed))
        {
            break;
        }
    }

    *state = s_IrqStatus[instance].eventType;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_IrqHandler
 * Description   : Set the status of the Interrupt Event
 * This can be done only if the Semaphore that locks the event is unlock.
 * If an NCF Alarm calls the registered Callback or if a global Callback
 * is registered
 *
 * Implements    : FCCU_DRV_IrqHandler_Activity
 * END**************************************************************************/
void FCCU_DRV_IrqHandler(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
    if (FCCU_GetIntStatus(base, FCCU_INT_EOUT))
    {
        FCCU_DRV_IrqStatusSetEvent(instance, FCCU_ISR_IRQ_EOUT);
        FCCU_ClearIntFlag(base, FCCU_INT_EOUT);
    }
#endif

    if (FCCU_GetIntStatus(base, FCCU_INT_TIMEOUT))
    {
        FCCU_DRV_IrqStatusSetEvent(instance, FCCU_ISR_IRQ_TIMEOUT);
        FCCU_ClearIntFlag(base, FCCU_INT_TIMEOUT);
    }

    if (FCCU_GetIntStatus(base, FCCU_INT_ALARM))
    {
        FCCU_DRV_IrqStatusSetEvent(instance, FCCU_ISR_IRQ_ALARM);
        FCCU_DRV_IrqAlarmCallback(instance);
        FCCU_ClearIntFlag(base, FCCU_INT_ALARM);
    }

    if (NULL != s_IrqStatus[instance].callBackFunction)
    {
        s_IrqStatus[instance].callBackFunction(s_IrqStatus[instance].callbackParam);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_IrqAlarmCallback
 * Description   : NCF Alarm calls the registered Callback
 * In case of IRQ_Alarm checks the triggered No Critical Fault
 * If configured callback then call it.
 *
 * Implements    : FCCU_DRV_IrqAlarmCallback_Activity
 * END**************************************************************************/
void FCCU_DRV_IrqAlarmCallback(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    uint32_t mask, tmpMask;
    uint8_t volatile index;
    uint8_t volatile bit_position;
    uint8_t register_index, aux_val;
    if (FCCU_ISR_IRQ_ALARM == s_IrqStatus[instance].eventType)
    {
        /* Update faults status */
        FCCU_SetOperationRun(base, FCCU_RUN_OP10);
        for (index = 0u; index < s_IrqStatus[instance].ncfConfigNb; index++)
        {
            register_index = s_ncfConfigurations[instance][index]->functionID >> (5U); /* divide by 32 */
            bit_position = (s_ncfConfigurations[instance][index]->functionID & (0x1FU)); /* modulo by 32 */
            /* Volatile MISRA Rule 13.2 */
            mask = (1UL << bit_position);
            tmpMask = mask & base->NCF_S[register_index];
            if ((register_index < FCCU_NCF_S_COUNT) && (mask == tmpMask))
            {
                FCCU_SetNCFKey(base, FEATURE_FCCU_NCF_KEY);
                /* Clear Fault */
                FCCU_ClearNCFsStatus(base, s_ncfConfigurations[instance][index]->functionID);
                if (NULL != s_ncfConfigurations[instance][index]->callback)
                {
                    /* Volatile MISRA Rule 13.2 */
                    aux_val = index;
                    s_ncfConfigurations[instance][aux_val]->callback(s_ncfConfigurations[instance][aux_val]->callbackParam);
                }
            }
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_SetNcfConfig
 * Description   : Set the Configured No Critical Faults
 * Put FCCU in config Mode and sets the configured NCFs and after switch back FCCU
 * to normal state
 *
 * Implements    : FCCU_DRV_SetNcfConfig_Activity
 * END**************************************************************************/
status_t FCCU_DRV_SetNcfConfig(const uint32_t instance,
                               fccu_config_ncf_t const * param)
{

    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_OTHER;

    status = FCCU_DRV_SetConfigState(instance);
    if (STATUS_SUCCESS ==  status)
    {
        /* Expected FCCU to be in Config Mode
         * Set NonCritical Fault Active associated with the ID no */
        FCCU_SetNCFConfig(base, param->functionID, param->hwSwRecovery);

        /* Allow Trigger Alarm or Fault State */
        FCCU_SetNCFEnable(base, param->functionID, true);

        FCCU_SetNCFTimeoutEnable(base, param->functionID, param->timeoutEnable);
        /* Set Interrupt Sources Type as Reaction Setting */
        FCCU_SetIntSource(base, FCCU_INT_ALARM, param->functionID, FCCU_DRV_CheckNcfReaction(param->reactionType, FCCU_INT_ALARM));
        FCCU_SetIntSource(base, FCCU_INT_NMI, param->functionID, FCCU_DRV_CheckNcfReaction(param->reactionType, FCCU_INT_NMI));
        FCCU_SetIntSource(base, FCCU_INT_EOUT, param->functionID, FCCU_DRV_CheckNcfReaction(param->reactionType, FCCU_INT_EOUT));

        /* Set reset reaction associated with the ID no */
        FCCU_SetNCFStatusConfig(base, param->functionID, param->reset);
        /* Return FCCU to Normal State Operation */
        status = FCCU_DRV_SetNormalState(instance);
    }

    /* Expected to be STATUS_SUCCESS */
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_EoutSet
 * Description   : This function will set the EOUT configuration that need to be
 * done in FCCU Config State, if pointer to configuration is NULL will deactivate
 * EOUT Signals.
 * In case FCCU is in normal State will try to put in Config state if no errors
 * detected and the put back FCCU in normal state, in case of Alarm & Fault State
 * the function will report error.
 *
 * Implements    : FCCU_DRV_EoutSet_Activity
 * END**************************************************************************/
status_t FCCU_DRV_EoutSet(const uint32_t instance,
                          const fccu_eout_config_t * const param)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(param != NULL);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_OTHER;
    fccu_faults_flags_t faults;
    fccu_status_t fccuState;
    uint32_t faultsCheck;
    uint8_t idx;
    bool wasNormal = false;

    fccuState = FCCU_DRV_GetConfigState(instance);
    if (FCCU_STATUS_NORMAL == fccuState)
    {
        FCCU_DRV_ReadFaults(instance, &faults);
        faultsCheck = 0U;
        for (idx = 0U; idx < FCCU_NCF_S_COUNT; idx++)
        {
            faultsCheck += faults.faultsFlags[idx];
        }

        if (faultsCheck != 0U)
        {
            status = STATUS_FCCU_ERROR_FAULT_DETECTED;
            /* Exit and return error */
        }
        else
        {
            status = FCCU_DRV_SetConfigState(instance);
            wasNormal = true;
            fccuState = FCCU_DRV_GetConfigState(instance);
            /* Pass-trough in case of no error detect and put in config state  to set EOUT*/
        }
    }

    switch (fccuState)
    {
        case FCCU_STATUS_CONFIGURATION:
            /* Checks in case of the pass-through to FCCU to be set in config state */
			FCCU_SetFaultOutputActivate(base, param->activate);
			FCCU_SetFaultOutputControl(base, param->control);
#if FEATURE_FCCU_RCC_EN
            /* Set FCCU's redundancy control checker */
            FCCU_SetRedundancyControlChecker(base, param->setRcc);
#endif /* FEATURE_FCCU_RCC_EN */
#if !defined(FEATURE_FCCU_FOP_SUPPORT)
			/* Base on prescaler Value can be calculated (value/64)|(value%64) Max Allow Value is 127 */
			FCCU_SetFaultOutputPrescalerExtension(base, ((param->prescaler >> 0x6U) == 0U) ? false : true); /*divide by 64*/
			FCCU_SetFaultOutputPrescaler(base, (param->prescaler & 0x3FU)); /*modulo by 64*/
#endif /* FEATURE_FCCU_FOP_SUPPORT */
#if FEATURE_FCCU_OPEN_DRAIN_EN
			FCCU_SetOpenDrain(base, param->openDrain);
#endif
/* Depends on CPU what type of IRQ Support */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
			FCCU_SetFaultOutputTogglingTime(base, param->toggleTime);
			FCCU_SetFaultOutputExtension(base, param->extMode);
			FCCU_SetIRQDMATriggerTime(base,param->triggerTimeIRQDMA);
#endif
			FCCU_SetBiFOFaultyInterval(base,param->deltaFaultInterval);
			FCCU_SetFaultOutputSwitchingMode(base, param->switchMode);
			FCCU_SetFaultOutputPolaritySelection(base, param->polarity);
			FCCU_SetFaultOutputMode(base, param->mode);

            /* if the FCCU state was previous in normal state and was a pass-through from Normal */
            if (wasNormal)
            {
                status = FCCU_DRV_SetNormalState(instance);
            }
            else
            {
                status = STATUS_SUCCESS;
            }
            break;
        case FCCU_STATUS_ALARM:
        case FCCU_STATUS_FAULT:
            /* fall-through */
            /* Expect to be in Alarm or Fault State */
            status = STATUS_FCCU_ERROR_SET_EOUT;
            break;
        default:
            /* Do nothings */
            break;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_GetDefaultConfig
 * Description   : This function will get default value of configuration for FCCU module.
 *
 * Implements    : FCCU_DRV_GetDefaultConfig_Activity
 * END**************************************************************************/
void FCCU_DRV_GetDefaultConfig(fccu_control_t * controlConfig,
                               fccu_config_ncf_t * ncfConfig)
{
    DEV_ASSERT(controlConfig != NULL);
    DEV_ASSERT(ncfConfig != NULL);

    /*! @brief EOUT configuration */
    static fccu_eout_config_t eoutConfig =
    {
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
        .toggleTime            = 100U,
        .extMode               = FCCU_FO_EXT_DEFAULT,
        .triggerTimeIRQDMA     = 0U,
#endif
        .activate              = true,
        .control               = FCCU_FO_CONTROLLED_BY_FSM,
#if FEATURE_FCCU_RCC_EN
        /* Set FCCU's redundancy control checker */
        .setRcc                = FCCU_RCC12_EN,
#endif /* FEATURE_FCCU_RCC_EN */
#if FEATURE_FCCU_OPEN_DRAIN_EN
        .openDrain             = false,
#endif
        .switchMode            = true,
        .mode                  = FCCU_FO_DUAL_RAIL,
#if !defined(FEATURE_FCCU_FOP_SUPPORT)
        .prescaler             = 2U,
#endif
        .phase                 = FCCU_FO_OPPOSITE_PHASE_10,
        .polarity              = false,
        .deltaFaultInterval    = 0U
    };

    /*! @brief Control configuration */
#if FEATURE_FCCU_FILTER_EN
    controlConfig->filterBypass     = false;
    controlConfig->filterWidth      = FCCU_FILTERWIDTH_UP_TO_50_US;
#endif
    controlConfig->debugEnable      = false;
    controlConfig->irqEnableType    = FCCU_IRQ_EN_NOIRQ;
    controlConfig->ncfTimeout       = 5000U;
    controlConfig->configRun        = (fccu_eout_config_t *)&eoutConfig;
    controlConfig->lockType         = FCCU_LOCK_TYPE_NO_LOCK;
    controlConfig->ncfConfigNumber  = 1U;
    controlConfig->callbackIsr      = NULL;
    controlConfig->callbackIsrParam = NULL;

    /*! @brief Non-critical fault configuration */
    ncfConfig->functionID        = 2U;
    ncfConfig->hwSwRecovery      = FCCU_NCF_SW_REC_FAULT;
    ncfConfig->reset             = FCCU_NCFS_NO_RESET;
    ncfConfig->timeoutEnable     = false;
    ncfConfig->reactionType      = 0U;
    ncfConfig->callback          = NULL;
    ncfConfig->callbackParam     = NULL;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_Init
 * Description   : This initialize FCCU peripheral state machine as PEX generated
 * and configure the alarms.
 *
 * Implements    : FCCU_DRV_Init_Activity
 * END**************************************************************************/
status_t FCCU_DRV_Init(const uint32_t instance,
                       fccu_control_t config,
                       fccu_config_ncf_t const ** param)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT((void *)&config != NULL);
    DEV_ASSERT(param != NULL);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_OTHER;
    uint8_t index;
    /* Initialize  Timer for OSIF_GetMiliseconds enable the TimeOut feature*/
    OSIF_TimeDelay(0);
    s_ncfConfigurations[instance] = param;
    /* Initialize IRQ Status for FCCU Instance */
    s_IrqStatus[instance].semaphore        = FCCU_SEMAPHORE_UNLOCK;
    s_IrqStatus[instance].eventType       = FCCU_ISR_NO;
    s_IrqStatus[instance].ncfConfigNb     = config.ncfConfigNumber;
    s_IrqStatus[instance].callBackFunction = config.callbackIsr;
    s_IrqStatus[instance].callbackParam    = config.callbackIsrParam;

    FCCU_SetDebugMode(base, config.debugEnable);
#if FEATURE_FCCU_FILTER_EN
    FCCU_SetFilterByPass(base, config.filterBypass);
    FCCU_SetFilterWidth(base, config.filterWidth);
#endif

/* Depends on CPU what type of IRQ Support */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
    /* Need to register IRQ and set params */
    if (FCCU_IRQ_EN_EOUT_WITH_CFG_TO == config.irqEnableType)
    {
        FCCU_SetIntEnable(base, FCCU_INT_TIMEOUT, true);
        FCCU_SetIntEnable(base, FCCU_INT_EOUT, true);
        INT_SYS_InstallHandler(FCCU_MISC_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
        INT_SYS_InstallHandler(FCCU_EOUT_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
        INT_SYS_EnableIRQ(FCCU_EOUT_IRQn);
        INT_SYS_EnableIRQ(FCCU_MISC_IRQn);
    }

    if (FCCU_IRQ_EN_EOUT == config.irqEnableType)
    {
        FCCU_SetIntEnable(base, FCCU_INT_TIMEOUT, false);
        FCCU_SetIntEnable(base, FCCU_INT_EOUT, true);
        INT_SYS_InstallHandler(FCCU_EOUT_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
        INT_SYS_EnableIRQ(FCCU_EOUT_IRQn);
    }
#endif

    if (FCCU_IRQ_EN_CGF_TO == config.irqEnableType)
    {
        FCCU_SetIntEnable(base, FCCU_INT_TIMEOUT, true);
/* Depends on CPU what type of IRQ Support */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
        FCCU_SetIntEnable(base, FCCU_INT_EOUT, false);
#endif
        INT_SYS_InstallHandler(FCCU_MISC_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
        INT_SYS_EnableIRQ(FCCU_MISC_IRQn);
    }

    for (index = 0u; index < s_IrqStatus[instance].ncfConfigNb; index++)
    {
        if (FCCU_DRV_CheckNcfReaction((uint8_t)s_ncfConfigurations[instance][index]->reactionType, FCCU_INT_ALARM) == true)
        {
#if defined(FEATURE_FCCU_MISC_ALARM_TIMEOUT_IRQn)
            if (FCCU_IRQ_EN_CGF_TO != config.irqEnableType)
            {
                INT_SYS_InstallHandler(FCCU_MISC_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
                INT_SYS_EnableIRQ(FCCU_MISC_IRQn);
            }
#else
            INT_SYS_InstallHandler(FCCU_ALARM_IRQn, (isr_t)FCCU0_IRQ_Handler, NULL);
            INT_SYS_EnableIRQ(FCCU_ALARM_IRQn);
#endif
            break;
        }
    }

    status = FCCU_DRV_SetConfigState(instance);
    if (STATUS_SUCCESS == status)
    {
        /* Here FCCU Need to be in Config State To allow configuration */
        FCCU_SetNCFTimeout(base, config.ncfTimeout);
        /* Apply the configuration from EOUT state config */
        (void)FCCU_DRV_EoutSet(instance, config.configRun);
        /* Set Configured events for Non Critical Faults */
        for (index = 0u; index < s_IrqStatus[instance].ncfConfigNb; index++)
        {
            status = FCCU_DRV_SetNcfConfig(instance, s_ncfConfigurations[instance][index]);
            if (STATUS_SUCCESS != status)
            {
                break;
            }
        }

        /* Save configuration Registers FCCU need to be in Normal State */
        if (STATUS_SUCCESS == status)
        {
            status = FCCU_DRV_SetNormalState(instance);
            if (STATUS_SUCCESS == status)
            {
                /* Apply Locking Configuration */
                FCCU_DRV_LockConfig(instance, config.lockType);
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_GetFreezeStatus
 * Description   : This will update the freeze status registers based on type selected
 * and will return the value of it, in case of operation of update will fail will return
 * error status, in case of success will update the value and return success.
 *
 * Implements    : FCCU_DRV_GetFreezeStatus_Activity
 * END**************************************************************************/
status_t FCCU_DRV_GetFreezeStatus(const uint32_t instance,
                                  const fccu_freeze_type_t type,
                                  uint8_t * const value)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(value != NULL);
    const FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_FCCU_ERROR_UPDATE_FREEZE;
    switch (type)
    {
        case FCCU_FRZ_NORMAL_ALARM:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP4);
            break;
        case FCCU_FRZ_ALARM_FAULT:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP5);
            break;
        case FCCU_FRZ_NORMAL_FAULT:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP6);
            break;
        case FCCU_FRZ_FAULT_ALARM:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP7);
            break;
#if FEATURE_FCCU_RCC_EN
        case FCCU_FRZ_SELF_CHECK_RCC:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP8);
            break;
#endif
        default:
            /* Do nothing */
            break;
    }

    if (FCCU_OP_SUCCESSFUL == FCCU_GetOperationStatus(base))
    {
        *value = FCCU_GetFreezeStatus(base, type);
        status = STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_ClearAllFreezeStatus
 * Description   : This will clear all freeze status registers, in case the operation
 * fails will return error else will return success
 *
 * Implements    : FCCU_DRV_ClearAllFreezeStatus_Activity
 * END**************************************************************************/
status_t FCCU_DRV_ClearAllFreezeStatus(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    const FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_SUCCESS;
    FCCU_DRV_RunOperation(instance, FCCU_RUN_OP13);
    if (FCCU_OP_SUCCESSFUL != FCCU_GetOperationStatus(base))
    {
        status = STATUS_FCCU_ERROR_CLEAR_FREEZE;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_GetXtmrValue
 * Description   : This will update and return the XTMR value based on user selection
 *
 * Implements    : FCCU_DRV_GetXtmrValue_Activity
 * END**************************************************************************/
uint32_t FCCU_DRV_GetXtmrValue(const uint32_t instance,
                               const fccu_xtmr_type_t type)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    const FCCU_Type * const base = s_fccuBase[instance];
    uint32_t value = 0;
    switch(type)
    {
        case FCCU_XTMR_ALARM:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP17);
            if (FCCU_OP_SUCCESSFUL == FCCU_GetOperationStatus(base))
            {
                value = FCCU_GetCounterValue(base);
            }
            break;
        case FCCU_XTMR_CONFIG:
             FCCU_DRV_RunOperation(instance, FCCU_RUN_OP19);
             if (FCCU_OP_SUCCESSFUL == FCCU_GetOperationStatus(base))
             {
                 value = FCCU_GetCounterValue(base);
             }
            break;
/* Depends on CPU what type of IRQ Support */
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
        case FCCU_XTMR_ETMR:
            FCCU_DRV_RunOperation(instance, FCCU_RUN_OP20);
             if (FCCU_OP_SUCCESSFUL == FCCU_GetOperationStatus(base))
             {
                 value = FCCU_GetCounterValue(base);
             }
            break;
#endif
        default:
            /* Do Nothing */
            break;
    }

    return value;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_SetNcfFake
 * Description   : This function will trigger a NonCritical Fault fake to allow
 * a test of behavior of configured NonCritical Faults. This function is in scope
 * of Debug the Configuration of FCCU Module.
 *
 * Implements    : FCCU_DRV_SetNcfFake_Activity
 * END**************************************************************************/
void FCCU_DRV_SetNcfFake(const uint32_t instance,
                         const uint8_t ncfValue)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(ncfValue <= FEATURE_FCCU_MAX_FAULTS_NO);
    FCCU_Type * const base = s_fccuBase[instance];
    FCCU_SetNCFFake(base, ncfValue);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_Deinit
 * Description   : This initialize FCCU peripheral state machine as PEX generated
 * and configure the alarms.
 *
 * Implements    : FCCU_DRV_Deinit_Activity
 * END**************************************************************************/
status_t FCCU_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    uint8_t index;
    status_t status = STATUS_FCCU_ERROR_OTHER;
    s_ncfConfigurations[instance] = NULL;
    /* De-initialize IRQ Status for FCCU Instance */
    s_IrqStatus[instance].semaphore        = FCCU_SEMAPHORE_UNLOCK;
    s_IrqStatus[instance].eventType        = FCCU_ISR_NO;
    s_IrqStatus[instance].ncfConfigNb      = 0;
    s_IrqStatus[instance].callBackFunction = NULL;
    s_IrqStatus[instance].callbackParam    = NULL;

    FCCU_SetDebugMode(base, false);
#if FEATURE_FCCU_FILTER_EN
    FCCU_SetFilterByPass(base, false);
    FCCU_SetFilterWidth(base, FCCU_FILTERWIDTH_UP_TO_50_US);
#endif
    /* Disable Interrupts */
    INT_SYS_DisableIRQGlobal();
    FCCU_SetIntEnable(base, FCCU_INT_TIMEOUT, false);
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
    FCCU_SetIntEnable(base, FCCU_INT_EOUT, false);
    INT_SYS_DisableIRQ(FCCU_EOUT_IRQn);
#endif
    INT_SYS_DisableIRQ(FCCU_MISC_IRQn);
#ifndef FEATURE_FCCU_MISC_ALARM_TIMEOUT_IRQn
    INT_SYS_DisableIRQ(FCCU_ALARM_IRQn);
#endif
    INT_SYS_EnableIRQGlobal();

    status = FCCU_DRV_SetConfigState(instance);
    if (STATUS_SUCCESS == status)
    {
        /* Here FCCU Need to be in Config State To allow configuration */
        FCCU_SetNCFTimeout(base, 0U);
        /* Apply Default Values for EOUT */
        FCCU_SetFaultOutputActivate(base, false);
        FCCU_SetFaultOutputControl(base, FCCU_FO_CONTROLLED_BY_FSM);
#if !defined(FEATURE_FCCU_FOP_SUPPORT)
        FCCU_SetFaultOutputPrescalerExtension(base, false);
        FCCU_SetFaultOutputPrescaler(base, 0);
#endif /* FEATURE_FCCU_FOP_SUPPORT */
#if FEATURE_FCCU_RCC_EN
        /* Set FCCU's redundancy control checker */
        FCCU_SetRedundancyControlChecker(base, FCCU_RCC_NO);
#endif /* FEATURE_FCCU_RCC_EN */
#if FEATURE_FCCU_OPEN_DRAIN_EN
        FCCU_SetOpenDrain(base, false);
#endif
#if (FEATURE_FCCU_IRQ_EN_MASK == 9U)
        FCCU_SetFaultOutputTogglingTime(base, 0U);
        FCCU_SetFaultOutputExtension(base, FCCU_FO_EXT_DEFAULT);
        FCCU_SetIRQDMATriggerTime(base,0);
#endif
        FCCU_SetBiFOFaultyInterval(base,0);
        FCCU_SetFaultOutputSwitchingMode(base, false);
        FCCU_SetFaultOutputPolaritySelection(base, false);
        FCCU_SetFaultOutputMode(base, FCCU_FO_DUAL_RAIL);
        /* Set Non Critical Faults Registers */
        for (index = 0U; index < FCCU_NCF_CFG_COUNT; index++)
        {
            base->NCF_CFG[index] = 0UL;
        }
        for (index = 0U; index < FCCU_NCFS_CFG_COUNT; index++)
        {
            base->NCFS_CFG[index] = 0UL;
        }
        for (index = 0U; index < FCCU_NCF_E_COUNT; index++)
        {
            base->NCF_E[index] = 0UL;
        }
        for (index = 0U; index < FCCU_NCF_TOE_COUNT; index++)
        {
            base->NCF_TOE[index] = 0UL;
        }
        for (index = 0U; index < FCCU_IRQ_ALARM_EN_COUNT; index++)
        {
            base->IRQ_ALARM_EN[index] = 0UL;
        }
        for (index = 0U; index < FCCU_NMI_EN_COUNT; index++)
        {
            base->NMI_EN[index] = 0UL;
        }
        for (index = 0U; index < FCCU_EOUT_SIG_EN_COUNT; index++)
        {
            base->EOUT_SIG_EN[index] = 0UL;
        }
        status = FCCU_DRV_SetNormalState(instance);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FCCU_DRV_DisableFault
 * Description   : This disable FCCU Non Critical Fault source number if it wasn't
 * triggered already
 *
 * Implements    : FCCU_DRV_DisableFault_Activity
 * END**************************************************************************/
status_t FCCU_DRV_DisableFault(const uint32_t instance,
                               uint8_t faultIdx)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    FCCU_Type * const base = s_fccuBase[instance];
    status_t status = STATUS_SUCCESS;
    fccu_status_t fccuState;

    /* Check if the NonCritical Fault is enabled else is disabled and report success */
    if (FCCU_GetNCFStatus(base, faultIdx))
    {
        fccuState = FCCU_DRV_GetConfigState(instance);
        /* Update Status Fault Register with OP10 command */
        FCCU_DRV_RunOperation(instance, FCCU_RUN_OP10);

        /* Check if the Disable Fault wasn't triggered */
        if (FCCU_GetNCFsStatus(base,faultIdx))
        {
            /* Report Fault Detected to be handled by user*/
            status = STATUS_FCCU_ERROR_FAULT_DETECTED;
        }
        /* If fault detected don't disable it report STATUS_FCCU_ERROR_FAULT_DETECTED */
        else
        {
            status = FCCU_DRV_SetConfigState(instance);
            if (STATUS_SUCCESS == status)
            {
                /* Disable Fault Source */
                FCCU_ClearNCFStatus(base, faultIdx);
                /* Disable Reset Reaction */
                FCCU_SetNCFStatusConfig(base, faultIdx, FCCU_NCFS_NO_RESET);
                FCCU_SetNCFEnable(base,faultIdx,false);
                /* Disable Alarm */
                FCCU_SetNCFTimeoutEnable(base,faultIdx,false);
                if (FCCU_STATUS_NORMAL == fccuState)
                {
                    /* Report success if FCCU was restored to normal state and disabled fault */
                    status = FCCU_DRV_SetNormalState(instance);
                }
                else
                {
                    status = STATUS_SUCCESS;
                }
            }
        }
    }

    return status;
}

#if FEATURE_FCCU_CONTROL_MODE_EN
/*FUNCTION**********************************************************************
 * Function Name : FCCU_DRV_GetChipMode
 * Description   : This function capture chip modes—as defined by the MC_ME module
 * based on the last event selected.
 *
 * Implements    : FCCU_DRV_GetChipMode_Activity
 * END**************************************************************************/
 void FCCU_DRV_GetChipMode(const uint32_t instance,
                        fccu_chip_mode_t recentMode,
                        fccu_mode_info_t * const infoPtr)
{
    DEV_ASSERT(instance < FCCU_INSTANCE_COUNT);
    DEV_ASSERT(infoPtr != NULL);
    const FCCU_Type * const base = s_fccuBase[instance];
    FCCU_GetModeControllerStatus(base, recentMode, infoPtr);
}
#endif /* FEATURE_FCCU_CONTROL_MODE_EN */

/*******************************************************************************
* EOF
*******************************************************************************/
