/*
 * Copyright 2017-2018 NXP
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
 * @file pass_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, cannot cast from
 * 'essentially unsigned' to 'essentially enum<i>'
 * This cast is required to return the enumeration type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (different essential type categories)
 * This cast is required to return the enumeration type.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower
 * or different essential type
 * This cast is required to return the enumeration type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */

#include "pass_driver.h"
#include "device_registers.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_GetLifeCycle
 * Description   : Get the lifecycle status of the device.
 *
 * Implements    : PASS_DRV_GetLifeCycle_Activity
 *END**************************************************************************/
pass_life_cycle_t PASS_DRV_GetLifeCycle(void)
{
    const PASS_Type * base = PASS;

    return (pass_life_cycle_t)((base->LCSTAT & PASS_LCSTAT_LIFE_MASK) >> PASS_LCSTAT_LIFE_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_CheckCensorship
 * Description   : This function is used to check if the device is censored or not.
 *
 * Implements    : PASS_DRV_CheckCensorship_Activity
 *END**************************************************************************/
bool PASS_DRV_CheckCensorship(void)
{
    const PASS_Type * base = PASS;
    bool ret = true;

    if ((base->LCSTAT & PASS_LCSTAT_CNS_MASK) != 0U)
    {
        ret = false;
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_UnlockPassgroup
 * Description   : Unlock the accessing to the PASSGROUP registers (LOCK0, LOCK1, LOCK2, LOCK3).
 *  By default, after resetting, the PASSGROUP registers are always locked. So user have to call
 *  this function first to unlock PASSGROUP registers before calling functions: PASS_DRV_SetWriteEraseState,
 *  PASS_DRV_SetReadState
 *
 * Implements    : PASS_DRV_UnlockPassgroup_Activity
 *END**************************************************************************/
status_t PASS_DRV_UnlockPassgroup(uint8_t passgroup,
                                  const uint32_t * password)
{
    DEV_ASSERT(passgroup < PASS_PG_COUNT);
    DEV_ASSERT(password != NULL);

    PASS_Type * const base = PASS;
    status_t error = STATUS_SUCCESS;
    uint8_t i = 0;

    /* Select the passgroup to unlock */
    base->CHSEL = 0;
    base->CHSEL = PASS_CHSEL_GRP(passgroup);

    /* Set password to CINn registers to unlock */
    for (i = 0; i < PASS_CIN_COUNT; i++)
    {
        base->CIN[i] = password[i];
    }

    /* After unlocking, PGL bit of LOCK3 register must be 0 */
    if ((base->PG[passgroup].LOCK3 & PASS_LOCK3_PGL_MASK) != 0U)
    {
        error = STATUS_ERROR;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_PassgroupIsUnlock
 * Description   : This function is used to check if a given passgroup is unlocked or not.
 *
 * Implements    : PASS_DRV_PassgroupIsUnlock_Activity
 *END**************************************************************************/
bool PASS_DRV_PassgroupIsUnlock(uint8_t passgroup)
{
    const PASS_Type * base = PASS;
    bool result = true;

    if ((base->PG[passgroup].LOCK3 & PASS_LOCK3_PGL_MASK) != 0U)
    {
        result = false;        /* Passgroup is locked */
    }

    return result;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_SetWriteEraseState
 * Description   : Lock or unlock for write and erase operation for the given blocks of Flash memory.
 *  This function can be only used when PASSGROUP registers are unlocked. User can use PASS_DRV_UnlockPassgroup()
 *  function to unlock the PASSGROUP registers.
 *  NOTE: This function only sets the state temporarily. After resetting, the new state will be reloaded from DCF records.
 *
 * Implements    : PASS_DRV_SetWriteEraseState_Activity
 *END**************************************************************************/
status_t PASS_DRV_SetWriteEraseState(pass_flash_block_t blockType,
                                     uint8_t passgroup,
                                     uint32_t newState)
{
    DEV_ASSERT(passgroup < PASS_PG_COUNT);

    PASS_Type * base = PASS;
    status_t error = STATUS_SUCCESS;
    /* Check if passgroup has been unlocked or not */
    if ((base->PG[passgroup].LOCK3 & PASS_LOCK3_PGL_MASK) != 0U)
    {
        error = STATUS_ERROR;
    }
    else
    {
        switch (blockType)
        {
        case PASS_FLASH_BLOCK_LOW_MIDDLE:
            base->PG[passgroup].LOCK0 = newState;
            break;
        case  PASS_FLASH_BLOCK_HIGH:
            base->PG[passgroup].LOCK1 = newState;
            break;
        case PASS_FLASH_BLOCK_L256K:
            base->PG[passgroup].LOCK2 = newState;
            break;
        case PASS_FLASH_BLOCK_U256K:
            base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_U_256LCK_MASK;        /* Clear the U256LCK field */
            base->PG[passgroup].LOCK3 |= PASS_LOCK3_U_256LCK(newState);    /* Set the new state to U256LCK field */
            break;
        default:
            error = STATUS_ERROR;
            break;
        }
    }
    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_SetReadState
 * Description   : Lock or unlock for read operation for the given regions of Flash memory.
 *  This function can be only used when PASSGROUP registers are unlocked. User can use PASS_DRV_UnlockPassgroup()
 *  function to unlock the PASSGROUP registers.
 *  NOTE: This function only sets the state temporarily. After resetting, the new state will be reloaded from DCF records.
 *
 * Implements    : PASS_DRV_SetReadState_Activity
 *END**************************************************************************/
status_t PASS_DRV_SetReadState(pass_flash_region_t region,
                               uint8_t passgroup,
                               pass_read_state_t newState)
{
    DEV_ASSERT(passgroup < PASS_PG_COUNT);

    status_t error = STATUS_SUCCESS;
    PASS_Type * base = PASS;

    switch (region)
    {
/* MCU has region 0 */
#if (defined(FEATURE_PASS_HAS_RL0))
    case PASS_FLASH_REGION_RL0:
        base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_RL0_MASK;
        base->PG[passgroup].LOCK3 |= PASS_LOCK3_RL0(newState);
        break;
#endif
/* MCU has region 1 */
#if (defined(FEATURE_PASS_HAS_RL1))
    case PASS_FLASH_REGION_RL1:
        base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_RL1_MASK;
        base->PG[passgroup].LOCK3 |= PASS_LOCK3_RL1(newState);
        break;
#endif
/* MCU has region 2 */
#if (defined(FEATURE_PASS_HAS_RL2))
    case PASS_FLASH_REGION_RL2:
        base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_RL2_MASK;
        base->PG[passgroup].LOCK3 |= PASS_LOCK3_RL2(newState);
        break;
#endif
/* MCU has region 3 */
#if (defined(FEATURE_PASS_HAS_RL3))
    case PASS_FLASH_REGION_RL3:
        base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_RL3_MASK;
        base->PG[passgroup].LOCK3 |= PASS_LOCK3_RL3(newState);
        break;
#endif
/* MCU has region 4 */
#if (defined(FEATURE_PASS_HAS_RL4))
    case PASS_FLASH_REGION_RL4:
        base->PG[passgroup].LOCK3 &= ~PASS_LOCK3_RL4_MASK;
        base->PG[passgroup].LOCK3 |= PASS_LOCK3_RL4(newState);
        break;
#endif
    default:
        error = STATUS_ERROR;
        break;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_GetWriteEraseStatus
 * Description   : Check if the given blocks of Flash memory is writable/erasable or not
 *
 * Implements    : PASS_DRV_GetWriteEraseStatus_Activity
 *END**************************************************************************/
status_t PASS_DRV_GetWriteEraseStatus(pass_flash_block_t blockType,
                                      uint8_t passgroup,
                                      uint32_t * status)
{
    DEV_ASSERT(passgroup < PASS_PG_COUNT);
    DEV_ASSERT(status != NULL);

    const PASS_Type * base = PASS;
    status_t error = STATUS_SUCCESS;

    switch (blockType)
    {
    case PASS_FLASH_BLOCK_LOW_MIDDLE:
        *status = base->PG[passgroup].LOCK0;
        break;
    case PASS_FLASH_BLOCK_HIGH:
        *status = base->PG[passgroup].LOCK1;
        break;
    case PASS_FLASH_BLOCK_L256K:
        *status = base->PG[passgroup].LOCK2;
        break;
    case PASS_FLASH_BLOCK_U256K:
        *status = (base->PG[passgroup].LOCK3 & PASS_LOCK3_U_256LCK_MASK);
        break;
    default:
        error = STATUS_ERROR;
        break;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PASS_DRV_GetReadStatus
 * Description   : Check if the given blocks of Flash memory is readable or not
 *
 * Implements    : PASS_DRV_GetReadStatus_Activity
 *END**************************************************************************/
status_t PASS_DRV_GetReadStatus(pass_flash_region_t region,
                                uint8_t passgroup,
                                pass_read_state_t * status)
{
    DEV_ASSERT(passgroup < PASS_PG_COUNT);
    DEV_ASSERT(status != NULL);

    const PASS_Type * base = PASS;
    status_t error = STATUS_SUCCESS;

    switch (region)
    {
/* MCU has region 0 */
#if (defined(FEATURE_PASS_HAS_RL0))
    case PASS_FLASH_REGION_RL0:
        *status = (pass_read_state_t)((base->PG[passgroup].LOCK3 & PASS_LOCK3_RL0_MASK) >> PASS_LOCK3_RL0_SHIFT);
        break;
#endif
/* MCU has region 1 */
#if (defined(FEATURE_PASS_HAS_RL1))
    case PASS_FLASH_REGION_RL1:
        *status = (pass_read_state_t)((base->PG[passgroup].LOCK3 & PASS_LOCK3_RL1_MASK) >> PASS_LOCK3_RL1_SHIFT);
        break;
#endif
/* MCU has region 2 */
#if (defined(FEATURE_PASS_HAS_RL2))
    case PASS_FLASH_REGION_RL2:
        *status = (pass_read_state_t)((base->PG[passgroup].LOCK3 & PASS_LOCK3_RL2_MASK) >> PASS_LOCK3_RL2_SHIFT);
        break;
#endif
/* MCU has region 3 */
#if (defined(FEATURE_PASS_HAS_RL3))
    case PASS_FLASH_REGION_RL3:
        *status = (pass_read_state_t)((base->PG[passgroup].LOCK3 & PASS_LOCK3_RL3_MASK) >> PASS_LOCK3_RL3_SHIFT);
        break;
#endif
/* MCU has region 4 */
#if (defined(FEATURE_PASS_HAS_RL4))
    case PASS_FLASH_REGION_RL4:
        *status = (pass_read_state_t)((base->PG[passgroup].LOCK3 & PASS_LOCK3_RL4_MASK) >> PASS_LOCK3_RL4_SHIFT);
        break;
#endif
    default:
        error = STATUS_ERROR;
        break;
    }

    return error;
}
