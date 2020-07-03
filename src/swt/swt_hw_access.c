/*
 * Copyright 2017 NXP.
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
 * @file swt_hw_access.c
 */

#include <stddef.h>
#include "swt_hw_access.h"

/*******************************************************************************
 * Code
 *******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_UnLock
 * Description   : Unlock SWT registers.
 * - The soft lock is cleared by writing the unlock sequence to the service register.
 * - The hard lock can only be cleared by a reset.
 *
 *END**************************************************************************/
status_t SWT_UnLock(SWT_Type * const base)
{
    status_t ret = STATUS_SUCCESS;

    /* Hard lock */
    if ((base->CR & SWT_CR_HLK_MASK) != 0U)
    {
        ret = STATUS_ERROR;
    }
    /* Soft lock */
    else if ((base->CR & SWT_CR_SLK_MASK) != 0U)
    {
        /* Unlocks sequence */
        base->SR = SWT_SR_WSC(FEATURE_SWT_UNLOCK_VALUE1);
        base->SR = SWT_SR_WSC(FEATURE_SWT_UNLOCK_VALUE2);
        /* Waits unlock complete */
        while ((base->CR & SWT_CR_SLK_MASK) != 0U)
        {
            /* Do nothing */
        }
    }
    /* Unlock */
    else
    {
        /* Do nothing */
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_Lock
 * Description   : Lock SWT registers.
 * When locked, the SWT_CR, SWT_TO, SWT_WN and SWT_SK registers are read-only.
 *
 *END**************************************************************************/
void SWT_Lock(SWT_Type * const base,
              swt_lock_t lockConfig)
{
    /* Configures lock bits */
    switch (lockConfig)
    {
        /* Hard lock */
        case SWT_HARDLOCK:
            base->CR |= SWT_CR_HLK(1U);
            break;
        /* Soft lock */
        case SWT_SOFTLOCK:
            base->CR |= SWT_CR_SLK(1U);
            break;
        /* Unlock */
        default:
            /* Do nothing */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_Reset
 * Description   : Resets the SWT.
 *
 *END**************************************************************************/
void SWT_Reset(SWT_Type * const base)
{
    /* Clears interrupt flag */
    base->IR = SWT_IR_TIF_MASK;
    /* Resets timeout value */
    base->TO = FEATURE_SWT_TO_RESET_VALUE;
    /* Resets window value */
    base->WN = FEATURE_SWT_WN_RESET_VALUE;
    /* Resets service key value */
    base->SK = FEATURE_SWT_SK_RESET_VALUE;
    /* Resets control register */
    base->CR = FEATURE_SWT_CR_RESET_VALUE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_Config
 * Description   : Configures the SWT.
 *
 *END**************************************************************************/
status_t SWT_Config(SWT_Type * const base,
                    const swt_user_config_t * const userConfigPtr)
{
    DEV_ASSERT(userConfigPtr != NULL);

    uint32_t cr = base->CR;
    /* Unlocks the SWT */
    status_t ret = SWT_UnLock(base);

    /* Check unlock status */
    if (ret == STATUS_SUCCESS)
    {
        /* Checks timeout value */
        if (userConfigPtr->timeoutValue >= FEATURE_SWT_TO_MINIMUM_VALUE)
        {
            /* Clears the bits used for configuration */
            cr &= ~(FEATURE_SWT_MAP_MASK 
                  | SWT_CR_SMD_MASK 
                  | SWT_CR_RIA_MASK 
                  | SWT_CR_WND_MASK 
                  | SWT_CR_ITR_MASK 
            #if FEATURE_SWT_HAS_CLOCK_SELECT
                  | SWT_CR_CSL_MASK
            #endif
            #if FEATURE_SWT_HAS_STOP_MODE
                  | SWT_CR_STP_MASK
            #endif
                  | SWT_CR_FRZ_MASK);
            /* Sets control configuration */
            cr |= (FEATURE_SWT_MAP(userConfigPtr->mapConfig)
                   | SWT_CR_SMD(userConfigPtr->serviceMode)
                   | SWT_CR_RIA(userConfigPtr->invalidReset ? 1UL : 0UL)
                   | SWT_CR_WND(userConfigPtr->winEnable ? 1UL : 0UL)
                   | SWT_CR_ITR(userConfigPtr->intEnable ? 1UL : 0UL)
            #if FEATURE_SWT_HAS_CLOCK_SELECT
                   | SWT_CR_CSL(userConfigPtr->clockSelect ? 1UL : 0UL)
            #endif
            #if FEATURE_SWT_HAS_STOP_MODE
                   | SWT_CR_STP(userConfigPtr->stop ? 1UL : 0UL)
            #endif
                   | SWT_CR_FRZ(userConfigPtr->debug ? 1UL : 0UL)
                   | SWT_CR_WEN(1UL));
            /* Configures lock bits */
            switch (userConfigPtr->lockConfig)
            {
                /* Hard lock */
                case SWT_HARDLOCK:
                    cr |= SWT_CR_HLK(1UL);
                    break;
                /* Soft lock */
                case SWT_SOFTLOCK:
                    cr |= SWT_CR_SLK(1UL);
                    break;
                /* Unlock */
                default:
                    /* Do nothing */
                    break;
            }

            /* Clears interrupt flags */
            base->IR |= SWT_IR_TIF_MASK;

            /* Sets timeout value */
            base->TO = userConfigPtr->timeoutValue;

            /* Sets window value */
            if (userConfigPtr->winEnable)
            {
                base->WN = userConfigPtr->windowValue;
            }

            /* Initializes initial service key value */
            if (userConfigPtr->serviceMode == SWT_KS_SEQ_MODE)
            {
                base->SK = SWT_SK_SK(userConfigPtr->initKey);
            }

            /* Enable the SWT */
            base->CR = cr;
        }
        else
        {
            ret = STATUS_ERROR;
        }
    }

    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_GetServiceMode
 * Description   : Gets current service mode.
 *
 *END**************************************************************************/
swt_service_mode_t SWT_GetServiceMode(const SWT_Type * const base)
{
    swt_service_mode_t mode;

    /* Gets current service mode */
    switch ((base->CR & SWT_CR_SMD_MASK) >> SWT_CR_SMD_SHIFT)
    {
    #if FEATURE_SWT_SUPPORT_WATCHPOINT
        /* Incremental Address Execution */
        case 3U:
            mode = SWT_IA_EXE_MODE;
            break;
        /* Fixed Address Execution */
        case 2U:
            mode = SWT_FA_EXE_MODE;
            break;
    #endif
        /* Keyed Service Sequence */
        case 1U:
            mode = SWT_KS_SEQ_MODE;
            break;
        /* Fixed Service Sequence */
        default:
            mode = SWT_FS_SEQ_MODE;
            break;
    }

    return mode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SWT_WriteReadIntReg
 * Description   : Write and read interrupt register follow mask.
 *
 *END**************************************************************************/
uint32_t SWT_WriteReadIntReg(SWT_Type * const base,
                             uint32_t mask,
                             uint32_t value)
{
    base->IR = (base->IR & ~mask) | value;
    return base->IR;
}

/*******************************************************************************
 * EOF
 *******************************************************************************/
