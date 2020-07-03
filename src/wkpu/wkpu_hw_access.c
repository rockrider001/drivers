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
/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * The function is defined for use by application code.
 */

#include "wkpu_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef FEATURE_WKPU_SUPPORT_INTERRUPT_REQUEST
/*FUNCTION**********************************************************************
 *
 * Function Name : WKPU_EnableInterrupt
 * Description   : This function enable interrupt and wake-up for channel mask.
 *
 *END**************************************************************************/
void WKPU_EnableInterrupt(WKPU_Type * const base,
                          uint32_t channelMask,
                          bool enable)
{
    /* Enable interrupt request */
    WKPU_EnableInterruptRequest(base, channelMask, enable);

    /* Enable wake-up request */
    WKPU_EnableWakeupRequest(base, channelMask, enable);

    /* Clear ISR flag */
    WKPU_ClearInterruptFlag(base, channelMask);
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
