/*
 * Copyright 2018-2019 NXP
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
 * @file mcan_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, Function not defined with external linkage.
 * The functions are not defined static because they are referenced in .s startup files.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as CAN instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, A conversion should not be
 * performed between a pointer to object and an integer type.
 * The cast is required as CAN instance base addresses are defined as unsigned
 * integers in the header file, but the registers are accessed via pointers to
 * structures.
 *
 */
 
#include "mcan_irq.h"
#include "mcan_hw_access.h"
 
#if (M_CAN_INSTANCE_COUNT > 0U)
/* Implementation of MCAN0 IRQ handler for interrupts */
void MCAN_IRQHandler(void)
{
    uint8_t i, flag;

    if ((SIU->HLT2 & SIU_HLT2_MCANA_MASK) == 0U)
    {
        if ((M_CAN_0->ILE & M_CAN_ILE_EINT0_MASK) == M_CAN_ILE_EINT0_MASK)
        {
            for(i=0U;i<32U;i++)
            {
                flag = MCAN_GetInterruptLine(M_CAN_0, i);
                if (flag == 0U)
                {
                    MCAN_InterrupHandler(flag);
                }
            }
        }
    }
    if ((SIU->HLT2 & SIU_HLT2_MCANB_MASK) == 0U)
    {
        if ((M_CAN_1->ILE & M_CAN_ILE_EINT1_MASK) == M_CAN_ILE_EINT1_MASK)
        {
            for(i=0U;i<32U;i++)
            {
                flag = MCAN_GetInterruptLine(M_CAN_1, i);
                if (flag == 1U)
                {
                    MCAN_InterrupHandler(flag);
                }
            }
        }
    }
}

#endif
