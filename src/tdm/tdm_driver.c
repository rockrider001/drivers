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
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * Local variable is used in function to save memory.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External function could be made static
 * These functions are used by user.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 * Required to cast an address to pointer
 */

#include "tdm_driver.h"

#define STO_KEY_VALUE 0x55AA5A5AUL
#define TDR_SIZE 2048UL
#define MIN_WRITE_SIZE 8UL
#define EMPTY_CHECK_SIZE 4UL

/*FUNCTION**********************************************************************
 *
 * Function Name : TDM_DRV_GetTDRStatus
 * Description   : Get a TDR status
 * Implements    : TDM_DRV_GetTDRStatus_Activity
 *
 *END**************************************************************************/
bool TDM_DRV_GetTDRStatus(uint8_t tdrIndex)
{
    DEV_ASSERT(tdrIndex < TDR_COUNT);
    return (((TDM->TDRSR >> (uint32_t)tdrIndex) & 1UL) == 0UL) ? false : true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TDM_DRV_BypassTDRByKey
 * Description   : Bypass TDR by load key to sto_key register
 * Implements    : TDM_DRV_BypassTDRByKey_Activity
 *
 *END**************************************************************************/
void TDM_DRV_BypassTDRByKey(uint8_t tdrIndex)
{
    DEV_ASSERT(tdrIndex < TDR_COUNT);
    TDM->STO_KEY[tdrIndex] = STO_KEY_VALUE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TDM_DRV_WriteDiary
 * Description   : Search for the first empty flash region with specific size
 * Implements    : TDM_DRV_WriteDiary_Activity
 *
 *END**************************************************************************/
status_t TDM_DRV_WriteDiary(uint8_t tdrIndex, uint8_t* Data, uint32_t size)
{
    DEV_ASSERT(tdrIndex < TDR_COUNT);
    DEV_ASSERT(Data != NULL);
    DEV_ASSERT((size >= MIN_WRITE_SIZE) && (size <= TDR_SIZE));
    DEV_ASSERT(TDM->DBA != 0UL);
    DEV_ASSERT((size % EMPTY_CHECK_SIZE) == 0UL);

    uint32_t addr = TDM->DBA + (tdrIndex * TDR_SIZE);
    uint32_t stop_addr = addr + TDR_SIZE;
    uint32_t i;
    bool empty = false;
    status_t ret = STATUS_SUCCESS;
    flash_context_data_t pCtxData;
    flash_state_t opResult;

    while ((addr + size) < stop_addr)
    {
        empty = true;
        for (i = 0; i < size; i += EMPTY_CHECK_SIZE)
        {
            if (*(uint32_t*)(addr+i) != 0xffffffffUL)
            {
                empty = false;
                break;
            }
        }
        if (empty)
        {
            break;
        }
        else
        {
            addr += size;
        }
    }
    if (!empty)
    {
        ret = STATUS_TDM_DIARY_FULL;
    }
    else
    {
        ret = FLASH_DRV_Program(&pCtxData,addr,size,(uint32_t)Data);
        if (STATUS_SUCCESS == ret)
        {
            do
            {
                ret = FLASH_DRV_CheckProgramStatus(&pCtxData, &opResult);
            }while(ret == STATUS_FLASH_INPROGRESS);
        }
    }
    return ret;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TDM_DRV_GetDiaryCount
 * Description   : Count how many empty flash memory with specified size left
 * Implements    : TDM_DRV_GetDiaryCount_Activity
 *
 *END**************************************************************************/
uint32_t TDM_DRV_GetDiaryCount(uint8_t tdrIndex, uint32_t size)
{
    DEV_ASSERT(tdrIndex < TDR_COUNT);
    DEV_ASSERT((size >= MIN_WRITE_SIZE) && (size <= TDR_SIZE));
    DEV_ASSERT(TDM->DBA != 0UL);
    DEV_ASSERT((size % EMPTY_CHECK_SIZE) == 0UL);

    uint32_t addr = TDM->DBA + (tdrIndex * TDR_SIZE);
    uint32_t stop_addr = addr + TDR_SIZE;
    uint32_t i;
    uint32_t c = 0;
    bool empty;
    while ((addr + size) < stop_addr)
    {
        empty = true;
        for (i = 0; i < size; i += EMPTY_CHECK_SIZE)
        {
            if (*(uint32_t*)(addr+i) != 0xffffffffUL)
            {
                empty = false;
                break;
            }
        }
        if (empty)
        {
            c += 1UL;
        }
        addr += size;
    }
    return c;
}
/******************************************************************************/
/* EOF */
/******************************************************************************/
