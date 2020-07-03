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

#ifndef TDM_DRIVER_H_
#define TDM_DRIVER_H_

#include <stddef.h>
#include <stdbool.h>
#include "status.h"
#include "flash_c55_driver.h"
#include "device_registers.h"

/*!
 * @addtogroup tdm_driver
 * @{
 */
#if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @name TDM Driver
 * @{
 */
/*!
 * @brief Return the locked status of a TDR.
 * @return Return the locked status of a TDR. True means that TDR is locked, false otherwise.
 */
bool TDM_DRV_GetTDRStatus(uint8_t tdrIndex);

/*!
 * @brief Bypass a tamper detection region by loading STO_KEY register with correct value.
 *
 * @param[in] tdrIndex Tamper detection regions' index to be bypassed.
 */
void TDM_DRV_BypassTDRByKey(uint8_t tdrIndex);

/*!
 * @brief Get the write count a tamper detection region has left with the size of diary record provided.
 *
 * @param[in] tdrIndex Tamper detection regions' index.
 * @param[in] size Size of a diary record. This number must be multiple of 8.
 * @return Count number.
 */
uint32_t TDM_DRV_GetDiaryCount(uint8_t tdrIndex, uint32_t size);

/*!
 * @brief Write a diary record for a tamper detection region
 *
 * @param[in] tdrIndex Index of tamper detection region that diary will be written to.
 * @param[in] Data Pointer to diary record to be written.
 * @param[in] size Size of diary record. This number must be multiple of 8.
 * @return Status of diary writting operation
 *         - STATUS_TDM_DIARY_FULL: TDR's diary is full.
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_ERROR : Flash program operation is unsuccessful
 *         - STATUS_BUSY : Flash operation busy status
 *         - STATUS_FLASH_ERROR_ENABLE : It is impossible to enable an operation
 */
status_t TDM_DRV_WriteDiary(uint8_t tdrIndex, uint8_t* Data, uint32_t size);

/*@}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/
#endif /* TDM_DRIVER_H_ */
/******************************************************************************/
/* EOF */
/******************************************************************************/
