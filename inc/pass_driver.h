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

#ifndef PASS_DRIVER_H
#define PASS_DRIVER_H

#include "status.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/******************************************************************************
 * Definitions.
 *****************************************************************************/

/*! @brief Definitions for life cycle. Each device has a specific life cycle at a
 * time reflecting the production status.
 * Implements : pass_life_cycle_t_Class
 */
typedef enum
{
    PASS_LC_MCU_PRODUCTION = 0x06,         /* MCU production cycle */
    PASS_LC_CUSTOMER_DELIVERY = 0x03,      /* Custom delivery cycle */
    PASS_LC_OEM_PRODUCTION = 0x02,         /* OEM production cycle */
    PASS_LC_IN_FIELD = 0x07,               /* In Field cycle */
    PASS_LC_FAILURE_ANALYSIS = 0x00        /* Failure analysis cycle */
} pass_life_cycle_t;

/*! @brief Definitions for block types of FLASH memory.
 * These blocks are used to select which block to be locked for write/erase operation.
 * The address and size of a block are depend on the device you are working on.
 * Please refer to Reference Manual for more information.
 * Implements : pass_flash_block_t_Class
 */
typedef enum
{
    PASS_FLASH_BLOCK_LOW_MIDDLE,  /* Low and middle blocks */
    PASS_FLASH_BLOCK_HIGH,        /* High blocks */
    PASS_FLASH_BLOCK_L256K,       /* Lower 256KB of Flash */
    PASS_FLASH_BLOCK_U256K        /* Upper 256KB of Flash */
} pass_flash_block_t;

/*! @brief Definitions for regions of FLASH memory.
 * These regions are used to select which region to be locked for read operation.
 * The address and size of a region are depend on the device you are working on.
 * Please refer to Reference Manual for more information.
 * Implements : pass_flash_region_t_Class
 */
typedef enum
{
    PASS_FLASH_REGION_RL0,       /* The definition for region 0 */
    PASS_FLASH_REGION_RL1,       /* The definition for region 1 */
    PASS_FLASH_REGION_RL2,       /* The definition for region 2  */
    PASS_FLASH_REGION_RL3,       /* The definition for region 3 */
    PASS_FLASH_REGION_RL4        /* The definition for region 4 */
} pass_flash_region_t;

/*! @brief Read states for regions.
 * Implements : pass_read_state_t_Class
 */
typedef enum
{
    PASS_READ_STATE_UNLOCKED = 0,        /* A region is readable */
    PASS_READ_STATE_LOCKED = 1           /* A region is not readable */
} pass_read_state_t;

/*************************************************************************************************
 * API
 ************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief This function is used to get the current life cycle of the device.
 *
 * @return    The current life cycle of the device.
 */
pass_life_cycle_t PASS_DRV_GetLifeCycle(void);

/*!
 * @brief This function is used to check if the device is censored or not.
 * Device will be censored when the DCF Censorship client contains any other value than 0x55AA.
 * Once device is censored, user have to provide a JTAG password to be able to access the device from the debugger.
 *
 * @return    true if device is censored
 *            false if device is not censored
 */
bool PASS_DRV_CheckCensorship(void);

/*!
 * @brief This function is used to unlock the accessing to the PASSGROUP registers (LOCK0, LOCK1, LOCK2, LOCK3).
 *  By default, after resetting, the PASSGROUP registers are always locked. So user have to call this function
 *  first to unlock PASSGROUP registers before calling functions: PASS_DRV_SetWriteEraseState, PASS_DRV_SetReadState.
 *  To be able to use this function, user have to provide a 256 bits password. This password will be compared to the
 *  password stored in the UTEST area (PASS password group 0, PASS password group 1, PASS password group 2, PASS password group 3).
 *  If password is matched, this function will return STATUS_SUCCESS. Otherwise, STATUS_ERROR will be returned.
 *
 * @param[in] passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 * @param[in] password    The password to unlock. The length of password must be 8 words (256 bits)
 *
 * @return    STATUS_SUCCESS   if successful;
 *            STATUS_ERROR     if failed
 */
status_t PASS_DRV_UnlockPassgroup(uint8_t passgroup,
                                  const uint32_t * password);

/*!
 * @brief This function is used to check if a given passgroup is unlocked or not.
 *
 * @param[in] passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 *
 * @return    true - passgroup is unlocked, false - passgroup is locked
 */
bool PASS_DRV_PassgroupIsUnlock(uint8_t passgroup);

/*!
 * @brief This function is used to lock or unlock for write and erase operation for the given blocks of Flash memory.
 *  This function can be only used when PASSGROUP registers are unlocked. User can use PASS_DRV_UnlockPassgroup()
 *  function to unlock the PASSGROUP registers.
 *  NOTE: This function only sets the state temporarily. After resetting, the new state will be reloaded from DCF records.
 *
 * @param[in] blockType   This parameter is used to select which memory area to be manipulated.
 * @param[in] passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 * @param[in] newState    Contain the new state of blocks to be set. A value of 1 in a bit of newState signifies that
                          the corresponding block will be locked for program and erase.
 *
 * @return    STATUS_SUCCESS   if successful;
 *            STATUS_ERROR     if failed
 */
status_t PASS_DRV_SetWriteEraseState(pass_flash_block_t blockType,
                                     uint8_t passgroup,
                                     uint32_t newState);

/*!
 * @brief This function is used to lock or unlock for read operation for the given regions of Flash memory.
 *  This function can be only used when PASSGROUP registers are unlocked. User can use PASS_DRV_UnlockPassgroup()
 *  function to unlock the PASSGROUP registers.
 *  NOTE: This function only sets the state temporarily. After resetting, the new state will be reloaded from DCF records.
 *
 * @param[in] region      This parameter is used to select which memory region to be manipulated.
 * @param[in] passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 * @param[in] newState    The new state of region.
 *
 * @return    STATUS_SUCCESS   if successful;
 *            STATUS_ERROR     if failed
 */
status_t PASS_DRV_SetReadState(pass_flash_region_t region,
                               uint8_t passgroup,
                               pass_read_state_t newState);

/*!
 * @brief This function is used to check if the given blocks of Flash memory is writable/erasable or not.
 *
 * @param[in]  blockType   This parameter is used to select which memory area to be manipulated.
 * @param[in]  passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 * @param[out] status      Contain the current state of blocks. A value of 1 in a bit of "status" signifies that
                           the corresponding block is locked for program and erase.
 *
 * @return    STATUS_SUCCESS   if successful;
 *            STATUS_ERROR     if failed
 */
status_t PASS_DRV_GetWriteEraseStatus(pass_flash_block_t blockType,
                                      uint8_t passgroup,
                                      uint32_t * status);

/*!
 * @brief This function is used to check if the given blocks of Flash memory is readable or not.
 *
 * @param[in]  region      This parameter is used to select which memory region to be manipulated.
 * @param[in]  passgroup   Select the which group to be unlocked, valid value is 0, 1, 2, 3
 * @param[out] status      Contain the current state of region. 1 - locked; 0 - unlocked
 *
 * @return    STATUS_SUCCESS   if successful;
 *            STATUS_ERROR     if failed
 */
status_t PASS_DRV_GetReadStatus(pass_flash_region_t region,
                                uint8_t passgroup,
                                pass_read_state_t * status);

#if defined(__cplusplus)
}
#endif

#endif /* PASS_DRIVER_H*/
