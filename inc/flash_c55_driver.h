/*
 * Copyright 2017-2019 NXP
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
 * @file flash_c55_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.3, Global typedef not referenced.
 * flash_callback_t can be used in user code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * This is required to enable the use of a macro needed by
 * the user code (even if the macro is not used inside the flash driver code)
 * or driver code in the future.
 */


#ifndef FLASH_C55_DRIVER_H
#define FLASH_C55_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @defgroup flash_c55_driver FLASH Driver of C55 module
 * @ingroup flash_c55
 * @details This section describes the programming interface of the FLASH driver.
 * @{
 */


/*!
 * @name Size of a programming in byte
 * @{
 */
/*! @brief size of a word in byte */
#define C55_WORD_SIZE               4U
/*! @brief size of a double word in byte */
#define C55_DWORD_SIZE              8U
/*! @brief size of a page in byte */
#define C55_PAGE_SIZE               32U
/*! @brief size of a quad-page in byte */
#define C55_QPAGE_SIZE              128U
/*! @brief Fast program boundary */
#define C55_FAST_PGM_ALIGN          0x1000U
/*@}*/

/*! @brief A delay of about 20us to suspend a program operation */
#define FLASH_C55_PROGRAM_SUSPEND_DELAY  0xA0U
/*! @brief A delay of about 40us to suspend a erase operation */
#define FLASH_C55_ERASE_SUSPEND_DELAY    0x140U
/*! @brief Cycles counter used in FLASH_DRV_Resume about 500ns at 400MHz system clock */
#define FLASH_RESUME_WAIT                15U
/*! @brief Cycles counter used in FLASH_DRV_Suspend */
#define FLASH_SUSPEND_WAIT               450U
/*! @brief Cycles counter used in FLASH_DRV_ArrayIntegritySuspend/Resume */
#define FLASH_USER_TEST_WAIT             450U
/*! @brief Cycles counter used in FLASH_DRV_Abort */
#define FLASH_ABORT_WAIT                 450U

/*! @brief The password in UTest mode which written to the UT0 register */
#define C55_USER_TEST_ENABLE_PASSWORD      0xF9F99999U

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*!
 * @brief The Flash Address Space.
 *
 * This is used to indicators for getting/setting block lock state.
 * Implements : flash_address_space_t_Class
 */
typedef enum
{
    C55_BLOCK_LOW         = 0x00U,   /*!< Low address space */
    C55_BLOCK_MID         = 0x01U,   /*!< Mid address space */
    C55_BLOCK_HIGH        = 0x02U,   /*!< High address space */
    C55_BLOCK_256K_FIRST  = 0x03U,   /*!< The first of 256K address space */
    C55_BLOCK_256K_SECOND = 0x04U,   /*!< The second of 256K address space */
    C55_BLOCK_UTEST       = 0x05U    /*!< UTest address space */
} flash_address_space_t;

/*!
 * @brief Space erase options.
 *
 * This is used to select which flash area to be erased.
 * Implements : flash_erase_option_t_Class
 */
typedef enum
{
    ERS_OPT_MAIN_SPACE        = 0x01U,       /*!< Erase main address space */
    ERS_OPT_UTEST_SPACE       = 0x02U        /*!< Erase UTEST address space */
} flash_erase_option_t;

/*!
 * @brief Declarations for flash suspend operation and resume operation and user test check state.
 *
 * This is used to indicators for suspending state, resuming state and operation is broken by
 * single bit correction or double bit detection.
 * Implements : flash_state_t_Class
 */
typedef enum
{
    C55_OK                           = 0x00U,   /*!< Successful operation */
    C55_SUS_NOTHING                  = 0x10U,   /*!< No program/erase operation */
    C55_PGM_WRITE                    = 0x11U,   /*!< A program sequence in interlock write stage. */
    C55_ERS_WRITE                    = 0x12U,   /*!< An erase sequence in interlock write stage. */
    C55_ERS_SUS_PGM_WRITE            = 0x13U,   /*!< An erase-suspend program sequence in interlock write stage. */
    C55_PGM_SUS                      = 0x14U,   /*!< The program operation is in suspend state */
    C55_ERS_SUS                      = 0x15U,   /*!< The erase operation is in suspend state */
    C55_ERS_SUS_PGM_SUS              = 0x16U,   /*!< The erase-suspended program operation is in suspend state */
    C55_USER_TEST_SUS                = 0x17U,   /*!< The UTest check operation is in suspend state */

    C55_RES_NOTHING                  = 0x20U,   /*!< No suspended program/erase operation */
    C55_RES_PGM                      = 0x21U,   /*!< The program operation is resumed */
    C55_RES_ERS                      = 0x22U,   /*!< The erase operation is resumed */
    C55_RES_ERS_PGM                  = 0x23U,   /*!< The erase-suspended program operation is resumed */
    C55_RES_USER_TEST                = 0x24U,   /*!< The UTest check operation is resumed */

    C55_USER_TEST_BREAK_SBC          = 0x30U,   /*!< The UTest check operation is broken by Single bit correction */
    C55_USER_TEST_BREAK_DBD          = 0x31U    /*!< The UTest check operation is broken by Double bit detection */
} flash_state_t;

/*!
 * @brief Next Array Integrity Break Point options.
 *
 * This is used to select next break point option during array integrity check or user margin read.
 * Implements : flash_ai_break_option_t_Class
 */
typedef enum
{
    C55_AI_BREAK_NONE              = 0x00U,   /*!< No break at all */
    C55_AI_BREAK_ON_DBD            = 0x01U,   /*!< Break on Double bit detection */
    C55_AI_BREAK_ON_DBD_SBC        = 0x02U    /*!< Break on both Double bit detection and Single bit correction */
} flash_ai_break_option_t;

/*!
 * @brief Array Integrity Sequence.
 *
 * This is used to determines the address sequence to be used during array integrity checks
 * Implements : flash_ai_sequence_option_t_Class
 */
typedef enum
{
    C55_AI_ADDR_SEQ_PROPRIETARY    = 0x00U,    /*!< Array integrity sequence is proprietary sequence */
    C55_AI_ADDR_SEQ_LINEAR         = 0x01U     /*!< Array integrity sequence is sequential */
} flash_ai_sequence_option_t;

/*!
 * @brief Declarations of margin levels.
 *
 * This is used to selects the margin level that is being checked.
 * Implements : flash_margin_option_t_Class
 */
typedef enum
{
    C55_MARGIN_LEVEL_PROGRAM    = 0x00U,   /*!< a programmed level */
    C55_MARGIN_LEVEL_ERASE      = 0x01U    /*!< a erased level */
} flash_margin_option_t;

/*******************************************************************************
* Null Callback function definition
*******************************************************************************/
/*!
 * @name Null Callback function definition
 * @{
 */
/*! @brief  Null callback */
#define NULL_CALLBACK      ((flash_callback_t)0xFFFFFFFFU)
/*@}*/

/*******************************************************************************
* Callback function prototype
*******************************************************************************/
/*! @brief Call back function pointer data type
 *
 *   If using callback in the application.
 */
typedef void (* flash_callback_t)(void);

/*!
 * @brief MISR structure.
 * Implements : flash_misr_t_Class
 */
typedef struct
{
    uint32_t arrMISRValue[C55FMC_UM_COUNT + 1U];    /*!< The value of MISR */
} flash_misr_t;

/*!
 * @brief The structure store the context of data. This consists of parameters input are
 * necessary for the APIs.
 * Implements : flash_context_data_t_Class
 */
typedef struct
{
    uint32_t dest;                      /*!< The context destination address of the program operation */
    uint32_t size;                      /*!< The context size of the program operation */
    uint32_t source;                    /*!< The context source of the program operation */
    flash_misr_t * pMisr;               /*!< The context MISR values of the operation in user test mode */
} flash_context_data_t;

/*!
 * @brief Block select structure for low, mid, high and 256K address space.
 * Implements : flash_block_select_t_Class
 */
typedef struct
{
    uint32_t lowBlockSelect;              /*!< The low block selection */
    uint32_t midBlockSelect;              /*!< The mid block selection */
    uint32_t highBlockSelect;             /*!< The high block selection */
    uint32_t first256KBlockSelect;        /*!< The first of 256K block selection */
    uint32_t second256KBlockSelect;       /*!< The second of 256K block selection */
} flash_block_select_t;


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name FLASH DRIVER API
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Check the program operation
 *
 * This function will check the status of on-going high voltage
 * in program operation.
 *
 * @param[in] pCtxData A data structure for storing context variables.
 * @param[out] opResult The value return the state of flash.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_FLASH_INPROGRESS : In progress status
 *         - STATUS_ERROR : The program operation is unsuccessful
 *         - STATUS_BUSY : Operation busy status
 *         - STATUS_FLASH_ERROR_ENABLE : It is impossible to enable an operation
 */
status_t FLASH_DRV_CheckProgramStatus(flash_context_data_t * pCtxData,
                                      flash_state_t * opResult);

/*!
 * @brief Check the erase operation
 *
 * This function will check the status of on-going high voltage
 * in erase operation.
 *
 * @param[out] opResult The value return the state of flash.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_FLASH_INPROGRESS : In progress status
 *         - STATUS_ERROR : The erase operation is unsuccessful
 */
status_t FLASH_DRV_CheckEraseStatus(flash_state_t * opResult);

/*!
 * @brief Check the operation in user test mode
 *
 * This function will check the status array integrity check
 * in user test mode.
 *
 * @param[in] pCtxData A data structure for storing context variables.
 * @param[out] opResult The value return the state of flash.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_ERROR : Operation failure status
 *         - STATUS_FLASH_INPROGRESS : In progress status
 */
status_t FLASH_DRV_CheckUserTestStatus(const flash_context_data_t * pCtxData,
                                       flash_state_t * opResult);

/*!
 * @brief Initializes the FLASH module
 *
 * This function will initialize flash module and clear all error flags.
 *
 * @return Execution status (success)
 */
status_t FLASH_DRV_Init(void);

/*!
 * @brief Set the block is locked/unlocked state
 *
 * This function will set the block lock state for Low/Middle/High/256K
 * address space on the C55 module to protect them from program or erase.
 *
 * @param[in] lockedBlockSelection Indicating the address space of the block lock register to be read.
 * @param[in] lockedBlockState The block locks be set to the specified address space and protection level.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_ERROR : Operation failure status
 */
status_t FLASH_DRV_SetLock(flash_address_space_t lockedBlockSelection,
                           uint32_t lockedBlockState);

/*!
 * @brief Get the block locking status
 *
 * This function will check the block locking status of
 * Low/Middle/High/256K address spaces in the C55 module.
 *
 * @param[in] lockedBlockSelection Indicating the address space of the block lock register to be read.
 * @param[out] lockedBlockState The block locks be set to the specified address space and protection level.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 */
status_t FLASH_DRV_GetLock(flash_address_space_t lockedBlockSelection,
                           uint32_t * lockedBlockState);

/*!
 * @brief Erase the selected block in the flash memory
 *
 * This function is to do erase operation for multi-blocks on single
 * flash module according to user's input arguments via main interface. The target
 * flash module status will be checked in advance to return relevant error code if any.
 * This function only sets the high voltage without waiting for the operation to be finished.
 * Instead, user must call FLASH_DRV_CheckEraseStatus function to confirm the successful
 * completion of this operation.
 *
 * @param[in] eraseOption The option is to select user�s expected erase operation.
 * @param[in] blockSelect Select the array blocks in low, mid, high and 256K address space for erasing.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_BUSY : Operation busy status
 *         - STATUS_FLASH_ERROR_ENABLE : It is impossible to enable an operation
 */
status_t FLASH_DRV_Erase(flash_erase_option_t eraseOption,
                         flash_block_select_t * blockSelect);

/*!
 * @brief Check the previous erase operation
 *
 * This function is to do blank check for the previous erase operation.
 * It will verify whether the expected flash range is blank or not. In case of mismatch,
 * the failed address and failed destination will be saved and relevant error code will
 * be returned.
 *
 * @param[in] dest Destination address to be checked.
              dest must be aligned with 4 bytes.
 * @param[in] size Size in bytes of the flash region to check.
              size must be aligned with 4 bytes.
 * @param[in] numOfWordCycle The number of word need to check in each cycle.
 * @param[out] pFailedAddress Return the address of the first non-blank flash location in the checking region.
 * @param[in] CallBack The CallBack function to implement the user's application.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_ERROR : Operation failure status
 */
status_t FLASH_DRV_BlankCheck(uint32_t dest,
                              uint32_t size,
                              uint32_t numOfWordCycle,
                              uint32_t * pFailedAddress,
                              void (*CallBack)(void));

/*!
 * @brief Program operation in C55 flash
 *
 * This function is to do program operation for single or
 * multi-programmable size via different interface on targeted flash module according
 * to user's input arguments. The targeted flash module status will be checked
 * in advance to return relevant error code if any.
 * This function only set the high voltage without waiting for the operation
 * to be finished. Instead, user must call FLASH_DRV_CheckProgramStatus function to confirm
 * the successful completion of this operation.
 *
 * @param[in] pCtxData A data structure for storing context variables.
 * @param[in] dest Destination address to be programmed in flash memory.
              dest must be aligned with 8 bytes.
 * @param[in] size Size in bytes of the flash region to be programmed.
              size must be aligned with 4 bytes.
 * @param[in] source Source program buffer address.
              source must be aligned with 4 bytes.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_BUSY : Operation busy status
 *         - STATUS_FLASH_ERROR_ENABLE : It is impossible to enable an operation
 */
status_t FLASH_DRV_Program(flash_context_data_t * pCtxData,
                           uint32_t dest,
                           uint32_t size,
                           uint32_t source);

/*!
 * @brief Verify the previous program operation
 *
 * This function is to verify the previous program operation.
 * It will verify if the programmed flash range matches the corresponding source
 * data buffer. In case of mismatch, the failed address, failed destination and
 * failed source will be saved and relevant error code will be returned.
 *
 * @param[in] dest Destination address to be verified in flash memory.
              dest must be aligned with 4 bytes.
 * @param[in] size Size in byte of the flash region to verify.
              size must be aligned with 4 bytes.
 * @param[in] source Verify source buffer address.
              source must be aligned with 4 bytes.
 * @param[in] numOfWordCycle The number of word need to verify in each cycle.
 * @param[out] pFailedAddress Return the first failing address in flash.
 * @param[in] CallBack The CallBack function to implement the user's application.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 *         - STATUS_ERROR : Operation failure status
 */
status_t FLASH_DRV_ProgramVerify(uint32_t dest,
                                 uint32_t size,
                                 uint32_t source,
                                 uint32_t numOfWordCycle,
                                 uint32_t * pFailedAddress,
                                 void (*CallBack)(void));

/*!
 * @brief Perform a sum of data integrity in the flash memory range
 *
 * This function will perform a 32-bit sum over the specified
 * flash memory range without carry, which provides a rapid method for data
 * integrity checking.
 *
 * @param[in] dest Destination address to be summed in flash memory.
              dest must be be aligned with 4 bytes.
 * @param[in] size Size in bytes of the flash region to check sum.
              size must be aligned with 4 bytes.
 * @param[in] numOfWordCycle The number of word need to calculate sum in each cycle.
 * @param[out] pSum Returns the sum value.
 * @param[in] CallBack The CallBack function to implement the user's application.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful
 */
status_t FLASH_DRV_CheckSum(uint32_t dest,
                            uint32_t size,
                            uint32_t numOfWordCycle,
                            uint32_t * pSum,
                            void (*CallBack)(void));

/*!
 * @brief Suspend a operation is being in progress
 *
 * This function will check if there is any high voltage operation
 * being in progress on the C55 module and if this operation can be suspended.
 * This function will suspend the ongoing operation if it can be suspended.
 *
 * @param[out] suspendState The suspend state of C55 module after the function being called.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_Suspend(flash_state_t * suspendState);

/*!
 * @brief Resume the previous suspend operation
 *
 * This function will check if there is any suspended erase
 * or program operation on the C55 module, and will resume the suspended operation
 * if there is any.
 *
 * @param[out] resumeState The resume state of C55 module after the function being called.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_Resume(flash_state_t * resumeState);

/*!
 * @brief Abort a program or erase operation
 *
 * This function will abort a program or erase operation in user
 * mode and clear all PGM, PSUS, ERS, ESUS, EHV bits in MCR via main interface.
 *
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_Abort(void);

/*!
 * @brief Get the over-program protection status
 *
 * This function will return the over-program protection status
 * via main interface. This value shows blocks that are protected from being
 * over programmed.
 *
 * @param[in] blkProtIndicator The block indicator to get over-program protection status.
 * @param[out] blkProtState The bit map for over-program protection information of specific address space.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_GetOverPgmProtStatus(flash_address_space_t blkProtIndicator,
                                        uint32_t * blkProtState);

/*!
 * @brief Get the failing address in memory
 *
 * This function will get the failing address in the event of ECC
 * event error, Single Bit Correction, as well as providing the address of a
 * failure that may have occurred in a program/erase operation.
 *
 * @return uint32_t Return the address is failed in the event or single bit correction.
 */
uint32_t FLASH_DRV_GetFailedAddress(void);

/*!
 * @brief Check the array integrity of the flash memory
 *
 * This function will check the array integrity of the flash via
 * main interface. The user specified address sequence is used for array integrity
 * reads and the operation is done on the specified blocks. The MISR values calculated
 * by the hardware is compared to the values passed by the user, if they are not
 * the same, then an error code is returned.
 * In order to support asynchronous design, this function will store the necessary
 * information to 'pCtxData' (ex: user provided MISR value) and be terminated without
 * waiting for completion of this operation. User should call �FLASH_DRV_CheckUserTestStatus� to
 * check the on-going status of this function. And once finish, it will do comparison
 * between MISR values provided by user which is currently stored in 'pSSDConfig' and
 * MISR values generated by hardware and return an appropriate code according to
 * this compared result.
 *
 * @param[in] pCtxData A data structure for storing context variables.
 * @param[in] blockSelect Select the array blocks in low, mid, high and 256K address space for checking.
 * @param[in] breakOption Specify an option to allow stopping the operation on errors.
 * @param[in] addrSeq Determine the address sequence to be used during array integrity checks.
 * @param[in/out] pMisrValue Address of a MISR structure contains the MISR values calculated by off-line tool.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 *         - STATUS_BUSY : New operation cannot be performed while previous high voltage operation in progress.
 *         - STATUS_FLASH_ERROR_NO_BLOCK : No blocks have been enabled for Array Integrity check.
 *         - STATUS_FLASH_ERROR_ENABLE :  It's impossible to enable an operation
 */
status_t FLASH_DRV_ArrayIntegrityCheck(flash_context_data_t * pCtxData,
                                       flash_block_select_t * blockSelect,
                                       flash_ai_break_option_t breakOption,
                                       flash_ai_sequence_option_t addrSeq,
                                       flash_misr_t * pMisrValue);

/*!
 * @brief Suspend an on-going array integrity check
 *
 * This function will check if there is an on-going array integrity
 * check of the flash and suspend it via main interface.
 *
 * @param[out] suspendState the suspend state on user test mode after calling the function.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_ArrayIntegritySuspend(flash_state_t * suspendState);

/*!
 * @brief Resume the previous suspend operation
 *
 * This function will check if there is an on-going array integrity
 * check of the flash being suspended and resume it via main interface.
 *
 * @param[out] resumeState The resume state on user�s test mode after calling the function.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 */
status_t FLASH_DRV_ArrayIntegrityResume(flash_state_t * resumeState);

/*!
 * @brief Check the user margin read of the flash memory
 *
 * This function will check the user margin reads of the flash
 * via main interface. The user specified margin level is used for reads and the
 * operation is done on the specified blocks. The MISR values calculated by
 * the hardware are compared to the values passed by the user, if they are not
 * the same, then an error code is returned.
 * In order to support asynchronous design, this function will store the necessary
 * information to 'pCtxData' (ex: user provided MISR value) and be terminated without
 * waiting for completion of this operation. User should call �FLASH_DRV_CheckUserTestStatus�
 * to check the on-going status of this function. And once finish, it will do comparison
 * between MISR values provided by user which are currently stored in 'pSSDConfig'
 * and MISR values generated by hardware and return an appropriate code according to
 * this compared result.
 *
 * @param[in] pCtxData A data structure for storing context variables.
 * @param[in] blockSelect Select the array blocks in low, mid, high and 256K address space for checking.
 * @param[in] breakOption An option to allow stopping the operation on errors.
 * @param[in] marginLevel The margin level to be used during margin read checks.
 * @param[in/out] pMisrValue Address of a MISR structure contains the MISR values calculated by the user.
 * @return Operation status
 *         - STATUS_SUCCESS : The operation is successful.
 *         - STATUS_BUSY : New operation cannot be performed while previous high voltage operation in progress.
 *         - STATUS_FLASH_ERROR_NO_BLOCK : No blocks have been enabled for Array Integrity check.
 *         - STATUS_FLASH_ERROR_ENABLE :  It's impossible to enable an operation
 */
status_t FLASH_DRV_UserMarginReadCheck(flash_context_data_t * pCtxData,
                                       flash_block_select_t * blockSelect,
                                       flash_ai_break_option_t breakOption,
                                       flash_margin_option_t marginLevel,
                                       flash_misr_t * pMisrValue);

/*!
 * @brief Enable the command complete interrupt.
 *
 * This function will enable the command complete interrupt is generated when
 * a program erase complete or suspend program, erase. User must enable the flash
 * interrupt in interrupt controller when uses this function.
 *
 * @return operation status
 *        - STATUS_SUCCESS:         Operation was successful.
 */
status_t FLASH_DRV_EnableCmdCompleteInterupt(void);

/*!
 * @brief Disable the command program, erase, suspend complete interrupt.
 *
 */
void FLASH_DRV_DisableCmdCompleteInterupt(void);

/*! @} */
#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FLASH_C55_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
