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

#ifndef CSE_HW_ACCESS_H
#define CSE_HW_ACCESS_H

#include "cse_driver.h"
#include "device_registers.h"
#include "interrupt_manager.h"

/*!
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros defines register access masks for CSE registers, not covered by
 * the platform header file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * Function-like macros are used to extract key bank and key ID from the key
 * index; also used for converting register values to error code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required for passing buffer addresses to CSE firmware; the CSE firmware
 * command parameters are defined as unsigned integers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long.
 * The cast is required for passing buffer addresses to CSE firmware; the CSE firmware
 * command parameters are defined as unsigned integers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address (base address of the module).
 */

/*! @file cse_hw_access.h */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Macro used for unused command parameters */
#define CSE_UNUSED_PARAM          0U

/*! @brief Macro used to enable/disable CSE interrupt request */
#define CSE_IRQ_NUMBER            CSE_IRQ_IRQn

/*! @brief Macro used for checking buffer length */
#define CSE_BUFF_LEN_CHECK_MASK   0x0FU

/*! @brief Macro used for checking buffer address */
#define CSE_BUFF_ADDR_CHECK_MASK   0x03U

/*! @brief Macro used to convert buffer length in bytes to number of 128-bits blocks */
#define CSE_BUFF_BLOCK_COUNT(x)   ((x) >> 4U)

/*! @brief Macro used to compute size in bits for a number of 128-bits blocks */
#define CSE_BLOCK_BITS_COUNT(x)    ((x) << 7U)

/*! @brief Security error codes offset in status.h */
#define CSE_ERC_OFFSET            0x400U

/*! @brief Macro used to convert CSE error code (CSE2HTS register) to SDK error code */
#define CSE_CONVERT_ERC(x)        ((status_t)((x) | CSE_ERC_OFFSET))

/*! @brief Mask for stripping the key id from 'cse_key_id_t' */
#define CSE_KEY_ID_MASK           0x0FU

/*! @brief Shift value for retrieving KBS bit from 'cse_key_id_t' */
#define CSE_KEY_ID_SHIFT          0x04U

/*! @brief Macro used for retrieving key id to be passed as command parameter */
#define CSE_CMD_KEY_ID(x)         (((uint32_t)(x)) & CSE_KEY_ID_MASK)

/*! @brief Macro used for retrieving KBS bit to update command structure */
#define CSE_CMD_KBS(x)            ((((uint32_t)(x)) & ~CSE_KEY_ID_MASK) >> CSE_KEY_ID_SHIFT)

/*! @brief Mask of the key bank select bit inside the 32 bits command */
#define CSE_CMD_KBS_MASK          0x80000000U

/*! @brief Maximum length for the MAC message */
#define CSE_MAC_MAX_MSG_LEN       0x800000000U

/*! @brief Timeout for the INIT_CSE command */
#define CSE_INIT_TIMEOUT          10U


/*******************************************************************************
 * CODE
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Prepares the CSE command structure.
 *
 * This function writes the command structure with the appropriate command and
 * parameters.
 *
 * @param[in] param1 Command parameter 1.
 * @param[in] param2 Command parameter 2.
 * @param[in] param3 Command parameter 3.
 * @param[in] param4 Command parameter 4.
 * @param[in] param5 Command parameter 5.
 */
void CSE_PrepareCommand(uint32_t param1,
                        uint32_t param2,
                        uint32_t param3,
                        uint32_t param4,
                        uint32_t param5);

/*!
 * @brief Prepares the CSE ECB encrypt command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * @param[out] cipherText Pointer to the cipher text buffer.
 */
void CSE_PrepareEncryptEcbCmd(cse_key_id_t keyId, const uint8_t *plainText,
                              uint32_t length, uint8_t *cipherText);

/*!
 * @brief Prepares the CSE ECB decrypt command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * @param[out] plainText Pointer to the plain text buffer.
 */
void CSE_PrepareDecryptEcbCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                              uint32_t length, uint8_t *plainText);

/*!
 * @brief Prepares the CSE CBC encrypt command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] cipherText Pointer to the cipher text buffer.
 */
void CSE_PrepareEncryptCbcCmd(cse_key_id_t keyId, const uint8_t *plainText,
                              uint32_t length, const uint8_t *iv, uint8_t *cipherText);

/*!
 * @brief Prepares the CSE CBC decrypt command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] plainText Text Pointer to the cipher text buffer.
 */
void CSE_PrepareDecryptCbcCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                              uint32_t length, const uint8_t *iv, uint8_t *plainText);

/*!
 * @brief Prepares the CSE generate MAC command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] mac Pointer to the buffer containing the result of the CMAC
 * computation.
 */
void CSE_PrepareGenerateMacCmd(cse_key_id_t keyId, const uint8_t *msg,
                               uint64_t msgLen, uint8_t *mac);

/*!
 * @brief Prepares the CSE verify MAC command.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @param[in] macLen Number of bits of the CMAC to be compared.
 */
void CSE_PrepareVerifyMacCmd(cse_key_id_t keyId, const uint8_t *msg,
                             uint64_t msgLen, const uint8_t *mac, uint8_t macLen);

/*!
 * @brief Prepares the CSE load key command.
 *
 * @param[in] keyId Key ID used to perform the cryptographic operation.
 * @param[in] m1 Pointer to the buffer containing m1.
 * @param[in] m2 Pointer to the buffer containing m2.
 * @param[in] m3 Pointer to the buffer containing m3.
 * @param[out] m4 Pointer to the buffer containing m4.
 * @param[out] m5 Pointer to the buffer containing m5.
 */
void CSE_PrepareLoadKeyCmd(cse_key_id_t keyId, const uint8_t *m1, const uint8_t *m2,
                           const uint8_t *m3, uint8_t *m4, uint8_t *m5);

/*!
 * @brief Prepares the CSE load plain key command.
 *
 * @param[in] plainKey Pointer to the 128-bit buffer containing the key
 */
static inline void CSE_PrepareLoadPlainKeyCmd(const uint8_t *plainKey)
{
    CSE_PrepareCommand((uint32_t)plainKey, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE export RAM key command.
 *
 * @param[out] m1 Pointer to the buffer containing m1.
 * @param[out] m2 Pointer to the buffer containing m2.
 * @param[out] m3 Pointer to the buffer containing m3.
 * @param[out] m4 Pointer to the buffer containing m4.
 * @param[out] m5 Pointer to the buffer containing m5.
 */
static inline void CSE_PrepareExportRamKeyCmd(uint8_t *m1, uint8_t *m2, uint8_t *m3,
                                              uint8_t *m4, uint8_t *m5)
{
    CSE_PrepareCommand((uint32_t)m1, (uint32_t)m2,
                       (uint32_t)m3, (uint32_t)m4, (uint32_t)m5);
}

/*!
 * @brief Prepares the CSE init RNG command.
 */
static inline void CSE_PrepareInitRngCmd(void)
{
    CSE_PrepareCommand(CSE_UNUSED_PARAM, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE extend PRNG seed command.
 *
 * @param[in] entropy Pointer to the 128-bit buffer containing the entropy.
 */
static inline void CSE_PrepareExtendPrngSeedCmd(const uint8_t *entropy)
{
    CSE_PrepareCommand((uint32_t)entropy, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE generate random number command.
 *
 * @param[out] rnd Pointer to the 128-bit buffer containing the random value.
 */
static inline void CSE_PrepareGenerateRndCmd(uint8_t *rnd)
{
    CSE_PrepareCommand((uint32_t)rnd, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE get ID command.
 *
 * @param[in] challenge Pointer to the 128-bit buffer containing Challenge data.
 * @param[out] uid Pointer to 120 bit buffer where the UID will be stored.
 * @param[out] mac Pointer to the 128 bit buffer where the MAC generated over
 * challenge and UID and status  will be stored.
 */
static inline void CSE_PrepareGetIdCmd(const uint8_t *challenge, uint8_t *uid, uint8_t *mac)
{
    CSE_PrepareCommand((uint32_t)challenge, (uint32_t)uid,
                       CSE_UNUSED_PARAM, (uint32_t)mac, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE generate true random number command.
 *
 * @param[out] trnd Pointer to the 128-bit buffer containing the random value.
 */
static inline void CSE_PrepareGenerateTrndCmd(uint8_t *trnd)
{
    CSE_PrepareCommand((uint32_t)trnd, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE secure boot command.
 */
static inline void CSE_PrepareSecureBootCmd(uint32_t bootImageSize, const uint8_t *bootImagePtr)
{
    CSE_PrepareCommand(bootImageSize, (uint32_t)bootImagePtr,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE boot failure command.
 */
static inline void CSE_PrepareBootFailureCmd(void)
{
    CSE_PrepareCommand(CSE_UNUSED_PARAM, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE boot ok command.
 */
static inline void CSE_PrepareBootOkCmd(void)
{
    CSE_PrepareCommand(CSE_UNUSED_PARAM, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE debug challenge command.
 */
static inline void CSE_PrepareDbgChalCmd(uint8_t *challenge)
{
    CSE_PrepareCommand((uint32_t)challenge, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE debug authorization command.
 */
static inline void CSE_PrepareDbgAuthCmd(const uint8_t *authorization)
{
    CSE_PrepareCommand((uint32_t)authorization, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE MP Compress command.
 */
static inline void CSE_PrepareMPCompressCmd(const uint8_t * msg, uint64_t *msgLenBits,
                                            uint8_t * mpCompress)
{
    CSE_PrepareCommand((uint32_t)msgLenBits, (uint32_t)msg, (uint32_t)mpCompress,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

#if FEATURE_CSE_FLASHLESS_CONFIG

/*!
* @brief Prepares the CSE Publish Key Image command.
*/
static inline void CSE_PreparePublishKeyImgCmd(uint32_t firmwareAddr, uint8_t counter,
                                               uint32_t *kia)
{
   CSE_PrepareCommand(firmwareAddr, (uint32_t)counter, (uint32_t)kia,
                      CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

#endif /* FEATURE_CSE_FLASHLESS_CONFIG */

#if FEATURE_CSE_HAS_SECURE_RAM

/*!
 * @brief Prepares the CSE load secure RAM command.
 *
 */
static inline void CSE_PrepareLoadSecRAMCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                                            const uint8_t* iv, const uint8_t *mac)
{
    CSE_PrepareCommand((uint32_t)keyId, (uint32_t)cipherText, (uint32_t)iv,
                       (uint32_t)mac, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE open secure RAM command.
 *
 */
static inline void CSE_PrepareOpenSecRAMCmd(uint32_t addr)
{
    CSE_PrepareCommand(addr, CSE_UNUSED_PARAM,
                       CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);
}

/*!
 * @brief Prepares the CSE export secure RAM command.
 *
 */
static inline void CSE_PrepareExportSecRAMCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                                              const uint8_t* iv, const uint8_t *mac)
{
    CSE_PrepareCommand((uint32_t)keyId, (uint32_t)cipherText, (uint32_t)iv,
                       (uint32_t)mac, CSE_UNUSED_PARAM);
}

#endif /* FEATURE_CSE_HAS_SECURE_RAM */

/*!
 * @brief Sends the command to CSE.
 */
static inline void CSE_SendCmd(uint32_t cmd)
{
    CSE->CMD = cmd;
}

/*!
 * @brief Returns the result of the last MAC verification (true - successful verification)
 */
bool CSE_GetMacVerifResult(void);

/*!
 * @brief Returns the SR result of the last Get ID command
 */
static inline uint8_t CSE_GetUIDSregResult(void)
{
    return (uint8_t)(CSE->P[2] & 0xFFUL);
}

/*!
 * @brief Retrieves the CSE error code.
 */
static inline uint32_t CSE_GetErrCode(void)
{
    return (uint32_t)CSE->ECR;
}

/*!
 * @brief Enables/Disables CSE interrupt.
 *
 * @param[in] enable Enables/Disables CSE interrupt.
 */
static inline void CSE_SetInterrupt(bool enable)
{
    if (enable)
    {
        CSE->CR |= CSE_CR_CIE_MASK;
    }
    else
    {
        CSE->CR &= (~CSE_CR_CIE_MASK);
    }
}

/*!
 * @brief Clears the CSE command complete interrupt flag.
 */
static inline void CSE_ClearIntFlag(void)
{
    CSE->IR = CSE_IR_CIF_MASK;
}

/*!
 * @brief Returns true if CSE is busy processing a command
 * and false if the command has completed.
 */
static inline bool CSE_IsBusy(void)
{
    return ((CSE->SR & CSE_SR_BSY_MASK) != 0U);
}


#if defined(__cplusplus)
}
#endif

#endif /* CSE_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
