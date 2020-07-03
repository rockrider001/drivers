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

#ifndef CSE_DRV_H
#define CSE_DRV_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "status.h"
#include "osif.h"
#include "callbacks.h"
#include "device_registers.h"

/*! @file cse_driver.h */

/* */
/* */
/* */

/*!
 * @addtogroup cse_driver
 * @{
 */

/*!
 * @brief Specify the KeyID to be used to implement the requested cryptographic
 * operation.
 *
 * Implements : cse_key_id_t_Enumeration
 */
typedef enum {
    CSE_SECRET_KEY = 0x0U,
    CSE_MASTER_ECU,
    CSE_BOOT_MAC_KEY,
    CSE_BOOT_MAC,
    CSE_KEY_1,
    CSE_KEY_2,
    CSE_KEY_3,
    CSE_KEY_4,
    CSE_KEY_5,
    CSE_KEY_6,
    CSE_KEY_7,
    CSE_KEY_8,
    CSE_KEY_9,
    CSE_KEY_10,
    CSE_RAM_KEY,
    CSE_KEY_11 = 0x14U,
    CSE_KEY_12,
    CSE_KEY_13,
    CSE_KEY_14,
    CSE_KEY_15,
    CSE_KEY_16,
    CSE_KEY_17,
    CSE_KEY_18,
    CSE_KEY_19,
    CSE_KEY_20,
#if FEATURE_CSE_HAS_AVK
    CSE_AVK_KEY = 0x100U
#endif /* FEATURE_CSE_HAS_AVK */
} cse_key_id_t;

/*!
 * @brief CSE commands which follow the same values as the SHE command definition.
 *
 * Implements : cse_cmd_t_Enumeration
 */
typedef enum {
    CSE_CMD_NONE = 0U,
    CSE_CMD_ENC_ECB,
    CSE_CMD_ENC_CBC,
    CSE_CMD_DEC_ECB,
    CSE_CMD_DEC_CBC,
    CSE_CMD_GENERATE_MAC,
    CSE_CMD_VERIFY_MAC,
    CSE_CMD_LOAD_KEY,
    CSE_CMD_LOAD_PLAIN_KEY,
    CSE_CMD_EXPORT_RAM_KEY,
    CSE_CMD_INIT_RNG,
    CSE_CMD_EXTEND_SEED,
    CSE_CMD_RND,
    CSE_CMD_SECURE_BOOT,
    CSE_CMD_BOOT_FAILURE,
    CSE_CMD_BOOT_OK,
    CSE_CMD_GET_ID,
    CSE_CMD_CANCEL,
    CSE_CMD_DBG_CHAL,
    CSE_CMD_DBG_AUTH,
    CSE_CMD_TRNG_RND,
    CSE_CMD_INIT,
    CSE_CMD_MP_COMPRESS,
#if FEATURE_CSE_FLASHLESS_CONFIG
    CSE_CMD_PUBLISH_KEY_IMG,
#endif /* FEATURE_CSE_FLASHLESS_CONFIG */
#if FEATURE_CSE_HAS_SECURE_RAM
    CSE_CMD_LOAD_SEC_RAM,
    CSE_CMD_OPEN_SEC_RAM,
    CSE_CMD_EXPORT_SEC_RAM
#endif /* FEATURE_CSE_HAS_SECURE_RAM */
} cse_cmd_t;

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *       modified by users. Also, contents of the structure are subject to change in
 *       future releases.
 *
 * Implements : cse_state_t_Class
 */
typedef struct {
    bool cmdInProgress;           /*!< Specifies if a command is in progress */
    bool blockingCmd;             /*!< Specifies if a command is blocking or asynchronous */
    cse_cmd_t cmd;                /*!< Specifies the type of the command in execution */
    security_callback_t callback; /*!< The callback invoked when a command is complete */
    void *callbackParam;          /*!< User parameter for the command completion callback */
    semaphore_t cmdComplete;      /*!< Synchronization object for synchronous operation */
    status_t cmdStatus;           /*!< Error code for the last command */
    bool rngInit;                 /*!< Specifies if the internal RNG state is initialized */
    bool *macVerifStatus;         /*!< Specifies the result of the last executed MAC verification command */
} cse_state_t;


/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the internal state of the driver and enables the CSE interrupt.
 *
 * @param[in] state Pointer to the state structure which will be used for holding
 * the internal state of the driver.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_Init(cse_state_t *state);

/*!
 * @brief Clears the internal state of the driver and disables the CSE interrupt.
 *
 * @return STATUS_SUCCESS.
 */
status_t CSE_DRV_Deinit(void);

/*!
 * @brief Installs a user callback for the command complete event.
 *
 * This function installs a user callback for the command complete event.
 *
 * @return Pointer to the previous callback.
 */
security_callback_t CSE_DRV_InstallCallback(security_callback_t callbackFunction, void * callbackParam);

/*!
 * @brief Performs the AES-128 encryption in ECB mode.
 *
 * This function performs the AES-128 encryption in ECB mode of the input
 * plain text buffer
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_EncryptECB(cse_key_id_t keyId, const uint8_t *plainText,
                            uint32_t length, uint8_t *cipherText, uint32_t timeout);

/*!
 * @brief Performs the AES-128 decryption in ECB mode.
 *
 * This function performs the AES-128 decryption in ECB mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation
 * @param[in] cipherText Pointer to the cipher text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_DecryptECB(cse_key_id_t keyId, const uint8_t *cipherText,
                             uint32_t length, uint8_t *plainText, uint32_t timeout);

/*!
 * @brief Performs the AES-128 encryption in CBC mode.
 *
 * This function performs the AES-128 encryption in CBC mode of the input
 * plaintext buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_EncryptCBC(cse_key_id_t keyId, const uint8_t *plainText, uint32_t length,
                            const uint8_t *iv, uint8_t *cipherText, uint32_t timeout);

/*!
 * @brief Performs the AES-128 decryption in CBC mode.
 *
 * This function performs the AES-128 decryption in CBC mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_DecryptCBC(cse_key_id_t keyId, const uint8_t *cipherText, uint32_t length,
                            const uint8_t* iv, uint8_t *plainText, uint32_t timeout);

/*!
 * @brief Calculates the MAC of a given message using CMAC with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] mac Pointer to the buffer containing the result of the CMAC
 * computation.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_GenerateMAC(cse_key_id_t keyId, const uint8_t *msg,
                             uint64_t msgLen, uint8_t *mac, uint32_t timeout);

/*!
 * @brief Verifies the MAC of a given message using CMAC with AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_VerifyMAC(cse_key_id_t keyId, const uint8_t *msg, uint64_t msgLen,
                           const uint8_t *mac, uint8_t macLen,
                           bool *verifStatus, uint32_t timeout);

/*!
 * @brief Asynchronously performs the AES-128 encryption in ECB mode.
 *
 * This function performs the AES-128 encryption in ECB mode of the input
 * plain text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_EncryptECBAsync(cse_key_id_t keyId, const uint8_t *plainText,
                                 uint32_t length, uint8_t *cipherText);

/*!
 * @brief Asynchronously performs the AES-128 decryption in ECB mode.
 *
 * This function performs the AES-128 decryption in ECB mode of the input
 * cipher text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation
 * @param[in] cipherText Pointer to the cipher text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_DecryptECBAsync(cse_key_id_t keyId, const uint8_t *cipherText,
                                  uint32_t length, uint8_t *plainText);

/*!
 * @brief Asynchronously performs the AES-128 encryption in CBC mode.
 *
 * This function performs the AES-128 encryption in CBC mode of the input
 * plaintext buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 *            @note Should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_EncryptCBCAsync(cse_key_id_t keyId, const uint8_t *plainText,
                                 uint32_t length, const uint8_t *iv, uint8_t *cipherText);

/*!
 * @brief Asynchronously performs the AES-128 decryption in CBC mode.
 *
 * This function performs the AES-128 decryption in CBC mode of the input
 * cipher text buffer, in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_DecryptCBCAsync(cse_key_id_t keyId, const uint8_t *cipherText,
                                 uint32_t length, const uint8_t* iv, uint8_t *plainText);

/*!
 * @brief Asynchronously calculates the MAC of a given message using CMAC with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128,
 * in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] mac Pointer to the buffer containing the result of the CMAC
 * computation.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_GenerateMACAsync(cse_key_id_t keyId, const uint8_t *msg,
                                  uint64_t msgLen, uint8_t *mac);

/*!
 * @brief Asynchronously verifies the MAC of a given message using CMAC with AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128,
 * in an asynchronous manner.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_VerifyMACAsync(cse_key_id_t keyId, const uint8_t *msg, uint64_t msgLen,
                                const uint8_t *mac, uint8_t macLen, bool *verifStatus);

/*!
 * @brief Updates an internal key per the SHE specification.
 *
 * This function updates an internal key per the SHE specification.
 *
 * @param[in] keyId KeyID of the key to be updated.
 * @param[in] m1 Pointer to the 128-bit M1 message containing the UID, Key ID
 * and Authentication Key ID.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] m2 Pointer to the 256-bit M2 message contains the new security
 * flags, counter and the key value all encrypted using a derived key generated
 * from the Authentication Key.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] m3 Pointer to the 128-bit M3 message is a MAC generated over
 * messages M1 and M2.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m4 Pointer to a 256 bits buffer where the computed M4 parameter
 * is stored.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m5 Pointer to a 128 bits buffer where the computed M5 parameter
 * is stored.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_LoadKey(cse_key_id_t keyId, const uint8_t *m1, const uint8_t *m2,
                         const uint8_t *m3, uint8_t *m4, uint8_t *m5, uint32_t timeout);

/*!
 * @brief Updates the RAM key memory slot with a 128-bit plaintext.
 *
 * The function updates the RAM key memory slot with a 128-bit plaintext. The
 * key is loaded without encryption and verification of the key, i.e. the key is
 * handed over in plaintext. A plain key can only be loaded into the RAM_KEY
 * slot.
 *
 * @param[in] plainKey Pointer to the 128-bit buffer containing the key that
 * needs to be copied in RAM_KEY slot.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_LoadPlainKey(const uint8_t *plainKey, uint32_t timeout);

/*!
 * @brief Exports the RAM_KEY into a format compatible with the messages
 * used for LOAD_KEY.
 *
 * @param[out] m1 Pointer to a buffer where the M1 parameter will be exported.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m2 Pointer to a buffer where the M2 parameter will be exported.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m3 Pointer to a buffer where the M3 parameter will be exported.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m4 Pointer to a buffer where the M4 parameter will be exported.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] m5 Pointer to a buffer where the M5 parameter will be exported.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_ExportRAMKey(uint8_t *m1, uint8_t *m2, uint8_t *m3,
                              uint8_t *m4, uint8_t *m5, uint32_t timeout);

/*!
 * @brief Initializes the seed for the PRNG.
 *
 * The function must be called before CMD_RND after every power cycle/reset.
 *
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_InitRNG(uint32_t timeout);

/*!
 * @brief Extends the seed of the PRNG.
 *
 * Extends the seed of the PRNG by compressing the former seed value and the
 * supplied entropy into a new seed. This new seed is then to be used to
 * generate a random number by invoking the CMD_RND command. The random number
 * generator must be initialized by CMD_INIT_RNG before the seed may be
 * extended.
 *
 * @param[in] entropy Pointer to a 128-bit buffer containing the entropy.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_ExtendSeed(const uint8_t *entropy, uint32_t timeout);

/*!
 * @brief Generates a vector of 128 random bits.
 *
 * The function returns a vector of 128 random bits. The random number generator
 * has to be initialized by calling CSE_DRV_InitRNG before random numbers can
 * be supplied.
 *
 * @param[out] rnd Pointer to a 128-bit buffer where the generated random number
 * has to be stored.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_GenerateRND(uint8_t *rnd, uint32_t timeout);

/*!
 * @brief Returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * This function returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * @param[in] challenge Pointer to the 128-bit buffer containing Challenge data.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[out] uid Pointer to 128 bit buffer where the UID will be stored. The 8
 * least significant bits will be set to 0 by CSE.
 * @param[out] sreg Value of the status register.
 * @param[out] mac Pointer to the 128 bit buffer where the MAC generated over
 * challenge and UID and status  will be stored.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_GetID(const uint8_t *challenge, uint8_t *uid,
                       uint8_t *sreg, uint8_t *mac, uint32_t timeout);

/*!
 * @brief Generates a vector of 128 random bits using TRNG.
 *
 * The function returns a vector of 128 true random bits, using the TRNG.
 *
 * @param[out] trnd Pointer to a 128-bit buffer where the generated random number
 * has to be stored.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_GenerateTRND(uint8_t *trnd, uint32_t timeout);

/*!
 * @brief Executes the SHE secure boot protocol.
 *
 * The function loads the command processor firmware and memory slot data from
 * the CSE Flash blocks, and then it executes the SHE secure boot protocol.
 *
 * @param[in] bootImageSize Boot image size (in bytes).
 * @param[in] bootImagePtr Boot image start address.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_SecureBoot(uint32_t bootImageSize, const uint8_t *bootImagePtr,
                            uint32_t timeout);

/*!
 * @brief Signals a failure detected during later stages of the boot process.
 *
 * The function is called during later stages of the boot process to detect a
 * failure.
 *
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_BootFailure(uint32_t timeout);

/*!
 * @brief Marks a successful boot verification during later stages of the boot
 * process.
 *
 * The function is called during later stages of the boot process to mark
 * successful boot verification.
 *
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_BootOK(uint32_t timeout);

/*!
 * @brief Obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * This function obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * @param[out] challenge Pointer to the 128-bit buffer where the challenge data
 * will be stored.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_DbgChal(uint8_t *challenge, uint32_t timeout);

/*!
 * @brief Erases all user keys and enables internal debugging if the
 * authorization is confirmed by CSE.
 *
 * This function erases all user keys and enables internal debugging if the
 * authorization is confirmed by CSE.
 *
 * @param[in] authorization Pointer to the 128-bit buffer containing the
 * authorization value.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution.
 */
status_t CSE_DRV_DbgAuth(const uint8_t *authorization, uint32_t timeout);

/*!
 * @brief Compresses the given messages using the Miyaguchi-Preneel
 * compression algorithm implemented in software.
 *
 * This function is a software implementation of Miyaguchi-Preneel compression,
 * running on the host. It is defined mainly for obtaining M1->M5 for secure
 * key update.
 *
 * @param[in] msg Pointer to the message to be compressed. Message will be
 * automatically padded as per SHE specification if it does not already meet the
 * full 128-bit block size requirement.
 *            @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] msgLen The number of bits to be compressed (maximum 4GB).
 * @param[out] mpCompress Pointer to the 128 bit buffer storing the compressed
 * data.
 *             @note Address passed in this parameter must be 32 bit aligned.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_MPCompress(const uint8_t *msg, uint64_t msgLen,
                            uint8_t *mpCompress, uint32_t timeout);

/*!
 * @brief Checks the status of the execution of an asynchronous command.
 *
 * This function checks the status of the execution of an asynchronous command.
 * If the command is still in progress, returns STATUS_BUSY.
 *
 * @return Error Code after command execution.
 */
status_t CSE_DRV_GetAsyncCmdStatus(void);

/*!
 * @brief Cancels a previously initiated command.
 *
 * This function cancels any on-going CSE command.
 *
 * @return STATUS_SUCCESS.
 */
status_t CSE_DRV_CancelCommand(void);

#if FEATURE_CSE_FLASHLESS_CONFIG

/*!
 * @brief Sets the Key Image Address registers.
 *
 * In the flash-less configuration, before either the SECURE_BOOT or INIT_CSE
 * commands are issued the KIAn registers must be initialized with addresses of
 * the latest key images. Both the CSE_KAI0 and CSE_KAI1 addresses are checked
 * during the SECURE_BOOT and CSE_INIT commands for the latest key image. Two KIAn
 * registers are provided to aid in the recovery from an unexpected reset or power
 * down during storage of the key image in non-volatile memory. The KIAn registers
 * must point to valid memory and may be programmed to the same value. These
 * registers are not used in the secure flash configuration.
 *
 * @note In the flash-less configuration, this function must be called prior to the
 * driver initialization.
 *
 * @param[in] kia0 First key image address.
 * @param[in] kia1 Second key image address.
 *
 * @return STATUS_SUCCESS.
 */
status_t CSE_DRV_SetKeyImageAddress(uint32_t kia0, uint32_t kia1);

/*!
 * @brief Generates a 1024 byte key image block and writes it to the
 * location specified by the Key Image Address parameter.
 *
 * In the flash-less configuration this command should be issued after one or
 * more KEY_LOAD commands. If the Secure Counter is incremented and the BOOT_MAC
 * value is preset at boot time, the PUBLISH_KEY_IMG command can only be executed
 * if CSE3_SR[BOK]=1, CSE3_SR[BFN]=1 and CSE3_SR[EDB]=0. The key image block
 * should be stored in non-volatile memory before updating the OTP Secure Counter.
 *
 * @note The PRNG must be initialized before issuing the PUBLISH_KEY_IMG command.
 *
 * @param[in] firmwareAddr The start address of the CSE firmware image provided
 * by NXP.
 * @param[in] incSecCounter If true, increase the Secure Counter.
 * @param[out] kia Location where the key image block will be written.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 *
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_PublishKeyImage(uint32_t firmwareAddr, bool incSecCounter, uint32_t *kia,
                                 uint32_t timeout);

#endif /* FEATURE_CSE_FLASHLESS_CONFIG */

#if FEATURE_CSE_HAS_SECURE_RAM

/*!
 * @brief Gets the address and size of the Secure RAM.
 *
 * @param[out] addr Secure RAM address.
 * @param[out] size Secure RAM size.
 *
 * @return STATUS_SUCCESS.
 */
status_t CSE_DRV_GetSecureRAMInfo(uint32_t *addr, uint32_t *size);

/*!
 * @brief Initializes the secure RAM and upon successful decryption and
 * authentication of the initialization data puts the secure RAM in the
 * secure state.
 *
 * The key selected by the Key ID must have the SEC_RAM security flag set
 * and its start address attribute must match the secure RAM start address.
 * The ciphertext address parameter points to the initialization data
 * encrypted in CBC mode using the specified 128-bit IV. The size of the
 * ciphertext block matches the size of the secure RAM. The 128-bit CMAC
 * input value must match the calculated CMAC over the ciphertext block
 * and IV.
 *
 * @param[in] keyId Key used for deriving the encryption and authentication
 * keys used for Secure RAM.
 * @param[in] cipherText Ciphertext used for initializing the Secure RAM.
 * @param[in] iv Initialization Vector used for decryption of the data.
 * @param[in] mac CMAC used for verification of the ciphertext block and IV.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 *
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_LoadSecureRAM(cse_key_id_t keyId, const uint8_t *cipherText,
                               const uint8_t* iv, const uint8_t *mac, uint32_t timeout);

/*!
 * @brief Resets to zero the secure RAM and puts the secure RAM in the open state.
 *
 * This command can not be issued when the secure RAM is in the secure state.
 * The secure RAM start address input parameter must match the secure RAM
 * start address.
 *
 * @param[in] addr Secure RAM address.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 *
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_OpenSecureRAM(uint32_t addr, uint32_t timeout);

/*!
 * @brief Generates a ciphertext block, 128-bit IV and a 128-bit CMAC output
 * to store the contents of the secure RAM in system memory.
 *
 * The ciphertext block generated is the size of the secure RAM. Upon successful
 * completion of the command the secure RAM is put into the secure state. The key
 * selected by the Key ID must have the SEC_RAM security flag set and its start
 * address attribute must match the secure RAM start address. If the secure RAM
 * is in the secure state, the same key used for the LOAD_SEC_RAM command must be
 * used for the EXPORT_SEC_RAM command. The PRNG must be initialized before issuing
 * the EXPORT_SEC_RAM command.
 *
 * @param[in] keyId Key used for deriving the encryption and authentication
 * keys used for Secure RAM.
 * @param[out] cipherText Ciphertext block representing the contents of the Secure
 * RAM.
 * @param[out] iv Initialization Vector used for decryption of the data.
 * @param[out] mac CMAC used for verification of the ciphertext block and IV.
 * @param[in] timeout Timeout in ms; the function returns STATUS_TIMEOUT if the
 * command is not finished in the allocated period.
 *
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is STATUS_SUCCESS.
 */
status_t CSE_DRV_ExportSecureRAM(cse_key_id_t keyId, const uint8_t *cipherText,
                                 const uint8_t* iv, const uint8_t *mac, uint32_t timeout);

#endif /* FEATURE_CSE_HAS_SECURE_RAM */


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* CSE_DRV_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
