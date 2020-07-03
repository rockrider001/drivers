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
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. Also, the called functions
 * do not store the address into variables with lifetime longer then its own call.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * These are symbols weak symbols defined in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower or different
 * essential type.
 * This is required by the conversion of a bit-field of a register into enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from
 * 'essentially unsigned' to 'essentially enum<i>'.
 * All possible values are covered by the enumeration, direct casting is used to optimize code.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite
 * expression (different essential type categories).
 * This is required by the conversion of a bit-field of a register into enum type.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required for checking buffer address alignment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from pointer to unsigned long.
 * The cast is required for checking buffer address alignment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 */

#include "cse_hw_access.h"


/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to runtime state structure.*/
static cse_state_t * s_cseStatePtr = NULL;


/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/* Waits on the synchronization object and updates the internal flags */
static void CSE_DRV_WaitCommandCompletion(uint32_t timeout);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_Init
 * Description   : This function initializes the internal state of the driver
 * and enables the CSE interrupt.
 *
 * Implements    : CSE_DRV_Init_Activity
 * END**************************************************************************/
status_t CSE_DRV_Init(cse_state_t * state)
{
    /* Check the driver is initialized */
    DEV_ASSERT(state != NULL);

    status_t semaStatus;

    /* Save the driver state structure */
    s_cseStatePtr = state;

    /* Clear the contents of the state structure */
    s_cseStatePtr->cmdInProgress = false;
    s_cseStatePtr->blockingCmd = false;
    s_cseStatePtr->callback = NULL;
    s_cseStatePtr->callbackParam = NULL;
    s_cseStatePtr->cmd = CSE_CMD_NONE;
    s_cseStatePtr->cmdStatus = STATUS_SUCCESS;
    s_cseStatePtr->rngInit = false;
    s_cseStatePtr->macVerifStatus = NULL;

    /* Create the synchronization semaphore */
    semaStatus = OSIF_SemaCreate(&s_cseStatePtr->cmdComplete, 0U);
    if (semaStatus == STATUS_ERROR)
    {
        return STATUS_ERROR;
    }

    /* Enable CSE IRQ */
    CSE_SetInterrupt(true);
    INT_SYS_EnableIRQ(CSE_IRQ_NUMBER);

    /* INIT_CSE must be issued before any other command when secure boot is not enabled */
#if (FEATURE_CSE_SECURE_BOOT_ENABLED == 0U)
    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;

    /* Prepare the command */
    CSE_PrepareCommand(CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM, CSE_UNUSED_PARAM);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_INIT);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(CSE_INIT_TIMEOUT);
#endif /* FEATURE_CSE_SECURE_BOOT_ENABLED */

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_Deinit
 * Description   : This function clears the internal state of the driver and
 * disables the CSE interrupt.
 *
 * Implements    : CSE_DRV_Deinit_Activity
 * END**************************************************************************/
status_t CSE_DRV_Deinit()
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Clear the contents of the state structure */
    uint8_t *clearStructPtr;
    uint8_t idx;
    clearStructPtr = (uint8_t *)s_cseStatePtr;
    for (idx = 0; idx < sizeof(cse_state_t); idx++)
    {
        clearStructPtr[idx] = 0;
    }

    /* Free the internal state reference */
    s_cseStatePtr = NULL;

    /* Disable CSE IRQ */
    INT_SYS_DisableIRQ(CSE_IRQ_NUMBER);
    CSE_SetInterrupt(false);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_InstallCallback
 * Description   : This function installs a user callback for the command
 * complete event.
 *
 * Implements    : CSE_DRV_InstallCallback_Activity
 * END**************************************************************************/
security_callback_t CSE_DRV_InstallCallback(security_callback_t callbackFunction, void * callbackParam)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    security_callback_t currentCallback = s_cseStatePtr->callback;
    s_cseStatePtr->callback = callbackFunction;
    s_cseStatePtr->callbackParam = callbackParam;

    return currentCallback;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_EncryptECB
 * Description   : This function performs the AES-128 encryption in ECB mode of
 * the input plain text buffer.
 *
 * Implements    : CSE_DRV_EncryptECB_Activity
 * END**************************************************************************/
status_t CSE_DRV_EncryptECB(cse_key_id_t keyId, const uint8_t *plainText,
                            uint32_t length, uint8_t *cipherText, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_EncryptECBAsync(keyId, plainText, length, cipherText);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DecryptECB
 * Description   : This function performs the AES-128 decryption in ECB mode of
 * the input cipher text buffer.
 *
 * Implements    : CSE_DRV_DecryptECB_Activity
 * END**************************************************************************/
status_t CSE_DRV_DecryptECB(cse_key_id_t keyId, const uint8_t *cipherText,
                             uint32_t length, uint8_t *plainText, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_DecryptECBAsync(keyId, cipherText, length, plainText);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_EncryptCBC
 * Description   : This function performs the AES-128 encryption in CBC mode of
 * the input plain text buffer.
 *
 * Implements    : CSE_DRV_EncryptCBC_Activity
 * END**************************************************************************/
status_t CSE_DRV_EncryptCBC(cse_key_id_t keyId, const uint8_t *plainText, uint32_t length,
                            const uint8_t *iv, uint8_t *cipherText, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_EncryptCBCAsync(keyId, plainText, length, iv, cipherText);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DecryptCBC
 * Description   : This function performs the AES-128 decryption in CBC mode of
 * the input cipher text buffer.
 *
 * Implements    : CSE_DRV_DecryptCBC_Activity
 * END**************************************************************************/
status_t CSE_DRV_DecryptCBC(cse_key_id_t keyId, const uint8_t *cipherText, uint32_t length,
                            const uint8_t* iv, uint8_t *plainText, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_DecryptCBCAsync(keyId, cipherText, length, iv, plainText);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GenerateMAC
 * Description   : This function calculates the MAC of a given message using CMAC
 * with AES-128.
 *
 * Implements    : CSE_DRV_GenerateMAC_Activity
 * END**************************************************************************/
status_t CSE_DRV_GenerateMAC(cse_key_id_t keyId, const uint8_t *msg,
                             uint64_t msgLen, uint8_t *mac, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_GenerateMACAsync(keyId, msg, msgLen, mac);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_VerifyMAC
 * Description   : This function verifies the MAC of a given message using CMAC
 * with AES-128.
 *
 * Implements    : CSE_DRV_VerifyMAC_Activity
 * END**************************************************************************/
status_t CSE_DRV_VerifyMAC(cse_key_id_t keyId, const uint8_t *msg, uint64_t msgLen,
                           const uint8_t *mac, uint8_t macLen,
                           bool *verifStatus, uint32_t timeout)
{
    status_t status;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Specify this is a blocking function - returns upon command completion */
    s_cseStatePtr->blockingCmd = true;

    /* Launch the command with the parameters received */
    status = CSE_DRV_VerifyMACAsync(keyId, msg, msgLen, mac, macLen, verifStatus);

    if (status == STATUS_SUCCESS)
    {
        /* Wait for the command to complete */
        CSE_DRV_WaitCommandCompletion(timeout);

        return s_cseStatePtr->cmdStatus;
    }
    else
    {
        return status;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_EncryptECBAsync
 * Description   : This function performs the AES-128 encryption in ECB mode of
 * the input plain text buffer, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_EncryptECBAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_EncryptECBAsync(cse_key_id_t keyId, const uint8_t *plainText,
                                 uint32_t length, uint8_t *cipherText)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(plainText != NULL);
    DEV_ASSERT(cipherText != NULL);
    /* Check the buffers addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)plainText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the buffer length is multiple of 16 bytes */
    DEV_ASSERT((length & CSE_BUFF_LEN_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_ENC_ECB;

    /* Prepare the command */
    CSE_PrepareEncryptEcbCmd(keyId, plainText, length, cipherText);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_ENC_ECB);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DecryptECBAsync
 * Description   : This function performs the AES-128 decryption in ECB mode of
 * the input cipher text buffer, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_DecryptECBAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_DecryptECBAsync(cse_key_id_t keyId, const uint8_t *cipherText,
                                 uint32_t length, uint8_t *plainText)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(plainText != NULL);
    DEV_ASSERT(cipherText != NULL);
    /* Check the buffers addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)plainText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the buffer length is multiple of 16 bytes */
    DEV_ASSERT((length & CSE_BUFF_LEN_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_DEC_ECB;

    /* Prepare the command */
    CSE_PrepareDecryptEcbCmd(keyId, cipherText, length, plainText);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_DEC_ECB);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_EncryptCBCAsync
 * Description   : This function performs the AES-128 encryption in CBC mode of
 * the input plain text buffer, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_EncryptCBCAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_EncryptCBCAsync(cse_key_id_t keyId, const uint8_t *plainText,
                                 uint32_t length, const uint8_t *iv, uint8_t *cipherText)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(plainText != NULL);
    DEV_ASSERT(cipherText != NULL);
    DEV_ASSERT(iv != NULL);
    /* Check the buffers addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)plainText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)iv) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the buffer length is multiple of 16 bytes */
    DEV_ASSERT((length & CSE_BUFF_LEN_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_ENC_CBC;

    /* Prepare the command */
    CSE_PrepareEncryptCbcCmd(keyId, plainText, length, iv, cipherText);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_ENC_CBC);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DecryptCBCAsync
 * Description   : This function performs the AES-128 decryption in CBC mode of
 * the input cipher text buffer, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_DecryptCBCAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_DecryptCBCAsync(cse_key_id_t keyId, const uint8_t *cipherText,
                                 uint32_t length, const uint8_t* iv, uint8_t *plainText)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(plainText != NULL);
    DEV_ASSERT(cipherText != NULL);
    DEV_ASSERT(iv != NULL);
    /* Check the buffers addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)plainText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)iv) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the buffer length is multiple of 16 bytes */
    DEV_ASSERT((length & CSE_BUFF_LEN_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_DEC_CBC;

    /* Prepare the command */
    CSE_PrepareDecryptCbcCmd(keyId, cipherText, length, iv, plainText);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_DEC_CBC);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GenerateMACAsync
 * Description   : This function calculates the MAC of a given message using CMAC
 * with AES-128, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_GenerateMACAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_GenerateMACAsync(cse_key_id_t keyId, const uint8_t *msg,
                             uint64_t msgLen, uint8_t *mac)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(msg != NULL);
    DEV_ASSERT(mac != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)mac) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)msg) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the message length is valid */
    DEV_ASSERT(msgLen < CSE_MAC_MAX_MSG_LEN);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_GENERATE_MAC;

    /* Prepare the command */
    CSE_PrepareGenerateMacCmd(keyId, msg, msgLen, mac);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_GENERATE_MAC);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_VerifyMACAsync
 * Description   : This function verifies the MAC of a given message using CMAC
 * with AES-128, in an asynchronous manner.
 *
 * Implements    : CSE_DRV_VerifyMACAsync_Activity
 * END**************************************************************************/
status_t CSE_DRV_VerifyMACAsync(cse_key_id_t keyId, const uint8_t *msg, uint64_t msgLen,
                                const uint8_t *mac, uint8_t macLen, bool *verifStatus)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(msg != NULL);
    DEV_ASSERT(mac != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)msg) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)mac) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the message length is valid */
    DEV_ASSERT(msgLen < CSE_MAC_MAX_MSG_LEN);
    /* Check the mac length is valid */
    DEV_ASSERT(macLen <= 128U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->macVerifStatus = verifStatus;
    s_cseStatePtr->cmd = CSE_CMD_VERIFY_MAC;

    /* Prepare the command */
    CSE_PrepareVerifyMacCmd(keyId, msg, msgLen, mac, macLen);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_VERIFY_MAC);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_LoadKey
 * Description   : Updates an internal key per the SHE specification.
 *
 * Implements    : CSE_DRV_LoadKey_Activity
 * END**************************************************************************/
status_t CSE_DRV_LoadKey(cse_key_id_t keyId, const uint8_t *m1, const uint8_t *m2,
                         const uint8_t *m3, uint8_t *m4, uint8_t *m5, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(m1 != NULL);
    DEV_ASSERT(m2 != NULL);
    DEV_ASSERT(m3 != NULL);
    DEV_ASSERT(m4 != NULL);
    DEV_ASSERT(m5 != NULL);
    /* Check the buffer addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)m1) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m2) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m3) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m4) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m5) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_LOAD_KEY;

    /* Prepare the command */
    CSE_PrepareLoadKeyCmd(keyId, m1, m2, m3, m4, m5);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_LOAD_KEY);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_LoadPlainKey
 * Description   : Updates the RAM key memory slot with a 128-bit plaintext.
 *
 * Implements    : CSE_DRV_LoadPlainKey_Activity
 * END**************************************************************************/
status_t CSE_DRV_LoadPlainKey(const uint8_t * plainKey, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer address is valid */
    DEV_ASSERT(plainKey != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)plainKey) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_LOAD_PLAIN_KEY;

    /* Prepare the command */
    CSE_PrepareLoadPlainKeyCmd(plainKey);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_LOAD_PLAIN_KEY);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_ExportRAMKey
 * Description   : Exports the RAM_KEY into a format compatible with the messages
 * used for LOAD_KEY.
 *
 * Implements    : CSE_DRV_ExportRAMKey_Activity
 * END**************************************************************************/
status_t CSE_DRV_ExportRAMKey(uint8_t *m1, uint8_t *m2, uint8_t *m3,
                              uint8_t *m4, uint8_t *m5, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(m1 != NULL);
    DEV_ASSERT(m2 != NULL);
    DEV_ASSERT(m3 != NULL);
    DEV_ASSERT(m4 != NULL);
    DEV_ASSERT(m5 != NULL);
    /* Check the buffer addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)m1) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m2) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m3) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m4) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)m5) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_EXPORT_RAM_KEY;

    /* Prepare the command */
    CSE_PrepareExportRamKeyCmd(m1, m2, m3, m4, m5);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_EXPORT_RAM_KEY);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_InitRNG
 * Description   : Initializes the seed for the PRNG.
 *
 * Implements    : CSE_DRV_InitRNG_Activity
 * END**************************************************************************/
status_t CSE_DRV_InitRNG(uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_INIT_RNG;

    /* Prepare the command */
    CSE_PrepareInitRngCmd();

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_INIT_RNG);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    if (s_cseStatePtr->cmdStatus == STATUS_SUCCESS)
    {
        s_cseStatePtr->rngInit = true;
    }
    else
    {
        s_cseStatePtr->rngInit = false;
    }

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_ExtendSeed
 * Description   : Extends the seed for the PRNG.
 *
 * Implements    : CSE_DRV_ExtendSeed_Activity
 * END**************************************************************************/
status_t CSE_DRV_ExtendSeed(const uint8_t *entropy, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer address is valid */
    DEV_ASSERT(entropy != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)entropy) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* RNG must be initialized before extending the seed */
    DEV_ASSERT(s_cseStatePtr->rngInit);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_EXTEND_SEED;

    /* Prepare the command */
    CSE_PrepareExtendPrngSeedCmd(entropy);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_EXTEND_SEED);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GenerateRND
 * Description   : Generates a vector of 128 random bits.
 *
 * Implements    : CSE_DRV_GenerateRND_Activity
 * END**************************************************************************/
status_t CSE_DRV_GenerateRND(uint8_t *rnd, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer address is valid */
    DEV_ASSERT(rnd != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)rnd) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* RNG must be initialized before generating the random value */
    DEV_ASSERT(s_cseStatePtr->rngInit);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_RND;

    /* Prepare the command */
    CSE_PrepareGenerateRndCmd(rnd);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_RND);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GetID
 * Description   : Returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * Implements    : CSE_DRV_GetID_Activity
 * END**************************************************************************/
status_t CSE_DRV_GetID(const uint8_t *challenge, uint8_t *uid,
                       uint8_t *sreg, uint8_t *mac, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(challenge != NULL);
    DEV_ASSERT(uid != NULL);
    DEV_ASSERT(sreg != NULL);
    DEV_ASSERT(mac != NULL);
    /* Check the buffer addresses are 32 bit aligned */
    DEV_ASSERT((((uint32_t)challenge) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    DEV_ASSERT((((uint32_t)mac) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_GET_ID;

    /* Prepare the command */
    CSE_PrepareGetIdCmd(challenge, uid, mac);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_GET_ID);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    *sreg = CSE_GetUIDSregResult();

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GenerateTRND
 * Description   :  Generates a vector of 128 random bits using TRNG.
 *
 * Implements    : CSE_DRV_GenerateTRND_Activity
 * END**************************************************************************/
status_t CSE_DRV_GenerateTRND(uint8_t *trnd, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer address is valid */
    DEV_ASSERT(trnd != NULL);
    /* Check the buffer address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)trnd) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_TRNG_RND;

    /* Prepare the command */
    CSE_PrepareGenerateTrndCmd(trnd);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_TRNG_RND);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_SecureBoot
 * Description   : This function executes the SHE secure boot protocol.
 *
 * Implements    : CSE_DRV_SecureBoot_Activity
 * END**************************************************************************/
status_t CSE_DRV_SecureBoot(uint32_t bootImageSize, const uint8_t *bootImagePtr,
                            uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the pointer is valid */
    DEV_ASSERT(bootImagePtr != NULL);
    /* Check the boot image address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)bootImagePtr) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_SECURE_BOOT;

    /* Prepare the command */
    CSE_PrepareSecureBootCmd(bootImageSize, bootImagePtr);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_SECURE_BOOT);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_BootFailure
 * Description   : This function signals a failure detected during later stages
 * of the boot process.
 *
 * Implements    : CSE_DRV_BootFailure_Activity
 * END**************************************************************************/
status_t CSE_DRV_BootFailure(uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_BOOT_FAILURE;

    /* Prepare the command */
    CSE_PrepareBootFailureCmd();

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_BOOT_FAILURE);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_BootOK
 * Description   : This function marks a successful boot verification during
 * later stages of the boot process.
 *
 * Implements    : CSE_DRV_BootOK_Activity
 * END**************************************************************************/
status_t CSE_DRV_BootOK(uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_BOOT_OK;

    /* Prepare the command */
    CSE_PrepareBootOkCmd();

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_BOOT_OK);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DbgChal
 * Description   : This function obtains a random number which the user shall
 * use along with the MASTER_ECU_KEY and UID to return an authorization request.
 *
 * Implements    : CSE_DRV_DbgChal_Activity
 * END**************************************************************************/
status_t CSE_DRV_DbgChal(uint8_t *challenge, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the pointer is valid */
    DEV_ASSERT(challenge != NULL);
    /* Check the boot image address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)challenge) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_DBG_CHAL;

    /* Prepare the command */
    CSE_PrepareDbgChalCmd(challenge);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_DBG_CHAL);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_DbgAuth
 * Description   : This function erases all user keys and enables internal
 * debugging if the authorization is confirmed by CSE.
 *
 * Implements    : CSE_DRV_DbgAuth_Activity
 * END**************************************************************************/
status_t CSE_DRV_DbgAuth(const uint8_t *authorization, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the pointer is valid */
    DEV_ASSERT(authorization != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->cmd = CSE_CMD_DBG_AUTH;

    /* Prepare the command */
    CSE_PrepareDbgAuthCmd(authorization);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_DBG_AUTH);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_MPCompress
 * Description   : This function implements a Miyaguchi-Preneel compression
 * in software.
 *
 * Implements    : CSE_DRV_MPCompress_Activity
 * END**************************************************************************/
status_t CSE_DRV_MPCompress(const uint8_t * msg, uint64_t msgLen,
                            uint8_t * mpCompress, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the buffer addresses are valid */
    DEV_ASSERT(msg != NULL);
    DEV_ASSERT(mpCompress != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_MP_COMPRESS;

    /* Prepare the command */
    CSE_PrepareMPCompressCmd(msg, &msgLen, mpCompress);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_MP_COMPRESS);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GetAsyncCmdStatus
 * Description   : This function checks the status of the execution of an
 * asynchronous command. If the command is still in progress, returns STATUS_BUSY.
 *
 * Implements    : CSE_DRV_GetAsyncCmdStatus_Activity
 * END**************************************************************************/
status_t CSE_DRV_GetAsyncCmdStatus(void)
{
    if (!s_cseStatePtr->cmdInProgress)
    {
        return s_cseStatePtr->cmdStatus;
    }

    return STATUS_BUSY;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_CancelCommand
 * Description   : Cancels a previously initiated command.
 *
 * Implements    : CSE_DRV_CancelCommand_Activity
 * END**************************************************************************/
status_t CSE_DRV_CancelCommand(void)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Set the blocking flag so the synchronization semaphore is posted in the ISR */
    s_cseStatePtr->blockingCmd = true;

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_CANCEL);

    /* Wait for the cancelled command to complete */
    (void)OSIF_SemaWait(&s_cseStatePtr->cmdComplete, 1U);

    /* Clear the blocking flag */
    s_cseStatePtr->blockingCmd = false;

    return STATUS_SUCCESS;
}

#if FEATURE_CSE_FLASHLESS_CONFIG

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_SetKeyImageAddress
 * Description   : Sets the Key Image Address registers.
 *
 * Implements    : CSE_DRV_SetKeyImageAddress_Activity
 * END**************************************************************************/
status_t CSE_DRV_SetKeyImageAddress(uint32_t kia0, uint32_t kia1)
{
    CSE->KIA[0] = kia0;
    CSE->KIA[1] = kia1;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_PublishKeyImage
 * Description   : Generates a 1024 byte key image block and writes it to the
 * location specified by the Key Image Address parameter.
 *
 * Implements    : CSE_DRV_PublishKeyImage_Activity
 * END**************************************************************************/
status_t CSE_DRV_PublishKeyImage(uint32_t firmwareAddr, bool incSecCounter, uint32_t *kia,
                                 uint32_t timeout)
{
    uint8_t counter;

    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the key image address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)kia) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_PUBLISH_KEY_IMG;

    counter = (uint8_t)(CSE->OTP & CSE_OTP_SEC_CNT_MASK);
    if (incSecCounter)
    {
        counter++;
    }

    /* Prepare the command */
    CSE_PreparePublishKeyImgCmd(firmwareAddr, counter, kia);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_PUBLISH_KEY_IMG);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

#endif /* FEATURE_CSE_FLASHLESS_CONFIG */

#if FEATURE_CSE_HAS_SECURE_RAM

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_GetSecureRAMInfo
 * Description   : Gets the address and size of the Secure RAM.
 *
 * Implements    : CSE_DRV_GetSecureRAMInfo_Activity
 * END**************************************************************************/
status_t CSE_DRV_GetSecureRAMInfo(uint32_t *addr, uint32_t *size)
{
    *addr = CSE->SRA0;
    *size = CSE->SRS0;

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_LoadSecureRAM
 * Description   : Initializes the secure RAM and upon successful decryption and
 * authentication of the initialization data puts the secure RAM in the
 * secure state.
 *
 * Implements    : CSE_DRV_LoadSecureRAM_Activity
 * END**************************************************************************/
status_t CSE_DRV_LoadSecureRAM(cse_key_id_t keyId, const uint8_t *cipherText,
                               const uint8_t* iv, const uint8_t *mac, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the ciphertext address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the IV address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)iv) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the MAC address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)mac) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_LOAD_SEC_RAM;

    /* Prepare the command */
    CSE_PrepareLoadSecRAMCmd(keyId, cipherText, iv, mac);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_LOAD_SEC_RAM);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_OpenSecureRAM
 * Description   : Resets to zero the secure RAM and puts the secure RAM in the open
 * state.
 *
 * Implements    : CSE_DRV_OpenSecureRAM_Activity
 * END**************************************************************************/
status_t CSE_DRV_OpenSecureRAM(uint32_t addr, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_OPEN_SEC_RAM;

    /* Prepare the command */
    CSE_PrepareOpenSecRAMCmd(addr);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_OPEN_SEC_RAM);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_ExportSecureRAM
 * Description   : Generates a ciphertext block, 128-bit IV and a 128-bit CMAC output
 * to store the contents of the secure RAM in system memory.
 *
 * Implements    : CSE_DRV_ExportSecureRAM_Activity
 * END**************************************************************************/
status_t CSE_DRV_ExportSecureRAM(cse_key_id_t keyId, const uint8_t *cipherText,
                               const uint8_t* iv, const uint8_t *mac, uint32_t timeout)
{
    /* Check the driver is initialized */
    DEV_ASSERT(s_cseStatePtr != NULL);
    /* Check the ciphertext address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)cipherText) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the IV address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)iv) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);
    /* Check the MAC address is 32 bit aligned */
    DEV_ASSERT((((uint32_t)mac) & CSE_BUFF_ADDR_CHECK_MASK) == 0U);

    /* Check there is no other command in execution */
    if (CSE_IsBusy() || s_cseStatePtr->cmdInProgress)
    {
        return STATUS_BUSY;
    }

    /* Update the internal flags */
    s_cseStatePtr->cmdInProgress = true;
    s_cseStatePtr->blockingCmd = true;
    s_cseStatePtr->cmd = CSE_CMD_EXPORT_SEC_RAM;

    /* Prepare the command */
    CSE_PrepareExportSecRAMCmd(keyId, cipherText, iv, mac);

    /* Send the command to CSE */
    CSE_SendCmd(CSE_CMD_EXPORT_SEC_RAM);

    /* Wait for the command to complete */
    CSE_DRV_WaitCommandCompletion(timeout);

    return s_cseStatePtr->cmdStatus;
}

#endif /* FEATURE_CSE_HAS_SECURE_RAM */

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_IRQ_IRQHandler
 * Description   : Implementation of the CSE interrupt handler. Handles completed
 * command events.
 *
 * END**************************************************************************/
void CSE_IRQ_IRQHandler(void)
{
    if (s_cseStatePtr->cmdInProgress)
    {
        /* Wait for the BUSY flag to be cleared by hw */
        while (CSE_IsBusy()) {}

        /* Retrieve the error code of last command */
        uint32_t err = CSE_GetErrCode();

        /* Update the internal driver status */
        if (err == 0U)
        {
            s_cseStatePtr->cmdStatus = STATUS_SUCCESS;
        }
        else
        if (err == 0xBU)
        {
            s_cseStatePtr->cmdStatus = STATUS_BUSY;
        }
        else
        {
            s_cseStatePtr->cmdStatus = CSE_CONVERT_ERC(err);
        }

        /* If the command was VERIFY_MAC, retrieve the result of the verification */
        if ((s_cseStatePtr->cmd == CSE_CMD_VERIFY_MAC) && (s_cseStatePtr->macVerifStatus != NULL))
        {
            *s_cseStatePtr->macVerifStatus = CSE_GetMacVerifResult();
            s_cseStatePtr->macVerifStatus = NULL;
        }

        /* Call the user callback, if available */
        if (s_cseStatePtr->callback != NULL)
        {
            s_cseStatePtr->callback((uint32_t)s_cseStatePtr->cmd, s_cseStatePtr->callbackParam);
        }

        if (s_cseStatePtr->blockingCmd)
        {
            /* Update the internal blocking flag */
            s_cseStatePtr->blockingCmd = false;

            /* Update the synchronization object */
            (void)OSIF_SemaPost(&s_cseStatePtr->cmdComplete);
        }

        /* Update the internal busy flag */
        s_cseStatePtr->cmdInProgress = false;
        /* No command in execution at this point */
        s_cseStatePtr->cmd = CSE_CMD_NONE;
    }

    /* Clear the interrupt flag */
    CSE_ClearIntFlag();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_DRV_WaitCommandCompletion
 * Description   : Waits on the synchronization semaphore and updates the flags.
 *
 * END**************************************************************************/
static void CSE_DRV_WaitCommandCompletion(uint32_t timeout)
{
    status_t syncStatus;

    /* Wait for command completion */
    syncStatus = OSIF_SemaWait(&s_cseStatePtr->cmdComplete, timeout);

    /* Update the busy flag and status if timeout expired */
    if (syncStatus == STATUS_TIMEOUT)
    {
        (void)CSE_DRV_CancelCommand();
        s_cseStatePtr->blockingCmd = false;
        s_cseStatePtr->cmdStatus = STATUS_TIMEOUT;
    }
}

/******************************************************************************
 * EOF
 *****************************************************************************/
