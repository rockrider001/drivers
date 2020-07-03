/*
 * Copyright 2018 NXP
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

#include "cse_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Static variable storing the 64-bits message length; a reference to this
 * variable is passed to CSE for 'generate MAC' command;
 */
static uint64_t s_msgLen;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareCommand
 * Description   : Prepares the CSE command parameters.
 *
 *END**************************************************************************/
void CSE_PrepareCommand(uint32_t param1,
                        uint32_t param2,
                        uint32_t param3,
                        uint32_t param4,
                        uint32_t param5)
{
    CSE->P[0U] = param1;
    CSE->P[1U] = param2;
    CSE->P[2U] = param3;
    CSE->P[3U] = param4;
    CSE->P[4U] = param5;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareEncryptEcbCmd
 * Description   : Prepares the CSE ECB encrypt command.
 *
 *END**************************************************************************/
void CSE_PrepareEncryptEcbCmd(cse_key_id_t keyId, const uint8_t *plainText,
                              uint32_t length, uint8_t *cipherText)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    CSE_PrepareCommand(cmdKeyId, CSE_BUFF_BLOCK_COUNT(length), (uint32_t)plainText,
                       (uint32_t)cipherText, CSE_UNUSED_PARAM);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareDecryptEcbCmd
 * Description   : Prepares the CSE ECB decrypt command.
 *
 *END**************************************************************************/
void CSE_PrepareDecryptEcbCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                              uint32_t length, uint8_t *plainText)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    CSE_PrepareCommand(cmdKeyId, CSE_BUFF_BLOCK_COUNT(length), (uint32_t)cipherText,
                       (uint32_t)plainText, CSE_UNUSED_PARAM);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareEncryptCbcCmd
 * Description   : Prepares the CSE CBC encrypt command.
 *
 *END**************************************************************************/
void CSE_PrepareEncryptCbcCmd(cse_key_id_t keyId, const uint8_t *plainText,
                              uint32_t length, const uint8_t *iv, uint8_t *cipherText)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    CSE_PrepareCommand(cmdKeyId, (uint32_t)iv, CSE_BUFF_BLOCK_COUNT(length),
                       (uint32_t)plainText, (uint32_t)cipherText);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareDecryptCbcCmd
 * Description   : Prepares the CSE CBC decrypt command.
 *
 *END**************************************************************************/
void CSE_PrepareDecryptCbcCmd(cse_key_id_t keyId, const uint8_t *cipherText,
                              uint32_t length, const uint8_t *iv, uint8_t *plainText)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    CSE_PrepareCommand(cmdKeyId, (uint32_t)iv, CSE_BUFF_BLOCK_COUNT(length),
                       (uint32_t)cipherText, (uint32_t)plainText);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareGenerateMacCmd
 * Description   : Prepares the CSE generate MAC command.
 *
 *END**************************************************************************/
void CSE_PrepareGenerateMacCmd(cse_key_id_t keyId, const uint8_t *msg,
                               uint64_t msgLen, uint8_t *mac)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    /* Save the message length in the internal driver variable */
    s_msgLen = msgLen;

    CSE_PrepareCommand(cmdKeyId, (uint32_t)(&s_msgLen),
                       (uint32_t)msg, (uint32_t)mac, CSE_UNUSED_PARAM);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareVerifyMacCmd
 * Description   : Prepares the CSE verify MAC command.
 *
 *END**************************************************************************/
void CSE_PrepareVerifyMacCmd(cse_key_id_t keyId, const uint8_t *msg,
                             uint64_t msgLen, const uint8_t *mac, uint8_t macLen)
{
    uint32_t cmdKeyId;

    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }
    /* Get the command key ID (strip the KBS bit) */
    cmdKeyId = CSE_CMD_KEY_ID(keyId);

    /* Save the message length in the internal driver variable */
    s_msgLen = msgLen;

    CSE_PrepareCommand(cmdKeyId, (uint32_t)(&s_msgLen),
                       (uint32_t)msg, (uint32_t)mac, (uint32_t)macLen);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_PrepareLoadKeyCmd
 * Description   : Prepares the CSE load key command.
 *
 *END**************************************************************************/
void CSE_PrepareLoadKeyCmd(cse_key_id_t keyId, const uint8_t *m1, const uint8_t *m2,
                           const uint8_t *m3, uint8_t *m4, uint8_t *m5)
{
    /* Check the key bank and update command accordingly */
    if (CSE_CMD_KBS(keyId) != 0U)
    {
        CSE->CR |= CSE_CR_KBS_MASK;
    }
    else
    {
        CSE->CR &= ~CSE_CR_KBS_MASK;
    }

    CSE_PrepareCommand((uint32_t)m1, (uint32_t)m2, (uint32_t)m3,
                       (uint32_t)m4, (uint32_t)m5);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSE_GetMacVerifResult
 * Description   : Returns the result of the last MAC verification.
 *
 *END**************************************************************************/
bool CSE_GetMacVerifResult(void)
{
    return (CSE->P[4] == 0U);
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
