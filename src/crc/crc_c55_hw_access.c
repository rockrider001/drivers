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

#include "crc_c55_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Initial checksum */
#define CRC_INITIAL_SEED        (0U)

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_Init
 * Description   : This function initializes the module to default configuration
 * (Default polynomial: 0x1021U,
 * Swap byte-wise CRC_INP input data: false,
 * Swap bit-wise CRC_INP input data: true,
 * Swap CRC_OUTP content: false,
 * Inversion CRC_OUTP content: false,
 * No complement of checksum read,
 * CRC_BITS_16_CCITT).
 *
 *END**************************************************************************/
void CRC_Init(CRC_Type * const base)
{
    /* Set CRC mode to 32-bit */
    CRC_SetPolyReg(base, FEATURE_CRC_DEFAULT_POLYNOMIAL);

    /* Sets transpose and inversion checksum to none */
    CRC_SetWriteTranspose(base, FEATURE_CRC_DEFAULT_WRITE_TRANSPOSE);
    CRC_SetReadTranspose(base, false);
    CRC_SetFXorMode(base, false);

    /* CRC seed initialization */
    CRC_SetSeedReg(base, CRC_INITIAL_SEED);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_SetWriteTranspose
 *
 * This function sets the CRC transpose type for writes
 *
*END**************************************************************************/
void CRC_SetWriteTranspose(CRC_Type * const base,
                           crc_transpose_t transp)
{
    switch(transp)
    {
        case CRC_TRANSPOSE_NONE:                /* Case no transpose */
            CRC_SetSwapBytewise(base, false);
            CRC_SetSwapBitwise(base, false);
            break;
        case CRC_TRANSPOSE_BITS:                /* Case bits transpose */
            CRC_SetSwapBytewise(base, false);
            CRC_SetSwapBitwise(base, true);
            break;
        case CRC_TRANSPOSE_BYTES:               /* Case bytes transpose */
            CRC_SetSwapBytewise(base, true);
            CRC_SetSwapBitwise(base, false);
            break;
        default:
            /* Do nothing */
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_GetWriteTranspose
 *
 * This function gets the CRC transpose type for writes
 *
*END**************************************************************************/
crc_transpose_t CRC_GetWriteTranspose(const CRC_Type * const base)
{
    crc_transpose_t tempTranspose;
    bool tempByte;
    bool tempBit;

    /* Get swap byte-wise */
    tempByte = CRC_GetSwapBytewise(base);
    /* Get swap bit-wise */
    tempBit = CRC_GetSwapBitwise(base);

    if(tempByte && (!tempBit))         /* Case bytes transpose */
    {
        tempTranspose = CRC_TRANSPOSE_BYTES;
    }
    else if((!tempByte) && tempBit)    /* Case bits transpose */
    {
        tempTranspose = CRC_TRANSPOSE_BITS;
    }
    else                               /* Case no transpose */
    {
        tempTranspose = CRC_TRANSPOSE_NONE;
    }

    return tempTranspose;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CRC_GetCrcResult
 * Description   : This function returns the current result of the CRC calculation.
 *
 *END**************************************************************************/
uint32_t CRC_GetCrcResult(const CRC_Type * const base)
{
    crc_mode_polynomial_t polynomial = CRC_GetPolyReg(base);
    uint32_t result;

    switch(polynomial)
    {
        case CRC_BITS_32:
            /* Returns 32 bits of CRC calculation */
            result = CRC_GetDataOutp(base);
            break;
        case CRC_BITS_16_CCITT:
            /* Returns lower 16 bits of CRC calculation */
            result = CRC_GetDataOutp(base) & 0xFFFFU;
            break;
        case CRC_BITS_8:
#if (FEATURE_CRC_BITS_8_H2F == 1U)
        case CRC_BITS_8_H2F:
#endif
        default:
            /* fall-through */
            /* Returns lowest 8 bits of CRC calculation */
            result = CRC_GetDataOutp(base) & 0xFFU;
            break;
    }

    return result;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
