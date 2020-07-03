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
 
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "phy.h"
#include "phy_shared.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ID_1_OCTET0_MASK  0xFC00U  /**< Bits belonging to the octet 0 of OUI */
#define ID_1_OCTET1_MASK  0x03FCU  /**< Bits belonging to the octet 1 of OUI */
#define ID_1_OCTET2_MASK  0x0003U  /**< Bits belonging to the octet 2 of OUI */

#define ID_1_OCTET0_SHIFT 10U
#define ID_1_OCTET1_SHIFT 2U
#define ID_1_OCTET2_SHIFT 6U      /**< Left Shift to reach correct position in OUI byte */

#define ID_2_OCTET2_MASK  0xFC00U  /**< Bits belonging to the third octet of OUI */
#define ID_2_OCTET2_SHIFT 10U

#define ID_2_TYPE_MASK  0x03F0U
#define ID_2_TYPE_SHIFT 4U
#define ID_2_REV_MASK   0x000FU


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint8_t swap(const uint8_t byte);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PHY_SHARED_ExtractPhyID
 * Description   : Extract PHY ID from two PHY ID registers
 *
 *END**************************************************************************/
void PHY_SHARED_ExtractPhyID(const uint16_t id1, const uint16_t id2, phy_id_t* id)
{
    uint8_t ouiOctet0 = (uint8_t) ((id1 & (uint16_t) ID_1_OCTET0_MASK) >> ID_1_OCTET0_SHIFT);
    uint8_t ouiOctet1 = (uint8_t) ((id1 & (uint16_t) ID_1_OCTET1_MASK) >> ID_1_OCTET1_SHIFT);
    uint8_t ouiOctet2 = (uint8_t) (id1 & (uint16_t) ID_1_OCTET2_MASK) << ID_1_OCTET2_SHIFT;

    ouiOctet2 |= (uint8_t) ((id2 & (uint16_t) ID_2_OCTET2_MASK) >> ID_2_OCTET2_SHIFT);

    ouiOctet0 = swap(ouiOctet0);
    ouiOctet1 = swap(ouiOctet1);
    ouiOctet2 = swap(ouiOctet2);

    id->oui = (((uint32_t) ouiOctet0) << 16U) | (((uint32_t) ouiOctet1) << 8U) | ((uint32_t) ouiOctet2);

    id->typeNo     = (uint8_t)((id2 & ID_2_TYPE_MASK) >> ID_2_TYPE_SHIFT);
    id->revisionNo = (uint8_t)(id2 & ID_2_REV_MASK);

    return;
}

static uint8_t swap(const uint8_t byte)
{
    uint8_t swappedByte = byte;
    uint8_t temp;

    temp = swappedByte & (uint8_t) 0x0FU;
    temp = (uint8_t) (((uint16_t) temp) << 4U);
    swappedByte = ((swappedByte & (uint8_t) 0xF0U) >> 4U) | temp;

    temp = swappedByte & (uint8_t) 0x33U;
    temp = (uint8_t) (((uint16_t) temp) << 2U);    
    swappedByte = ((swappedByte & (uint8_t) 0xCCU) >> 2U) | temp;

    temp = swappedByte & (uint8_t) 0x55U;
    temp = (uint8_t) (((uint16_t) temp) << 1U);   
    swappedByte = ((swappedByte & (uint8_t) 0xAAU) >> 1U) | temp;

    return swappedByte;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
