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

#ifndef PHY_SHARED_H
#define PHY_SHARED_H

#include <stdint.h>
#include "phy.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Private Data Format */
/* This section defines the meaning of the fields within the private section of the driver configuration */

#define PRIVATE_ANEG_CAPABLE  0x00000001U    /**< Flag that defines if a device is capable of auto-negotiation */
#define PRIVATE_ANEG_MSCTRL   0x00000002U    /**< Flag that defines if a device is using master/slave settings during auto-negotiation (i.e. if register 9 is available) */
#define PRIVATE_ANEG_ENABLED  0x00000004U    /**< Flag that defines if auto-negotiation is enabled */
#define PRIVATE_1000BASET     0x00000008U    /**< Flag that defines if a device is 1000BASE-T capable */
#define PRIVATE_TJA1100       0x00001000U    /**< Flag that defines that a device is a TJA1100 --> limited wake/sleep support */
#define PRIVATE_LINKUP        0x80000000U    /**< Flag that defines if the link is up */
#define PRIVATE_ANEG_PENDING  0x40000000U    /**< Flag that defines if a completion of auto-negotiation is expected */

#ifdef __cplusplus
extern "C"{
#endif

void PHY_SHARED_ExtractPhyID(const uint16_t id1, const uint16_t id2, phy_id_t* id);

#ifdef __cplusplus
}
#endif



#endif /* PHY_SHARED_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
