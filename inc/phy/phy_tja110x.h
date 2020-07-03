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

#ifndef PHY_TJA110X_H
#define PHY_TJA110X_H

#include "phy.h"

#ifdef __cplusplus
extern "C"{
#endif

/*!
 * @brief TJA110x PHY local wakeup pin timer
 */
typedef enum
{
    PHY_TJA110X_LOCALWUTIME_SHORTEST = 3,  /**< shortest (10 us to 40 us) */
    PHY_TJA110X_LOCALWUTIME_SHORT    = 2,  /**< short (100 us to 200 us) */
    PHY_TJA110X_LOCALWUTIME_LONG     = 1,  /**< long (250 us to 500 us) */
    PHY_TJA110X_LOCALWUTIME_LONGEST  = 0   /**< longest (10 ms to 20 ms) */
} phy_tja110x_localWuTime_t;

/*!
 * @brief TJA110x PHY extended configuration structure
 */
typedef struct
{
	bool fwdPhyLocal;                 /**< local wake-up forwarding */
	bool fwdPhyRemote;                /**< remote wake-up forwarding */
	bool localWakeupPhy;              /**< local wake-up */
	bool remoteWakeupPhy;             /**< remote wake-up */
	bool reducedMiiDriveStrength;     /**< MII output driver strength reduced */
	bool reducedClkDriveStrength;     /**< CLK_IN_OUT output driver strength reduced */
	bool clkHold;                     /**< XTAL and CLK_IN_OUT output remain active until device switched to Sleep mode via SMI */
	bool wakeInRatiometricThreshold;  /**< local wake configuration: ratiometric input threshold */
	phy_tja110x_localWuTime_t localWuTimer;  /**< local wake-up timer */
} phy_tja110x_config_t;

/*!
 * @brief TJA110x PHY driver structure.
 */
extern phy_driver_t PHY_driver_tja110x;

#ifdef __cplusplus
}
#endif

#endif /* PHY_TJA110X_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
 
