/*
 * Copyright 2017 - 2019 NXP
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

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 4.9, Function-like macro.
 * The macro defines a bitmask used to access status flags.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.9, could define variable 'eMIOSModeAccept' at block scope
 * The variable has a bit of large size and initialzing time for it is big so it must remain global.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are static data and it's used in local source files only.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */

#include "emios_common.h"
#include "emios_hw_access.h"

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint32_t const eMIOSModeAccept[3][12] = {
    {/*GPIO*/0xFFFFFFFFUL, /*SAIC*/0xFFFFFFFFUL, /*SAOC*/0xFFFFFFFFUL, /*IPWM*/0x0000FEFEUL, /*IPM*/0x0000FEFEUL, /*DAOC*/0x0000FEFEUL, /*MC*/0x01C10101UL,
    /*MCB*/0x01C101FFUL, /*OPWFMB*/0x01C101FFUL, /*OPWMCB*/0x000000FEUL, /*OPWMB*/0xFFFFFFFFUL, /*OPWMT*/0xFFFFFFFFUL },
    {/*GPIO*/0xFFFFFFFFUL, /*SAIC*/0xFFFFFFFFUL, /*SAOC*/0xFFFFFFFFUL, /*IPWM*/0x0000FEFEUL, /*IPM*/0x0000FEFEUL, /*DAOC*/0x0000FEFEUL, /*MC*/0x01C10101UL,
    /*MCB*/0x01C10101UL, /*OPWFMB*/0x01C10101UL, /*OPWMCB*/0x00000000UL, /*OPWMB*/0xFFFFFFFFUL, /*OPWMT*/0xFFFFFFFFUL },
    {/*GPIO*/0xFFFFFFFFUL, /*SAIC*/0xFFFFFFFFUL, /*SAOC*/0xFFFFFFFFUL, /*IPWM*/0x00000000UL, /*IPM*/0x00000000UL, /*DAOC*/0x00000000UL, /*MC*/0x01C10101UL,
    /*MCB*/0x01C10101UL, /*OPWFMB*/0x01C10101UL, /*OPWMCB*/0x00000000UL, /*OPWMB*/0xFFFFFFFFUL, /*OPWMT*/0xFFFFFFFFUL }
};

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_ValidateMode
 * Description   : Validate a eMIOS mode can run on the channel or not.
 *END**************************************************************************/
bool EMIOS_ValidateMode(uint8_t emiosGroup,
                        uint8_t channel,
                        uint8_t mode)
{
    return ((((eMIOSModeAccept[emiosGroup][mode] >> channel) & 0x01UL) == 1UL) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_ValidateInternalCnt
 * Description   : Check a channel accept internal counter or not.
 *END**************************************************************************/
bool EMIOS_ValidateInternalCnt(uint8_t emiosGroup,
                               uint8_t channel)
{
    bool ret = false;

    if ((emiosGroup == (uint8_t)EMIOS_GROUP0) && (channel < 8U))
    {
        ret = true;
    }
    else
    {
        if ((channel    == (uint8_t)EMIOS_CNT_BUSA_DRIVEN)
            || (channel == (uint8_t)EMIOS_CNT_BUSB_DRIVEN)
            || (channel == (uint8_t)EMIOS_CNT_BUSC_DRIVEN)
            || (channel == (uint8_t)EMIOS_CNT_BUSD_DRIVEN)
            || (channel == (uint8_t)EMIOS_CNT_BUSE_DRIVEN)
            || (channel == (uint8_t)EMIOS_CNT_BUSF_DRIVEN)
            )
            {
                ret = true;
            }
    }

    return ret;
}
#endif /* FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_ValidateChannel
 * Description   : Validate a eMIOS channel can support.
 *END**************************************************************************/
bool EMIOS_ValidateChannel(uint8_t inChVal, uint8_t * outChVal)
{
    bool tmp = false;
#if (FEATURE_EMIOS_BUS_B_SELECT == 1U)
    if (inChVal <= 7U)
    {
        *outChVal = FEATURE_EMIOS_BUS_B_SELECT_OFFSET(inChVal);
        tmp = true;
    }
#endif
#if (FEATURE_EMIOS_BUS_C_SELECT == 1U)
    if ((inChVal >= 8U) && (inChVal <= 15U))
    {
        *outChVal = FEATURE_EMIOS_BUS_C_SELECT_OFFSET(inChVal);
        tmp = true;
    }
#endif
#if (FEATURE_EMIOS_BUS_D_SELECT == 1U)
    if ((inChVal >= 16U) && (inChVal <= 23U))
    {
        *outChVal = FEATURE_EMIOS_BUS_D_SELECT_OFFSET(inChVal);
        tmp = true;
    }
#endif
#if (FEATURE_EMIOS_BUS_E_SELECT == 1U)
    if ((inChVal >= 24U) && (inChVal <= 31U))
    {
        *outChVal = FEATURE_EMIOS_BUS_E_SELECT_OFFSET(inChVal);
        tmp = true;
    }
#endif

    return tmp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_AllowEnterDebugMode
 * Description   : Enables the eMIOS to freeze the registers of the Unified Channels
 * when Debug Mode is requested at MCU level. To set a channel enters freeze state,
 * should be setting EMIOS_AllowEnterDebugMode then EMIOS_ChannelEnterDebugMode.
 *END**************************************************************************/
 void EMIOS_AllowEnterDebugMode(uint8_t emiosGroup)
{
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);

    eMIOS_MCR_SET_FRZ(emiosGroup, 1UL);
}

/*******************************************************************************
* EOF
******************************************************************************/
