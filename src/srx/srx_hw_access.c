/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
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
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize base pointers with unsigned integer values.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include <stddef.h>
#include "device_registers.h"
#include "srx_hw_access.h"
#include "srx_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/**
 * Used for bringing the register to a default reset state
 */
#define SRX_GBL_CTRL_RESET_VALUE    (0x00010010u)
#define SRX_CHNL_EN_RESET_VALUE    (0x00000000u)
#define SRX_DATA_CTRL1_RESET_VALUE  (0x11000000u)
#define SRX_FDMA_CTRL_RESET_VALUE   (0x00000000u)
#define SRX_SDMA_CTRL_RESET_VALUE   (0x00000000u)
#define SRX_FRDY_IE_RESET_VALUE (0x00000000u)
#define SRX_SRDY_IE_RESET_VALUE (0x00000000u)
#define SRX_CHn_CLK_CTRL_RESET_VALUE    (0x00008000u)
#define SRX_CHn_CONFIG_RESET_VALUE  (0x00000104u)

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * Base pointers for the peripheral registers.
 */
static SRX_Type * const s_srxBase[SRX_INSTANCE_COUNT] = SRX_BASE_PTRS;

#if defined(ERRATA_E7425) /* Enable only when we have this active */
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_ErrataE7425Workaroud
 * Description   : Workarounds for E7425.
 *
 *END**************************************************************************/
bool SRX_DRV_HW_ErrataE7425Workaroud(const uint32_t instance, const uint32_t channel)
{
    static uint8_t remCount[SRX_INSTANCE_COUNT][SRX_CHANNEL_COUNT]; /* Remaining clearing count */
    bool rval = true;

    /* Only the channel has PAUSE pulse detection enabled */
    if ((s_srxBase[instance]->CH[channel].CONFIG & (uint32_t)SRX_CONFIG_PAUSE_EN_MASK) != 0u)
    {
        /* NUM_EDGES_ERR reporting depends on errata */
        if ((s_srxBase[instance]->CH[channel].STATUS
                & (uint32_t)SRX_STATUS_NUM_EDGES_ERR_MASK) != 0u)
        {
            if (remCount[instance][channel] > 0u)
            {
                /* False event */
                rval = false;

                /* Clear */
                s_srxBase[instance]->CH[channel].STATUS |= SRX_STATUS_NUM_EDGES_ERR_MASK;
                remCount[instance][channel]--;
            }
        }

        /* Reload counters */
        if ((s_srxBase[instance]->CH[channel].STATUS
                & (uint32_t)(SRX_STATUS_NIB_VAL_ERR_MASK | SRX_STATUS_FMSG_CRC_ERR_MASK)) != 0u)
        {
            remCount[instance][channel] = 2u;
        }
        else if ((s_srxBase[instance]->CH[channel].STATUS
                & (uint32_t)(SRX_STATUS_CAL_LEN_ERR_MASK | SRX_STATUS_FMSG_OFLW_MASK)) != 0u)
        {
            remCount[instance][channel] = 1u;
        }
        else
        {
            /* MISRA */
        }
    }

    return rval;
}
#endif /* defined(ERRATA_E7425) */

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetTimestampPrescaler
 * Description   : This function sets the timestamp prescaler value
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetTimestampPrescaler(const uint32_t instance, const uint8_t value)
{
    /* Ensure clearing before modification */
    s_srxBase[instance]->GBL_CTRL &= ~SRX_GBL_CTRL_TSPRSC_MASK;
    s_srxBase[instance]->GBL_CTRL |= SRX_GBL_CTRL_TSPRSC(value);

#ifdef FEATURE_SRX_HAS_NIB_LEN_VAR_LIMIT
    /* Jitter compensation enabled */
    s_srxBase[instance]->GBL_CTRL |= SRX_GBL_CTRL_NIB_LEN_VAR_LIMIT(1);
#endif
}

#ifdef FEATURE_SRX_DMA_HAS_FIFO
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFifoWm
 * Description   : This function sets the FIFO watermark level.
 * The level describes the number of valid messages that are to
 * be stored in the FIFO queue before a DMA transfer is triggered.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFifoWm(const uint32_t instance, const uint8_t value)
{
    uint8_t locVal;

    /* Valid values are between 1 and FEATURE_SRX_MAX_FIFO_SIZE */
    locVal = (value < 1u) ? 1u : ((value > FEATURE_SRX_MAX_FIFO_SIZE) ? FEATURE_SRX_MAX_FIFO_SIZE : value);

    /* Ensure clearing before modification */
    s_srxBase[instance]->GBL_CTRL &= ~SRX_GBL_CTRL_FIFOWM_MASK;
    s_srxBase[instance]->GBL_CTRL |= SRX_GBL_CTRL_FIFOWM(locVal);
}
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

#ifdef FEATURE_SRX_DMA_HAS_FIFO
/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFifoState
 * Description   : Sets the enables state (active/inactive) for the DMA FIFO
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFifoState(const uint32_t instance, const bool state)
{
    /* No pre-clearing required */
    if(state)
    {
        s_srxBase[instance]->GBL_CTRL |= SRX_GBL_CTRL_FIFO_EN(1u);
    }
    else
    {
        s_srxBase[instance]->GBL_CTRL &= ~SRX_GBL_CTRL_FIFO_EN(1u);
    }
}
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetChannelPrescaler
 * Description   : Sets the channel prescaler.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetChannelPrescaler(const uint32_t instance, const uint8_t channel, const uint16_t prescaler)
{
    uint16_t lPre;
    uint16_t maxVal;

    /* Prescaler limited to maximum range of SRX_CLK_CTRL_PRSC_WIDTH bits */
    maxVal = (uint16_t)((uint16_t)((uint16_t)1u << SRX_CLK_CTRL_PRSC_WIDTH) - 1u);
    lPre = (prescaler > maxVal) ? maxVal : prescaler;

    s_srxBase[instance]->CH[channel].CLK_CTRL &= ~SRX_CLK_CTRL_PRSC_MASK;
    s_srxBase[instance]->CH[channel].CLK_CTRL |= SRX_CLK_CTRL_PRSC(lPre);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetChannelCompensation
 * Description   : Sets the channel compensation state (enable / disable).
 * This enables automatic compesation for deviations in the receve clock.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetChannelCompensation(const uint32_t instance, const uint8_t channel, const bool state)
{
    /* No pre clearing required */
    if(state)
    {
        s_srxBase[instance]->CH[channel].CLK_CTRL |= SRX_CLK_CTRL_COMP_EN(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CLK_CTRL &= ~SRX_CLK_CTRL_COMP_EN(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetBusIdleCnt
 * Description   : Sets the value for the Bus Idle Count paramter.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetBusIdleCnt(const uint32_t instance, const uint8_t channel, const srx_diag_idle_cnt_cfg_t count)
{
    /* Clear first */
    s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_BUS_IDLE_CNT_MASK;

    switch(count)
    {
    case SRX_BUS_IDLE_DISABLED:
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_BUS_IDLE_CNT(0u);
        break;

    case SRX_BUS_IDLE_245_CLK_TICKS:
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_BUS_IDLE_CNT(1u);
        break;

    case SRX_BUS_IDLE_508_CLK_TICKS:
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_BUS_IDLE_CNT(2u);
        break;

    case SRX_BUS_IDLE_1016_CLK_TICKS:
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_BUS_IDLE_CNT(4u);
        break;

    case SRX_BUS_IDLE_2032_CLK_TICKS:
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_BUS_IDLE_CNT(8u);
        break;

    default: /* Disabled since there was a clear first */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetCalRng
 * Description   : Sets the value for the Calibration Range paramter.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetCalRng(const uint32_t instance, const uint8_t channel, const srx_diag_calib_pulse_var_cfg_t range)
{
    /* No pre clearing required */
    if(range == SRX_CALIB_VAR_25_PERCENT)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_CAL_RNG(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_CAL_RNG(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetPpChkSel
 * Description   : Sets the value for the Pause Pulse Check paramter.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetPpChkSel(const uint32_t instance, const uint8_t channel, const srx_diag_pulse_cfg_t check)
{
    /* No pre clearing required */
    if(check == SRX_PULSE_CHECK_PAUSE)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_PP_CHKSEL(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_PP_CHKSEL(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetPausePulseEnable
 * Description   : Enables / disables the detection of pause pulses.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetPausePulseEnable(const uint32_t instance, const uint8_t channel, const srx_diag_pause_pulse_cfg_t stat)
{
    /* No pre clearing required */
    if(stat == SRX_PAUSE_PULSE_ENABLED)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_PAUSE_EN(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_PAUSE_EN(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetSuccCalChk
 * Description   : Sets the value of the Successive Calibration Check parameter
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetSuccCalChk(const uint32_t instance, const uint8_t channel, const srx_diag_succ_cal_check_cfg_t type)
{
    /* No pre clearing required */
    if(type == SRX_SUCC_CAL_CHK_PREFFERED)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_SUCC_CAL_CHK(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_SUCC_CAL_CHK(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetSlowCrcType
 * Description   : Sets the value of the Slow Message CRC Type parameter
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetSlowCrcType(const uint32_t instance, const uint8_t channel, const srx_msg_crc_t type)
{
    /* No pre clearing required */
    if(type == SRX_CRC_LEGACY)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_SCRC_TYPE(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_SCRC_TYPE(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastCrcType
 * Description   : Sets the value of the Fast Message CRC Type parameter
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastCrcType(const uint32_t instance, const uint8_t channel, const srx_msg_crc_t type)
{
    /* No pre clearing required */
    if(type == SRX_CRC_LEGACY)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_FCRC_TYPE(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_FCRC_TYPE(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastCrcIncStatus
 * Description   : Sets the Include Status Nibble in CRC calculation parameter.
 * This is valid only for Fast messages.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastCrcIncStatus(const uint32_t instance, const uint8_t channel, const bool status)
{
    /* No pre clearing required */
    if(status)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_FCRC_SC_EN(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_FCRC_SC_EN(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastDisableCrc
 * Description   : Sets the enabling / disabling of CRC verification for Fast
 * messages
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastDisableCrc(const uint32_t instance, const uint8_t channel, const bool status)
{
    /* No pre clearing required */
    if(status)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_FCRC_CHK_OFF(1u);
    }
    else
    {
        s_srxBase[instance]->CH[channel].CONFIG &= ~SRX_CONFIG_FCRC_CHK_OFF(1u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastNumNibbles
 * Description   : Configures the number of nibbles for the Fast channel.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastNumNibbles(const uint32_t instance, const uint8_t channel, const uint8_t num)
{
    uint32_t locVal = (num < (uint8_t)1u) ? (uint32_t)1u : ((num > (uint8_t)6u) ? (uint32_t)6u : (uint32_t)num);

    /* Clear the current value */
    s_srxBase[instance]->DATA_CTRL1 &= ~(SRX_DATA_CTRL1_NIBBCH0_MASK >> (4u * channel));

    /* Write it back */
    s_srxBase[instance]->DATA_CTRL1 |= (locVal << (28u - (4u * channel))); /* Register layout */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastDma
 * Description   : Enables / disables DMA transfers for the given Fast channel.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastDma(const uint32_t instance, const uint8_t channel, const bool enable)
{
    /* Just mask the bit */
    /* Predefined masks not useable */
    if(enable)
    {
        s_srxBase[instance]->FDMA_CTRL |= (uint32_t)((uint32_t)1u << channel);
    }
    else
    {
        s_srxBase[instance]->FDMA_CTRL &= (uint32_t)(~((uint32_t)1u << channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetSlowDma
 * Description   : Enables / disables DMA transfers for the given Slow channel.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetSlowDma(const uint32_t instance, const uint8_t channel, const bool enable)
{
    /* Just mask the bit */
    /* Predefined masks not useable */
    if(enable)
    {
        s_srxBase[instance]->SDMA_CTRL |= (uint32_t)((uint32_t)1u << channel);
    }
    else
    {
        s_srxBase[instance]->SDMA_CTRL &= (uint32_t)(~((uint32_t)1u << channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetChannelStatus
 * Description   : Enables / disables reception for the given channels.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetChannelStatus(const uint32_t instance, const uint8_t channel, const bool enable)
{
    if(enable)
    {
        s_srxBase[instance]->CHNL_EN |= (uint32_t)((uint32_t)1u << channel);
    }
    else
    {
        s_srxBase[instance]->CHNL_EN &= (uint32_t)(~((uint32_t)1u << channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetPeripheralStatus
 * Description   : Enables / disables the peripheral
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetPeripheralStatus(const uint32_t instance, const bool enable)
{
    if(enable)
    {
        s_srxBase[instance]->GBL_CTRL |= SRX_GBL_CTRL_SENT_EN(1u);
    }
    else
    {
        s_srxBase[instance]->GBL_CTRL &= ~SRX_GBL_CTRL_SENT_EN(1u);
    }
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetEventConfig
 * Description   : Enables events received in a mask. These events will signal
 * diagnostics notifications to the application.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetEventConfig(const uint32_t instance, const uint8_t channel, const srx_event_t events)
{
    /* Clear all masks first */
    s_srxBase[instance]->CH[channel].CONFIG &= ~(SRX_CONFIG_IE_CAL_RESYNC_MASK
                                     | SRX_CONFIG_IE_CAL_20_25_MASK
                                     | SRX_CONFIG_IE_SMSG_OFLW_MASK
                                     | SRX_CONFIG_IE_FMSG_OFLW_MASK
                                     | SRX_CONFIG_IE_PP_DIAG_ERR_MASK
                                     | SRX_CONFIG_IE_CAL_LEN_ERR_MASK
                                     | SRX_CONFIG_IE_CAL_DIAG_ERR_MASK
                                     | SRX_CONFIG_IE_NIB_VAL_ERR_MASK
                                     | SRX_CONFIG_IE_SMSG_CRC_ERR_MASK
                                     | SRX_CONFIG_IE_FMSG_CRC_ERR_MASK
                                     | SRX_CONFIG_IE_NUM_EDGES_ERR_MASK);

    /* One by one enablers */
    if((events & (srx_event_t)SRX_EV_CAL_RESYNC) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_CAL_RESYNC(1u);
    }

    if((events & (srx_event_t)SRX_EV_CAL_20_25) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_CAL_20_25(1u);
    }

    if((events & (srx_event_t)SRX_EV_SMSG_OFLW) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_SMSG_OFLW(1u);
    }

    if((events & (srx_event_t)SRX_EV_PP_DIAG_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_PP_DIAG_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_CAL_DIAG_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_CAL_DIAG_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_SMSG_CRC_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_SMSG_CRC_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_NUM_EDGES_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_NUM_EDGES_ERR(1u);
    }

#if defined(ERRATA_E7425) /* Enable anyway so we can implement the workaround */

    s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_NIB_VAL_ERR(1u);
    s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_FMSG_CRC_ERR(1u);
    s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_CAL_LEN_ERR(1u);
    s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_FMSG_OFLW(1u);

#else /* Normal behavior */

    if((events & (srx_event_t)SRX_EV_NIB_VAL_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_NIB_VAL_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_FMSG_CRC_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_FMSG_CRC_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_CAL_LEN_ERR) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_CAL_LEN_ERR(1u);
    }

    if((events & (srx_event_t)SRX_EV_FMSG_OFLW) != 0u)
    {
        s_srxBase[instance]->CH[channel].CONFIG |= SRX_CONFIG_IE_FMSG_OFLW(1u);
    }

#endif /* defined(ERRATA_E7425) */
 }

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetActiveEvents
 * Description   : Returns current active diagnostics events
 *
 *END**************************************************************************/
srx_event_t SRX_DRV_HW_GetActiveEvents(const uint32_t instance, const uint8_t channel)
{
    uint32_t regState = s_srxBase[instance]->CH[channel].STATUS;
    uint32_t regGlobal = s_srxBase[instance]->GBL_STATUS;
    srx_event_t evTemp = 0u;

    /* Put events in temp var */
    if((regState & (uint32_t)SRX_STATUS_BUS_IDLE_MASK) != 0u)
    {
        evTemp |= SRX_EV_BUS_IDLE;
    }

    if((regState & (uint32_t)SRX_STATUS_CAL_RESYNC_MASK) != 0u)
    {
        evTemp |= SRX_EV_CAL_RESYNC;
    }

    if((regState & (uint32_t)SRX_STATUS_CAL_20_25_MASK) != 0u)
    {
        evTemp |= SRX_EV_CAL_20_25;
    }

    if((regState & (uint32_t)SRX_STATUS_SMSG_OFLW_MASK) != 0u)
    {
        evTemp |= SRX_EV_SMSG_OFLW;
    }

    if((regState & (uint32_t)SRX_STATUS_FMSG_OFLW_MASK) != 0u)
    {
        evTemp |= SRX_EV_FMSG_OFLW;
    }

    if((regState & (uint32_t)SRX_STATUS_PP_DIAG_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_PP_DIAG_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_CAL_LEN_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_CAL_LEN_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_CAL_DIAG_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_CAL_DIAG_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_NIB_VAL_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_NIB_VAL_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_SMSG_CRC_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_SMSG_CRC_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_FMSG_CRC_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_FMSG_CRC_ERR;
    }

    if((regState & (uint32_t)SRX_STATUS_NUM_EDGES_ERR_MASK) != 0u)
    {
        evTemp |= SRX_EV_NUM_EDGES_ERR;
    }

#ifdef FEATURE_SRX_DMA_HAS_FIFO
    if((regGlobal & (uint32_t)SRX_GBL_STATUS_FMFO_MASK) != 0u)
    {
        evTemp |= SRX_EV_FIFO_OVERFLOW;
    }
#endif

    if((regGlobal & (uint32_t)SRX_GBL_STATUS_FMDU_MASK) != 0u)
    {
        evTemp |= SRX_EV_FDMA_UNDERFLOW;
    }

    if((regGlobal & (uint32_t)SRX_GBL_STATUS_SMDU_MASK) != 0u)
    {
        evTemp |= SRX_EV_SDMA_UNDERFLOW;
    }

    return evTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_ClearActiveEvents
 * Description   : Clears the flags for the events in the given mask.
 *
 *END**************************************************************************/
void SRX_DRV_HW_ClearActiveEvents(const uint32_t instance, const uint8_t channel, const srx_event_t events)
{
    uint32_t clrMask = 0u;
    uint32_t gClrMask = 0u; /* For global events */

    /* Put events in temp var */
    if((events & (srx_event_t)SRX_EV_BUS_IDLE) != 0u)
    {
        clrMask |= SRX_STATUS_BUS_IDLE_MASK;
    }

    if((events & (srx_event_t)SRX_EV_CAL_RESYNC) != 0u)
    {
        clrMask |= SRX_STATUS_CAL_RESYNC_MASK;
    }

    if((events & (srx_event_t)SRX_EV_CAL_20_25) != 0u)
    {
        clrMask |= SRX_STATUS_CAL_20_25_MASK;
    }

    if((events & (srx_event_t)SRX_EV_SMSG_OFLW) != 0u)
    {
        clrMask |= SRX_STATUS_SMSG_OFLW_MASK;
    }

    if((events & (srx_event_t)SRX_EV_FMSG_OFLW) != 0u)
    {
        clrMask |= SRX_STATUS_FMSG_OFLW_MASK;
    }

    if((events & (srx_event_t)SRX_EV_PP_DIAG_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_PP_DIAG_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_CAL_LEN_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_CAL_LEN_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_CAL_DIAG_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_CAL_DIAG_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_NIB_VAL_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_NIB_VAL_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_SMSG_CRC_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_SMSG_CRC_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_FMSG_CRC_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_FMSG_CRC_ERR_MASK;
    }

    if((events & (srx_event_t)SRX_EV_NUM_EDGES_ERR) != 0u)
    {
        clrMask |= SRX_STATUS_NUM_EDGES_ERR_MASK;
    }

    /* RMW the channel status register */
    s_srxBase[instance]->CH[channel].STATUS |= clrMask;

#ifdef FEATURE_SRX_DMA_HAS_FIFO
    if((events & (srx_event_t)SRX_EV_FIFO_OVERFLOW) != 0u)
    {
        gClrMask |= SRX_GBL_STATUS_FMFO_MASK;
    }
#endif /* FEATURE_SRX_DMA_HAS_FIFO */

    if((events & (srx_event_t)SRX_EV_FDMA_UNDERFLOW) != 0u)
    {
        gClrMask |= SRX_GBL_STATUS_FMDU_MASK;
    }

    if((events & (srx_event_t)SRX_EV_SDMA_UNDERFLOW) != 0u)
    {
        gClrMask |= SRX_GBL_STATUS_SMDU_MASK;
    }

    /* Global RMW */
    s_srxBase[instance]->GBL_STATUS |= gClrMask;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetFastRxInterrupt
 * Description   : Enables / disables Interrupts for the Rx
 * event on the Slow channel.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetFastRxInterrupt(const uint32_t instance, const uint8_t channel, const bool enable)
{
    /* Just mask the bit */
    /* Predefined masks not usable */
    if(enable)
    {
        s_srxBase[instance]->FRDY_IE |= (uint32_t)((uint32_t)1u << channel);
    }
    else
    {
        s_srxBase[instance]->FRDY_IE &= (uint32_t)(~((uint32_t)1u << channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_SetSlowRxInterruptStatus
 * Description   : Enables / disables Interrupts for the Rx
 * event on the Slow channel.
 *
 *END**************************************************************************/
void SRX_DRV_HW_SetSlowRxInterruptStatus(const uint32_t instance, const uint8_t channel, const bool enable)
{
    /* Just mask the bit */
    /* Predefined masks not usable */
    if(enable)
    {
        s_srxBase[instance]->SRDY_IE |= (uint32_t)((uint32_t)1u << channel);
    }
    else
    {
        s_srxBase[instance]->SRDY_IE &= (uint32_t)(~((uint32_t)1u << channel));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetFastRxStatus
 * Description   : Returns current Rx status for the Fast channel.
 * This will return true if there
 * is a new message present in the buffer.
 *
 *END**************************************************************************/
bool SRX_DRV_HW_GetFastRxStatus(const uint32_t instance, const uint32_t channel)
{
    /* Just pass the status */
    return (((uint32_t)(s_srxBase[instance]->FMSG_RDY & ((uint32_t)1u << channel)) != 0u) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetSlowRxStatus
 * Description   : Returns current Rx status for the Slow channel.
 * This will return true if there
 * is a new message present in the buffer.
 *
 *END**************************************************************************/
bool SRX_DRV_HW_GetSlowRxStatus(const uint32_t instance, const uint32_t channel)
{
    /* Just pass the status */
    return (((uint32_t)(s_srxBase[instance]->SMSG_RDY & ((uint32_t)1u << channel)) != 0u) ? true : false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetFastRawMsg
 * Description   : Returns current Fast message in the raw format (unformatted).
 *
 *END**************************************************************************/
void SRX_DRV_HW_GetFastRawMsg(const uint32_t instance, const uint8_t channel, srx_raw_msg_t * rawMsg)
{
    /* Populate according to registers */
    rawMsg->dataField0 = s_srxBase[instance]->CHANNEL[channel].FMSG_DATA;
    rawMsg->dataField1 = s_srxBase[instance]->CHANNEL[channel].FMSG_CRC;
    rawMsg->dataField2 = s_srxBase[instance]->CHANNEL[channel].FMSG_TS;

    /* Clear the RDY channels */
    s_srxBase[instance]->FMSG_RDY |= (uint32_t)((uint32_t)1u << channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetSlowRawMsg
 * Description   : Returns current Slow message in the raw format (unformatted).
 *
 *END**************************************************************************/
void SRX_DRV_HW_GetSlowRawMsg(const uint32_t instance, const uint8_t channel, srx_raw_msg_t * rawMsg)
{
    /* Populate according to registers */
    rawMsg->dataField0 = s_srxBase[instance]->CHANNEL[channel].SMSG_BIT3;
    rawMsg->dataField1 = s_srxBase[instance]->CHANNEL[channel].SMSG_BIT2;
    rawMsg->dataField2 = s_srxBase[instance]->CHANNEL[channel].SMSG_TS;

    /* Clear the RDY channels */
    s_srxBase[instance]->SMSG_RDY |= (uint32_t)((uint32_t)1u << channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_ConvertFastRaw
 * Description   : Converts an unformatted (raw) Fast message
 * into a formatted one.
 *
 *END**************************************************************************/
void SRX_DRV_HW_ConvertFastRaw(srx_fast_msg_t * msg, const srx_raw_msg_t * rawMsg)
{
    /* FMSG_DATA register */
    msg->channelNumber = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_FMSG_DATA_CHNUM_MASK) >> SRX_FMSG_DATA_CHNUM_SHIFT);
    msg->statusField = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_FMSG_DATA_SCNIB_MASK) >> SRX_FMSG_DATA_SCNIB_SHIFT);

    /* Get full data and then shift depending on configured number of nibbles */
    msg->data = rawMsg->dataField0 & (SRX_FMSG_DATA_DNIB1_MASK
                                        | SRX_FMSG_DATA_DNIB2_MASK
                                        | SRX_FMSG_DATA_DNIB3_MASK
                                        | SRX_FMSG_DATA_DNIB4_MASK
                                        | SRX_FMSG_DATA_DNIB5_MASK
                                        | SRX_FMSG_DATA_DNIB6_MASK);

    /* FMSG_CRC register */
    msg->crc = (uint8_t)((uint32_t)(rawMsg->dataField1 & SRX_FMSG_CRC_CRC4b_MASK) >> SRX_FMSG_CRC_CRC4b_SHIFT);

    /* FMSG TS register */
    msg->timeStamp = rawMsg->dataField2;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_ConvertSlowRaw
 * Description   : Converts an unformatted (raw) Slow message
 * into a formatted one.
 *
 *END**************************************************************************/
void SRX_DRV_HW_ConvertSlowRaw(srx_slow_msg_t * msg, const srx_raw_msg_t * rawMsg)
{
    srx_slow_msg_type_t locType;

    /* Channel number */
    msg->channelNumber = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_CHNUM_MASK) >> SRX_SMSG_BIT3_CHNUM_SHIFT);

    /* CRC */
    msg->crc = (uint8_t)((uint32_t)(rawMsg->dataField1 & SRX_SMSG_BIT2_SMCRC_MASK) >> SRX_SMSG_BIT2_SMCRC_SHIFT);

    /* Timestamp */
    msg->timeStamp = rawMsg->dataField2;

    /* Lower 12 bits of data */
    msg->data = (uint16_t)(rawMsg->dataField1 & SRX_SMSG_BIT2_DATA_MASK);

    /* This is more complicated due to message type */
    if((rawMsg->dataField0 & (uint32_t)SRX_SMSG_BIT3_TYPE_MASK) != 0u)
    {
        /* Enhanced */
        if((rawMsg->dataField0 & (uint32_t)SRX_SMSG_BIT3_CFG_MASK) != 0u)
        {
            /* 4 bit ID, C = 1 */
            locType = SRX_SLOW_TYPE_ENHANCED_4_BIT;
            msg->data |= (uint16_t)((uint32_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_ID3_0_DATA15_12_MASK)
                            >> SRX_SMSG_BIT3_ID3_0_DATA15_12_SHIFT) << 12u);
            msg->id = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_ID7_4_ID3_0_MASK)
                            >> SRX_SMSG_BIT3_ID7_4_ID3_0_SHIFT);
        }
        else
        {
            /* 8 bit ID, C = 0 */
            locType = SRX_SLOW_TYPE_ENHANCED_8_BIT;
            msg->id = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_ID3_0_DATA15_12_MASK)
                            >> SRX_SMSG_BIT3_ID3_0_DATA15_12_SHIFT);
            msg->id |= (uint8_t)((uint32_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_ID7_4_ID3_0_MASK)
                            >> SRX_SMSG_BIT3_ID7_4_ID3_0_SHIFT) << 4u);
        }
    }
    else
    {
        /* Standard */
        locType = SRX_SLOW_TYPE_SHORT;
        msg->id = (uint8_t)((uint32_t)(rawMsg->dataField0 & SRX_SMSG_BIT3_ID7_4_ID3_0_MASK)
                            >> SRX_SMSG_BIT3_ID7_4_ID3_0_SHIFT);
    }

    /* Message type */
    msg->type = locType;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_ResetPeripheral
 * Description   : Disables the peripheral and brings it's
 * register into a default state
 *
 *END**************************************************************************/
void SRX_DRV_HW_ResetPeripheral(const uint32_t instance)
{
    uint8_t chInd;

    /* Peripheral wide */
    s_srxBase[instance]->GBL_CTRL = SRX_GBL_CTRL_RESET_VALUE;
    s_srxBase[instance]->CHNL_EN = SRX_CHNL_EN_RESET_VALUE;
    s_srxBase[instance]->DATA_CTRL1 = SRX_DATA_CTRL1_RESET_VALUE;
    s_srxBase[instance]->FDMA_CTRL = SRX_FDMA_CTRL_RESET_VALUE;
    s_srxBase[instance]->SDMA_CTRL = SRX_SDMA_CTRL_RESET_VALUE;
    s_srxBase[instance]->FRDY_IE = SRX_FRDY_IE_RESET_VALUE;
    s_srxBase[instance]->SRDY_IE = SRX_SRDY_IE_RESET_VALUE;

    /* For each channel */
    for(chInd = 0u; chInd < SRX_CHANNEL_COUNT; chInd++)
    {
        s_srxBase[instance]->CH[chInd].CLK_CTRL = SRX_CHn_CLK_CTRL_RESET_VALUE;
        s_srxBase[instance]->CH[chInd].CONFIG = SRX_CHn_CONFIG_RESET_VALUE;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetSlowDmaRegStartAddr
 * Description   : Returns the start address for the Fast DMA buffer
 *
 *END**************************************************************************/
volatile const uint32_t * SRX_DRV_HW_GetSlowDmaRegStartAddr(const uint32_t instance)
{
    return &s_srxBase[instance]->DMA_SMSG_BIT3;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SRX_DRV_HW_GetFastDmaRegStartAddr
 * Description   : Returns the start address for the Slow DMA buffer
 *
 *END**************************************************************************/
volatile const uint32_t * SRX_DRV_HW_GetFastDmaRegStartAddr(const uint32_t instance)
{
    return &s_srxBase[instance]->DMA_FMSG_DATA;
}

/*******************************************************************************
* EOF
*******************************************************************************/
