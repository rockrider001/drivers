/*
 * Copyright 2019 NXP
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
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include "emios_qd_driver.h"
#include "emios_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_QD_IpfCalibrationChannel
 * Description   : Calibration channel for input programmer filter.
 *
 *END**************************************************************************/
static uint8_t EMIOS_DRV_QD_IpfCalibrationChannel(uint8_t channel)
{
    uint8_t restChannel = 0U;
    uint8_t tmpChannel  = 0U;
    if (channel > 0U)
    {
        (void)tmpChannel;
        restChannel = channel - 1U;
    }
    else
    {
#if FEATURE_EMIOS_BUS_E_SELECT
        tmpChannel = 31U;
#elif FEATURE_EMIOS_BUS_D_SELECT
        tmpChannel = 23U;
#elif FEATURE_EMIOS_BUS_C_SELECT
        tmpChannel = 15U;
#else
        tmpChannel = 7U;
#endif
        (void)EMIOS_ValidateChannel(tmpChannel, &restChannel);
    }

    return restChannel;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_QuadDecodeStart
 * Description   : Initialization EMIOS Quad decode mode and started measurement.
 *
 * Implements    : EMIOS_DRV_QuadDecodeStart_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_QuadDecodeStart(uint8_t emiosGroup,
                                   uint8_t channel,
                                   const emios_qd_config_t *qdParam)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(qdParam != NULL);
    status_t status = STATUS_SUCCESS;
    uint8_t ipfChannel = EMIOS_DRV_QD_IpfCalibrationChannel(restChannel);

    if ((qdParam->mode == EMIOS_MODE_COUNT_AND_DIR_ENCODE) ||
        (qdParam->mode == EMIOS_MODE_PHASE_ENCODE))
    {
        /* Cleared Configure registers */
        eMIOS[emiosGroup]->UC[restChannel].C = 0UL; /* Disable channel pre-scaler (reset default) */
        /* Configuration for Un-1 */
        EMIOS_SetUCRegCEdpol(emiosGroup, ipfChannel, (uint32_t)qdParam->ipfConfig.inputActiveMode);
        EMIOS_SetUCRegCEdsel(emiosGroup, ipfChannel, (uint32_t)qdParam->ipfConfig.triggerMode);
        EMIOS_SetUCRegCIf(emiosGroup, ipfChannel, (uint32_t)qdParam->ipfConfig.filterInput);
        EMIOS_SetUCRegCFck(emiosGroup, ipfChannel, qdParam->ipfConfig.filterEn ? 0UL : 1UL);
#if !defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegCUcpre(emiosGroup, ipfChannel, (uint32_t)qdParam->ipfConfig.internalPrescaler);
#endif
        EMIOS_SetUCRegCUcpren(emiosGroup, ipfChannel, qdParam->ipfConfig.internalPrescalerEn ? 1UL : 0UL);
        /* Configuration for Un */
        EMIOS_SetUCRegCNT(emiosGroup, restChannel, qdParam->initialCounterVal);
        EMIOS_SetUCRegA(emiosGroup, restChannel, qdParam->matchesValue);
        EMIOS_SetUCRegCEdpol(emiosGroup, restChannel, (uint32_t)qdParam->inputActiveMode);
        EMIOS_SetUCRegCEdsel(emiosGroup, restChannel, (uint32_t)qdParam->triggerMode);
        EMIOS_SetUCRegCIf(emiosGroup, restChannel, (uint32_t)qdParam->filterInput);
        EMIOS_SetUCRegCFck(emiosGroup, restChannel, qdParam->filterEn ? 0UL : 1UL);
        EMIOS_SetUCRegCFen(emiosGroup, restChannel, qdParam->enableFlagIsr ? 1UL : 0UL);
#if !defined(FEATURE_EMIOS_PRESCALER_SELECT_BITS)
        EMIOS_SetUCRegCUcpre(emiosGroup, restChannel, (uint32_t)qdParam->internalPrescaler);
#endif
        EMIOS_SetUCRegCUcpren(emiosGroup, restChannel, qdParam->internalPrescalerEn ? 1UL : 0UL);
        EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)qdParam->mode);
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_QuadDecodeStop
 * Description   : De-activates quadrature decoder mode.
 *
 * Implements    : EMIOS_DRV_QuadDecodeStop_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_QuadDecodeStop(uint8_t emiosGroup,
                                  uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    status_t status = STATUS_SUCCESS;
    uint32_t mode = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    if ((mode == (uint32_t)EMIOS_MODE_COUNT_AND_DIR_ENCODE) ||
        (mode == (uint32_t)EMIOS_MODE_PHASE_ENCODE))
    {
        /* Cleared Configure registers */
        eMIOS[emiosGroup]->UC[restChannel].C = 0UL; /* Disable channel pre-scaler (reset default) */
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_QuadDecodeGetState
 * Description   : Return the current quadrature decoder state
 * (counter value, overflow flag and overflow direction)
 *
 * Implements    : EMIOS_DRV_QuadDecodeGetState_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_QuadDecodeGetState(uint8_t emiosGroup,
                                      uint8_t channel,
                                      emios_qd_state_t * const qdState)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(qdState != NULL);
    status_t status = STATUS_SUCCESS;
    uint32_t mode = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    if ((mode == (uint32_t)EMIOS_MODE_COUNT_AND_DIR_ENCODE) ||
        (mode == (uint32_t)EMIOS_MODE_PHASE_ENCODE))
    {
        qdState->counterValue    = EMIOS_GetUCRegCNT(emiosGroup, restChannel);
        qdState->overRun         = EMIOS_GetUCRegSOverrun(emiosGroup, restChannel);
        qdState->overflowCounter = EMIOS_GetUCRegSOverflow(emiosGroup, restChannel);
        qdState->overFlag        = EMIOS_GetUCRegSFlagState(emiosGroup, restChannel);
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_QuadDecodeGetDefaultConfig
 * Description   : This function will get the default configuration values
 * in the structure which is used as a common use-case.
 * Return        : None
 * Implements    : EMIOS_DRV_QuadDecodeGetDefaultConfig_Activity
 *END**************************************************************************/
void EMIOS_DRV_QuadDecodeGetDefaultConfig(emios_qd_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    /* Set default configures for Un-1 */
    config->ipfConfig.inputActiveMode     = EMIOS_NEGATIVE_PULSE;
    config->ipfConfig.triggerMode         = EMIOS_TRIGGER_EDGE_FALLING;
    config->ipfConfig.filterInput         = EMIOS_INPUT_FILTER_BYPASS;
    config->ipfConfig.filterEn            = true;
    config->ipfConfig.internalPrescaler   = EMIOS_CLOCK_DIVID_BY_1;
    config->ipfConfig.internalPrescalerEn = true;
    /* Set default configures for Un */
    config->mode                = EMIOS_MODE_COUNT_AND_DIR_ENCODE;
    config->initialCounterVal   = 0U;
    config->matchesValue        = 1000U;
    config->inputActiveMode     = EMIOS_NEGATIVE_PULSE;
    config->internalPrescaler   = EMIOS_CLOCK_DIVID_BY_1;
    config->internalPrescalerEn = true;
    config->filterInput         = EMIOS_INPUT_FILTER_BYPASS;
    config->filterEn            = true;
    config->triggerMode         = EMIOS_TRIGGER_EDGE_FALLING;
    config->enableFlagIsr       = true;
}

/*******************************************************************************
* EOF
******************************************************************************/
