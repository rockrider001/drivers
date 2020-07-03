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
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.13, Pointer parameter could be declared as pointing to const
 * This is a pointer to the driver context structure which is for internal use only, and the application
 * must make no assumption about the content of this structure. Therefore it is irrelevant for the application
 * whether the structure is changed in the function or not. The fact that in a particular implementation of some
 * functions there is no write in the context structure is an implementation detail and there is no reason to
 * propagate it in the interface. That would compromise the stability of the interface, if this implementation
 * detail is changed in later releases or on other platforms.
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 */

#include "emios_oc_driver.h"
#include "emios_mc_driver.h"
#include "emios_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Static functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_InitModeSaoc
 * Description   : initializer, Output mode : Saoc
 *END**************************************************************************/
static void EMIOS_DRV_OC_InitModeSaoc(uint8_t emiosGroup,
                                      uint8_t channel,
                                      const emios_oc_param_t *ocParam)
{
    /* Validate matchValue <= Period of using bus */
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(ocParam->matchLeadingEdgeValue <= EMIOS_DATA_REG_MAX_VAL);

    /* Valid SAOC with channels supported- All channel can run this mode. Done */
    /* Configure registers */
    EMIOS_SetUCRegA(emiosGroup, channel, ocParam->matchLeadingEdgeValue);
    EMIOS_SetUCRegCEdsel(emiosGroup, channel, ((((uint8_t)ocParam->outputActiveMode & 0x02U) == 0U) ? 0UL : 1UL));
    EMIOS_SetUCRegCEdpol(emiosGroup, channel, ((((uint8_t)ocParam->outputActiveMode & 0x01U) == 0U) ? 0UL : 1UL));

}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_InitModeEdge
 * Description   : initializer, Output mode : Edge
 *END**************************************************************************/
static void EMIOS_DRV_OC_InitModeEdge(uint8_t emiosGroup,
                                      uint8_t channel,
                                      const emios_oc_param_t *ocParam)
{
    /* Validate matchLeadingEdgeValue and matchTrailingEdgeValue <= Period of using bus */
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(ocParam->matchLeadingEdgeValue <= EMIOS_DATA_REG_MAX_VAL);
    DEV_ASSERT(ocParam->matchTrailingEdgeValue <= EMIOS_DATA_REG_MAX_VAL);
    DEV_ASSERT(ocParam->outputActiveMode != EMIOS_OUTPUT_ACTIVE_TOGGLE);

#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    /* Valid DAOC with channels supported */
    if (EMIOS_ValidateMode(emiosGroup, channel, (uint8_t)EMIOS_GMODE_DAOC) == false)
    {
        DEV_ASSERT(false);
    }
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */

    /* Configure registers */
    EMIOS_SetUCRegA(emiosGroup, channel, ocParam->matchLeadingEdgeValue);
    EMIOS_SetUCRegB(emiosGroup, channel, ocParam->matchTrailingEdgeValue);

    EMIOS_SetUCRegCEdpol(emiosGroup, channel, (uint32_t)ocParam->outputActiveMode);
}


/******************************************************************************
* API
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_InitOutputCompareMode
 * Description   : Initialize Output Compare Mode.
 * In the DAOC mode, the leading and trailing edges of the variable pulse width output
 * are generated by matches occurring on 2 comparators.
 * In SAOC mode, a match value is loaded and then to be compared with the selected time base.
 * When a match occurs, the output flip-flop is toggled or the value (set by outputActiveMode)
 * is transferred to it.
 * Implements    : EMIOS_DRV_OC_InitOutputCompareMode_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_OC_InitOutputCompareMode(uint8_t emiosGroup,
                                            uint8_t channel,
                                            const emios_oc_param_t *ocParam)
{
    uint8_t restChannel = 0U;
    uint8_t busSelect = channel;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);

#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(ocParam != NULL);
    DEV_ASSERT((ocParam->mode == EMIOS_MODE_DAOC_FSET_TRAILING_MATCH) || \
               (ocParam->mode == EMIOS_MODE_DAOC_FSET_BOTH_MATCH) || \
               (ocParam->mode == EMIOS_MODE_SAOC));

#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    uint32_t temp;

    /* Get mode of operation of the Unified Channel */
    temp = EMIOS_GetUCRegCMode( emiosGroup, restChannel);
#ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL
    bool validMCMode = false;
    bool validMCBMode = false;

    if (ocParam->timebase == EMIOS_BUS_SEL_INTERNAL)
    {
        /* Choose internal counter, this channel mode should be Type X or G */
        DEV_ASSERT(EMIOS_ValidateInternalCnt(emiosGroup, channel));
    }
    /* Valid MC or MCB with channel supported */
    validMCMode  = EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_MC);
    validMCBMode = EMIOS_ValidateMode(emiosGroup, restChannel, (uint8_t)EMIOS_GMODE_MCB);

    DEV_ASSERT((eMIOS[emiosGroup]->UC[restChannel].C == 0UL) ||
               (((validMCMode == true) || (validMCBMode == true)) && (temp == (uint32_t)EMIOS_MODE_MCB_UP_COUNTER_INT_CLK)));
#else
    /* Check that device was initialized */
    DEV_ASSERT((eMIOS[emiosGroup]->UC[restChannel].C == 0UL) || (temp == (uint32_t)EMIOS_MODE_MCB_UP_COUNTER_INT_CLK));
#endif /* ifndef FEATURE_EMIOS_ALL_MODE_ON_CHANNEL */
#endif /* if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT) */

    if (ocParam->mode == EMIOS_MODE_SAOC)
    {
        EMIOS_DRV_OC_InitModeSaoc(emiosGroup, restChannel, ocParam);
    }
    else
    {
        EMIOS_DRV_OC_InitModeEdge(emiosGroup, restChannel, ocParam);
    }

    EMIOS_SetUCRegCBsl(emiosGroup, restChannel, (uint32_t)ocParam->timebase);
    EMIOS_SetUCRegCMode(emiosGroup, restChannel, (uint32_t)ocParam->mode);

    if (ocParam->mode != EMIOS_MODE_SAOC)
    {
        /* Configure registers for DAOC mode, to generate an output pulse,
           this mode need to be updated match value first. */
        EMIOS_SetUCRegA(emiosGroup, restChannel, ocParam->matchLeadingEdgeValue);
        EMIOS_SetUCRegB(emiosGroup, restChannel, ocParam->matchTrailingEdgeValue);
    }
    else
    {
        if(ocParam->outputActiveMode == EMIOS_OUTPUT_ACTIVE_DISABLE)
        {
#ifndef FEATURE_EMIOS_UC_DISABLE
            if (ocParam->timebase == EMIOS_BUS_SEL_A)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSA_DRIVEN;
            }
#if FEATURE_EMIOS_BUS_F_SELECT
            else if (ocParam->timebase == EMIOS_BUS_SEL_F)
            {
                busSelect = (uint8_t)EMIOS_CNT_BUSF_DRIVEN;
            }
#endif
            else if (ocParam->timebase == EMIOS_BUS_SEL_BCDE)
            {
                busSelect = (uint8_t)(channel & 0xF8U);
            }
            else
            {
                busSelect = channel;
            }
#endif
            EMIOS_DRV_OutputDisable(emiosGroup, busSelect);
        }
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_SetSingleActOutputCmpMatch
 * Description   : Update new Single Action Output Compare Match value.
 * SAOC Match value can be updated at any time thus modifying the match value
 * which will reflect in the output signal generated by the channel.
 * Implements    : EMIOS_DRV_OC_SetSingleActOutputCmpMatch_Activity
 *END**************************************************************************/
void EMIOS_DRV_OC_SetSingleActOutputCmpMatch(uint8_t emiosGroup,
                                             uint8_t channel,
                                             uint32_t newMatchValue)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(EMIOS_GetUCRegCMode(emiosGroup, restChannel) == (uint32_t)EMIOS_MODE_SAOC);

    /* Validate newMatchValue <= Period of using bus */
    DEV_ASSERT(newMatchValue <= EMIOS_DATA_REG_MAX_VAL);

    /* Write new match value to A register */
    EMIOS_SetUCRegA(emiosGroup, restChannel, newMatchValue);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_GetSingleActOutputCmpMatch
 * Description   : Get Match value in SAOC mode
 * Implements    : EMIOS_DRV_OC_GetSingleActOutputCmpMatch_Activity
 *END**************************************************************************/
uint32_t EMIOS_DRV_OC_GetSingleActOutputCmpMatch(uint8_t emiosGroup,
                                                 uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(EMIOS_GetUCRegCMode(emiosGroup, restChannel) == (uint32_t)EMIOS_MODE_SAOC);

    return EMIOS_GetUCRegA(emiosGroup, restChannel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_ForceSingleActOutputCmpMatch
 * Description   : An output compare match can be simulated in software by call
 * EMIOS_DRV_OC_ForceSingleActOutputCmpMatch.
 * In this case, the FLAG bit is not set and have no trigger raised.
 * Implements    : EMIOS_DRV_OC_ForceSingleActOutputCmpMatch_Activity
 *END**************************************************************************/
void EMIOS_DRV_OC_ForceSingleActOutputCmpMatch(uint8_t emiosGroup,
                                               uint8_t channel)
{
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(EMIOS_GetUCRegCMode(emiosGroup, restChannel) == (uint32_t)EMIOS_MODE_SAOC);

    EMIOS_SetUCRegCForcma(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_SetDoubleActOutputCmpMatch
 * Description   : Update Leading edge and Trailing edge position in DAOC mode.
 * Implements    : EMIOS_DRV_OC_SetDoubleActOutputCmpMatch_Activity
 *END**************************************************************************/
void EMIOS_DRV_OC_SetDoubleActOutputCmpMatch(uint8_t emiosGroup,
                                             uint8_t channel,
                                             uint32_t newLeadingEdgeVal,
                                             uint32_t newTrailingEdgeVal)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_DAOC_FSET_TRAILING_MATCH) || \
               (temp == (uint32_t)EMIOS_MODE_DAOC_FSET_BOTH_MATCH));
    (void) temp;

    /* Validate matchLeadingEdgeValue and matchTrailingEdgeValue <= Period of using bus */
    DEV_ASSERT(newLeadingEdgeVal <= EMIOS_DATA_REG_MAX_VAL);
    DEV_ASSERT(newTrailingEdgeVal <= EMIOS_DATA_REG_MAX_VAL);

    EMIOS_SetUCRegA(emiosGroup, restChannel, newLeadingEdgeVal);
    EMIOS_SetUCRegB(emiosGroup, restChannel, newTrailingEdgeVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_GetDoubleActOutputCmpMatch
 * Description   : Get DAOC Match value
 * Implements    : EMIOS_DRV_OC_GetDoubleActOutputCmpMatch_Activity
 *END**************************************************************************/
status_t EMIOS_DRV_OC_GetDoubleActOutputCmpMatch(uint8_t emiosGroup,
                                                 uint8_t channel,
                                                 uint32_t * const retLeadingEdgeVal,
                                                 uint32_t * const retTrailingEdgeVal)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    DEV_ASSERT(retLeadingEdgeVal != NULL);
    DEV_ASSERT(retTrailingEdgeVal != NULL);

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_DAOC_FSET_TRAILING_MATCH) || \
               (temp == (uint32_t)EMIOS_MODE_DAOC_FSET_BOTH_MATCH));
    (void) temp;

    *retLeadingEdgeVal  = EMIOS_GetUCRegA(emiosGroup, restChannel);
    *retTrailingEdgeVal = EMIOS_GetUCRegB(emiosGroup, restChannel);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_ForceDoubleActOutputCmpLeadingMatch
 * Description   : Allow the software to force the output flipflop to the level corresponding
 * to a comparison event in comparator A.
 * Note that the FLAG bit is not affected by these forced operations.
 * Implements    : EMIOS_DRV_OC_ForceDoubleActOutputCmpLeadingMatch_Activity
 *END**************************************************************************/
void EMIOS_DRV_OC_ForceDoubleActOutputCmpLeadingMatch(uint8_t emiosGroup,
                                                      uint8_t channel)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif

    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_DAOC_FSET_TRAILING_MATCH) || \
               (temp == (uint32_t)EMIOS_MODE_DAOC_FSET_BOTH_MATCH));
    (void) temp;

    EMIOS_SetUCRegCForcma(emiosGroup, restChannel, 1UL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EMIOS_DRV_OC_ForceDoubleActOutputCmpTrailingMatch
 * Description   : Allow the software to force the output flipflop to the level corresponding
 * to a comparison event in comparator B.
 * Note that the FLAG bit is not affected by these forced operations.
 * Implements    : EMIOS_DRV_OC_ForceDoubleActOutputCmpTrailingMatch_Activity
 *END**************************************************************************/
void EMIOS_DRV_OC_ForceDoubleActOutputCmpTrailingMatch(uint8_t emiosGroup,
                                                       uint8_t channel)
{
    uint32_t temp;
    uint8_t restChannel = 0U;
    bool restValidate   = EMIOS_ValidateChannel(channel, &restChannel);
    DEV_ASSERT(emiosGroup < EMIOS_NUMBER_GROUP_MAX);
    DEV_ASSERT(restValidate == true);
#ifndef DEV_ERROR_DETECT
    (void)restValidate;
#endif
    temp = EMIOS_GetUCRegCMode(emiosGroup, restChannel);

    DEV_ASSERT((temp == (uint32_t)EMIOS_MODE_DAOC_FSET_TRAILING_MATCH) || \
               (temp == (uint32_t)EMIOS_MODE_DAOC_FSET_BOTH_MATCH));
    (void) temp;

    EMIOS_SetUCRegCForcmb(emiosGroup, restChannel, 1UL);
}

/*******************************************************************************
* EOF
******************************************************************************/
