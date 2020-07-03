/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2019 NXP
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
 * @file edma_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 4.9, Function-like macro defined.
 * The macro is used to merge differences between HWV2 and HWV3.
 */
 
#include "edma_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
 
/*******************************************************************************
 * Definitions
 ******************************************************************************/ 
#ifdef FEATURE_DMA_HWV3
    #define DMA_TCD(X)     BASE_TCD(base, X)
#endif
#ifdef FEATURE_DMA_HWV2
    #define DMA_TCD(X)     base->TCD[X]
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_Init
 * Description   : Initializes eDMA module to known state.
 *END**************************************************************************/
void EDMA_Init(DMA_Type * base)
{
    uint8_t idx;
    /* Clear the bit of CR register */
#ifdef FEATURE_DMA_HWV3
    uint32_t regValTemp;
    regValTemp = BASE_MP(base,CSR);
#ifndef FEATURE_DMA_NOT_SUPPORT_BUFFER_WRITE
    regValTemp &= ~(DMA_MP_CSR_EBW_MASK);
#endif
    regValTemp &= ~(DMA_MP_CSR_EDBG_MASK);
    regValTemp &= ~(DMA_MP_CSR_ERCA_MASK);
    regValTemp &= ~(DMA_MP_CSR_HAE_MASK);
    regValTemp &= ~(DMA_MP_CSR_HALT_MASK);
    regValTemp &= ~(DMA_MP_CSR_GCLC_MASK);
    regValTemp &= ~(DMA_MP_CSR_GMRC_MASK);
    regValTemp &= ~(DMA_MP_CSR_ECX_MASK);
    regValTemp &= ~(DMA_MP_CSR_CX_MASK);
    regValTemp &= ~(DMA_MP_CSR_ACTIVE_ID_MASK);
    regValTemp &= ~(DMA_MP_CSR_ACTIVE_MASK);
    BASE_MP(base,CSR) = regValTemp;
    for (idx = 0U; idx < FEATURE_DMA_CHANNELS; idx++)
    {
        BASE_MP(base,CH_GRPRI[idx]) = 0U;
    }
#endif
#ifdef FEATURE_DMA_HWV2
    uint32_t regValTemp;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_CLM_MASK);
    regValTemp &= ~(DMA_CR_CX_MASK);
    regValTemp &= ~(DMA_CR_ECX_MASK);
    regValTemp &= ~(DMA_CR_EDBG_MASK);
    regValTemp &= ~(DMA_CR_EMLM_MASK);
    regValTemp &= ~(DMA_CR_ERCA_MASK);
    base->CR = regValTemp;
#endif
    for (idx = 0U; idx < FEATURE_DMA_CHANNELS; idx++)
    {
        EDMA_TCDClearReg(base, idx);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_CancelTransfer
 * Description   : Cancels the remaining data transfer.
 *END**************************************************************************/
void EDMA_CancelTransfer(DMA_Type * base)
{
    uint32_t regValTemp;
#ifdef FEATURE_DMA_HWV3
    regValTemp = BASE_MP(base,CSR);
    regValTemp &= ~(DMA_MP_CSR_CX_MASK);
    regValTemp |= DMA_MP_CSR_CX(1U);
    BASE_MP(base,CSR) = regValTemp;
    while ((BASE_MP(base,CSR) & DMA_MP_CSR_CX_MASK) != 0UL)
    {}
#endif
#ifdef FEATURE_DMA_HWV2
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_CX_MASK);
    regValTemp |= DMA_CR_CX(1U);
    base->CR = regValTemp;
    while ((base->CR & DMA_CR_CX_MASK) != 0UL)
    {}
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_CancelTransferWithError
 * Description   : Cancels the remaining data transfer and treat it as error.
 *END**************************************************************************/
void EDMA_CancelTransferWithError(DMA_Type * base)
{
    uint32_t regValTemp;
#ifdef FEATURE_DMA_HWV3
    regValTemp = BASE_MP(base,CSR);
    regValTemp &= ~(DMA_MP_CSR_ECX_MASK);
    regValTemp |= DMA_MP_CSR_ECX(1U);
    BASE_MP(base,CSR) = regValTemp;
    while ((BASE_MP(base,CSR) & DMA_MP_CSR_ECX_MASK) != 0UL)
    {}
#endif
#ifdef FEATURE_DMA_HWV2
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_ECX_MASK);
    regValTemp |= DMA_CR_ECX(1U);
    base->CR = regValTemp;
    while ((base->CR & DMA_CR_ECX_MASK) != 0UL)
    {}
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetHaltOnErrorCmd
 * Description   : Halts or does not halt the eDMA module when an error occurs.
 *END**************************************************************************/
void EDMA_SetHaltOnErrorCmd(DMA_Type * base, bool haltOnError)
{
    uint32_t regValTemp;
#ifdef FEATURE_DMA_HWV3
    regValTemp = BASE_MP(base,CSR);
    regValTemp &= ~(DMA_MP_CSR_HAE_MASK);
    regValTemp |= DMA_MP_CSR_HAE(haltOnError ? 1UL : 0UL);
    BASE_MP(base,CSR) = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_HOE_MASK);
    regValTemp |= DMA_CR_HOE(haltOnError ? 1UL : 0UL);
    base->CR = regValTemp;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetChannelPriority
 * Description   : Sets the eDMA channel priority.
 *END**************************************************************************/
void EDMA_SetChannelPriority(DMA_Type * base, uint8_t channel, edma_channel_priority_t priority)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV3
    volatile uint32_t regValTemp;
    regValTemp = DMA_TCD(channel).CH_PRI;
    regValTemp &= (uint32_t)~(DMA_TCD_CH_PRI_APL_MASK);
    regValTemp |= (uint32_t)DMA_TCD_CH_PRI_APL(priority);
    DMA_TCD(channel).CH_PRI = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    uint8_t regValTemp;
    uint8_t index = (uint8_t)FEATURE_DMA_CHN_TO_DCHPRI_INDEX(channel);
    regValTemp = base->DCHPRI[index];
    regValTemp &= (uint8_t)~(DMA_DCHPRI_CHPRI_MASK);
    regValTemp |= (uint8_t)DMA_DCHPRI_CHPRI(priority);
    base->DCHPRI[index] = regValTemp;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetChannelArbitrationMode
 * Description   : Sets the channel arbitration algorithm.
 *END**************************************************************************/
void EDMA_SetChannelArbitrationMode(DMA_Type * base, edma_arbitration_algorithm_t channelArbitration)
{
    uint32_t regValTemp;
#ifdef FEATURE_DMA_HWV3
    regValTemp = BASE_MP(base,CSR);
    regValTemp &= ~(DMA_MP_CSR_ERCA_MASK);
    regValTemp |= DMA_MP_CSR_ERCA(channelArbitration);
    BASE_MP(base,CSR) = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_ERCA_MASK);
    regValTemp |= DMA_CR_ERCA(channelArbitration);
    base->CR = regValTemp;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_GetChannelArbitrationMode
 * Description   : Gets the channel arbitration algorithm.
 *END**************************************************************************/
edma_arbitration_algorithm_t EDMA_GetChannelArbitrationMode(const DMA_Type * base)
{
    edma_arbitration_algorithm_t retVal;
#ifdef FEATURE_DMA_HWV3
    if ((BASE_MP(base,CSR) & DMA_MP_CSR_ERCA_MASK) != 0U)
#endif
#ifdef FEATURE_DMA_HWV2
    if ((base->CR & DMA_CR_ERCA_MASK) != 0U)
#endif
    {
        retVal = EDMA_ARBITRATION_ROUND_ROBIN;
    }
    else
    {
        retVal = EDMA_ARBITRATION_FIXED_PRIORITY;
    }
    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetErrorIntCmd
 * Description   : Enable/Disable error interrupt for channels.
 *END**************************************************************************/
void EDMA_SetErrorIntCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#ifdef FEATURE_DMA_HWV3
    uint32_t regValTemp;
    regValTemp = DMA_TCD(channel).CH_CSR;
    if (enable)
    {
        regValTemp |= DMA_TCD_CH_CSR_EEI_MASK;
    }
    else
    {
        regValTemp &= ~(DMA_TCD_CH_CSR_EEI_MASK);
    }
    DMA_TCD(channel).CH_CSR = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    if (enable)
    {
        base->SEEI = channel;
    }
    else
    {
        base->CEEI = channel;
    }
#endif
}

#ifdef FEATURE_DMA_HWV2
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_GetErrorIntStatusFlag
 * Description   : Gets the eDMA error interrupt status.
 *END**************************************************************************/
void EDMA_GetErrorIntStatusFlag(const DMA_Type * base, edma_error_register_t * errReg)
{
#if (FEATURE_DMA_CHANNELS > 32U)
    errReg->errh = (uint32_t)base->ERRH;
    errReg->errl = (uint32_t)base->ERRL;
#else
    errReg->errh = 0UL;
    errReg->errl = (uint32_t)base->ERR;
#endif
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_ClearErrorIntStatusFlag
 * Description   : Clears the error interrupt status for the eDMA channel or 
 *                 channels.
 *END**************************************************************************/
void EDMA_ClearErrorIntStatusFlag(DMA_Type * base, uint8_t channel)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV3
    DMA_TCD(channel).CH_ES |= DMA_TCD_CH_ES_ERR_MASK;
#endif
#ifdef FEATURE_DMA_HWV2
    base->CERR = (uint8_t)channel;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetDmaRequestCmd
 * Description   : Enable/Disable dma request for channel or all channels.
 *END**************************************************************************/
void EDMA_SetDmaRequestCmd(DMA_Type * base, uint8_t channel,bool enable)
{
#ifdef FEATURE_DMA_HWV3
    uint32_t regValTemp;
    regValTemp = DMA_TCD(channel).CH_CSR;
    if (enable)
    {
        regValTemp |= DMA_TCD_CH_CSR_ERQ(1U);
    }
    else
    {
        regValTemp &= ~(DMA_TCD_CH_CSR_ERQ_MASK);
    }
    DMA_TCD(channel).CH_CSR = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    if (enable)
    {
        base->SERQ = channel;
    }
    else
    {
        base->CERQ = channel;
    }
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_ClearDoneStatusFlag
 * Description   : Clears the done status for a channel or all channels.
 *END**************************************************************************/
void EDMA_ClearDoneStatusFlag(DMA_Type * base, uint8_t channel)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV3
    DMA_TCD(channel).CH_CSR |= DMA_TCD_CH_CSR_DONE_MASK;
#endif
#ifdef FEATURE_DMA_HWV2
    base->CDNE = (uint8_t)channel;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TriggerChannelStart
 * Description   : Triggers the eDMA channel.
 *END**************************************************************************/
void EDMA_TriggerChannelStart(DMA_Type * base, uint8_t channel)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV3
    DMA_TCD(channel).CSR |= DMA_TCD_CSR_START_MASK;
#endif
#ifdef FEATURE_DMA_HWV2
    base->SSRT = (uint8_t)channel;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_ClearIntStatusFlag
 * Description   : Clears the interrupt status for the eDMA channel or all channels.
 *END**************************************************************************/
void EDMA_ClearIntStatusFlag(DMA_Type * base, uint8_t channel)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV3
    DMA_TCD(channel).CH_INT |= DMA_TCD_CH_INT_INT_MASK;
#endif
#ifdef FEATURE_DMA_HWV2
    base->CINT = (uint8_t)channel;
#endif
}

#ifdef FEATURE_DMA_HWV3
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetChannelPriorityGroup
 * Description   : Configures DMA channel group priority.
 *END**************************************************************************/
void EDMA_SetChannelPriorityGroup(DMA_Type * base, uint8_t channel, edma_group_priority_t channelPriorityGroup)
{
    BASE_MP(base,CH_GRPRI[channel]) = DMA_CH_GRPRI_GRPRI(channelPriorityGroup);
}
#endif

#ifdef FEATURE_DMA_HWV2
#if (FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetGroupPriority
 * Description   : Configures DMA group priorities.
 *END**************************************************************************/
void EDMA_SetGroupPriority(DMA_Type * base, const edma_user_config_t * userConfig)
{
    uint32_t regValTemp = base->CR;
    uint32_t mask = 0UL;
#if (FEATURE_DMA_CHANNELS <= 32U)    
    mask = DMA_CR_GRP0PRI_MASK | DMA_CR_GRP1PRI_MASK;
    regValTemp &= ~mask;
    if (userConfig->groupPriority == EDMA_GRP0_PRIO_HIGH_GRP1_PRIO_LOW)
    {
        regValTemp |= DMA_CR_GRP0PRI_MASK;
    }
    else
    {
        regValTemp |= DMA_CR_GRP1PRI_MASK;
    }
#elif (FEATURE_DMA_CHANNELS > 32U)
    mask = DMA_CR_GRP0PRI_MASK | DMA_CR_GRP1PRI_MASK | DMA_CR_GRP2PRI_MASK | DMA_CR_GRP3PRI_MASK;
    regValTemp &= ~mask;    
    regValTemp |= DMA_CR_GRP0PRI(userConfig->prioGroup0);
    regValTemp |= DMA_CR_GRP1PRI(userConfig->prioGroup1);
    regValTemp |= DMA_CR_GRP2PRI(userConfig->prioGroup2);
    regValTemp |= DMA_CR_GRP3PRI(userConfig->prioGroup3);
#endif

    base->CR = regValTemp;
}
#endif
#endif

#ifdef FEATURE_DMA_HWV2
#if (FEATURE_DMA_CHANNEL_GROUP_COUNT > 0x1U)
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetGroupArbitrationMode
 * Description   : Sets the group arbitration algorithm.
 *END**************************************************************************/
void EDMA_SetGroupArbitrationMode(DMA_Type * base, edma_arbitration_algorithm_t groupArbitration)
{
    uint32_t regValTemp;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_ERGA_MASK);
    regValTemp |= DMA_CR_ERGA(groupArbitration);
    base->CR = regValTemp;
}
#endif
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_SetMinorLoopMappingCmd
 * Description   : Enables/Disables the minor loop mapping.
 *END**************************************************************************/
void EDMA_SetMinorLoopMappingCmd(DMA_Type * base, uint8_t channel, bool enable)
{
    uint32_t regValTemp;
#ifdef FEATURE_DMA_HWV3    
    regValTemp = DMA_TCD(channel).NBYTES.MLOFFNO;
    regValTemp &= ~(DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK);
    regValTemp &= ~(DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK);
    regValTemp |= DMA_TCD_NBYTES_MLOFFNO_SMLOE(enable ? 1UL : 0UL);
    regValTemp |= DMA_TCD_NBYTES_MLOFFNO_DMLOE(enable ? 1UL : 0UL);
    DMA_TCD(channel).NBYTES.MLOFFNO = regValTemp;
#endif
#ifdef FEATURE_DMA_HWV2
    (void)channel;
    regValTemp = base->CR;
    regValTemp &= ~(DMA_CR_EMLM_MASK);
    regValTemp |= DMA_CR_EMLM(enable ? 1UL : 0UL);
    base->CR = regValTemp;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDClearReg
 * Description   : Set registers to 0 for hardware TCD of eDMA channel.
 *END**************************************************************************/
void EDMA_TCDClearReg(DMA_Type * base, uint8_t channel)
{
#ifdef FEATURE_DMA_HWV3
    DMA_TCD(channel).CH_CSR &= (DMA_TCD_CH_CSR_ERQ_MASK | DMA_TCD_CH_CSR_EEI_MASK);
    DMA_TCD(channel).CH_ES |= DMA_TCD_CH_ES_ERR_MASK;
    DMA_TCD(channel).CH_INT |= DMA_TCD_CH_INT_INT_MASK;
    DMA_TCD(channel).CH_SBR = 0U;
    DMA_TCD(channel).NBYTES.MLOFFNO &= (DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK | DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK);
    DMA_TCD(channel).SADDR = 0U;
    DMA_TCD(channel).SOFF = 0;
    DMA_TCD(channel).ATTR = 0U;
    DMA_TCD(channel).SLAST = 0;
    DMA_TCD(channel).DADDR = 0U;
    DMA_TCD(channel).DOFF = 0;
    DMA_TCD(channel).CITER.ELINKNO = 0U;
    DMA_TCD(channel).DLASTSGA = 0;
    DMA_TCD(channel).CSR = 0U;
    DMA_TCD(channel).BITER.ELINKNO = 0U;
#endif
#ifdef FEATURE_DMA_HWV2
    DMA_TCD(channel).NBYTES.MLNO = 0U;
    DMA_TCD(channel).SADDR = 0U;
    DMA_TCD(channel).SOFF = 0;
    DMA_TCD(channel).ATTR = 0U;
    DMA_TCD(channel).SLAST = 0;
    DMA_TCD(channel).DADDR = 0U;
    DMA_TCD(channel).DOFF = 0;
    DMA_TCD(channel).CITER.ELINKNO = 0U;
    DMA_TCD(channel).DLASTSGA = 0;
    DMA_TCD(channel).CSR = 0U;
    DMA_TCD(channel).BITER.ELINKNO = 0U;
#endif
}

#ifdef FEATURE_DMA_ENGINE_STALL
/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetEngineStall
 * Description   : Configures DMA engine to stall for a number of cycles after 
 *                 each R/W.
 *END**************************************************************************/
void EDMA_TCDSetEngineStall(DMA_Type * base, uint8_t channel, edma_engine_stall_t cycles)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif    
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= ~(DMA_TCD_CSR_BWC_MASK);    
    regValTemp |= DMA_TCD_CSR_BWC(cycles);
    DMA_TCD(channel).CSR = regValTemp;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcAddr
 * Description   : Configures the source address for the hardware TCD.
 *END**************************************************************************/
void EDMA_TCDSetSrcAddr(DMA_Type * base, uint8_t channel, uint32_t address)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).SADDR = address;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcOffset
 * Description   : Configures the source address signed offset for the hardware TCD.
 *END**************************************************************************/
void EDMA_TCDSetSrcOffset(DMA_Type * base, uint8_t channel, int16_t offset)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).SOFF = (uint16_t)offset;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetAttribute
 * Description   : Configures the transfer attribute for eDMA channel.
 *END**************************************************************************/
void EDMA_TCDSetAttribute(
                DMA_Type * base, uint8_t channel,
                edma_modulo_t srcModulo, edma_modulo_t destModulo,
                edma_transfer_size_t srcTransferSize, edma_transfer_size_t destTransferSize)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = (uint16_t)(DMA_TCD_ATTR_SMOD(srcModulo) | DMA_TCD_ATTR_SSIZE(srcTransferSize));
    regValTemp |= (uint16_t)(DMA_TCD_ATTR_DMOD(destModulo) | DMA_TCD_ATTR_DSIZE(destTransferSize));
    DMA_TCD(channel).ATTR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcTransferSize
 * Description   : Sets the source transfer size.
 *END**************************************************************************/
void EDMA_TCDSetSrcTransferSize(DMA_Type * base, uint8_t channel, edma_transfer_size_t size)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).ATTR;
    regValTemp &= (uint16_t)(~(DMA_TCD_ATTR_SSIZE_MASK));
    regValTemp |= (uint16_t)(DMA_TCD_ATTR_SSIZE((uint16_t)size));
    DMA_TCD(channel).ATTR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetDestTransferSize
 * Description   : Sets the destination transfer size.
 *END**************************************************************************/
void EDMA_TCDSetDestTransferSize(DMA_Type * base, uint8_t channel, edma_transfer_size_t size)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).ATTR;
    regValTemp &= (uint16_t)(~(DMA_TCD_ATTR_DSIZE_MASK));
    regValTemp |= (uint16_t)(DMA_TCD_ATTR_DSIZE((uint16_t)size));
    DMA_TCD(channel).ATTR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetNbytes
 * Description   : Configures the nbytes for eDMA channel.
 *END**************************************************************************/
void EDMA_TCDSetNbytes(DMA_Type * base, uint8_t channel, uint32_t nbytes)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif

#ifdef FEATURE_DMA_HWV2
    if ((base->CR & DMA_CR_EMLM_MASK) != 0UL)
    {     
#endif
        uint32_t sdmloe = DMA_TCD(channel).NBYTES.MLOFFNO & (DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK | DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK);
        if (sdmloe == 0UL)
        {
            DMA_TCD(channel).NBYTES.MLOFFNO = (nbytes & DMA_TCD_NBYTES_MLOFFNO_NBYTES_MASK);
        }
        else
        {
            uint32_t regValTemp;
            regValTemp = DMA_TCD(channel).NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_NBYTES_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_NBYTES(nbytes);
            DMA_TCD(channel).NBYTES.MLOFFYES = regValTemp;            
        }
#ifdef FEATURE_DMA_HWV2		
    }
    else
    {
        DMA_TCD(channel).NBYTES.MLNO = nbytes;
    }
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcMinorLoopOffsetCmd
 * Description   : Enables/disables the source minor loop offset feature for 
 *                 the TCD.
 *END**************************************************************************/
void EDMA_TCDSetSrcMinorLoopOffsetCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV2
    if ((base->CR & DMA_CR_EMLM_MASK) != 0UL)
    {
#endif
        uint32_t regValTemp;
        regValTemp = DMA_TCD(channel).NBYTES.MLOFFYES;
        regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_SMLOE_MASK);
        regValTemp |= DMA_TCD_NBYTES_MLOFFYES_SMLOE(enable ? 1UL : 0UL);
        DMA_TCD(channel).NBYTES.MLOFFYES = regValTemp;
#ifdef FEATURE_DMA_HWV2
	}
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcMinorLoopOffsetCmd
 * Description   : Enables/disables the destination minor loop offset feature 
 *                 for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetDestMinorLoopOffsetCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV2
    if ((base->CR & DMA_CR_EMLM_MASK) != 0UL)
    {
#endif		
        uint32_t regValTemp;
        regValTemp = DMA_TCD(channel).NBYTES.MLOFFYES;
        regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_DMLOE_MASK);
        regValTemp |= DMA_TCD_NBYTES_MLOFFYES_DMLOE(enable ? 1UL : 0UL);
        DMA_TCD(channel).NBYTES.MLOFFYES = regValTemp;
#ifdef FEATURE_DMA_HWV2
    }
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetMinorLoopOffset
 * Description   : Configures the minor loop offset for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetMinorLoopOffset(DMA_Type * base, uint8_t channel, int32_t offset)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
#ifdef FEATURE_DMA_HWV2
    if ((base->CR & DMA_CR_EMLM_MASK) != 0UL)
    {
#endif
        uint32_t sdmloe = DMA_TCD(channel).NBYTES.MLOFFNO & (DMA_TCD_NBYTES_MLOFFNO_SMLOE_MASK | DMA_TCD_NBYTES_MLOFFNO_DMLOE_MASK);
        if (sdmloe != 0UL)
        {
            uint32_t regValTemp;
            regValTemp = DMA_TCD(channel).NBYTES.MLOFFYES;
            regValTemp &= ~(DMA_TCD_NBYTES_MLOFFYES_MLOFF_MASK);
            regValTemp |= DMA_TCD_NBYTES_MLOFFYES_MLOFF(offset);
            DMA_TCD(channel).NBYTES.MLOFFYES = regValTemp;
        }
#ifdef FEATURE_DMA_HWV2
    }
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetSrcLastAdjust
 * Description   : Configures the last source address adjustment for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetSrcLastAdjust(DMA_Type * base, uint8_t channel, int32_t size)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).SLAST = (uint32_t)size;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetDestAddr
 * Description   : Configures the destination address for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetDestAddr(DMA_Type * base, uint8_t channel, uint32_t address)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).DADDR = address;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetDestOffset
 * Description   : Configures the destination address signed offset for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetDestOffset(DMA_Type * base, uint8_t channel, int16_t offset)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).DOFF = (uint16_t)offset;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetDestLastAdjust
 * Description   : Configures the last source address adjustment.
 *END**************************************************************************/
void EDMA_TCDSetDestLastAdjust(DMA_Type * base, uint8_t channel, int32_t adjust)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).DLASTSGA = (uint32_t)adjust;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetScatterGatherLink
 * Description   : Configures the memory address of the next TCD, in 
 *                 Scatter/Gather mode.
 *END**************************************************************************/
void EDMA_TCDSetScatterGatherLink(DMA_Type * base, uint8_t channel, uint32_t nextTCDAddr)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    DMA_TCD(channel).DLASTSGA = nextTCDAddr;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetScatterGatherCmd
 * Description   : Enables/Disables the scatter/gather feature for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetScatterGatherCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_ESG_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_ESG(enable ? 1UL : 0UL);
    DMA_TCD(channel).CSR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetChannelMajorLink
 * Description   : Configures the major channel link the TCD.
 *END**************************************************************************/
void EDMA_TCDSetChannelMajorLink(DMA_Type * base, uint8_t channel, uint32_t majorLinkChannel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_MAJORLINKCH_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_MAJORLINKCH(majorLinkChannel);
    DMA_TCD(channel).CSR = regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_MAJORELINK_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_MAJORELINK(enable ? 1UL : 0UL);
    DMA_TCD(channel).CSR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetDisableDmaRequestAfterTCDDoneCmd
 * Description   : Disables/Enables the DMA request after the major loop 
 *                 completes for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetDisableDmaRequestAfterTCDDoneCmd(DMA_Type * base, uint8_t channel, bool disable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_DREQ_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_DREQ(disable ? 1UL : 0UL);
    DMA_TCD(channel).CSR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetMajorHalfCompleteIntCmd
 * Description   : Enables/Disables the half complete interrupt for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetMajorHalfCompleteIntCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_INTHALF_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_INTHALF(enable ? 1UL : 0UL);
    DMA_TCD(channel).CSR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetMajorCompleteIntCmd
 * Description   : Enables/Disables the interrupt after the major loop completes 
 *                 for the TCD.
 *END**************************************************************************/
void EDMA_TCDSetMajorCompleteIntCmd(DMA_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp;
    regValTemp = DMA_TCD(channel).CSR;
    regValTemp &= (uint16_t)~(DMA_TCD_CSR_INTMAJOR_MASK);
    regValTemp |= (uint16_t)DMA_TCD_CSR_INTMAJOR(enable ? 1UL : 0UL);
    DMA_TCD(channel).CSR = regValTemp;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDSetChannelMinorLink
 * Description   : Set Channel minor link for hardware TCD.
 *END**************************************************************************/
void EDMA_TCDSetChannelMinorLink(DMA_Type * base, uint8_t channel, uint32_t linkChannel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
    DEV_ASSERT(linkChannel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp = 0U;
    if (enable)
    {
        regValTemp = DMA_TCD(channel).BITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_ELINK_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_ELINK(1UL);
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_LINKCH(linkChannel);
        DMA_TCD(channel).BITER.ELINKYES = regValTemp;
        regValTemp = DMA_TCD(channel).CITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_ELINK_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_ELINK(1UL);
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_LINKCH(linkChannel);
        DMA_TCD(channel).CITER.ELINKYES = regValTemp;
    }
    else
    {
        regValTemp = DMA_TCD(channel).BITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_ELINK_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_ELINK(0UL);
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_LINKCH(linkChannel);
        DMA_TCD(channel).BITER.ELINKNO = regValTemp;
        regValTemp = DMA_TCD(channel).CITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_ELINK_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_ELINK(0UL);
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_LINKCH_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_LINKCH(linkChannel);
        DMA_TCD(channel).CITER.ELINKNO = regValTemp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCD_TCDSetMajorCount
 * Description   : Sets the major iteration count according to minor loop
 * channel link setting.
 *END**************************************************************************/
void EDMA_TCDSetMajorCount(DMA_Type * base, uint8_t channel, uint32_t count)
{    
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t regValTemp = 0U;
    if ((DMA_TCD(channel).BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) != 0U)
    {
        regValTemp = DMA_TCD(channel).BITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKYES_BITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKYES_BITER(count);
        DMA_TCD(channel).BITER.ELINKYES = regValTemp;
        regValTemp = DMA_TCD(channel).CITER.ELINKYES;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKYES_CITER_LE_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKYES_CITER_LE(count);
        DMA_TCD(channel).CITER.ELINKYES = regValTemp;
    }
    else
    {
        regValTemp = DMA_TCD(channel).BITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_BITER_ELINKNO_BITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_BITER_ELINKNO_BITER(count);
        DMA_TCD(channel).BITER.ELINKNO = regValTemp;
        regValTemp = DMA_TCD(channel).CITER.ELINKNO;
        regValTemp &= (uint16_t)~(DMA_TCD_CITER_ELINKNO_CITER_MASK);
        regValTemp |= (uint16_t)DMA_TCD_CITER_ELINKNO_CITER(count);
        DMA_TCD(channel).CITER.ELINKNO = regValTemp;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : EDMA_TCDGetCurrentMajorCount
 * Description   : Gets the current major iteration count according to minor
 * loop channel link setting.
 *END**************************************************************************/
uint32_t EDMA_TCDGetCurrentMajorCount(const DMA_Type * base, uint8_t channel)
{    
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMA_CHANNELS);
#endif
    uint16_t result = 0U;
    if ((DMA_TCD(channel).BITER.ELINKNO & DMA_TCD_BITER_ELINKNO_ELINK_MASK) != 0U)
    {
        result = (uint16_t)((DMA_TCD(channel).CITER.ELINKYES & DMA_TCD_CITER_ELINKYES_CITER_LE_MASK) >> DMA_TCD_CITER_ELINKYES_CITER_LE_SHIFT);
    }
    else
    {
        result = (uint16_t)((DMA_TCD(channel).CITER.ELINKNO & DMA_TCD_CITER_ELINKNO_CITER_MASK) >> DMA_TCD_CITER_ELINKNO_CITER_SHIFT);
    }
    return (uint32_t) result;
}

#ifdef FEATURE_DMAMUX_AVAILABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : DMAMUX_init
 * Description   : Initialize the dmamux module to the reset state.
 *END**************************************************************************/
void DMAMUX_Init(DMAMUX_Type * base)
{
    uint8_t idx;
    for (idx = 0U; idx < FEATURE_DMAMUX_CHANNELS; idx++)
    {
        base->CHCFG[idx] = 0U;
    }
}
#endif

#ifdef FEATURE_DMAMUX_AVAILABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : DMAMUX_SetChannelCmd
 * Description   : Enables/Disables the DMAMUX channel.
 *END**************************************************************************/
void DMAMUX_SetChannelCmd(DMAMUX_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMAMUX_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_ENBL_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_ENBL(enable ? 1U : 0U);
    base->CHCFG[regIndex] = regValTemp;
}
#endif

#ifdef FEATURE_DMAMUX_AVAILABLE
#ifdef FEATURE_DMAMUX_HAS_TRIG
/*FUNCTION**********************************************************************
 *
 * Function Name : DMAMUX_SetChannelTrigger
 * Description   : Configure DMA Channel Trigger bit in DMAMUX.
 *END**************************************************************************/
void DMAMUX_SetChannelTrigger(DMAMUX_Type * base, uint8_t channel, bool enable)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMAMUX_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp;
    regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_TRIG_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_TRIG(enable ? 1U : 0U);
    base->CHCFG[regIndex] = regValTemp;
}
#endif
#endif

#ifdef FEATURE_DMAMUX_AVAILABLE
/*FUNCTION**********************************************************************
 *
 * Function Name : DMAMUX_SetChannelSource
 * Description   : Configures the DMA request for the DMAMUX channel.
 *END**************************************************************************/
void DMAMUX_SetChannelSource(DMAMUX_Type * base, uint8_t channel, uint8_t source)
{
#if defined (CUSTOM_DEVASSERT) || defined (DEV_ERROR_DETECT)
    DEV_ASSERT(channel < FEATURE_DMAMUX_CHANNELS);
#endif
    uint32_t regIndex = FEATURE_DMAMUX_CHN_REG_INDEX(channel);
    uint8_t regValTemp;
    regValTemp = base->CHCFG[regIndex];
    regValTemp &= (uint8_t)~(DMAMUX_CHCFG_SOURCE_MASK);
    regValTemp |= (uint8_t)DMAMUX_CHCFG_SOURCE(source);
    base->CHCFG[regIndex] = regValTemp;
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
