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

#include "lfast_hw_access.h"
#include "sipi_hw_access.h"

/*!
 * @file zipwire_hw_access.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 15.5, Return statement before end of function.
 * The return statement before end of function is used for simpler code structure
 * and better readability.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 13.1, side effect in initializer
 * Array is initialized with addresses to volatile memory, pointing to registers on
 * which read operations have no side effects.
 *
 */

/*******************************************************************************
 * Register access arrays
 ******************************************************************************/
__IO uint32_t * CCR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CCR0, &SIPI->CCR1, &SIPI->CCR2, &SIPI->CCR3}};
__IO uint32_t * CSR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CSR0, &SIPI->CSR1, &SIPI->CSR2, &SIPI->CSR3}};
__IO uint32_t * CIR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CIR0, &SIPI->CIR1, &SIPI->CIR2, &SIPI->CIR3}};
__IO uint32_t * CTOR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CTOR0, &SIPI->CTOR1, &SIPI->CTOR2, &SIPI->CTOR3}};
__IO uint32_t * CAR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CAR0, &SIPI->CAR1, &SIPI->CAR2, &SIPI->CAR3}};
__IO uint32_t * CDR[SIPI_INSTANCE_COUNT][SIPI_CHANNEL_COUNT] = {{&SIPI->CDR0, &SIPI->CDR1, &SIPI->CDR2[0], &SIPI->CDR3}};

/*******************************************************************************
 * Private functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_SendIclcFrame
 * Description   : This function sends an ICLC frame with the given payload.
 *
 *END**************************************************************************/
static status_t LFAST_SendIclcFrame(LFAST_Type *base, uint8_t payload, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool frameTransmitted = false;
    status_t stat;

    /* Set the playload and send the frame */
    LFAST_SetIclcPayload(base, payload);
    LFAST_IclcFrameRequest(base);
    /* Wait for ICLC frame transmission to be confirmed */
    do
    {
        frameTransmitted = LFAST_IclcFrameTransmittedFlag(base);
        tempTimeout--;
    }
    while ((!frameTransmitted) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcFrameTransmittedFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitPingResponse
 * Description   : This function polls the ping response flag to check if slave
 *                 confirmed it's status.
 *
 *END**************************************************************************/
static status_t LFAST_WaitPingResponse(LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool pingResponse = false;
    status_t stat;

    /* Check if the ping response was received */
    do
    {
        pingResponse = LFAST_IclcPingFrameResponseSuccessfulFlag(base);
        tempTimeout--;
    }
    while ((!pingResponse) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcPingFrameResponseSuccessfulFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitMasterSpeedModeSwitch
 * Description   : This function polls the master speed mode flag to check whether
 *                 high speed mode has been activated for both rx and tx interfaces.
 *
 *END**************************************************************************/
static status_t LFAST_WaitMasterSpeedModeSwitch(const LFAST_Type *base, bool tx, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool speedModeChanged = false;
    status_t stat;

    /* Wait for the master speed mode change */
    do
    {
        if (tx)
        {
            speedModeChanged = LFAST_GetTxSpeedMode(base);
        }
        else
        {
            speedModeChanged = LFAST_GetRxSpeedMode(base);
        }
        tempTimeout--;
    }
    while ((!speedModeChanged) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitSlaveTxEnable
 * Description   : This function polls the tx enabled flag to check whether the
 *                 tx interface has been enabled through ICLC frame by the master.
 *
 *END**************************************************************************/
static status_t LFAST_WaitSlaveTxEnable(const LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool txEnabled = false;
    status_t stat;

    /* Wait for tx to be enabled through ICLC frame from master */
    do
    {
        txEnabled = LFAST_IsTxEnabled(base);
        tempTimeout--;
    }
    while ((!txEnabled) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitPingRequest
 * Description   : This function polls the ping request flag; if a ping request
 *                 is received from the master, the automatic response will
 *                 confirm slave status.
 *
 *END**************************************************************************/
static status_t LFAST_WaitPingRequest(LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool pingRequestReceived = false;
    status_t stat;

    /* Wait for ping request; automatic response will confirm slave status */
    do
    {
        pingRequestReceived = LFAST_IclcPingFrameRequestReceivedFlag(base);
        tempTimeout--;
    }
    while ((!pingRequestReceived) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcPingFrameRequestReceivedFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitPllOnCommand
 * Description   : This function checks whether the command to turn on the PLL
 *                 was received from the master.
 *
 *END**************************************************************************/
static status_t LFAST_WaitPllOnCommand(LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool pllOnCmdReceived = false;
    status_t stat;

    /* Wait for PLL start request */
    do
    {
        pllOnCmdReceived = LFAST_IclcPingFrameRequestReceivedFlag(base);
        tempTimeout--;
    }
    while ((!pllOnCmdReceived) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcPllOnReceivedFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitPllActive
 * Description   : This function polls the PLL active flag, so no other operation
 *                 is done until the PLL is stable.
 *
 *END**************************************************************************/
static status_t LFAST_WaitPllActive(const LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool pllActive = false;
    status_t stat;

    /* Wait for PLL disable signal to be negated */
    do
    {
        pllActive = LFAST_PllActive(base);
        tempTimeout--;
    }
    while ((!pllActive) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitPllLock
 * Description   : This function waits for the PLL locked flag to be asserted.
 *
 *END**************************************************************************/
static status_t LFAST_WaitPllLock(const LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool pllLocked = false;
    status_t stat;

    /* Wait for PLL to lock. */
    do
    {
        pllLocked = LFAST_PllLocked(base);
        tempTimeout--;
    }
    while ((!pllLocked) && ((timeout == 0U) || (tempTimeout > 0U)));

    /* Check if the operation has timed out */
    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitTxSpeedModeChange
 * Description   : This function waits for the tx speed mode change command to be
 *                 received from the master.
 *
 *END**************************************************************************/
static status_t LFAST_WaitTxSpeedModeChange(LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool txFastModeCmdReceived = false;
    status_t stat;
    /* Speed of Tx interface is changed by receiving a ICLC frame with 0x80 payload */
    do
    {
        txFastModeCmdReceived = LFAST_IclcSlaveTxFastModeReceivedFlag(base);
        tempTimeout--;
    }
    while((!txFastModeCmdReceived) && ((timeout == 0U) || (tempTimeout > 0U)));

    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcSlaveTxFastModeReceivedFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_WaitRxSpeedModeChange
 * Description   : This function waits for the rx speed mode change command to be
 *                 received from the master.
 *
 *END**************************************************************************/
static status_t LFAST_WaitRxSpeedModeChange(LFAST_Type *base, uint32_t timeout)
{
    uint32_t tempTimeout = timeout;
    bool rxFastModeCmdReceived = false;
    status_t stat;
    /* Speed of Rx interface is changed by receiving a ICLC frame with 0x10 payload */
    do
    {
        rxFastModeCmdReceived = LFAST_IclcSlaveRxFastModeReceivedFlag(base);
        tempTimeout--;
    }
    while((!rxFastModeCmdReceived) && ((timeout == 0U) || (tempTimeout > 0U)));

    if ((timeout == 0U) || (tempTimeout > 0U))
    {
        LFAST_ClearIclcSlaveRxFastModeReceivedFlag(base);
        stat = STATUS_SUCCESS;
    }
    else
    {
        stat = STATUS_TIMEOUT;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_MasterHighSpeedInit
 * Description   : This function launches the master high speed mode initialization
 *                 procedure; it follows all the steps needed to synchronize with
 *                 the slave in high speed.
 *
 *END**************************************************************************/
static status_t LFAST_MasterHighSpeedInit(LFAST_Type *base, uint32_t timeout)
{
    status_t stat;

    /* Enable pll on master side */
    LFAST_TurnOnPll(base);

    /* Enable pll on slave side (ICLC 0x02) */
    stat = LFAST_SendIclcFrame(base, LFAST_ENABLE_SLAVE_PLL_CMD, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Change slave tx speed mode (ICLC 0x80) */
    stat = LFAST_SendIclcFrame(base, LFAST_CHANGE_SLAVE_TX_SPEED_MODE_CMD, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Change slave rx speed mode (ICLC 0x10) */
    stat = LFAST_SendIclcFrame(base, LFAST_CHANGE_SLAVE_RX_SPEED_MODE_CMD, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Disable ICLC frames to write master data rate */
    LFAST_EnableIclcSequence(base, false);

    /* Change master tx/rx speed mode */
    LFAST_EnableIclcSpeedModeChange(base, false);
    LFAST_SetRxSpeedMode(base, LFAST_HIGH_SPEED);
    LFAST_SetTxSpeedMode(base, LFAST_HIGH_SPEED);

    /* Wait for the master rx speed mode change */
    stat = LFAST_WaitMasterSpeedModeSwitch(base, false, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }
    /* Wait for the master tx speed mode change */
    stat = LFAST_WaitMasterSpeedModeSwitch(base, true, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Enable ICLC frames to confirm slave status */
    LFAST_EnableIclcSequence(base, true);

    /* Check slave status (ICLC 0x00) */
    LFAST_SetIclcPayload(base, LFAST_CHECK_SLAVE_STATUS_CMD);
    stat = LFAST_SendIclcFrame(base, LFAST_CHECK_SLAVE_STATUS_CMD, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Disable tx arbitrer */
    LFAST_EnableTxArbitrer(base, false);

    /* Slave speed mode change confirmed by successful ping response */
    stat = LFAST_WaitPingResponse(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Disable ICLC frames */
    LFAST_EnableIclcSequence(base, false);
    /* Enable tx arbitrer */
    LFAST_EnableTxArbitrer(base, true);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_SlaveHighSpeedInit
 * Description   : This function launches the slave high speed mode initialization
 *                 procedure; it follows all the steps needed to synchronize with
 *                 the master in high speed.
 *
 *END**************************************************************************/
static status_t LFAST_SlaveHighSpeedInit(LFAST_Type *base, uint32_t timeout)
{
    status_t stat;

    /* Wait for PLL start request */
    stat = LFAST_WaitPllOnCommand(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Wait for PLL to become active. */
    stat = LFAST_WaitPllActive(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Wait for PLL to lock. */
    stat = LFAST_WaitPllLock(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Speed of Tx interface is changed by receiving a ICLC frame with 0x80 payload */
    stat = LFAST_WaitTxSpeedModeChange(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Speed of Rx interface is changed by receiving a ICLC frame with 0x10 payload */
    stat = LFAST_WaitRxSpeedModeChange(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Wait for ping request; automatic response will confirm slave status */
    stat = LFAST_WaitPingRequest(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_MasterInit
 * Description   : This function initializes the LFAST master.
 *
 *END**************************************************************************/
status_t LFAST_MasterInit(LFAST_Type *base,
                          lfast_pll_refclk_div_t preDiv,
                          uint8_t fbDiv,
                          lfast_speed_mode_t speedMode,
                          lfast_low_speed_clk_t lsClkDiv,
                          uint32_t timeout,
                          uint32_t attempts)
{
    uint32_t tempAttempts = attempts;
    bool slaveTxConfirmed = false;
    status_t stat;

    /* Soft reset LFAST interface */
    LFAST_Reset(base);

    /* Configure LFAST interface as per data sheet */
    LFAST_SetRateChangeDelay(base, LFAST_DATA_RATE_CHANGE_DELAY);
    base->SLCR = LFAST_WAKEUP_DELAY_CONTROL;
    LFAST_SetPllReferenceDivisionFactor(base, (uint8_t)preDiv);
    LFAST_SetPllFeedbackDivisionFactor(base, fbDiv);

    /* Configure LFAST interface */
    base->LCR = LFAST_LCR_LVTXOE_MASK | LFAST_LCR_LVRFEN_MASK | LFAST_LCR_LVRXOP_TR_MASK | LFAST_LCR_LVTXOP_MASK;

    /* Configure as master */
    LFAST_SetRole(base, LFAST_MASTER);

    /* Select low speed clock input */
    LFAST_SetLowSpeedClk(base, lsClkDiv);

    /* Enable data frames */
    LFAST_EnableData(base, true);

    /* Enable LFAST module */
    LFAST_Enable(base, true);

    /* Enable receiver and transmitter */
    LFAST_EnableRx(base, true);
    LFAST_EnableTx(base, true);

    /* Enable ICLC frames */
    LFAST_EnableIclcSequence(base, true);

    /* Enable tx arbiter */
    LFAST_EnableTxArbitrer(base, true);

    /* Try to enable slave tx and confirm synchronization */
    while ((!slaveTxConfirmed) && ((attempts == 0U) || (tempAttempts > 0U)))
    {
        tempAttempts--;

        /* Enable slave tx interface (ICLC 0x31) */
        stat = LFAST_SendIclcFrame(base, LFAST_ENABLE_SLAVE_TX_CMD, timeout);
        if (stat != STATUS_SUCCESS)
        {
            continue;
        }

        /* Check slave status (ICLC 0x00) */
        LFAST_SetIclcPayload(base, LFAST_CHECK_SLAVE_STATUS_CMD);
        stat = LFAST_SendIclcFrame(base, LFAST_CHECK_SLAVE_STATUS_CMD, timeout);
        if (stat != STATUS_SUCCESS)
        {
            continue;
        }

        /* Slave status confirmed by successful ping response */
        stat = LFAST_WaitPingResponse(base, timeout);
        if (stat == STATUS_SUCCESS)
        {
            slaveTxConfirmed = true;
        }
        else
        {
            continue;
        }
    }
    /* Return TIMEOUT if slave status is not confirmed by ping response
     * and the maximum number of attempts has been reached */
    if (!slaveTxConfirmed)
    {
        return STATUS_TIMEOUT;
    }

    /* Speed mode change */
    if (speedMode == LFAST_HIGH_SPEED)
    {
        stat = LFAST_MasterHighSpeedInit(base, timeout);
        if (stat != STATUS_SUCCESS)
        {
            return stat;
        }
    }
    else
    {
        /* Disable ICLC frames */
        LFAST_EnableIclcSequence(base, false);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LFAST_SlaveInit
 * Description   : This function initializes the LFAST slave.
 *
 *END**************************************************************************/
status_t LFAST_SlaveInit(LFAST_Type *base,
                         lfast_pll_refclk_div_t preDiv,
                         uint8_t fbDiv,
                         lfast_speed_mode_t speedMode,
                         lfast_low_speed_clk_t lsClkDiv,
                         uint32_t timeout)
{
    status_t stat;

    /* Soft reset LFAST interface */
    LFAST_Reset(base);

    /* Configure LFAST interface as per data sheet */
    LFAST_SetRateChangeDelay(base, LFAST_DATA_RATE_CHANGE_DELAY);
    base->SLCR = LFAST_WAKEUP_DELAY_CONTROL;
    LFAST_SetPllReferenceDivisionFactor(base, (uint8_t)preDiv);
    LFAST_SetPllFeedbackDivisionFactor(base, fbDiv);

    /* Configure LFAST interface */
    base->LCR = LFAST_LCR_LVTXOE_MASK | LFAST_LCR_LVRFEN_MASK | LFAST_LCR_LVRXOP_TR_MASK | LFAST_LCR_LVTXOP_MASK;

    /* Configure as slave */
    LFAST_SetRole(base, LFAST_SLAVE);

    /* Select low speed clock input */
    LFAST_SetLowSpeedClk(base, lsClkDiv);

    /* Enable data frames */
    LFAST_EnableData(base, true);

    /* Enable LFAST module */
    LFAST_Enable(base, true);

    /* Enable receiver and transmitter */
    LFAST_EnableRx(base, true);

    /* Enable speed mode change upon ICLC frames reception */
    LFAST_EnableIclcSpeedModeChange(base, true);

    /* Enable automatic ping response */
    LFAST_EnableAutomaticPingResponse(base, true);

    /* Wait for tx to be enabled through ICLC frame from master */
    stat = LFAST_WaitSlaveTxEnable(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Enable tx arbitrer */
    LFAST_EnableTxArbitrer(base, true);

    /* Wait for ping request; automatic response will confirm slave status */
    stat = LFAST_WaitPingRequest(base, timeout);
    if (stat != STATUS_SUCCESS)
    {
        return stat;
    }

    /* Speed mode change */
    if (speedMode == LFAST_HIGH_SPEED)
    {
        stat = LFAST_SlaveHighSpeedInit(base, timeout);
        if (stat != STATUS_SUCCESS)
        {
            return stat;
        }
    }

    return STATUS_SUCCESS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

