/*
 * Copyright 2018 NXP
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
 * @file pins_siu_driver.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 */

#include "pins_driver.h"
#include "siu_hw_access.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_Init
 * Description   : This function configures the pins with the options provided
 * in the given structure.
 *
 * Implements    : PINS_DRV_Init_Activity
 *END**************************************************************************/
status_t PINS_DRV_Init(uint32_t pinCount,
                       const pin_settings_config_t config[])
{
    uint32_t i;
    for (i = 0U; i < pinCount; i++)
    {
        PINS_Init(&config[i]);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_WritePin
 * Description   : This function writes the given pin from a port, with the given
 * value ('0' represents LOW, '1' represents HIGH).
 *
 * Implements    : PINS_DRV_WritePin_Activity
 *END**************************************************************************/
void PINS_DRV_WritePin(GPIO_Type * const base,
                       pins_channel_type_t pin,
                       pins_level_type_t value)
{
    PINS_WritePin(base, pin, value);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_WritePins
 * Description   : This function writes all pins configured as output with the
 * values given in the parameter pins. '0' represents LOW, '1' represents HIGH.
 *
 * Implements    : PINS_DRV_WritePins_Activity
 *END**************************************************************************/
void PINS_DRV_WritePins(GPIO_Type * const base,
                        pins_channel_type_t pins)
{
    PINS_WritePins(base, pins);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_GetPinsOutput
 * Description   : This function returns the current output that is written to a
 * port. Only pins that are configured as output will have meaningful values.
 *
 * Implements    : PINS_DRV_GetPinsOutput_Activity
 *END**************************************************************************/
pins_channel_type_t PINS_DRV_GetPinsOutput(const GPIO_Type * const base)
{
    return PINS_GetPinsOutput(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_SetPins
 * Description   : This function configures output pins listed in parameter pins
 * (bits that are '1') to have a value of 'set' (HIGH). Pins corresponding to '0'
 * will be unaffected.
 *
 * Implements    : PINS_DRV_SetPins_Activity
 *END**************************************************************************/
void PINS_DRV_SetPins(GPIO_Type * const base,
                      pins_channel_type_t pins)
{
    PINS_SetPins(base, pins);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ClearPins
 * Description   : This function configures output pins listed in parameter pins
 * (bits that are '1') to have a 'cleared' value (LOW). Pins corresponding to '0'
 * will be unaffected.
 *
 * Implements    : PINS_DRV_ClearPins_Activity
 *END**************************************************************************/
void PINS_DRV_ClearPins(GPIO_Type * const base,
                        pins_channel_type_t pins)
{
    PINS_ClearPins(base, pins);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_TogglePins
 * Description   : This function toggles output pins listed in parameter pins
 * (bits that are '1'). Pins corresponding to '0' will be unaffected.
 *
 * Implements    : PINS_DRV_TogglePins_Activity
 *END**************************************************************************/
void PINS_DRV_TogglePins(GPIO_Type * const base,
                         pins_channel_type_t pins)
{
    PINS_TogglePins(base, pins);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ReadPins
 * Description   : This function returns the current input values from a port.
 * Only pins configured as input will have meaningful values.
 *
 * Implements    : PINS_DRV_ReadPins_Activity
 *END**************************************************************************/
pins_channel_type_t PINS_DRV_ReadPins(const GPIO_Type * const base)
{
    return PINS_ReadPins(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_SetPullSel
 * Description   : This function configures the internal resistor.
 *
 * Implements    : PINS_DRV_SetPullSel_Activity
 *END**************************************************************************/
void PINS_DRV_SetPullSel(PORT_Type * const base,
                         uint32_t pin,
                         port_pull_config_t pullConfig)
{
    PINS_SetPullSel(base, pin, pullConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ConfigOutputMux
 * Description   : This function configures the output muxing and gate.
 *
 * Implements    : PINS_DRV_ConfigOutputMux_Activity
 *END**************************************************************************/
void PINS_DRV_ConfigOutputMux(PORT_Type * const base,
                              uint32_t pin,
                              bool enable,
                              port_mux_t mux)
{
    PINS_ConfigOutputMux(base, pin, enable, mux);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ConfigInputMux
 * Description   : This function configures the input muxing and gate.
 *
 * Implements    : PINS_DRV_ConfigInputMux_Activity
 *END**************************************************************************/
void PINS_DRV_ConfigInputMux(PORT_Type * const base,
                             uint32_t pin,
                             bool enable,
                             port_mux_t mux,
                             siu_input_config_t inputConfig)
{
    PINS_ConfigInputMux(base, pin, enable, mux, inputConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ConfigDigitalFilterLength
 * Description   : This function configures the digital filter length.
 *
 * Implements    : PINS_DRV_ConfigDigitalFilterLength_Activity
 *END**************************************************************************/
void PINS_DRV_ConfigDigitalFilterLength(uint8_t length)
{
    PINS_ConfigDigitalFilterLength(length);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_SetExInt
 * Description   : This function configures the external interrupt.
 *
 * Implements    : PINS_DRV_SetExInt_Activity
 *END**************************************************************************/
void PINS_DRV_SetExInt(siu_interrupt_config_t intConfig)
{
    PINS_SetExInt(intConfig);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ClearPinExIntFlag
 * Description   : This function clears the individual pin external interrupt
 * status flag.
 *
 * Implements    : PINS_DRV_ClearPinExIntFlag_Activity
 *END**************************************************************************/
void PINS_DRV_ClearPinExIntFlag(uint32_t eirqPinIdx)
{
    PINS_ClearPinExIntFlag(eirqPinIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_GetPinExIntFlag
 * Description   : This function gets the individual pin external interrupt
 * status flag.
 *
 * Implements    : PINS_DRV_GetPinExIntFlag_Activity
 *END**************************************************************************/
bool PINS_DRV_GetPinExIntFlag(uint32_t eirqPinIdx)
{
    return PINS_GetPinExIntFlag(eirqPinIdx);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ClearExIntFlag
 * Description   : This function clears the entire external interrupt status flag.
 *
 * Implements    : PINS_DRV_ClearExIntFlag_Activity
 *END**************************************************************************/
void PINS_DRV_ClearExIntFlag(void)
{
    PINS_ClearExIntFlag();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_GetExIntFlag
 * Description   : This function reads the entire external interrupt status flag.
 *
 * Implements    : PINS_DRV_GetExIntFlag_Activity
 *END**************************************************************************/
uint32_t PINS_DRV_GetExIntFlag(void)
{
    return PINS_GetExIntFlag();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_ClearOverrunIntFlag
 * Description   : This function clears the entire overrun interrupt status flag.
 *
 * Implements    : PINS_DRV_ClearOverrunIntFlag_Activity
 *END**************************************************************************/
void PINS_DRV_ClearOverrunIntFlag(void)
{
    PINS_ClearOverrunIntFlag();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_GetOverrunIntFlag
 * Description   : This function reads the entire overrun interrupt status flag.
 *
 * Implements    : PINS_DRV_GetOverrunIntFlag_Activity
 *END**************************************************************************/
uint32_t PINS_DRV_GetOverrunIntFlag(void)
{
    return PINS_GetOverrunIntFlag();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : PINS_DRV_GetFilterInputValue
 * Description   : This function gets the entire filter input values of IRQ and
 * NMI pins.
 *
 * Implements    : PINS_DRV_GetFilterInputValue_Activity
 *END**************************************************************************/
uint32_t PINS_DRV_GetFilterInputValue(void)
{
    return PINS_GetFilterInputValue();
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
