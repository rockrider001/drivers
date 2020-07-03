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

/**
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
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially Boolean'
 * to 'essentially unsigned'. This is required by the conversion of a bool into a bit.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3,  Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression
 * (wider essential type for the destination).
 * This is required by the conversion of a 16 bits register into uint32_t type.
 */

#include "flexpwm_driver.h"
#include "interrupt_manager.h"
#include "clock_manager.h"
#include <assert.h>
#include <stddef.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

 #define FLEXPWM_DTSRCSEL_SIZE  (16U)
 #define FLEXPWM_DTSRCSEL_VALUE (4U)
 #define FLEXPWM_SWCOUT_SIZE    (8U)
 #define FLEXPWM_SWCOUT_VALUE   (2U)
 #define FLEXPWM_INIT_VAL       (0U)

/*******************************************************************************
 * Variables
 ******************************************************************************/
 /*! @brief Table of base addresses for eFlexPWM instances. */
static FlexPWM_Type * const g_flexpwmBase[FlexPWM_INSTANCE_COUNT] = FlexPWM_BASE_PTRS;

/*! @brief Interrupt vectors for the eFlexPWM peripheral. */
static const IRQn_Type g_flexpwmCmpIrqId[FlexPWM_INSTANCE_COUNT][FlexPWM_Compare_IRQS_COUNT] = FLEXPWM_COMPARE_IRQS;
static const IRQn_Type g_flexpwmReloadIrqId[FlexPWM_INSTANCE_COUNT][FlexPWM_Reload_IRQS_COUNT] = FLEXPWM_RELOAD_IRQS;
static const IRQn_Type g_flexpwmCapIrqId[FlexPWM_INSTANCE_COUNT][FlexPWM_Capture_IRQS_COUNT] = FLEXPWM_CAPTURE_IRQS;
static const IRQn_Type g_flexpwmRerrIrqId[FlexPWM_Reload_Error_IRQS_COUNT] = FLEXPWM_RELOAD_ERROR_IRQS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_Deinit
 * Description   : Deinitialize an instance of the FlexPWM module.
 * The function deinitializes an instance of the FlexPWM module restoring all the
 * registers to their reset values.
 *
 * Implements    : FLEXPWM_DRV_Deinit_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    uint16_t subModule;

    /* Stop Counter*/
    base->MCTRL     = FlexPWM_MCTRL_CLDOK_MASK;
    base->DTSRCSEL  = 0x0U;

    /* Clear the subModule specific settings*/
    for (subModule = 0U; subModule < FlexPWM_SUB_COUNT; subModule++)
    {
        base->SUB[subModule].INIT       = 0x0U;
        /* Make sure that FlexPWM_DTSRCSEL register is reset to its default value 0x0U. */
        base->SUB[subModule].CTRL2 &= (uint16_t)(~(FlexPWM_CTRL2_FRCEN_MASK |
                                                FlexPWM_CTRL2_FORCE_SEL_MASK));
        base->SUB[subModule].CTRL2 |= (uint16_t)FlexPWM_CTRL2_FORCE_SEL(ForceOutputLocalForce);
        base->SUB[subModule].CTRL2 |= (uint16_t)FlexPWM_CTRL2_FORCE(true);

        base->SUB[subModule].CTRL2      = 0x0U;
        base->SUB[subModule].CTRL1      = (uint16_t)(FlexPWM_CTRL1_FULL_MASK |
                                                     FlexPWM_CTRL1_LDMOD_MASK);
        base->SUB[subModule].VAL0       = 0x0U;
        base->SUB[subModule].VAL1       = 0x0U;
        base->SUB[subModule].VAL2       = 0x0U;
        base->SUB[subModule].VAL3       = 0x0U;
        base->SUB[subModule].VAL4       = 0x0U;
        base->SUB[subModule].VAL5       = 0x0U;
        base->SUB[subModule].OCTRL      = 0x0U;
        base->SUB[subModule].STS        = (uint16_t)FLEXPWM_ALL_INT_FLAG;
        base->SUB[subModule].INTEN      = 0x0U;
        base->SUB[subModule].DMAEN      = 0x0U;
        base->SUB[subModule].TCTRL      = 0x0U;
        base->SUB[subModule].DISMAP     = (uint16_t)(FlexPWM_DISMAP_DISA_MASK |
                                                     FlexPWM_DISMAP_DISB_MASK |
                                                     FlexPWM_DISMAP_DISX_MASK);
        base->SUB[subModule].DTCNT0     = FlexPWM_DTCNT0_DTCNT0_MASK;
        base->SUB[subModule].DTCNT1     = FlexPWM_DTCNT1_DTCNT1_MASK;
        base->SUB[subModule].CAPTCTRLX  = 0x0U;
        base->SUB[subModule].CAPTCMPX   = 0x0U;
        /* Set LDOK bits in order to load the buffered registers */
        base->MCTRL                     = FlexPWM_MCTRL_LDOK(1UL << subModule);
        base->SUB[subModule].CTRL1      = FlexPWM_CTRL1_FULL_MASK;
    }
    base->OUTEN     = 0x0U;
    base->MASK      = 0x0U;
    base->SWCOUT    = 0x0U;
    base->FCTRL     = 0x0U;
    base->FSTS      = FlexPWM_FSTS_FFLAG_MASK;
    base->FFILT     = 0x0U;
#ifdef FEATURE_FLEXPWM_HAS_FCTRL2
    base->FCTRL2    = 0x0U;
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetDefaultConfig
 * Description   : This function shall populate the Structures needed for
 *                 initializing the driver with the default values
 *
 * Implements    : FLEXPWM_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_GetDefaultConfig(flexpwm_module_setup_t * const moduleSetupParams,
                                  flexpwm_module_signal_setup_t * const signalParams)
{
    DEV_ASSERT(moduleSetupParams != NULL);
    DEV_ASSERT(signalParams != NULL);

    moduleSetupParams->cntrInitSel = InitSrcLocalSync;
    moduleSetupParams->clkSrc = ClkSrcPwmPeriphClk;
    moduleSetupParams->prescaler = PwmDividedBy1;
    moduleSetupParams->clockFreq = 0U;
    moduleSetupParams->chnlPairOper = FlexPwmIndependent;
    moduleSetupParams->complementarySourceSel = FlexPwmComplementarySource23;
    moduleSetupParams->reloadLogic = FlexPwmReloadPwmFullCycle;
    moduleSetupParams->reloadSource = FLEXPWM_LOCAL_RELOAD_SIGNAL;
    moduleSetupParams->reloadFreq = PwmLoadEvery1Oportunity;
    moduleSetupParams->forceTrig = ForceOutputLocalForce;

    signalParams->pwmPeriod = 10000U;
    signalParams->flexpwmType = FlexPwmCentreAligned;
    signalParams->pwmAPulseWidth = 5000U;
    signalParams->pwmBPulseWidth = 2500U;
    signalParams->pwmAShift = 0U;
    signalParams->pwmBShift = 0U;
    signalParams->pwmAOuten = true;
    signalParams->pwmBOuten = true;
    signalParams->pwmXOuten = false;
    signalParams->pwmAPolarity = false;
    signalParams->pwmBPolarity = false;
    signalParams->pwmXPolarity = false;
    signalParams->pwmAFaultDisableMask = 0U;
    signalParams->pwmAfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0;
    signalParams->pwmBFaultDisableMask = 0U;
    signalParams->pwmBfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0;
    signalParams->pwmXFaultDisableMask = 0U;
    signalParams->pwmXfaultState = FLEXPWM_OUTPUT_STATE_LOGIC_0;

}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetupPwm
 * Description   : Set pwm signal properties and module settings.
 * This function will set the pwm signal for the FlexPWM instance including the
 * module settings for the mentioned signal.
 *
 * Implements    : FLEXPWM_DRV_SetupPwm_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetupPwm(const uint32_t instance, const flexpwm_module_t subModule,
                          flexpwm_module_setup_t * const moduleSetupParams,
                          const flexpwm_module_signal_setup_t * const signalParams)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(moduleSetupParams != NULL);
    DEV_ASSERT(signalParams != NULL);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    /* Table of FlexPWM clock names as defined in clock manager. */
    static const clock_names_t s_flexpwmClkNames[FlexPWM_INSTANCE_COUNT] = FLEXPWM_CLOCK_NAMES;

    clock_names_t instanceClkName = s_flexpwmClkNames[instance];

    status_t clkStatus;
    uint32_t clockFreq;

    /* Clear CTRL2 flags*/
    base->SUB[subModule].CTRL2 &= ~((uint16_t)(FlexPWM_CTRL2_INDEP_MASK | FlexPWM_CTRL2_INIT_SEL_MASK |
                                               FlexPWM_CTRL2_CLK_SEL_MASK | FlexPWM_CTRL2_FORCE_SEL_MASK));

    /* Set submodule settings */
    base->SUB[subModule].CTRL2 |= FlexPWM_CTRL2_INDEP(moduleSetupParams->chnlPairOper)      |
                                  FlexPWM_CTRL2_INIT_SEL(moduleSetupParams->cntrInitSel)    |
                                  FlexPWM_CTRL2_CLK_SEL(moduleSetupParams->clkSrc)          |
                                  FlexPWM_CTRL2_RELOAD_SEL(moduleSetupParams->reloadSource) |
                                  FlexPWM_CTRL2_FORCE_SEL(moduleSetupParams->forceTrig);

    /* Clear CTRL1 flags*/
    base->SUB[subModule].CTRL1 &= ~((uint16_t)(FlexPWM_CTRL1_PRSC_MASK | FlexPWM_CTRL1_FULL_MASK |
                                               FlexPWM_CTRL1_HALF_MASK | FlexPWM_CTRL1_LDFQ_MASK |
                                               FlexPWM_CTRL1_LDMOD_MASK));

    /* Set submodule settings */
    base->SUB[subModule].CTRL1 |= FlexPWM_CTRL1_PRSC(moduleSetupParams->prescaler)                      |
                                  FlexPWM_CTRL1_FULL(moduleSetupParams->reloadLogic)                    |
                                  FlexPWM_CTRL1_HALF(((uint16_t)moduleSetupParams->reloadLogic) >> (1U))|
                                  FlexPWM_CTRL1_LDFQ(moduleSetupParams->reloadFreq);

    /* Buffered registers of this submodule are loaded and take effect immediately upon LDOK being set. */
    if(moduleSetupParams->reloadLogic == FlexPwmReloadImmediate)
    {
        base->SUB[subModule].CTRL1 |= (uint16_t)FlexPWM_CTRL1_LDMOD(1U);
    }

    /* Clear IPOL bit from MCTRL corresponding to the subModule that will be configured*/
    base->MCTRL &= ~((uint16_t)FlexPWM_MCTRL_IPOL(((uint16_t)1U) << (uint16_t)subModule));

    /* Set the source for complementary PWM generation */
    base->MCTRL |= FlexPWM_MCTRL_IPOL(((uint16_t)moduleSetupParams->complementarySourceSel) << (uint16_t)subModule);

    /* reset deadtime registers */
    base->SUB[subModule].DTCNT0 &= ~FlexPWM_DTCNT0_DTCNT0_MASK;
    base->SUB[subModule].DTCNT1 &= ~FlexPWM_DTCNT1_DTCNT1_MASK;

    /* Set up the fault input pin for the PWM output disable */
    base->SUB[subModule].DISMAP = (uint16_t)FlexPWM_DISMAP_DISA(signalParams->pwmAFaultDisableMask) | \
                                  (uint16_t)FlexPWM_DISMAP_DISB(signalParams->pwmBFaultDisableMask) | \
                                  (uint16_t)FlexPWM_DISMAP_DISX(signalParams->pwmXFaultDisableMask);
    /* Set up the fault state for the PWM output during fault conditions */
    base->SUB[subModule].OCTRL = FlexPWM_OCTRL_PWMAFS(signalParams->pwmAfaultState) | \
                                 FlexPWM_OCTRL_PWMBFS(signalParams->pwmBfaultState) | \
                                 FlexPWM_OCTRL_PWMXFS(signalParams->pwmXfaultState);

    switch(moduleSetupParams->clkSrc) {
    case ClkSrcPwmPeriphClk:
        /* Get the FlexPWM clock as configured in the clock manager */
        clkStatus = CLOCK_SYS_GetFreq(instanceClkName, &clockFreq);
        DEV_ASSERT(clkStatus == STATUS_SUCCESS);
        (void) clkStatus;
        moduleSetupParams->clockFreq = clockFreq / (1UL << moduleSetupParams->prescaler );
        DEV_ASSERT(moduleSetupParams->clockFreq != 0UL);
        DEV_ASSERT(signalParams->pwmAPulseWidth <= moduleSetupParams->clockFreq);
        DEV_ASSERT(signalParams->pwmBPulseWidth <= moduleSetupParams->clockFreq);
        break;
    case ClkSrcPwmExtClk:
        /* Do nothing, the clock is the same as passed in the configuration structure */
        break;
    case ClkSrcPwm0Clk:
        /* Do nothing, the clock is the same as passed in the configuration structure*/
        break;
    default:
        /* Do nothing*/
        break;
    }

    /* Configure the PWM signal */
    FLEXPWM_DRV_UpdatePwmSignal(instance, subModule, signalParams);

    /*Clear output polarity bits*/
    base->SUB[subModule].OCTRL &= ~((uint16_t)(FlexPWM_OCTRL_POLA_MASK | FlexPWM_OCTRL_POLB_MASK |
                                               FlexPWM_OCTRL_POLX_MASK));

    /* Set output polarity */
    base->SUB[subModule].OCTRL |= FlexPWM_OCTRL_POLA(((signalParams->pwmAPolarity == true) ? 1UL : 0UL)) |
                                  FlexPWM_OCTRL_POLB(((signalParams->pwmBPolarity == true) ? 1UL : 0UL)) |
                                  FlexPWM_OCTRL_POLX(((signalParams->pwmXPolarity == true) ? 1UL : 0UL));

}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_UpdatePwmSignal
 * Description   : Set the pwm signal properties.
 * This function will set the settings of the PWM signal.
 *
 * Implements    : FLEXPWM_DRV_UpdatePwmSignal_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_UpdatePwmSignal(const uint32_t instance, const flexpwm_module_t subModule,
                                 const flexpwm_module_signal_setup_t * const signalParams)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(signalParams != NULL);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    uint16_t pwmOffset;

    if (signalParams->pwmAOuten == true)
    {
        base->OUTEN |= FlexPWM_OUTEN_PWMA_EN(1UL << subModule);
    }
    else
    {
        base->OUTEN &= ~FlexPWM_OUTEN_PWMA_EN(1UL << subModule);
    }

    if (signalParams->pwmBOuten == true)
    {
        base->OUTEN |= FlexPWM_OUTEN_PWMB_EN(1UL << subModule);
    }
    else
    {
        base->OUTEN &= ~FlexPWM_OUTEN_PWMB_EN(1UL << subModule);
    }

    if (signalParams->pwmXOuten == true)
    {
        base->OUTEN |= FlexPWM_OUTEN_PWMX_EN(1UL << subModule);
    }
    else
    {
        base->OUTEN &= ~FlexPWM_OUTEN_PWMX_EN(1UL << subModule);
    }

    if (signalParams->flexpwmType == FlexPwmDoubleSwitching)
    {
        base->SUB[subModule].CTRL1 |= ((uint16_t)FlexPWM_CTRL1_DBLEN(1UL));
    }
    else
    {
        base->SUB[subModule].CTRL1 &= ~((uint16_t)FlexPWM_CTRL1_DBLEN(1UL));
    }

    base->SUB[subModule].VAL1 = FlexPWM_VAL1_VAL1(signalParams->pwmPeriod);
    base->SUB[subModule].VAL0 = FlexPWM_VAL0_VAL0(signalParams->pwmPeriod / 2U);
    base->SUB[subModule].INIT = FLEXPWM_INIT_VAL;

    switch(signalParams->flexpwmType)
    {
    case FlexPwmCentreAligned:
        /* Calculate offset for centring the pwm for PWMA*/
        pwmOffset = (uint16_t)((signalParams->pwmPeriod - signalParams->pwmAPulseWidth) / 2U);
        base->SUB[subModule].VAL2 = pwmOffset;
        base->SUB[subModule].VAL3 = pwmOffset + signalParams->pwmAPulseWidth;

        /* Calculate offset for centring the pwm for PWMB*/
        pwmOffset = (uint16_t)((signalParams->pwmPeriod - signalParams->pwmBPulseWidth) / 2U);
        base->SUB[subModule].VAL4 = pwmOffset;
        base->SUB[subModule].VAL5 = pwmOffset + signalParams->pwmBPulseWidth;
        break;
    case FlexPwmEdgeAligned:
        base->SUB[subModule].VAL2 = FLEXPWM_INIT_VAL;
        base->SUB[subModule].VAL4 = FLEXPWM_INIT_VAL;
        base->SUB[subModule].VAL3 = signalParams->pwmAPulseWidth;
        base->SUB[subModule].VAL5 = signalParams->pwmBPulseWidth;
        break;
    case FlexPwmPhaseShifted:
        /* Check if the pulse widths are the same */
        DEV_ASSERT(signalParams->pwmAPulseWidth == signalParams->pwmBPulseWidth);

        /* Configure PWM A pulse width */
        base->SUB[subModule].VAL2 = signalParams->pwmAShift;
        base->SUB[subModule].VAL3 = signalParams->pwmAShift + signalParams->pwmAPulseWidth;

        /* Configure PWM B pulse width */
        base->SUB[subModule].VAL4 = signalParams->pwmBShift;
        base->SUB[subModule].VAL5 = signalParams->pwmBShift + signalParams->pwmBPulseWidth;
        break;
    case FlexPwmDoubleSwitching:
        /* Calculate offset for centring the pwm for PWMA*/
        pwmOffset = (uint16_t)((signalParams->pwmPeriod - signalParams->pwmAPulseWidth) / 2U);
        base->SUB[subModule].VAL2 = pwmOffset;
        base->SUB[subModule].VAL3 = pwmOffset + signalParams->pwmAPulseWidth;

        /* Calculate offset for centring the pwm for PWMB*/
        pwmOffset = (uint16_t)((signalParams->pwmPeriod - signalParams->pwmBPulseWidth) / 2U);
        base->SUB[subModule].VAL4 = pwmOffset;
        base->SUB[subModule].VAL5 = pwmOffset + signalParams->pwmBPulseWidth;
        break;
    default:
        /* do nothing*/
        DEV_ASSERT(false);
        break;
    }

    /*Clear output polarity bits*/
    base->SUB[subModule].OCTRL &= ~((uint16_t)(FlexPWM_OCTRL_POLA_MASK | FlexPWM_OCTRL_POLB_MASK | FlexPWM_OCTRL_POLX_MASK));

    /* Set output polarity */
    base->SUB[subModule].OCTRL |= FlexPWM_OCTRL_POLA(((signalParams->pwmAPolarity == true) ? 1UL : 0UL)) |
                                  FlexPWM_OCTRL_POLB(((signalParams->pwmBPolarity == true) ? 1UL : 0UL)) |
                                  FlexPWM_OCTRL_POLX(((signalParams->pwmXPolarity == true) ? 1UL : 0UL));

}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_UpdatePwmPeriod
 * Description   : Update pwm period.
 * This function will update the PWM signal period. The function will also update
 * the mid cycle reload point(VAL0) to half the PWM period.
 *
 * Implements    : FLEXPWM_DRV_UpdatePwmPeriod_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_UpdatePwmPeriod(const uint32_t instance, const flexpwm_module_t subModule,
                                 const uint32_t pwmPeriod)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].VAL1 = FlexPWM_VAL1_VAL1(pwmPeriod);
    base->SUB[subModule].VAL0 = FlexPWM_VAL0_VAL0(pwmPeriod / 2U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_UpdatePulseWidth
 * Description   : Update pwm pulse width.
 * This function will update the PWM signal pulse width.
 *
 * Implements    : FLEXPWM_DRV_UpdatePulseWidth_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_UpdatePulseWidth(const uint32_t instance, const flexpwm_module_t subModule,
                                  const uint16_t pwmAPulseWidth, const uint16_t pwmBPulseWidth,
                                  const flexpwm_signal_type_t pwmType)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    uint16_t pwmOffset;
    uint16_t pwmShift;

    switch(pwmType)
    {
    case FlexPwmCentreAligned:
        /* Calculate offset for centring the pwm for PWMA*/
        pwmOffset = ((base->SUB[subModule].VAL1 - pwmAPulseWidth) / 2U);
        base->SUB[subModule].VAL2 = pwmOffset;
        base->SUB[subModule].VAL3 = pwmOffset + pwmAPulseWidth;

        /* Calculate offset for centring the pwm for PWMB*/
        pwmOffset = ((base->SUB[subModule].VAL1 - pwmBPulseWidth) / 2U);
        base->SUB[subModule].VAL4 = pwmOffset;
        base->SUB[subModule].VAL5 = pwmOffset + pwmBPulseWidth;
        break;
    case FlexPwmEdgeAligned:
        base->SUB[subModule].VAL2 = FLEXPWM_INIT_VAL;
        base->SUB[subModule].VAL4 = FLEXPWM_INIT_VAL;
        base->SUB[subModule].VAL3 = pwmAPulseWidth;
        base->SUB[subModule].VAL5 = pwmBPulseWidth;
        break;
    case FlexPwmPhaseShifted:
        /* Check if the pulse widths are the same */
        DEV_ASSERT(pwmAPulseWidth == pwmBPulseWidth);

        /* Configure PWM A pulse width */
        pwmShift = base->SUB[subModule].VAL2;
        base->SUB[subModule].VAL3 = pwmShift + pwmAPulseWidth;

        /* Configure PWM B pulse width */
        pwmShift = base->SUB[subModule].VAL4;
        base->SUB[subModule].VAL5 = pwmShift + pwmBPulseWidth;
        break;
    case FlexPwmDoubleSwitching:
        /* Calculate offset for centring the pwm for PWMA*/
        pwmOffset = ((base->SUB[subModule].VAL1 - pwmAPulseWidth) / 2U);
        base->SUB[subModule].VAL2 = pwmOffset;
        base->SUB[subModule].VAL3 = pwmOffset + pwmAPulseWidth;

        /* Calculate offset for centring the pwm for PWMB*/
        pwmOffset = ((base->SUB[subModule].VAL1 - pwmBPulseWidth) / 2U);
        base->SUB[subModule].VAL4 = pwmOffset;
        base->SUB[subModule].VAL5 = pwmOffset + pwmBPulseWidth;
        break;
    default:
        /* do nothing*/
        DEV_ASSERT(false);
        break;
    }
 }

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_UpdateMidCycleReload
 * Description   : Set FlexPWM mid cycle reload value.
 * This function set the mid cycle reload value(VAL0).
 *
 * Implements    : FLEXPWM_DRV_UpdateMidCycleReload_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_UpdateMidCycleReload(const uint32_t instance, const flexpwm_module_t subModule,
                                      const uint16_t value)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].VAL0 = FlexPWM_VAL0_VAL0(value);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_LoadCommands
 * Description   : Set FlexPWM trigger command.
 * This function will load the effective values in the modules registers based
 * on the provided mask
 *
 * Implements    : FLEXPWM_DRV_LoadCommands_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_LoadCommands(const uint32_t instance, const uint32_t subModules)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    /* Set the corresponding bit so that the new values are loaded */
    base->MCTRL |= FlexPWM_MCTRL_LDOK(subModules);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ClearLDOK
 * Description   : Clear the LDOK signal for specified sub-modules.
 * This function will clears the LDOK bits for the sub-modules specified in the
 * provided mask.
 *
 * Implements    : FLEXPWM_DRV_ClearLDOK_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ClearLDOK(const uint32_t instance, const uint32_t subModuleMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    /* Set the corresponding bit so that the specific LDOK bits are cleared */
    base->MCTRL |= FlexPWM_MCTRL_CLDOK(subModuleMask);
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetCounterValue
 * Description   : This function will return the current sub-module counter value.
 *
 * Implements    : FLEXPWM_DRV_GetCounterValue_Activity
 *END**************************************************************************/
uint16_t FLEXPWM_DRV_GetCounterValue(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    const FlexPWM_Type * const base = g_flexpwmBase[instance];

    /* Gets the timer channel counts */
    return (base->SUB[subModule].CNT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetPeriod
 * Description   : This function will return the current sub-module PWM period.
 *
 * Implements    : FLEXPWM_DRV_GetPeriod_Activity
 *END**************************************************************************/
uint16_t FLEXPWM_DRV_GetPeriod(const uint32_t instance,
                               const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    const FlexPWM_Type * const base = g_flexpwmBase[instance];

    /* Gets the count value */
    return (base->SUB[subModule].VAL1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetCmpRegValue
 * Description   : This function will return the current value in one of the
 *                 sub-module value registers.
 *
 * Implements    : FLEXPWM_DRV_GetCmpRegValue_Activity
 *END**************************************************************************/
uint16_t FLEXPWM_DRV_GetCmpRegValue(const uint32_t instance,
                                    const flexpwm_module_t subModule,
                                    const flexpwm_val_regs_t valReg)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    FlexPWM_Type * const base = g_flexpwmBase[instance];

    volatile uint16_t * valueRegisters[6] = {
                     &(base->SUB[subModule].VAL0), &(base->SUB[subModule].VAL1),
                     &(base->SUB[subModule].VAL2), &(base->SUB[subModule].VAL3),
                     &(base->SUB[subModule].VAL4), &(base->SUB[subModule].VAL5)};

    return *(valueRegisters[(uint16_t)valReg]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetCmpRegValue
 * Description   : This function will set the current value in one of the
 *                 sub-module value registers.
 *
 * Implements    : FLEXPWM_DRV_SetCmpRegValue_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetCmpRegValue(const uint32_t instance,
                                const flexpwm_module_t subModule,
                                const flexpwm_val_regs_t valReg,
                                const uint16_t newValue)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    FlexPWM_Type * const base = g_flexpwmBase[instance];

    volatile uint16_t * valueRegisters[6] = {
                     &(base->SUB[subModule].VAL0), &(base->SUB[subModule].VAL1),
                     &(base->SUB[subModule].VAL2), &(base->SUB[subModule].VAL3),
                     &(base->SUB[subModule].VAL4), &(base->SUB[subModule].VAL5)};

    *(valueRegisters[(uint16_t)valReg]) = newValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetTriggerCmd
 * Description   : Set FlexPWM trigger command.
 * This function will set the trigger command for the specified PWM subModule
 *
 * Implements    : FLEXPWM_DRV_SetTriggerCmd_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetTriggerCmd(const uint32_t instance,
                               const flexpwm_module_t subModule,
                               const flexpwm_val_regs_t trigger,
                               const bool activate)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    if (activate == true)
    {
        base->SUB[subModule].TCTRL |= FlexPWM_TCTRL_OUT_TRIG_EN(1UL << trigger);
    }
    else
    {
        base->SUB[subModule].TCTRL &= ~ ((uint16_t)FlexPWM_TCTRL_OUT_TRIG_EN(1UL << trigger));
    }

 }

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetTriggerVal
 * Description   : Set FlexPWM Trigger value.
 * This function will set the trigger value for the PWM subModule
 *
 * Implements    : FLEXPWM_DRV_SetTriggerVal_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetTriggerVal(const uint32_t instance, const flexpwm_module_t subModule,
                               const flexpwm_val_regs_t trigger, const uint16_t triggerVal)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    /* Table of FlexPWM compare value registers. */

    volatile uint16_t * valueRegisters[6] = {&(base->SUB[subModule].VAL0), &(base->SUB[subModule].VAL1),
                                          &(base->SUB[subModule].VAL2), &(base->SUB[subModule].VAL3),
                                          &(base->SUB[subModule].VAL4), &(base->SUB[subModule].VAL5)};

    *(valueRegisters[(uint16_t)trigger]) = triggerVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_CounterStart
 * Description   : Stop FlexPWM counter.
 * This function starts the the PWM sub-module counters.
 *
 * Implements    : FLEXPWM_DRV_CounterStart_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_CounterStart(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->MCTRL |= FlexPWM_MCTRL_LDOK(1UL << subModule);
    base->MCTRL |= FlexPWM_MCTRL_RUN(1UL << subModule);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_CounterStop
 * Description   : Stop FlexPWM counter.
 * This function will stop the PWM subModule counters.
 *
 * Implements    : FLEXPWM_DRV_CounterStop_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_CounterStop(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->MCTRL &= ~FlexPWM_MCTRL_RUN(1UL << subModule);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ForcePwmOutput
 * Description   : Set FlexPWM pwm forced output signal.
 * This function will set the force output signal for the PWM signal.
 *
 * Implements    : FLEXPWM_DRV_ForcePwmOutput_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ForcePwmOutput(const uint32_t instance, const flexpwm_module_t subModule, const bool forceInit,
                                const bool forceEnable, const flexpwm_force_output_trigger_t forceTrig)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    uint16_t ctrl2;

    base->SUB[subModule].CTRL2 &= (uint16_t)(~(FlexPWM_CTRL2_FRCEN_MASK |
                                   FlexPWM_CTRL2_FORCE_SEL_MASK));

    base->SUB[subModule].CTRL2 |= FlexPWM_CTRL2_FRCEN(forceEnable) |
                                  FlexPWM_CTRL2_FORCE_SEL(forceTrig);

    ctrl2 = base->SUB[subModule].CTRL2;
    ctrl2 |= FlexPWM_CTRL2_FORCE(((forceInit == true) ? 1UL : 0UL));
    base->SUB[subModule].CTRL2 = ctrl2;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_EnableModuleInterrupts
 * Description   : Enable interrupts for the FlexPWM module.
 * This function will enable the module interrupts.
 *
 * Implements    : FLEXPWM_DRV_EnableModuleInterrupts_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_EnableModuleInterrupts(const uint32_t instance, const flexpwm_module_t subModule,
                                        const uint16_t interruptMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    if ((interruptMask & ((uint16_t)FLEXPWM_CMP_ALL_VAL_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN |= FlexPWM_INTEN_CMPIE(interruptMask);
        INT_SYS_EnableIRQ(g_flexpwmCmpIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_RELOAD_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN |= FlexPWM_INTEN_RIE(interruptMask >> 6U);
        INT_SYS_EnableIRQ(g_flexpwmReloadIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_CAPTURE_X0_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN |= FlexPWM_INTEN_CX0IE(interruptMask >> 7U);
        INT_SYS_EnableIRQ(g_flexpwmCapIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_CAPTURE_X1_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN |= FlexPWM_INTEN_CX1IE(interruptMask >> 8U);
        INT_SYS_EnableIRQ(g_flexpwmCapIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_RELOAD_ERR_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN |= FlexPWM_INTEN_REIE(interruptMask >> 9U);
        INT_SYS_EnableIRQ(g_flexpwmRerrIrqId[instance]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DisableModuleInterrupts
 * Description   : Disable interrupts for the FlexPWM module.
 * This function will disable the module interrupts.
 *
 * Implements    : FLEXPWM_DRV_DisableModuleInterrupts_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DisableModuleInterrupts(const uint32_t instance, const flexpwm_module_t subModule,
                                         const uint16_t interruptMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    if ((interruptMask & ((uint16_t)FLEXPWM_CMP_ALL_VAL_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN &= ~FlexPWM_INTEN_CMPIE(interruptMask);
        INT_SYS_DisableIRQ(g_flexpwmCmpIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_RELOAD_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN &= ~FlexPWM_INTEN_RIE(interruptMask >> 6U);
        INT_SYS_DisableIRQ(g_flexpwmReloadIrqId[instance][subModule]);
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_CAPTURE_X0_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN &= ~FlexPWM_INTEN_CX0IE(interruptMask >> 7U);
        if ((base->SUB[subModule].INTEN & ((uint16_t)FLEXPWM_CAPTURE_X1_INT_ENABLE)) == 0x0U)
        {
            INT_SYS_DisableIRQ(g_flexpwmCapIrqId[instance][subModule]);
        }
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_CAPTURE_X1_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN &= ~FlexPWM_INTEN_CX1IE(interruptMask >> 8U);
        if ((base->SUB[subModule].INTEN & ((uint16_t)FLEXPWM_CAPTURE_X0_INT_ENABLE)) == 0x0U)
        {
            INT_SYS_DisableIRQ(g_flexpwmCapIrqId[instance][subModule]);
        }
    }
    if ((interruptMask & ((uint16_t)FLEXPWM_RELOAD_ERR_INT_ENABLE)) != 0x0U)
    {
        base->SUB[subModule].INTEN &= ~FlexPWM_INTEN_REIE(interruptMask >> 9U);
        INT_SYS_DisableIRQ(g_flexpwmRerrIrqId[instance]);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ConfigureInputCapture
 * Description   : Configures the input capture circuit based on the configuration structure.
 *
 * Implements    : FLEXPWM_DRV_ConfigureInputCapture_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ConfigureInputCapture(const uint32_t instance, const flexpwm_module_t subModule,
                                       const flexpwm_input_capture_config_t * const inputCapture)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    /* Clear the bit-fields that will be set */
    base->SUB[subModule].CAPTCTRLX &= ~((uint16_t)(FlexPWM_CAPTCTRLX_CFXWM_MASK | FlexPWM_CAPTCTRLX_EDGCNTX_EN_MASK |
                                        FlexPWM_CAPTCTRLX_INPSELX_MASK | FlexPWM_CAPTCTRLX_EDGX1_MASK |
                                        FlexPWM_CAPTCTRLX_EDGX0_MASK | FlexPWM_CAPTCTRLX_ONESHOTX_MASK));

    /* Set the input capture based on the settings in the configuration structure */
    base->SUB[subModule].CAPTCTRLX |= (FlexPWM_CAPTCTRLX_CFXWM(inputCapture->watermark) |
                                       FlexPWM_CAPTCTRLX_EDGCNTX_EN(inputCapture->edgeCntEn) |
                                       FlexPWM_CAPTCTRLX_INPSELX(inputCapture->inputSelect) |
                                       FlexPWM_CAPTCTRLX_EDGX1(inputCapture->edgeSelect1) |
                                       FlexPWM_CAPTCTRLX_EDGX0(inputCapture->edgeSelect0) |
                                       FlexPWM_CAPTCTRLX_ONESHOTX(inputCapture->oneShot));

    /* Clear the edge compare value. */
    base->SUB[subModule].CAPTCMPX &= ~FlexPWM_CAPTCMPX_EDGCMPX_MASK;

    /* Set the new the edge compare value based on the configuration structure. */
    base->SUB[subModule].CAPTCMPX |= FlexPWM_CAPTCMPX_EDGCMPX(inputCapture->edgeCompareVal);


}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_StartInputCapture
 * Description   : Starts the input capture event.
 *
 * Implements    : FLEXPWM_DRV_StartInputCapture_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_StartInputCapture(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CAPTCTRLX |= FlexPWM_CAPTCTRLX_ARMX_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_StopInputCapture
 * Description   : Stops the input capture event.
 *
 * Implements    : FLEXPWM_DRV_StopInputCapture_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_StopInputCapture(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CAPTCTRLX &= ~FlexPWM_CAPTCTRLX_ARMX_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetEdgeCounter
 * Description   : Returns the number of edges counted to the moment.
 *
 * Implements    : FLEXPWM_DRV_GetEdgeCounter_Activity
 *END**************************************************************************/
uint32_t FLEXPWM_DRV_GetEdgeCounter(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    const FlexPWM_Type * const base = g_flexpwmBase[instance];
    uint32_t edgeCounterValue = (uint32_t)base->SUB[subModule].CAPTCMPX;

    return ((edgeCounterValue & FlexPWM_CAPTCMPX_EDGCNTX_MASK) >> FlexPWM_CAPTCMPX_EDGCNTX_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetEdgeCompare
 * Description   : Set the number of edges to be counted
 *
 * Implements    : FLEXPWM_DRV_SetEdgeCompare_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetEdgeCompare(const uint32_t instance, const flexpwm_module_t subModule,
                                const uint16_t value)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    uint16_t compareValue;

    compareValue = base->SUB[subModule].CAPTCMPX;
    compareValue &= ~FlexPWM_CAPTCMPX_EDGCMPX_MASK;
    compareValue |= FlexPWM_CAPTCMPX_EDGCMPX(value);

    base->SUB[subModule].CAPTCMPX = compareValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_EnableEdgeCounting
 * Description   : Enable the edge counting functionality.
 *
 * Implements    : FLEXPWM_DRV_EnableEdgeCounting_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_EnableEdgeCounting(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CAPTCTRLX |= FlexPWM_CAPTCTRLX_EDGCNTX_EN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DisableEdgeCounting
 * Description   : Disable the edge counting functionality.
 *
 * Implements    : FLEXPWM_DRV_DisableEdgeCounting_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DisableEdgeCounting(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CAPTCTRLX &= ~FlexPWM_CAPTCTRLX_EDGCNTX_EN_MASK;
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetCaptureValue
 * Description   : This function populates the user given array with the values in
 *                 the capture register FIFO for which the capture has taken place.
 * Note: The user must allocate the memory pointed by registerValue
 * accordingly dimensioned with the number of words.
 *
 * Implements    : FLEXPWM_DRV_GetCaptureValue_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_GetCaptureValue(const uint32_t instance,
                                 const flexpwm_module_t subModule,
                                 const flexpwm_input_capture_counter_t counterRegister,
                                 uint16_t * const registerValue,
                                 const uint8_t noWords)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(registerValue != NULL);
    DEV_ASSERT(noWords != 0U);
    DEV_ASSERT(noWords <= FLEXPWM_NUM_DEEP_FIFO_CAPTVAL);

    const FlexPWM_Type * const base = g_flexpwmBase[instance];
    uint8_t capturedWords = 0U;
    uint8_t i;

    switch(counterRegister)
    {
    case FLEXPWM_INPUT_CAPTURE_COUNTER_0:
        capturedWords = (uint8_t)((base->SUB[subModule].CAPTCTRLX &
                         FlexPWM_CAPTCTRLX_CX0CNT_MASK) >>
                         FlexPWM_CAPTCTRLX_CX0CNT_SHIFT);

        for(i = 0; (i < noWords) && (i < capturedWords); i++)
        {
            registerValue[i] = base->SUB[subModule].CVAL0;
        }
        break;
    case FLEXPWM_INPUT_CAPTURE_COUNTER_1:
        capturedWords = (uint8_t)((base->SUB[subModule].CAPTCTRLX &
                         FlexPWM_CAPTCTRLX_CX1CNT_MASK) >>
                         FlexPWM_CAPTCTRLX_CX1CNT_SHIFT);

        for(i = 0; (i < noWords) && (i < capturedWords); i++)
        {
            registerValue[i] = base->SUB[subModule].CVAL1;
        }
        break;
    default:
        /* Do nothing */
        break;
    }
}

 /*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetCaptureCycle
 * Description   : This function returns the number of cycles for which the
 *                 capture has taken place.
 *
 * Implements    : FLEXPWM_DRV_GetCaptureCycle_Activity
 *END**************************************************************************/
uint16_t FLEXPWM_DRV_GetCaptureCycle(const uint32_t instance, const flexpwm_module_t subModule,
                                     const flexpwm_input_capture_counter_t counterRegister)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    const FlexPWM_Type * const  base = g_flexpwmBase[instance];
    uint16_t registerValue = 0U;

    switch(counterRegister)
    {
    case FLEXPWM_INPUT_CAPTURE_COUNTER_0:
        registerValue = base->SUB[subModule].CVAL0CYC >> FlexPWM_CVAL0CYC_CVAL0CYC_SHIFT;
        break;
    case FLEXPWM_INPUT_CAPTURE_COUNTER_1:
        registerValue = base->SUB[subModule].CVAL1CYC >> FlexPWM_CVAL1CYC_CVAL1CYC_SHIFT;
        break;
    default:
        /* Do nothing */
        break;
    }

    return registerValue;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetChannelMode
 * Description   : Modifies the input capture channel mode.
 *
 * Implements    : FLEXPWM_DRV_SetChannelMode_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetChannelMode(const uint32_t instance, const flexpwm_module_t subModule,
                                const flexpwm_input_capture_edge_select_t edgeSelect,
                                const flexpwm_input_capture_counter_t counterRegister)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * const  base = g_flexpwmBase[instance];

    switch(counterRegister)
    {
    case FLEXPWM_INPUT_CAPTURE_COUNTER_0:
        base->SUB[subModule].CAPTCTRLX = (base->SUB[subModule].CAPTCTRLX &
                                         (~FlexPWM_CAPTCTRLX_EDGX0_MASK)) |
                                          FlexPWM_CAPTCTRLX_EDGX0(edgeSelect);
        break;
    case FLEXPWM_INPUT_CAPTURE_COUNTER_1:
        base->SUB[subModule].CAPTCTRLX = (base->SUB[subModule].CAPTCTRLX &
                                         (~FlexPWM_CAPTCTRLX_EDGX1_MASK)) |
                                          FlexPWM_CAPTCTRLX_EDGX1(edgeSelect);
        break;
    default:
        /* Do nothing */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DebugEnable
 * Description   : Enable the debug mode of the module.
 *
 * Implements    : FLEXPWM_DRV_DebugEnable_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DebugEnable(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CTRL2 |= FlexPWM_CTRL2_DBGEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DebugDisable
 * Description   : Disable the debug mode of the module.
 *
 * Implements    : FLEXPWM_DRV_DebugDisable_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DebugDisable(const uint32_t instance, const flexpwm_module_t subModule)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].CTRL2 &= ~FlexPWM_CTRL2_DBGEN_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetDeadtime
 * Description   : Updates the value used for the deadtime logic in terms of clock cycles.
 *
 * Implements    : FLEXPWM_DRV_SetDeadtime_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetDeadtime(const uint32_t instance, const flexpwm_module_t subModule,
                             const uint16_t value, const flexpwm_deadtime_counter_t counterRegister)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(value <= FlexPWM_DTCNT0_DTCNT0_MASK);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    switch(counterRegister)
    {
    case FLEXPWM_DEADTIME_COUNTER_0:
        base->SUB[subModule].DTCNT0 &= ~FlexPWM_DTCNT0_DTCNT0_MASK;
        base->SUB[subModule].DTCNT0 |= FlexPWM_DTCNT0_DTCNT0(value);
        break;
    case FLEXPWM_DEADTIME_COUNTER_1:
        base->SUB[subModule].DTCNT1 &= ~FlexPWM_DTCNT1_DTCNT1_MASK;
        base->SUB[subModule].DTCNT1 |= FlexPWM_DTCNT1_DTCNT1(value);
        break;
    default:
        /* Do nothing */
        break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SelectDeadtimeSource
 * Description   : Updates the source selection for the deadtime logic.
 *
 * Implements    : FLEXPWM_DRV_SelectDeadtimeSource_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SelectDeadtimeSource(const uint32_t instance, const uint16_t index, const uint16_t value)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(index < FLEXPWM_DTSRCSEL_SIZE);
    DEV_ASSERT(value < FLEXPWM_DTSRCSEL_VALUE);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    uint16_t deadtimeSelection;

    deadtimeSelection = base->DTSRCSEL;

    /* Used 3UL in order to mask the SELxx_x bits-fields */
    deadtimeSelection &= ~((uint16_t)(3UL << index));
    deadtimeSelection |= (((uint16_t)(value << index)) & ((uint16_t)(3UL << index)));

    base->DTSRCSEL = deadtimeSelection;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SelectSwCtrlOutput
 * Description   : Selects the value to be supplied to the deadtime
 *                 generator instead of the PWM signal.
 *
 * Implements    : FLEXPWM_DRV_SelectSwCtrlOutput_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SelectSwCtrlOutput(const uint32_t instance, const uint32_t index, const uint32_t value)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(index < FLEXPWM_SWCOUT_SIZE);
    DEV_ASSERT(value < FLEXPWM_SWCOUT_VALUE);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    uint16_t swControl;

    swControl = base->SWCOUT;
    swControl &= ~((uint16_t)(1UL << index));
    swControl |= (((uint16_t)(value << index)) & ((uint16_t)(1UL << index)));

    base->SWCOUT = swControl;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_MaskOutput
 * Description   : Selects the masked PWM outputs based on masks for each PWM signal.
 *
 * Implements    : FLEXPWM_DRV_MaskOutput_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_MaskOutput(const uint32_t instance, const uint16_t pwmAMask,
                            const uint16_t pwmBMask, const uint16_t pwmXMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    uint16_t maskOutput;

    maskOutput = base->MASK;
    maskOutput &= ~((uint16_t)(FlexPWM_MASK_MASKA_MASK | FlexPWM_MASK_MASKB_MASK | FlexPWM_MASK_MASKX_MASK));
    maskOutput |= (pwmAMask << FlexPWM_MASK_MASKA_SHIFT) | (pwmBMask << FlexPWM_MASK_MASKB_SHIFT) |
                  (pwmXMask << FlexPWM_MASK_MASKX_SHIFT);

    base->MASK = maskOutput;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ForceApplyMask
 * Description   : Forces the appliance of the PWM masks.
 *
 * Implements    : FLEXPWM_DRV_ForceApplyMask_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ForceApplyMask(const uint32_t instance, const uint32_t mask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->MASK |= FlexPWM_MASK_UPDATE_MASK(mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetInterruptFlag
 * Description   : Returns the interrupt flags based on the mask provided.
 *
 * Implements    : FLEXPWM_DRV_GetInterruptFlag_Activity
 *END**************************************************************************/
uint32_t FLEXPWM_DRV_GetInterruptFlag(const uint32_t instance, const flexpwm_module_t subModule,
                                      const uint32_t interruptFlag)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    const FlexPWM_Type * const base = g_flexpwmBase[instance];

    return (base->SUB[subModule].STS & interruptFlag);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ClearInterruptFlag
 * Description   : Clears the interrupt flags based on the mask provided.
 *
 * Implements    : FLEXPWM_DRV_ClearInterruptFlag_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ClearInterruptFlag(const uint32_t instance, const flexpwm_module_t subModule,
                                    const uint16_t interruptFlag)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->SUB[subModule].STS |= (((uint16_t)interruptFlag) & ((uint16_t) FLEXPWM_ALL_INT_FLAG));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetupFaultProtection
 * Description   : Configures the fault protection according to the configuration structure passed in by the user
 *
 * Implements    : FLEXPWM_DRV_SetupFaultProtection_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetupFaultProtection(const uint32_t instance,
                                      const flexpwm_fault_protection_config_t * const faultProtection)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(faultProtection != NULL);
    DEV_ASSERT(faultProtection->filterCount < (1u << FlexPWM_FFILT_FILT_CNT_WIDTH));
    DEV_ASSERT((faultProtection->faultLevelMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);
    DEV_ASSERT((faultProtection->autoFaultClearMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);
    DEV_ASSERT((faultProtection->failSafeModeMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);
    DEV_ASSERT((faultProtection->fullCycleMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);
#if (FEATURE_FLEXPWM_HAS_COMBINATIONAL_PATH == 1U)
    DEV_ASSERT((faultProtection->combinationalMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);
#endif

    FlexPWM_Type * base = g_flexpwmBase[instance];
    /* Setup fault level, auto clear fault and fail safe mode */
    base->FCTRL &= ~((uint16_t)(FlexPWM_FCTRL_FLVL_MASK | FlexPWM_FCTRL_FAUTO_MASK | FlexPWM_FCTRL_FSAFE_MASK));
    base->FCTRL = (uint16_t)(FlexPWM_FCTRL_FSAFE(faultProtection->failSafeModeMask) | FlexPWM_FCTRL_FAUTO(faultProtection->autoFaultClearMask) | \
                             FlexPWM_FCTRL_FLVL(faultProtection->faultLevelMask));
    /* Setup full cycle re-enable PWM output */
    base->FSTS &= ~FlexPWM_FSTS_FFULL_MASK;
    base->FSTS |= FlexPWM_FSTS_FFULL(faultProtection->fullCycleMask);
#if (FEATURE_FLEXPWM_HAS_COMBINATIONAL_PATH == 1U)
    /* Setup combinational path from fault inputs to PWM outputs */
    base->FCTRL2 &= ~FlexPWM_FCTRL2_NOCOMB_MASK;
    base->FCTRL2 = FlexPWM_FCTRL2_NOCOMB(faultProtection->combinationalMask);
#endif
    /* Setup fault filter */
    base->FFILT &= ~((uint16_t)(FlexPWM_FFILT_FILT_PER_MASK | FlexPWM_FFILT_FILT_CNT_MASK | FlexPWM_FFILT_GSTR_MASK));
    base->FFILT = (uint16_t)(FlexPWM_FFILT_FILT_PER(faultProtection->filterPeriod) | FlexPWM_FFILT_FILT_CNT(faultProtection->filterCount) | \
                             FlexPWM_FFILT_GSTR((faultProtection->glitchStretchEnable == true) ? (uint16_t)1U : (uint16_t)0U));

}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetFaultFilterPeriod
 * Description   : Sets the input fault filter period.
 *
 * Implements    : FLEXPWM_DRV_SetFaultFilterPeriod_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetFaultFilterPeriod(const uint32_t instance, const uint32_t period)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(period < (1UL << FlexPWM_FFILT_FILT_PER_WIDTH));

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->FFILT &= ~FlexPWM_FFILT_FILT_PER_MASK;
    base->FFILT |=  FlexPWM_FFILT_FILT_PER(period);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SetFaultFilterCounter
 * Description   : Sets the input fault filter counter.
 *
 * Implements    : FLEXPWM_DRV_SetFaultFilterCounter_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SetFaultFilterCounter(const uint32_t instance, const uint32_t counter)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT(counter < (1UL << FlexPWM_FFILT_FILT_CNT_WIDTH));

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->FFILT &= ~FlexPWM_FFILT_FILT_CNT_MASK;
    base->FFILT |=  FlexPWM_FFILT_FILT_CNT(counter);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_EnableFaultGlitchStretch
 * Description   : Enables the fault glitch stretching for the input capture
 *                 circuit.
 *
 * Implements    : FLEXPWM_DRV_EnableFaultGlitchStretch_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_EnableFaultGlitchStretch(const uint32_t instance)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->FFILT |= FlexPWM_FFILT_GSTR_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DisableFaultGlitchStretch
 * Description   : Disables the fault glitch stretching for the input capture
 *                 circuit.
 *
 * Implements    : FLEXPWM_DRV_DisableFaultGlitchStretch_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DisableFaultGlitchStretch(const uint32_t instance)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->FFILT &= ~FlexPWM_FFILT_GSTR_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_EnableFaultInterrupt
 * Description   : Enables fault interrupt requests that are set to '1' in the faultInputMask.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to enable multiple interrupt requests.
 *
 * Implements    : FLEXPWM_DRV_EnableFaultInterrupt_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_EnableFaultInterrupt(const uint32_t instance,
                                      const uint8_t faultInputMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT((faultInputMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    const IRQn_Type flexpwmFaultIrqId[FlexPWM_Fault_Input_IRQS_COUNT] = FLEXPWM_FAULT_INPUT_IRQS;

    base->FCTRL |= FlexPWM_FCTRL_FIE(faultInputMask);
    INT_SYS_EnableIRQ(flexpwmFaultIrqId[instance]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_DisableFaultInterrupt
 * Description   : Disables fault interrupt requests that are set to '1' in the faultInputMask.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to disable multiple interrupt requests.
 *
 * Implements    : FLEXPWM_DRV_DisableFaultInterrupt_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_DisableFaultInterrupt(const uint32_t instance,
                                       const uint8_t faultInputMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT((faultInputMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);

    FlexPWM_Type * base = g_flexpwmBase[instance];

    base->FCTRL &= ~FlexPWM_FCTRL_FIE(faultInputMask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_ClearFaultFlags
 * Description   : Clears fault flags corresponding to the bits enabled in the faultInputMask input parameter.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to clear multiple fault flags.
 *
 * Implements    : FLEXPWM_DRV_ClearFaultFlags_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_ClearFaultFlags(const uint32_t instance,
                                 const uint8_t faultInputMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT((faultInputMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    uint16_t tmp = 0U;

    tmp = base->FSTS & (~FlexPWM_FSTS_FFLAG_MASK);
    base->FSTS = tmp | FlexPWM_FSTS_FFLAG(faultInputMask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_GetFaultFlags
 * Description   : Gets fault flags corresponding to the bits enabled in the faultInputMask input parameter.
 * The faultInputMask input parameter can be set using FLEXPWM_FAULT_INPUT_ defines. Bitwise OR the macro defines
 * to get multiple fault flag.
 *
 * Implements    : FLEXPWM_DRV_GetFaultFlags_Activity
 *END**************************************************************************/
uint32_t FLEXPWM_DRV_GetFaultFlags(const uint32_t instance,
                                   const uint8_t faultInputMask)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);
    DEV_ASSERT((faultInputMask & ~FLEXPWM_FAULT_INPUT_ALL) == 0U);

    const FlexPWM_Type * base = g_flexpwmBase[instance];

    return (uint32_t)(base->FSTS & (uint16_t)faultInputMask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FLEXPWM_DRV_SimulateFault
 * Description   : Simulates a fault condition. Calling this function causes a simulated fault to be
 * sent into all of the fault filters.
 *
 * Implements    : FLEXPWM_DRV_SimulateFault_Activity
 *END**************************************************************************/
void FLEXPWM_DRV_SimulateFault(const uint32_t instance, const bool faultState)
{
    DEV_ASSERT(instance < FlexPWM_INSTANCE_COUNT);

    FlexPWM_Type * base = g_flexpwmBase[instance];
    uint16_t tmp = 0U;

    tmp = base->FSTS & (~(uint16_t)FlexPWM_FSTS_FTEST_MASK);

    base->FSTS = tmp | (uint16_t)FlexPWM_FSTS_FTEST((faultState == true) ? (uint16_t)1U : (uint16_t)0U);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
