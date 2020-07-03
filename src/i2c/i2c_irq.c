/*
 * Copyright 2017 NXP
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

#include "device_registers.h"
#include "i2c_hw_access.h"

/*!
 * @i2c_irq.c
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * These are weak symbols defined in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External could be made static.
 * Function is defined for usage by application code.
 */


/*******************************************************************************
 * Code
 ******************************************************************************/

#if (I2C_INSTANCE_COUNT > 0u)

#if defined(I2C_ON_S32Rx7x_PLATFORM)

void I2C1_IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(0);
}

/* Implementation of I2C1 module handler */
void I2C2_IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(1);
}

#else

#if defined(I2C_ON_S32V23x_PLATFORM)

void I2C0_IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(0);
}

void I2C1_IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(1);
}

void I2C2_IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(2);
}

#else

void I2C_DRV_Module0IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(0);
}

/* Implementation of I2C1 module handler */
void I2C_DRV_Module1IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(1);
}

/* Implementation of I2C2 module handler */
void I2C_DRV_Module2IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(2);
}

/* Implementation of I2C3 module handler */
void I2C_DRV_Module3IRQHandler(void)
{
    I2C_DRV_ModuleIRQHandler(3);
}

#endif

#endif

#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
