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
 * @file edma_irq.c
 * @page misra_violations MISRA-C:2012 violations
 * 
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable.
 * The code is not dynamically linked. An absolute stack address is obtained
 * when taking the address of the near auto variable. A source of error in
 * writing dynamic code is that the stack segment may be different from the data
 * segment.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * These are symbols weak symbols defined in platform startup files (.s).
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, external could be made static.
 * The functions are called by the interrupt controller when the appropriate event
 * occurs.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * This is required for initializing pointers to the module's memory map, which 
 * is located at a fixed address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */

#include "edma_irq.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
 
/*******************************************************************************
 * Prototypes
 ******************************************************************************/   
/**************************************/
/* DMA Channel Ored Interrupt Vectors */
/**************************************/
/* DMA0 has groups of 8 ored channels */
#ifdef FEATURE_DMA0_CH0_CH7_ORED_IRQ_LINES
void DMA0_Ch0_Ch7_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH8_CH15_ORED_IRQ_LINES
void DMA0_Ch8_Ch15_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH16_CH23_ORED_IRQ_LINES
void DMA0_Ch16_Ch23_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH24_CH31_ORED_IRQ_LINES
void DMA0_Ch24_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH32_CH39_ORED_IRQ_LINES
void DMA0_Ch32_Ch39_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH40_CH47_ORED_IRQ_LINES
void DMA0_Ch40_Ch47_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH48_CH55_ORED_IRQ_LINES
void DMA0_Ch48_Ch55_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH56_CH63_ORED_IRQ_LINES
void DMA0_Ch56_Ch63_IRQHandler(void);
#endif
/* DMA1 has groups of 8 ored channels */
#ifdef FEATURE_DMA1_CH0_CH7_ORED_IRQ_LINES
void DMA1_Ch0_Ch7_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH8_CH15_ORED_IRQ_LINES
void DMA1_Ch8_Ch15_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH16_CH23_ORED_IRQ_LINES
void DMA1_Ch16_Ch23_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH24_CH31_ORED_IRQ_LINES
void DMA1_Ch24_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH32_CH39_ORED_IRQ_LINES
void DMA1_Ch32_Ch39_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH40_CH47_ORED_IRQ_LINES
void DMA1_Ch40_Ch47_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH48_CH55_ORED_IRQ_LINES
void DMA1_Ch48_Ch55_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH56_CH63_ORED_IRQ_LINES
void DMA1_Ch56_Ch63_IRQHandler(void);
#endif
/***************************************/
/* DMA0 has groups of 16 ored channels */
#ifdef FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES
void DMA0_Ch0_Ch15_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES
void DMA0_Ch16_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH32_CH47_ORED_IRQ_LINES
void DMA0_Ch32_Ch47_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH48_CH63_ORED_IRQ_LINES
void DMA0_Ch48_Ch63_IRQHandler(void);
#endif
/* DMA1 has groups of 16 ored channels */
#ifdef FEATURE_DMA1_CH0_CH15_ORED_IRQ_LINES
void DMA1_Ch0_Ch15_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH16_CH31_ORED_IRQ_LINES
void DMA1_Ch16_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH32_CH47_ORED_IRQ_LINES
void DMA1_Ch32_Ch47_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH48_CH63_ORED_IRQ_LINES
void DMA1_Ch48_Ch63_IRQHandler(void);
#endif
/***************************************/
/* DMA0 has groups of 32 ored channels */
#ifdef FEATURE_DMA0_CH0_CH31_ORED_IRQ_LINES
void DMA0_Ch0_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA0_CH32_CH63_ORED_IRQ_LINES
void DMA0_Ch32_Ch63_IRQHandler(void);
#endif
/* DMA1 has groups of 32 ored channels */
#ifdef FEATURE_DMA1_CH0_CH31_ORED_IRQ_LINES
void DMA1_Ch0_Ch31_IRQHandler(void);
#endif
#ifdef FEATURE_DMA1_CH32_CH63_ORED_IRQ_LINES
void DMA1_Ch32_Ch63_IRQHandler(void);
#endif


/*******************************************/
/* eDMA Channel Separate Interrupt Vectors */
/*******************************************/ 
#ifdef FEATURE_DMA_SEPARATE_IRQ_LINES_PER_CHN
/** DMA0 has separate IRQ Lines for channels */
    #ifndef FEATURE_DMA0_CH0_CH31_ORED_IRQ_LINES
        #ifndef FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES
            #ifndef FEATURE_DMA0_CH0_CH7_ORED_IRQ_LINES
                void DMA0_Ch0_IRQHandler(void);
                void DMA0_Ch1_IRQHandler(void);
                void DMA0_Ch2_IRQHandler(void);
                void DMA0_Ch3_IRQHandler(void);
                #if (FEATURE_DMA_CHANNELS > 4U)
                void DMA0_Ch4_IRQHandler(void);
                void DMA0_Ch5_IRQHandler(void);
                void DMA0_Ch6_IRQHandler(void);
                void DMA0_Ch7_IRQHandler(void);
                #endif
            #endif
            #if (FEATURE_DMA_CHANNELS > 8U)
                #ifndef FEATURE_DMA0_CH8_CH15_ORED_IRQ_LINES                   
                void DMA0_Ch8_IRQHandler(void);
                void DMA0_Ch9_IRQHandler(void);
                void DMA0_Ch10_IRQHandler(void);
                void DMA0_Ch11_IRQHandler(void);
                void DMA0_Ch12_IRQHandler(void);
                void DMA0_Ch13_IRQHandler(void);
                void DMA0_Ch14_IRQHandler(void);
                void DMA0_Ch15_IRQHandler(void);
                #endif
            #endif
        #endif
        #if (FEATURE_DMA_CHANNELS > 16U)
            #ifndef FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH16_CH23_ORED_IRQ_LINES           
                void DMA0_Ch16_IRQHandler(void);
                void DMA0_Ch17_IRQHandler(void);
                void DMA0_Ch18_IRQHandler(void);
                void DMA0_Ch19_IRQHandler(void);
                void DMA0_Ch20_IRQHandler(void);
                void DMA0_Ch21_IRQHandler(void);
                void DMA0_Ch22_IRQHandler(void);
                void DMA0_Ch23_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA0_CH24_CH31_ORED_IRQ_LINES
                void DMA0_Ch24_IRQHandler(void);
                void DMA0_Ch25_IRQHandler(void);
                void DMA0_Ch26_IRQHandler(void);
                void DMA0_Ch27_IRQHandler(void);
                void DMA0_Ch28_IRQHandler(void);
                void DMA0_Ch29_IRQHandler(void);
                void DMA0_Ch30_IRQHandler(void);
                void DMA0_Ch31_IRQHandler(void);
                #endif
            #endif
        #endif
    #endif
    #if (FEATURE_DMA_CHANNELS > 32U)
        #ifndef FEATURE_DMA0_CH32_CH63_ORED_IRQ_LINES
            #ifndef FEATURE_DMA0_CH32_CH47_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH32_CH39_ORED_IRQ_LINES
                void DMA0_Ch32_IRQHandler(void);
                void DMA0_Ch33_IRQHandler(void);
                void DMA0_Ch34_IRQHandler(void);
                void DMA0_Ch35_IRQHandler(void);
                void DMA0_Ch36_IRQHandler(void);
                void DMA0_Ch37_IRQHandler(void);
                void DMA0_Ch38_IRQHandler(void);
                void DMA0_Ch39_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA0_CH40_CH47_ORED_IRQ_LINES
                void DMA0_Ch40_IRQHandler(void);
                void DMA0_Ch41_IRQHandler(void);
                void DMA0_Ch42_IRQHandler(void);
                void DMA0_Ch43_IRQHandler(void);
                void DMA0_Ch44_IRQHandler(void);
                void DMA0_Ch45_IRQHandler(void);
                void DMA0_Ch46_IRQHandler(void);
                void DMA0_Ch47_IRQHandler(void);
                #endif
            #endif
            #ifndef FEATURE_DMA0_CH48_CH63_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH48_CH55_ORED_IRQ_LINES
                void DMA0_Ch48_IRQHandler(void);
                void DMA0_Ch49_IRQHandler(void);
                void DMA0_Ch50_IRQHandler(void);
                void DMA0_Ch51_IRQHandler(void);
                void DMA0_Ch52_IRQHandler(void);
                void DMA0_Ch53_IRQHandler(void);
                void DMA0_Ch54_IRQHandler(void);
                void DMA0_Ch55_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA0_CH56_CH63_ORED_IRQ_LINES
                void DMA0_Ch56_IRQHandler(void);
                void DMA0_Ch57_IRQHandler(void);
                void DMA0_Ch58_IRQHandler(void);
                void DMA0_Ch59_IRQHandler(void);
                void DMA0_Ch60_IRQHandler(void);
                void DMA0_Ch61_IRQHandler(void);
                void DMA0_Ch62_IRQHandler(void);
                void DMA0_Ch63_IRQHandler(void); 
                #endif
            #endif 
        #endif          
    #endif
/** DMA1 has separate IRQ Lines for channels */    
#if (DMA_INSTANCE_COUNT > 1U)
    #ifndef FEATURE_DMA1_CH0_CH31_ORED_IRQ_LINES
        #ifndef FEATURE_DMA1_CH0_CH15_ORED_IRQ_LINES
            #ifndef FEATURE_DMA1_CH0_CH7_ORED_IRQ_LINES
                void DMA1_Ch0_IRQHandler(void);
                void DMA1_Ch1_IRQHandler(void);
                void DMA1_Ch2_IRQHandler(void);
                void DMA1_Ch3_IRQHandler(void);
                #if (FEATURE_DMA_CHANNELS > 4U)
                void DMA1_Ch4_IRQHandler(void);
                void DMA1_Ch5_IRQHandler(void);
                void DMA1_Ch6_IRQHandler(void);
                void DMA1_Ch7_IRQHandler(void);
                #endif
            #endif
            #if (FEATURE_DMA_CHANNELS > 8U)
                #ifndef FEATURE_DMA1_CH8_CH15_ORED_IRQ_LINES
                void DMA1_Ch8_IRQHandler(void);
                void DMA1_Ch9_IRQHandler(void);
                void DMA1_Ch10_IRQHandler(void);
                void DMA1_Ch11_IRQHandler(void);
                void DMA1_Ch12_IRQHandler(void);
                void DMA1_Ch13_IRQHandler(void);
                void DMA1_Ch14_IRQHandler(void);
                void DMA1_Ch15_IRQHandler(void);
                #endif
            #endif
        #endif
        #if (FEATURE_DMA_CHANNELS > 16U)
            #ifndef FEATURE_DMA1_CH16_CH31_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH16_CH23_ORED_IRQ_LINES            
                void DMA1_Ch16_IRQHandler(void);
                void DMA1_Ch17_IRQHandler(void);
                void DMA1_Ch18_IRQHandler(void);
                void DMA1_Ch19_IRQHandler(void);
                void DMA1_Ch20_IRQHandler(void);
                void DMA1_Ch21_IRQHandler(void);
                void DMA1_Ch22_IRQHandler(void);
                void DMA1_Ch23_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA1_CH24_CH31_ORED_IRQ_LINES   
                void DMA1_Ch24_IRQHandler(void);
                void DMA1_Ch25_IRQHandler(void);
                void DMA1_Ch26_IRQHandler(void);
                void DMA1_Ch27_IRQHandler(void);
                void DMA1_Ch28_IRQHandler(void);
                void DMA1_Ch29_IRQHandler(void);
                void DMA1_Ch30_IRQHandler(void);
                void DMA1_Ch31_IRQHandler(void);
                #endif
            #endif
        #endif
    #endif
    #if (FEATURE_DMA_CHANNELS > 32U)
        #ifndef FEATURE_DMA1_CH32_CH63_ORED_IRQ_LINES
            #ifndef FEATURE_DMA1_CH32_CH47_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH32_CH39_ORED_IRQ_LINES
                void DMA1_Ch32_IRQHandler(void);
                void DMA1_Ch33_IRQHandler(void);
                void DMA1_Ch34_IRQHandler(void);
                void DMA1_Ch35_IRQHandler(void);
                void DMA1_Ch36_IRQHandler(void);
                void DMA1_Ch37_IRQHandler(void);
                void DMA1_Ch38_IRQHandler(void);
                void DMA1_Ch39_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA1_CH40_CH47_ORED_IRQ_LINES
                void DMA1_Ch40_IRQHandler(void);
                void DMA1_Ch41_IRQHandler(void);
                void DMA1_Ch42_IRQHandler(void);
                void DMA1_Ch43_IRQHandler(void);
                void DMA1_Ch44_IRQHandler(void);
                void DMA1_Ch45_IRQHandler(void);
                void DMA1_Ch46_IRQHandler(void);
                void DMA1_Ch47_IRQHandler(void);
                #endif
            #endif
            #ifndef FEATURE_DMA1_CH48_CH63_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH48_CH55_ORED_IRQ_LINES
                void DMA1_Ch48_IRQHandler(void);
                void DMA1_Ch49_IRQHandler(void);
                void DMA1_Ch50_IRQHandler(void);
                void DMA1_Ch51_IRQHandler(void);
                void DMA1_Ch52_IRQHandler(void);
                void DMA1_Ch53_IRQHandler(void);
                void DMA1_Ch54_IRQHandler(void);
                void DMA1_Ch55_IRQHandler(void);
                #endif
                #ifndef FEATURE_DMA1_CH56_CH63_ORED_IRQ_LINES
                void DMA1_Ch56_IRQHandler(void);
                void DMA1_Ch57_IRQHandler(void);
                void DMA1_Ch58_IRQHandler(void);
                void DMA1_Ch59_IRQHandler(void);
                void DMA1_Ch60_IRQHandler(void);
                void DMA1_Ch61_IRQHandler(void);
                void DMA1_Ch62_IRQHandler(void);
                void DMA1_Ch63_IRQHandler(void);
                #endif  
            #endif
        #endif    
    #endif
#endif
#if (DMA_INSTANCE_COUNT > 2U)
#error "DMA IRQ Handler case is not covered!"
#endif
#endif /* FEATURE_DMA_SEPARATE_IRQ_LINES_PER_CHN */


/********************************/
/* eDMA Error Interrupt Vectors */
/********************************/
#ifdef FEATURE_DMA_HAS_ERROR_IRQ
    #if (DMA_INSTANCE_COUNT == 1U)
        #if (FEATURE_DMA_ERROR_INTERRUPT_LINES == 1U)
            #if (FEATURE_DMA_CHANNELS == 4U)
                #define DMA0_CH0_CH3_ERR_IRQ
                void DMA0_Ch0_Ch3_Error_IRQHandler(void);
            #elif (FEATURE_DMA_CHANNELS == 16U)
                #define DMA0_CH0_CH15_ERR_IRQ
                void DMA0_Ch0_Ch15_Error_IRQHandler(void);
            #elif (FEATURE_DMA_CHANNELS == 32U)
                #define DMA0_CH0_CH31_ERR_IRQ
                void DMA0_Ch0_Ch31_Error_IRQHandler(void);
            #elif (FEATURE_DMA_CHANNELS == 64U)
                #define DMA0_CH0_CH63_ERR_IRQ
                void DMA0_Ch0_Ch63_Error_IRQHandler(void);                
            #else
                #error "DMA IRQ Handler case is not covered!"
            #endif
        #else
            #error "DMA IRQ Handler case is not covered!"
        #endif
    #elif (DMA_INSTANCE_COUNT == 2U)
        #if (FEATURE_DMA_ERROR_INTERRUPT_LINES == 1U)
            #if (FEATURE_DMA_CHANNELS == 16U)
                #define DMA0_CH0_CH15_ERR_IRQ
                #define DMA1_CH0_CH15_ERR_IRQ
                void DMA0_Ch0_Ch15_Error_IRQHandler(void); 
                void DMA1_Ch0_Ch15_Error_IRQHandler(void); 
            #elif (FEATURE_DMA_CHANNELS == 32U)
                #define DMA0_CH0_CH31_ERR_IRQ
                #define DMA1_CH0_CH31_ERR_IRQ                
                void DMA0_Ch0_Ch31_Error_IRQHandler(void); 
                void DMA1_Ch0_Ch31_Error_IRQHandler(void); 
            #elif (FEATURE_DMA_CHANNELS == 64U)
                #define DMA0_CH0_CH63_ERR_IRQ
                #define DMA1_CH0_CH63_ERR_IRQ                    
                void DMA0_Ch0_Ch63_Error_IRQHandler(void); 
                void DMA1_Ch0_Ch63_Error_IRQHandler(void); 
            #else
                #error "DMA IRQ Handler case is not covered!"
            #endif 
        #elif (FEATURE_DMA_ERROR_INTERRUPT_LINES == 2U)
            #if (FEATURE_DMA_CHANNELS == 32U)
                #define DMA0_CH0_CH15_ERR_IRQ
                #define DMA0_CH16_CH31_ERR_IRQ
                #define DMA1_CH0_CH15_ERR_IRQ
                #define DMA1_CH16_CH31_ERR_IRQ
                void DMA0_Ch0_Ch15_Error_IRQHandler(void);
                void DMA0_Ch16_Ch31_Error_IRQHandler(void);
                void DMA1_Ch0_Ch15_Error_IRQHandler(void);
                void DMA1_Ch16_Ch31_Error_IRQHandler(void);                
            #elif (FEATURE_DMA_CHANNELS == 64U)
                #define DMA0_CH0_CH31_ERR_IRQ
                #define DMA0_CH32_CH63_ERR_IRQ
                #define DMA1_CH0_CH31_ERR_IRQ
                #define DMA1_CH32_CH63_ERR_IRQ                
                void DMA0_Ch0_Ch31_Error_IRQHandler(void);
                void DMA0_Ch32_Ch63_Error_IRQHandler(void);
                void DMA1_Ch0_Ch31_Error_IRQHandler(void);
                void DMA1_Ch32_Ch63_Error_IRQHandler(void);                
            #else
                #error "DMA IRQ Handler case is not covered!"
            #endif
        #else
            #error "DMA IRQ Handler case is not covered!"
        #endif
    #endif
#endif /* FEATURE_DMA_HAS_ERROR_IRQ */

/*******************************************************************************
 * Code
 ******************************************************************************/
#ifdef FEATURE_DMA_SEPARATE_IRQ_LINES_PER_CHN
/**********************************/
/* DMA0 Channel Interrupt Vectors */
/**********************************/
#if (DMA_INSTANCE_COUNT > 0U)
    #ifndef FEATURE_DMA0_CH0_CH31_ORED_IRQ_LINES
        #ifndef FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES
            #ifndef FEATURE_DMA0_CH0_CH7_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch0_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(0U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch1_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(1U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch2_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(2U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch3_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(3U);
                }
                #if (FEATURE_DMA_CHANNELS > 4U)
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch4_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(4U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch5_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(5U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch6_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(6U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch7_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(7U);
                }
                #endif /* (FEATURE_DMA_CHANNELS > 4U) */
            #endif
            #if (FEATURE_DMA_CHANNELS > 8U)
                #ifndef FEATURE_DMA0_CH8_CH15_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch8_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(8U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch9_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(9U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch10_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(10U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch11_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(11U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch12_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(12U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch13_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(13U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch14_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(14U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch15_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(15U);
                }
                #endif 
            #endif /* (FEATURE_DMA_CHANNELS > 8U) */
        #endif
        #if (FEATURE_DMA_CHANNELS > 16U)
            #ifndef FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH16_CH23_ORED_IRQ_LINES
                void DMA0_Ch16_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(16U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch17_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(17U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch18_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(18U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch19_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(19U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch20_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(20U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch21_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(21U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch22_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(22U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch23_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(23U);
                }
                #endif
                #ifndef FEATURE_DMA0_CH24_CH31_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch24_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(24U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch25_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(25U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch26_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(26U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch27_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(27U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch28_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(28U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch29_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(29U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch30_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(30U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch31_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(31U);
                }
                #endif 
            #endif
        #endif /* (FEATURE_DMA_CHANNELS > 32U) */
    #endif 
    #if (FEATURE_DMA_CHANNELS > 32U)
        #ifndef FEATURE_DMA0_CH32_CH63_ORED_IRQ_LINES
            #ifndef FEATURE_DMA0_CH32_CH47_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH32_CH39_ORED_IRQ_LINES            
                void DMA0_Ch32_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(32U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch33_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(33U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch34_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(34U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch35_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(35U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch36_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(36U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch37_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(37U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch38_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(38U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch39_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(39U);
                }
                #endif
                #ifndef FEATURE_DMA0_CH40_CH47_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch40_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(40U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch41_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(41U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch42_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(42U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch43_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(43U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch44_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(44U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch45_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(45U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch46_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(46U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch47_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(47U);
                }
                #endif
            #endif
            #ifndef FEATURE_DMA0_CH48_CH63_ORED_IRQ_LINES
                #ifndef FEATURE_DMA0_CH48_CH55_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch48_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(48U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch49_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(49U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch50_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(50U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch51_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(51U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch52_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(52U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch53_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(53U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch54_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(54U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch55_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(55U);
                }
                #endif
                #ifndef FEATURE_DMA0_CH56_CH63_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch56_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(56U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch57_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(57U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch58_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(58U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch59_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(59U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch60_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(60U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch61_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(61U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch62_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(62U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA0_Ch63_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(63U);
                }           
                #endif 
            #endif
        #endif           
    #endif /* (FEATURE_DMA_CHANNELS > 32U) */
#endif /* #if (DMA_INSTANCE_COUNT > 0U) */    
/**********************************/
/* DMA1 Channel Interrupt Vectors */
/**********************************/
#if (DMA_INSTANCE_COUNT > 1U)
    #ifndef FEATURE_DMA1_CH0_CH31_ORED_IRQ_LINES
        #ifndef FEATURE_DMA1_CH0_CH15_ORED_IRQ_LINES
            #ifndef FEATURE_DMA1_CH0_CH7_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch0_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(64U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch1_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(65U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch2_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(66U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch3_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(67U);
                }
                #if (FEATURE_DMA_CHANNELS > 4U)
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch4_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(68U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch5_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(69U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch6_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(70U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch7_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(71U);
                }
                #endif
            #endif
            #if (FEATURE_DMA_CHANNELS > 8U)
                #ifndef FEATURE_DMA1_CH8_CH15_ORED_IRQ_LINES            
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch8_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(72U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch9_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(73U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch10_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(74U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch11_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(75U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch12_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(76U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch13_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(77U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch14_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(78U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch15_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(79U);
                }
                #endif
            #endif
        #endif
        #if (FEATURE_DMA_CHANNELS > 16U)
            #ifndef FEATURE_DMA1_CH16_CH31_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH16_CH23_ORED_IRQ_LINES           
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch16_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(80U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch17_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(81U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch18_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(82U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch19_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(83U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch20_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(84U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch21_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(85U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch22_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(86U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch23_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(87U);
                }
                #endif
                #ifndef FEATURE_DMA1_CH24_CH31_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch24_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(88U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch25_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(89U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch26_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(90U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch27_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(91U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch28_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(92U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch29_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(93U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch30_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(94U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch31_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(95U);
                }
                #endif
            #endif
        #endif
    #endif
    #if (FEATURE_DMA_CHANNELS > 32U)
        #ifndef FEATURE_DMA1_CH32_CH63_ORED_IRQ_LINES
            #ifndef FEATURE_DMA1_CH32_CH47_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH32_CH39_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch32_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(96U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch33_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(97U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch34_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(98U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch35_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(99U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch36_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(100U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch37_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(101U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch38_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(102U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch39_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(103U);
                }
                #endif
                #ifndef FEATURE_DMA1_CH40_CH47_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch40_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(104U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch41_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(105U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch42_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(106U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch43_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(107U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch44_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(108U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch45_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(109U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch46_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(110U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch47_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(111U);
                }
                #endif
            #endif
            #ifndef FEATURE_DMA1_CH48_CH63_ORED_IRQ_LINES
                #ifndef FEATURE_DMA1_CH48_CH55_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch48_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(112U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch49_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(113U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch50_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(114U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch51_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(115U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch52_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(116U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch53_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(117U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch54_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(118U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch55_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(119U);
                }
                #endif
                #ifndef FEATURE_DMA1_CH56_CH63_ORED_IRQ_LINES
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch56_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(120U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch57_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(121U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch58_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(122U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch59_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(123U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch60_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(124U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch61_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(125U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch62_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(126U);
                }
                /*! @brief DMA IRQ handler with the same name in the startup code*/
                void DMA1_Ch63_IRQHandler(void)
                {
                    EDMA_DRV_IRQHandler(127U);
                }   
                #endif
            #endif
        #endif 
    #endif           
#endif /* #if (DMA_INSTANCE_COUNT > 1U) */
#if (DMA_INSTANCE_COUNT > 2U)
    #error "DMA IRQ Handler case is not covered!"
#endif
#endif /* FEATURE_DMA_SEPARATE_IRQ_LINES_PER_CHN */


#ifdef FEATURE_DMA_HWV2
/*************************************************/
/* HWV2: DMA0 16 Channels Ored Interrupt Vectors */
/*************************************************/
#ifdef FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES
    /*! @brief DMA0_Ch0_Ch15_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch15_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint32_t flags = (uint32_t)edmaRegBase->INT;
        uint8_t virtualChannel = 0U;
        flags = flags & 0x0000FFFFU;
        /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES */
#ifdef FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES
    /*! @brief DMA0_Ch16_Ch31_IRQHandler with the same name in the startup code*/
    void DMA0_Ch16_Ch31_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint32_t flags = (uint32_t)edmaRegBase->INT;
        uint8_t virtualChannel = 16U;
        flags = (flags >> 16U) & 0x0000FFFFU;
        /* Check all the flags from 16 to 31 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES */
/************************************************/
/* HWV2: DMA1 8 Channels Ored Interrupt Vectors */
/************************************************/
#ifdef FEATURE_DMA1_CH32_CH39_ORED_IRQ_LINES
    /*! @brief DMA1_Ch32_Ch39_IRQHandler with the same name in the startup code*/
    void DMA1_Ch32_Ch39_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint32_t flags = (uint32_t)edmaRegBase->INTH;
        uint8_t virtualChannel = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 32U);
        flags = flags & 0x000000FFU;
        /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA1_CH32_CH39_ORED_IRQ_LINES */
#ifdef FEATURE_DMA1_CH40_CH47_ORED_IRQ_LINES
    /*! @brief DMA1_Ch40_Ch47_IRQHandler with the same name in the startup code*/
    void DMA1_Ch40_Ch47_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint32_t flags = (uint32_t)edmaRegBase->INTH;
        uint8_t virtualChannel = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 40U);
        flags = (flags >> 8U) & 0x000000FFU;
        /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA1_CH40_CH47_ORED_IRQ_LINES */
#ifdef FEATURE_DMA1_CH48_CH55_ORED_IRQ_LINES
    /*! @brief DMA1_Ch48_Ch55_IRQHandler with the same name in the startup code*/
    void DMA1_Ch48_Ch55_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint32_t flags = (uint32_t)edmaRegBase->INTH;
        uint8_t virtualChannel = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 48U);
        flags = (flags >> 16U) & 0x000000FFU;
        /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA1_CH48_CH55_ORED_IRQ_LINES */
#ifdef FEATURE_DMA1_CH56_CH63_ORED_IRQ_LINES
    /*! @brief DMA1_Ch56_Ch63_IRQHandler with the same name in the startup code*/
    void DMA1_Ch56_Ch63_IRQHandler(void)
    {
        /* Read the status flags register */
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint32_t flags = (uint32_t)edmaRegBase->INTH;
        uint8_t virtualChannel = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 56U);
        flags = (flags >> 24U) & 0x000000FFU;
        /* Check all the flags from 0 to 15 and call the handler for the appropriate channel */
        while (flags > 0U)
        {
            if ((flags & 1U) > 0U)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            virtualChannel++;
            flags >>= 1U;
        }
    }
#endif /* FEATURE_DMA1_CH56_CH63_ORED_IRQ_LINES */


#ifdef FEATURE_DMA_HAS_ERROR_IRQ
/**************************************/
/* HWV2: DMA0 Error Interrupt Vectors */
/**************************************/
#ifdef DMA0_CH0_CH3_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch3_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch3_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);      
        for (virtualChannel  = 0U;
             virtualChannel <= 3U;
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA0_CH0_CH3_ERR_IRQ */
#ifdef DMA0_CH0_CH15_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch15_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch15_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);
        for (virtualChannel  = 0U;
             virtualChannel <= 15U;
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA0_CH0_CH15_ERR_IRQ */
#ifdef DMA0_CH16_CH31_ERR_IRQ
    /*! @brief DMA0_Ch16_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch16_Ch31_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);      
        for (virtualChannel  = 16U;
             virtualChannel <= 31U;
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA0_CH16_CH31_ERR_IRQ */   
#ifdef DMA0_CH0_CH31_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch31_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);       
        for (virtualChannel  = 0U;
             virtualChannel <= 31U;
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA0_CH0_CH31_ERR_IRQ */  
#ifdef DMA0_CH32_CH63_ERR_IRQ
    /*! @brief DMA0_Ch32_Ch63_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch32_Ch63_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);        
        for (virtualChannel  = 32U;
             virtualChannel <= 63U;
             virtualChannel++)
        {
            if((errFlag.errh & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errh = errFlag.errh >> 1U;
        }
    }
#endif /* DMA0_CH32_CH63_ERR_IRQ */
#ifdef DMA0_CH0_CH63_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch63_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch63_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);        
        for (virtualChannel  = 0U;
             virtualChannel <= 31U;
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
        for (virtualChannel  = 32U;
             virtualChannel <= 63U;
             virtualChannel++)
        {
            if((errFlag.errh & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errh = errFlag.errh >> 1U;
        }        
    }
#endif /* DMA0_CH0_CH63_ERR_IRQ */
/**************************************/
/* HWV2: DMA1 Error Interrupt Vectors */
/**************************************/
#ifdef DMA1_CH0_CH15_ERR_IRQ
    /*! @brief DMA1_Ch0_Ch15_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch15_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);       
        for (virtualChannel  = (uint8_t)(FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U));
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 15U);
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA1_CH0_CH15_ERR_IRQ */
#ifdef DMA1_CH16_CH31_ERR_IRQ
    /*! @brief DMA1_Ch16_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch16_Ch31_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);       
        for (virtualChannel  = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 16U);
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 31U);
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA1_CH16_CH31_ERR_IRQ */
#ifdef DMA1_CH0_CH31_ERR_IRQ
    /*! @brief DMA1_Ch0_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch31_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);
        for (virtualChannel  = (uint8_t)(FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U));
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 31U);
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
    }
#endif /* DMA1_CH0_CH31_ERR_IRQ */
#ifdef DMA1_CH32_CH63_ERR_IRQ
    /*! @brief DMA1_Ch32_Ch63_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch32_Ch63_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);
        for (virtualChannel  = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 32U);
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 63U);
             virtualChannel++)
        {
            if((errFlag.errh & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errh = errFlag.errh >> 1U;
        }
    }
#endif /* DMA1_CH32_CH63_ERR_IRQ */
#ifdef DMA1_CH0_CH63_ERR_IRQ
    /*! @brief DMA1_Ch0_Ch63_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch63_Error_IRQHandler(void)
    {    
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t virtualChannel = 0U;
        edma_error_register_t errFlag;
        EDMA_GetErrorIntStatusFlag(edmaRegBase, &errFlag);
        for (virtualChannel  = (uint8_t)(FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U));
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 31U);
             virtualChannel++)
        {
            if((errFlag.errl & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errl = errFlag.errl >> 1U;
        }
        for (virtualChannel  = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 32U);
             virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 63U);
             virtualChannel++)
        {
            if((errFlag.errh & EDMA_ERR_LSB_MASK) != 0U)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            errFlag.errh = errFlag.errh >> 1U;
        }        
    }
#endif /* DMA1_CH0_CH63_ERR_IRQ */
#endif /* FEATURE_DMA_HAS_ERROR_IRQ */
#endif /* FEATURE_DMA_HWV2 */

#ifdef FEATURE_DMA_HWV3
/*********************************************/
/* HWV3: DMA0 Channel Ored Interrupt Vectors */
/*********************************************/
#ifdef FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES
    /*! @brief DMA0_Ch0_Ch15_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch15_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = 0U; 
            virtualChannel <= 15U; 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_INT & DMA_TCD_CH_INT_INT_MASK) == DMA_TCD_CH_INT_INT_MASK)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* FEATURE_DMA0_CH0_CH15_ORED_IRQ_LINES */
#ifdef FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES
    /*! @brief DMA0_Ch16_Ch31_IRQHandler with the same name in the startup code*/
    void DMA0_Ch16_Ch31_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t index = 16U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = 16U; 
            virtualChannel <= 31U; 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_INT & DMA_TCD_CH_INT_INT_MASK) == DMA_TCD_CH_INT_INT_MASK)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* FEATURE_DMA0_CH16_CH31_ORED_IRQ_LINES */
#ifdef FEATURE_DMA1_CH0_CH15_ORED_IRQ_LINES
    /*! @brief DMA1_Ch0_Ch15_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch15_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = (uint8_t)(FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)); 
            virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 15U); 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_INT & DMA_TCD_CH_INT_INT_MASK) == DMA_TCD_CH_INT_INT_MASK)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* FEATURE_DMA1_CH0_CH15_ORED_IRQ_LINES */
#ifdef FEATURE_DMA1_CH16_CH31_ORED_IRQ_LINES 
    /*! @brief DMA1_Ch16_Ch31_IRQHandler with the same name in the startup code*/
    void DMA1_Ch16_Ch31_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t index = 16U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 16U); 
            virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 31U); 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_INT & DMA_TCD_CH_INT_INT_MASK) == DMA_TCD_CH_INT_INT_MASK)
            {
                EDMA_DRV_IRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* FEATURE_DMA1_CH16_CH31_ORED_IRQ_LINES */


#ifdef FEATURE_DMA_HAS_ERROR_IRQ
/**************************************/
/* HWV3: DMA0 Error Interrupt Vectors */
/**************************************/
#ifdef DMA0_CH0_CH15_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch15_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch15_Error_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = 0U; 
            virtualChannel <= 15U; 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_ES & DMA_TCD_CH_ES_ERR_MASK) == DMA_TCD_CH_ES_ERR_MASK)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* DMA0_CH0_CH15_ERR_IRQ */
#ifdef DMA0_CH0_CH31_ERR_IRQ
    /*! @brief DMA0_Ch0_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA0_Ch0_Ch31_Error_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(0U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = 0U; 
            virtualChannel <= 31U; 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_ES & DMA_TCD_CH_ES_ERR_MASK) == DMA_TCD_CH_ES_ERR_MASK)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* DMA0_CH0_CH31_ERR_IRQ */
/**************************************/
/* HWV3: DMA1 Error Interrupt Vectors */
/**************************************/
#ifdef DMA1_CH0_CH15_ERR_IRQ
    /*! @brief DMA1_Ch0_Ch15_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch15_Error_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = 0U; 
            virtualChannel <= 15U; 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_ES & DMA_TCD_CH_ES_ERR_MASK) == DMA_TCD_CH_ES_ERR_MASK)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* DMA0_CH0_CH15_ERR_IRQ */
#ifdef DMA1_CH0_CH31_ERR_IRQ
    /*! @brief DMA1_Ch0_Ch31_Error_IRQHandler with the same name in the startup code*/
    void DMA1_Ch0_Ch31_Error_IRQHandler(void)
    {
        const DMA_Type * edmaRegBase = EDMA_DRV_GetDmaRegBaseAddr(1U);
        uint8_t index = 0U;
        uint8_t virtualChannel = 0U;
        /* Check all the flags and call the handler for the appropriate channel */
        for(virtualChannel  = (uint8_t)(FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)); 
            virtualChannel <= (uint8_t)((FEATURE_DMA_CHANNELS * (DMA_INSTANCE_COUNT - 1U)) + 31U); 
            virtualChannel++)
        {
            if ((BASE_TCD(edmaRegBase,index).CH_ES & DMA_TCD_CH_ES_ERR_MASK) == DMA_TCD_CH_ES_ERR_MASK)
            {
                EDMA_DRV_ErrorIRQHandler(virtualChannel);
            }
            index++;
        }
    }
#endif /* DMA1_CH0_CH31_ERR_IRQ */
#endif /* FEATURE_DMA_HAS_ERROR_IRQ */
#endif /* FEATURE_DMA_HWV3 */

/*******************************************************************************
 * EOF
 ******************************************************************************/
