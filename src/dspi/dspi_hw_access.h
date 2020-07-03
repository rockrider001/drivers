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
 
 /*!
 * @file dspi_hw_access.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
  * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined
 * This is required to allow the compatibility of the same source code with all device which 
 * integrates DSPI or SPI peripherals.
 *
* @section [global]
* Violates MISRA 2012 Advisory Rule 2.5, local macro not referenced
* The header defines macros for all registers, but not all of them are used in DSPI driver.
*
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
*
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially 
 * unsigned' to 'essentially Boolean'.
 * This is required by the conversion of a bit into a bool.
 */
 
#ifndef DSPI_HEADER_PRIVATE_H_
#define DSPI_HEADER_PRIVATE_H_

#include "device_registers.h"
#define DSPI_PRESCALER_NUM              (0x04u)
#define DSPI_SCALER_NUM                 (0xFu)
#define DSPI_EXTENDED_FRAME_SIZE_MASK   (0x10u)
#define DSPI_EXTENDED_FRAME_SIZE_SHIFT  (0x04u)

/* Due to some naming differences between MPC574X platforms is necessary to redefine registers naming*/
#if (defined(CPU_MPC5744P) || defined(CPU_MPC5743P) || defined(CPU_MPC5742P) || defined(CPU_MPC5741P) || defined(CPU_S32R274) || defined(CPU_S32R372) || defined(CPU_S32V232) || \
    defined(CPU_S32V234)   || defined(S32S247TV_SERIES))

#define DSPI_CTAR_COUNT                           SPI_CTAR_COUNT 
#define DSPI_Type                                 SPI_Type    
#define DSPI_MCR_HALT_MASK                        SPI_MCR_HALT_MASK                           
#define DSPI_MCR_HALT_SHIFT                       SPI_MCR_HALT_SHIFT                          
#define DSPI_MCR_HALT_WIDTH                       SPI_MCR_HALT_WIDTH                          
#define DSPI_MCR_HALT(x)                          SPI_MCR_HALT(x)                             
#define DSPI_MCR_SMPL_PT_MASK                     SPI_MCR_SMPL_PT_MASK                        
#define DSPI_MCR_SMPL_PT_SHIFT                    SPI_MCR_SMPL_PT_SHIFT                       
#define DSPI_MCR_SMPL_PT_WIDTH                    SPI_MCR_SMPL_PT_WIDTH                       
#define DSPI_MCR_SMPL_PT(x)                       SPI_MCR_SMPL_PT(x)                          
#define DSPI_MCR_CLR_RXF_MASK                     SPI_MCR_CLR_RXF_MASK                        
#define DSPI_MCR_CLR_RXF_SHIFT                    SPI_MCR_CLR_RXF_SHIFT                       
#define DSPI_MCR_CLR_RXF_WIDTH                    SPI_MCR_CLR_RXF_WIDTH                       
#define DSPI_MCR_CLR_RXF(x)                       SPI_MCR_CLR_RXF(x)                          
#define DSPI_MCR_CLR_TXF_MASK                     SPI_MCR_CLR_TXF_MASK                        
#define DSPI_MCR_CLR_TXF_SHIFT                    SPI_MCR_CLR_TXF_SHIFT                       
#define DSPI_MCR_CLR_TXF_WIDTH                    SPI_MCR_CLR_TXF_WIDTH                       
#define DSPI_MCR_CLR_TXF(x)                       SPI_MCR_CLR_TXF(x)                          
#define DSPI_MCR_DIS_RXF_MASK                     SPI_MCR_DIS_RXF_MASK                        
#define DSPI_MCR_DIS_RXF_SHIFT                    SPI_MCR_DIS_RXF_SHIFT                       
#define DSPI_MCR_DIS_RXF_WIDTH                    SPI_MCR_DIS_RXF_WIDTH                       
#define DSPI_MCR_DIS_RXF(x)                       SPI_MCR_DIS_RXF(x)                          
#define DSPI_MCR_DIS_TXF_MASK                     SPI_MCR_DIS_TXF_MASK                        
#define DSPI_MCR_DIS_TXF_SHIFT                    SPI_MCR_DIS_TXF_SHIFT                       
#define DSPI_MCR_DIS_TXF_WIDTH                    SPI_MCR_DIS_TXF_WIDTH                       
#define DSPI_MCR_DIS_TXF(x)                       SPI_MCR_DIS_TXF(x)                          
#define DSPI_MCR_MDIS_MASK                        SPI_MCR_MDIS_MASK                           
#define DSPI_MCR_MDIS_SHIFT                       SPI_MCR_MDIS_SHIFT                          
#define DSPI_MCR_MDIS_WIDTH                       SPI_MCR_MDIS_WIDTH                          
#define DSPI_MCR_MDIS(x)                          SPI_MCR_MDIS(x)                             
#define DSPI_MCR_PCSIS_MASK                       SPI_MCR_PCSIS_MASK                          
#define DSPI_MCR_PCSIS_SHIFT                      SPI_MCR_PCSIS_SHIFT                         
#define DSPI_MCR_PCSIS_WIDTH                      SPI_MCR_PCSIS_WIDTH                         
#define DSPI_MCR_PCSIS(x)                         SPI_MCR_PCSIS(x)                            
#define DSPI_MCR_ROOE_MASK                        SPI_MCR_ROOE_MASK                           
#define DSPI_MCR_ROOE_SHIFT                       SPI_MCR_ROOE_SHIFT                          
#define DSPI_MCR_ROOE_WIDTH                       SPI_MCR_ROOE_WIDTH                          
#define DSPI_MCR_ROOE(x)                          SPI_MCR_ROOE(x)
#define DSPI_MCR_PCSSE_MASK                       SPI_MCR_PCSSE_MASK                          
#define DSPI_MCR_PCSSE_SHIFT                      SPI_MCR_PCSSE_SHIFT                         
#define DSPI_MCR_PCSSE_WIDTH                      SPI_MCR_PCSSE_WIDTH                         
#define DSPI_MCR_PCSSE(x)                         SPI_MCR_PCSSE(x)  
#define DSPI_MCR_MTFE_MASK                        SPI_MCR_MTFE_MASK                           
#define DSPI_MCR_MTFE_SHIFT                       SPI_MCR_MTFE_SHIFT                          
#define DSPI_MCR_MTFE_WIDTH                       SPI_MCR_MTFE_WIDTH                          
#define DSPI_MCR_MTFE(x)                          SPI_MCR_MTFE(x)                             
#define DSPI_MCR_FRZ_MASK                         SPI_MCR_FRZ_MASK                            
#define DSPI_MCR_FRZ_SHIFT                        SPI_MCR_FRZ_SHIFT                           
#define DSPI_MCR_FRZ_WIDTH                        SPI_MCR_FRZ_WIDTH                           
#define DSPI_MCR_FRZ(x)                           SPI_MCR_FRZ(x)                              
#define DSPI_MCR_DCONF_MASK                       SPI_MCR_DCONF_MASK                          
#define DSPI_MCR_DCONF_SHIFT                      SPI_MCR_DCONF_SHIFT                         
#define DSPI_MCR_DCONF_WIDTH                      SPI_MCR_DCONF_WIDTH                         
#define DSPI_MCR_DCONF(x)                         SPI_MCR_DCONF(x)                            
#define DSPI_MCR_CONT_SCKE_MASK                   SPI_MCR_CONT_SCKE_MASK                      
#define DSPI_MCR_CONT_SCKE_SHIFT                  SPI_MCR_CONT_SCKE_SHIFT                     
#define DSPI_MCR_CONT_SCKE_WIDTH                  SPI_MCR_CONT_SCKE_WIDTH                     
#define DSPI_MCR_CONT_SCKE(x)                     SPI_MCR_CONT_SCKE(x)                        
#define DSPI_MCR_MSTR_MASK                        SPI_MCR_MSTR_MASK                           
#define DSPI_MCR_MSTR_SHIFT                       SPI_MCR_MSTR_SHIFT                          
#define DSPI_MCR_MSTR_WIDTH                       SPI_MCR_MSTR_WIDTH                          
#define DSPI_MCR_MSTR(x)                          SPI_MCR_MSTR(x)
#define DSPI_TCR_SPI_TCNT_MASK                    SPI_TCR_SPI_TCNT_MASK                       
#define DSPI_TCR_SPI_TCNT_SHIFT                   SPI_TCR_SPI_TCNT_SHIFT                      
#define DSPI_TCR_SPI_TCNT_WIDTH                   SPI_TCR_SPI_TCNT_WIDTH                      
#define DSPI_TCR_SPI_TCNT(x)                      SPI_TCR_SPI_TCNT(x)                                                   
#define DSPI_CTAR_BR_MASK                         SPI_CTAR_BR_MASK                            
#define DSPI_CTAR_BR_SHIFT                        SPI_CTAR_BR_SHIFT                           
#define DSPI_CTAR_BR_WIDTH                        SPI_CTAR_BR_WIDTH                           
#define DSPI_CTAR_BR(x)                           SPI_CTAR_BR(x)                              
#define DSPI_CTAR_DT_MASK                         SPI_CTAR_DT_MASK                            
#define DSPI_CTAR_DT_SHIFT                        SPI_CTAR_DT_SHIFT                           
#define DSPI_CTAR_DT_WIDTH                        SPI_CTAR_DT_WIDTH                           
#define DSPI_CTAR_DT(x)                           SPI_CTAR_DT(x)                              
#define DSPI_CTAR_ASC_MASK                        SPI_CTAR_ASC_MASK                           
#define DSPI_CTAR_ASC_SHIFT                       SPI_CTAR_ASC_SHIFT                          
#define DSPI_CTAR_ASC_WIDTH                       SPI_CTAR_ASC_WIDTH                          
#define DSPI_CTAR_ASC(x)                          SPI_CTAR_ASC(x)                             
#define DSPI_CTAR_CSSCK_MASK                      SPI_CTAR_CSSCK_MASK                         
#define DSPI_CTAR_CSSCK_SHIFT                     SPI_CTAR_CSSCK_SHIFT                        
#define DSPI_CTAR_CSSCK_WIDTH                     SPI_CTAR_CSSCK_WIDTH                        
#define DSPI_CTAR_CSSCK(x)                        SPI_CTAR_CSSCK(x)                           
#define DSPI_CTAR_PBR_MASK                        SPI_CTAR_PBR_MASK                           
#define DSPI_CTAR_PBR_SHIFT                       SPI_CTAR_PBR_SHIFT                          
#define DSPI_CTAR_PBR_WIDTH                       SPI_CTAR_PBR_WIDTH                          
#define DSPI_CTAR_PBR(x)                          SPI_CTAR_PBR(x)                             
#define DSPI_CTAR_PDT_MASK                        SPI_CTAR_PDT_MASK                           
#define DSPI_CTAR_PDT_SHIFT                       SPI_CTAR_PDT_SHIFT                          
#define DSPI_CTAR_PDT_WIDTH                       SPI_CTAR_PDT_WIDTH                          
#define DSPI_CTAR_PDT(x)                          SPI_CTAR_PDT(x)                             
#define DSPI_CTAR_PASC_MASK                       SPI_CTAR_PASC_MASK                          
#define DSPI_CTAR_PASC_SHIFT                      SPI_CTAR_PASC_SHIFT                         
#define DSPI_CTAR_PASC_WIDTH                      SPI_CTAR_PASC_WIDTH                         
#define DSPI_CTAR_PASC(x)                         SPI_CTAR_PASC(x)                            
#define DSPI_CTAR_PCSSCK_MASK                     SPI_CTAR_PCSSCK_MASK                        
#define DSPI_CTAR_PCSSCK_SHIFT                    SPI_CTAR_PCSSCK_SHIFT                       
#define DSPI_CTAR_PCSSCK_WIDTH                    SPI_CTAR_PCSSCK_WIDTH                       
#define DSPI_CTAR_PCSSCK(x)                       SPI_CTAR_PCSSCK(x)                          
#define DSPI_CTAR_LSBFE_MASK                      SPI_CTAR_LSBFE_MASK                         
#define DSPI_CTAR_LSBFE_SHIFT                     SPI_CTAR_LSBFE_SHIFT                        
#define DSPI_CTAR_LSBFE_WIDTH                     SPI_CTAR_LSBFE_WIDTH                        
#define DSPI_CTAR_LSBFE(x)                        SPI_CTAR_LSBFE(x)                           
#define DSPI_CTAR_CPHA_MASK                       SPI_CTAR_CPHA_MASK                          
#define DSPI_CTAR_CPHA_SHIFT                      SPI_CTAR_CPHA_SHIFT                         
#define DSPI_CTAR_CPHA_WIDTH                      SPI_CTAR_CPHA_WIDTH                         
#define DSPI_CTAR_CPHA(x)                         SPI_CTAR_CPHA(x)                            
#define DSPI_CTAR_CPOL_MASK                       SPI_CTAR_CPOL_MASK                          
#define DSPI_CTAR_CPOL_SHIFT                      SPI_CTAR_CPOL_SHIFT                         
#define DSPI_CTAR_CPOL_WIDTH                      SPI_CTAR_CPOL_WIDTH                         
#define DSPI_CTAR_CPOL(x)                         SPI_CTAR_CPOL(x)                            
#define DSPI_CTAR_FMSZ_MASK                       SPI_CTAR_FMSZ_MASK                          
#define DSPI_CTAR_FMSZ_SHIFT                      SPI_CTAR_FMSZ_SHIFT                         
#define DSPI_CTAR_FMSZ_WIDTH                      SPI_CTAR_FMSZ_WIDTH                         
#define DSPI_CTAR_FMSZ(x)                         SPI_CTAR_FMSZ(x)                            
#define DSPI_CTAR_DBR_MASK                        SPI_CTAR_DBR_MASK                           
#define DSPI_CTAR_DBR_SHIFT                       SPI_CTAR_DBR_SHIFT                          
#define DSPI_CTAR_DBR_WIDTH                       SPI_CTAR_DBR_WIDTH                          
#define DSPI_CTAR_DBR(x)                          SPI_CTAR_DBR(x)                                                    
#define DSPI_CTAR_SLAVE_CPHA_MASK                 SPI_CTAR_SLAVE_CPHA_MASK                    
#define DSPI_CTAR_SLAVE_CPHA_SHIFT                SPI_CTAR_SLAVE_CPHA_SHIFT                   
#define DSPI_CTAR_SLAVE_CPHA_WIDTH                SPI_CTAR_SLAVE_CPHA_WIDTH                   
#define DSPI_CTAR_SLAVE_CPHA(x)                   SPI_CTAR_SLAVE_CPHA(x)                      
#define DSPI_CTAR_SLAVE_CPOL_MASK                 SPI_CTAR_SLAVE_CPOL_MASK                    
#define DSPI_CTAR_SLAVE_CPOL_SHIFT                SPI_CTAR_SLAVE_CPOL_SHIFT                   
#define DSPI_CTAR_SLAVE_CPOL_WIDTH                SPI_CTAR_SLAVE_CPOL_WIDTH                   
#define DSPI_CTAR_SLAVE_CPOL(x)                   SPI_CTAR_SLAVE_CPOL(x)                      
#define DSPI_CTAR_SLAVE_FMSZ_MASK                 SPI_CTAR_SLAVE_FMSZ_MASK                    
#define DSPI_CTAR_SLAVE_FMSZ_SHIFT                SPI_CTAR_SLAVE_FMSZ_SHIFT                   
#define DSPI_CTAR_SLAVE_FMSZ_WIDTH                SPI_CTAR_SLAVE_FMSZ_WIDTH                   
#define DSPI_CTAR_SLAVE_FMSZ(x)                   SPI_CTAR_SLAVE_FMSZ(x)
#define DSPI_SR_POPNXTPTR_MASK                    SPI_SR_POPNXTPTR_MASK                       
#define DSPI_SR_POPNXTPTR_SHIFT                   SPI_SR_POPNXTPTR_SHIFT                      
#define DSPI_SR_POPNXTPTR_WIDTH                   SPI_SR_POPNXTPTR_WIDTH                      
#define DSPI_SR_POPNXTPTR(x)                      SPI_SR_POPNXTPTR(x)                         
#define DSPI_SR_RXCTR_MASK                        SPI_SR_RXCTR_MASK                           
#define DSPI_SR_RXCTR_SHIFT                       SPI_SR_RXCTR_SHIFT                          
#define DSPI_SR_RXCTR_WIDTH                       SPI_SR_RXCTR_WIDTH                          
#define DSPI_SR_RXCTR(x)                          SPI_SR_RXCTR(x)                             
#define DSPI_SR_TXNXTPTR_MASK                     SPI_SR_TXNXTPTR_MASK                        
#define DSPI_SR_TXNXTPTR_SHIFT                    SPI_SR_TXNXTPTR_SHIFT                       
#define DSPI_SR_TXNXTPTR_WIDTH                    SPI_SR_TXNXTPTR_WIDTH                       
#define DSPI_SR_TXNXTPTR(x)                       SPI_SR_TXNXTPTR(x)                          
#define DSPI_SR_TXCTR_MASK                        SPI_SR_TXCTR_MASK                           
#define DSPI_SR_TXCTR_SHIFT                       SPI_SR_TXCTR_SHIFT                          
#define DSPI_SR_TXCTR_WIDTH                       SPI_SR_TXCTR_WIDTH                          
#define DSPI_SR_TXCTR(x)                          SPI_SR_TXCTR(x)                             
#define DSPI_SR_RFDF_MASK                         SPI_SR_RFDF_MASK                            
#define DSPI_SR_RFDF_SHIFT                        SPI_SR_RFDF_SHIFT                           
#define DSPI_SR_RFDF_WIDTH                        SPI_SR_RFDF_WIDTH                           
#define DSPI_SR_RFDF(x)                           SPI_SR_RFDF(x)                              
#define DSPI_SR_RFOF_MASK                         SPI_SR_RFOF_MASK                            
#define DSPI_SR_RFOF_SHIFT                        SPI_SR_RFOF_SHIFT                           
#define DSPI_SR_RFOF_WIDTH                        SPI_SR_RFOF_WIDTH                           
#define DSPI_SR_RFOF(x)                           SPI_SR_RFOF(x)                              
#define DSPI_SR_TFFF_MASK                         SPI_SR_TFFF_MASK                            
#define DSPI_SR_TFFF_SHIFT                        SPI_SR_TFFF_SHIFT                           
#define DSPI_SR_TFFF_WIDTH                        SPI_SR_TFFF_WIDTH                           
#define DSPI_SR_TFFF(x)                           SPI_SR_TFFF(x)                              
#define DSPI_SR_TFUF_MASK                         SPI_SR_TFUF_MASK                            
#define DSPI_SR_TFUF_SHIFT                        SPI_SR_TFUF_SHIFT                           
#define DSPI_SR_TFUF_WIDTH                        SPI_SR_TFUF_WIDTH                           
#define DSPI_SR_TFUF(x)                           SPI_SR_TFUF(x)                              
#define DSPI_SR_EOQF_MASK                         SPI_SR_EOQF_MASK                            
#define DSPI_SR_EOQF_SHIFT                        SPI_SR_EOQF_SHIFT                           
#define DSPI_SR_EOQF_WIDTH                        SPI_SR_EOQF_WIDTH                           
#define DSPI_SR_EOQF(x)                           SPI_SR_EOQF(x)                              
#define DSPI_SR_TXRXS_MASK                        SPI_SR_TXRXS_MASK                           
#define DSPI_SR_TXRXS_SHIFT                       SPI_SR_TXRXS_SHIFT                          
#define DSPI_SR_TXRXS_WIDTH                       SPI_SR_TXRXS_WIDTH                          
#define DSPI_SR_TXRXS(x)                          SPI_SR_TXRXS(x)                             
#define DSPI_SR_TCF_MASK                          SPI_SR_TCF_MASK                             
#define DSPI_SR_TCF_SHIFT                         SPI_SR_TCF_SHIFT                            
#define DSPI_SR_TCF_WIDTH                         SPI_SR_TCF_WIDTH                            
#define DSPI_SR_TCF(x)                            SPI_SR_TCF(x)                                                            
#define DSPI_RSER_RFDF_DIRS_MASK                  SPI_RSER_RFDF_DIRS_MASK                     
#define DSPI_RSER_RFDF_DIRS_SHIFT                 SPI_RSER_RFDF_DIRS_SHIFT                    
#define DSPI_RSER_RFDF_DIRS_WIDTH                 SPI_RSER_RFDF_DIRS_WIDTH                    
#define DSPI_RSER_RFDF_DIRS(x)                    SPI_RSER_RFDF_DIRS(x)                       
#define DSPI_RSER_RFDF_RE_MASK                    SPI_RSER_RFDF_RE_MASK                       
#define DSPI_RSER_RFDF_RE_SHIFT                   SPI_RSER_RFDF_RE_SHIFT                      
#define DSPI_RSER_RFDF_RE_WIDTH                   SPI_RSER_RFDF_RE_WIDTH                      
#define DSPI_RSER_RFDF_RE(x)                      SPI_RSER_RFDF_RE(x)                         
#define DSPI_RSER_RFOF_RE_MASK                    SPI_RSER_RFOF_RE_MASK                       
#define DSPI_RSER_RFOF_RE_SHIFT                   SPI_RSER_RFOF_RE_SHIFT                      
#define DSPI_RSER_RFOF_RE_WIDTH                   SPI_RSER_RFOF_RE_WIDTH                      
#define DSPI_RSER_RFOF_RE(x)                      SPI_RSER_RFOF_RE(x)                         
#define DSPI_RSER_TFFF_DIRS_MASK                  SPI_RSER_TFFF_DIRS_MASK                     
#define DSPI_RSER_TFFF_DIRS_SHIFT                 SPI_RSER_TFFF_DIRS_SHIFT                    
#define DSPI_RSER_TFFF_DIRS_WIDTH                 SPI_RSER_TFFF_DIRS_WIDTH                    
#define DSPI_RSER_TFFF_DIRS(x)                    SPI_RSER_TFFF_DIRS(x)                       
#define DSPI_RSER_TFFF_RE_MASK                    SPI_RSER_TFFF_RE_MASK                       
#define DSPI_RSER_TFFF_RE_SHIFT                   SPI_RSER_TFFF_RE_SHIFT                      
#define DSPI_RSER_TFFF_RE_WIDTH                   SPI_RSER_TFFF_RE_WIDTH                      
#define DSPI_RSER_TFFF_RE(x)                      SPI_RSER_TFFF_RE(x)                         
#define DSPI_RSER_TFUF_RE_MASK                    SPI_RSER_TFUF_RE_MASK                       
#define DSPI_RSER_TFUF_RE_SHIFT                   SPI_RSER_TFUF_RE_SHIFT                      
#define DSPI_RSER_TFUF_RE_WIDTH                   SPI_RSER_TFUF_RE_WIDTH                      
#define DSPI_RSER_TFUF_RE(x)                      SPI_RSER_TFUF_RE(x)                         
#define DSPI_RSER_EOQF_RE_MASK                    SPI_RSER_EOQF_RE_MASK                       
#define DSPI_RSER_EOQF_RE_SHIFT                   SPI_RSER_EOQF_RE_SHIFT                      
#define DSPI_RSER_EOQF_RE_WIDTH                   SPI_RSER_EOQF_RE_WIDTH                      
#define DSPI_RSER_EOQF_RE(x)                      SPI_RSER_EOQF_RE(x)                         
#define DSPI_RSER_TCF_RE_MASK                     SPI_RSER_TCF_RE_MASK                        
#define DSPI_RSER_TCF_RE_SHIFT                    SPI_RSER_TCF_RE_SHIFT                       
#define DSPI_RSER_TCF_RE_WIDTH                    SPI_RSER_TCF_RE_WIDTH                       
#define DSPI_RSER_TCF_RE(x)                       SPI_RSER_TCF_RE(x)                                             
#define DSPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_MASK   SPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_MASK      
#define DSPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_SHIFT  SPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_SHIF      
#define DSPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_WIDTH  SPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD_WIDT      
#define DSPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD(x)     SPI_PUSHR_FIFO_CMD_PUSHR_FIFO_CMD(x)                           
#define DSPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_MASK     SPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_MASK        
#define DSPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_SHIFT    SPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_SHIFT       
#define DSPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_WIDTH    SPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX_WIDTH       
#define DSPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX(x)       SPI_PUSHR_FIFO_TX_PUSHR_FIFO_TX(x)                               
#define DSPI_PUSHR_TXDATA_MASK                    SPI_PUSHR_TXDATA_MASK                 
#define DSPI_PUSHR_TXDATA_SHIFT                   SPI_PUSHR_TXDATA_SHIFT                
#define DSPI_PUSHR_TXDATA_WIDTH                   SPI_PUSHR_TXDATA_WIDTH                
#define DSPI_PUSHR_TXDATA(x)                      SPI_PUSHR_TXDATA(x)                   
#define DSPI_PUSHR_PCS_MASK                       SPI_PUSHR_PCS_MASK                    
#define DSPI_PUSHR_PCS_SHIFT                      SPI_PUSHR_PCS_SHIFT                   
#define DSPI_PUSHR_PCS_WIDTH                      SPI_PUSHR_PCS_WIDTH                   
#define DSPI_PUSHR_PCS(x)                         SPI_PUSHR_PCS(x)                      
#define DSPI_PUSHR_CTCNT_MASK                     SPI_PUSHR_CTCNT_MASK                  
#define DSPI_PUSHR_CTCNT_SHIFT                    SPI_PUSHR_CTCNT_SHIFT                 
#define DSPI_PUSHR_CTCNT_WIDTH                    SPI_PUSHR_CTCNT_WIDTH                 
#define DSPI_PUSHR_CTCNT(x)                       SPI_PUSHR_CTCNT(x)                    
#define DSPI_PUSHR_EOQ_MASK                       SPI_PUSHR_EOQ_MASK                    
#define DSPI_PUSHR_EOQ_SHIFT                      SPI_PUSHR_EOQ_SHIFT                   
#define DSPI_PUSHR_EOQ_WIDTH                      SPI_PUSHR_EOQ_WIDTH                   
#define DSPI_PUSHR_EOQ(x)                         SPI_PUSHR_EOQ(x)                      
#define DSPI_PUSHR_CTAS_MASK                      SPI_PUSHR_CTAS_MASK                   
#define DSPI_PUSHR_CTAS_SHIFT                     SPI_PUSHR_CTAS_SHIFT                  
#define DSPI_PUSHR_CTAS_WIDTH                     SPI_PUSHR_CTAS_WIDTH                  
#define DSPI_PUSHR_CTAS(x)                        SPI_PUSHR_CTAS(x)                     
#define DSPI_PUSHR_CONT_MASK                      SPI_PUSHR_CONT_MASK                   
#define DSPI_PUSHR_CONT_SHIFT                     SPI_PUSHR_CONT_SHIFT                  
#define DSPI_PUSHR_CONT_WIDTH                     SPI_PUSHR_CONT_WIDTH                  
#define DSPI_PUSHR_CONT(x)                        SPI_PUSHR_CONT(x)                                          
#define DSPI_PUSHR_SLAVE_TXDATA_MASK              SPI_PUSHR_SLAVE_TXDATA_MASK                 
#define DSPI_PUSHR_SLAVE_TXDATA_SHIFT             SPI_PUSHR_SLAVE_TXDATA_SHIFT                
#define DSPI_PUSHR_SLAVE_TXDATA_WIDTH             SPI_PUSHR_SLAVE_TXDATA_WIDTH                
#define DSPI_PUSHR_SLAVE_TXDATA(x)                SPI_PUSHR_SLAVE_TXDATA(x)                                                
#define DSPI_POPR_RXDATA_MASK                     SPI_POPR_RXDATA_MASK                        
#define DSPI_POPR_RXDATA_SHIFT                    SPI_POPR_RXDATA_SHIFT                       
#define DSPI_POPR_RXDATA_WIDTH                    SPI_POPR_RXDATA_WIDTH                       
#define DSPI_POPR_RXDATA(x)                       SPI_POPR_RXDATA(x)                                                    
#define DSPI_TXFR_TXDATA_MASK                     SPI_TXFR_TXDATA_MASK                        
#define DSPI_TXFR_TXDATA_SHIFT                    SPI_TXFR_TXDATA_SHIFT                       
#define DSPI_TXFR_TXDATA_WIDTH                    SPI_TXFR_TXDATA_WIDTH                       
#define DSPI_TXFR_TXDATA(x)                       SPI_TXFR_TXDATA(x)                          
#define DSPI_TXFR_TXCMD_TXDATA_MASK               SPI_TXFR_TXCMD_TXDATA_MASK                  
#define DSPI_TXFR_TXCMD_TXDATA_SHIFT              SPI_TXFR_TXCMD_TXDATA_SHIFT                 
#define DSPI_TXFR_TXCMD_TXDATA_WIDTH              SPI_TXFR_TXCMD_TXDATA_WIDTH                 
#define DSPI_TXFR_TXCMD_TXDATA(x)                 SPI_TXFR_TXCMD_TXDATA(x)                                                
#define DSPI_RXFR_RXDATA_MASK                     SPI_RXFR_RXDATA_MASK                        
#define DSPI_RXFR_RXDATA_SHIFT                    SPI_RXFR_RXDATA_SHIFT                       
#define DSPI_RXFR_RXDATA_WIDTH                    SPI_RXFR_RXDATA_WIDTH                       
#define DSPI_RXFR_RXDATA(x)                       SPI_RXFR_RXDATA(x) 
                         
#if (defined (S32V23x_SERIES) || defined(S32S247TV_SERIES))
#define DSPI_CTARE_DTCP_MASK                      SPI_CTARE_DTCP_MASK
#define DSPI_CTARE_DTCP_SHIFT                     SPI_CTARE_DTCP_SHIFT
#define DSPI_CTARE_DTCP_WIDTH                     SPI_CTARE_DTCP_WIDTH
#define DSPI_CTARE_DTCP(x)                        SPI_CTARE_DTCP(x)
#define DSPI_CTARE_FMSZE_MASK                     SPI_CTARE_FMSZE_MASK
#define DSPI_CTARE_FMSZE_SHIFT                    SPI_CTARE_FMSZE_SHIFT
#define DSPI_CTARE_FMSZE_WIDTH                    SPI_CTARE_FMSZE_WIDTH
#define DSPI_CTARE_FMSZE(x)                       SPI_CTARE_FMSZE(x)
#define DSPI_MCR_XSPI_MASK                        SPI_MCR_XSPI_MASK
#define DSPI_MCR_XSPI_SHIFT                       SPI_MCR_XSPI_SHIFT
#define DSPI_MCR_XSPI_WIDTH                       SPI_MCR_XSPI_WIDTH
#define DSPI_MCR_XSPI(x)                          SPI_MCR_XSPI(x)
#endif

#endif

/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_MSTR(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_MSTR_MASK);
    tmp |= DSPI_MCR_MSTR(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_CONT_SCKE(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_CONT_SCKE_MASK);
    tmp |= DSPI_MCR_CONT_SCKE(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_DCONF(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_DCONF_MASK);
    tmp |= DSPI_MCR_DCONF(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_PCSIS(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_PCSIS_MASK);
    tmp |= DSPI_MCR_PCSIS(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_MDIS(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_MDIS_MASK);
    tmp |= DSPI_MCR_MDIS(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_CLR_TXF(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_CLR_TXF_MASK);
    tmp |= DSPI_MCR_CLR_TXF(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_CLR_RXF(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_CLR_RXF_MASK);
    tmp |= DSPI_MCR_CLR_RXF(to_set);
    baseAddr->MCR = tmp;
}
#if(defined(FEATURE_DSPI_HAS_EXTENDED_MODE))
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_XSPI(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_XSPI_MASK);
    tmp |= DSPI_MCR_XSPI(to_set);
    baseAddr->MCR = tmp;
}
#endif

/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_MCR_HALT(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MCR;
    tmp &= ~(DSPI_MCR_HALT_MASK);
    tmp |= DSPI_MCR_HALT(to_set);
    baseAddr->MCR = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_FMSZ(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_FMSZ_MASK);
    tmp |= DSPI_CTAR_FMSZ(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_CPOL(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_CPOL_MASK);
    tmp |= DSPI_CTAR_CPOL(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_CPHA(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_CPHA_MASK);
    tmp |= DSPI_CTAR_CPHA(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_LSBFE(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_LSBFE_MASK);
    tmp |= DSPI_CTAR_LSBFE(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_PCSSCK(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_PCSSCK_MASK);
    tmp |= DSPI_CTAR_PCSSCK(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_PASC(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_PASC_MASK);
    tmp |= DSPI_CTAR_PASC(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_PDT(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_PDT_MASK);
    tmp |= DSPI_CTAR_PDT(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_PBR(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_PBR_MASK);
    tmp |= DSPI_CTAR_PBR(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_CSSCK(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_CSSCK_MASK);
    tmp |= DSPI_CTAR_CSSCK(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_ASC(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_ASC_MASK);
    tmp |= DSPI_CTAR_ASC(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_DT(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_DT_MASK);
    tmp |= DSPI_CTAR_DT(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_BR(DSPI_Type* baseAddr, uint8_t index, uint8_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR[index];
    tmp &= ~(DSPI_CTAR_BR_MASK);
    tmp |= DSPI_CTAR_BR(to_set);
    baseAddr->MODE.CTAR[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_SLAVE_FMSZ(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR_SLAVE[0];
    tmp &= ~(DSPI_CTAR_SLAVE_FMSZ_MASK);
    tmp |= DSPI_CTAR_SLAVE_FMSZ(to_set);
    baseAddr->MODE.CTAR_SLAVE[0] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTAR_SLAVE_CPHA(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->MODE.CTAR_SLAVE[0];
    tmp &= ~(DSPI_CTAR_SLAVE_CPHA_MASK);
    tmp |= DSPI_CTAR_SLAVE_CPHA(to_set);
    baseAddr->MODE.CTAR_SLAVE[0] = tmp;
}

/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @return
 */
static inline uint32_t DSPI_Get_SR_TXCTR(const DSPI_Type* baseAddr)
{
    uint32_t tmp = baseAddr->SR;
    tmp = (tmp & DSPI_SR_TXCTR_MASK) >> DSPI_SR_TXCTR_SHIFT;
    return ( uint32_t ) (tmp);
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @return
 */
static inline uint32_t DSPI_Get_SR_RXCTR(const DSPI_Type* baseAddr)
{
    uint32_t tmp = baseAddr->SR;
    tmp = (tmp & DSPI_SR_RXCTR_MASK) >> DSPI_SR_RXCTR_SHIFT;
    return ( uint32_t ) (tmp);
}

/*!
 * @brief
 * @param baseAddr dspi base pointer
 */
static inline void DSPI_Clear_SR_EOQF(DSPI_Type* baseAddr)
{
    baseAddr->SR = DSPI_SR_EOQF(1);
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 */
static inline void DSPI_Clear_SR_TFUF(DSPI_Type* baseAddr)
{
    baseAddr->SR = DSPI_SR_TFUF(1);
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 */
static inline void DSPI_Clear_SR_TFFF(DSPI_Type* baseAddr)
{
    baseAddr->SR = DSPI_SR_TFFF(1);
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 */
static inline void DSPI_Clear_SR_RFOF(DSPI_Type* baseAddr)
{
    baseAddr->SR = DSPI_SR_RFOF(1);
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 */
static inline void DSPI_Clear_SR_RFDF(DSPI_Type* baseAddr)
{
    baseAddr->SR = DSPI_SR_RFDF(1);
}

#if(defined(FEATIRES_DSPI_HAS_SHARED_INTERRUPTS))
   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline bool DSPI_Get_RSER_RFDF_RE(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->RSER;
       tmp = (tmp & DSPI_RSER_RFDF_RE_MASK) >> DSPI_RSER_RFDF_RE_SHIFT;
       return ( bool ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline bool DSPI_Get_RSER_RFOF_RE(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->RSER;
       tmp = (tmp & DSPI_RSER_RFOF_RE_MASK) >> DSPI_RSER_RFOF_RE_SHIFT;
       return ( bool ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline bool DSPI_Get_RSER_TFFF_RE(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->RSER;
       tmp = (tmp & DSPI_RSER_TFFF_RE_MASK) >> DSPI_RSER_TFFF_RE_SHIFT;
       return ( bool ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline bool DSPI_Get_RSER_TFUF_RE(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->RSER;
       tmp = (tmp & DSPI_RSER_TFUF_RE_MASK) >> DSPI_RSER_TFUF_RE_SHIFT;
       return ( bool ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline uint32_t DSPI_Get_SR_RFDF(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->SR;
       tmp = (tmp & DSPI_SR_RFDF_MASK) >> DSPI_SR_RFDF_SHIFT;
       return ( uint32_t ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline uint32_t DSPI_Get_SR_RFOF(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->SR;
       tmp = (tmp & DSPI_SR_RFOF_MASK) >> DSPI_SR_RFOF_SHIFT;
       return ( uint32_t ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline uint32_t DSPI_Get_SR_TFFF(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->SR;
       tmp = (tmp & DSPI_SR_TFFF_MASK) >> DSPI_SR_TFFF_SHIFT;
       return ( uint32_t ) (tmp);
   }

   /*!
    * @brief
    * @param baseAddr dspi base pointer
    * @return
    */
   static inline uint32_t DSPI_Get_SR_TFUF(const DSPI_Type* baseAddr)
   {
       uint32_t tmp = baseAddr->SR;
       tmp = (tmp & DSPI_SR_TFUF_MASK) >> DSPI_SR_TFUF_SHIFT;
       return ( uint32_t ) (tmp);
   }
#endif

/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_TFUF_RE(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_TFUF_RE_MASK);
    tmp |= DSPI_RSER_TFUF_RE(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_TFFF_RE(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_TFFF_RE_MASK);
    tmp |= DSPI_RSER_TFFF_RE(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_TFFF_DIRS(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_TFFF_DIRS_MASK);
    tmp |= DSPI_RSER_TFFF_DIRS(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_RFOF_RE(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_RFOF_RE_MASK);
    tmp |= DSPI_RSER_RFOF_RE(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_RFDF_RE(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_RFDF_RE_MASK);
    tmp |= DSPI_RSER_RFDF_RE(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_RSER_RFDF_DIRS(DSPI_Type* baseAddr, uint32_t to_set)
{
    uint32_t tmp = baseAddr->RSER;
    tmp &= ~(DSPI_RSER_RFDF_DIRS_MASK);
    tmp |= DSPI_RSER_RFDF_DIRS(to_set);
    baseAddr->RSER = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @return
 */
static inline uint32_t DSPI_Get_POPR_RXDATA(const DSPI_Type* baseAddr)
{
    uint32_t tmp = baseAddr->POPR;
    tmp = (tmp & DSPI_POPR_RXDATA_MASK) >> DSPI_POPR_RXDATA_SHIFT;
    return ( uint32_t ) (tmp);
}
#if(defined(FEATURE_DSPI_HAS_EXTENDED_MODE))
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTARE_FMSZE(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->CTARE[index];
    tmp &= ~(DSPI_CTARE_FMSZE_MASK);
    tmp |= DSPI_CTARE_FMSZE(to_set);
    baseAddr->CTARE[index] = tmp;
}
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Set_CTARE_DTCP(DSPI_Type* baseAddr, uint8_t index, uint32_t to_set)
{
    uint32_t tmp = baseAddr->CTARE[index];
    tmp &= ~(DSPI_CTARE_DTCP_MASK);
    tmp |= DSPI_CTARE_DTCP(to_set);
    baseAddr->CTARE[index] = tmp;
}
#endif
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
#ifndef FEATURE_DSPI_HAS_EXTENDED_MODE

static inline void DSPI_Set_PUSHR(DSPI_Type* baseAddr, uint32_t to_set)
{
    baseAddr->PUSHR.PUSHR = to_set;
}
#endif
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
#if(defined(FEATURE_DSPI_HAS_EXTENDED_MODE))
static inline void DSPI_Set_PUSHR_CMD(DSPI_Type* baseAddr, uint16_t to_set)
{
    baseAddr->PUSHR.FIFO.CMD = to_set;
}
#endif
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
#if(defined(FEATURE_DSPI_HAS_EXTENDED_MODE))
static inline void DSPI_Set_PUSHR_TX(DSPI_Type* baseAddr, uint16_t to_set)
{
    if((baseAddr->MCR & DSPI_MCR_MSTR_MASK) == DSPI_MCR_MSTR_MASK)
    {
        baseAddr->PUSHR.FIFO.TX = to_set;
    }
    else
    {
        baseAddr->PUSHR.SLAVE = to_set;
    }
}
#endif
/*!
 * @brief
 * @param baseAddr dspi base pointer
 * @param to_set
 */
static inline void DSPI_Reset(DSPI_Type* baseAddr)
{
    uint32_t i;
    baseAddr->MCR = DSPI_MCR_CLR_RXF_MASK;
    i = baseAddr->POPR;
    baseAddr->MCR = DSPI_MCR_MDIS_MASK | DSPI_MCR_HALT_MASK | DSPI_MCR_CLR_RXF_MASK | DSPI_MCR_CLR_TXF_MASK;
    baseAddr->TCR = 0;
    for (i=0U; i< DSPI_CTAR_COUNT; i++)
    {
        baseAddr->MODE.CTAR[i] = 0;
        #if (defined(FEATURE_DSPI_HAS_EXTENDED_MODE))
        baseAddr->CTARE[i] = 0;
        #endif
    }
    baseAddr->SR = 0xFFFFFFFFU;
    baseAddr->RSER = 0;
}
#endif /* DSPI_HEADER_PRIVATE_H_ */
