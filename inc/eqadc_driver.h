/*
 * Copyright 2018-2019 NXP
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

#ifndef EQADC_DRIVER_H
#define EQADC_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"
#include "edma_driver.h"

/*! @file eqadc_driver.h
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The macros define bitmasks used to access different flags.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * Function-like macros are created to allow access to on-chip ADC internal registers.
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
 */

/*!
 * @addtogroup eqadc_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief This enumeration defines the internal timebase counter clock prescaler
 *
 * Implements : eqadc_timebase_prescale_t_Class
 */
typedef enum
{
    EQADC_TIMEBASE_PRESCALE_DISABLED = 0u,    /*!< The timebase counter clock is disabled */
    EQADC_TIMEBASE_PRESCALE_1        = 1u,    /*!< The system clock divided by 1 is selected */
    EQADC_TIMEBASE_PRESCALE_2        = 2u,    /*!< The system clock divided by 2 is selected */
    EQADC_TIMEBASE_PRESCALE_4        = 3u,    /*!< The system clock divided by 4 is selected */
    EQADC_TIMEBASE_PRESCALE_6        = 4u,    /*!< The system clock divided by 6 is selected */
    EQADC_TIMEBASE_PRESCALE_8        = 5u,    /*!< The system clock divided by 8 is selected */
    EQADC_TIMEBASE_PRESCALE_10       = 6u,    /*!< The system clock divided by 10 is selected */
    EQADC_TIMEBASE_PRESCALE_12       = 7u,    /*!< The system clock divided by 12 is selected */
    EQADC_TIMEBASE_PRESCALE_16       = 8u,    /*!< The system clock divided by 16 is selected */
    EQADC_TIMEBASE_PRESCALE_32       = 9u,    /*!< The system clock divided by 32 is selected */
    EQADC_TIMEBASE_PRESCALE_64       = 10u,   /*!< The system clock divided by 64 is selected */
    EQADC_TIMEBASE_PRESCALE_128      = 11u,   /*!< The system clock divided by 128 is selected */
    EQADC_TIMEBASE_PRESCALE_256      = 12u,   /*!< The system clock divided by 256 is selected */
    EQADC_TIMEBASE_PRESCALE_512      = 13u    /*!< The system clock divided by 512 is selected */
} eqadc_timebase_prescale_t;

/*!
 * @brief This enumeration selects the timebase source
 *
 * Implements : eqadc_timebase_sel_t_Class
 */
typedef enum
{
    EQADC_TIMEBASE_SEL_INTERNAL      = 0u,    /*!< Select the internal timebase */
    EQADC_TIMEBASE_SEL_IMPORTED_SRV1 = 1u,    /*!< Select imported timebase 1 indicated by SRV1 bit field of EQADC_STACCCR */
    EQADC_TIMEBASE_SEL_IMPORTED_SRV2 = 2u     /*!< Select imported timebase 2 indicated by SRV2 bit field of EQADC_STACCCR */
} eqadc_timebase_sel_t;

/*!
 * @brief This enumeration defines the on-chip ADC clock prescaler values
 *
 * Implements : eqadc_prescale_t_Class
 */
typedef enum
{
    EQADC_PRESCALE_2  = 0u,    /*!< The system clock divided by 2 is selected */
    EQADC_PRESCALE_4  = 1u,    /*!< The system clock divided by 4 is selected */
    EQADC_PRESCALE_6  = 2u,    /*!< The system clock divided by 6 is selected */
    EQADC_PRESCALE_8  = 3u,    /*!< The system clock divided by 8 is selected */
    EQADC_PRESCALE_10 = 4u,    /*!< The system clock divided by 10 is selected */
    EQADC_PRESCALE_12 = 5u,    /*!< The system clock divided by 12 is selected */
    EQADC_PRESCALE_14 = 6u,    /*!< The system clock divided by 14 is selected */
    EQADC_PRESCALE_16 = 7u,    /*!< The system clock divided by 16 is selected */
    EQADC_PRESCALE_18 = 8u,    /*!< The system clock divided by 18 is selected */
    EQADC_PRESCALE_20 = 9u,    /*!< The system clock divided by 20 is selected */
    EQADC_PRESCALE_22 = 10u,   /*!< The system clock divided by 22 is selected */
    EQADC_PRESCALE_24 = 11u,   /*!< The system clock divided by 24 is selected */
    EQADC_PRESCALE_26 = 12u,   /*!< The system clock divided by 26 is selected */
    EQADC_PRESCALE_28 = 13u,   /*!< The system clock divided by 28 is selected */
    EQADC_PRESCALE_30 = 14u,   /*!< The system clock divided by 30 is selected */
    EQADC_PRESCALE_32 = 15u,   /*!< The system clock divided by 32 is selected */
    EQADC_PRESCALE_34 = 16u,   /*!< The system clock divided by 34 is selected */
    EQADC_PRESCALE_36 = 17u,   /*!< The system clock divided by 36 is selected */
    EQADC_PRESCALE_38 = 18u,   /*!< The system clock divided by 38 is selected */
    EQADC_PRESCALE_40 = 19u,   /*!< The system clock divided by 40 is selected */
    EQADC_PRESCALE_42 = 20u,   /*!< The system clock divided by 42 is selected */
    EQADC_PRESCALE_44 = 21u,   /*!< The system clock divided by 44 is selected */
    EQADC_PRESCALE_46 = 22u,   /*!< The system clock divided by 46 is selected */
    EQADC_PRESCALE_48 = 23u,   /*!< The system clock divided by 48 is selected */
    EQADC_PRESCALE_50 = 24u,   /*!< The system clock divided by 50 is selected */
    EQADC_PRESCALE_52 = 25u,   /*!< The system clock divided by 52 is selected */
    EQADC_PRESCALE_54 = 26u,   /*!< The system clock divided by 54 is selected */
    EQADC_PRESCALE_56 = 27u,   /*!< The system clock divided by 56 is selected */
    EQADC_PRESCALE_58 = 28u,   /*!< The system clock divided by 58 is selected */
    EQADC_PRESCALE_60 = 29u,   /*!< The system clock divided by 60 is selected */
    EQADC_PRESCALE_62 = 30u,   /*!< The system clock divided by 62 is selected */
    EQADC_PRESCALE_64 = 31u,   /*!< The system clock divided by 64 is selected */
    EQADC_PRESCALE_NOT_USED = 255u /*! The system clock is selected - maximum frequency */
} eqadc_prescale_t;

/*!
 * @brief This enumeration defines the command fifo (cfifo) operation modes
 *
 * Implements : eqadc_cfifo_mode_t_Class
 */
typedef enum
{
    EQADC_CFIFO_MODE_DISABLED                       = 0u,    /*! CFIFO operation mode is Disabled */
    EQADC_CFIFO_MODE_SW_TRIG_SINGLE                 = 1u,    /*! CFIFO operation mode is Software Trigger, Single Scan */
    EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_SINGLE  = 2u,    /*! CFIFO operation mode is Low Level Gated External Trigger, Single Scan */
    EQADC_CFIFO_MODE_HIGH_LVL_GATED_EXT_TRIG_SINGLE = 3u,    /*! CFIFO operation mode is High Level Gated External Trigger, Single Scan */
    EQADC_CFIFO_MODE_FALLING_EDGE_EXT_TRIG_SINGLE   = 4u,    /*! CFIFO operation mode is Falling Edge External Trigger, Single Scan */
    EQADC_CFIFO_MODE_RISING_EDGE_EXT_TRIG_SINGLE    = 5u,    /*! CFIFO operation mode is Rising Edge External Trigger, Single Scan */
    EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_SINGLE     = 6u,    /*! CFIFO operation mode is Falling or Rising Edge External Trigger, Single Scan */
    EQADC_CFIFO_MODE_SW_TRIG_CONT                   = 9u,    /*! CFIFO operation mode is Software Trigger, Continuous Scan */
    EQADC_CFIFO_MODE_LOW_LVL_GATED_EXT_TRIG_CONT    = 10u,   /*! CFIFO operation mode is Low Level Gated External Trigger, Continuous Scan */
    EQADC_CFIFO_MODE_HIGH_LVL_GATED_EXT_TRIG_CONT   = 11u,   /*! CFIFO operation mode is High Level Gated External Trigger, Continuous Scan */
    EQADC_CFIFO_MODE_FALLING_EDGE_EXT_TRIG_CONT     = 12u,   /*! CFIFO operation mode is Falling Edge External Trigger, Continuous Scan */
    EQADC_CFIFO_MODE_RISING_EDGE_EXT_TRIG_CONT      = 13u,   /*! CFIFO operation mode is Rising Edge External Trigger, Continuous Scan */
    EQADC_CFIFO_MODE_BOTH_EDGES_EXT_TRIG_CONT       = 14u    /*! CFIFO operation mode is Falling or Rising Edge External Trigger, Continuous Scan */
} eqadc_cfifo_mode_t;

/*!
 * @brief This enumeration defines the available command fifo advance trigger modes
 *
 * Implements : eqadc_atrig_mode_t_Class
 */
typedef enum
{
    EQADC_ATRIG_MODE_DISABLED                       = 0u,    /*! CFIFO0 advance trigger operation mode is Disabled */
    EQADC_ATRIG_MODE_SW_TRIG_SINGLE                 = 1u,    /*! CFIFO0 advance trigger operation mode is Software Trigger, Single Scan */
    EQADC_ATRIG_MODE_LOW_LVL_GATED_EXT_TRIG_SINGLE  = 2u,    /*! CFIFO0 advance trigger operation mode is Low Level Gated External Trigger, Single Scan */
    EQADC_ATRIG_MODE_HIGH_LVL_GATED_EXT_TRIG_SINGLE = 3u,    /*! CFIFO0 advance trigger operation mode is High Level Gated External Trigger, Single Scan */
    EQADC_ATRIG_MODE_FALLING_EDGE_EXT_TRIG_SINGLE   = 4u,    /*! CFIFO0 advance trigger operation mode is Falling Edge External Trigger, Single Scan */
    EQADC_ATRIG_MODE_RISING_EDGE_EXT_TRIG_SINGLE    = 5u,    /*! CFIFO0 advance trigger operation mode is Rising Edge External Trigger, Single Scan */
    EQADC_ATRIG_MODE_BOTH_EDGES_EXT_TRIG_SINGLE     = 6u     /*! CFIFO0 advance trigger operation mode is Falling or Rising Edge External Trigger, Single Scan */
} eqadc_atrig_mode_t;

/*!
 * @brief This enumeration defines the clock sources selectable for Trigger Burst Generator
 *
 * Implements : eqadc_tbg_clksel_t_Class
 */
typedef enum
{
    EQADC_TBG_CLKSEL_DISABLED = 0u,    /*! Trigger Burst Generator is disabled */
    EQADC_TBG_CLKSEL_SYSTEM   = 1u,    /*! The EQADC system clock is selected */
    EQADC_TBG_CLKSEL_ADC_0    = 2u,    /*! The on-chip ADC0 clock is selected */
    EQADC_TBG_CLKSEL_ADC_1    = 3u     /*! The on-chip ADC1 clock is selected */
} eqadc_tbg_clksel_t;

/*!
 * @brief This enumeration defines the available pull resistor types for input channels
 *
 * Implements : eqadc_chan_pull_t_Class
 */
typedef enum
{
    EQADC_CHAN_PULL_DISABLED    = 0u,    /*! No Pull resistors connected to the channel */
    EQADC_CHAN_PULL_UP_EN       = 1u,    /*! Pull Up resistor connected to the channel */
    EQADC_CHAN_PULL_DOWN_EN     = 2u,    /*! Pull Down resistor connected to the channel */
    EQADC_CHAN_PULL_UP_DOWN_EN  = 3u     /*! Pull Up and Pull Down resistors connected to the channel */
} eqadc_chan_pull_t;

/*!
 * @brief This enumeration defines the available pull resistor values for input channels
 *
 * Implements : eqadc_pull_strength_t_Class
 */
typedef enum
{
    EQADC_PULL_STRENGTH_200KOHM = 1u,   /*! 200 Kohms pull resistor is selected */
    EQADC_PULL_STRENGTH_100KOHM = 2u,   /*! 100 Kohms pull resistor is selected */
    EQADC_PULL_STRENGTH_5KOHM   = 3u    /*! 5 Kohms (Approx.) pull resistor is selected.
                                            This is not available when both PULL UP and PULL DOWN are enabled */
} eqadc_pull_strength_t;

/*!
 * @brief This enumeration defines the possible values for command fifo (cfifo) status
 *
 * Implements : eqadc_cfifo_status_t_Class
 */
typedef enum
{
    CFIFO_STATUS_IDLE             = 0u,   /*! CFIFO is idle */
    CFIFO_STATUS_WAITING_FOR_TRIG = 2u,   /*! CFIFO is waiting for trigger */
    CFIFO_STATUS_TRIGGERED        = 3u    /*! CFIFO is triggered */
} eqadc_cfifo_status_t;

/*!
 * @brief This enumeration defines the message tag values. It can be used for selecting the destination rfifo for the conversion result
 *
 * Implements : eqadc_message_tag_t_Class
 */
typedef enum
{
    EQADC_MESSAGE_TAG_RFIFO0            = 0u,   /*! Result is sent to RFIFO 0 */
    EQADC_MESSAGE_TAG_RFIFO1            = 1u,   /*! Result is sent to RFIFO 1 */
    EQADC_MESSAGE_TAG_RFIFO2            = 2u,   /*! Result is sent to RFIFO 2 */
    EQADC_MESSAGE_TAG_RFIFO3            = 3u,   /*! Result is sent to RFIFO 3 */
    EQADC_MESSAGE_TAG_RFIFO4            = 4u,   /*! Result is sent to RFIFO 4 */
    EQADC_MESSAGE_TAG_RFIFO5            = 5u,   /*! Result is sent to RFIFO 5 */
    EQADC_MESSAGE_TAG_NULL              = 8u,   /*! Null Message Received */
    EQADC_MESSAGE_TAG_RESERVED_CUSTOM_0 = 9u,   /*! Reserved for customer use */
    EQADC_MESSAGE_TAG_RESERVED_CUSTOM_1 = 10u   /*! Reserved for customer use */
} eqadc_message_tag_t;

/*!
 * @brief This enumeration defines the possible sampling time values (expressed in on-chip ADC clock cycles).
 *
 * Implements : eqadc_sampling_time_t_Class
 */
typedef enum
{
    EQADC_SAMPLING_TIME_2_CYCLES   = 0u,    /*! 2 sampling cycles */
    EQADC_SAMPLING_TIME_8_CYCLES   = 1u,    /*! 8 sampling cycles */
    EQADC_SAMPLING_TIME_64_CYCLES  = 2u,    /*! 64 sampling cycles */
    EQADC_SAMPLING_TIME_128_CYCLES = 3u     /*! 128 sampling cycles */
} eqadc_sampling_time_t;


/*!
 * @brief This enumeration selects the alternate configuration which may be used for a conversion command
 *
 * Implements : eqadc_alt_config_sel_t_Class
 */
typedef enum
{
    EQADC_ALT_CONFIG_SEL_DISABLED  = 0x0u,   /*! Select Standard Configuration for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_1  = 0x8u,   /*! Select Alternate Configuration 1 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_2  = 0x9u,   /*! Select Alternate Configuration 2 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_3  = 0xAu,   /*! Select Alternate Configuration 3 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_4  = 0xBu,   /*! Select Alternate Configuration 4 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_5  = 0xCu,   /*! Select Alternate Configuration 5 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_6  = 0xDu,   /*! Select Alternate Configuration 6 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_7  = 0xEu,   /*! Select Alternate Configuration 7 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_8  = 0xFu,   /*! Select Alternate Configuration 8 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_9  = 0x10u,  /*! Select Alternate Configuration 9 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_10 = 0x11u,  /*! Select Alternate Configuration 10 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_11 = 0x12u,  /*! Select Alternate Configuration 11 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_12 = 0x13u,  /*! Select Alternate Configuration 12 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_13 = 0x14u,  /*! Select Alternate Configuration 13 for the conversion command */
    EQADC_ALT_CONFIG_SEL_CONFIG_14 = 0x15u   /*! Select Alternate Configuration 14 for the conversion command */
} eqadc_alt_config_sel_t;

/*!
 * @brief This enumeration selects the type of operation for a configuration command
 *
 * Implements : eqadc_config_cmd_type_t_Class
 */
typedef enum
{
    EQADC_CONFIG_CMD_TYPE_WRITE = 0u,   /*! The configuration command is write command */
    EQADC_CONFIG_CMD_TYPE_READ  = 1u    /*! The configuration command is read command */
} eqadc_config_cmd_type_t;

/*!
 * @brief This enumeration defines the available values for the STAC Bus timebase bits used to control which time slots the EQADC selects to obtain pre-defined external time bases
 *
 * Implements : eqadc_stac_bits_sel_t_Class
 */
typedef enum
{
    EQADC_STAC_BITS_SEL_TBASE_0_15 = 0u,   /*! Select 16 bits[0:15] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_1_16 = 1u,   /*! Select 16 bits[1:16] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_2_17 = 2u,   /*! Select 16 bits[2:17] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_3_18 = 3u,   /*! Select 16 bits[3:18] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_4_19 = 4u,   /*! Select 16 bits[4:19] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_5_20 = 5u,   /*! Select 16 bits[5:20] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_6_21 = 6u,   /*! Select 16 bits[6:21] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_7_22 = 7u,   /*! Select 16 bits[7:22] from the total of 24 bits timebase */
    EQADC_STAC_BITS_SEL_TBASE_8_23 = 8u    /*! Select 16 bits[8:23] from the total of 24 bits timebase */
} eqadc_stac_bits_sel_t;

/*!
 * @brief This enumeration selects the type of request issued by a fifo
 *
 * Implements : eqadc_req_type_sel_t_Class
 */
typedef enum
{
    EQADC_REQ_TYPE_SEL_INT = 0u,    /*! Interrupt request will be generated by fifo */
    EQADC_REQ_TYPE_SEL_DMA = 1u     /*! DMA request will be generated by fifo */
} eqadc_req_type_sel_t;

/*!
 * @brief This enumeration defines the available resolution values
 *
 * Implements : eqadc_resolution_t_Class
 */
typedef enum
{
    EQADC_RESOLUTION_12_BITS = 0u,   /*! ADC result is 12-bits resolution */
    EQADC_RESOLUTION_10_BITS = 1u,   /*! ADC result is 10-bits resolution */
    EQADC_RESOLUTION_8_BITS  = 2u    /*! ADC result is 8-bits resolution */
} eqadc_resolution_t;

/*!
 * @brief This enumeration defines the available pregain values
 *
 * Implements : eqadc_pregain_t_Class
 */
typedef enum
{
    EQADC_PREGAIN_X1 = 0u,   /*!< Input gain is 1 */
    EQADC_PREGAIN_X2 = 1u,   /*!< Input gain is 2 */
    EQADC_PREGAIN_X4 = 2u    /*!< Input gain is 4 */
} eqadc_pregain_t;

/*!
 * @brief Structure for on-chip ADC properties.
 *
 * This structure is used for configuring on-chip ADC properties.
 *
 * Implements : eqadc_adc_config_t_Class
 */
typedef struct
{
    uint8_t adcIdx;                     /*! Index of the on-chip ADC to be configured */
    bool extMuxEn;                      /*! true/false - enable/disable the external multiplexer. If enabled, external multiplexer channels can be used.
                                         *   NOTE: Cannot be enabled for both on-chip ADCs at the same time. */
    eqadc_timebase_sel_t timebaseSel;   /*! Selects the source of the time information to be used as timestamp.
                                         *   Overridden when using alternate configurations, by equivalent struct member in eqadc_alternate_config_t */
    eqadc_prescale_t clkPrescale;       /*! Selects the prescale value. If clkOddPrescaleEn is true, then the prescaler value is clkPrescale+1. */
    bool clkOddPrescaleEn;              /*! true/false - odd/even divide factor is selected */
    bool clkDutySel;                    /*! Clock Duty Rate select (only for odd divide factors)
                                         * true  - clock high pulse is 1 clock longer than the low interval
                                         * false - clock low interval is 1 clock longer than the low interval */
    uint16_t adcGain;                   /*! Selects the calibration gain constant applied to conversion results (GCCR). The value is expressed as fixed point 1Q14 (1 bit for the integer part, 14 bits for fractional part)
                                         *   Calibrated result is equal to: uncalibrated_result x adcGain + adcOffset + 2 */
    uint16_t adcOffset;                 /*! Selects the calibration offset constant applied to conversion results (OCCR). The value is expressed as signed 14 bits - negative values should be expressed as two's complement.
                                         *   Calibrated result is equal to: uncalibrated_result x adcGain + adcOffset + 2 */

    bool immediateConvCmdEn;            /*! true  - The EQADC aborts any current conversions and immediately starts the conversion commands from CFIFO0.
                                         *          The commands which were already executing or pending, will (re)start execution after commands from CFIFO0 have completed.
                                         * false - The EQADC executes commands in normal priority scheme. When CFIFO0 is triggered, its conversion command can be put
                                         *          behind 2 pending conversion commands in the Cbuffer, commands which might be from lower priority cfifos.
                                         *          EQADC_MCR.ICEA */
} eqadc_adc_config_t;


/*!
 * @brief Structure for command fifo (cfifo) properties.
 *
 * This structure is used for configuring command fifo (cfifo) properties. Configures fields in registers CFCR, SIUL_ISEL
 *
 * Implements : eqadc_cfifo_config_t_Class
 */
typedef struct
{
    uint8_t cfifoIdx;               /*! Index of the selected command fifo (cfifo) */
    eqadc_cfifo_mode_t opMode;      /*! Select the cfifo operation mode */

    eqadc_req_type_sel_t fillReqSel;/*! Select if a DMA or interrupt request is generated when CFIFO Fill Flag (CFFFx) is set (i.e. cfifo is not full) IDCR CFFS */
    uint8_t dmaVirtualChan;         /*! DMA virtual channel to be used if cfifo is to be filled by DMA (fillReqSel is set to DMA) */
    uint32_t * sourcePtr;           /*! Pointer to the memory address from where the commands will be copied by the DMA, if DMA transfer is enabled */
    uint32_t sourceLength;          /*! Length (number of uint32_t) of the source buffer pointed by sourcePtr */
    edma_callback_t callback;       /*! Callback function called when the rQueue has been filled by the DMA */
    void * callbackParam;           /*! Parameter to be passed to the callback function pointer */

    uint8_t trigSel;                /*! Select the trigger source for the cfifo. Has effect if opMode is not SW triggered. SIU_ISEL 4-7*/

    eqadc_tbg_clksel_t tbgClksel;   /*! Select the clock used for Trigger Burst Generator SIU_TBG_CR Clksel. If selected clock is disabled the Trigger Burst Generator functionality is disabled
                                     *  and module operation is transparent regarding to triggers hw triggers selected via trigSel.
                                     *  For details please refer to dedicated chapter from SIU module in the reference manual. */
    uint32_t tbgTriggerPeriod;      /*! Trigger period for Trigger Burst Generator - the time interval between the triggers in a burst.
                                     *  For details please refer to dedicated chapter from SIU module in the reference manual. SIU_TBG_CR TRIGPER */
    /*! Members only used for CFIFO0 */
    bool entryNumExtensionEn;       /*! true - enable extension of cfifo entries
                                     *  false - cfifo has normal number of entries.
                                     *  NOTE: only available for CFIFO0. CFCR CFEEE0 */
    eqadc_atrig_mode_t advanceTrigMode;  /*! Select the trigger mode for the ATRIG0 trigger signal in streaming mode.
                                          * If DISABLE is selected, streaming mode is disabled.
                                          *  NOTE: only available for CFIFO0 */
    uint8_t advanceTrigSel;         /*! SIU_ISEL9. Select the signal used as advance trigger in streaming mode.
                                     *  NOTE: only available for CFIFO0. Only used when streaming mode is enabled.  */

} eqadc_cfifo_config_t;

/*!
 * @brief Structure for DMA parameters for moving data from a result fifo (rfifo) into system memory.
 *
 * This structure is used for configuring DMA parameters for moving data from a result fifo (rfifo) into system memory.
 *
 * Implements : eqadc_result_dma_config_t_Class
 */
typedef struct
{
    uint8_t rfifoIdx;            /*! Index of the rfifo from which results should be transfered via DMA */
    uint8_t dmaVirtualChan;      /*! DMA virtual channel to be used if rfifo is to be drained by DMA (drainReqSel is set to DMA) */
    uint16_t * destPtr;          /*! Pointer to the memory address where the results will be copied by the DMA, if DMA transfer is enabled */
    uint32_t destLength;         /*! Length (num of uint16_t words) of the destination buffer pointed by destPtr */
    edma_callback_t callback;    /*! Callback function called when the rQueue has been filled by the DMA */
    void * callbackParam;        /*! Parameter to be passed to the callback function pointer */
    bool enHalfDestCallback;     /*! Enable the callback to be called when half of the destination buffer has been filled */
} eqadc_result_dma_config_t;

/*!
 * @brief Structure for seting properties of EQADC alternate configurations
 *
 * This structure is used for seting properties of EQADC alternate configurations.
 *
 * Implements : eqadc_alternate_config_t_Class
 */
typedef struct
{
    eqadc_alt_config_sel_t altConfigSel; /*! Select the alternate configuration register */
    /*! Members for ACR register bit-fields*/
    bool resultInhibit;                /*! true/false - If true, inhibits the transfer of the result data from the peripheral module to the result queue. (ACR RET_INH)
                                        *  When the selected companion module is Decimation Filter, setting to 'true' will set the filter in PRE-FILL special mode. */
    uint8_t dest;                      /*! Select the primary destination of the conversion results */
    bool signedResEn;                  /*! true/false - result sent to primary destination (selected by dest1) is right justified signed/unsigned. (ACR FMTA) */
    bool returnPSI;                    /*! true/false - companion module results from this eQADC are sent to other PSI/to the same eQADC using same PSI. Used to obtain double sample rate. (ACR RPSI)*/
    eqadc_resolution_t adcResolution;  /*! Resolution of the results converted via this alternate configuration (ACR RESSEL) */
    eqadc_timebase_sel_t timebaseSel;  /*! Select the time information to be used as timestamp for alternate conversion commands. (ACR ATBSEL)
                                        *  Overrides eqadc_adc_config_t.timebaseSel when this alternate configuration is used */
    eqadc_pregain_t adcPregain;        /*! Pregain value (ACR PRE_GAIN)*/
    /*! Member for AGR register - gain for conversion results only from alt configs 1 & 2, for each ADC */
    uint16_t adcGain[EQADC_NUM_ADC];   /*! Gain calibration constant used for results for alternate configurations 1 or 2.
                                        *  NOTE: field is only used for alternate configurations 1 and 2.
                                        *  The value is expressed as fixed point 1Q14 (1 bit for the integer part, 14 bits for fractional part)
                                        *  Calibrated result is equal to: uncalibrated_result x adcGain + adcOffset + 2  */
    /*! Member for AOR register - offset for conversion results only from alt configs 1 & 2, for each ADC */
    uint16_t adcOffset[EQADC_NUM_ADC]; /*! Offset calibration constant used for results for alternate configurations 1 or 2.
                                        *  NOTE: field is only used for alternate configurations 1 and 2.
                                        *  The value is expressed as fixed point 1Q14 (1 bit for the integer part, 14 bits for fractional part)
                                        *  Calibrated result is equal to: uncalibrated_result x adcGain + adcOffset + 2  */
} eqadc_alternate_config_t;

/*!
 * @brief Structure for setting properties of EQADC extended alternate configurations
 *
 * This structure is used for setting properties of EQADC extended alternate configurations.
 * It can be used to route conversion results to a second destination (besides rfifo).
 * Configures EACR resiter bit-fields
 *
 * Implements : eqadc_ext_alternate_config_t_Class
 */
typedef struct
{
    eqadc_alt_config_sel_t extAltConfigSel;  /*! Select the extended alternate configuration register - mapped 1:1 with alternate configuration registers. */
    bool resultInhibit2;               /*! true/false - If true, inhibits the transfer of the result data from the peripheral module to the result queue. (EACR RET_INH2)
                                        *  When the selected companion module is Decimation Filter, setting to 'true' will set the filter in PRE-FILL special mode. */
    uint8_t dest2;                     /*! Select the second destination of the conversion results (EACR DEST2)*/
    bool signedResEn2;                 /*! true/false - result sent to second destination (selected by dest2) is right justified signed/unsigned (EACR FMTA2)*/
    bool returnPSI2;                   /*! true/false - companion module results from this eQADC are sent to other PSI/to the same eQADC using same PSI. Used to obtain double sample rate. (EACR RPSI2) */
    bool flushEn2;                     /*! true/false - enable/disable flush command to DEST2 if specified by FFMT when DEST > 0. (EACR FLEN2) */
    bool msgTagEn2;                    /*! true/false - enable/disable message tag to be used to the result transfer of DEST2 (EACR TEN2)*/
    eqadc_message_tag_t msgTag2;       /*! Message TAG 2 (EACR MESSAGE_TAG2)*/
} eqadc_ext_alternate_config_t;

/*!
 * @brief Structure for configuring EQADC pull resistor properties
 *
 * This structure is used for configuring EQADC pull resistor properties. It is mapped to PUDCR internal on-chip ADC register bitfields.
 *
 * Implements : eqadc_chan_pull_config_t_Class
 */
typedef struct
{
    uint8_t adcInputChanIdx;                 /*! Index of the ADC input channel */
    eqadc_chan_pull_t chanPull;              /*! Pull resistor type */
    eqadc_pull_strength_t chanPullStrength;  /*! Pull resistor strength */
} eqadc_chan_pull_config_t;

/*!
 * @brief Structure for configuring EQADC STAC Bus Client properties
 *
 * This structure is used for configuring EQADC STAC Bus Client properties. It is mapped to STACCCR register bitfields.
 *
 * Implements : eqadc_stac_bus_client_config_t_Class
 */
typedef struct
{
    uint32_t clientIdx;                     /*! Index of the STAC Bus Client */
    uint8_t serverDataSlotSel;              /*! STAC Bus Server Data Slot select: the slot number that contains the desired time base value sent by the STAC Bus server */
    eqadc_stac_bits_sel_t timebaseBitsSel;  /*! Timebase Bits select: select 16 bits from the total of 24 bits that are received from the STAC Bus interface */
} eqadc_stac_bus_client_config_t;


/*!
 * @brief Structure for configuring EQADC parameters and functionalities.
 *
 * This structure is used for configuring EQADC parameters and functionalities.
 *
 * Implements : eqadc_config_t_Class
 */
typedef struct
{
    bool dbgReqMaskEn;          /*! true/false - allow/ignore debug requests.
                                 *   If true, the EQADC enters debug mode when a debug mode entry request arrives. Otherwise debug requests are ignored. (MCR DBG) */
    uint8_t digitalFilterLen;   /*! Length of the External Trigger Digital Filter (ETDFR DFL). Specifies the minimum number of system clocks that must be counted to recognize a logic state change. EQADC_ETDFR
                                 *   The actual sample period is equal to: SystemClockPeriod * (2 ^ DFL) + 1 SystemClockPeriod */
    eqadc_timebase_prescale_t timebasePrescale;                 /*! System clock divide factor used for the time base counter clock. It determines at what frequency
                                                                 *   the time base counter will run or if it is disabled. (TSCR internal register) */
    eqadc_adc_config_t * adcConfigArray;                        /*! Pointer to an array with ADC configuration structures - for configuring on-chip ADC parameters. */
    uint8_t numAdcConfigs;                                      /*! Number of elements available in adcConfigArray */
    eqadc_cfifo_config_t * cfifoConfigArray;                    /*! Pointer to an array with command fifo (cfifo) configuration structures. */
    uint8_t numCfifoConfigs;                                    /*! Number of elements available in numCfifoConfigs */
    eqadc_result_dma_config_t * resultDmaConfigArray;           /*! Pointer to an array with configuration structures for DMA transferring from result fifos.
                                                                 *   If a result fifo (rfifo) is selected for DMA support, then the EQADC will issue DMA request
                                                                 *   whenever RFIFO Drain Flag (RFDFx) is set (i.e. there is a result available in the rfifo) and
                                                                 *   the result will be transfered to the corresponding configured destination buffer.
                                                                 *   If an rfifo is not selected for DMA support, the EQADC will by default issue an interrupt request whenever the RFIFO Drain Flag (RFDFx) is set. */
    uint8_t numResultDmaConfigs;                                /*! Number of elements available in resultDmaConfigArray */
    eqadc_alternate_config_t * alternateConfigArray;            /*! Pointer to an array with alternate configuration structures. */
    uint8_t numAlternateConfigs;                                /*! Number of elements available in alternateConfigArray */
    eqadc_ext_alternate_config_t * extAlternateConfigArray;     /*! Pointer to an array with extended alternate configuration structures. */
    uint8_t numExtAlternateConfigs;                             /*! Number of elements available in extAlternateConfigArray.
                                                                 *   If >0, MCR DAM bit is set to 1, selecting Extended Alternate Configuration Registers to send results to two destinations.
                                                                 *   If ==0, MCR DAM bit is set to 0, selecting Legacy Mode: use a defined set of pairs of companion module destinations in the PSI. */
    eqadc_chan_pull_config_t * chanPullConfigArray;             /*! Pointer to an array of input channel pull resistors configuration structures. */
    uint8_t numChanPullConfigs;                                 /*! Number of elements available in chanPullConfigArray */
    eqadc_stac_bus_client_config_t * stacBusClientConfigArray;  /*! Pointer to an array with STAC bus client configuration structures. */
    uint8_t numStacBusClientConfigs;                            /*! Number of elements available in stacBusClientConfigArray */
} eqadc_config_t;


/*!
 * @brief Selects the fifo status information
 *
 * This enumeration defines the status information which may be retrieved for a command or result fifo.
 * The entries cover fields available in FISR.
 *
 * Implements : eqadc_fifo_status_sel_t_Class
 */
typedef enum
{
    EQADC_FIFO_STATUS_SEL_NON_COHERENCY         = 0u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_TRIG_OVERRUN          = 1u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_PAUSE                 = 2u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_END_OF_QUEUE          = 3u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_CFIFO_UNDERFLOW       = 4u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_CFIFO_SINGLE_SCAN     = 5u,   /*! Corresp to single bit value. Cannot be cleared. */
    EQADC_FIFO_STATUS_SEL_CFIFO_FILL            = 6u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_RFIFO_OVERFLOW        = 7u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_RFIFO_DRAIN           = 8u,   /*! Corresp to single bit value. */
    EQADC_FIFO_STATUS_SEL_CFIFO_ENTRY_CNTR      = 9u,   /*! Corresp to multiple bit value. */
    EQADC_FIFO_STATUS_SEL_CFIFO_TRANS_NEXT_PTR  = 10u,  /*! Corresp to multiple bit value. */
    EQADC_FIFO_STATUS_SEL_RFIFO_ENTRY_CNTR      = 11u,  /*! Corresp to multiple bit value. */
    EQADC_FIFO_STATUS_SEL_RFIFO_POP_NEXT_PTR    = 12u,  /*! Corresp to multiple bit value. */
    EQADC_FIFO_STATUS_SEL_ALL                   = 13u   /*! ONLY used for clear operation. Cannot be used with GetFifoStatus function */
} eqadc_fifo_status_sel_t;


/*!
 * @brief Defines for selecting status flags to be cleared
 * Implements : EQADC_FIFO_STATUS_FLAG_Class
 */
#define EQADC_FIFO_STATUS_FLAG_NON_COHERENCY     (EQADC_FISR_NCFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_TRIG_OVERRUN      (EQADC_FISR_TORFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_PAUSE             (EQADC_FISR_PFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_END_OF_QUEUE      (EQADC_FISR_EOQFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_CFIFO_UNDERFLOW   (EQADC_FISR_CFUFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_CFIFO_FILL        (EQADC_FISR_CFFFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_RFIFO_OVERFLOW    (EQADC_FISR_RFOFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN       (EQADC_FISR_RFDFX_MASK)
#define EQADC_FIFO_STATUS_FLAG_ALL               (EQADC_FIFO_STATUS_FLAG_NON_COHERENCY | EQADC_FIFO_STATUS_FLAG_TRIG_OVERRUN | \
                                                  EQADC_FIFO_STATUS_FLAG_PAUSE | EQADC_FIFO_STATUS_FLAG_END_OF_QUEUE | \
                                                  EQADC_FIFO_STATUS_FLAG_CFIFO_UNDERFLOW | EQADC_FIFO_STATUS_FLAG_CFIFO_FILL | \
                                                  EQADC_FIFO_STATUS_FLAG_RFIFO_OVERFLOW | EQADC_FIFO_STATUS_FLAG_RFIFO_DRAIN)

/*!
 * @brief Defines for selecting fifo interrupt sources
 * Implements : EQADC_INT_EN_Class
 */
#define EQADC_INT_EN_NON_COHERENCY           (EQADC_IDCR0_NCIE1_MASK)   /*! Mask for enabling Non-Coherency Interrupt  */
#define EQADC_INT_EN_TRIG_OVERRUN            (EQADC_IDCR0_TORIE1_MASK)  /*! Mask for enabling Trigger Overrun Interrupt*/
#define EQADC_INT_EN_PAUSE                   (EQADC_IDCR0_PIE1_MASK)    /*! Mask for enabling Pause Interrupt */
#define EQADC_INT_EN_END_OF_QUEUE            (EQADC_IDCR0_EOQIE1_MASK)  /*! Mask for enabling End of Queue Interrupt */
#define EQADC_INT_EN_CFIFO_UNDERFLOW         (EQADC_IDCR0_CFUIE1_MASK)  /*! Mask for enabling cfifo Underflow Interrupt */
#define EQADC_INT_EN_CFIFO_FILL              (EQADC_IDCR0_CFFE1_MASK)   /*! Mask for enabling generation of an interrupt when cfifo is not full */
#define EQADC_INT_EN_RFIFO_OVERFLOW          (EQADC_IDCR0_RFOIE1_MASK)  /*! Mask for enabling RFIFO Overflow Interrupt */
#define EQADC_INT_EN_RFIFO_DRAIN             (EQADC_IDCR0_RFDE1_MASK)   /*! Mask for enabling generation of an interrupt when RFIFO has at least one valid entry in it */
#define EQADC_INT_EN_ALL                     (EQADC_INT_EN_NON_COHERENCY | EQADC_INT_EN_TRIG_OVERRUN | EQADC_INT_EN_PAUSE | \
                                              EQADC_INT_EN_END_OF_QUEUE | EQADC_INT_EN_CFIFO_UNDERFLOW | EQADC_INT_EN_CFIFO_FILL | \
                                              EQADC_INT_EN_RFIFO_OVERFLOW | EQADC_INT_EN_RFIFO_DRAIN)

/*!
 * @brief Defines for selecting fifo DMA requests
 * Implements : EQADC_DMA_REQ_EN_Class
 */
#define EQADC_DMA_REQ_EN_CFIFO_FILL          (EQADC_IDCR0_CFFE1_MASK)   /*! Mask for enabling DMA request when CFIFO is not full */
#define EQADC_DMA_REQ_EN_RFIFO_DRAIN         (EQADC_IDCR0_RFDE1_MASK)   /*! Mask for enabling DMA request when RFIFO has at least one valid entry in it */
#define EQADC_DMA_REQ_EN_ALL                 (EQADC_DMA_REQ_EN_CFIFO_FILL | EQADC_DMA_REQ_EN_RFIFO_DRAIN)


/*!
 * @brief Structure for conversion commands - used for setting conversion parameters
 *
 * This structure is used for conversion commands - used for setting conversion parameters
 *
 * Implements : eqadc_conv_cmd_t_Class
 */
typedef struct
{
    bool eoq;                           /*! true/false - set/clear End Of Queue bit for the current command */
    bool pause;                         /*! true/false - enter/do not enter 'waiting for trigger' state after transfer of the current command */
    bool repeatStart;                   /*! true/false - indicates if current command is the start point of the sub-queue to be repeated */
    uint8_t cbufferNum;                 /*! Select the destination command buffer (cBuffer). Each cBuffer is mapped to a single on chip ADC.  */

    bool calibEn;                       /*! true/false - if true, the returning conversion result will be calibrated using the following formula:
                                             calibrated result = uncalibrated_result x adcGain + adcOffset + 2. \n
                                             NOTE: when alternate configurations are selected, values of adcGain and adcOffset are used from the alternate configuration
                                             parameters (for instances where they are available) instead of from ADC config parameters. */
    eqadc_message_tag_t msgTag;         /*! Select the destination RFIFO */
    eqadc_sampling_time_t samplingTime; /*! Select the sampling time in ADC clock cycles.
                                         *   Has effect in standard format. For alternate configurations format (altConfigSel != 0), has effect only if ADC_ACR.DEST == 0. */
    bool timeStampReq;                  /*! true/false - if true, the on-chip ADC returns a timestamp for the current conversion command after the conversion result is sent to the rfifos */
    uint8_t channelNum;                 /*! Input channel number to be measured by the command */
    bool signEn;                        /*! true/false - conversion data is right justified signed/unsigned.
                                             Only used in standard format (altConfigSel == DISABLED) */
    eqadc_alt_config_sel_t altConfigSel;  /*! Select which of the alternate configurations is used for current command.
                                           *   If an alternate configuration is selected, the alternate configuration format is used for the command.
                                           *   Otherwise, the standard configuration format is used for the command.
                                           *   When one of the alternate configurations is selected, the conversion result can be routed to one of the RFIFOs and/or
                                           *   to the parallel side interface to communicate with an on-chip companion module.
                                           *   The selected alternate configuration needs to be configured separately using eqadc_alternate_config_t */
    /*! Members only available in alternate configurations format (altConfigSel != DISABLED) */
    bool flushCompanion;                  /*! true/false - if true and destination configured for the selected corresponding alternate configuration is != 0,
                                           *   send a flush (soft-reset) signal to the selected companion module.
                                           *   NOTE: only used when an alternate configuration is selected, and has effect only if result is routed to a companion module (ADC_ACR.DEST != 0). */
} eqadc_conv_cmd_t;


/*!
 * @brief This enumeration selects which set of calibration gain and offset to be configured
 *
 * Implements : eqadc_calibration_target_t_Class
 */
typedef enum
{
    EQADC_CALIBRATION_TARGET_MAIN          = EQADC_ALT_CONFIG_SEL_DISABLED,   /*! Configure main ADC gain and offset - corresponding to GCCR and OCCR  internal registers */
    EQADC_CALIBRATION_TARGET_ALT_CONFIG_1  = EQADC_ALT_CONFIG_SEL_CONFIG_1,   /*! Configure ADC gain and offset for Alternate Configuration 1 - corresponding to AGR1 and AOR1 internal registers */
    EQADC_CALIBRATION_TARGET_ALT_CONFIG_2  = EQADC_ALT_CONFIG_SEL_CONFIG_2    /*! Configure ADC gain and offset for Alternate Configuration 2 - corresponding to AGR2 and AOR2 internal registers */
} eqadc_calibration_target_t;


/*!
 * @brief Structure for parameters useb by the calibration function
 *
 * This structure is used for parameters useb by the calibration function
 *
 * Implements : eqadc_calibration_params_t_Class
 */
typedef struct
{
    uint32_t rfifoIdx;                        /*! Index of the result fifo (rfifo) to be used for calibration procedure */
    uint32_t cfifoIdx;                        /*! Index of the command fifo (cfifo) to be used for calibration procedure */
    eqadc_calibration_target_t calibTarget;   /*! Select which set of gain and offset params to be calibrated */
    uint32_t timeout;                         /*! Maximum duration allowed for the calibration procedure, expressed in milliseconds */
    eqadc_sampling_time_t samplingTime;       /*! Sampling time to be used for conversions on reference channels */
    uint16_t adcGain;                         /*! ADC gain calibration value, resulted from the calibration procedure. Value is expressed as fixed point 1Q14 format */
    int16_t adcOffset;                        /*! ADC offset calibration value, resulted from the calibration procedure. Value is expressed as 14 bit signed two's complement */
} eqadc_calibration_params_t;


/*!
 * @brief Defines addresses of the on-chip ADC registers
 *
 * This enumeration defines the addresses of the on-chip ADC registers.
 *
 * Implements : eqadc_onchip_adc_reg_address_t_Class
 */
typedef enum
{
    EQADC_ONCHIP_ADC_REG_ADDRESS_CR         = 0x01u,    /*! The address of ADC0/ADC1 Configuration Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_TSCR       = 0x02u,    /*! The address of Time Stamp Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_TBCR       = 0x03u,    /*! The address of Time Base Counter Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_GCCR       = 0x04u,    /*! The address of ADC0/ADC1 Gain Calibration Constant Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_OCCR       = 0x05u,    /*! The address of ADC0/ADC1 Offset Calibration Constant Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR1       = 0x30u,    /*! The address of Alternate Configuration 1 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_AGR1       = 0x31u,    /*! The address of ADC0/ADC1 Alternate Gain 1 Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_AOR1       = 0x32u,    /*! The address of ADC0/ADC1 Alternate Offset 1 Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR1      = 0x33u,    /*! The address of Extended Alternate Configuration 1 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR2       = 0x34u,    /*! The address of Alternate Configuration 2 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_AGR2       = 0x35u,    /*! The address of ADC0/ADC1 Alternate Gain 2 Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_AOR2       = 0x36u,    /*! The address of ADC0/ADC1 Alternate Offset 2 Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR2      = 0x37u,    /*! The address of Extended Alternate Configuration 2 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR3       = 0x38u,    /*! The address of Alternate Configuration 3 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR3      = 0x3Bu,    /*! The address of Extended Alternate Configuration 3 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR4       = 0x3Cu,    /*! The address of Alternate Configuration 4 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR4      = 0x3Fu,    /*! The address of AExtended Alternate Configuration 4 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR5       = 0x40u,    /*! The address of Alternate Configuration 5 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR5      = 0x43u,    /*! The address of Extended Alternate Configuration 5 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR6       = 0x44u,    /*! The address of Alternate Configuration 6 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR6      = 0x47u,    /*! The address of Extended Alternate Configuration 6 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR7       = 0x48u,    /*! The address of Alternate Configuration 7 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR7      = 0x4Bu,    /*! The address of Extended Alternate Configuration 7 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR8       = 0x4Cu,    /*! The address of Alternate Configuration 8 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR8      = 0x4Fu,    /*! The address of Extended Alternate Configuration 8 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR9       = 0x50u,    /*! The address of Alternate Configuration 9 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR9      = 0x53u,    /*! The address of AExtended Alternate Configuration 9 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR10      = 0x54u,    /*! The address of Alternate Configuration 10 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR10     = 0x57u,    /*! The address of Extended Alternate Configuration 10 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR11      = 0x58u,    /*! The address of Alternate Configuration 11 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR11     = 0x5Bu,    /*! The address of Extended Alternate Configuration 11 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR12      = 0x5Cu,    /*! The address of Alternate Configuration 12 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR12     = 0x5Fu,    /*! The address of Extended Alternate Configuration 12 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR13      = 0x60u,    /*! The address of Alternate Configuration 13 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR13     = 0x63u,    /*! The address of Extended Alternate Configuration 13 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_ACR14      = 0x64u,    /*! The address of Alternate Configuration 14 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_EACR14     = 0x67u,    /*! The address of Extended Alternate Configuration 14 Control Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR0     = 0x70u,    /*! The address of Pull Up/Down Control Register 0 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR1     = 0x71u,    /*! The address of Pull Up/Down Control Register 1 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR2     = 0x72u,    /*! The address of Pull Up/Down Control Register 2 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR3     = 0x73u,    /*! The address of Pull Up/Down Control Register 3 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR4     = 0x74u,    /*! The address of Pull Up/Down Control Register 4 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR5     = 0x75u,    /*! The address of Pull Up/Down Control Register 5 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR6     = 0x76u,    /*! The address of Pull Up/Down Control Register 6 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR7     = 0x77u,    /*! The address of Pull Up/Down Control Register 7 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR8     = 0x78u,    /*! The address of Pull Up/Down Control Register 8 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR9     = 0x79u,    /*! The address of Pull Up/Down Control Register 9 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR10    = 0x7Au,    /*! The address of Pull Up/Down Control Register 10 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR11    = 0x7Bu,    /*! The address of Pull Up/Down Control Register 11 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR12    = 0x7Cu,    /*! The address of Pull Up/Down Control Register 12 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR13    = 0x7Du,    /*! The address of Pull Up/Down Control Register 13 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR14    = 0x7Eu,    /*! The address of Pull Up/Down Control Register 14 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR15    = 0x7Fu,    /*! The address of Pull Up/Down Control Register 15 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR16    = 0x80u,    /*! The address of Pull Up/Down Control Register 16 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR17    = 0x81u,    /*! The address of Pull Up/Down Control Register 17 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR18    = 0x82u,    /*! The address of Pull Up/Down Control Register 18 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR19    = 0x83u,    /*! The address of Pull Up/Down Control Register 19 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR20    = 0x84u,    /*! The address of Pull Up/Down Control Register 20 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR21    = 0x85u,    /*! The address of Pull Up/Down Control Register 21 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR22    = 0x86u,    /*! The address of Pull Up/Down Control Register 22 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR23    = 0x87u,    /*! The address of Pull Up/Down Control Register 23 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR24    = 0x88u,    /*! The address of Pull Up/Down Control Register 24 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR25    = 0x89u,    /*! The address of Pull Up/Down Control Register 25 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR26    = 0x8Au,    /*! The address of Pull Up/Down Control Register 26 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR27    = 0x8Bu,    /*! The address of Pull Up/Down Control Register 27 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR28    = 0x8Cu,    /*! The address of Pull Up/Down Control Register 28 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR29    = 0x8Du,    /*! The address of Pull Up/Down Control Register 29 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR30    = 0x8Eu,    /*! The address of Pull Up/Down Control Register 30 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR31    = 0x8Fu,    /*! The address of Pull Up/Down Control Register 31 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR32    = 0x90u,    /*! The address of Pull Up/Down Control Register 32 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR33    = 0x91u,    /*! The address of Pull Up/Down Control Register 33 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR34    = 0x92u,    /*! The address of Pull Up/Down Control Register 34 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR35    = 0x93u,    /*! The address of Pull Up/Down Control Register 35 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR36    = 0x94u,    /*! The address of Pull Up/Down Control Register 36 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR37    = 0x95u,    /*! The address of Pull Up/Down Control Register 37 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR38    = 0x96u,    /*! The address of Pull Up/Down Control Register 38 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_PUDCR39    = 0x97u,    /*! The address of Pull Up/Down Control Register 39 */
    EQADC_ONCHIP_ADC_REG_ADDRESS_STACTBR1   = 0xA0u,    /*! The address of STAC Bus Time Base 1 Register */
    EQADC_ONCHIP_ADC_REG_ADDRESS_STACTBR2   = 0xA1u     /*! The address of STAC Bus Time Base 1 Register */
} eqadc_onchip_adc_reg_address_t;


/*!
 * @brief Structure for configuration commands - used for reading/writing on-chip ADC registers
 *
 * This structure is used for configuration commands - used for reading/writing on-chip ADC registers
 *
 * Implements : eqadc_config_cmd_t_Class
 */
typedef struct
{
    bool eoq;                        /*! true/false - set/clear End Of Queue bit for the current command */
    bool pause;                      /*! true/false - enter/do not enter 'waiting for trigger' state after transfer of the current command */
    bool repeatStart;                /*! true/false - indicates if current command is the start point of the sub-queue to be repeated */
    uint8_t cbufferNum;              /*! Select the destination command buffer (cBuffer). Each cBuffer is mapped to a single on chip ADC.
                                      *   Some registers are mapped 1:1 with on-chip ADC instances, and need to be accessed via corresponding cbuffer index.
                                      *   Others can be accessed by config commands sent to any cbuffer index. */
    eqadc_config_cmd_type_t type;    /*! Specify the type of config command: READ or WRITE config */
    eqadc_onchip_adc_reg_address_t adcRegAddress;   /*! Address for selecting an internal on-chip ADC register */
    uint16_t adcRegValue;            /*! Only used for WRITE config commands */
    eqadc_message_tag_t msgTag;      /*! Select the destination RFIFO. NOTE: only used for READ config commands. */
} eqadc_config_cmd_t;


/*!
 * @brief Macros and defines for accessing on-chip ADC register bitfields.
 * Implements : EQADC_ADC_Class
 */
/*! ADC CR Bit Fields */
#define EQADC_ADC_CR_CLK_PS_MASK                 0x1Fu
#define EQADC_ADC_CR_CLK_PS_SHIFT                0u
#define EQADC_ADC_CR_CLK_PS_WIDTH                5u
#define EQADC_ADC_CR_CLK_PS(x)                   ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_CLK_PS_SHIFT))&EQADC_ADC_CR_CLK_PS_MASK))
#define EQADC_ADC_CR_CLK_SEL_MASK                0x20u
#define EQADC_ADC_CR_CLK_SEL_SHIFT               5u
#define EQADC_ADC_CR_CLK_SEL_WIDTH               1u
#define EQADC_ADC_CR_CLK_SEL(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_CLK_SEL_SHIFT))&EQADC_ADC_CR_CLK_SEL_MASK))
#define EQADC_ADC_CR_CLK_DTY_MASK                0x40u
#define EQADC_ADC_CR_CLK_DTY_SHIFT               6u
#define EQADC_ADC_CR_CLK_DTY_WIDTH               1u
#define EQADC_ADC_CR_CLK_DTY(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_CLK_DTY_SHIFT))&EQADC_ADC_CR_CLK_DTY_MASK))
#define EQADC_ADC_CR_ODD_PS_MASK                 0x80u
#define EQADC_ADC_CR_ODD_PS_SHIFT                7u
#define EQADC_ADC_CR_ODD_PS_WIDTH                1u
#define EQADC_ADC_CR_ODD_PS(x)                   ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_ODD_PS_SHIFT))&EQADC_ADC_CR_ODD_PS_MASK))
#define EQADC_ADC_CR_TBSEL_MASK                  0x300u
#define EQADC_ADC_CR_TBSEL_SHIFT                 8u
#define EQADC_ADC_CR_TBSEL_WIDTH                 2u
#define EQADC_ADC_CR_TBSEL(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_TBSEL_SHIFT))&EQADC_ADC_CR_TBSEL_MASK))
#define EQADC_ADC_CR_EMUX_MASK                   0x800u
#define EQADC_ADC_CR_EMUX_SHIFT                  11u
#define EQADC_ADC_CR_EMUX_WIDTH                  1u
#define EQADC_ADC_CR_EMUX(x)                     ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_EMUX_SHIFT))&EQADC_ADC_CR_EMUX_MASK))
#define EQADC_ADC_CR_EN_MASK                     0x8000u
#define EQADC_ADC_CR_EN_SHIFT                    15u
#define EQADC_ADC_CR_EN_WIDTH                    1u
#define EQADC_ADC_CR_EN(x)                       ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_CR_EN_SHIFT))&EQADC_ADC_CR_EN_MASK))
/*! ADC TSCR Bit Fields */
#define EQADC_ADC_TSCR_TBC_CLK_PS_MASK           0xFu
#define EQADC_ADC_TSCR_TBC_CLK_PS_SHIFT          0u
#define EQADC_ADC_TSCR_TBC_CLK_PS_WIDTH          4u
#define EQADC_ADC_TSCR_TBC_CLK_PS(x)             ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_TSCR_TBC_CLK_PS_SHIFT))&EQADC_ADC_TSCR_TBC_CLK_PS_MASK))
/*! ADC TBCR Bit Fields */
#define EQADC_ADC_TBCR_TBC_VALUE_MASK            0xFFFFu
#define EQADC_ADC_TBCR_TBC_VALUE_SHIFT           0u
#define EQADC_ADC_TBCR_TBC_VALUE_WIDTH           16u
#define EQADC_ADC_TBCR_TBC_VALUE(x)              ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_TBCR_TBC_VALUE_SHIFT))&EQADC_ADC_TBCR_TBC_VALUE_MASK))
/*! ADC GCCR Bit Fields */
#define EQADC_ADC_GCCR_GCC_MASK                  0x7FFFu
#define EQADC_ADC_GCCR_GCC_SHIFT                 0u
#define EQADC_ADC_GCCR_GCC_WIDTH                 15u
#define EQADC_ADC_GCCR_GCC(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_GCCR_GCC_SHIFT))&EQADC_ADC_GCCR_GCC_MASK))
/*! ADC OCCR Bit Fields */
#define EQADC_ADC_OCCR_OCC_MASK                  0x3FFFu
#define EQADC_ADC_OCCR_OCC_SHIFT                 0u
#define EQADC_ADC_OCCR_OCC_WIDTH                 14u
#define EQADC_ADC_OCCR_OCC(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_OCCR_OCC_SHIFT))&EQADC_ADC_OCCR_OCC_MASK))
/*! ADC ACR Bit Fields */
#define EQADC_ADC_ACR_PRE_GAIN_MASK              0x3u
#define EQADC_ADC_ACR_PRE_GAIN_SHIFT             0u
#define EQADC_ADC_ACR_PRE_GAIN_WIDTH             2u
#define EQADC_ADC_ACR_PRE_GAIN(x)                ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_PRE_GAIN_SHIFT))&EQADC_ADC_ACR_PRE_GAIN_MASK))
#define EQADC_ADC_ACR_ATBSEL_MASK                0xCu
#define EQADC_ADC_ACR_ATBSEL_SHIFT               2u
#define EQADC_ADC_ACR_ATBSEL_WIDTH               2u
#define EQADC_ADC_ACR_ATBSEL(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_ATBSEL_SHIFT))&EQADC_ADC_ACR_ATBSEL_MASK))
#define EQADC_ADC_ACR_RESSEL_MASK                0xC0u
#define EQADC_ADC_ACR_RESSEL_SHIFT               6u
#define EQADC_ADC_ACR_RESSEL_WIDTH               2u
#define EQADC_ADC_ACR_RESSEL(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_RESSEL_SHIFT))&EQADC_ADC_ACR_RESSEL_MASK))
#define EQADC_ADC_ACR_RPSI_MASK                  0x100u
#define EQADC_ADC_ACR_RPSI_SHIFT                 8u
#define EQADC_ADC_ACR_RPSI_WIDTH                 1u
#define EQADC_ADC_ACR_RPSI(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_RPSI_SHIFT))&EQADC_ADC_ACR_RPSI_MASK))
#define EQADC_ADC_ACR_FMTA_MASK                  0x200u
#define EQADC_ADC_ACR_FMTA_SHIFT                 9u
#define EQADC_ADC_ACR_FMTA_WIDTH                 1u
#define EQADC_ADC_ACR_FMTA(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_FMTA_SHIFT))&EQADC_ADC_ACR_FMTA_MASK))
#define EQADC_ADC_ACR_DEST_MASK                  0x3C00u
#define EQADC_ADC_ACR_DEST_SHIFT                 10u
#define EQADC_ADC_ACR_DEST_WIDTH                 4u
#define EQADC_ADC_ACR_DEST(x)                    ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_DEST_SHIFT))&EQADC_ADC_ACR_DEST_MASK))
#define EQADC_ADC_ACR_RET_INH_MASK               0x8000u
#define EQADC_ADC_ACR_RET_INH_SHIFT              15u
#define EQADC_ADC_ACR_RET_INH_WIDTH              1u
#define EQADC_ADC_ACR_RET_INH(x)                 ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_ACR_RET_INH_SHIFT))&EQADC_ADC_ACR_RET_INH_MASK))
/*! ADC AGR Bit Fields */
#define EQADC_ADC_AGR_ALTGCC_MASK                0x7FFFu
#define EQADC_ADC_AGR_ALTGCC_SHIFT               0u
#define EQADC_ADC_AGR_ALTGCC_WIDTH               15u
#define EQADC_ADC_AGR_ALTGCC(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_AGR_ALTGCC_SHIFT))&EQADC_ADC_AGR_ALTGCC_MASK))
/*! ADC AOR Bit Fields */
#define EQADC_ADC_AOR_ALTOCC_MASK                0x3FFFu
#define EQADC_ADC_AOR_ALTOCC_SHIFT               0u
#define EQADC_ADC_AOR_ALTOCC_WIDTH               14u
#define EQADC_ADC_AOR_ALTOCC(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_AOR_ALTOCC_SHIFT))&EQADC_ADC_AOR_ALTOCC_MASK))
/*! ADC EACR Bit Fields */
#define EQADC_ADC_EACR_MESSAGE_TAG2_MASK         0xFu
#define EQADC_ADC_EACR_MESSAGE_TAG2_SHIFT        0u
#define EQADC_ADC_EACR_MESSAGE_TAG2_WIDTH        4u
#define EQADC_ADC_EACR_MESSAGE_TAG2(x)           ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_MESSAGE_TAG2_SHIFT))&EQADC_ADC_EACR_MESSAGE_TAG2_MASK))
#define EQADC_ADC_EACR_TEN2_MASK                 0x10u
#define EQADC_ADC_EACR_TEN2_SHIFT                4u
#define EQADC_ADC_EACR_TEN2_WIDTH                1u
#define EQADC_ADC_EACR_TEN2(x)                   ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_TEN2_SHIFT))&EQADC_ADC_EACR_TEN2_MASK))
#define EQADC_ADC_EACR_FLEN2_MASK                0x80u
#define EQADC_ADC_EACR_FLEN2_SHIFT               7u
#define EQADC_ADC_EACR_FLEN2_WIDTH               1u
#define EQADC_ADC_EACR_FLEN2(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_FLEN2_SHIFT))&EQADC_ADC_EACR_FLEN2_MASK))
#define EQADC_ADC_EACR_RPSI2_MASK                0x100u
#define EQADC_ADC_EACR_RPSI2_SHIFT               8u
#define EQADC_ADC_EACR_RPSI2_WIDTH               1u
#define EQADC_ADC_EACR_RPSI2(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_RPSI2_SHIFT))&EQADC_ADC_EACR_RPSI2_MASK))
#define EQADC_ADC_EACR_FMTA2_MASK                0x200u
#define EQADC_ADC_EACR_FMTA2_SHIFT               9u
#define EQADC_ADC_EACR_FMTA2_WIDTH               1u
#define EQADC_ADC_EACR_FMTA2(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_FMTA2_SHIFT))&EQADC_ADC_EACR_FMTA2_MASK))
#define EQADC_ADC_EACR_DEST2_MASK                0x3C00u
#define EQADC_ADC_EACR_DEST2_SHIFT               10u
#define EQADC_ADC_EACR_DEST2_WIDTH               4u
#define EQADC_ADC_EACR_DEST2(x)                  ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_DEST2_SHIFT))&EQADC_ADC_EACR_DEST2_MASK))
#define EQADC_ADC_EACR_RET_INH2_MASK             0x8000u
#define EQADC_ADC_EACR_RET_INH2_SHIFT            15u
#define EQADC_ADC_EACR_RET_INH2_WIDTH            1u
#define EQADC_ADC_EACR_RET_INH2(x)               ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_EACR_RET_INH2_SHIFT))&EQADC_ADC_EACR_RET_INH2_MASK))
/*! ADC PUDCR Bit Fields */
#define EQADC_ADC_PUDCR_PULL_STR_MASK            0x300u
#define EQADC_ADC_PUDCR_PULL_STR_SHIFT           8u
#define EQADC_ADC_PUDCR_PULL_STR_WIDTH           2u
#define EQADC_ADC_PUDCR_PULL_STR(x)              ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_PUDCR_PULL_STR_SHIFT))&EQADC_ADC_PUDCR_PULL_STR_MASK))
#define EQADC_ADC_PUDCR_CH_PULL_MASK             0x3000u
#define EQADC_ADC_PUDCR_CH_PULL_SHIFT            12u
#define EQADC_ADC_PUDCR_CH_PULL_WIDTH            2u
#define EQADC_ADC_PUDCR_CH_PULL(x)               ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_PUDCR_CH_PULL_SHIFT))&EQADC_ADC_PUDCR_CH_PULL_MASK))
/*! ADC STACTBR Bit Fields */
#define EQADC_ADC_STACTBR_TB_VALUE_MASK          0xFFFFu
#define EQADC_ADC_STACTBR_TB_VALUE_SHIFT         0u
#define EQADC_ADC_STACTBR_TB_VALUE_WIDTH         16u
#define EQADC_ADC_STACTBR_TB_VALUE(x)            ((uint16_t)(((uint16_t)(((uint16_t)(x))<<EQADC_ADC_STACTBR_TB_VALUE_SHIFT))&EQADC_ADC_STACTBR_TB_VALUE_MASK))


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#if defined (__cplusplus)
extern "C" {
#endif

/*!
 * @brief Populates the config structure with default values. Config structure members which are pointer to arrays need to be initialized to point to memory allocated by the caller,
 * before the function call. For each array, the function will populate with the number of elements set in the corresponding config struct member.
 *
 * This function populates the config structure with default values.
 * Config structure members which are pointer to arrays need to be initialized to point to memory allocated by the caller, before the function call.
 * For each array, the function will populate with default values a number of members equal with the number of elements set in the corresponding config struct member.
 * E.g. for initializing N members in 'chanPullConfigArray', the config structure needs to be initialized before being passed to the config function:
 * \n configStruct.numChanPullConfigs = N;
 * \n allocate array: eqadc_chan_pull_config_t allocatedChanPullConfigArray[N];
 * \n init pointer in configStruct: configStruct.chanPullConfigArray = allocatedChanPullConfigArray;
 *
 * The default values configured in all configuration arrays allow a minimum functionality and are ready to use - except eqadc_result_dma_config_t - which requires additional
 * configuration depending on how the DMA controller is initialized externally of EQADC driver.
 *
 * @param config - pointer to the configuration structure which needs to be initialized. Some of the members are input values - please refer to description.
 * @return void
 */
void EQADC_DRV_GetDefaultConfig(eqadc_config_t * const config);

/*!
 * @brief Initializes the selected EQADC instance general settings and on-chip ADC settings.
 *
 * The function initializes general EQADC settings and on-chip ADC settings.
 * Shall configure also the on-chip ADC registers corresponding to structure members from the config parameter, by issuing config cmds.
 *
 * For cfifos, if opMode is single-scan level- or edge-trigger mode, the EQADC_DRV_Init function shall enable detection
 * of HW trigger events, by setting CFCR SSE bit.
 *
 * Note: the gain and offset values configured by the function, will be overwritten if EQADC_DRV_DoCalibration() is called.
 *
 * @param[in] instance - instance number
 * @param[in] config - pointer to the configuration structure
 * @return void
 */
void EQADC_DRV_Init(const uint32_t instance,
                    const eqadc_config_t * const config);

/*!
 * @brief Resets the selected EQADC instance general settings and on-chip ADC settings.
 *
 * The function resets to default the general EQADC settings and on-chip ADC settings.
 * The function shall disable the on-chip ADC instances.
 * Note: the function only stops and does not release the DMA virtual channels
 *
 * @param[in] instance - instance number
 * @return void
 */
void EQADC_DRV_Reset(const uint32_t instance);


/*!
 * @brief Calibrates the selected ADC module for the selected EQADC instance.
 *
 * The function calibrates the selected ADC module for the selected EQADC instance.
 * The calculated gain and offset values are written in the corresponding EQADC registers, overwriting the values configured during EQADC_DRV_Init().
 * The values are also returned in the structure received as input.
 * The function should be called after EQADC_DRV_Init().
 * Important note: the function assumes that during calibration procedure the selected ADC module, CFIFO and RFIFO are not used for other conversions.
 *
 * @param[in] instance - instance number
 * @param[in] adcIdx - index of the ADC module to be calibrated
 * @param[in/out] calibParam - pointer to the structure containing calibration params and calibration results
 * @return STATUS_SUCCESS if the calibration procedure executed in the provided timeout duration\n
 * STATUS_TIMEOUT if the calibration procedure did not complete in the provided timeout duration\n
 * STATUS_ERROR if saturation occurs on the calculated gain value
 */
status_t EQADC_DRV_DoCalibration(const uint32_t instance,
                                 const uint8_t adcIdx,
                                 eqadc_calibration_params_t * const calibParam);


/*!
 * @brief Sets the Single-Scan Enable (SSE) bit for the selected cfifo or cfifo pair (if inSync is set to true)
 *
 * This function sets the Single-Scan Enable (SSE) bit of the selected CFIFO.
 * If the selected cfifo is programmed into single-scan level- or edge-trigger mode, it enables the detection of trigger events.\n
 * If the selected cfifo is programmed into single-scan software trigger mode, it issues a software trigger.\n
 * It has no effect when the selected cfifo is in continuous-scan or disabled mode.\n
 * The bit cannot be cleared by software writes. It gets cleared automatically via HW mechanism - for details please refer to reference manual on CFCR SSEx and FISR SSSx.
 * \n NOTE: the 'inSync' parameter permits simultaneous SW triggering of cfifo pairs, allowing in-phase ADC clocks.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected cfifo
 * @param[in] inSync - select if operation is executed for single cfifo or a pair of cfifos
 *                     \n false - set SSE bit only for the selected cfifo
 *                     \n true - set SSE bit for the selected cfifo and its pair: cfifo(2N) and cfifo(2N+1).
 *                     If selected cfifo index is 2N, its pair is (2N+1). If selected cfifo index is (2N+1), its pair is 2N.
 *                     e.g. if (cfifoIdx == 1 and inSync == true) SSE is set simultaneously for cfifo0 and cfifo1
 * @return void
 */
void EQADC_DRV_SetSingleScanEnBit(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const bool inSync);

/*!
 * @brief Enables Immediate Conversion feature for CFIFO0.
 *
 * This function enables Immediate Conversion feature for CFIFO0.
 * When enabled, the EQADC aborts any current conversions and immediately starts the conversion commands from CFIFO0.
 * The commands which were already executing or pending, will (re)start execution after commands from CFIFO0 have completed.
 * When disabled, the EQADC executes commands in normal priority scheme. When CFIFO0 is triggered, its conversion command can be put
 * behind 2 pending conversion commands in the Cbuffer, commands which might be from lower priority cfifos.
 *
 * @param[in] instance - instance number
 * @param[in] adcIdx - index of the on-chip ADC for which to enable the Immediate Conversion feature
 * @return void
 */
void EQADC_DRV_EnableImmediateConvCmd(const uint32_t instance,
                                      const uint32_t adcIdx);

/*!
 * @brief Disables Immediate Conversion feature for CFIFO0.
 *
 * This function disables Immediate Conversion feature for CFIFO0.
 * When disabled, the EQADC executes commands in normal priority scheme. When CFIFO0 is triggered, its conversion command can be put
 * behind 2 pending conversion commands in the Cbuffer, commands which might be from lower priority cfifos.
 * When enabled, the EQADC aborts any current conversions and immediately starts the conversion commands from CFIFO0.
 * The commands which were already executing or pending, will (re)start execution after commands from CFIFO0 have completed.
 *
 * @param[in] instance - instance number
 * @param[in] adcIdx - index of the on-chip ADC for which to enable the Immediate Conversion feature
 * @return void
 */
void EQADC_DRV_DisableImmediateConvCmd(const uint32_t instance,
                                       const uint32_t adcIdx);

/********** Interrupts and status flags **************/

/*!
 * @brief Returns the selected status information for a fifo (command or result fifos, depending on the status info selected).
 *
 * The function returns the selected status information for a fifo (command or result fifos, depending on the status info selected).
 * The function may return single or multiple bits, depending on the selected status information.
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] statusInfoSel - status info to retrieve
 * @return uint8_t - status bits corresponding to the selected status information
 */
uint8_t EQADC_DRV_GetFifoStatus(const uint32_t instance,
                                const uint32_t fifoIdx,
                                const eqadc_fifo_status_sel_t statusInfoSel);

/*!
 * @brief Clears the status flags corresponding to the bits enabled in the mask input parameter
 *
 * The function clears the status flags corresponding to the bits enabled in the mask input parameter.
 * The selected flags may correspond to both command or result fifos.
 * The 'bitmask' input parameter can be set using EQADC_FIFO_STATUS_FLAG_ defines.
 *
 * NOTE: some flags can only be cleared in certain conditions
 * e.g. CFIFO FILL and RFIFO DRAIN can only be cleared if DMA for selected fifo is disabled.
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] bitmask - select which flags to be clear - to be used with EQADC_FIFO_STATUS_FLAG_ defines
 * @return void
 */
void EQADC_DRV_ClearFifoStatus(const uint32_t instance,
                               const uint32_t fifoIdx,
                               uint32_t bitmask);

/*!
 * @brief Enables one or more interrupt request sources of a fifo (command or result fifo, depending on the interrupt source selected).
 *
 * The function enables one or more interrupt request sources of a fifo (command or result fifo, depending on the interrupt source selected).
 * The function accepts as parameter a bitmask selecting which of the interrupts to be enabled. The bitmask should be set using API defines EQADC_INT_EN_.
 * \n NOTE: RFIFO_DRAIN and CFIFO_FILL interrupts can only be enabled/disabled if DMA support is not configured for draining the rfifo/filling the cfifo.
 * When DMA support is enabled, RFIFO_DRAIN and CFIFO_FILL interrupts are disabled. The DMA can be enabled for each rfifo and cfifo using eqadc_config_t and EQADC_DRV_Init().
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] bitmask - select which interrupts to be enabled
 * @return void
 */
void EQADC_DRV_EnableIntReq(const uint32_t instance,
                            const uint32_t fifoIdx,
                            uint16_t bitmask);

/*!
 * @brief Disables one or more interrupt request sources of a fifo (command or result fifo, depending on the interrupt source selected).
 *
 * The function disables one or more interrupt request sources of a fifo (command or result fifo, depending on the interrupt source selected).
 * The function accepts as parameter a bitmask selecting which of the interrupts to be disabled. The bitmask should be set using API defines EQADC_INT_EN_.
 * \n NOTE: RFIFO_DRAIN and CFIFO_FILL interrupts can only be enabled/disabled if DMA support is not configured for draining rfifos/filling cfifos.
 * When DMA support is enabled, RFIFO_DRAIN and CFIFO_FILL interrupts are disabled. The DMA can be enabled for each rfifo and cfifo using eqadc_config_t and EQADC_DRV_Init().
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] bitmask - select which interrupts to be disabled
 * @return void
 */
void EQADC_DRV_DisableIntReq(const uint32_t instance,
                             const uint32_t fifoIdx,
                             uint16_t bitmask);


/*!
 * @brief Enables one or more events for which a fifo issues DMA requests (command or result fifo, depending on the DMA request selected).
 *
 * The function enables one or more events for which a fifo issues DMA requests (command or result fifo, depending on the DMA request selected).
 * The function accepts as parameter a bitmask selecting which of the requests to be enabled. The bitmask should be set using API defines EQADC_DMA_REQ_EN_.
 * \n NOTE: DMA requests for RFIFO_DRAIN/CFIFO_FILL can only be enabled/disabled if DMA support is configured for draining the rfifo/filling the cfifo.
 * The DMA can be enabled for each rfifo and cfifo using eqadc_config_t and EQADC_DRV_Init().
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] bitmask - select which DMA requests to be enabled
 * @return void
 */
void EQADC_DRV_EnableDmaReq(const uint32_t instance,
                            const uint32_t fifoIdx,
                            uint16_t bitmask);

/*!
 * @brief Disables one or more events for which a fifo issues DMA requests (command or result fifo, depending on the DMA request selected).
 *
 * The function disables one or more events for which a fifo issues DMA requests (command or result fifo, depending on the DMA request selected).
 * The function accepts as parameter a bitmask selecting which of the requests to be disabled. The bitmask should be set using API defines EQADC_DMA_REQ_EN_.
 * \n NOTE: DMA requests for RFIFO_DRAIN/CFIFO_FILL can only be enabled/disabled if DMA support is configured for draining the rfifo/filling the cfifo.
 * The DMA can be enabled for each rfifo and cfifo using eqadc_config_t and EQADC_DRV_Init().
 *
 * @param[in] instance - instance number
 * @param[in] fifoIdx - index of fifo (command or result fifo)
 * @param[in] bitmask - select which DMA requests to be enabled
 * @return void
 */
void EQADC_DRV_DisableDmaReq(const uint32_t instance,
                             const uint32_t fifoIdx,
                             uint16_t bitmask);


/********** Conversion commands **************/

/*!
 * @brief Pushes the conversion command in the selected command fifo (cfifo).
 *
 * The function pushes the conversion command in the selected command fifo (cfifo), by writing the command into CFPRx register.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected command fifo
 * @param[in] convCmd - pointer to the conversion command
 * @return void
 */
void EQADC_DRV_PushCfifoConvCmd(const uint32_t instance,
                                const uint32_t cfifoIdx,
                                const eqadc_conv_cmd_t * const convCmd);

/*!
 * @brief Writes the array of conversion commands from convCmdArray to the memory location pointed by cQueueDest.
 * The command struct members are demapped to the command register bitfields, to be prepared for DMA transfer.
 *
 * This function writes the array of conversion commands from convCmdArray to the memory location pointed by cQueueDest.
 * The command struct members are demapped to the command register bitfields, to be prepared for DMA transfer.
 * This way the cQueueDest is prepared to be directly copied using DMA transfer into the register which loads the command in cfifo (CFPRx).
 * \n NOTE: buffer pointed by cQueueDest must be already allocated by the user before calling the function and must be 32bits aligned.
 * \n NOTE: the minimum size of the buffer pointed by cQueueDest must be (numCmds x 4) bytes
 *
 * This function can be used together with EQADC_DRV_WriteMemConfigCmd() to create hybrid command queues (with both conversion commands and configuration commands).
 * E.g.:
 * uint32_t cQueue[N] allocated and configured for DMA command structure via EQADC_DRV_Config().
 * EQADC_DRV_WriteMemConvCmd(convCmdArray0, cQueue, numConvCmd0);
 * cSubQueueStartPtr = cQueue + numConvCmd0 + 1;
 * EQADC_DRV_WriteMemConfigCmd(configCmdArray0, cSubQueueStartPtr, numConfigCmd0);
 * cSubQueueStartPtr += numConfigCmd0 + 1;
 * EQADC_DRV_WriteMemConfigCmd(convCmdArray1, cSubQueueStartPtr, numConvCmd1);
 *
 * @param[in] convCmdArray - pointer to an array of conversion commands
 * @param[in] cQueueDest - pointer to the destination memory location
 * @param[in] numCmds - number of commands available in convCmdArray, and number of uint32_t words which will be written in cQueueDest
 * @return void
 */
void EQADC_DRV_WriteMemConvCmd(const eqadc_conv_cmd_t * const convCmdArray,
                               uint32_t * const cQueueDest,
                               const uint8_t numCmds);


/*!
 * @brief Pushes the configuration command in the selected command fifo (cfifo).
 *
 * The function pushes the configuration command in the selected command fifo (cfifo), by writing the command into CFPRx register.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected command fifo
 * @param[in] configCmd - pointer to the configuration command
 * @return void
 */
void EQADC_DRV_PushCfifoConfigCmd(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const eqadc_config_cmd_t * const configCmd);


/*!
 * @brief Writes the array of configuration commands from configCmdArray to the memory location pointed by cQueueDest.
 * The command struct members are demapped to the command register bitfields, to be prepared for DMA transfer.
 *
 * This function writes the array of configuration commands from configCmdArray to the memory location pointed by cQueueDest.
 * The command struct members are demapped to the command register bitfields, to be prepared for DMA transfer.
 * This way the cQueueDest is prepared to be directly copied using DMA transfer into the register which loads the command in cfifo (CFPRx).
 * \n NOTE: buffer pointed by cQueueDest must be already allocated by the user before calling the function and must be 32bits aligned.
 * \n NOTE: the minimum size of the buffer pointed by cQueueDest must be (numCmds x 4) bytes
 *
 * This function can be used together with EQADC_DRV_WriteMemConvCmd() to create hybrid command queues (with both conversion commands and configuration commands).
 * E.g.:
 * uint32_t cQueue[N] allocated and configured for DMA command structure via EQADC_DRV_Config().
 * EQADC_DRV_WriteMemConvCmd(convCmdArray0, cQueue, numConvCmd0);
 * cSubQueueStartPtr = cQueue + numConvCmd0 + 1;
 * EQADC_DRV_WriteMemConfigCmd(configCmdArray0, cSubQueueStartPtr, numConfigCmd0);
 * cSubQueueStartPtr += numConfigCmd0 + 1;
 * EQADC_DRV_WriteMemConfigCmd(convCmdArray1, cSubQueueStartPtr, numConvCmd1);
 *
 * @param[in] configCmdArray - pointer to an array of configuration commands
 * @param[in] cQueueDest - pointer to the destination memory location
 * @param[in] numCmds - number of commands available in configCmdArray, and number of uint32_t words which will be written in cQueueDest
 * @return void
 */
void EQADC_DRV_WriteMemConfigCmd(const eqadc_config_cmd_t * const configCmdArray,
                                 uint32_t * const cQueueDest,
                                 const uint8_t numCmds);


/*!
 * @brief Pops one result from the selected result fifo (rfifo).
 *
 * The function pops one result from the selected result fifo (rfifo), by reading the corresponding RFPRn register.
 * When RFIFOx is not empty, returns the next unread entry value of RFIFOx and causes the RFCTRx field to be decremented by one in (EQADC_FISRn).
 * When the RFIFOx is empty, returns undefined data value and does not decrement the RFCTRx value.
 *
 * @param[in] instance - instance number
 * @param[in] rfifoIdx - index of the selected result fifo
 * @return uint16_t - If rfifo is not empty, returns the next unread entry of rfifo. If rfifo is empty, returns undefined data.
 */
uint16_t EQADC_DRV_PopRfifoData(const uint32_t instance,
                                const uint32_t rfifoIdx);

/*!
 * @brief Returns the value of the selected command fifo entry (cfifo).
 *
 * The function returns the value of a selected command fifo entry (cfifo), from corresponding CFxRn and CF0Rn registers.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected command fifo
 * @param[in] entryIdx - index of the entry in the command fifo
 * @return uint32_t - value of the selected cfifo entry
 */
uint32_t EQADC_DRV_GetCfifoEntry(const uint32_t instance,
                                 const uint32_t cfifoIdx,
                                 const uint32_t entryIdx);

/*!
 * @brief Returns the value of the selected result fifo entry (rfifo).
 *
 * The function returns the value of a selected result fifo entry (rfifo), from corresponding RFxRn register.
 * Calling this function does not pop the result from the RFIFO.
 *
 * @param[in] instance - instance number
 * @param[in] rfifoIdx - index of the selected result fifo
 * @param[in] entryIdx - index of the entry in the result fifo
 * @return uint16_t - value of the selected rfifo entry
 */
uint16_t EQADC_DRV_GetRfifoEntry(const uint32_t instance,
                                 const uint32_t rfifoIdx,
                                 const uint32_t entryIdx);


/********** Other functionalities **************/

/*!
 * @brief Returns the number of commands which have been completely transferred from the selected cfifo.
 *
 * The function returns the number of commands which have been completely transfered from the selected cfifo, from CFTCR TC_CF.
 * The transfer of entries bound for the on-chip ADCs is considered completed when they are stored in the appropriate CBuffer.
 * The EQADC increments the transfer counter value by one after a command is transferred.
 * The counter resets to zero after EQADC completes transferring a command with an asserted EOQ bit.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected command fifo
 * @return uint16_t - number of transfered commands
 */
uint16_t EQADC_DRV_GetTransferCount(const uint32_t instance,
                                    const uint32_t cfifoIdx);

/*!
 * @brief Sets the transfer counter register (CFTCR TC_CF).
 *
 * The function sets the value for transfer counter register (CFTCR TC_CF).
 * The transfer of entries bound for the on-chip ADCs is considered completed when they are stored in the appropriate CBuffer.
 * The EQADC increments the transfer counter value by one after a command is transferred.
 * The counter resets to zero after EQADC completes transferring a command with an asserted EOQ bit.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the selected command fifo
 * @param[in] value - new value for the transfer counter register
 * @return void
 */
void EQADC_DRV_SetTransferCount(const uint32_t instance,
                                const uint32_t cfifoIdx,
                                const uint16_t value);

/*!
 * @brief Returns the current status of the selected cfifo.
 *
 * The function returns the current status of the selected cfifo, from CFSR register.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the command fifo
 * @return eqadc_cfifo_status_t - status of the selected cfifo
 */
eqadc_cfifo_status_t EQADC_DRV_GetCfifoStatusCurrent(const uint32_t instance,
                                                     const uint32_t cfifoIdx);


/*!
 * @brief Returns the cfifo status of previously completed command transfer from a cfifo to a CBuffer (on-chip ADC).
 *
 * The function returns the cfifo status of previously completed command transfer from a cfifo to a CBuffer (on-chip ADC), from CFSSR register.
 * The register values are captured at the beginning of a command transfer to the corresponding CBuffer
 * (each on-chip ADC has a single internal CBuffer).
 * The status is captured at the time a current command transfer to CBuffer is initiated (CFSSR TCB).
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the command fifo source
 * @param[in] adcIdx - index of the on-chip ADC (CBuffer) to which the command has been transfered
 * @return eqadc_cfifo_status_t - cfifo status of previously completed command transfer from a cfifo to a CBuffer
 */
eqadc_cfifo_status_t EQADC_DRV_GetCfifoStatusSnapshot(const uint32_t instance,
                                                      const uint32_t cfifoIdx,
                                                      const uint32_t adcIdx);


/*!
 * @brief Returns the index of the cfifo from which the last command was transfered to the selected CBuffer (on-chip ADC).
 *
 * The function returns the index of the cfifo from which the last command was transfered to the selected CBuffer (on-chip ADC), from CFSSR LCFTCB.
 * (each on-chip ADC has a single internal CBuffer).
 *
 * @param[in] instance - instance number
 * @param[in] adcIdx - index of the on-chip ADC (CBuffer)
 * @return uint8_t - index of the cfifo from which the last command was transfered
 */
uint8_t EQADC_DRV_GetCfifoLastCmdSource(const uint32_t instance,
                                        const uint32_t adcIdx);


/*!
 * @brief Returns the value of Transfer Counter for the last cfifo transfer to the selected CBuffer (on-chip ADC).
 *
 * The function returns the value of Transfer Counter for the last cfifo transfer to the selected CBuffer (on-chip ADC), from CFSSR TC_LCFTCB.
 * (each on-chip ADC has a single internal CBuffer).
 *
 * @param[in] instance - instance number
 * @param[in] adcIdx - index of the on-chip ADC (CBuffer)
 * @return uint8_t - index of the cfifo from which the last command was transfered
 */
uint16_t EQADC_DRV_GetCfifoTransferCountLastCmd(const uint32_t instance,
                                                const uint32_t adcIdx);


/*!
 * @brief Sets the operating mode for the selected CFIFO
 *
 * If the selected CFIFO mode (cfifoOpMode) is different than DISABLED, and already configured
 * CFIFO mode is not DISABLED or CFIFO status is different than IDLE, the function returns STATUS_ERROR.
 * Otherwise the function updates the cfifo operating mode and returns STATUS_SUCCESS.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the command fifo
 * @param[in] cfifoOpMode - operating mode to be configured
 * @return status_t
 * STATUS_SUCCESS - cfifo operating mode updated successfully
 * STATUS_ERROR - cfifo operating mode was not updated
 */
status_t EQADC_DRV_SetCfifoOpMode(const uint32_t instance,
                                  const uint32_t cfifoIdx,
                                  const eqadc_cfifo_mode_t cfifoOpMode);


/*!
 * @brief Get the operating mode for the selected CFIFO
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the command fifo
 * @return eqadc_cfifo_mode_t operating mode
 */
eqadc_cfifo_mode_t EQADC_DRV_GetCfifoOpMode(const uint32_t instance,
                                            const uint32_t cfifoIdx);


/*!
 * @brief Invalidates all entries in the selected CFIFO, by setting CFINVx
 *
 * If current CFIFO mode is not DISABLED or CFIFO status is different than IDLE,
 * the function returns STATUS_ERROR.
 * Otherwise the function invalidates the cfifo and returns STATUS_SUCCESS.
 * For all effects of setting CFINVx please refer to Reference Manual.
 *
 * @param[in] instance - instance number
 * @param[in] cfifoIdx - index of the command fifo
 * @return status_t
 * STATUS_SUCCESS - cfifo invalidated successfully
 * STATUS_ERROR - cfifo has not been invalidated
 */
status_t EQADC_DRV_InvalidateCfifo(const uint32_t instance,
                                   const uint32_t cfifoIdx);


/*!
 * @brief Flushes results in the selected RFIFO
 *
 * Returns STATUS_ERROR if selected RFIFO is configured for draining results with DMA
 * or Drain Event (either DMA or Interrupt) is enabled for the selected RFIFO.
 * Returns STATUS_TIMEOUT if the selected RFIFO is not empty in the provided timeout interval
 * Important note: the function does not check if there are any on-going conversions which might
 * complete after the function has ended. In this case, results might end up in the rfifo after
 * the function has completed successfully. It is the user's responsibility to make sure that
 * no new conversion results are routed to the rfifo.
 *
 * @param[in] instance - instance number
 * @param[in] rfifoIdx - index of the result fifo
 * @param[in] timeout - timeout in milliseconds
 * @return status_t
 * STATUS_SUCCESS - rfifo flushed successfully
 * STATUS_ERROR - rfifo has not been flushed because preconditions are not met
 * STATUS_TIMEOUT - rfifo has not been flushed completely because timeout occurred
 */
status_t EQADC_DRV_FlushRfifo(const uint32_t instance,
                              const uint32_t rfifoIdx,
                              const uint32_t timeout);


#if defined (__cplusplus)
}
#endif

/*! @}*/

#endif /* EQADC_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
