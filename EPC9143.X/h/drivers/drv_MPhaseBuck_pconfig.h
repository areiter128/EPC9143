/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DRV_MULTIPHASE_BUCK_CONVERTER_CONFIG_H
#define	DRV_MULTIPHASE_BUCK_CONVERTER_CONFIG_H

#include <xc.h> // include processor files - each processor file is guarded.  

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/* MULTIPHASE DECLARATIONS 
 * *********************************************************************************************/
#include "globals.h"
    
#define MPHBUCK_NO_OF_PHASES    BUCK_NO_OF_PHASES

/* *********************************************************************************************/

/*!MPHBUCK STARTUP SETTINGS
 * ***************************************************************************************************
 * Summary:
 * Power Converter Startup Settings
 * 
 * Description:
 * Startup Timings defined in syscfg_startup.h are used to calculate the number of scheduler
 * ticks to set up the software timer inside the state machine. These equations are assuming
 * that the state machine is on the same frequency as the scheduler base call clock. 
 * As this is not the case when more than one task is executed in parallel, a runtime
 * function is provided inside the state machine adjusting the given period.
 * 
 * *************************************************************************************************** */


#if defined (__P33SMPS_EP__)
    
    // The PwM module is configured in master time base mode
    
    #define REG_PTCON   0x0000
    
    typedef struct {
        volatile PTCONBITS regPTCON;     // PTCON: PWM TIME BASE CONTROL REGISTER
        volatile uint16_t regPTCON2;    // PTCON2: PWM CLOCK DIVIDER SELECT REGISTER
        volatile uint16_t regPTPER;     // PTPER: PWM PRIMARY MASTER TIME BASE PERIOD REGISTER
        volatile uint16_t regSEVTCMP;   // SEVTCMP: PWM SPECIAL EVENT COMPARE REGISTER
        volatile uint16_t res1;         // (not used memory region)
        volatile uint16_t regMDC;       // MDC: PWM MASTER DUTY CYCLE REGISTER
        volatile uint16_t res2;         // (not used memory region)
        volatile uint16_t regSTCON;     // STCON: PWM SECONDARY MASTER TIME BASE CONTROL REGISTER
        volatile uint16_t regSTCON2;    // STCON2: PWM SECONDARY CLOCK DIVIDER SELECT REGISTER
        volatile uint16_t regSTPER;     // STPER: PWM SECONDARY MASTER TIME BASE PERIOD REGISTER
        volatile uint16_t regSSEVTCMP;  // SSEVTCMP: PWM SECONDARY SPECIAL EVENT COMPARE REGISTER
        volatile uint16_t res3;         // (not used memory region)
        volatile uint16_t res4;         // (not used memory region)
        volatile uint16_t regCHOP;      // CHOP: PWM CHOP CLOCK GENERATOR REGISTER
        volatile uint16_t res5;         // (not used memory region)
        volatile uint16_t regPWMKEY;    // PWMKEY: PWM PROTECTION LOCK/UNLOCK KEY REGISTER
    } __attribute__((packed)) PWM_MODULE_REGISTER_CONFIG_t; // PWM Module Register Set

/* PWM Register Set Address Offset */
#define PWM_REGISTER_OFFSET     (uint16_t)((uint16_t*)&PWMCON2 - (uint16_t*)&PWMCON1)
    
/*  PWMCONx: PWMx CONTROL REGISTER (x = 1 to 8)
 
    PWMCON1bits.FLTSTAT= 0; // Bit 15: Fault Interrupt Status bit
    PWMCON1bits.CLSTAT = 0; // Bit 14: Current Limit Interrupt Status bit
    PWMCON1bits.TRGSTAT= 0; // Bit 13: Trigger Interrupt Status bit (read only)
    PWMCON1bits.FLTIEN = 0; // Bit 12: Fault Interrupt Enable bit (disabled)
    PWMCON1bits.CLIEN  = 0; // Bit 11: Current Limit Interrupt Enable bit (disabled)
    PWMCON1bits.TRGIEN = 0; // Bit 10: Trigger Interrupt disabled
    PWMCON1bits.ITB    = 0; // Bit 9:  PTPER register provides timing for this PWMx generator
    PWMCON1bits.MDCS   = 0; // Bit 8:  PDCx and SDCx registers provide duty cycle information for this PWMx generator
    PWMCON1bits.DTC    = 0b00; // Bit [7:6]:  Positive dead time is actively applied for all Output modes
                            // Bit [5:4]: (unimplemented)
    PWMCON1bits.MTBS   = 0; // Bit 3:  PWMx generator uses the primary master time base for synchronization and 
                            //         the clock source for the PWMx generation logic
    PWMCON1bits.CAM    = 0; // Bit 2:  Edge-Aligned mode is enabled
    PWMCON1bits.XPRES  = 0; // Bit 1:  External pins do not affect the PWMx time base
    PWMCON1bits.IUE    = 0; // Bit 0:  Updates to the active Duty Cycle, Phase Offset, Dead-Time and local 
                            //         Time Base Period registers are synchronized to the local PWMx time base
*/
    
#define PWMCONx_CONFIG  0b0000000000000000
    
/*  IOCONx: PWMx I/O CONTROL REGISTER (x = 1 to 8)

    IOCONxbits.PENH = 0;    // Bit 15: GPIO module controls the PWMxH pin => Control is handed over to PWM module when enabled
    IOCONxbits.PENL = 0;    // Bit 14: GPIO module controls the PWMxL pin => Control is handed over to PWM module when enabled
    IOCONxbits.POLH = 0;    // Bit 13: PWMxH pin is active-high
    IOCONxbits.POLL = 0;    // Bit 12: PWMxL pin is active-high

    IOCONxbits.PMOD = 0b00; // Bit [11:10]: PWMx I/O pin pair is in the Complementary Output mode

    IOCONxbits.OVRENH = 1;  // Bit 9: OVRDAT1 provides data for output on the PWMxH pin
    IOCONxbits.OVRENL = 1;  // Bit 8: OVRDAT0 provides data for output on the PWMxL pin
    IOCONxbits.OVRDAT = 0b00; // Bit [7:6]: Software Override Pin States: PWMxH = LOW / PWMxL = LOW

    IOCONxbits.FLTDAT = 0b00; // Bit [5:4]: FAULT Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCONxbits.CLDAT = 0b00;  // Bit [3:2]: Current Limit Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCONxbits.SWAP = 0;    // Bit 1: PWMxH and PWMxL pins are mapped to their respective pins
    IOCONxbits.OSYNC = 1;   // Bit 0: Output overrides via the OVRDAT<1:0> bits are synchronized to the PWMx time base
*/
    
#define IOCONx_CONFIG   0b0000001100000001
#define PWM_OVR_HOLD    0b0000001100000000
#define PWM_PEN_ENABLE  0b1100000000000000
    
/*  FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER (x = 1 to 8)

    FCLCONxbits.IFLTMOD = 0;    // Bit 15: Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the 
                                //         PWMxH and PWMxL outputs; the PWM Fault mode maps FLTDAT<1:0> to the 
                                //         PWMxH and PWMxL outputs
    FCLCONxbits.CLSRC = 0b11111; // Bit [14:10]: Current-Limit Control Signal Source Selection: FLT31
    FCLCONxbits.CLPOL = 0b0;    // Bit 9: The selected current-limit source is active-high
    FCLCONxbits.CLMOD = 0;      // Bit 8: Current-Limit mode is disabled
    FCLCONxbits.FLTSRC = 0b11111; // Bit [7:3]: Fault Control Signal Source Selection: Fault 31 (Default)
    FCLCONxbits.FLTPOL = 0b0;   // Bit 2: The selected Fault source is active-high
    FCLCONxbits.FLTMOD = 0b11;  // Bit [1:0]: Fault input is disabled
 */    
#define FCLCONx_CONFIG  0b0111110011111011
    
/* TRGCONx: PWMx TRIGGER CONTROL REGISTER (x = 1 to 8)
   
    TRGCONxbits.TRGDIV  = 0b0000;   // Bit [15:12]: Trigger output for every trigger event
                                    // Bit [11:8]: (unimplemented)
    TRGCONxbits.DTM     = 0;        // Bit 7: Secondary trigger event is not combined with the primary trigger event to 
                                    //        create a PWM trigger; two separate PWM triggers are generated
                                    // Bit 6: (unimplemented)
    TRGCONxbits.TRGSTRT = 0b000000; // Bit [5:0]: Wait 0 PWM cycles before generating the first trigger event after 
                                    //            the module is enabled
 */    
#define TRGCONx_CONFIG  0b0000000000000000

/*  LEBCONx: PWMx LEADING-EDGE BLANKING (LEB) CONTROL REGISTER (x = 1 to 8)

    LEBCONxbits.PHR = 1;    // Bit 15: Rising edge of PWMxH will trigger the Leading-Edge Blanking counter
    LEBCONxbits.PHF = 0;    // Bit 14: Leading-Edge Blanking ignores the falling edge of PWMxH
    LEBCONxbits.PLR = 0;    // Bit 13: Leading-Edge Blanking ignores the rising edge of PWMxL
    LEBCONxbits.PLF = 0;    // Bit 12: Leading-Edge Blanking ignores the falling edge of PWMxL
    LEBCONxbits.FLTLEBEN = 0; //Bit 11:  Leading-Edge Blanking is not applied to the selected Fault input
    LEBCONxbits.CLLEBEN = 0; // Bit 10: Leading-Edge Blanking is not applied to the selected current-limit input
                            // Bit [9:6]: (unimplemented)
    LEBCONxbits.BCH = 0;    // Bit 5: No blanking when the selected blanking signal is high
    LEBCONxbits.BCL = 0;    // Bit 4: No blanking when the selected blanking signal is low
    LEBCONxbits.BPHH = 0;   // Bit 3: No blanking when the PWMxH output is high
    LEBCONxbits.BPHL = 0;   // Bit 2: No blanking when the PWMxH output is low
    LEBCONxbits.BPLH = 0;   // Bit 1: No blanking when the PWMxL output is high
    LEBCONxbits.BPLL = 0;   // Bit 0: No blanking when the PWMxL output is low
*/    
#define LEBCONx_CONFIG  0b1000000000000000
    
/*  AUXCONx: PWMx AUXILIARY CONTROL REGISTER (x = 1 to 8)

    AUXCONxbits.HRPDIS   = 0;       // Bit 15: High-resolution PWMx period is enabled
    AUXCONxbits.HRDDIS   = 0;       // Bit 14: High-resolution PWMx duty cycle is enabled
                                    // Bit [13:12]: (unimplemented)
    AUXCONxbits.BLANKSEL = 0b0000;  // Bit [11:8]: No state blanking
                                    // Bit [7:6]: (unimplemented)
    AUXCONxbits.CHOPSEL  = 0b0000;  // Bit [5:2]: Chop clock generator is selected as the chop clock source
    AUXCONxbits.CHOPHEN  = 0;       // Bit 1: PWMxH chopping function is disabled
    AUXCONxbits.CHOPLEN  = 0;       // Bit 0: PWMxL chopping function is disabled
*/    
#define AUXCONx_CONFIG  0b0000000000000000
    
/*!PWM_CHANNEL_REGISTER_CONFIG_t
 * ***************************************************************************************************
 * Summary:
 * Declaration of a complete PWM channel register set
 * 
 * Description:
 * This data structure is used to set the configuration of any PWM channel, which can 
 * then be written to a specific PWM channel instance. In case of a multiphase converter,
 * all PWM channels required to drive the power converter will be set up using the same 
 * configuration. Thus, by using a defined set up for one channel can be applied to all
 * channels. Specific, user defined differences, such as phase shifts, are taken from the 
 * MPHBUCK_POWER_CONTROLLER_t power converter control object handed over as function parameter.
 * 
 * *************************************************************************************************** */
    
typedef struct {
    volatile uint16_t regPWMCON;    // PWMCONx: PWMx CONTROL REGISTER
    volatile uint16_t regIOCON;     // IOCONx: PWMx I/O CONTROL REGISTER
    volatile uint16_t regFCLCON;    // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER
    volatile uint16_t regPDC;       // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    volatile uint16_t regPHASE;     // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER
    volatile uint16_t regDTR;       // DTRx: PWMx DEAD-TIME REGISTER
    volatile uint16_t regALTDTR;    // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER
    volatile uint16_t regSDC;       // SDCx: PWMx SECONDARY DUTY CYCLE REGISTER
    volatile uint16_t regSPHASE;    // SPHASEx: PWMx SECONDARY PHASE-SHIFT REGISTER
    volatile uint16_t regTRIG;      // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER
    volatile uint16_t regTRGCON;    // TRGCONx: PWMx TRIGGER CONTROL REGISTER
    volatile uint16_t regSTRIG;     // STRIGx: PWMx SECONDARY TRIGGER COMPARE VALUE REGISTER
    volatile uint16_t regPWMCAP;    // PWMCAPx: PWMx PRIMARY TIME BASE CAPTURE REGISTER
    volatile uint16_t regLEBCON;    // LEBCONx: PWMx LEADING-EDGE BLANKING (LEB) CONTROL REGISTER
    volatile uint16_t regLEBDLY;    // LEBDLYx: PWMx LEADING-EDGE BLANKING DELAY REGISTER
    volatile uint16_t regAUXCON;    // AUXCONx: PWMx AUXILIARY CONTROL REGISTER
} __attribute__((packed)) PWM_CHANNEL_REGISTER_CONFIG_t; // PWM Channel Register Set

// ADC specific declarations
#define ADC_CORE_MASK           0b0000000010001111
#define ADC_CORE0_MASK_INDEX    0b0000000000000001
#define ADC_CORE1_MASK_INDEX    0b0000000000000010
#define ADC_CORE2_MASK_INDEX    0b0000000000000100
#define ADC_CORE3_MASK_INDEX    0b0000000000001000
#define ADC_SHRCORE_MASK_INDEX  0b0000000010000000

#define ADC_CORE_CAL_RUN        0b00000001
#define ADC_CORE_CAL_EN         0b00000010
#define ADC_CORE_CAL_DIFF       0b00000100
#define ADC_CORE_CAL_RDY        0b10000000

#elif defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)
    
    typedef struct {
        
    } PWM_REGISTER_CONFIG_t;
    
#endif

    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* DRV_MULTIPHASE_BUCK_CONVERTER_CONFIG_H */

