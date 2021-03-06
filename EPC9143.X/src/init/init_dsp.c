/*
 * File:   init_dsp.c
 * Author: M91406
 *
 * Created on October 16, 2018, 12:00 PM
 */


#include <xc.h>
#include "mcal/p33SMPS_dsp.h"

/* === Initialize CPU ===================================================== */
volatile uint16_t DSP_Initialize(void) {

    // This routine initializes the default configuration of the 
    // DSP core for performing SMPS control loop libaries.
    // If any other section of the firmware needs to use the DSP 
    // with different settings, this code module would have to 
    // make sure the core configuraiton is reset to these defaults
    // when the SMPS control library is executed.
    //
    // Please note: 
    // The configuration of the control library allows to add its own 
    // core configuration save & restore cycle in the control code.
    // Please enable this function in the control loop configuration
    // if conflicts between different DSP configurations cannot be 
    // resolved differently within the firmware.
    
    CORCONbits.ACCSAT = CORCON_ACCSAT_931;  // Accumulator Saturation Mode Selection: 9.31 saturation (super saturation)
    CORCONbits.IF = CORCON_IF_FRACTIONAL;   // Integer or Fractional Multiplier Mode Selection: Fractional mode is enabled for DSP multiply
    CORCONbits.RND = CORCON_RND_UNBIASED;   // Rounding Mode Selection: Unbiased (convergent) rounding is enabled
    CORCONbits.SATA = CORCON_SATA_ON;       // ACCA Saturation Enable: Accumulator A saturation is enabled
    CORCONbits.SATB = CORCON_SATB_ON;       // ACCB Saturation Enable: Accumulator B saturation is enabled
    CORCONbits.SATDW = CORCON_SATDW_ON;     // Data Space Write from DSP Engine Saturation Enable: Data Space write saturation is enabled
    CORCONbits.US = CORCON_US_SIGNED;       // DSP Multiply Unsigned/Signed Control: DSP engine multiplies are signed
    CORCONbits.VAR = CORCON_VAR_FIXED;      // Variable Exception Processing Latency Control: Fixed exception processing is enabled

//    CORCONbits.DL = CORCON_DL_0;              // (read only)
//    CORCONbits.EDT = CORCON_EDT_RUN;          // control bit => do not set during configuration
//    CORCONbits.SFA = CORCON_SFA_ACTIVE;       // (read only)
//    CORCONbits.IPL3 = CORCON_IPL3_STAT_LT7;   // (read only)
    
    Nop();
    Nop();
    Nop();
    
    return(1);
}

