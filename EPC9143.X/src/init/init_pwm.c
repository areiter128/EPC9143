
#include "init/init_pwm.h"

// Private Function Call Prototypes
volatile uint16_t PWM_ModuleInitialize(void);
volatile uint16_t PWM1_Initialize(void);
volatile uint16_t PWM2_Initialize(void);


volatile uint16_t ConfigurePWM(void) {
    
    volatile uint16_t fres=1;
    
    fres &= PWM_ModuleInitialize();
    fres &= PWM1_Initialize();
    fres &= PWM2_Initialize();
    
    return(fres);
}

/* *********************************************************************************
 * PRIVATE FUNCTIONS
 * ********************************************************************************/

volatile uint16_t PWM_ModuleInitialize(void) {

    // Initialize PWM module
    
    volatile uint16_t fres=1;
    
    
    // PTCON: PWM PRIMARY TIME BASE CONTROL REGISTER
    PTCONbits.PTEN = 0;     // Disable PWM module while being configured
    PTCONbits.PTSIDL = 0;   // PWM time base runs in CPU idle mode
    PTCONbits.SESTAT = 0;   // Clear Special Event Status Bit
    PTCONbits.SEIEN = 0;    // Disable Special Event Interrupt
    PTCONbits.EIPU = 0;     // Disable Immediate Period Update
    PTCONbits.SYNCPOL = 0;  // SYNCIx/SYNCO1 is active-high
    PTCONbits.SYNCOEN = 0;  // SYNCO1 output is disabled
    PTCONbits.SYNCEN = 0;   // External synchronization of primary time base is disabled
    PTCONbits.SYNCSRC = 0b000;  // Synchronization Source Selection: SYNCI1
    PTCONbits.SEVTPS = 0b0000;  // PWM Special Event Trigger Output Post-Scaler Selection: 1:1
                                //  => post-scaler generates a Special Event Trigger on every compare match event
    
    // PTCON2: PWM CLOCK DIVIDER SELECT REGISTER
    PTCON2bits.PCLKDIV = 0b000; // Divide-by-1, maximum PWM timing resolution (power-on default)

    // PWM SECONDARY MASTER TIME BASE CONTROL REGISTER
    STCONbits.SESTAT = 0;   // Clear Special Event Status Bit
    STCONbits.SEIEN = 0;    // Disable Special Event Interrupt
    STCONbits.EIPU = 0;     // Disable Immediate Period Update
    STCONbits.SYNCPOL = 0;  // SYNCIx/SYNCO1 is active-high
    STCONbits.SYNCOEN = 0;  // SYNCO1 output is disabled
    STCONbits.SYNCEN = 0;   // External synchronization of primary time base is disabled
    STCONbits.SYNCSRC = 0b000;  // Synchronization Source Selection: SYNCI1
    STCONbits.SEVTPS = 0b0000;  // PWM Special Event Trigger Output Post-Scaler Selection: 1:1
                                //  => post-scaler generates a Special Event Trigger on every compare match event

    // STCON2: PWM SECONDARY CLOCK DIVIDER SELECT REGISTER
    STCON2bits.PCLKDIV = 0b000; // Divide-by-1, maximum PWM timing resolution (power-on default)
    
    PTPER = PWM_PERIOD; // PWM PRIMARY MASTER TIME BASE PERIOD REGISTER
    SEVTCMP = 0;        // PWM SPECIAL EVENT COMPARE REGISTER

    STPER = 0;          // PWM SECONDARY MASTER TIME BASE PERIOD REGISTER
    SSEVTCMP = 0;       // PWM SECONDARY SPECIAL EVENT COMPARE REGISTER
    
    // CHOP: PWM CHOP CLOCK GENERATOR REGISTER
    //    Value is in 8.32 ns increments. The frequency of the chop clock signal is given by:
    //    Chop Frequency = 1/(16.64 * (CHOP<7:3> + 1) * Primary Master PWM Input Clock Period)    
    CHOPbits.CHPCLKEN = 0; // Disable Chop Clock Generator
    CHOPbits.CHOPCLK = 0;  // Chop Clock Divider bits
    
    MDC = 0; // PWM MASTER DUTY CYCLE REGISTER (not used, PDCx registers provide individual duty cycle)
//    PWMKEY = 0; // PWM PROTECTION LOCK/UNLOCK KEY REGISTER

    
    return (fres);
}

volatile uint16_t PWM1_Initialize(void) {
    
    volatile uint16_t fres=1;
    
    // PWMCONx: PWMx CONTROL REGISTER (x = 1 to 8)
    PWMCON1bits.FLTIEN = 0; // Fault Interrupt disabled
    PWMCON1bits.CLIEN = 0; // Current Limit Interrupt disabled
    PWMCON1bits.TRGIEN = 0; // Trigger Interrupt disabled
    PWMCON1bits.ITB = 0; // PTPER register provides timing for this PWMx generator
    PWMCON1bits.MDCS = 0; // PDCx and SDCx registers provide duty cycle information for this PWMx generator
    PWMCON1bits.DTC = 0b00; // Positive dead time is actively applied for all Output modes
    PWMCON1bits.MTBS = 0; // PWMx generator uses the primary master time base for synchronization and 
                          // the clock source for the PWMx generation logic
    PWMCON1bits.CAM = 0; // Edge-Aligned mode is enabled
    PWMCON1bits.XPRES = 0; // External pins do not affect the PWMx time base
    PWMCON1bits.IUE = 0; // Updates to the active Duty Cycle, Phase Offset, Dead-Time and local 
                         // Time Base Period registers are synchronized to the local PWMx time base
    
    
    // IOCONx: PWMx I/O CONTROL REGISTER (x = 1 to 8)
    IOCON1bits.PMOD = 0b00; // PWMx I/O pin pair is in the Complementary Output mode

    IOCON1bits.PENH = 0; // GPIO module controls the PWMxH pin => Control is handed over to PWM module when enabled
    IOCON1bits.PENL = 0; // GPIO module controls the PWMxL pin => Control is handed over to PWM module when enabled
    IOCON1bits.POLH = 0; // PWMxH pin is active-high
    IOCON1bits.POLL = 0; // PWMxL pin is active-high

    IOCON1bits.OVRDAT = 0b00; // Software Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON1bits.OVRENH = 1; // OVRDAT1 provides data for output on the PWMxH pin
    IOCON1bits.OVRENL = 1; // OVRDAT0 provides data for output on the PWMxL pin
    IOCON1bits.OSYNC = 1; // Output overrides via the OVRDAT<1:0> bits are synchronized to the PWMx time base

    IOCON1bits.FLTDAT = 0b00; // FAULT Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON1bits.CLDAT = 0b00; // Current Limit Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON1bits.SWAP = 0; // PWMxH and PWMxL pins are mapped to their respective pins
    
    // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER (x = 1 to 8)
    FCLCON1bits.IFLTMOD = 0; // Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the 
                             // PWMxH and PWMxL outputs; the PWM Fault mode maps FLTDAT<1:0> to the 
                             // PWMxH and PWMxL outputs
    FCLCON1bits.CLSRC = 0b11111; // Current-Limit Control Signal Source Selection: FLT31
    FCLCON1bits.CLPOL = 0b0; // The selected current-limit source is active-high
    FCLCON1bits.CLMOD = 0; // Current-Limit mode is disabled
    FCLCON1bits.FLTSRC = 0b11111; // Fault Control Signal Source Selection: Fault 31 (Default)
    FCLCON1bits.FLTPOL = 0b0; // The selected Fault source is active-high
    FCLCON1bits.FLTMOD = 0b11; // Fault input is disabled
    
    // LEBCONx: PWMx LEADING-EDGE BLANKING (LEB) CONTROL REGISTER (x = 1 to 8)
    LEBCON1bits.PHR = 1; // Rising edge of PWMxH will trigger the Leading-Edge Blanking counter
    LEBCON1bits.PHF = 0; // Leading-Edge Blanking ignores the falling edge of PWMxH
    LEBCON1bits.PLR = 0; // Leading-Edge Blanking ignores the rising edge of PWMxL
    LEBCON1bits.PLF = 0; // Leading-Edge Blanking ignores the falling edge of PWMxL
    LEBCON1bits.FLTLEBEN = 0; // Leading-Edge Blanking is not applied to the selected Fault input
    LEBCON1bits.CLLEBEN = 0; // Leading-Edge Blanking is not applied to the selected current-limit input
    LEBCON1bits.BCH = 0; // No blanking when the selected blanking signal is high
    LEBCON1bits.BCL = 0; // No blanking when the selected blanking signal is low
    LEBCON1bits.BPHH = 0; // No blanking when the PWMxH output is high
    LEBCON1bits.BPHL = 0; // No blanking when the PWMxH output is low
    LEBCON1bits.BPLH = 0; // No blanking when the PWMxL output is high
    LEBCON1bits.BPLL = 0; // No blanking when the PWMxL output is low

    // TRGCONx: PWMx TRIGGER CONTROL REGISTER (x = 1 to 8)
    TRGCON1bits.TRGDIV = 0b0000; // Trigger output for every trigger event
    TRGCON1bits.DTM = 0; // Secondary trigger event is not combined with the primary trigger event to 
                         // create a PWM trigger; two separate PWM triggers are generated
    TRGCON1bits.TRGSTRT = 0b000000; // Wait 0 PWM cycles before generating the first trigger event after 
                                    // the module is enabled
    
    // AUXCONx: PWMx AUXILIARY CONTROL REGISTER (x = 1 to 8)
    AUXCON1bits.HRPDIS = 0; // High-resolution PWMx period is enabled
    AUXCON1bits.HRDDIS = 0; // High-resolution PWMx duty cycle is enabled
    AUXCON1bits.BLANKSEL = 0b0000; // No state blanking
    AUXCON1bits.CHOPSEL = 0b0000; // Chop clock generator is selected as the chop clock source
    AUXCON1bits.CHOPHEN = 0; // PWMxH chopping function is disabled
    AUXCON1bits.CHOPLEN = 0; // PWMxL chopping function is disabled
    
    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER (x = 1 to 8)
    PDC1 = PWM_LEB_DELAY; // Set initial primary duty ratio 
    
    // SDCx: PWMx SECONDARY DUTY CYCLE REGISTER (x = 1 to 8)
    SDC1 = 0; // Set initial secondary duty ratio 

    // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER (x = 1 to 8)
    PHASE1 = PWM_PHASE_SHIFT; // Set initial primary phase shift

    // SPHASEx: PWMx SECONDARY PHASE-SHIFT REGISTER (x = 1 to 8)
    SPHASE1 = 0; // Set initial secondary phase shift
    
    // DTRx: PWMx DEAD-TIME REGISTER (x = 1 to 8)
    DTR1 = PWM_DT_RISING; // Set initial dead time (rising edge)
    
    // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER (x = 1 to 8)
    ALTDTR1 = PWM_DT_FALLING; // Set initial dead time (falling edge)
    
    // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    TRIG1 = IOUT_ADCTRIG_DLY; // Set initial primary trigger

    // STRIGx: PWMx SECONDARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    STRIG1 = 0; // Set initial secondary trigger

    // LEBDLYx: PWMx LEADING-EDGE BLANKING DELAY REGISTER (x = 1 to 8)
    LEBDLY1 = PWM_LEB_DELAY; // Set initial Leading Edge Blanking delay

    // PWMCAPx: PWMx PRIMARY TIME BASE CAPTURE REGISTER (x = 1 to 8)
    PWMCAP1 = 0; // Clear PWM capture register
    
    
    return(fres);
}


volatile uint16_t PWM2_Initialize(void) {
    
    volatile uint16_t fres=1;
    
    // PWMCONx: PWMx CONTROL REGISTER (x = 1 to 8)
    PWMCON2bits.FLTIEN = 0; // Fault Interrupt disabled
    PWMCON2bits.CLIEN = 0; // Current Limit Interrupt disabled
    PWMCON2bits.TRGIEN = 0; // Trigger Interrupt disabled
    PWMCON2bits.ITB = 0; // PTPER register provides timing for this PWMx generator
    PWMCON2bits.MDCS = 0; // PDCx and SDCx registers provide duty cycle information for this PWMx generator
    PWMCON2bits.DTC = 0b00; // Positive dead time is actively applied for all Output modes
    PWMCON2bits.MTBS = 0; // PWMx generator uses the primary master time base for synchronization and 
                          // the clock source for the PWMx generation logic
    PWMCON2bits.CAM = 0; // Edge-Aligned mode is enabled
    PWMCON2bits.XPRES = 0; // External pins do not affect the PWMx time base
    PWMCON2bits.IUE = 0; // Updates to the active Duty Cycle, Phase Offset, Dead-Time and local 
                         // Time Base Period registers are synchronized to the local PWMx time base
    
    
    // IOCONx: PWMx I/O CONTROL REGISTER (x = 1 to 8)
    IOCON2bits.PMOD = 0b00; // PWMx I/O pin pair is in the Complementary Output mode

    IOCON2bits.PENH = 0; // GPIO module controls the PWMxH pin => Control is handed over to PWM module when enabled
    IOCON2bits.PENL = 0; // GPIO module controls the PWMxL pin => Control is handed over to PWM module when enabled
    IOCON2bits.POLH = 0; // PWMxH pin is active-high
    IOCON2bits.POLL = 0; // PWMxL pin is active-high

    IOCON2bits.OVRDAT = 0b00; // Software Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON2bits.OVRENH = 1; // OVRDAT1 provides data for output on the PWMxH pin
    IOCON2bits.OVRENL = 1; // OVRDAT0 provides data for output on the PWMxL pin
    IOCON2bits.OSYNC = 1; // Output overrides via the OVRDAT<1:0> bits are synchronized to the PWMx time base

    IOCON2bits.FLTDAT = 0b00; // FAULT Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON2bits.CLDAT = 0b00; // Current Limit Override Pin States: PWMxH = LOW / PWMxL = LOW
    IOCON2bits.SWAP = 0; // PWMxH and PWMxL pins are mapped to their respective pins
    
    // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER (x = 1 to 8)
    FCLCON2bits.IFLTMOD = 0; // Normal Fault mode: Current-Limit mode maps CLDAT<1:0> bits to the 
                             // PWMxH and PWMxL outputs; the PWM Fault mode maps FLTDAT<1:0> to the 
                             // PWMxH and PWMxL outputs
    FCLCON2bits.CLSRC = 0b11111; // Current-Limit Control Signal Source Selection: FLT31
    FCLCON2bits.CLPOL = 0b0; // The selected current-limit source is active-high
    FCLCON2bits.CLMOD = 0; // Current-Limit mode is disabled
    FCLCON2bits.FLTSRC = 0b11111; // Fault Control Signal Source Selection: Fault 31 (Default)
    FCLCON2bits.FLTPOL = 0b0; // The selected Fault source is active-high
    FCLCON2bits.FLTMOD = 0b11; // Fault input is disabled
    
    // LEBCONx: PWMx LEADING-EDGE BLANKING (LEB) CONTROL REGISTER (x = 1 to 8)
    LEBCON2bits.PHR = 1; // Rising edge of PWMxH will trigger the Leading-Edge Blanking counter
    LEBCON2bits.PHF = 0; // Leading-Edge Blanking ignores the falling edge of PWMxH
    LEBCON2bits.PLR = 0; // Leading-Edge Blanking ignores the rising edge of PWMxL
    LEBCON2bits.PLF = 0; // Leading-Edge Blanking ignores the falling edge of PWMxL
    LEBCON2bits.FLTLEBEN = 0; // Leading-Edge Blanking is not applied to the selected Fault input
    LEBCON2bits.CLLEBEN = 0; // Leading-Edge Blanking is not applied to the selected current-limit input
    LEBCON2bits.BCH = 0; // No blanking when the selected blanking signal is high
    LEBCON2bits.BCL = 0; // No blanking when the selected blanking signal is low
    LEBCON2bits.BPHH = 0; // No blanking when the PWMxH output is high
    LEBCON2bits.BPHL = 0; // No blanking when the PWMxH output is low
    LEBCON2bits.BPLH = 0; // No blanking when the PWMxL output is high
    LEBCON2bits.BPLL = 0; // No blanking when the PWMxL output is low

    // TRGCONx: PWMx TRIGGER CONTROL REGISTER (x = 1 to 8)
    TRGCON2bits.TRGDIV = 0b0000; // Trigger output for every trigger event
    TRGCON2bits.DTM = 0; // Secondary trigger event is not combined with the primary trigger event to 
                         // create a PWM trigger; two separate PWM triggers are generated
    TRGCON2bits.TRGSTRT = 0b000000; // Wait 0 PWM cycles before generating the first trigger event after 
                                    // the module is enabled
    
    // AUXCONx: PWMx AUXILIARY CONTROL REGISTER (x = 1 to 8)
    AUXCON2bits.HRPDIS = 0; // High-resolution PWMx period is enabled
    AUXCON2bits.HRDDIS = 0; // High-resolution PWMx duty cycle is enabled
    AUXCON2bits.BLANKSEL = 0b0000; // No state blanking
    AUXCON2bits.CHOPSEL = 0b0000; // Chop clock generator is selected as the chop clock source
    AUXCON2bits.CHOPHEN = 0; // PWMxH chopping function is disabled
    AUXCON2bits.CHOPLEN = 0; // PWMxL chopping function is disabled
    
    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER (x = 1 to 8)
    PDC2 = PWM_LEB_DELAY; // Set initial primary duty ratio 
    
    // SDCx: PWMx SECONDARY DUTY CYCLE REGISTER (x = 1 to 8)
    SDC2 = 0; // Set initial secondary duty ratio 

    // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER (x = 1 to 8)
    PHASE2 = 0; // Set initial primary phase shift

    // SPHASEx: PWMx SECONDARY PHASE-SHIFT REGISTER (x = 1 to 8)
    SPHASE2 = 0; // Set initial secondary phase shift
    
    // DTRx: PWMx DEAD-TIME REGISTER (x = 1 to 8)
    DTR2 = PWM_DT_RISING; // Set initial dead time (rising edge)
    
    // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER (x = 1 to 8)
    ALTDTR2 = PWM_DT_FALLING; // Set initial dead time (falling edge)
    
    // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    TRIG2 = IOUT_ADCTRIG_DLY; // Set initial primary trigger

    // STRIGx: PWMx SECONDARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    STRIG2 = VOUT_ADCTRIG_DLY; // Set initial secondary trigger

    // LEBDLYx: PWMx LEADING-EDGE BLANKING DELAY REGISTER (x = 1 to 8)
    LEBDLY2 = PWM_LEB_DELAY; // Set initial Leading Edge Blanking delay

    // PWMCAPx: PWMx PRIMARY TIME BASE CAPTURE REGISTER (x = 1 to 8)
    PWMCAP2 = 0; // Clear PWM capture register

    
    return(fres);
}

volatile uint16_t hspwm_OVR_Hold(void) {
    
    volatile uint16_t fres=1;
    
    IOCON1bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON1bits.OVRENL = 1; // Set Override Enable for PWMxL 
    IOCON2bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON2bits.OVRENL = 1; // Set Override Enable for PWMxL 
    
    return(fres);
}

volatile uint16_t hspwm_OVR_Release(void) {
    
    volatile uint16_t fres=1;
    
    IOCON1bits.OVRENH = 0; // Clear Override Enable for PWMxH
    IOCON1bits.OVRENL = 0; // Clear Override Enable for PWMxL 
    IOCON2bits.OVRENH = 0; // Clear Override Enable for PWMxH
    IOCON2bits.OVRENL = 0; // Clear Override Enable for PWMxL 
    
    return(fres);
}

volatile uint16_t hspwm_Enable(void) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    
    IOCON1bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON1bits.OVRENL = 1; // Set Override Enable for PWMxL 
    IOCON2bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON2bits.OVRENL = 1; // Set Override Enable for PWMxL 
    
    PTCONbits.PTEN = 1; // Enable PWM module
    
    for(i=5000; i>0; i--); // Short delay
    
    IOCON1bits.PENH = 1; // PWMxH ownership is assigned to PWM generator
    IOCON1bits.PENL = 1; // PWMxL ownership is assigned to PWM generator
    IOCON2bits.PENH = 1; // PWMxH ownership is assigned to PWM generator
    IOCON2bits.PENL = 1; // PWMxL ownership is assigned to PWM generator

    return(fres);
}

volatile uint16_t hspwm_Disable(void) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    
    IOCON1bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON1bits.OVRENL = 1; // Set Override Enable for PWMxL 
    IOCON2bits.OVRENH = 1; // Set Override Enable for PWMxH
    IOCON2bits.OVRENL = 1; // Set Override Enable for PWMxL 
    
    for(i=5000; i>0; i--); // Short delay

    IOCON1bits.PENH = 0; // PWMxH ownership is assigned to PWM generator
    IOCON1bits.PENL = 0; // PWMxL ownership is assigned to PWM generator
    IOCON2bits.PENH = 0; // PWMxH ownership is assigned to PWM generator
    IOCON2bits.PENL = 0; // PWMxL ownership is assigned to PWM generator

    PTCONbits.PTEN = 0; // Enable PWM module
    
    return(fres);
}



