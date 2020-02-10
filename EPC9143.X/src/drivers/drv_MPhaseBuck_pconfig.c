/*
 * File:   drv_MPhaseBuck_pconfig.c
 * Author: M91406
 *
 * Created on February 5, 2020, 3:29 PM
 */


#include "drivers/drv_MPhaseBuck_pconfig.h"
#include "drivers/drv_MPhaseBuck_control.h"

/* PRIVATE VARIABLES */
volatile uint16_t adcore_mask=0;
volatile uint16_t adcore_diff_mask=0;

volatile uint16_t MPhaseBuck_PWM_Module_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {

    volatile uint16_t fres=1;

    #if defined (__P33SMPS_EP__)
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
    
    PTPER = pInstance->swnode.period; // PWM PRIMARY MASTER TIME BASE PERIOD REGISTER
    SEVTCMP = 0;        // PWM SPECIAL EVENT COMPARE REGISTER

    STPER = 0;          // PWM SECONDARY MASTER TIME BASE PERIOD REGISTER
    SSEVTCMP = 0;       // PWM SECONDARY SPECIAL EVENT COMPARE REGISTER
    
    // CHOP: PWM CHOP CLOCK GENERATOR REGISTER
    //    Value is in 8.32 ns increments. The frequency of the chop clock signal is given by:
    //    Chop Frequency = 1/(16.64 * (CHOP<7:3> + 1) * Primary Master PWM Input Clock Period)    
    CHOPbits.CHPCLKEN = 0; // Disable Chop Clock Generator
    CHOPbits.CHOPCLK = 0;  // Chop Clock Divider bits
    
    MDC = 0; // PWM MASTER DUTY CYCLE REGISTER (not used, PDCx registers provide individual duty cycle)
    
    #elif defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)

    #pragma message "device type not supported by this driver"

    #endif
    
    return(fres);
}

volatile uint16_t MPhaseBuck_PWM_Channel_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0, channel=0;
    volatile uint16_t *ptrPWMRegister;
    volatile PWM_CHANNEL_REGISTER_CONFIG_t* ptrPWMChannelConfig;
    volatile PWM_CHANNEL_REGISTER_CONFIG_t pwm_ch;
    
    #if defined (__P33SMPS_EP__)

    pwm_ch.regPWMCON = PWMCONx_CONFIG;  // PWMCONx: PWMx CONTROL REGISTER (x = 1 to 8)
    pwm_ch.regIOCON  = IOCONx_CONFIG;   // IOCONx: PWMx I/O CONTROL REGISTER (x = 1 to 8)
    pwm_ch.regFCLCON = FCLCONx_CONFIG;  // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER (x = 1 to 8)
    pwm_ch.regTRGCON = TRGCONx_CONFIG;  // TRGCONx: PWMx TRIGGER CONTROL REGISTER (x = 1 to 8)
    pwm_ch.regLEBCON = LEBCONx_CONFIG;  // LEBCONx: PWMx LEADING-EDGE BLANKING (LEB) CONTROL REGISTER (x = 1 to 8)
    pwm_ch.regAUXCON = AUXCONx_CONFIG;  // AUXCONx: PWMx AUXILIARY CONTROL REGISTER (x = 1 to 8)
    
    pwm_ch.regPDC = PWM_LEB_DELAY;      // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER (x = 1 to 8)
    pwm_ch.regSDC = 0;                  // SDCx: PWMx SECONDARY DUTY CYCLE REGISTER (x = 1 to 8)
    pwm_ch.regPHASE = PWM_PHASE_SHIFT;  // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER (x = 1 to 8)
    pwm_ch.regSPHASE = 0;               // SPHASEx: PWMx SECONDARY PHASE-SHIFT REGISTER (x = 1 to 8)
    pwm_ch.regDTR = PWM_DT_RISING;      // DTRx: PWMx DEAD-TIME REGISTER (x = 1 to 8)
    pwm_ch.regALTDTR = PWM_DT_FALLING;  // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER (x = 1 to 8)
    pwm_ch.regTRIG = IOUT_ADCTRIG_DLY;  // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    pwm_ch.regSTRIG = 0;                // STRIGx: PWMx SECONDARY TRIGGER COMPARE VALUE REGISTER (x = 1 to 8)
    pwm_ch.regLEBDLY = PWM_LEB_DELAY;   // LEBDLYx: PWMx LEADING-EDGE BLANKING DELAY REGISTER (x = 1 to 8)
    pwm_ch.regPWMCAP = 0;               // PWMCAPx: PWMx PRIMARY TIME BASE CAPTURE REGISTER (x = 1 to 8)
    
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        channel = (pInstance->swnode.pwm_instance[i] - 1);  // Get PWM instance from data structure
                                                            // (PWM channel index starts from 1)
        
        // Get PWM channel register block start address
        ptrPWMRegister = (volatile uint16_t*)(&PWMCON1 + (channel * PWM_REGISTER_OFFSET));
        ptrPWMChannelConfig = (volatile PWM_CHANNEL_REGISTER_CONFIG_t*)ptrPWMRegister;
        *ptrPWMChannelConfig = pwm_ch; //Write default configuration to PWM channel register block

        
        // Write custom settings

        ptrPWMRegister = (volatile uint16_t*)(&PHASE1 + (channel * PWM_REGISTER_OFFSET));
        *ptrPWMRegister = pInstance->swnode.phase[i];

        ptrPWMRegister = (volatile uint16_t*)(&PDC1 + (channel * PWM_REGISTER_OFFSET));
        *ptrPWMRegister = pInstance->swnode.duty_ratio_init[i];

        ptrPWMRegister = (volatile uint16_t*)(&DTR1 + (channel * PWM_REGISTER_OFFSET));
        *ptrPWMRegister = pInstance->swnode.dead_time_rising[i];
        
        ptrPWMRegister = (volatile uint16_t*)(&ALTDTR1 + (channel * PWM_REGISTER_OFFSET));
        *ptrPWMRegister = pInstance->swnode.dead_time_falling[i];

        ptrPWMRegister = (volatile uint16_t*)(&TRGCON1 + (channel * PWM_REGISTER_OFFSET));
        *ptrPWMRegister |= ((pInstance->swnode.trigger_scaler[i] - 1) << 12);
        *ptrPWMRegister |= pInstance->swnode.trigger_offset[i];
        
    }
    
    #elif defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)

    #pragma message "device type not supported by this driver"

    #endif
    
    return(fres);
}

volatile uint16_t MPhaseBuck_ADC_Module_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres=1;

    #if defined (__P33SMPS_EP__)
    
    // ADCON1L: ADC CONTROL REGISTER 1 LOW
    ADCON1Lbits.ADON = 0;       // Disable ADC while being configured
    ADCON1Lbits.ADSIDL = 1;     // Run ADC while in CPU is in idle mode
    ADCON1Lbits.NRE = 1;        // Enable Noise Reduction TAD control
    
    // ADCON1H: ADC CONTROL REGISTER 1 HIGH
    ADCON1Hbits.FORM = 0;       // ADC produces results in integer format (right-aligned)
    ADCON1Hbits.SHRRES = 0b11;  // ADC results are 12-bit long
    
    // ADCON2L: ADC CONTROL REGISTER 2 LOW
    ADCON2Lbits.REFCIE = 0;     // Band Gap and Reference Voltage Ready Common Interrupt is disabled
    ADCON2Lbits.REFERCIE = 0;   // Band Gap and Reference Voltage Error Common Interrupt is disabled
    ADCON2Lbits.EIEN = 1;       // Early Interrupt feature is enabled
    ADCON2Lbits.SHREISEL = 0b111; // Early interrupt of shared ADC core is generated eight TADCORE clocks early
    ADCON2Lbits.SHRADCS = 0b0000001; // Shared ADC Core Input Clock Divider = 2 Source Clock Periods
    
    // ADCON2H: ADC CONTROL REGISTER 2 HIGH
    ADCON2Hbits.SHRSAMC = 0b0000000111;    // Shared ADC Core Sample Time Selection = 10 TAD clocks

    // ADCON3L: ADC CONTROL REGISTER 3 LOW
    ADCON3Lbits.REFSEL = 0b000;     // AVDD is ADC reference (no other option)
    ADCON3Lbits.SUSPEND = 0;        // All ADC triggers are active
    ADCON3Lbits.SUSPCIE = 0;        // All ADC core common interrupts are disabled when SUSPEND is set
    ADCON3Lbits.SHRSAMP = 0;        // Sampling is controlled by the shared ADC core hardware
    ADCON3Lbits.CNVRTCH = 0;        // Clear Single Software Trigger Control bit
    ADCON3Lbits.SWLCTRG = 0;        // No Software Level-Sensitive Common Triggers are generated
    ADCON3Lbits.SWCTRG = 0;         // Clear Software Common Trigger control bit
    ADCON3Lbits.CNVCHSEL = 4;       // Channel Number Selection for Software Individual Channel Conversion Trigger
                                    // (#4 is the first ADC input connected to the shared core)

    // ADCON3H: ADC CONTROL REGISTER 3 HIGH
    ADCON3Hbits.CLKSEL = 0b11;      // APLL is input clock to ADC
    ADCON3Hbits.CLKDIV = 0b000001;  // ADC Module Clock Source Divider = 2 Source Clock Periods (TAD = ~16.7ns)
    ADCON3Hbits.SHREN = 0;          // Shared Core is disabled
    ADCON3Hbits.C0EN = 0;           // Dedicated core #0 is disabled
    ADCON3Hbits.C1EN = 0;           // Dedicated core #1 is disabled
    ADCON3Hbits.C2EN = 0;           // Dedicated core #2 is disabled
    ADCON3Hbits.C3EN = 0;           // Dedicated core #3 is disabled

    // ADCON4L: ADC CONTROL REGISTER 4 LOW
    ADCON4Lbits.SAMC0EN = 0;        // Dedicated ADC Core #0 Conversion Delay is disabled
    ADCON4Lbits.SAMC1EN = 0;        // Dedicated ADC Core #1 Conversion Delay is disabled
    ADCON4Lbits.SAMC2EN = 0;        // Dedicated ADC Core #2 Conversion Delay is disabled
    ADCON4Lbits.SAMC3EN = 0;        // Dedicated ADC Core #3 Conversion Delay is disabled
    ADCON4Lbits.SYNCTRG0 = 0;       // Dedicated ADC Core #0 trigger is synchronized to ADC core clock
    ADCON4Lbits.SYNCTRG1 = 0;       // Dedicated ADC Core #1 trigger is synchronized to ADC core clock
    ADCON4Lbits.SYNCTRG2 = 0;       // Dedicated ADC Core #2 trigger is synchronized to ADC core clock
    ADCON4Lbits.SYNCTRG3 = 0;       // Dedicated ADC Core #3 trigger is synchronized to ADC core clock

    // ADCON4H: ADC CONTROL REGISTER 4 HIGH
    ADCON4Hbits.C0CHS = 0b00;       // Dedicated ADC Core 0 Input Channel Selection: AN0 (default)
    ADCON4Hbits.C1CHS = 0b00;       // Dedicated ADC Core 1 Input Channel Selection: AN1 (default)
    ADCON4Hbits.C2CHS = 0b00;       // Dedicated ADC Core 2 Input Channel Selection: AN2 (default)
    ADCON4Hbits.C3CHS = 0b00;       // Dedicated ADC Core 3 Input Channel Selection: AN3 (default)

    // ADCON5L: ADC CONTROL REGISTER 5 LOW
    ADCON5Lbits.SHRPWR = 0;         // Power to Shared ADC Core is turned off
    ADCON5Lbits.C0PWR = 0;          // Power to Dedicated ADC Core #0 is turned off
    ADCON5Lbits.C1PWR = 0;          // Power to Dedicated ADC Core #1 is turned off
    ADCON5Lbits.C2PWR = 0;          // Power to Dedicated ADC Core #2 is turned off
    ADCON5Lbits.C3PWR = 0;          // Power to Dedicated ADC Core #3 is turned off

    // ADCON5H: ADC CONTROL REGISTER 5 HIGH
    ADCON5Hbits.WARMTIME = 0b1111;  // Warm-Up Time of ADC = 32768 Source Clock Periods
    ADCON5Hbits.SHRCIE = 0;         // Shared ADC Core Ready Common Interrupt is disabled
    ADCON5Hbits.C0CIE = 0;          // Dedicated ADC Core #0 Ready Common Interrupt is disabled
    ADCON5Hbits.C1CIE = 0;          // Dedicated ADC Core #1 Ready Common Interrupt is disabled
    ADCON5Hbits.C2CIE = 0;          // Dedicated ADC Core #2 Ready Common Interrupt is disabled
    ADCON5Hbits.C3CIE = 0;          // Dedicated ADC Core #3 Ready Common Interrupt is disabled
    
    // ADCORE0L: DEDICATED ADC CORE x CONTROL REGISTER LOW 
    ADCORE0Lbits.SAMC = 0b0000000000; // Dedicated ADC Core x Conversion Delay Selection = 2 TADCORE
    
    // ADCORE0H: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE0Hbits.ADCS = 0b0000001;  // ADC Core x Input Clock Divider = 2 Source Clock Periods; 
    ADCORE0Hbits.EISEL = 0b111;     // Early interrupt is generated eight TADCORE clocks early
    ADCORE0Hbits.RES = 0b11;        // Dedicated ADC core x resolution is 12 bit

    // ADCORE1L: DEDICATED ADC CORE x CONTROL REGISTER LOW 
    ADCORE1Lbits.SAMC = 0b0000000000; // Dedicated ADC Core x Conversion Delay Selection = 2 TADCORE
    
    // ADCORE1H: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE1Hbits.ADCS = 0b0000001;  // ADC Core x Input Clock Divider = 2 Source Clock Periods; 
    ADCORE1Hbits.EISEL = 0b111;     // Early interrupt is generated eight TADCORE clocks early
    ADCORE1Hbits.RES = 0b11;        // Dedicated ADC core x resolution is 12 bit

    // ADCORE2L: DEDICATED ADC CORE x CONTROL REGISTER LOW 
    ADCORE2Lbits.SAMC = 0b0000000000; // Dedicated ADC Core x Conversion Delay Selection = 2 TADCORE
    
    // ADCORE2H: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE2Hbits.ADCS = 0b0000001;  // ADC Core x Input Clock Divider = 2 Source Clock Periods; 
    ADCORE2Hbits.EISEL = 0b111;     // Early interrupt is generated eight TADCORE clocks early
    ADCORE2Hbits.RES = 0b11;        // Dedicated ADC core x resolution is 12 bit
    
    // ADCORE3L: DEDICATED ADC CORE x CONTROL REGISTER LOW 
    ADCORE3Lbits.SAMC = 0b0000000000; // Dedicated ADC Core x Conversion Delay Selection = 2 TADCORE
    
    // ADCORE3H: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE3Hbits.ADCS = 0b0000001;  // ADC Core x Input Clock Divider = 2 Source Clock Periods; 
    ADCORE3Hbits.EISEL = 0b111;     // Early interrupt is generated eight TADCORE clocks early
    ADCORE3Hbits.RES = 0b11;        // Dedicated ADC core x resolution is 12 bit

    // Clear all ADC input channel settings
    ADLVLTRGL = 0x0000;
    ADLVLTRGH = 0x0000;
    ADEIEL    = 0x0000;
    ADEIEH    = 0x0000;
    ADIEL     = 0x0000;
    ADIEH     = 0x0000;
    ADMOD0L   = 0x0000;
    ADMOD0H   = 0x0000;
    #ifdef ADMOD1L
    ADMOD1L   = 0x0000;
    #endif
    #ifdef ADMOD1H
    ADMOD1H   = 0x0000;
    #endif
    ADTRIG0L  = 0x0000;
    ADTRIG0H  = 0x0000;
    ADTRIG1L  = 0x0000;
    ADTRIG1H  = 0x0000;
    ADTRIG2L  = 0x0000;
    ADTRIG2H  = 0x0000;
    ADTRIG3L  = 0x0000;
    ADTRIG3H  = 0x0000;
    ADTRIG4L  = 0x0000;
    ADTRIG4H  = 0x0000;
    #ifdef ADTRIG5L
    ADTRIG5L  = 0x0000;
    #endif
    #ifdef ADTRIG5H
    ADTRIG5H  = 0x0000;
    #endif
    
    // If Band Gap and Reference Voltage Ready Flag bit (read only) is set
    //  and Band Gap and Reference Voltage Error Flag bit (read only) is cleared...
    if((ADCON2Hbits.REFRDY) && (!ADCON2Hbits.REFERR)) {
        
    } 
    
    #elif defined (__P33SMPS_CH__) || defined (__P33SMPS_CK__)

    #pragma message "device type not supported by this driver"

    #endif
    
    return(fres);
}

volatile uint16_t MPhaseBuck_ADC_Channel_Initialize(volatile MPHBUCK_ADC_INPUT_SETTINGS_t* pInstance) {
    
    volatile uint16_t fres=1;
    volatile uint8_t* ptrADCRegister;
    volatile uint8_t bit_offset;
    
    // Initialize ADC input registers
    if (pInstance->enable) {

        // Write level trigger setting
        if (pInstance->adc_input < 16) {
            ADLVLTRGL |= ((uint16_t)(pInstance->level_trigger) << pInstance->adc_input);
            ADEIEL |= ((uint16_t)(pInstance->early_interrupt_enable) << pInstance->adc_input);
            ADIEL |= ((uint16_t)(pInstance->interrupt_enable) << pInstance->adc_input);
        }
        else if (pInstance->adc_input < 32) {
            ADLVLTRGH |= ((uint16_t)(pInstance->level_trigger) << (pInstance->adc_input - 16));
            ADEIEH |= ((uint16_t)(pInstance->early_interrupt_enable) << (pInstance->adc_input - 16));
            ADIEH |= ((uint16_t)(pInstance->interrupt_enable) << (pInstance->adc_input - 16));
        }
        else {
            return(0); // ADC input number out of range
        }

        // write input mode setting
        ptrADCRegister = (volatile uint8_t *)((volatile uint8_t *)&ADMOD0L + (volatile uint8_t)(pInstance->adc_input >> 8));
        if (pInstance->adc_input < 8)
            bit_offset = (2 * pInstance->adc_input);
        else if (pInstance->adc_input < 16)
            bit_offset = (2 * (pInstance->adc_input-8));
        else if (pInstance->adc_input < 24)
            bit_offset = (2 * (pInstance->adc_input-16));
        else if (pInstance->adc_input < 32)
            bit_offset = (2 * (pInstance->adc_input-24));
        else
            return(0); // ADC input number out of range
        
        *ptrADCRegister |= ((unsigned int)pInstance->signed_result << bit_offset);
        *ptrADCRegister |= ((unsigned int)pInstance->differential_input << (bit_offset + 1));
       
        // Write ADC trigger source setting
        ptrADCRegister = (volatile uint8_t *)((volatile uint8_t *)&ADTRIG0L + pInstance->adc_input);
        *ptrADCRegister = pInstance->trigger_source;
        
        // Register ADC core to be active
        switch (pInstance->adc_core) {
            case 0:
                adcore_mask |= ADC_CORE0_MASK_INDEX;
                if (pInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE0_MASK_INDEX;
                break;
            case 1:
                adcore_mask |= ADC_CORE1_MASK_INDEX;
                if (pInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE1_MASK_INDEX;
                break;
            case 2:
                adcore_mask |= ADC_CORE2_MASK_INDEX;
                if (pInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE2_MASK_INDEX;
                break;
            case 3:
                adcore_mask |= ADC_CORE3_MASK_INDEX;
                if (pInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE3_MASK_INDEX;
                break;
            default:
                adcore_mask |= ADC_SHRCORE_MASK_INDEX;
                if (pInstance->differential_input)
                    adcore_diff_mask |= ADC_SHRCORE_MASK_INDEX;
                break;
        }
        
    }

    
    return(fres);
}

#if defined (__P33SMPS_EP__)
volatile uint16_t MPhaseBuck_ADC_EnableAndCalibrateADC(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    volatile uint16_t timeout=0;
    volatile uint16_t adcore_mask_compare=0;
    volatile uint8_t* ptrADCRegister;
    
    // Turn on ADC module
    ADCON1Lbits.ADON = 1;

    ADCON5L = adcore_mask;    // Enable power to all used ADC cores
    adcore_mask_compare = ((adcore_mask << 8) | adcore_mask); // Set ADC Core Ready Bit Mask
    
    while ((ADCON5L != adcore_mask_compare) & (timeout++ < 30000)); // Wait until ADC cores are ready
    if (timeout >= 30000) return(0); // Skip if calibration was unsuccessful
    ADCON3H = adcore_mask; // Turn on ADC cores

    // Execute calibration for used cores
    for (i=0; i<8; i++) {
        
        if (adcore_mask & (0x0001 << i)) {
            
            // Get register address
            ptrADCRegister = (volatile uint8_t *)((volatile uint8_t *)&ADCAL0L + (uint8_t)i);

            *ptrADCRegister |= ADC_CORE_CAL_EN; // Turn on calibration mode

            if(adcore_diff_mask & (0x0001 << i))    // Turn on or bypass differential mode calibration
                *ptrADCRegister |= ADC_CORE_CAL_DIFF;

            *ptrADCRegister |= ADC_CORE_CAL_RUN; // Run calibration
            
            // Wait until calibration is complete
            timeout = 0;    // Reset timeout counter
            while (!(*ptrADCRegister & ADC_CORE_CAL_RDY) & (timeout++ < 30000));  // Wait until calibration is complete
            if (timeout >= 10000) return(0); // Skip if calibration was unsuccessful    

            *ptrADCRegister &= (~ADC_CORE_CAL_EN); // Turn off calibration mode
            
        }
    }

    return(fres);
}
#endif

/*!MPhaseBuck_PWM_Hold
 * ***********************************************************************************************
 * Disables the PWM outputs of the multiphase converter
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_PWM_Hold(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    volatile uint16_t* ptrPWMRegister;
    volatile uint16_t reg_offset=0;

    // Disable phase current controllers and PWM outputs
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        // Disable PWM outputs
        reg_offset = (volatile uint16_t)((pInstance->swnode.pwm_instance[i]-1) * PWM_REGISTER_OFFSET);
        ptrPWMRegister = (volatile uint16_t*)(&IOCON1 + reg_offset);
        *ptrPWMRegister |= (volatile uint16_t)PWM_OVR_HOLD;

    }
    
    return(fres);
}

/*!MPhaseBuck_PWM_Release
 * ***********************************************************************************************
 * Enables the PWM outputs of the multiphase converter
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_PWM_Release(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    volatile uint16_t* ptrPWMRegister;
    volatile uint16_t reg_offset=0;

    // Disable phase current controllers and PWM outputs
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        // Enable PWM outputs
        reg_offset = (volatile uint16_t)((pInstance->swnode.pwm_instance[i]-1) * PWM_REGISTER_OFFSET);
        ptrPWMRegister = (volatile uint16_t*)(&IOCON1 + reg_offset);
        *ptrPWMRegister &= (volatile uint16_t)(~PWM_OVR_HOLD);

    }

    return(fres);
}

/*!MPhaseBuck_PWM_Enable
 * ***********************************************************************************************
 * Disables the PWM outputs of the multiphase converter
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_PWM_Enable(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t i=0;
    volatile uint16_t* ptrPWMRegister;
    volatile uint16_t reg_offset=0;

    // Enable override bits (disables the PWM outputs) and disconnect PWM generator from IO pins
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        // Disable PWM outputs
        reg_offset = (volatile uint16_t)((pInstance->swnode.pwm_instance[i]-1) * PWM_REGISTER_OFFSET);
        ptrPWMRegister = (volatile uint16_t*)(&IOCON1 + reg_offset);
        *ptrPWMRegister |= (volatile uint16_t) PWM_OVR_HOLD;
        *ptrPWMRegister &= (volatile uint16_t)(~PWM_PEN_ENABLE);

    }

    PTCONbits.PTEN = true; // Enable PWM module

    // Connect PWM generator from IO pins
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        // Connect PWM generator with pins
        reg_offset = (volatile uint16_t)((pInstance->swnode.pwm_instance[i]-1) * PWM_REGISTER_OFFSET);
        ptrPWMRegister = (volatile uint16_t*)(&IOCON1 + reg_offset);
        *ptrPWMRegister |= (volatile uint16_t)PWM_PEN_ENABLE;

    }

    // Set status bit in controller status word
    pInstance->status.bits.pwm_started = PTCONbits.PTEN;
    
    // Override bits remain set => need to be cleared in user code
    return(PTCONbits.PTEN);
    
}


/*!MPhaseBuck_PWM_Disable
 * ***********************************************************************************************
 * Disables the PWM outputs of the multiphase converter
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_PWM_Disable(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t i=0;
    volatile uint16_t* ptrPWMRegister;
    volatile uint16_t reg_offset=0;

    // Enable override bits (disables the PWM outputs) and disconnect PWM generator from IO pins
    for (i=0; i<pInstance->swnode.number_of_phases; i++) {
        
        // Disable PWM outputs
        reg_offset = (volatile uint16_t)((pInstance->swnode.pwm_instance[i]-1) * PWM_REGISTER_OFFSET);
        ptrPWMRegister = (volatile uint16_t*)(&IOCON1 + reg_offset);
        *ptrPWMRegister |= PWM_OVR_HOLD;
        *ptrPWMRegister &= (volatile uint16_t)(~PWM_PEN_ENABLE);

    }

    PTCONbits.PTEN = false; // Disable PWM module
    
    return(1-PTCONbits.PTEN);

}

