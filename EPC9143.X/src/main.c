/*
 * File:   main.c
 * Author: M91406
 *
 * Created on July 8, 2019, 1:52 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"


int main(void) {

    volatile uint16_t timeout = 0;
    
    Oscillator_Initialize(); // Set up system oscillator and auxiliary clock
    SystemTimer_Initialize(); // Set up Timer1 as scheduler time base
    DSP_Initialize();
    ConfigureGPIOs();        // Initialize common device GPIOs
    
    // Configure power control state machine and objects
    PowerControl_Initialize();
    PowerControl_Execute();
    mph_buck.status.bits.enable = true;
    
    // Basic setup of common power controller peripheral modules
//    ConfigurePWM();  // Set up PWM module (basic module configuration)
//    ConfigureADC();  // Set up Analog-To-Digital converter module
//    init_vin_adc();     // Initialize ADC Channel to measure input voltage
//    fault_check_init(); // Initialize fault monitor objects
    
    // Enable Timer1
    T1CONbits.TON = 1; 
    
	// Main loop
    while (1) {

        // wait for timer1 to expire
        while ((!_T1IF) && (timeout++ < MAIN_TMR_TIMEOUT));
        timeout = 0; // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit

		// Execute port controller
        PowerControl_Execute();
//        FaultCheck_Execute();

		
        Nop();
        
    }


    return (0);
}
