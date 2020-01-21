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
     
    ConfigureOscillator();        // Set up system oscillator for 100 MIPS operation
    init_aclk();        // Set up Auxiliary PLL for 500 MHz (source clock to PWM module)
    init_timer1();      // Set up Timer1 as scheduler time base
    init_gpio();        // Initialize common device GPIOs
    
    // Basic setup of common power controller peripheral modules
    init_pwm_module();  // Set up PWM module (basic module configuration)
    init_adc_module();  // Set up Analog-To-Digital converter module
    init_vin_adc();     // Initialize ADC Channel to measure input voltage
    fault_check_init(); // Initialize fault monitor objects
    
    // Enable Timer1
    T1CONbits.TON = 1; 
    
	// Main loop
    while (1) {

        // wait for timer1 to expire
        while ((!_T1IF) && (timeout++ < TMR1_TIMEOUT));
        timeout = 0; // Reset timeout counter
        _T1IF = 0; // reset Timer1 interrupt flag bit

		// Execute port controller
        PowerControl_Execute();
        FaultCheck_Execute();

		
        Nop();
        
    }


    return (0);
}
