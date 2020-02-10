/*
 * File:   init_gpio.c
 * Author: M91406.LA
 *
 * Created on November 2, 2019, 2:19 PM
 */


#include <xc.h>
#include <stdint.h>

volatile uint16_t ConfigureGPIOs(void) {
    
    // Reset all analog functions
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    #ifdef ANSELC
    ANSELC = 0x0000;
    #endif
    #ifdef ANSELD
    ANSELD = 0x0000;
    #endif
    #ifdef ANSELE
    ANSELE = 0x0000;
    #endif
    
    
    //Set I/O pins to low level as default
    LATA = 0;
    LATB = 0;
    //all inputs are for ADC or the enable line, 1=input, 0=output
    TRISA = (1 << 0) | (1 << 1) | (1 << 2); //0b0111;
    TRISB = (1 << 0) | (1 << 5) | (1 << 7);
    //set analog pins to free others to be digital
    ANSELA = 0b111;
    ANSELB = (1 << 0);
    
    //need to assign PWM4 to re-programmable pins
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6));
    _RP40R = 0b0110100; //PWM4L
    _RP47R = 0b0110011; //PWM4H

     //allocate UART pins
//    RPINR18bits.U1RXR = 0x0027;    //;   //RB7->UART1:U1RX
//    RPOR5bits.RP38R = 0x0001;    //;   //RB6->UART1:U1TX    
    __builtin_write_OSCCONL(OSCCON | (1 << 6));    

    
#ifndef __DEBUG
    LATBbits.LATB7 = 0;     // Set PGC pin LOW
    TRISBbits.TRISB7 = 0;   // configure PGC pin as output for debugging
#endif
    
    return(1);
}
