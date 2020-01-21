/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <xc.h>

#include <stdint.h>          /* For uint16_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */

#include "globals.h"         

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration functions, reset source evaluation         */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c.                                          */
/*                                                                            */
/******************************************************************************/

/* Refer to the device Family Reference Manual Oscillator section for
information about available oscillator configurations.  Typically
this would involve configuring the oscillator tuning register or clock
switching using the compiler's __builtin_write_OSCCON functions.
Refer to the C Compiler for PIC24 MCUs and dsPIC DSCs User Guide in the
compiler installation directory /doc folder for documentation on the
__builtin functions.*/

/* TODO Add clock switching code if appropriate.  An example stub is below.   */
volatile uint16_t Oscillator_Initialize(void) {
    
    volatile uint16_t i=0, timeout=0;

    // Tune FRC oscillator up to ~7.5 MHz to come closer to even oscillator frequencies
    // and achieve maximum PWM resolution and minimum ADC latencies
    CLKDIVbits.FRCDIV = 0b000; // Internal Fast RC Oscillator Postscaler = FRC divided by 1 (default)
    OSCTUN = 28; // Tune FRC to ~7.467 MHz to match maximum CPU performance specs
    
    // Configure Oscillator to operate the device at 70 MIPS
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37*(75)/(2*2)=140Mhz (140.0MHz) for Fosc, Fcy = 70Mhz
    // Configure PLL prescaler, PLL postscaler, PLL divisor

    CLKDIVbits.PLLPRE = (PLL_N1 - 2); // N1 = 2
    PLLFBD = (PLL_M - 2); // M = PLLFBD + 2 
    CLKDIVbits.PLLPOST = ((PLL_N2 / 2) - 1); // N2 = 2

    // Initiate Clock Switch to FRC oscillator with PLL(NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);
    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

    /* Setup for the Auxiliary clock to use the FRC as the REFCLK */
    /* ((FRC * 16) / APSTSCLR) = (7.37 * 16) / 1 = 117.9 MHz */
    ACLKCONbits.FRCSEL = 1; /* FRC is input to Auxiliary PLL */
    ACLKCONbits.SELACLK = 1; /* Auxiliary Oscillator provides the clock source */
    ACLKCONbits.APSTSCLR = 7; /* Divide Auxiliary clock by 1 */
    ACLKCONbits.ENAPLL = 1; /* Enable 16x Auxiliary PLL */
    while ((ACLKCONbits.APLLCK != 1) && (timeout++<6000)); /* Wait for Auxiliary PLL to Lock */

    // see errata - APLLCK is set immediately
    for(i = 0; i < 6000; i++);
    
    return(1);
    
}

