/*
 * File:   ini_adc.c
 * Author: M91406.LA
 *
 * Created on November 2, 2019, 2:13 PM
 */


#include <xc.h>
#include <stdint.h>
#include "user.h"


volatile uint16_t ConfigureADC(void) {

    _ADON = 0; //turn off first before changing settings
    _FORM = 0; //Integer data format
    _CLKSEL = 0b11; // YZ this is APLL. 117.9 MHz //Fsys, 60MHz. Not Fsys (00) this is APLL. What is the frequency? 117.9 MHz
//    _CLKDIV = 6; // 117.9 MHz/ 7 = 16.84 MHz
    _CLKDIV = 0; // 117.9 MHz T_CORESRC, ADC conversion time is 310 ns

    ADCON1Hbits.FORM = 0;

//    ADCON1Hbits.SHRRES = 0b11; //12 bits
    ADCORE0Hbits.RES = 0b11; //12 bit
    ADCORE1Hbits.RES = 0b11; //12 bit
    ADCORE2Hbits.RES = 0b11; //12 bit
    ADCORE3Hbits.RES = 0b11;

    ADCORE0Hbits.ADCS = 0; //2 clock source periods
    ADCORE1Hbits.ADCS = 0; //2 clock source periods
    ADCORE2Hbits.ADCS = 0; //2 clock source periods
    ADCORE3Hbits.ADCS = 0; //
//    ADCON2Lbits.SHRADCS = 0;

    ADCON3Lbits.REFSEL = 0; //only choice, AVDD!
    //all are single ended, unsigned...no need to configure

//    ADCON2Hbits.SHRSAMC = 2; //4 TADCORE for sample time ... just a quick pick at an answer

    EnableAndCalibrateADC(); // extracted from ADC data sheet

    //set trigger sources for vin,vout,isen[0],isen[1]
    _TRGSRC0 = 0b00101; // PWM1
    _TRGSRC1 = 0b00101; // PWM1
    _TRGSRC2 = 0b00101; // PWM1
    _TRGSRC3 = 0b01000; // PWM4

//    ADTRIG1Lbits.TRGSRC4 = 0b00101; // PWM1 trigger
//    TRGCON1bits.DTM = 0; // No dual trigger mode
//    TRIG1 = ADCSAMPLEDELAY;
//    STRIG1bits.STRGCMP = ADCSAMPLEDELAY; //secondary trig compare value
//    STRIG1 = 1200; // PWM1L trigger

    TRGCON1bits.TRGDIV = 2; // Trigger generated every 3rd trigger event, sampling freq is 166 Hz
    TRGCON1bits.TRGSTRT = 0; // enable Trigger generated after 1 PWM cycles
    
//    TRIG2 = SADCSAMPLEDELAY;
    TRGCON4bits.TRGDIV = 2; // Trigger generated every 3rd trigger event
    TRGCON4bits.TRGSTRT = 0; // enable Trigger generated after 1 PWM cycles
    
    
    ADIELbits.IE0 = 1;
    _ADCAN0IP = 5;
    _ADCAN0IF = 0;
    _ADCAN0IE = 1;
    
    ADIELbits.IE3 = 1;
    _ADCAN3IP = 5;
    _ADCAN3IF = 0;
    _ADCAN3IE = 1;    


    return (1);
}



void EnableAndCalibrateADC(void) {
    
    // Set initialization time to maximum
    ADCON5Hbits.WARMTIME = 0b1011;
    // Turn on ADC module
    ADCON1Lbits.ADON = 1;

    // Set again - not sure why but this is part of the startup sequence
    //    as prescribed by Microchip
    ADCON5Hbits.WARMTIME = 0b1011;
    // Turn on analog power for dedicated core 0
    _C0PWR = 1;
    // Wait when the core 0 is ready for operation
    while (_C0RDY == 0);
    // Turn on digital power to enable triggers to the core 0
    _C0EN = 1;
    //repeat
    _C1PWR = 1;
    while (_C1RDY == 0);
    _C1EN = 1;

    _C2PWR = 1;
    while (_C2RDY == 0);
    _C2EN = 1;

    _C3PWR = 1;
    while (_C3RDY == 0);
    _C3EN = 1;

//    // Turn on analog power for shared core
//    _SHRPWR = 1;
//    // Wait when the shared core is ready for operation
//    while (_SHRRDY == 0);
//    // Turn on digital power to enable triggers to the shared core
//    _SHREN = 1;

    // Enable calibration for the dedicated core 0
    ADCAL0Lbits.CAL0EN = 1;
    // Single-ended input calibration
    ADCAL0Lbits.CAL0DIFF = 0;
    // Start calibration
    ADCAL0Lbits.CAL0RUN = 1;
    // Poll for the calibration end
    while (ADCAL0Lbits.CAL0RDY == 0);
    // Differential input calibration
    ADCAL0Lbits.CAL0DIFF = 1;
    // Start calibration
    ADCAL0Lbits.CAL0RUN = 1;
    // Poll for the calibration end
    while (ADCAL0Lbits.CAL0RDY == 0);
    // End the core 0 calibration
    ADCAL0Lbits.CAL0EN = 0;


    // Enable calibration for the dedicated core 1
    ADCAL0Lbits.CAL1EN = 1;
    // Single-ended input calibration
    ADCAL0Lbits.CAL1DIFF = 0;
    // Start calibration
    ADCAL0Lbits.CAL1RUN = 1;
    // Poll for the calibration end
    while (ADCAL0Lbits.CAL1RDY == 0);
    // Differential input calibration
    ADCAL0Lbits.CAL1DIFF = 1;
    // Start calibration
    ADCAL0Lbits.CAL1RUN = 1;
    // Poll for the calibration end
    while (ADCAL0Lbits.CAL1RDY == 0);
    // End the core 1 calibration
    ADCAL0Lbits.CAL1EN = 0;

    // Enable calibration for the dedicated core 2    
    _CAL2EN = 1;
    // Single-ended input calibration
    _CAL2DIFF = 0;
    // Start calibration
    _CAL2RUN = 1;
    // Poll for the calibration end
    while (_CAL2RDY == 0);
    // Differential input calibration
    _CAL2DIFF = 1;
    // Start calibration
    _CAL2RUN = 1;
    // Poll for the calibration end
    while (_CAL2RDY == 0);
    // End the core 1 calibration
    _CAL2EN = 0;

    // Enable calibration for the dedicated core 3    
    _CAL3EN = 1;
    // Single-ended input calibration
    _CAL3DIFF = 0;
    // Start calibration
    _CAL3RUN = 1;
    // Poll for the calibration end
    while (_CAL3RDY == 0);
    // Differential input calibration
    _CAL3DIFF = 1;
    // Start calibration
    _CAL3RUN = 1;
    // Poll for the calibration end
    while (_CAL3RDY == 0);
    // End the core 1 calibration
    _CAL3EN = 0;

//    // Enable calibration for the shared core
//    ADCAL1Hbits.CSHREN = 1;
//    // Single-ended input calibration
//    ADCAL1Hbits.CSHRDIFF = 0;
//    // Start calibration
//    ADCAL1Hbits.CSHRRUN = 1;
//    // Poll for the calibration end
//    while (ADCAL1Hbits.CSHRRDY == 0);
//    // Differential input calibration
//    ADCAL1Hbits.CSHRDIFF = 1;
//    // Start calibration
//    ADCAL1Hbits.CSHRRUN = 1;
//    // Poll for the calibration end
//    while (ADCAL1Hbits.CSHRRDY == 0);
//    // End the shared core calibration
//    ADCAL1Hbits.CSHREN = 0;

}
