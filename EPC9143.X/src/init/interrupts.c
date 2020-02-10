/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */

#include <xc.h>
#include <stdint.h>        /* Includes uint16_t definition   */
#include <stdbool.h>       /* Includes true/false definition */

#include "globals.h"


void __attribute__((interrupt,auto_psv, context)) _ADCAN0Interrupt(void)               
{                                                                          

    LATB |= (1 << 2);
    vout_adc = ADCBUF0; //don't make Q15
    
    newadcavailable = 1;//means that some ADC has triggered interrupt since its measurement is available
   

//    v_loop_Update(&v_loop);
/* fixed duty cycle*/
    PDC1 = DUTY_FIXED;
    PDC4 = DUTY_FIXED;   
    
    LATB &= ~(1 << 2);
    _ADCAN0IF = 0;  
    
}  

void __attribute__((interrupt,auto_psv, context)) _ADCAN3Interrupt(void)               
{     
    LATB |= (1 << 2);
    vin_adc = ADCBUF1;
    isen[0] = ADCBUF2;
    isen[1] = ADCBUF3;
    LATB &= ~(1 << 2);
    _ADCAN3IF = 0;  

}  

void __attribute__((interrupt,auto_psv, context)) _ADCAN4Interrupt(void)               
{     

//    iout_adc = ADCBUF4 + IOUTADC(0.2);
////    pi_curr_Update(&pi_curr);
//    i_loop_Update(&i_loop);
//    TRIG1 = (PWMPER >> 1) - i_trigger + ADC_TRIG_OFFSET;
//    PDC1 = PWMPER - duty_out;
//    TRIG4 = (PWMPER >> 1) - v_trigger;
//    vin_adc = ADCBUF1;


//    deltaDQ4 = deltaD_offset - MAXOFFSET;
//    PDC4 = PWMPER - duty_out + deltaDQ2;    
    _ADCAN4IF = 0;  

}  
void __attribute__((interrupt, auto_psv)) _PWM1Interrupt(void) {

    _PWM1IF = 0;
}

