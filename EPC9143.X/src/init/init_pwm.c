

#include <xc.h>
#include <stdint.h>
#include "user.h"

volatile uint16_t ConfigurePWM(void) {
    
    //clock at 960MHz, 1.04ns per tick       
    //PWM configuration
    _PTEN = 0;
    PTPER = PWMPER;
    MDC = 0;//MDC= duty cycle register; 0 means PDC and SDC registers provide duty cycle information for this PWM generator
    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;
    PDC4 = 0;

    
    // Channel 1, always used
    PHASE1 = 0;
    DTR1 = DEADTIME;
    ALTDTR1 = ALTDEADTIME;
 
    FCLCON1bits.FLTMOD = 0b11;
    FCLCON1bits.CLMOD = 0;      // disable current limit
    PWMCON1bits.TRGIEN = 1;
    PWMCON1bits.MDCS = 0;
//    PWMCON1bits.IUE = 1; // immediate update duty cycle


    // Channel 2 uses PWM4
    PHASE4 = (const int) (PWMPER+8)/2;
    DTR4 = DEADTIME;
    ALTDTR4 = ALTDEADTIME;

    FCLCON4bits.FLTMOD = 0b11;
    FCLCON4bits.CLMOD = 0;      // disable current limit
    PWMCON4bits.TRGIEN = 1;
    PWMCON4bits.MDCS = 0;
//    PWMCON4bits.IUE = 1; // immediate update duty cycle

    _PTEN = 1;
    
    return (1);
}
