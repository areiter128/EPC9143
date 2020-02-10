/*LICENSE *****************************************************************************************
 *
 * Software License Agreement
 *
 * Copyright (R) 2012 Microchip Technology Inc.  All rights reserved. Microchip licenses to you the
 * right to use, modify, copy and distribute Software only when embedded on a Microchip 
 * microcontroller or digital signal controller, which is integrated into your product or third 
 * party product (pursuant to the sublicense terms in the accompanying license agreement).
 *
 * You should refer to the license agreement accompanying this Software for additional information 
 * regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR 
 * IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR 
 * OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT  
 * LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS  
 * OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY  
 * THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 * ***********************************************************************************************/

#include <stdint.h>
#include "p33GS_hspwm.h"

// Device header file
#include <xc.h>


/*@@p33GS_hspwm.c
 * ************************************************************************************************
 * Summary:
 * Driver file for the dsPIC33FxxGS-PWM SFRs
 *
 * Description:
 * The SMPS PWM module offers a large number of registers and configuration options. This additional
 * driver file contains initialization routines for all required settings.
 * ***********************************************************************************************/


/*@@hspwm_reset_pwm_io
 * ************************************************************************************************
 * Summary:
 * Resets the IO control configuration of all PWM output pins
 *
 * Parameters:
 *	(none)
 * 
 * Description:
 * The PWM output pin configuration enables PWMxH/PWMxL outputs by default. This routine
 * sets them into GPIO mode, effectively disabling the PWM outputs.
 * ***********************************************************************************************/

uint16_t hspwm_reset_pwm_io(void)
{
    volatile uint16_t i=0;
    
    for (i=1; i<=HSPWM_CHANNEL_COUNT; i++)
    {
        hspwm_channel_output_disable(i, PWMx_HIGH_LOW);
    }

    return(1); 
}

/*@@hspwm_init_mtb
 * ************************************************************************************************
 * Summary:
 * Initializes the master time base configuration
 *
 * Parameters:
 *	index		= integer selecting primary (=1) or secondary (=2) time base
 *	regXTCON	= holds the register value for the PTCON/STCON register
 *				  => bit 15 of the PTCON register, which enables/disables the PWM module,
 *					 will automatically be masked out. Please enable the PWM module by 
 *					 calling hspwm_enable()
 *	regXTCON2	= PWM resolution divider
 *	period		= period value, depending on PWM resolution, sets the starting frequency
 *
 * Description:
 * Basic options like special interrupts, early updates, timebase synchronization options,
 * PWM resolution divider and starting period are set here.
 * ***********************************************************************************************/

uint16_t hspwm_init_mtb(uint16_t index, uint16_t regXTCON, uint16_t regXTCON2, uint16_t period)
{

volatile uint16_t *regptr;

	switch(index)
	{
	case 1:
	
		regptr  = (volatile uint16_t *)&PTCON;
		*regptr = ((regXTCON & REG_PTCON_VALID_BIT_MSK) & REG_PTCON_PWM_OFF_MASK);	// PWM ENABLE is masked out !!!
	
		regptr  = (volatile uint16_t *)&PTCON2;
		*regptr = (regXTCON2 & REG_PTCON2_VALID_BIT_MSK);

		regptr  = (volatile uint16_t *)&PTPER;
        *regptr = (period & REG_PTPER_VALID_BIT_MSK);

		break;

    #if defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

	case 2:
	
		regptr  = (volatile uint16_t *)&STCON;
		*regptr = (regXTCON & REG_STCON_VALID_BIT_MSK);
	
		regptr  = (volatile uint16_t *)&STCON2;
		*regptr = (regXTCON2 & REG_STCON2_VALID_BIT_MSK);
		
		regptr  = (volatile uint16_t *)&STPER;
        *regptr = (period & REG_STPER_VALID_BIT_MSK);

		break;

	#endif
	
	default:	
        return(0);
        break;

	}
	
	return(1);

}

/*@@hspwm_set_mtb_period
 * ************************************************************************************************
 * Summary:
 * Sets the master timebase period on the fly
 *
 * Parameters:
 *	index		= integer selecting primary (=1) or secondary (=2) timebase
 *	period		= period value, depending on PWM resolution, sets the starting frequency
 *
 * Description:
 * Enables the modification of the base frequency of the primary or secondary time base 
 * "on-the-fly"
 * ***********************************************************************************************/

uint16_t hspwm_set_mtb_period(uint16_t index, uint16_t period)
{

volatile uint16_t *regptr;

	switch(index)
	{
	case 1:

		regptr  = (volatile uint16_t *)&PTPER;
        *regptr = (period & REG_PTPER_VALID_BIT_MSK);
	
		break;

    #if defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

	case 2:
	
		regptr  = (volatile uint16_t *)&STPER;
        *regptr = (period & REG_STPER_VALID_BIT_MSK);
	
		break;

	#endif
	
	default:	
        return(0);
        break;

	}

	
	return(1);	

}

/*@@hspwm_get_mtb_period
 * ************************************************************************************************
 * Summary:
 * Reads the master timebase period on the fly
 *
 * Parameters:
 *	index		= integer selecting primary (=1) or secondary (=2) timebase
 *
 * Returns:
 *	period		= period value, depending on PWM resolution, sets the starting frequency
 *
 * Description:
 * Reads the base period of the primary or secondary time base 
 * ***********************************************************************************************/

uint16_t hspwm_get_mtb_period(uint16_t index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_buf=0;

	switch(index)
	{
	case 1:

		regptr  = (volatile uint16_t *)&PTPER;
        reg_buf = *regptr;
	
		break;

    #if defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

	case 2:
	
		regptr  = (volatile uint16_t *)&STPER;
        reg_buf = *regptr;
	
		break;

	#endif
	
	default:	
        reg_buf = 0;

	}

	return(reg_buf);	

}

/*@@hspwm_init_channel
 * ************************************************************************************************
 * Summary:
 * Initializes a specific PWM generator
 *
 * Parameters:
 *	index		= selects the register address range of the target pwm generator
 *	regPWMCON	= configures basic features of the pwm channel selected
 *	regIOCON	= configures the IO control of the associated PWMxH/PWMxL pins
 *	regFCLCON	= configures the FAULT and CURRENT LIMIT sources and features
 *	regLEBCON	= Configures the LEADING EDGE BLANKING (LEB) features 
 *				  => period is set by function hspwm_set_leb_period(...)
 *
 * Description:
 * Configures the basic settings and features of a specific PWM generator, its associated pins,
 * FAULT sources and behavior and leading edge blanking features
 * ***********************************************************************************************/

uint16_t hspwm_init_channel(uint16_t index, uint16_t regPWMCON, uint16_t regIOCON, 
		uint16_t regFCLCON, uint16_t regLEBCON)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	regptr  = (volatile uint16_t *)&PWMCON1 + reg_offset;
	*regptr = (regPWMCON & REG_PWMCONx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&IOCON1 + reg_offset;
	*regptr = (regIOCON & REG_IOCONx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&FCLCON1 + reg_offset;
	*regptr = (regFCLCON & REG_FCLCONx_VALID_BIT_MSK);
	
	regptr  = (volatile uint16_t *)&LEBCON1 + reg_offset;
    *regptr = (regLEBCON & REG_LEBCONx_VALID_BIT_MSK);
            
	return(1);

}

/*@@hspwm_set_duty_cycle
 * ************************************************************************************************
 * Summary:
 * Sets the duty cycle of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator (1...n)
 *  sub_index   = 
 *	duty_cycle  = sets the compare value of the duty cycle register
 *
 * Description:
 * Sets the duty cycle compare value of a specific PWM generator considering the different
 * registers and bit locations on different devices
 * ***********************************************************************************************/

uint16_t hspwm_set_duty_cycle(uint16_t index, uint16_t sub_index, uint16_t duty_cycle)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset = 0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch (sub_index){
        
        case 1: // Primary duty cycle register
            regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
            *regptr = (duty_cycle & REG_PDCx_VALID_BIT_MSK);
            break;
            
        case 2: // Secondary duty cycle register
            regptr  = (volatile uint16_t *)&SDC1 + reg_offset;
            *regptr = (duty_cycle & REG_SDCx_VALID_BIT_MSK);
            break;
            
        default:    // error
            return(0);
            break;
    }

	return(1);

}

/*@@hspwm_get_duty_cycle
 * ************************************************************************************************
 * Summary:
 * Reads the duty cycle of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator (1...n)
 * 
 * Returns:
 * 	duty_cycle	= reads the compare value
 *
 * Description:
 * Reads the duty cycle compare value of a specific PWM generator considering the different
 * registers and bit locations on different devices
 * ***********************************************************************************************/

uint16_t hspwm_get_duty_cycle(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset = 0;
volatile uint16_t reg_buf = 0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch(sub_index){
        
        case 1: // Primary duty cycle register
            regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
            reg_buf = (*regptr & REG_PDCx_VALID_BIT_MSK);
            break;

        case 2: // Secondary duty cycle register
            regptr  = (volatile uint16_t *)&SDC1 + reg_offset;
            reg_buf = (*regptr & REG_SDCx_VALID_BIT_MSK);
            break;

        default:    // error
            return(0);
            break;
    }

	return(reg_buf);

}

/*@@hspwm_set_phase
 * ************************************************************************************************
 * Summary:
 * Sets the phase/period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator (1...n)
 *  sub_index   = selected the primary or secondary phase register
 *	period		= sets the compare value
 *
 * Description:
 * Sets the phase/period compare value of a specific PWM generator considering the different
 * registers and bit locations on different devices
 * ***********************************************************************************************/

uint16_t hspwm_set_phase(uint16_t index, uint16_t sub_index, uint16_t period)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset = 0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch(sub_index){
        
        case 0: // Primary + Secondary phase/period register
            regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
            *regptr = (period & REG_PHASEx_VALID_BIT_MSK);
            
            regptr  = (volatile uint16_t *)&SPHASE1 + reg_offset;
            *regptr = (period & REG_SPHASEx_VALID_BIT_MSK);
            break;

        case 1: // Primary phase/period register
            regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
            *regptr = (period & REG_PHASEx_VALID_BIT_MSK);
            break;

        case 2: // Secondary phase/period register
            regptr  = (volatile uint16_t *)&SPHASE1 + reg_offset;
            *regptr = (period & REG_SPHASEx_VALID_BIT_MSK);
            break;

        default: // Error
            return(0);
            break;
    }

	return(1);

}

/*@@hspwm_get_duty_cycle
 * ************************************************************************************************
 * Summary:
 * Reads the phase/period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator (1...n)
 *  sub_index   = selects the primary or secondary phase register
 * 
 * Returns:
 * 	phase       = reads the compare value
 *
 * Description:
 * Reads the phase/period compare value of a specific PWM generator considering the different
 * registers and bit locations on different devices
 * ***********************************************************************************************/

uint16_t hspwm_get_phase(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset = 0;
volatile uint16_t reg_buf = 0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch(index){
        case 1: // Primary phase/period register
            regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
            reg_buf = *regptr;
            break;
            
        case 2: // Secondary phase/period register
            regptr  = (volatile uint16_t *)&SPHASE1 + reg_offset;
            reg_buf = *regptr;
            break;
            
        default:
            return(0);
            break;
    }
	return(reg_buf);

}

/*@@hspwm_set_sevtcmp_value
 * *****************************************************************************************************
 * Summary:
 * Sets the leading edge blanking period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM time base (1=primary or 2=secondary)
 *	period		= sets the compare value
 *
 * Description:
 * Sets the special event compare value of a specific time base considering the different
 * registers and bit locations on different devices
 * *****************************************************************************************************/

uint16_t hspwm_set_sevtcmp_value(uint16_t index, uint16_t period)
{

volatile uint16_t *regptr;
	
    switch(index){
        
    case 1: // Primary time base special event compare register
        
        regptr  = (volatile uint16_t *)&SEVTCMP;
        *regptr = (period & REG_SEVTCMP_VALID_BIT_MSK);
        break;

    #if defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)
        
    case 2: // Secondary time base special event compare register
        
        regptr  = (volatile uint16_t *)&SSEVTCMP;
        *regptr = (period & REG_SSEVTCMP_VALID_BIT_MSK);
        break;

    #endif
        
    default: 
        return(0); 
        break;
    }

	return(1);

}

/*@@hspwm_get_sevtcmp_value
 * *****************************************************************************************************
 * Summary:
 * Reads the leading edge blanking period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM time base (1=primary or 2=secondary)
 *	
 * Returns:
 *  period		= reads the compare value
 *
 * Description:
 * Reads the special event compare value of a specific time base considering the different
 * registers and bit locations on different devices
 * *****************************************************************************************************/

uint16_t hspwm_get_sevtcmp_value(uint16_t index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_buf=0;
	
    switch(index){
        
    case 1: // Primary time base special event compare register
        
        regptr  = (volatile uint16_t *)&SEVTCMP;
        reg_buf = (*regptr & REG_SEVTCMP_VALID_BIT_MSK);
        break;

    #if defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)
        
    case 2: // Secondary time base special event compare register
        
        regptr  = (volatile uint16_t *)&SSEVTCMP;
        reg_buf = (*regptr & REG_SSEVTCMP_VALID_BIT_MSK);
        break;

    #endif
        
    default: 
        reg_buf = 0; 
        break;
    }

	return(reg_buf);

}

/*@@hspwm_set_leb_period
 * *****************************************************************************************************
 * Summary:
 * Sets the leading edge blanking period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target pwm generator
 *	period		= configures basic features of the pwm channel selected
 *
 * Description:
 * Sets the leading edge blanking period of a specific channel considering the different
 * resolutions, registers and bit locations on different devices
 * *****************************************************************************************************/

uint16_t hspwm_set_leb_period(uint16_t index, uint16_t period)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);
	
	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    #if defined (__P33SMPS_FJA__) || defined (__P33SMPS_FJ__)

			regptr  = (volatile uint16_t *)&LEBCON1 + reg_offset;
			*regptr = (period & REG_LEBCONx_VALID_BIT_MSK);

    #elif defined (_P33SMPS_UEAA) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

			regptr  = (volatile uint16_t *)&LEBDLY1 + reg_offset;
			*regptr = (period & REG_LEBDLYx_VALID_BIT_MSK);

	#else
		return(0);
	
	#endif


	return(1);

}

/*@@hspwm_get_leb_period
 * *****************************************************************************************************
 * Summary:
 * Reads the leading edge blanking period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target pwm generator
 *
 * Returns:
 *   period     = leading edge blanking period of the selected channel
 * 
 * Description:
 * Reads the leading edge blanking period of a specific channel considering the different
 * resolutions, registers and bit locations on different devices
 * *****************************************************************************************************/

uint16_t hspwm_get_leb_period(uint16_t index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);
	
	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    #if defined (__P33SMPS_FJA__) || defined (__P33SMPS_FJ__)

			regptr  = (volatile uint16_t *)&LEBCON1 + reg_offset;
            reg_buf = (*regptr & REG_LEBCONx_LEB_BIT_MSK);

    #elif defined (__P33SMPS_FJC__) || defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

			regptr  = (volatile uint16_t *)&LEBDLY1 + reg_offset;
			reg_buf = (*regptr & REG_LEBDLYx_VALID_BIT_MSK);

	#else
		reg_buf = 0;
	
	#endif


	return(reg_buf);

}

/*@@hspwm_init_dead_time
 * *****************************************************************************************************
 * Summary:
 * Sets the dead time mode and periods at the leading and falling edge of a specific PWM channel
 *
 * Parameters:
 *	index		= selects the register address range of the target pwm generator
 *	period		= configures basic features of the pwm channel selected
 *
 * Description:
 * The dead time generator allows multiple configurations for the dead time between high- and 
 * low-side switch. This function configures the dead time generator offering all options.
 * *****************************************************************************************************/

uint16_t hspwm_init_dead_time(uint16_t index, uint16_t dead_time_mode, uint16_t rising_edge_period, uint16_t falling_edge_period)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset = 0;
volatile uint16_t reg_buf = 0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);
	
	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    regptr  = (volatile uint16_t *)&DTR1 + reg_offset;
    *regptr = (rising_edge_period & REG_DTRx_VALID_BIT_MSK);

    regptr  = (volatile uint16_t *)&ALTDTR1 + reg_offset;
    *regptr = (falling_edge_period & REG_ALTDTRx_VALID_BIT_MSK);

    regptr  = (volatile uint16_t *)&PWMCON1 + reg_offset;
    reg_buf = *regptr;
    
    switch(dead_time_mode){
    case REG_DTC_POSITIVE: 
            reg_buf &= REG_DTC_POSITIVE_SET_MSK; 
            break;

    case REG_DTC_NEGATIVE: 
            reg_buf &= REG_DTC_NEGATIVE_SET_MSK; 
            break;

    case REG_DTC_COMPENSATED: 
            reg_buf &= REG_DTC_COMPENSATED_SET_MSK; 
            break;

    default:
        reg_buf &= REG_DTC_DISASBLED_SET_MSK; 
        break;
    }
    
    *regptr = (reg_buf & REG_PWMCONx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_get_dead_time
 * *****************************************************************************************************
 * Summary:
 * Reads the dead time period of a specific channel
 *
 * Parameters:
 *	index		= selects the register address range of the target pwm generator
 *
 * Returns:
 *  rising_edge_period  = leading edge dead time of the selected channel
 *  falling_edge_period = falling edge dead time of the selected channel
 *  return              = 1: success, 0: failure
 * 
 * Description:
 * Reads the leading edge blanking period of a specific channel considering the different
 * resolutions, registers and bit locations on different devices.
 * This function uses pointers as parameters. A function call of the following form can 
 * be used to derive both dead times simultaneously:
 * 
 * uint16_t a=0, b=0;
 * hspwm_get_dead_times(1, &a, &b); // REading both dead times of PWM channel #1
 *
 * *****************************************************************************************************/

uint16_t hspwm_get_dead_times(uint16_t index, uint16_t *rising_edge_period, uint16_t *falling_edge_period)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
	
	if (index > HSPWM_CHANNEL_COUNT) return(0);
	
	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    regptr  = (volatile uint16_t *)&DTR1 + reg_offset;
    *rising_edge_period = (*regptr & REG_DTRx_VALID_BIT_MSK);

    regptr  = (volatile uint16_t *)&ALTDTR1 + reg_offset;
    *falling_edge_period = (*regptr & REG_ALTDTRx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_init_channel_timing_complementary
 * *****************************************************************************************************
 * Summary:
 * Initializes a specific PWM generator
 *
 * Parameters:
 *	index				= selects the register address range of the target pwm generator
 *	phase				= sets the frequency period (independent mode) or phase shift (master time base mode)
 *	duty_cycle			= sets the on-time of PWMxH
 *	dead_time			= sets the dead time at the leading edge of PWMxH
 *	alternate_dead_time	= sets the dead time at the falling edge of PWMxH
 *
 * Description:
 * Configures the basic timing settings of a specific PWM generator, configured in complementary 
 * mode or push-pull mode
 * *****************************************************************************************************/

uint16_t hspwm_init_channel_timing_complementary(uint16_t index, uint16_t phase, 
	uint16_t duty_cycle, uint16_t dead_time, uint16_t alternate_dead_time)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
	*regptr = (phase & REG_PHASEx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
	*regptr = (duty_cycle & REG_PDCx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&DTR1 + reg_offset;
	*regptr = (dead_time & REG_DTRx_VALID_BIT_MSK);
	
	regptr  = (volatile uint16_t *)&ALTDTR1 + reg_offset;
	*regptr = (alternate_dead_time & REG_ALTDTRx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_init_channel_timing_redundant
 * *****************************************************************************************************
 * Summary:
 * Initializes a specific PWM generator in redundant mode
 *
 * Parameters:
 *	index				= selects the register address range of the target pwm generator
 *	phase				= sets the frequency period (independent mode) or phase shift (master time base mode)
 *	duty_cycle			= sets the on-time of PWMxH
 *	adc_tigger			= sets the adc trigger counting from the leading edge of PWMxH
 *
 * Description:
 * Configures the basic timing settings of a specific PWM generator, configured in complementary
 * mode or push-pull mode
 * *****************************************************************************************************/

uint16_t hspwm_init_channel_timing_redundant(uint16_t index, uint16_t phase, 
		uint16_t duty_cycle)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
    *regptr = (phase & REG_PHASEx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
	*regptr = (duty_cycle & REG_PDCx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_init_channel_timing_push_pull
 * *****************************************************************************************************
 * Summary:
 * Initializes a specific PWM generator in push-pull mode
 *
 * Parameters:
 *	index				= selects the register address range of the target pwm generator
 *	phase				= sets the frequency period (independent mode) or phase shift (master time base mode)
 *	duty_cycle			= sets the on-time of PWMxH
 *	dead_time			= sets the dead time at the leading edge of PWMxH
 *	alternate_dead_time	= sets the dead time at the falling edge of PWMxH
 *
 * Description:
 * Configures the basic timing settings of a specific PWM generator, configured in complementary
 * mode or push-pull mode
 * *****************************************************************************************************/

uint16_t hspwm_init_channel_timing_push_pull(uint16_t index, uint16_t phase, 
		uint16_t duty_cycle, uint16_t dead_time, uint16_t alternate_dead_time)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
	*regptr = (phase & REG_PHASEx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
	*regptr = (duty_cycle & REG_PDCx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&DTR1 + reg_offset;
	*regptr = (dead_time & REG_DTRx_VALID_BIT_MSK);
	
	regptr  = (volatile uint16_t *)&ALTDTR1 + reg_offset;
	*regptr = (alternate_dead_time & REG_ALTDTRx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_init_channel_timing_true_independent
 * *****************************************************************************************************
 * Summary:
 * Initializes a specific PWM generator
 *
 * Parameters:
 *	index			= selects the register address range of the target pwm generator
 *	sub_index		= selects either primary (pin PWMxH) or secondary (pin PWMxL) output
 *	phase			= sets the frequency period (independent mode) or phase shift (master time base mode)
 *	duty_cycle		= sets the on-time of PWMxH
 *
 * Description:
 * Configures the basic timing settings of a specific PWM generator, pre-configured in complementary 
 * mode or push-pull mode
 * *****************************************************************************************************/

uint16_t hspwm_init_channel_timing_true_independent(uint16_t index, uint16_t sub_index, 
		uint16_t phase, uint16_t duty_cycle)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	switch(sub_index)
	{
		case 1:
            regptr  = (volatile uint16_t *)&PHASE1 + reg_offset;
            *regptr = (phase & REG_PHASEx_VALID_BIT_MSK);

            regptr  = (volatile uint16_t *)&PDC1 + reg_offset;
            *regptr = (duty_cycle & REG_PDCx_VALID_BIT_MSK);

			break;
			
		case 2:
            regptr  = (volatile uint16_t *)&SPHASE1 + reg_offset;
            *regptr = (phase & REG_SPHASEx_VALID_BIT_MSK);

            regptr  = (volatile uint16_t *)&SDC1 + reg_offset;
            *regptr = (duty_cycle & REG_SDCx_VALID_BIT_MSK);

			break;

		default:	
            return(0);
			
	}

	return(1);

}

/*@@hspwm_init_channel_adc_trigger
 * *****************************************************************************************************
 * Summary:
 * Initializes the trigger configuration of a specific PWM generator
 *
 * Parameters:
 *	index				 = selects the register address range of the target PWM generator
 *	regTRGCON			 = sets PWM TRIGGER CONTROL register value
 *	primary_adc_tigger	 = sets the primary ADC trigger counting from the leading edge of PWMxH
 *	secondary_adc_tigger = sets the secondary ADC trigger counting from the leading edge of PWMxH
 *
 * Description:
 * Configures the basic ADC trigger generated by the PWM module. The basic configuration set by 
 * TRGCON will affect the complete channel (both outputs PWMxH + PWM+L). The ADC trigger compare
 * value, however, can be set individually by using the sub_index and ADC parameters.
 * *****************************************************************************************************/

uint16_t hspwm_init_channel_adc_trigger(uint16_t index, uint16_t regTRGCON, 
			uint16_t primary_adc_trigger, uint16_t secondary_adc_trigger)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

	regptr  = (volatile uint16_t *)&TRGCON1 + reg_offset;
	*regptr = (regTRGCON & REG_TRGCON_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&TRIG1 + reg_offset;
	*regptr = (primary_adc_trigger & REG_TRIGx_VALID_BIT_MSK);

	regptr  = (volatile uint16_t *)&STRIG1 + reg_offset;
	*regptr = (secondary_adc_trigger & REG_STRIGx_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_set_adc_trigger_config
 * *****************************************************************************************************
 * Summary:
 * Sets the trigger configuration of a specific PWM generator
 *
 * Parameters:
 *	index				 = selects the register address range of the target PWM generator
 *	sub_index            = selects the primary or secondary ADC trigger register
 *	adc_tigger_config    = sets the ADC trigger configuration register value
 *
 * Description:
 * The ADC trigger configuration of primary or secondary trigger is set individually selected via 
 * the sub_index.
 * *****************************************************************************************************/

uint16_t hspwm_set_adc_trigger_config(uint16_t index, uint16_t adc_tigger_config)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    regptr  = (volatile uint16_t *)&TRGCON1 + reg_offset;
    *regptr = (adc_tigger_config & REG_TRGCON_VALID_BIT_MSK);

	return(1);

}

/*@@hspwm_set_adc_trigger
 * *****************************************************************************************************
 * Summary:
 * Sets the trigger location of a specific PWM generator
 *
 * Parameters:
 *	index				 = selects the register address range of the target PWM generator
 *	sub_index            = selects the primary or secondary ADC trigger register
 *	adc_tigger          = sets the ADC trigger counter compare value
 *
 * Description:
 * The ADC trigger compare value of primary or secondary trigger is set individually selected via 
 * the sub_index.
 * *****************************************************************************************************/

uint16_t hspwm_set_adc_trigger(uint16_t index, uint16_t sub_index, uint16_t adc_trigger)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch(sub_index){
        case 1:
            regptr  = (volatile uint16_t *)&TRIG1 + reg_offset;
            *regptr = (adc_trigger & REG_TRIGx_VALID_BIT_MSK);
            break;
            
        case 2:
            regptr  = (volatile uint16_t *)&STRIG1 + reg_offset;
            *regptr = (adc_trigger & REG_STRIGx_VALID_BIT_MSK);
            break;
        
        default:
            return(0);
            break;
    }

	return(1);

}

/*@@hspwm_get_adc_trigger
 * *****************************************************************************************************
 * Summary:
 * Reads the trigger location of a specific PWM generator
 *
 * Parameters:
 *	index				 = selects the register address range of the target PWM generator
 *	sub_index            = selects the primary or secondary ADC trigger register

 * Returns:
 *  adc_tigger           = Primary or secondary ADC trigger counter compare value
 *
 * Description:
 * The ADC trigger compare value of primary or secondary trigger is read individually selected via 
 * the sub_index.
 * *****************************************************************************************************/

uint16_t hspwm_get_adc_trigger(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);

    switch(sub_index){
        case 1:
            regptr  = (volatile uint16_t *)&TRIG1 + reg_offset;
            reg_buf = (*regptr & REG_TRIGx_VALID_BIT_MSK);
            break;
            
        case 2:
            regptr  = (volatile uint16_t *)&STRIG1 + reg_offset;
            reg_buf = (*regptr & REG_STRIGx_VALID_BIT_MSK);
            break;
        
        default:
            return(0);
            break;
    }

	return(reg_buf);

}

/*@@hspwm_channel_power_up()
 * *****************************************************************************************************
 * Summary:
 * Turns on the base power of the PWM module 
 *
 * Parameters: 
 *   index				 = selects the register address range of the target PWM generator
 *
 * Description:
 * 
 * *****************************************************************************************************/

uint16_t hspwm_channel_power_up(uint16_t index)
{
	 
	if (index > HSPWM_CHANNEL_COUNT) return(0);

    switch(index){
        #if (HSPWM_CHANNEL_COUNT >= 1)
        case 1: _PWM1MD = 0; Nop(); break;		// Turn on power to PWM channel #1
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 2)
        case 2: _PWM2MD = 0; Nop(); break;		// Turn on power to PWM channel #2
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 3)
        case 3: _PWM3MD = 0; Nop(); break;		// Turn on power to PWM channel #3
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 4)
        case 4: _PWM4MD = 0; Nop(); break;		// Turn on power to PWM channel #4
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 5)
        case 5: _PWM5MD = 0; Nop(); break;		// Turn on power to PWM channel #5
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 6)
        case 6: _PWM6MD = 0; Nop(); break;		// Turn on power to PWM channel #6
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 7)
        case 7: _PWM7MD = 0; Nop(); break;		// Turn on power to PWM channel #7
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 8)
        case 8: _PWM8MD = 0; Nop(); break;		// Turn on power to PWM channel #8
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 9)
        case 9: _PWM9MD = 0; Nop(); break;		// Turn on power to PWM channel #9
        #endif
    }
	 
	return(1);
	 
}

/*@@hspwm_channel_power_down()
 * *****************************************************************************************************
 * Summary:
 * Turns off the base power of the PWM module 
 *
 * Parameters: 
 *   index				 = selects the register address range of the target PWM generator
 *
 * Description:
 * 
 * *****************************************************************************************************/

uint16_t hspwm_channel_power_down(uint16_t index)
{
	 
	if (index > HSPWM_CHANNEL_COUNT) return(0);

    switch(index){
        #if (HSPWM_CHANNEL_COUNT >= 1)
        case 1: _PWM1MD = 1; Nop(); break;		// Turn off power to PWM channel #1
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 2)
        case 2: _PWM2MD = 1; Nop(); break;		// Turn off power to PWM channel #2
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 3)
        case 3: _PWM3MD = 1; Nop(); break;		// Turn off power to PWM channel #3
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 4)
        case 4: _PWM4MD = 1; Nop(); break;		// Turn off power to PWM channel #4
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 5)
        case 5: _PWM5MD = 1; Nop(); break;		// Turn off power to PWM channel #5
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 6)
        case 6: _PWM6MD = 1; Nop(); break;		// Turn off power to PWM channel #6
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 7)
        case 7: _PWM7MD = 1; Nop(); break;		// Turn off power to PWM channel #7
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 8)
        case 8: _PWM8MD = 1; Nop(); break;		// Turn off power to PWM channel #8
        #endif
        #if (HSPWM_CHANNEL_COUNT >= 9)
        case 9: _PWM8MD = 1; Nop(); break;		// Turn off power to PWM channel #9
        #endif
        default:
            return(0); 
            break;
    }
	 
	return(1);
	 
}

/*@@hspwm_module_power_up()
 * *****************************************************************************************************
 * Summary:
 * Turns on the base power of the PWM module 
 *
 * Parameters: (none)
 *
 * Description:
 * 
 * *****************************************************************************************************/

uint16_t hspwm_module_power_up(void)
{
	 
	_PWMMD = 0;		// Turn on power to PWM module
	 
	return(1 - _PWMMD);
	 
}

/*@@hspwm_module_power_down()
 * *****************************************************************************************************
 * Summary:
 * Turns on the base power of the PWM module 
 *
 * Parameters: (none)
 *
 * Description:
 * 
 * *****************************************************************************************************/

uint16_t hspwm_module_power_down(void)
{
	 
	_PWMMD = 1;		// Turn off power to PWM module
	 
	return(1);
	 
}

/*@@hspwm_module_enable()
 * *****************************************************************************************************
 * Summary:
 * Enables the PWM generator
 *
 * Parameters: (none)
 *
 * Description:
 * Enables the PWM module with previous configurations. This routine also incorporates an errata
 * on certain dsPIC GS devices encountering glitches when PWM is turned on.
 * *****************************************************************************************************/

uint16_t hspwm_module_enable(void)
{
	 
	_PTEN = 1;		// Enable PWM module
	 
	return(_PTEN);
	 
}

/*@@hspwm_module_disable()
 * *****************************************************************************************************
 * Summary:
 * Disables the PWM generator
 *
 * Parameters: (none)
 *
 * Description:
 * Disables the entire PWM module, which will also affect the port registers. Fault states 
 * at certain pins will be lost as every pin will be re-configured as GPIO.
 * *****************************************************************************************************/

uint16_t hspwm_module_disable(void)
{

	_PTEN = 0;		// Disable PWM module

	return(1 - _PTEN);

}

/*@@hspwm_ovr_output_hold()
 * *****************************************************************************************************
 * Summary:
 * Overrides specific PWM generator outputs
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator
 *  sub_index   = selects the high- or low-side output of the selected PWM generator
 *
 *                sub_index = 0:    PWMxH and PWMxL will be disabled
 *                sub_index = 1:    PWMxH will be disabled
 *                sub_index = 2:    PWMxL will be disabled
 *  
 * Description:
 * Sets the override state of the specific PWM generator high- or low-side output.
 * (selected PWM output will be disabled)
 * *****************************************************************************************************/

uint16_t hspwm_ovr_output_hold(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);
	regptr  = (volatile uint16_t *)&IOCON1 + reg_offset;
	reg_buf = *regptr;
	
	switch(sub_index)
	{
		case 0:		// Complementary/Redundant/Push-Pull Outputs are hold
			*regptr = ((reg_buf | REG_OVRENx_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 1:		// PWMxH Output is hold only
			*regptr = ((reg_buf | REG_OVRENH_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 2:		// PWMxL Output is hold only
			*regptr = ((reg_buf | REG_OVRENL_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		default:
			return(0);
			break;
	}

	return(1);
}

/*@@hspwm_ovr_output_release()
 * *****************************************************************************************************
 * Summary:
 * Releases specific PWM generator outputs
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator
 *  sub_index   = selects the high- or low-side output of the selected PWM generator
 *
 *                sub_index = 0:    PWMxH and PWMxL will be enabled
 *                sub_index = 1:    PWMxH will be enabled
 *                sub_index = 2:    PWMxL will be enabled
 *  
 * Description:
 * Resets the override state of the specific PWM generator high- or low-side output.
 * (selected PWM output will be enabled)
 * *****************************************************************************************************/

uint16_t hspwm_ovr_output_release(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);
	regptr  = (volatile uint16_t *)&IOCON1 + reg_offset;
	reg_buf = *regptr;
	
	switch(sub_index)
	{
		case 0:		// Complementary Outputs are released
			*regptr = ((reg_buf & REG_OVRENx_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 1:		// PWMxH Output is released only
			*regptr = ((reg_buf & REG_OVRENH_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 2:		// PWMxL Output is released only
			*regptr = ((reg_buf & REG_OVRENL_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		default:
			return(0);
			break;
	}

	return(1);
}

/*@@hspwm_channel_output_disable()
 * *****************************************************************************************************
 * Summary:
 * Disables specific PWM generator outputs
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator
 *  sub_index   = selects the high- or low-side output of the selected PWM generator
 *
 *                sub_index = 0:    PWMxH and PWMxL will be disabled
 *                sub_index = 1:    PWMxH will be disabled
 *                sub_index = 2:    PWMxL will be disabled
 *  
 * Description:
 * Clears the PENH/PENL state of the specific PWM generator high- or low-side output.
 * (selected PWM output will be disabled)
 * *****************************************************************************************************/

uint16_t hspwm_channel_output_disable(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);
	regptr  = (volatile uint16_t *)&IOCON1 + reg_offset;
	reg_buf = *regptr;
	
	switch(sub_index)
	{
		case 0:		// Complementary Outputs are switched to GPIO control
			*regptr = ((reg_buf & REG_PWMx_IO_CONTROL_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 1:		// PWMxH Output is switched to GPIO control
			*regptr = ((reg_buf & REG_PWMH_IO_CONTROL_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 2:		// PWMxL Output is switched to GPIO control
			*regptr = ((reg_buf & REG_PWML_IO_CONTROL_CLR_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		default:
			return(0);
			break;
	}

	return(1);
}

/*@@hspwm_channel_output_enable()
 * *****************************************************************************************************
 * Summary:
 * Releases specific PWM generator outputs
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator
 *  sub_index   = selects the high- or low-side output of the selected PWM generator
 *
 *                sub_index = 0:    PWMxH and PWMxL will be enabled
 *                sub_index = 1:    PWMxH will be enabled
 *                sub_index = 2:    PWMxL will be enabled
 *  
 * Description:
 * Sets the PENH/PENL state of the specific PWM generator high- or low-side output.
 * (selected PWM output will be enabled)
 * *****************************************************************************************************/

uint16_t hspwm_channel_output_enable(uint16_t index, uint16_t sub_index)
{

volatile uint16_t *regptr;
volatile uint16_t reg_offset=0;
volatile uint16_t reg_buf=0;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);
	regptr  = (volatile uint16_t *)&IOCON1 + reg_offset;
	reg_buf = *regptr;
	
	switch(sub_index)
	{
		case 0:		// Complementary Outputs are released
			*regptr = ((reg_buf | REG_PWMx_IO_CONTROL_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 1:		// PWMxH Output is released only
			*regptr = ((reg_buf | REG_PWMH_IO_CONTROL_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		case 2:		// PWMxL Output is released only
			*regptr = ((reg_buf | REG_PWML_IO_CONTROL_SET_MASK) & REG_IOCONx_VALID_BIT_MSK);
			break;
		default:
			return(0);
			break;
	}

	return(1);
}

/*@@hspwm_write_wp_fclcon()
 * *****************************************************************************************************
 * Summary:
 * The dsPIC33EP GS FCLCON write protection unlock sequence to write to FCLCON
 *
 * Parameters:
 *	index		= selects the register address range of the target PWM generator
 *
 * Description:
 * The dsPIC33EP GS families of devices support write protection features of certain PWM 
 * registers. This function has to be called to unlock registers before writing to any
 * protected register.
 * PWM register protection has to be enabled by setting the configuration bit 
 * #pragma config PWMLOCK = ON.
 * *****************************************************************************************************/
#if defined (__P33SMPS_EP2__) || defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

uint16_t hspwm_write_wp_fclcon(uint16_t index, uint16_t regFCLCON)
{

// volatile uint16_t *regptr;
// volatile uint16_t reg_offset=0, reg_buf;

	if (index > HSPWM_CHANNEL_COUNT) return(0);

    // TODO: complete UNLOAD and LOCK sequence for PWMLOCK registers

/*
	reg_offset = ((index-1) * HSPWM_CHANNEL_REG_OFFSET);
	regptr  = (volatile uint16_t *)&FCLCON1 + reg_offset;
    reg_buf = regFCLCON;
    
	asm volatile
	(	"mov #0xabcd, w10  ; Load first unlock key to w10 register  \n"
        "mov #0x4321, w11  ; Load second unlock key to w11 register \n"
        "mov #" + reg_buf + ", w0  ; Load desired value of FCLCONs register in w0 \n"
        "mov w10, PWMKEY ; Write first unlock key to PWMKEY register \n"
        "mov w11, PWMKEY ; Write second unlock key to PWMKEY register \n"
        "mov w0, " + regptr + " ; Write desired value to FCLCON1 register \n"
	);    

*/
    return(1);

}
/* *****************************************************************************************************
 * *****************************************************************************************************/

#endif
