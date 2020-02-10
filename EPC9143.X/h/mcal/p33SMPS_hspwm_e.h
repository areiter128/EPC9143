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

#ifndef _P33SMPS_SMPS_PWM_H_
#define _P33SMPS_SMPS_PWM_H_

#include <stdint.h>

/*@@p33SMPS_hspwm.h
 * ************************************************************************************************
 * Summary:
 * Header file with additional defines for the dsPIC33FJ/EPxxGS-PWM SFRs
 *
 * Description:
 * The SMPS PWM module offers a large number of registers and configuration options. This additional
 * header file contains defines for all required settings. This additional
 * header file contains defines for all required bit-settings of all related registers.
 * This file is an additional header file on top of the generic device header file.
 * ***********************************************************************************************/

// Prototypes
extern inline uint16_t hspwm_reset_pwm_io(void);

extern inline uint16_t hspwm_init_mtb(uint16_t index, uint16_t regXTCON, uint16_t regXTCON2, uint16_t period);
extern inline uint16_t hspwm_init_channel(uint16_t index, uint16_t regPWMCON, uint16_t regIOCON, uint16_t regFCLCON, uint16_t regLEBCON);
extern inline uint16_t hspwm_init_channel_timing_complementary(uint16_t index, uint16_t phase, uint16_t duty_cycle, uint16_t dead_time, uint16_t alternate_dead_time);
extern inline uint16_t hspwm_init_channel_timing_redundant(uint16_t index, uint16_t phase, uint16_t duty_cycle);
extern inline uint16_t hspwm_init_channel_timing_push_pull(uint16_t index, uint16_t phase, uint16_t duty_cycle, uint16_t dead_time, uint16_t alternate_dead_time);
extern inline uint16_t hspwm_init_channel_timing_true_independent(uint16_t index, uint16_t sub_index, uint16_t phase, uint16_t duty_cycle);
extern inline uint16_t hspwm_init_channel_adc_trigger(uint16_t index, uint16_t regTRGCON, uint16_t primary_adc_trigger, uint16_t secondary_adc_trigger);
extern inline uint16_t hspwm_set_adc_trigger_config(uint16_t index, uint16_t adc_tigger_config);
extern inline uint16_t hspwm_init_dead_time(uint16_t index, uint16_t dead_time_mode, uint16_t rising_edge_period, uint16_t falling_edge_period);

extern inline uint16_t hspwm_get_dead_times(uint16_t index, uint16_t *rising_edge_period, uint16_t *falling_edge_period);
extern inline uint16_t hspwm_set_mtb_period(uint16_t index, uint16_t period);
extern inline uint16_t hspwm_get_mtb_period(uint16_t index);
extern inline uint16_t hspwm_set_phase(uint16_t index, uint16_t sub_index, uint16_t period);
extern inline uint16_t hspwm_get_phase(uint16_t index, uint16_t sub_index);
extern inline uint16_t hspwm_set_duty_cycle(uint16_t index, uint16_t sub_index, uint16_t duty_cycle);
extern inline uint16_t hspwm_get_duty_cycle(uint16_t index, uint16_t sub_index);
extern inline uint16_t hspwm_set_leb_period(uint16_t index, uint16_t period);
extern inline uint16_t hspwm_get_leb_period(uint16_t index);
extern inline uint16_t hspwm_set_sevtcmp_value(uint16_t index, uint16_t period);
extern inline uint16_t hspwm_get_sevtcmp_value(uint16_t index);
extern inline uint16_t hspwm_set_adc_trigger(uint16_t index, uint16_t sub_index, uint16_t adc_trigger);
extern inline uint16_t hspwm_get_adc_trigger(uint16_t index, uint16_t sub_index);

extern inline uint16_t hspwm_channel_power_up(uint16_t index);
extern inline uint16_t hspwm_channel_power_down(uint16_t index);

extern inline uint16_t hspwm_module_power_up(void);
extern inline uint16_t hspwm_module_power_down(void);
extern inline uint16_t hspwm_module_enable(void);
extern inline uint16_t hspwm_module_disable(void);

extern inline uint16_t hspwm_ovr_output_hold(uint16_t index, uint16_t sub_index);
extern inline uint16_t hspwm_ovr_output_release(uint16_t index, uint16_t sub_index);
extern inline uint16_t hspwm_channel_output_disable(uint16_t index, uint16_t sub_index);
extern inline uint16_t hspwm_channel_output_enable(uint16_t index, uint16_t sub_index);

#if defined (__P33SMPS_EP__)

  extern inline uint16_t hspwm_write_wp_fclcon(uint16_t index, uint16_t regFCLCON);

#endif

// Device-specific Defines

#if   defined (__P33SMPS_FJA001__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif   defined (__P33SMPS_FJ101__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif   defined (__P33SMPS_FJA101__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ102__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJA102__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ202__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif   defined (__P33SMPS_FJA202__)
	#define HSPWM_CHANNEL_COUNT	2
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif   defined (__P33SMPS_FJ302__)
	#define HSPWM_CHANNEL_COUNT	3
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ402__)
	#define HSPWM_CHANNEL_COUNT	3
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ404__)
	#define HSPWM_CHANNEL_COUNT	3
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ502__)
	#define HSPWM_CHANNEL_COUNT	4
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ504__)
	#define HSPWM_CHANNEL_COUNT	4
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ406__)
	#define HSPWM_CHANNEL_COUNT	6
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ606__)
	#define HSPWM_CHANNEL_COUNT	6
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ608__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ610__)
	#define HSPWM_CHANNEL_COUNT	9
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ406__)
	#define HSPWM_CHANNEL_COUNT	6
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ606__)
	#define HSPWM_CHANNEL_COUNT	6
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ608__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_FJ610__)
	#define HSPWM_CHANNEL_COUNT	9
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010

#elif defined (__P33SMPS_EP202__)
	#define HSPWM_CHANNEL_COUNT	3
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP502__)
	#define HSPWM_CHANNEL_COUNT	5
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP504__)
	#define HSPWM_CHANNEL_COUNT	5
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP505__)
	#define HSPWM_CHANNEL_COUNT	5
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP506__)
	#define HSPWM_CHANNEL_COUNT	5
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010

#elif defined (__P33SMPS_EP702__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP704__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP705__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP706__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP708__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP804__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP805__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP806__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010
#elif defined (__P33SMPS_EP808__)
	#define HSPWM_CHANNEL_COUNT	8
	#define HSPWM_CHANNEL_REG_OFFSET	0x0010

#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Bit Masks for unimplemented register bits for all PWM registers
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if defined (__P33SMPS_FJ__)
// YGAR family of devices

    // TODO: Review YGAR bit patterns 
    #warning === BIT PATTERNS NEED REVIEW, TLAH SETTINGS ARE CORRECT ===
  
#define REG_PTCON_VALID_BIT_MSK            0b0100000001000000      // Primary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK           0b1111111111111000      // Primary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK          0b0000000000111000      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK           0b0000000000000000      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK			0b0000000000000000      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK			0b0000000000000111      // Leading Edge Blanking register
#define REG_LEBCONx_LEB_BIT_MSK         0b0000001111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK          0b0000000000000111      // Primary Special Event compare register
#define REG_SEVTCMP_BIT_MASK            0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK            0b0000000000000000      // PTPER register
#define REG_MDC_VALID_BIT_MSK              0b0000000000000000      // Master Duty Cycle register
#define REG_PHASEx_VALID_BIT_MSK           0b0000000000000000      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK          0b0000000000000000      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK             0b1100000000000000      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK          0b1100000000000000      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK           0b0000111101000000      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK            0b0000000000000111      // Primary Trigger register
#define REG_TRIGx_BIT_MASK              0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution
#define REG_STRIGx_VALID_BIT_MSK           0b0000000000000111      // Secondary Trigger register
#define REG_STRIGx_BIT_MASK             0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PWMCAPx_VALID_BIT_MSK          0b0000000000000111      // PWMxH Capture register        

  
#elif defined (__P33SMPS_FJC__)
// UEAA family of devices

  // TODO: Review UEAA bit patterns 
  #warning === BIT PATTERNS NEED REVIEW, TLAH SETTINGS ARE CORRECT ===

#define REG_PTCON_VALID_BIT_MSK            0b0100000000000000      // Primary PWM time base configuration register
#define REG_STCON_VALID_BIT_MSK            0b1110000000000000      // Secondary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK           0b1111111111111000      // Primary PWM clock divider register
#define REG_STCON2_VALID_BIT_MSK           0b1111111111111000      // Secondary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK          0b0000000000010000      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK           0b0000000000000000      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK			0b0000000000000000      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK			0b0000001111000000      // Leading Edge Blanking register
#define REG_LEBDLYx_VALID_BIT_MSK          0b1111000000000111      // Leading Edge Blanking Period register
#define REG_LEBDLYx_LEB_BIT_MSK         0b0000111111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK          0b0000000000000111      // Primary Special Event compare register
#define REG_SEVTCMP_BIT_MASK            0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution
#define REG_SSEVTCMP_UNUSED_MSK         0b0000000000000111      // Secondary Special Event compare register
#define REG_SSEVTCMP_VALID_BIT_MSK           0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK            0b0000000000000000      // PTPER register
#define REG_STPER_VALID_BIT_MSK            0b0000000000000000      // STPER register
#define REG_MDC_VALID_BIT_MSK              0b0000000000000000      // Master Duty Cycle register
#define REG_CHOP_VALID_BIT_MSK             0b0111110000000111      // On-Time Chopping Frequency register
#define REG_PHASEx_VALID_BIT_MSK           0b0000000000000000      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK          0b0000000000000000      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK             0b1100000000000000      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK          0b1100000000000000      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK           0b0000111101000000      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK            0b0000000000000111      // Primary Trigger register
#define REG_TRIGx_BIT_MASK              0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution
#define REG_STRIGx_VALID_BIT_MSK           0b0000000000000111      // Secondary Trigger register
#define REG_STRIGx_BIT_MASK             0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_AUXCONx_VALID_BIT_MSK          0b0011000011000000      // PWMx generator auxiliary control register
#define REG_PWMCAPx_VALID_BIT_MSK          0b0000000000000111      // PWMxH Capture register        


#elif defined (__P33SMPS_FJA__)
// UEAG family of devices
  
  // TODO: Review UAEG bit patterns 
  #warning === BIT PATTERNS NEED REVIEW, TLAH SETTINGS ARE CORRECT ===

#define REG_PTCON_VALID_BIT_MSK            0b0100000001000000      // Primary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK           0b1111111111111000      // Primary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK          0b0000000000111000      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK           0b0000000000000000      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK			0b0000000000000000      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK			0b0000000000000111      // Leading Edge Blanking register
#define REG_LEBCONx_LEB_BIT_MSK         0b0000001111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK          0b0000000000000111      // Primary Special Event compare register
#define REG_SEVTCMP_BIT_MASK            0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK            0b0000000000000000      // PTPER register
#define REG_MDC_VALID_BIT_MSK              0b0000000000000000      // Master Duty Cycle register
#define REG_CHOP_VALID_BIT_MSK             0b0111110000000111      // On-Time Chopping Frequency register
#define REG_PHASEx_VALID_BIT_MSK           0b0000000000000000      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK          0b0000000000000000      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK             0b0000000000000000      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK             0b1100000000000000      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK          0b1100000000000000      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK           0b0000111101000000      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK            0b0000000000000111      // Primary Trigger register
#define REG_TRIGx_BIT_MASK              0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution
#define REG_STRIGx_VALID_BIT_MSK           0b0000000000000111      // Secondary Trigger register
#define REG_STRIGx_BIT_MASK             0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_AUXCONx_VALID_BIT_MSK          0b0011111111000000      // PWMx generator auxiliary control register
#define REG_PWMCAPx_VALID_BIT_MSK          0b0000000000000111      // PWMxH Capture register        


#elif defined (__P33SMPS_EP2__)
// TLAL family of devices

#define REG_PWMKEY_VALID_BIT_MSK        0b1111111111111111      // PWM Lock/Unlock Sequence register

#define REG_PTCON_VALID_BIT_MSK         0b1011111111111111      // Primary PWM time base configuration register
#define REG_STCON_VALID_BIT_MSK         0b0001111111111111      // Secondary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK        0b0000000000000111      // Primary PWM clock divider register
#define REG_STCON2_VALID_BIT_MSK        0b0000000000000111      // Secondary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK       0b1111111111001111      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK        0b1111111111111111      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK   	0b1111111111111111      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK		0b1111110000111111      // Leading Edge Blanking register
#define REG_LEBDLYx_VALID_BIT_MSK       0b0000111111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK       0b1111111111111000      // Primary Special Event compare register
#define REG_SSEVTCMP_VALID_BIT_MSK      0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK         0b1111111111111111      // PTPER register
#define REG_STPER_VALID_BIT_MSK         0b1111111111111111      // STPER register
#define REG_MDC_VALID_BIT_MSK           0b1111111111111111      // Master Duty Cycle register
#define REG_CHOP_VALID_BIT_MSK          0b1000001111111000      // On-Time Chopping Frequency register
#define REG_PHASEx_VALID_BIT_MSK        0b1111111111111111      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK       0b1111111111111111      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK          0b0011111111111111      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK       0b0011111111111111      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK        0b1111000010111111      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK         0b1111111111111000      // Primary Trigger register
#define REG_STRIGx_VALID_BIT_MSK        0b1111111111111000      // Secondary Trigger register

#define REG_AUXCONx_VALID_BIT_MSK       0b1100111100111111      // PWMx generator auxiliary control register
#define REG_PWMCAPx_VALID_BIT_MSK       0b1111111111111000      // PWMxH Capture register        

#elif defined (__P33SMPS_EP5__)
// TLAH family of devices

#define REG_PWMKEY_VALID_BIT_MSK        0b1111111111111111      // PWM Lock/Unlock Sequence register

#define REG_PTCON_VALID_BIT_MSK         0b1011111111111111      // Primary PWM time base configuration register
#define REG_STCON_VALID_BIT_MSK         0b0001111111111111      // Secondary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK        0b0000000000000111      // Primary PWM clock divider register
#define REG_STCON2_VALID_BIT_MSK        0b0000000000000111      // Secondary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK       0b1111111111001111      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK        0b1111111111111111      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK   	0b1111111111111111      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK		0b1111110000111111      // Leading Edge Blanking register
#define REG_LEBDLYx_VALID_BIT_MSK       0b0000111111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK       0b1111111111111000      // Primary Special Event compare register
#define REG_SSEVTCMP_VALID_BIT_MSK      0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK         0b1111111111111111      // PTPER register
#define REG_STPER_VALID_BIT_MSK         0b1111111111111111      // STPER register
#define REG_MDC_VALID_BIT_MSK           0b1111111111111111      // Master Duty Cycle register
#define REG_CHOP_VALID_BIT_MSK          0b1000001111111000      // On-Time Chopping Frequency register
#define REG_PHASEx_VALID_BIT_MSK        0b1111111111111111      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK       0b1111111111111111      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK          0b0011111111111111      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK       0b0011111111111111      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK        0b1111000010111111      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK         0b1111111111111000      // Primary Trigger register
#define REG_STRIGx_VALID_BIT_MSK        0b1111111111111000      // Secondary Trigger register

#define REG_AUXCONx_VALID_BIT_MSK       0b1100111100111111      // PWMx generator auxiliary control register
#define REG_PWMCAPx_VALID_BIT_MSK       0b1111111111111000      // PWMxH Capture register        

#elif defined (__P33SMPS_EP7__)
// TLAH family of devices

#define REG_PWMKEY_VALID_BIT_MSK        0b1111111111111111      // PWM Lock/Unlock Sequence register

#define REG_PTCON_VALID_BIT_MSK         0b1011111111111111      // Primary PWM time base configuration register
#define REG_STCON_VALID_BIT_MSK         0b0001111111111111      // Secondary PWM time base configuration register
#define REG_PTCON2_VALID_BIT_MSK        0b0000000000000111      // Primary PWM clock divider register
#define REG_STCON2_VALID_BIT_MSK        0b0000000000000111      // Secondary PWM clock divider register

#define REG_PWMCONx_VALID_BIT_MSK       0b1111111111001111      // PWM generator channel configuration register
#define REG_IOCONx_VALID_BIT_MSK        0b1111111111111111      // PWM generator channel GPIO configuration register
#define REG_FCLCONx_VALID_BIT_MSK   	0b1111111111111111      // Fault/Current Limit register
#define REG_LEBCONx_VALID_BIT_MSK		0b1111110000111111      // Leading Edge Blanking register
#define REG_LEBDLYx_VALID_BIT_MSK       0b0000111111111000      // Leading Edge Blanking Period register

#define REG_SEVTCMP_VALID_BIT_MSK       0b1111111111111000      // Primary Special Event compare register
#define REG_SSEVTCMP_VALID_BIT_MSK      0b1111111111111000      // This mask has to be AND-ed to the 16-bit compare value, effectively reducing the resolution to 1/8th of the PWM resolution

#define REG_PTPER_VALID_BIT_MSK         0b1111111111111111      // PTPER register
#define REG_STPER_VALID_BIT_MSK         0b1111111111111111      // STPER register
#define REG_MDC_VALID_BIT_MSK           0b1111111111111111      // Master Duty Cycle register
#define REG_CHOP_VALID_BIT_MSK          0b1000001111111000      // On-Time Chopping Frequency register
#define REG_PHASEx_VALID_BIT_MSK        0b1111111111111111      // PHASEx register
#define REG_SPHASEx_VALID_BIT_MSK       0b1111111111111111      // SPHASEx register
#define REG_PDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_SDCx_VALID_BIT_MSK          0b1111111111111111      // PDCx registe
#define REG_DTRx_VALID_BIT_MSK          0b0011111111111111      // DTRx register
#define REG_ALTDTRx_VALID_BIT_MSK       0b0011111111111111      // ALTDTRx register

#define REG_TRGCON_VALID_BIT_MSK        0b1111000010111111      // Trigger Configuration register
#define REG_TRIGx_VALID_BIT_MSK         0b1111111111111000      // Primary Trigger register
#define REG_STRIGx_VALID_BIT_MSK        0b1111111111111000      // Secondary Trigger register

#define REG_AUXCONx_VALID_BIT_MSK       0b1100111100111111      // PWMx generator auxiliary control register
#define REG_PWMCAPx_VALID_BIT_MSK       0b1111111111111000      // PWMxH Capture register        

#endif

// General Flags
#define PWM_OFF							0		// Flag is used to shut down the pwm module
#define PWM_ON							1		// Flag is used to enable the pwm module

#define PWMx_HIGH						1		// PWM High Output
#define PWMx_LOW						2		// PWM Low Output
#define PWMx_HIGH_LOW					0		// PWM High & Low Outputs

#define PWM_PRIMARY_SECONDARY           0       // PWM Primary + Secondary Channel/TIme Base
#define PWM_PRIMARY                     1       // PWM Primary Channel/TIme Base
#define PWM_SECONDARY                   2       // PWM Secondary Channel/TIme Base
  
// PWM Time Base Settings
#define PWM_USE_MASTER_TIME_BASE		0		// The PWM counter is assigned to the Master Time Base
#define PWM_USE_INDEPENDENT_TIME_BASE	1		// The PWM counter is assigned to the Independent Time Base
#define PWM_USE_MASTER_DUTY_CYCLE		1		// The PWM Duty Cycle is assigned to the Master Duty Cycle
#define PWM_USE_INDEPENDENT_DUTY_CYCLE	0		// The PWM Duty Cycle is assigned to the Independent Duty Cycle

#define PWM_MASTER_TIME_BASE_PRIMARY	0		// In MTB mode, the PWM counter uses the Primary Master Time Base
#define PWM_MASTER_TIME_BASE_SECONDARY	1		// In MTB mode, the PWM counter uses the Secondary Master Time Base

// PWM Counter Modes
#define COUNTER_MODE_CENTER_ALIGNED		1		// The PWM counter will count up and down during one period
#define COUNTER_MODE_EDGE_ALIGNED		0		// The PWM counter will count up and reset during one period

// PWM Update Modes
#define UPDATE_MODE_IMMEDIATE			1		// New PWM settings will be applied immediately when written
#define UPDATE_MODE_PERIOD_RESET    	0		// New PWM settings will be applied during the reset into the next period

// Basic PWM Modes
#define PWM_COMPLEMENTARY				0b00	// PWM channel will generate complementary PWM
#define PWM_REDUNDANT					0b01	// PWM channel will generate redundant PWM
#define PWM_PUSH_PULL					0b10	// PWM channel will generate push-pull PWM
#define PWM_TRUE_INDEPENDENT			0b11	// PWM channel will generate true independent PWM

// Dead Time Modes
#define DEAD_TIME_COMPENSATED			0b11	// Dead Time Compensation Mode
#define DEAD_TIME_POSITIVE				0b00	// Positive Dead Time applied to all output modes
#define DEAD_TIME_NEGATIVE				0b01	// Negative Dead Time applied to all output modes
#define DEAD_TIME_DISABLED				0b10	// Dead Time Generator is disabled

// Interrupts
#define SEV_INTERRUPT_ENABLED           1       // Bit setting for enabled interrupts of Special Event Interrupt
#define SEV_INTERRUPT_DISABLED          0       // Bit setting for disabled interrupts of Special Event Interrupt

#define PWMx_INTERRUPT_ENABLED          1       // Bit setting for enabled interrupts of a dedicated PWM channel
#define PWMx_INTERRUPT_DISABLED         0       // Bit setting for disabled interrupts of a dedicated PWM channel
  
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PTCON - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Full 16 Bit Field Flags
#define REG_PTEN_ON						0b1000000000000000
#define REG_PTEN_OFF					0b0000000000000000
#define REG_PTCON_PWM_OFF_MASK          0b0111111111111111
  
#define REG_PTSIDL_ON					0b0010000000000000      // PWMx Time Base Stop in Idle Mode bit
#define REG_PTSIDL_OFF					0b0000000000000000

#define REG_SEIEN_ON					0b0000100000000000      // Special Event Interrupt Enable bit
#define REG_SEIEN_OFF					0b0000000000000000

#define REG_EIPU_ON						0b0000010000000000      // Enable Immediate Period Updates bit
#define REG_EIPU_OFF					0b0000000000000000

#define REG_SYNCPOL_INVERTED			0b0000001000000000      // Synchronize Input and Output Polarity bit
#define REG_SYNCPOL_HIGH				0b0000000000000000

#define REG_SYNCOEN_ON					0b0000000100000000      // Primary Time Base Synchronization Enable bit
#define REG_SYNCOEN_OFF					0b0000000000000000

#define REG_SYNCEN_ON					0b0000000010000000      // External Time Base Synchronization Enable bit
#define REG_SYNCEN_OFF					0b0000000000000000

#define REG_SYNCSRC_SYNCI1				0b0000000000000000      // Synchronous Source Selection bits
#define REG_SYNCSRC_SYNCI2				0b0000000000010000
#define REG_SYNCSRC_SYNCI3				0b0000000000100000
#define REG_SYNCSRC_SYNCI4				0b0000000000110000

#define REG_SEVTPS_1_TO_1				0b0000000000000000      // PWMx Special Event Trigger Output Postscaler Select bits
#define REG_SEVTPS_1_TO_2				0b0000000000000001
#define REG_SEVTPS_1_TO_3				0b0000000000000010
#define REG_SEVTPS_1_TO_4				0b0000000000000011
#define REG_SEVTPS_1_TO_5				0b0000000000000100
#define REG_SEVTPS_1_TO_6				0b0000000000000101
#define REG_SEVTPS_1_TO_7				0b0000000000000110
#define REG_SEVTPS_1_TO_8				0b0000000000000111
#define REG_SEVTPS_1_TO_9				0b0000000000001000
#define REG_SEVTPS_1_TO_10				0b0000000000001001
#define REG_SEVTPS_1_TO_11				0b0000000000001010
#define REG_SEVTPS_1_TO_12				0b0000000000001011
#define REG_SEVTPS_1_TO_13				0b0000000000001100
#define REG_SEVTPS_1_TO_14				0b0000000000001101
#define REG_SEVTPS_1_TO_15				0b0000000000001110
#define REG_SEVTPS_1_TO_16				0b0000000000001111

// Single Bit Field Flags
#define PTEN_ON							0b1
#define PTEN_OFF						0b0

#define PTSIDL_ON						0b1
#define PTSIDL_OFF						0b0

#define SEIEN_ON						0b1
#define SEIEN_OFF						0b0

#define EIPU_ON							0b1
#define EIPU_OFF						0b0

#define SYNCPOL_INVERTED				0b1
#define SYNCPOL_HIGH					0b0

#define SYNCOEN_ON						0b1
#define SYNCOEN_OFF						0b0

#define SYNCEN_ON						0b1
#define SYNCEN_OFF						0b0

#define SYNCSRC_SYNCI1					0b00
#define SYNCSRC_SYNCI2					0b01
#define SYNCSRC_SYNCI3					0b10
#define SYNCSRC_SYNCI4					0b11

#define SEVTPS_1_TO_1					0b0000
#define SEVTPS_1_TO_2					0b0001
#define SEVTPS_1_TO_3					0b0010
#define SEVTPS_1_TO_4					0b0011
#define SEVTPS_1_TO_5					0b0100
#define SEVTPS_1_TO_6					0b0101
#define SEVTPS_1_TO_7					0b0110
#define SEVTPS_1_TO_8					0b0111
#define SEVTPS_1_TO_9					0b1000
#define SEVTPS_1_TO_10					0b1001
#define SEVTPS_1_TO_11					0b1010
#define SEVTPS_1_TO_12					0b1011
#define SEVTPS_1_TO_13					0b1100
#define SEVTPS_1_TO_14					0b1101
#define SEVTPS_1_TO_15					0b1110
#define SEVTPS_1_TO_16					0b1111

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PTCON2 - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Full 16 Bit Field Flags
#define REG_PCLKDIV_1					0b0000000000000000
#define REG_PCLKDIV_2					0b0000000000000001
#define REG_PCLKDIV_4					0b0000000000000010
#define REG_PCLKDIV_8					0b0000000000000011
#define REG_PCLKDIV_16					0b0000000000000100
#define REG_PCLKDIV_32					0b0000000000000101
#define REG_PCLKDIV_64					0b0000000000000110

// Single Bit Field Flags
#define PCLKDIV_1						0b000
#define PCLKDIV_2						0b001
#define PCLKDIV_4						0b010
#define PCLKDIV_8						0b011
#define PCLKDIV_16						0b100
#define PCLKDIV_32						0b101
#define PCLKDIV_64						0b110

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PWMCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Full 16 Bit Field Flags
#define REG_FLTIEN_ON					0b0001000000000000
#define REG_FLTIEN_OFF					0b0000000000000000

#define REG_CLIEN_ON					0b0000100000000000
#define REG_CLIEN_OFF					0b0000000000000000

#define REG_TRGIEN_ON					0b0000010000000000
#define REG_TRGIEN_OFF					0b0000000000000000

#define REG_ITB_INDEPENDENT				0b0000001000000000
#define REG_ITB_MASTER					0b0000000000000000

#define REG_MDCS_INDEPENDENT			0b0000000000000000
#define REG_MDCS_MASTER					0b0000000100000000

#define REG_DTC_POSITIVE				0b0000000000000000
#define REG_DTC_NEGATIVE				0b0000000001000000
#define REG_DTC_DISASBLED				0b0000000010000000
#define REG_DTC_COMPENSATED				0b0000000011000000

#define REG_DTC_POSITIVE_SET_MSK    	0b1111111100111111
#define REG_DTC_NEGATIVE_SET_MSK		0b1111111101111111
#define REG_DTC_DISASBLED_SET_MSK		0b1111111110111111
#define REG_DTC_COMPENSATED_SET_MSK		0b1111111111111111
  
  
#define REG_DTCP_SH_LL_ON_LOW			0b0000000000000000
#define REG_DTCP_LH_SL_ON_LOW			0b0000000000100000

#define REG_MTBS_USE_PRIMARY_TIME_BASE		0b0000000000000000
#define REG_MTBS_USE_SECONDARY_TIME_BASE 	0b0000000000001000

#define REG_CAM_ON						0b0000000000000100
#define REG_CAM_OFF						0b0000000000000000

#define REG_XPRES_ON					0b0000000000000010
#define REG_XPRES_OFF					0b0000000000000000

#define REG_IUE_ON						0b0000000000000001
#define REG_IUE_OFF						0b0000000000000000

// Single Bit Field Flags
#define FLTIEN_ON						0b1
#define FLTIEN_OFF						0b0

#define CLIEN_ON						0b1
#define CLIEN_OFF						0b0

#define TRGIEN_ON						0b1
#define TRGIEN_OFF						0b0

#define ITB_INDEPENDENT					0b1
#define ITB_MASTER						0b0

#define MDCS_INDEPENDENT				0b0
#define MDCS_MASTER						0b01

#define DTC_POSITIVE					0b00
#define DTC_NEGATIVE					0b01
#define DTC_DISASBLED					0b10
#define DTC_COMPENSATED					0b11

#define DTCP_SH_LL_ON_LOW				0b0
#define DTCP_LH_SL_ON_LOW				0b1

#define MTBS_USE_PRIMARY_TIME_BASE		0b0
#define MTBS_USE_SECONDARY_TIME_BASE 	0b1

#define CAM_ON							0b1
#define CAM_OFF							0b0

#define XPRES_ON						0b1
#define XPRES_OFF						0b0

#define IUE_ON							0b1
#define IUE_OFF							0b0

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  IOCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Full 16 Bit Field Flags
#define REG_PWMH_IO_CONTROL_ON			0b1000000000000000
#define REG_PWMH_IO_CONTROL_OFF			0b0000000000000000

#define REG_PWML_IO_CONTROL_ON			0b0100000000000000
#define REG_PWML_IO_CONTROL_OFF			0b0000000000000000

#define REG_PWMH_IO_CONTROL_SET_MASK	0b1000000000000000
#define REG_PWMH_IO_CONTROL_CLR_MASK	0b0111111111111111
#define REG_PWML_IO_CONTROL_SET_MASK    0b0100000000000000
#define REG_PWML_IO_CONTROL_CLR_MASK    0b1011111111111111
#define REG_PWMx_IO_CONTROL_SET_MASK    0b1100000000000000
#define REG_PWMx_IO_CONTROL_CLR_MASK    0b0011111111111111
  
#define REG_POLH_ACTIVE_HIGH			0b0000000000000000
#define REG_POLH_ACTIVE_LOW				0b0010000000000000

#define REG_POLL_ACTIVE_HIGH			0b0000000000000000
#define REG_POLL_ACTIVE_LOW				0b0001000000000000

#define REG_PMOD_COMPLEMENTARY			0b0000000000000000
#define REG_PMOD_REDUNDANT				0b0000010000000000
#define REG_PMOD_PUSH_PULL				0b0000100000000000
#define REG_PMOD_TRUE_INDEPENDENT		0b0000110000000000

#define REG_OVRENH_ON					0b0000001000000000
#define REG_OVRENH_OFF					0b0000000000000000
#define REG_OVRENH_SET_MASK				0b0000001000000000
#define REG_OVRENH_CLR_MASK				0b1111110111111111

#define REG_OVRENL_ON					0b0000000100000000
#define REG_OVRENL_OFF					0b0000000000000000
#define REG_OVRENL_SET_MASK				0b0000000100000000
#define REG_OVRENL_CLR_MASK				0b1111111011111111

#define REG_OVRENx_SET_MASK				0b0000001100000000
#define REG_OVRENx_CLR_MASK				0b1111110011111111

#define REG_OVRDAT_HIGH_HIGH			0b0000000011000000
#define REG_OVRDAT_HIGH_LOW				0b0000000010000000
#define REG_OVRDAT_LOW_HIGH				0b0000000001000000
#define REG_OVRDAT_LOW_LOW				0b0000000000000000

#define REG_FLTDAT_HIGH_HIGH			0b0000000000110000
#define REG_FLTDAT_HIGH_LOW				0b0000000000100000
#define REG_FLTDAT_LOW_HIGH				0b0000000000010000
#define REG_FLTDAT_LOW_LOW				0b0000000000000000

#define REG_CLDAT_HIGH_HIGH				0b0000000000001100
#define REG_CLDAT_HIGH_LOW				0b0000000000001000
#define REG_CLDAT_LOW_HIGH				0b0000000000000100
#define REG_CLDAT_LOW_LOW				0b0000000000000000

#define REG_SWAP_ON						0b0000000000000010
#define REG_SWAP_OFF					0b0000000000000000

#define REG_OSYNC_ON					0b0000000000000001
#define REG_OSYNC_OFF					0b0000000000000000

// Single Bit Field Flags
#define PWMH_IO_CONTROL_ON				0b1
#define PWMH_IO_CONTROL_OFF				0b0

#define PWML_IO_CONTROL_ON				0b1
#define PWML_IO_CONTROL_OFF				0b0

#define POLH_HIGH						0b1
#define POLH_LOW						0b0

#define POLL_HIGH						0b1
#define POLL_LOW						0b0

#define PMOD_COMPLEMENTARY				0b00
#define PMOD_REDUNDANT					0b01
#define PMOD_PUSH_PULL					0b10
#define PMOD_TRUE_INDEPENDENT			0b11

#define OVRENH_ON						0b1
#define OVRENH_OFF						0b0

#define OVRENL_ON						0b1
#define OVRENL_OFF						0b0

#define OVRDAT_HIGH_HIGH				0b11
#define OVRDAT_HIGH_LOW					0b10
#define OVRDAT_LOW_HIGH					0b01
#define OVRDAT_LOW_LOW					0b00

#define FLTDAT_HIGH_HIGH				0b11
#define FLTDAT_HIGH_LOW					0b10
#define FLTDAT_LOW_HIGH					0b01
#define FLTDAT_LOW_LOW					0b00

#define CLDAT_HIGH_HIGH					0b11
#define CLDAT_HIGH_LOW					0b10
#define CLDAT_LOW_HIGH					0b01
#define CLDAT_LOW_LOW					0b00

#define SWAP_ON							0b1
#define SWAP_OFF						0b0

#define OSYNC_ON						0b1
#define OSYNC_OFF						0b0

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  FCLCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Full 16 Bit Field Flags
#define REG_IFLTMOD_INDEPENDENT			0b1000000000000000
#define REG_IFLTMOD_NORMAL				0b0000000000000000

// Single Bit Field Flags
#define IFLTMOD_INDEPENDENT				0b1
#define IFLTMOD_NORMAL					0b0

#if defined (__P33SMPS_FJ__) || defined (__P33SMPS_FJA__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the second generation with 18-44 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_CLSRC_FAULT1			0b0000000000000000
#define REG_CLSRC_FAULT2			0b0000010000000000
#define REG_CLSRC_FAULT3			0b0000100000000000
#define REG_CLSRC_FAULT4			0b0000110000000000
#define REG_CLSRC_FAULT5			0b0001000000000000
#define REG_CLSRC_FAULT6			0b0001010000000000
#define REG_CLSRC_FAULT7			0b0001100000000000
#define REG_CLSRC_FAULT8			0b0001110000000000

// Single Bit Field Flags
#define CLSRC_FAULT1				0b000
#define CLSRC_FAULT2				0b001
#define CLSRC_FAULT3				0b010
#define CLSRC_FAULT4				0b011
#define CLSRC_FAULT5				0b100
#define CLSRC_FAULT6				0b101
#define CLSRC_FAULT7				0b110
#define CLSRC_FAULT8				0b111

// On the second generation the analog comparators cannot be mapped to a PWM channel directly
// On this devices a comparator output has to be connected to a Fault input via Peripheral Pin 
// Select. For this function 4 virtual peripheral pins are available which are not connected to
// any physical pin.

#elif defined (__P33SMPS_FJC__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the third generation with 64-100 pin, NOT supporting
// peripheral pin select

// Full 16 Bit Field Flags
#define REG_CLSRC_FAULT1			0b0010000000000000
#define REG_CLSRC_FAULT2			0b0010010000000000
#define REG_CLSRC_FAULT3			0b0010100000000000
#define REG_CLSRC_FAULT4			0b0010110000000000
#define REG_CLSRC_FAULT5			0b0011000000000000
#define REG_CLSRC_FAULT6			0b0011010000000000
#define REG_CLSRC_FAULT7			0b0011100000000000
#define REG_CLSRC_FAULT8			0b0011110000000000

#define REG_CLSRC_FAULT9			0b0100000000000000
#define REG_CLSRC_FAULT10			0b0100010000000000
#define REG_CLSRC_FAULT11			0b0100100000000000
#define REG_CLSRC_FAULT12			0b0100110000000000
#define REG_CLSRC_FAULT13			0b0101000000000000
#define REG_CLSRC_FAULT14			0b0101010000000000
#define REG_CLSRC_FAULT15			0b0101100000000000
#define REG_CLSRC_FAULT16			0b0101110000000000

#define REG_CLSRC_FAULT17			0b0110000000000000
#define REG_CLSRC_FAULT18			0b0110010000000000
#define REG_CLSRC_FAULT19			0b0110100000000000
#define REG_CLSRC_FAULT20			0b0110110000000000
#define REG_CLSRC_FAULT21			0b0111000000000000
#define REG_CLSRC_FAULT22			0b0111010000000000
#define REG_CLSRC_FAULT23			0b0111100000000000

// On the third generation the comparator outputs can be mapped to a PWM channel directly

#define REG_CLSRC_ACMP1				0b0000000000000000
#define REG_CLSRC_ACMP2				0b0000010000000000
#define REG_CLSRC_ACMP3				0b0000100000000000
#define REG_CLSRC_ACMP4				0b0000110000000000

// Single Bit Field Flags
#define CLSRC_FAULT1				0b01000
#define CLSRC_FAULT2				0b01001
#define CLSRC_FAULT3				0b01010
#define CLSRC_FAULT4				0b01011
#define CLSRC_FAULT5				0b01100
#define CLSRC_FAULT6				0b01101
#define CLSRC_FAULT7				0b01110
#define CLSRC_FAULT8				0b01111

#define CLSRC_FAULT9				0b10000
#define CLSRC_FAULT10				0b10001
#define CLSRC_FAULT11				0b10010
#define CLSRC_FAULT12				0b10011
#define CLSRC_FAULT13				0b10100
#define CLSRC_FAULT14				0b10101
#define CLSRC_FAULT15				0b10110
#define CLSRC_FAULT16				0b10111

#define CLSRC_FAULT17				0b11000
#define CLSRC_FAULT18				0b11001
#define CLSRC_FAULT19				0b11010
#define CLSRC_FAULT20				0b11011
#define CLSRC_FAULT21				0b11100
#define CLSRC_FAULT22				0b11101
#define CLSRC_FAULT23				0b11110

// On the third generation the comparator outputs can be mapped to a PWM channel directly

#define CLSRC_ACMP1					0b00000
#define CLSRC_ACMP2					0b00001
#define CLSRC_ACMP3					0b00010
#define CLSRC_ACMP4					0b00011


#elif defined (__P33SMPS_EP2__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fourth generation with 28 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_CLSRC_NONE  			0b0000000000000000

#define REG_CLSRC_FAULT1			0b0000010000000000
#define REG_CLSRC_FAULT2			0b0000100000000000
#define REG_CLSRC_FAULT3			0b0000110000000000
#define REG_CLSRC_FAULT4			0b0001000000000000
#define REG_CLSRC_FAULT5			0b0001010000000000
#define REG_CLSRC_FAULT6			0b0001100000000000
#define REG_CLSRC_FAULT7			0b0001110000000000
#define REG_CLSRC_FAULT8			0b0010000000000000

#define REG_CLSRC_ACMP1				0b0011010000000000
#define REG_CLSRC_ACMP2				0b0011100000000000

// Single Bit Field Flags
#define CLSRC_FAULT1				0b00001
#define CLSRC_FAULT2				0b00010
#define CLSRC_FAULT3				0b00011
#define CLSRC_FAULT4				0b00100
#define CLSRC_FAULT5				0b00101
#define CLSRC_FAULT6				0b00110
#define CLSRC_FAULT7				0b00111
#define CLSRC_FAULT8				0b01000

// On the fourth generation the comparator outputs can be mapped to a PWM channel directly

#define CLSRC_ACMP1					0b01101
#define CLSRC_ACMP2					0b01110


#elif defined (__P33SMPS_EP5__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fifth generation with 28-64 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_CLSRC_NONE  			0b0000000000000000

#define REG_CLSRC_FAULT1			0b0000010000000000
#define REG_CLSRC_FAULT2			0b0000100000000000
#define REG_CLSRC_FAULT3			0b0000110000000000
#define REG_CLSRC_FAULT4			0b0001000000000000
#define REG_CLSRC_FAULT5			0b0001010000000000
#define REG_CLSRC_FAULT6			0b0001100000000000
#define REG_CLSRC_FAULT7			0b0001110000000000
#define REG_CLSRC_FAULT8			0b0010000000000000
#define REG_CLSRC_FAULT9			0b0010010000000000
#define REG_CLSRC_FAULT10			0b0010100000000000
#define REG_CLSRC_FAULT11			0b0010110000000000
#define REG_CLSRC_FAULT12			0b0011000000000000

#define REG_CLSRC_ACMP1				0b0011010000000000
#define REG_CLSRC_ACMP2				0b0011100000000000
#define REG_CLSRC_ACMP3				0b0011110000000000
#define REG_CLSRC_ACMP4				0b0100000000000000


// Single Bit Field Flags
#define CLSRC_FAULT1				0b00001
#define CLSRC_FAULT2				0b00010
#define CLSRC_FAULT3				0b00011
#define CLSRC_FAULT4				0b00100
#define CLSRC_FAULT5				0b00101
#define CLSRC_FAULT6				0b00110
#define CLSRC_FAULT7				0b00111
#define CLSRC_FAULT8				0b01000
#define CLSRC_FAULT9				0b01001
#define CLSRC_FAULT10				0b01010
#define CLSRC_FAULT11				0b01011
#define CLSRC_FAULT12				0b01100

// On the fourth generation the comparator outputs can be mapped to a PWM channel directly

#define CLSRC_ACMP1					0b01101
#define CLSRC_ACMP2					0b01110
#define CLSRC_ACMP3					0b01111
#define CLSRC_ACMP4					0b10000

  
#elif defined (__P33SMPS_EP7__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fifth generation with 28-64 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_CLSRC_NONE  			0b0000000000000000

#define REG_CLSRC_FAULT1			0b0000010000000000
#define REG_CLSRC_FAULT2			0b0000100000000000
#define REG_CLSRC_FAULT3			0b0000110000000000
#define REG_CLSRC_FAULT4			0b0001000000000000
#define REG_CLSRC_FAULT5			0b0001010000000000
#define REG_CLSRC_FAULT6			0b0001100000000000
#define REG_CLSRC_FAULT7			0b0001110000000000
#define REG_CLSRC_FAULT8			0b0010000000000000
#define REG_CLSRC_FAULT9			0b0010010000000000
#define REG_CLSRC_FAULT10			0b0010100000000000
#define REG_CLSRC_FAULT11			0b0010110000000000
#define REG_CLSRC_FAULT12			0b0011000000000000

#define REG_CLSRC_ACMP1				0b0011010000000000
#define REG_CLSRC_ACMP2				0b0011100000000000
#define REG_CLSRC_ACMP3				0b0011110000000000
#define REG_CLSRC_ACMP4				0b0100000000000000


// Single Bit Field Flags
#define CLSRC_FAULT1				0b00001
#define CLSRC_FAULT2				0b00010
#define CLSRC_FAULT3				0b00011
#define CLSRC_FAULT4				0b00100
#define CLSRC_FAULT5				0b00101
#define CLSRC_FAULT6				0b00110
#define CLSRC_FAULT7				0b00111
#define CLSRC_FAULT8				0b01000
#define CLSRC_FAULT9				0b01001
#define CLSRC_FAULT10				0b01010
#define CLSRC_FAULT11				0b01011
#define CLSRC_FAULT12				0b01100

// On the fourth generation the comparator outputs can be mapped to a PWM channel directly

#define CLSRC_ACMP1					0b01101
#define CLSRC_ACMP2					0b01110
#define CLSRC_ACMP3					0b01111
#define CLSRC_ACMP4					0b10000


#else

// If the selected device is unknown, a warning for invalid settings will be sent to the
// output window

#error -- fault input settings are not valid for selected device --

#endif

// Full 16 Bit Field Flags
#define REG_CLPOL_ACTIVE_HIGH		0b0000000000000000
#define REG_CLPOL_ACTIVE_LOW		0b0000001000000000

#define REG_CLMOD_ON				0b0000000100000000
#define REG_CLMOD_OFF				0b0000000000000000

// Single Bit Field Flags
#define CLPOL_ACTIVE_HIGH			0b0
#define CLPOL_ACTIVE_LOW			0b1

#define CLMOD_ON					0b1
#define CLMOD_OFF					0b0

#if defined (__P33SMPS_FJ__) || defined (__P33SMPS_FJA__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the second generation with 18-44 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_FLTSRC_FAULT1			0b0000000000000000
#define REG_FLTSRC_FAULT2			0b0000000000001000
#define REG_FLTSRC_FAULT3			0b0000000000010000
#define REG_FLTSRC_FAULT4			0b0000000000011000
#define REG_FLTSRC_FAULT5			0b0000000000100000
#define REG_FLTSRC_FAULT6			0b0000000000101000
#define REG_FLTSRC_FAULT7			0b0000000000110000
#define REG_FLTSRC_FAULT8			0b0000000000111000

// Single Bit Field Flags
#define FLTSRC_FAULT1				0b000
#define FLTSRC_FAULT2				0b001
#define FLTSRC_FAULT3				0b010
#define FLTSRC_FAULT4				0b011
#define FLTSRC_FAULT5				0b100
#define FLTSRC_FAULT6				0b101
#define FLTSRC_FAULT7				0b110
#define FLTSRC_FAULT8				0b111

// On the second generation the analog comparators cannot be mapped to a PWM channel directly
// On this devices a comparator output has to be connected to a Fault input via Peripheral Pin 
// Select. For this function 4 virtual peripheral pins are available which are not connected to
// any physical pin.

#elif defined (__P33SMPS_FJC__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the third generation with 64-100 pin, NOT supporting
// peripheral pin select

// Full 16 Bit Field Flags
#define REG_FLTSRC_FAULT1				0b0000000001000000
#define REG_FLTSRC_FAULT2				0b0000000001001000
#define REG_FLTSRC_FAULT3				0b0000000001010000
#define REG_FLTSRC_FAULT4				0b0000000001011000
#define REG_FLTSRC_FAULT5				0b0000000001100000
#define REG_FLTSRC_FAULT6				0b0000000001101000
#define REG_FLTSRC_FAULT7				0b0000000001110000
#define REG_FLTSRC_FAULT8				0b0000000001111000

#define REG_FLTSRC_FAULT9				0b0000000010000000
#define REG_FLTSRC_FAULT10				0b0000000010001000
#define REG_FLTSRC_FAULT11				0b0000000010010000
#define REG_FLTSRC_FAULT12				0b0000000010011000
#define REG_FLTSRC_FAULT13				0b0000000010100000
#define REG_FLTSRC_FAULT14				0b0000000010101000
#define REG_FLTSRC_FAULT15				0b0000000010110000
#define REG_FLTSRC_FAULT16				0b0000000010111000

#define REG_FLTSRC_FAULT17				0b0000000011000000
#define REG_FLTSRC_FAULT18				0b0000000011001000
#define REG_FLTSRC_FAULT19				0b0000000011010000
#define REG_FLTSRC_FAULT20				0b0000000011011000
#define REG_FLTSRC_FAULT21				0b0000000011100000
#define REG_FLTSRC_FAULT22				0b0000000011101000
#define REG_FLTSRC_FAULT23				0b0000000011110000

// On the third generation the comparator outputs can be mapped to a PWM channel directly

#define REG_FLTSRC_ACMP1				0b0000000000000000
#define REG_FLTSRC_ACMP2				0b0000000000001000
#define REG_FLTSRC_ACMP3				0b0000000000010000
#define REG_FLTSRC_ACMP4				0b0000000000011000

// Single Bit Field Flags
#define FLTSRC_FAULT1					0b01000
#define FLTSRC_FAULT2					0b01001
#define FLTSRC_FAULT3					0b01010
#define FLTSRC_FAULT4					0b01011
#define FLTSRC_FAULT5					0b01100
#define FLTSRC_FAULT6					0b01101
#define FLTSRC_FAULT7					0b01110
#define FLTSRC_FAULT8					0b01111

#define FLTSRC_FAULT9					0b10000
#define FLTSRC_FAULT10					0b10001
#define FLTSRC_FAULT11					0b10010
#define FLTSRC_FAULT12					0b10011
#define FLTSRC_FAULT13					0b10100
#define FLTSRC_FAULT14					0b10101
#define FLTSRC_FAULT15					0b10110
#define FLTSRC_FAULT16					0b10111

#define FLTSRC_FAULT17					0b11000
#define FLTSRC_FAULT18					0b11001
#define FLTSRC_FAULT19					0b11010
#define FLTSRC_FAULT20					0b11011
#define FLTSRC_FAULT21					0b11100
#define FLTSRC_FAULT22					0b11101
#define FLTSRC_FAULT23					0b11110

// On the third generation the comparator outputs can be mapped to a PWM channel directly

#define FLTSRC_ACMP1					0b00000
#define FLTSRC_ACMP2					0b00001
#define FLTSRC_ACMP3					0b00010
#define FLTSRC_ACMP4					0b00011

#elif defined (__P33SMPS_EP2__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fourth generation with 28 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_FLTSRC_NONE 				0b0000000000000000

#define REG_FLTSRC_FAULT1				0b0000000000001000
#define REG_FLTSRC_FAULT2				0b0000000000010000
#define REG_FLTSRC_FAULT3				0b0000000000011000
#define REG_FLTSRC_FAULT4				0b0000000000100000
#define REG_FLTSRC_FAULT5				0b0000000000101000
#define REG_FLTSRC_FAULT6				0b0000000000110000
#define REG_FLTSRC_FAULT7				0b0000000000111000
#define REG_FLTSRC_FAULT8				0b0000000001000000

#define REG_FLTSRC_ACMP1				0b0000000001101000
#define REG_FLTSRC_ACMP2				0b0000000001110000

// Single Bit Field Flags
#define FLTSRC_FAULT1					0b00001
#define FLTSRC_FAULT2					0b00010
#define FLTSRC_FAULT3					0b00011
#define FLTSRC_FAULT4					0b00100
#define FLTSRC_FAULT5					0b00101
#define FLTSRC_FAULT6					0b00110
#define FLTSRC_FAULT7					0b00111
#define FLTSRC_FAULT8					0b01000

#define FLTSRC_ACMP1                	0b01101
#define FLTSRC_ACMP2                	0b01110

#elif defined (__P33SMPS_EP5__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fifth generation with 28-64 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_FLTSRC_NONE 				0b0000000000000000

#define REG_FLTSRC_FAULT1				0b0000000000001000
#define REG_FLTSRC_FAULT2				0b0000000000010000
#define REG_FLTSRC_FAULT3				0b0000000000011000
#define REG_FLTSRC_FAULT4				0b0000000000100000
#define REG_FLTSRC_FAULT5				0b0000000000101000
#define REG_FLTSRC_FAULT6				0b0000000000110000
#define REG_FLTSRC_FAULT7				0b0000000000111000
#define REG_FLTSRC_FAULT8				0b0000000001000000
#define REG_FLTSRC_FAULT9				0b0000000001001000
#define REG_FLTSRC_FAULT10				0b0000000001010000
#define REG_FLTSRC_FAULT11				0b0000000001011000
#define REG_FLTSRC_FAULT12				0b0000000001100000

#define REG_FLTSRC_ACMP1				0b0000000001101000
#define REG_FLTSRC_ACMP2				0b0000000001110000
#define REG_FLTSRC_ACMP3				0b0000000001111000
#define REG_FLTSRC_ACMP4				0b0000000010000000

// Single Bit Field Flags
#define FLTSRC_FAULT1					0b00001
#define FLTSRC_FAULT2					0b00010
#define FLTSRC_FAULT3					0b00011
#define FLTSRC_FAULT4					0b00100
#define FLTSRC_FAULT5					0b00101
#define FLTSRC_FAULT6					0b00110
#define FLTSRC_FAULT7					0b00111
#define FLTSRC_FAULT8					0b01000
#define FLTSRC_FAULT9					0b01001
#define FLTSRC_FAULT10					0b01010
#define FLTSRC_FAULT11					0b01011
#define FLTSRC_FAULT12					0b01100

#define FLTSRC_ACMP1                	0b01101
#define FLTSRC_ACMP2                	0b01110
#define FLTSRC_ACMP3                	0b01111
#define FLTSRC_ACMP4                	0b10000

#elif defined (__P33SMPS_EP7__)

// Please note:
// The bit settings for the current limit sources are different on different generations of the 
// GS devices. These settings are for the fifth generation with 28-64 pin supporting peripheral
// pin select

// Full 16 Bit Field Flags
#define REG_FLTSRC_NONE 				0b0000000000000000

#define REG_FLTSRC_FAULT1				0b0000000000001000
#define REG_FLTSRC_FAULT2				0b0000000000010000
#define REG_FLTSRC_FAULT3				0b0000000000011000
#define REG_FLTSRC_FAULT4				0b0000000000100000
#define REG_FLTSRC_FAULT5				0b0000000000101000
#define REG_FLTSRC_FAULT6				0b0000000000110000
#define REG_FLTSRC_FAULT7				0b0000000000111000
#define REG_FLTSRC_FAULT8				0b0000000001000000
#define REG_FLTSRC_FAULT9				0b0000000001001000
#define REG_FLTSRC_FAULT10				0b0000000001010000
#define REG_FLTSRC_FAULT11				0b0000000001011000
#define REG_FLTSRC_FAULT12				0b0000000001100000

#define REG_FLTSRC_ACMP1				0b0000000001101000
#define REG_FLTSRC_ACMP2				0b0000000001110000
#define REG_FLTSRC_ACMP3				0b0000000001111000
#define REG_FLTSRC_ACMP4				0b0000000010000000

// Single Bit Field Flags
#define FLTSRC_FAULT1					0b00001
#define FLTSRC_FAULT2					0b00010
#define FLTSRC_FAULT3					0b00011
#define FLTSRC_FAULT4					0b00100
#define FLTSRC_FAULT5					0b00101
#define FLTSRC_FAULT6					0b00110
#define FLTSRC_FAULT7					0b00111
#define FLTSRC_FAULT8					0b01000
#define FLTSRC_FAULT9					0b01001
#define FLTSRC_FAULT10					0b01010
#define FLTSRC_FAULT11					0b01011
#define FLTSRC_FAULT12					0b01100

#define FLTSRC_ACMP1                	0b01101
#define FLTSRC_ACMP2                	0b01110
#define FLTSRC_ACMP3                	0b01111
#define FLTSRC_ACMP4                	0b10000

#else

// If the selected device is unknown, a warning for invalid settings will be sent to the
// output window

#warning -- fault input settings are not valid for selected device --

#endif

// Full 16 Bit Field Flags
#define REG_FLTPOL_ACTIVE_HIGH			0b0000000000000000
#define REG_FLTPOL_ACTIVE_LOW			0b0000000000000100

#define REG_FLTMOD_CYCLIC				0b0000000000000001
#define REG_FLTMOD_LATCHED				0b0000000000000000
#define REG_FLTMOD_DISABLED				0b0000000000000011

// Single Bit Field Flags
#define FLTPOL_ACTIVE_HIGH				0b0
#define FLTPOL_ACTIVE_LOW				0b1

#define FLTMOD_CYCLIC					0b01
#define FLTMOD_LATCHED					0b00
#define FLTMOD_DISABLED					0b11

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  LEBCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
// Full 16 Bit Field Flags
#define REG_LEBTRG_PWMxH_RISING_ON		0b1000000000000000
#define REG_LEBTRG_PWMxH_RISING_OFF		0b0000000000000000

#define REG_LEBTRG_PWMxH_FALLING_ON		0b0100000000000000
#define REG_LEBTRG_PWMxH_FALLING_OFF	0b0000000000000000

#define REG_LEBTRG_PWMxL_RISING_ON		0b0010000000000000
#define REG_LEBTRG_PWMxL_RISING_OFF		0b0000000000000000

#define REG_LEBTRG_PWMxL_FALLING_ON		0b0001000000000000
#define REG_LEBTRG_PWMxL_FALLING_OFF	0b0000000000000000

#define REG_FLTLEBEN_ON					0b0000100000000000
#define REG_FLTLEBEN_OFF				0b0000000000000000

#define REG_CLLEBEN_ON					0b0000010000000000
#define REG_CLLEBEN_OFF					0b0000000000000000

#define REG_BCH_ON						0b0000000000100000
#define REG_BCH_OFF						0b0000000000000000

#define REG_BCL_ON						0b0000000000010000
#define REG_BCL_OFF						0b0000000000000000

#define REG_BCHH_ON						0b0000000000001000
#define REG_BCHH_OFF					0b0000000000000000

#define REG_BCHL_ON						0b0000000000000100
#define REG_BCHL_OFF					0b0000000000000000

#define REG_BCLH_ON						0b0000000000000010
#define REG_BCLH_OFF					0b0000000000000000

#define REG_BCLL_ON						0b0000000000000001
#define REG_BCLL_OFF					0b0000000000000000

#define REG_LEB_PERIOD_MASK				0b0000001111111000
#define REG_LEBCON_RESET				0b0000000000000000

// Single Bit Field Flags
#define LEBTRG_PWMxH_RISING_ON			0b1
#define LEBTRG_PWMxH_RISING_OFF			0b0

#define LEBTRG_PWMxH_FALLING_ON			0b1
#define LEBTRG_PWMxH_FALLING_OFF		0b0

#define LEBTRG_PWMxL_RISING_ON			0b1
#define LEBTRG_PWMxL_RISING_OFF			0b0

#define LEBTRG_PWMxL_FALLING_ON			0b1
#define LEBTRG_PWMxL_FALLING_OFF		0b0

#define FLTLEBEN_ON						0b1
#define FLTLEBEN_OFF					0b0

#define CLLEBEN_ON						0b1
#define CLLEBEN_OFF						0b0

#define BCH_ON							0b1
#define BCH_OFF							0b0

#define BCL_ON							0b1
#define BCL_OFF							0b0

#define BCHH_ON							0b1
#define BCHH_OFF						0b0

#define BCHL_ON							0b1
#define BCHL_OFF						0b0

#define BCLH_ON							0b1
#define BCLH_OFF						0b0

#define BCLL_ON							0b1
#define BCLL_OFF						0b0

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  AUXCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#if defined (__P33SMPS_FJA__) || defined (__P33SMPS_FJC__)

// Full 16 Bit Field Flags
#define REG_HRPDIS_ON					0b1000000000000000
#define REG_HRPDIS_OFF					0b0000000000000000

#define REG_HRDDIS_ON					0b0100000000000000
#define REG_HRDDIS_OFF					0b0000000000000000

#define REG_BLANKSEL_DISABLED			0b0000000000000000
#define REG_BLANKSEL_PWM1H				0b0000000100000000
#define REG_BLANKSEL_PWM2H				0b0000001000000000
#define REG_BLANKSEL_PWM3H				0b0000001100000000
#define REG_BLANKSEL_PWM4H				0b0000010000000000
#define REG_BLANKSEL_PWM5H				0b0000010100000000
#define REG_BLANKSEL_PWM6H				0b0000011000000000
#define REG_BLANKSEL_PWM7H				0b0000011100000000
#define REG_BLANKSEL_PWM8H				0b0000100000000000
#define REG_BLANKSEL_PWM9H				0b0000100100000000

#define REG_CHOPSEL_DISABLED			0b0000000000000000
#define REG_CHOPSEL_PWM1H				0b0000000000000100
#define REG_CHOPSEL_PWM2H				0b0000000000001000
#define REG_CHOPSEL_PWM3H				0b0000000000001100
#define REG_CHOPSEL_PWM4H				0b0000000000010000
#define REG_CHOPSEL_PWM5H				0b0000000000010100
#define REG_CHOPSEL_PWM6H				0b0000000000011000
#define REG_CHOPSEL_PWM7H				0b0000000000011100
#define REG_CHOPSEL_PWM8H				0b0000000000100000
#define REG_CHOPSEL_PWM9H				0b0000000000100100

#define REG_CHOPHEN_ON					0b0000000000000010
#define REG_CHOPHEN_OFF					0b0000000000000000

#define REG_CHOPLEN_ON					0b0000000000000001
#define REG_CHOPLEN_OFF					0b0000000000000000

// Single Bit Field Flags

#define HRPDIS_ON					0b1
#define HRPDIS_OFF					0b0

#define HRDDIS_ON					0b1
#define HRDDIS_OFF					0b0

#define BLANKSEL_DISABLED			0b0000
#define BLANKSEL_PWM1H				0b0001
#define BLANKSEL_PWM2H				0b0010
#define BLANKSEL_PWM3H				0b0011
#define BLANKSEL_PWM4H				0b0100
#define BLANKSEL_PWM5H				0b0101
#define BLANKSEL_PWM6H				0b0110
#define BLANKSEL_PWM7H				0b0111
#define BLANKSEL_PWM8H				0b1000
#define BLANKSEL_PWM9H				0b1001

#define CHOPSEL_DISABLED			0b0000
#define CHOPSEL_PWM1H				0b0001
#define CHOPSEL_PWM2H				0b0010
#define CHOPSEL_PWM3H				0b0011
#define CHOPSEL_PWM4H				0b0100
#define CHOPSEL_PWM5H				0b0101
#define CHOPSEL_PWM6H				0b0110
#define CHOPSEL_PWM7H				0b0111
#define CHOPSEL_PWM8H				0b1000
#define CHOPSEL_PWM9H				0b1001

#define CHOPHEN_ON					0b1
#define CHOPHEN_OFF					0b0

#define CHOPLEN_ON					0b1
#define CHOPLEN_OFF					0b0

#elif defined (__P33SMPS_EP2__)

// Full 16 Bit Field Flags
#define REG_HRPDIS_ON					0b1000000000000000
#define REG_HRPDIS_OFF					0b0000000000000000

#define REG_HRDDIS_ON					0b0100000000000000
#define REG_HRDDIS_OFF					0b0000000000000000

#define REG_CHOPSEL_CHOP_CLK			0b0000000000000000
#define REG_CHOPSEL_PWM1H				0b0000000000000100
#define REG_CHOPSEL_PWM2H				0b0000000000001000
#define REG_CHOPSEL_PWM4H				0b0000000000010000

#define REG_CHOPHEN_ON					0b0000000000000010
#define REG_CHOPHEN_OFF					0b0000000000000000

#define REG_CHOPLEN_ON					0b0000000000000001
#define REG_CHOPLEN_OFF					0b0000000000000000

// Single Bit Field Flags

#define HRPDIS_ON					0b1
#define HRPDIS_OFF					0b0

#define HRDDIS_ON					0b1
#define HRDDIS_OFF					0b0

#define CHOPSEL_CHOP_CLK			0b0000
#define CHOPSEL_PWM1H				0b0001
#define CHOPSEL_PWM2H				0b0010
#define CHOPSEL_PWM4H				0b0100

#define CHOPHEN_ON					0b1
#define CHOPHEN_OFF					0b0

#define CHOPLEN_ON					0b1
#define CHOPLEN_OFF					0b0

  
#elif defined (__P33SMPS_EP5__)

// Full 16 Bit Field Flags
#define REG_HRPDIS_ON					0b1000000000000000
#define REG_HRPDIS_OFF					0b0000000000000000

#define REG_HRDDIS_ON					0b0100000000000000
#define REG_HRDDIS_OFF					0b0000000000000000

#define REG_BLANKSEL_DISABLED			0b0000000000000000
#define REG_BLANKSEL_PWM1H				0b0000000100000000
#define REG_BLANKSEL_PWM2H				0b0000001000000000
#define REG_BLANKSEL_PWM3H				0b0000001100000000
#define REG_BLANKSEL_PWM4H				0b0000010000000000
#define REG_BLANKSEL_PWM5H				0b0000010100000000

#define REG_CHOPSEL_CHOP_CLK			0b0000000000000000
#define REG_CHOPSEL_PWM1H				0b0000000000000100
#define REG_CHOPSEL_PWM2H				0b0000000000001000
#define REG_CHOPSEL_PWM3H				0b0000000000001100
#define REG_CHOPSEL_PWM4H				0b0000000000010000
#define REG_CHOPSEL_PWM5H				0b0000000000010100

#define REG_CHOPHEN_ON					0b0000000000000010
#define REG_CHOPHEN_OFF					0b0000000000000000

#define REG_CHOPLEN_ON					0b0000000000000001
#define REG_CHOPLEN_OFF					0b0000000000000000

// Single Bit Field Flags

#define HRPDIS_ON					0b1
#define HRPDIS_OFF					0b0

#define HRDDIS_ON					0b1
#define HRDDIS_OFF					0b0

#define BLANKSEL_DISABLED			0b0000
#define BLANKSEL_PWM1H				0b0001
#define BLANKSEL_PWM2H				0b0010
#define BLANKSEL_PWM3H				0b0011
#define BLANKSEL_PWM4H				0b0100
#define BLANKSEL_PWM5H				0b0101

#define CHOPSEL_CHOP_CLK			0b0000
#define CHOPSEL_PWM1H				0b0001
#define CHOPSEL_PWM2H				0b0010
#define CHOPSEL_PWM3H				0b0011
#define CHOPSEL_PWM4H				0b0100
#define CHOPSEL_PWM5H				0b0101

#define CHOPHEN_ON					0b1
#define CHOPHEN_OFF					0b0

#define CHOPLEN_ON					0b1
#define CHOPLEN_OFF					0b0

#elif defined (__P33SMPS_EP7__)

// Full 16 Bit Field Flags
#define REG_HRPDIS_ON					0b1000000000000000
#define REG_HRPDIS_OFF					0b0000000000000000

#define REG_HRDDIS_ON					0b0100000000000000
#define REG_HRDDIS_OFF					0b0000000000000000

#define REG_BLANKSEL_DISABLED			0b0000000000000000
#define REG_BLANKSEL_PWM1H				0b0000000100000000
#define REG_BLANKSEL_PWM2H				0b0000001000000000
#define REG_BLANKSEL_PWM3H				0b0000001100000000
#define REG_BLANKSEL_PWM4H				0b0000010000000000
#define REG_BLANKSEL_PWM5H				0b0000010100000000

#define REG_CHOPSEL_CHOP_CLK			0b0000000000000000
#define REG_CHOPSEL_PWM1H				0b0000000000000100
#define REG_CHOPSEL_PWM2H				0b0000000000001000
#define REG_CHOPSEL_PWM3H				0b0000000000001100
#define REG_CHOPSEL_PWM4H				0b0000000000010000
#define REG_CHOPSEL_PWM5H				0b0000000000010100

#define REG_CHOPHEN_ON					0b0000000000000010
#define REG_CHOPHEN_OFF					0b0000000000000000

#define REG_CHOPLEN_ON					0b0000000000000001
#define REG_CHOPLEN_OFF					0b0000000000000000

// Single Bit Field Flags

#define HRPDIS_ON					0b1
#define HRPDIS_OFF					0b0

#define HRDDIS_ON					0b1
#define HRDDIS_OFF					0b0

#define BLANKSEL_DISABLED			0b0000
#define BLANKSEL_PWM1H				0b0001
#define BLANKSEL_PWM2H				0b0010
#define BLANKSEL_PWM3H				0b0011
#define BLANKSEL_PWM4H				0b0100
#define BLANKSEL_PWM5H				0b0101

#define CHOPSEL_CHOP_CLK			0b0000
#define CHOPSEL_PWM1H				0b0001
#define CHOPSEL_PWM2H				0b0010
#define CHOPSEL_PWM3H				0b0011
#define CHOPSEL_PWM4H				0b0100
#define CHOPSEL_PWM5H				0b0101

#define CHOPHEN_ON					0b1
#define CHOPHEN_OFF					0b0

#define CHOPLEN_ON					0b1
#define CHOPLEN_OFF					0b0

#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  CHOP - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#if defined (__P33SMPS_FJC__) || defined (__P33SMPS_FJA__) || defined (__P33SMPS_EP2__) || \
    defined (__P33SMPS_EP5__) || defined (__P33SMPS_EP7__)

// Full 16 Bit Field Flags
#define REG_CHPCLKEN_ON					0b1000000000000000
#define REG_CHPCLKEN_OFF				0b0000000000000000

// Single Bit Field Flags

#define CHPCLKEN_ON						0b1
#define CHPCLKEN_OFF					0b0

#endif

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  TRGCONx - Register Flags
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
// Full 16 Bit Field Flags
#define REG_TRGDIV_STEPSIZE_1			0b0000000000000000
#define REG_TRGDIV_STEPSIZE_2			0b0001000000000000
#define REG_TRGDIV_STEPSIZE_3			0b0010000000000000
#define REG_TRGDIV_STEPSIZE_4			0b0011000000000000
#define REG_TRGDIV_STEPSIZE_5			0b0100000000000000
#define REG_TRGDIV_STEPSIZE_6			0b0101000000000000
#define REG_TRGDIV_STEPSIZE_7			0b0110000000000000
#define REG_TRGDIV_STEPSIZE_8			0b0111000000000000
#define REG_TRGDIV_STEPSIZE_9			0b1000000000000000
#define REG_TRGDIV_STEPSIZE_10			0b1001000000000000
#define REG_TRGDIV_STEPSIZE_11			0b1010000000000000
#define REG_TRGDIV_STEPSIZE_12			0b1011000000000000
#define REG_TRGDIV_STEPSIZE_13			0b1100000000000000
#define REG_TRGDIV_STEPSIZE_14			0b1101000000000000
#define REG_TRGDIV_STEPSIZE_15			0b1110000000000000
#define REG_TRGDIV_STEPSIZE_16			0b1111000000000000

#define REG_DUAL_TRIGGER_MODE_ON		0b0000000010000000
#define REG_DUAL_TRIGGER_MODE_OFF		0b0000000000000000

#define REG_TRGSTRT_POSTSCALE_0			0b0000000000000000
#define REG_TRGSTRT_POSTSCALE_1			0b0000000000000001
#define REG_TRGSTRT_POSTSCALE_2			0b0000000000000010
#define REG_TRGSTRT_POSTSCALE_3			0b0000000000000011
#define REG_TRGSTRT_POSTSCALE_4			0b0000000000000100
#define REG_TRGSTRT_POSTSCALE_5			0b0000000000000101
#define REG_TRGSTRT_POSTSCALE_6			0b0000000000000110
#define REG_TRGSTRT_POSTSCALE_7			0b0000000000000111

#define REG_TRGSTRT_POSTSCALE_8			0b0000000000001000
#define REG_TRGSTRT_POSTSCALE_9			0b0000000000001001
#define REG_TRGSTRT_POSTSCALE_10		0b0000000000001010
#define REG_TRGSTRT_POSTSCALE_11		0b0000000000001011
#define REG_TRGSTRT_POSTSCALE_12		0b0000000000001100
#define REG_TRGSTRT_POSTSCALE_13		0b0000000000001101
#define REG_TRGSTRT_POSTSCALE_14		0b0000000000001110
#define REG_TRGSTRT_POSTSCALE_15		0b0000000000001111

#define REG_TRGSTRT_POSTSCALE_16		0b0000000000010000
#define REG_TRGSTRT_POSTSCALE_17		0b0000000000010001
#define REG_TRGSTRT_POSTSCALE_18		0b0000000000010010
#define REG_TRGSTRT_POSTSCALE_19		0b0000000000010011
#define REG_TRGSTRT_POSTSCALE_20		0b0000000000010100
#define REG_TRGSTRT_POSTSCALE_21		0b0000000000010101
#define REG_TRGSTRT_POSTSCALE_22		0b0000000000010110
#define REG_TRGSTRT_POSTSCALE_23		0b0000000000010111

#define REG_TRGSTRT_POSTSCALE_24		0b0000000000011000
#define REG_TRGSTRT_POSTSCALE_25		0b0000000000011001
#define REG_TRGSTRT_POSTSCALE_26		0b0000000000011010
#define REG_TRGSTRT_POSTSCALE_27		0b0000000000011011
#define REG_TRGSTRT_POSTSCALE_28		0b0000000000011100
#define REG_TRGSTRT_POSTSCALE_29		0b0000000000011101
#define REG_TRGSTRT_POSTSCALE_30		0b0000000000011110
#define REG_TRGSTRT_POSTSCALE_31		0b0000000000011111

#define REG_TRGSTRT_POSTSCALE_32		0b0000000000100000
#define REG_TRGSTRT_POSTSCALE_33		0b0000000000100001
#define REG_TRGSTRT_POSTSCALE_34		0b0000000000100010
#define REG_TRGSTRT_POSTSCALE_35		0b0000000000100011
#define REG_TRGSTRT_POSTSCALE_36		0b0000000000100100
#define REG_TRGSTRT_POSTSCALE_37		0b0000000000100101
#define REG_TRGSTRT_POSTSCALE_38		0b0000000000100110
#define REG_TRGSTRT_POSTSCALE_39		0b0000000000100111

#define REG_TRGSTRT_POSTSCALE_40		0b0000000000101000
#define REG_TRGSTRT_POSTSCALE_41		0b0000000000101001
#define REG_TRGSTRT_POSTSCALE_42		0b0000000000101010
#define REG_TRGSTRT_POSTSCALE_43		0b0000000000101011
#define REG_TRGSTRT_POSTSCALE_44		0b0000000000101100
#define REG_TRGSTRT_POSTSCALE_45		0b0000000000101101
#define REG_TRGSTRT_POSTSCALE_46		0b0000000000101110
#define REG_TRGSTRT_POSTSCALE_47		0b0000000000101111

#define REG_TRGSTRT_POSTSCALE_48		0b0000000010110000
#define REG_TRGSTRT_POSTSCALE_49		0b0000000010110001
#define REG_TRGSTRT_POSTSCALE_50		0b0000000010110010
#define REG_TRGSTRT_POSTSCALE_51		0b0000000010110011
#define REG_TRGSTRT_POSTSCALE_52		0b0000000010110100
#define REG_TRGSTRT_POSTSCALE_53		0b0000000010110101
#define REG_TRGSTRT_POSTSCALE_54		0b0000000010110110
#define REG_TRGSTRT_POSTSCALE_55		0b0000000010110111

#define REG_TRGSTRT_POSTSCALE_56		0b0000000010111000
#define REG_TRGSTRT_POSTSCALE_57		0b0000000010111001
#define REG_TRGSTRT_POSTSCALE_58		0b0000000010111010
#define REG_TRGSTRT_POSTSCALE_59		0b0000000010111011
#define REG_TRGSTRT_POSTSCALE_60		0b0000000010111100
#define REG_TRGSTRT_POSTSCALE_61		0b0000000010111101
#define REG_TRGSTRT_POSTSCALE_62		0b0000000010111110
#define REG_TRGSTRT_POSTSCALE_63		0b0000000010111111

// Single Bit Field Flags
#define TRGDIV_STEP_1				0b0000
#define TRGDIV_STEP_2				0b0001
#define TRGDIV_STEP_3				0b0010
#define TRGDIV_STEP_4				0b0011
#define TRGDIV_STEP_5				0b0100
#define TRGDIV_STEP_6				0b0101
#define TRGDIV_STEP_7				0b0110
#define TRGDIV_STEP_8				0b0111
#define TRGDIV_STEP_9				0b1000
#define TRGDIV_STEP_10				0b1001
#define TRGDIV_STEP_11				0b1010
#define TRGDIV_STEP_12				0b1011
#define TRGDIV_STEP_13				0b1100
#define TRGDIV_STEP_14				0b1101
#define TRGDIV_STEP_15				0b1110
#define TRGDIV_STEP_16				0b1111

#define DUAL_TRIGGER_MODE_ON		0b1
#define DUAL_TRIGGER_MODE_OFF		0b0

#define TRGSTRT_POSTSCALE_0			0b000000
#define TRGSTRT_POSTSCALE_1			0b000001
#define TRGSTRT_POSTSCALE_2			0b000010
#define TRGSTRT_POSTSCALE_3			0b000011
#define TRGSTRT_POSTSCALE_4			0b000100
#define TRGSTRT_POSTSCALE_5			0b000101
#define TRGSTRT_POSTSCALE_6			0b000110
#define TRGSTRT_POSTSCALE_7			0b000111

#define TRGSTRT_POSTSCALE_8			0b001000
#define TRGSTRT_POSTSCALE_9			0b001001
#define TRGSTRT_POSTSCALE_10		0b001010
#define TRGSTRT_POSTSCALE_11		0b001011
#define TRGSTRT_POSTSCALE_12		0b001100
#define TRGSTRT_POSTSCALE_13		0b001101
#define TRGSTRT_POSTSCALE_14		0b001110
#define TRGSTRT_POSTSCALE_15		0b001111

#define TRGSTRT_POSTSCALE_16		0b010000
#define TRGSTRT_POSTSCALE_17		0b010001
#define TRGSTRT_POSTSCALE_18		0b010010
#define TRGSTRT_POSTSCALE_19		0b010011
#define TRGSTRT_POSTSCALE_20		0b010100
#define TRGSTRT_POSTSCALE_21		0b010101
#define TRGSTRT_POSTSCALE_22		0b010110
#define TRGSTRT_POSTSCALE_23		0b010111

#define TRGSTRT_POSTSCALE_24		0b011000
#define TRGSTRT_POSTSCALE_25		0b011001
#define TRGSTRT_POSTSCALE_26		0b011010
#define TRGSTRT_POSTSCALE_27		0b011011
#define TRGSTRT_POSTSCALE_28		0b011100
#define TRGSTRT_POSTSCALE_29		0b011101
#define TRGSTRT_POSTSCALE_30		0b011110
#define TRGSTRT_POSTSCALE_31		0b011111

#define TRGSTRT_POSTSCALE_32		0b100000
#define TRGSTRT_POSTSCALE_33		0b100001
#define TRGSTRT_POSTSCALE_34		0b100010
#define TRGSTRT_POSTSCALE_35		0b100011
#define TRGSTRT_POSTSCALE_36		0b100100
#define TRGSTRT_POSTSCALE_37		0b100101
#define TRGSTRT_POSTSCALE_38		0b100110
#define TRGSTRT_POSTSCALE_39		0b100111

#define TRGSTRT_POSTSCALE_40		0b101000
#define TRGSTRT_POSTSCALE_41		0b101001
#define TRGSTRT_POSTSCALE_42		0b101010
#define TRGSTRT_POSTSCALE_43		0b101011
#define TRGSTRT_POSTSCALE_44		0b101100
#define TRGSTRT_POSTSCALE_45		0b101101
#define TRGSTRT_POSTSCALE_46		0b101110
#define TRGSTRT_POSTSCALE_47		0b101111

#define TRGSTRT_POSTSCALE_48		0b110000
#define TRGSTRT_POSTSCALE_49		0b110001
#define TRGSTRT_POSTSCALE_50		0b110010
#define TRGSTRT_POSTSCALE_51		0b110011
#define TRGSTRT_POSTSCALE_52		0b110100
#define TRGSTRT_POSTSCALE_53		0b110101
#define TRGSTRT_POSTSCALE_54		0b110110
#define TRGSTRT_POSTSCALE_55		0b110111

#define TRGSTRT_POSTSCALE_56		0b111000
#define TRGSTRT_POSTSCALE_57		0b111001
#define TRGSTRT_POSTSCALE_58		0b111010
#define TRGSTRT_POSTSCALE_59		0b111011
#define TRGSTRT_POSTSCALE_60		0b111100
#define TRGSTRT_POSTSCALE_61		0b111101
#define TRGSTRT_POSTSCALE_62		0b111110
#define TRGSTRT_POSTSCALE_63		0b111111


  
#endif
// End of File _P33SMPS_SMPS_PWM_H_
