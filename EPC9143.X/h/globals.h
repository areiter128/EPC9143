/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   globals.h
 * Author: M91406
 * Comments: global defines of this application
 * Revision history: 
 * v1.0 initial version
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef APPLICATION_GLOBALS_HEADER_H
#define	APPLICATION_GLOBALS_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

// Include device driver files
#include "mcal/p33SMPS_devices.h"

// List of user included header files
//#include "init/init_fosc.h"
//#include "init/init_timer1.h"
//#include "init/init_gpio.h"
//
//#include "init/init_adc.h"
//#include "init/init_pwm.h"
//
//#include "task_PowerControl.h"
//#include "task_FaultHandler.h"


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*!Microcontroller and Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for device specific parameters
 * 
 * Description:
 * This section is used to define device specific parameters like clock settings, analog 
 * reference and resolution of ADC or DAC. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

    
/*!Microcontroller Abstraction
 * *************************************************************************************************
 * Summary:
 * Global Signal Mapping
 * 
 * Description:
 * 
 *  
 * *************************************************************************************************/
#define CONTROL_LOOP_INTERRUPT_PRIORITY     5   // all control loop ISRs are called on priority 5

#define VOUT_FB_ADC_INPUT       0
#define VOUT_FB_ADC_CORE        0
#define VOUT_FB_ADCBUF          ADCBUF0
#define _VOUT_FB_ADC_Interrupt  _ADCAN0Interrupt   
#define VOUT_FB_ADC_IF          _ADCAN0IF
#define VOUT_FB_ADC_IP          _ADCAN0IP
#define VOUT_FB_ADC_IE          _ADCAN0IE
#define VOUT_FB_TRGSRC          BUCK_PWM_PHB_STRGSRC // PWM Generator 2 secondary trigger
#define VOUT_FB_TRGSRC_REGISTER _TRGSRC0
#define VOUT_FB_ADIE            ADIELbits.IE0
#define VOUT_FB_ADEIE           ADEIELbits.EIEN0
#define VOUT_FB_ADCTRIG         BUCK_PWM_PHB_STRIG
    
#define VIN_FB_ADC_INPUT        1
#define VIN_FB_ADC_CORE         1
#define VIN_FB_ADCBUF           ADCBUF1
#define _VIN_FB_ADC_Interrupt   _ADCAN1Interrupt   
#define VIN_FB_ADC_IF           _ADCAN1IF
#define VIN_FB_ADC_IP           _ADCAN1IP
#define VIN_FB_ADC_IE           _ADCAN1IE
#define VIN_FB_TRGSRC           BUCK_PWM_PHB_STRGSRC // PWM Generator 2 secondary trigger
#define VIN_FB_TRGSRC_REGISTER  _TRGSRC1
#define VIN_FB_ADIE             ADIELbits.IE1
#define VIN_FB_ADEIE            ADEIELbits.EIEN1
#define VIN_FB_ADCTRIG          BUCK_PWM_PHB_STRIG
    
#define IOUT_A_ADC_INPUT        2
#define IOUT_A_FB_ADC_CORE      2
#define IOUT_A_FB_ADCBUF        ADCBUF2
#define _IOUT_A_FB_ADC_Interrupt _ADCAN2Interrupt   
#define IOUT_A_FB_ADC_IF        _ADCAN2IF
#define IOUT_A_FB_ADC_IP        _ADCAN2IP
#define IOUT_A_FB_ADC_IE        _ADCAN2IE
#define IOUT_A_FB_TRGSRC        BUCK_PWM_PHA_PTRGSRC // PWM Generator 1 primary trigger
#define IOUT_A_FB_TRGSRC_REGISTER _TRGSRC2
#define IOUT_A_FB_ADIE          ADIELbits.IE2
#define IOUT_A_FB_ADEIE         ADEIELbits.EIEN2
#define IOUT_A_FB_ADCTRIG       BUCK_PWM_PHA_PTRIG

#define IOUT_B_ADC_INPUT        3
#define IOUT_B_FB_ADC_CORE      3
#define IOUT_B_FB_ADCBUF        ADCBUF3
#define _IOUT_B_FB_ADC_Interrupt _ADCAN3Interrupt   
#define IOUT_B_FB_ADC_IF        _ADCAN3IF
#define IOUT_B_FB_ADC_IP        _ADCAN3IP
#define IOUT_B_FB_ADC_IE        _ADCAN3IE
#define IOUT_B_FB_TRGSRC        BUCK_PWM_PHB_PTRGSRC // PWM Generator 2 primary trigger
#define IOUT_B_FB_TRGSRC_REGISTER _TRGSRC3
#define IOUT_B_FB_ADIE          ADIELbits.IE3
#define IOUT_B_FB_ADEIE         ADEIELbits.EIEN3
#define IOUT_B_FB_ADCTRIG       BUCK_PWM_PHB_PTRIG

#define DBGPIN1_SET             {_LATB7 = 1;}
#define DBGPIN1_CLEAR           {_LATB7 = 0;}
    
/*!Device Clock Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for device clock settings
 * 
 * Description:
 * This section is used to define device specific parameters related to the core clock and 
 * auxiliary clock used to drive PWM, ADC and DAC.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

#define FRC_BASE_FREQUENCY        7370000 // Internal RC Frequency (Default)
#define FRC_TUNING                     31 // FRC Tuning Value
#define FRC_TUNING_FACTOR         0.00047 // Tuning factor per tick
    
#define OSC_PLL_N1  2   // Oscillator PLL Pre-Scaler = 2
#define OSC_PLL_M   75  // Oscillator PLL Multiplier = 75
#define OSC_PLL_N2  2   // Oscillator PLL Post-Scaler = 2
    
#define FRC_FREQUENCY   (uint32_t)(FRC_BASE_FREQUENCY * (1.0 + (FRC_TUNING * FRC_TUNING_FACTOR))) // Internal RC Frequency

#define FCY     (uint32_t)((((FRC_FREQUENCY / OSC_PLL_N2) * OSC_PLL_M ) / OSC_PLL_N2) / 2)
#define TCY     (float)(1.0 / (float)FCY)
#define FACLK   (uint32_t)(FRC_FREQUENCY * 16.0)
#define TACLK   (float)(1.0 / (float)FACLK)
#define FPWMCLK (uint32_t)(FACLK * 8.0)         // PWM Generator Base Clock Frequency in [Hz]
#define TPWMCLK (float)(1.0 / (float)FPWMCLK)   // PWM Generator Base Clock Period in [sec]

    
/*!State Machine Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for state-machine specific parameters
 * 
 * Description:
 * This section is used to define state-machine settings such as the main execution call interval. 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to be 
 * written to SFRs and variables.
 * 
 * *************************************************************************************************/
    
#define MAIN_EXECUTION_PERIOD   100e-6     // main state machine pace period in [sec]
#define MAIN_EXEC_PER           (uint16_t)((FCY * MAIN_EXECUTION_PERIOD)-1.0)
#define MAIN_TMR_TIMEOUT        50000U
    
/*!ADC Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device ADC
 * 
 * Description:
 * This section is used to define device specific parameters of ADC reference, resolution
 * and granularity to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/

#define ADC_REF             3.300 // ADC reference voltage in V
#define ADC_RES             12.0  // ADC resolution in [bit]
#define ADC_GRAN            (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]
#define ADC_SCALER_V2TICK   (float)(1.0/ADC_GRAN) // ADC value scaler Volt-2-Ticks

/*!PWM Settings
 * *************************************************************************************************
 * Summary:
 * Global defines for specific parameters of the device PWM
 * 
 * Description:
 * This section is used to define device specific parameters of PWM frequency, may duty ratio, 
 * leading edge blanking, slope compensation and ADC triggers.
 * granularity and slope timer frequency to calculate register values representing physical voltages.
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers 
 * to be written to SFRs
 * 
 * *************************************************************************************************/
    
#define BUCK_PWM_CHANNEL_PHA        1U // PWM Channel driving phase A
#define BUCK_PWM_PHA_PTRGSRC        0b00101 // Primary Trigger source value of PWM channel A
#define BUCK_PWM_PHA_STRGSRC        0b01111 // Secondary Trigger source value of PWM channel A
#define BUCK_PWM_PHA_DC             PDC1
#define BUCK_PWM_PHA_PTRIG          TRIG1
#define BUCK_PWM_PHA_STRIG          STRIG1
    
#define BUCK_PWM_CHANNEL_PHB        4U // PWM Channel driving phase B
#define BUCK_PWM_PHB_PTRGSRC        0b01000 // Primary trigger source value of PWM channel B
#define BUCK_PWM_PHB_STRGSRC        0b10010 // Secondary trigger source value of PWM channel B
#define BUCK_PWM_PHB_DC             PDC4
#define BUCK_PWM_PHB_PTRIG          TRIG4
#define BUCK_PWM_PHB_STRIG          STRIG4

#define BUCK_PWM_SEVTRGSRC          0b01110 // Special Event Trigger Source value

#define BUCK_PWM_TRG_SCALER         2U
#define BUCK_PWM_PHA_TRG_OFFSET     0U
#define BUCK_PWM_PHB_TRG_OFFSET     1U
    
#define SWITCHING_FREQUENCY         (float)500e+3   // Power Supply Switching Frequency in [Hz]
#define BUCK_NO_OF_PHASES           2U              // Number of phases of the converter
//------ macros
#define SWITCHING_PERIOD            (1.0/SWITCHING_FREQUENCY) // Power Supply Switching Period in [sec]
#define PWM_RES                     (1.0/FPWMCLK) // PWM Resolution
#define PWM_PERIOD                  (uint16_t)(SWITCHING_PERIOD / PWM_RES) // Measured in [tick = 1ns]
//------ 

//------ DEBUGGING SETTINGS / USER OPTIONS
#ifndef __VMC
  #define __VMC     0
#endif
#ifndef __ACMC
  #define __ACMC    1
#endif
    
#define BUCK_CONTROL_MODE       __VMC  // Set control mode 
                                        //      VMC  = Voltage Mode Control
                                        //      ACMC = Average Current Mode Control

#define FORCE_CONSTANT_CURRENT  false   // Flag used during debugging effectively disabling the 
                                        // outer voltage loop (used during current loop tuning)

#define ALLOW_AUTORUN           true    // When set, power converter will start up automatically
                                        // without GO command. When cleared, power converter is started
                                        // from user code by setting the GO command.
//------ 
    
#define MINIMUM_DUTY_RATIO          (float)0.005   // Minimum Duty Ratio in [%]
#define MAXIMUM_DUTY_RATIO          (float)0.500   // Maximum Duty Ratio in [%]
#define INITIAL_DUTY_RATIO          (float)0.010   // Minimum Duty Ratio in [%]
#define LEB_PERIOD                  (float)200e-9  // Leading Edge Blanking period in [sec]
#define VOUT_ADC_TRIGGER_DELAY      (float)50e-9   // ADC trigger delay in [sec] used to sample output voltage
#define IOUT_ADC_TRIGGER_DELAY      (float)400e-9   // ADC trigger delay in [sec] used to sample output voltage
#define PWM_MAIN_PHASE_SHIFT        (SWITCHING_PERIOD/BUCK_NO_OF_PHASES)   // Switching frequency phase shift in [sec]
    
//------ macros
#define PWM_DUTY_CYCLE_MIN          (uint16_t)(PWM_PERIOD * MINIMUM_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_DUTY_CYCLE_MAX          (uint16_t)(PWM_PERIOD * MAXIMUM_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_DUTY_CYCLE_INIT         (uint16_t)(PWM_PERIOD * INITIAL_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_LEB_DELAY               (uint16_t)(LEB_PERIOD / PWM_RES)  // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define PWM_PHASE_SHIFT             (uint16_t)(PWM_MAIN_PHASE_SHIFT / PWM_RES)   // PWM Phase Shift= [period/phases] x PWM resolution
    
#define VOUT_ADCTRIG_DLY            (uint16_t)(VOUT_ADC_TRIGGER_DELAY / PWM_RES)    // ADC trigger delay in [ticks] used to sample output voltage
#define IOUT_ADCTRIG_DLY            (uint16_t)(IOUT_ADC_TRIGGER_DELAY / PWM_RES)    // ADC trigger delay in [ticks] used to sample output voltage

#define PWM_DEAD_TIME_RISING        (float)50e-9 // Rising edge dead time 
#define PWM_DEAD_TIME_FALLING       (float)50e-9 // Falling edge dead time

#define PWM_DT_RISING               (uint16_t)(PWM_DEAD_TIME_RISING / PWM_RES)
#define PWM_DT_FALLING              (uint16_t)(PWM_DEAD_TIME_FALLING / PWM_RES)
    
/*!Hardware Abstraction
 * *************************************************************************************************
 * Summary:
 * Global defines for hardware specific parameters
 * 
 * Description:
 * This section is used to define hardware specific parameters such as output voltage dividers,
 * reference levels or feedback gains. Pre-compiler macros are used to translate physical  
 * values into binary (integer) numbers to be written to SFRs
 * 
 * *************************************************************************************************/
    
#define VOUT_NOMINAL          3.0      // Nominal output voltage in [V]
#define VOUT_MAXIMUM          (VOUT_NOMINAL + 0.5)      // Maximum output voltage in [V]
#define VOUT_HYSTERESIS       1.0       // Output voltage protection hysteresis in [V]
#define VOUT_UPPER_DEVIATION  2.0       // Upper output voltage deviation from reference in [V]
#define VOUT_LOWER_DEVIATION  0.8       // Lower output voltage deviation from reference in [V]
#define VOUT_FEEDBACK_OFFSET  0.0       // Static feedback offset in [V]
    
#define VOUT_R1           (float)18.0   // Upper voltage divider resistor in kOhm
#define VOUT_R2           (float)4.75   // Lower voltage divider resistor in kOhm

//~~~~~~~~~~~~~~~~~
#define VOUT_FB_GAIN      (float)((VOUT_R2) / (VOUT_R1 + VOUT_R2))
#define VOUT_FB_REF       (uint16_t)(VOUT_NOMINAL * VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_OVP          (uint16_t)(VOUT_MAXIMUM * VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_HYST         (uint16_t)(VOUT_HYSTERESIS * VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_UDEVI        (uint16_t)(VOUT_UPPER_DEVIATION * VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_LDEVI        (uint16_t)(VOUT_LOWER_DEVIATION * VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_OFFSET       (uint16_t)(VOUT_FEEDBACK_OFFSET * VOUT_FB_GAIN / ADC_GRAN)

//~~~~~~~~~~~~~~~~~
#define VIN_MINIMUM       (VOUT_NOMINAL + 5.0) // Minimum input voltage in [V]
#define VIN_MAXIMUM       54.0            // Maximum input voltage in [V]
#define VIN_HYSTERESIS    1.0             // Input voltage protection hysteresis in [V]
    
#define VIN_R1            (float)110.0    // Upper voltage divider resistor in kOhm
#define VIN_R2            (float)4.87     // Lower voltage divider resistor in kOhm

#define VIN_FB_GAIN       (float)((VIN_R2) / (VIN_R1 + VIN_R2))
#define VIN_UVLO          (uint16_t)(VIN_MINIMUM * VIN_FB_GAIN / ADC_GRAN)
#define VIN_OVLO          (uint16_t)(VIN_MAXIMUM * VIN_FB_GAIN / ADC_GRAN)
#define VIN_HYST          (uint16_t)(VIN_HYSTERESIS * VIN_FB_GAIN / ADC_GRAN)


#define IOUT_SHUNT_RESISTANCE   (float)0.001 // Shunt resistance in [Ohm]
#define IOUT_GAIN               (float)100.0 // Curernt Sense Gain
    
#define IOUT_FB_SCALER_I2V      (float)(IOUT_SHUNT_RESISTANCE * IOUT_GAIN) // Curernt Sense Scaler I2V
#define IOUT_FB_SCALER_TICKS    (float)(IOUT_FB_SCALER_I2V * ADC_SCALER_V2TICK)

#define IOUTx_FB_REFERENCE      (float)0.0  // Static reference in [A] when operated as constant current source 
#define IOUT_FEEDBACK_OFFSET    (float)0.0  // Static feedback offset visible at ADC input in [V]
#define IOUTx_FB_MINIMUM        (float)-0.2 // Minimum Current Limit in [A]
#define IOUTx_FB_MAXIMUM        (float)1.0  // Maximum Current Limit in [A]
#define IOUTx_OCL_MAXIMUM       (float)1.0  // Over Current Limit in [A]
#define IOUTx_OCL_STARTUP       (float)1.0  // Over Current Limit @ Startup in [A] => Inrush Current
    
#define IOUTx_FB_REF            (uint16_t)(IOUTx_FB_REFERENCE * IOUT_FB_SCALER_TICKS)
#define IOUTx_FB_OFFSET         (uint16_t)(IOUT_FEEDBACK_OFFSET / ADC_GRAN)
#define IOUTx_FB_MIN            (uint16_t)(IOUTx_FB_MINIMUM * IOUT_FB_SCALER_TICKS)
#define IOUTx_FB_MAX            (uint16_t)(IOUTx_FB_MAXIMUM * IOUT_FB_SCALER_TICKS)
#define IOUTx_OCL_MAX           (uint16_t)(IOUTx_OCL_MAXIMUM * IOUT_FB_SCALER_TICKS)
#define IBATx_OCL_STARTUP       (uint16_t)(IOUTx_OCL_STARTUP * IOUT_FB_SCALER_TICKS)
    
/*!Startup Behavior
 * *************************************************************************************************
 * Summary:
 * Global defines for soft-start specific parameters
 * 
 * Description:
 * This section is used to define power supply startup timing setting. The soft-start sequence 
 * is part of the power controller. It allows to program specific timings for Power On Delay,
 * Ramp Period and Power Good Delay. After the startup has passed these three timing periods,
 * the power supply is ending up in "normal" operation, continuously regulating the output until 
 * a fault is detected or the operating state is changed for any other reason.
 * 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to 
 * be written to SFRs and variables.
 * 
 * *************************************************************************************************/

#define POWER_ON_DELAY    100e-3    // power on delay in [sec]
#define RAMP_PERIOD       500e-3     // ramp period in [sec]
#define POWER_GOOD_DELAY  100e-3    // power good in [sec]

#define PODLY     (uint16_t)((POWER_ON_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define RPER      (uint16_t)((RAMP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)
#define PGDLY     (uint16_t)((POWER_GOOD_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define VREF_STEP (uint16_t)((VOUT_FB_REF / (RPER + 1.0)))
#define IREF_STEP (uint16_t)(((IOUTx_FB_MAX - IOUTx_FB_OFFSET) / (RPER + 1.0)))
    
/*!FAULT Shut Down and Recover
 * *************************************************************************************************
 * Summary:
 * Global defines for FAULT specific parameters
 * 
 * Description:
 * The parameters defined in this section specify FAULT shut down and recovery delay filters.
 * Delay filters are adjustable in n x main execution period (e.g. n x 100usec).
 * 
 * Pre-compiler macros are used to translate physical values into binary (integer) numbers to 
 * be written to SFRs and variables.
 * 
 * *************************************************************************************************/

#define FAULT_SHUT_DOWN_DELAY   (float)1e-3     // shut down delay in [sec]
#define FAULT_RECOVERY_DELAY    (float)300e-3   // recovery delay in [sec]

#define FLTTRP_DLY (uint16_t)((FAULT_SHUT_DOWN_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define RCVRY_DLY  (uint16_t)((FAULT_RECOVERY_DELAY / MAIN_EXECUTION_PERIOD)-1.0)



/*!Global Functions
 * *************************************************************************************************
 * Summary:
 * Declaration of global function prototypes 
 * 
 * Description:
 * All functions which should be globally accessible should be listed here
 *  
 * *************************************************************************************************/
extern volatile uint16_t Oscillator_Initialize(void);
extern volatile uint16_t SystemTimer_Initialize(void);
extern volatile uint16_t DSP_Initialize(void);
extern volatile uint16_t ConfigureGPIOs(void);
extern volatile uint16_t ConfigureUART(void);
//extern volatile uint16_t ConfigureADC(void);
//extern volatile uint16_t ConfigurePWM(void);
//extern volatile uint16_t ConfigureControlLoops(void);
extern volatile uint16_t PowerControlStateMachine(void);

extern void SendData(void);


    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* APPLICATION_GLOBALS_HEADER_H */

