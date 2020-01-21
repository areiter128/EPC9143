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

// List of user included header files
#include "init/init_fosc.h"
#include "init/init_timer1.h"
#include "init/init_gpio.h"

#include "init/init_adc.h"
#include "init/init_pwm.h"

#include "task_PowerControl.h"
#include "task_FaultHandler.h"


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

#define CPU_CLK_FREQUENCY        70000000 // CPU frequency in [Hz]
#define AUX_CLK_FREQUENCY       120000000 // Auxiliary Clock Frequency in [Hz]
#define PWM_CLK_FREQUENCY       960000000 // PWM Generator Base Clock Frequency in [Hz]
    
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
    
#define MAIN_EXECUTION_PERIOD    100e-6     // main state machine pace period in [sec]
#define MAIN_EXEC_PER            (uint16_t)((CPU_CLK_FREQUENCY * MAIN_EXECUTION_PERIOD)-1.0)

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

#define ADC_REF              3.300 // ADC reference voltage in V
#define ADC_RES              12.0  // ADC resolution in [bit]
#define ADC_GRAN             (float)(ADC_REF / pow(2.0, ADC_RES)) // ADC granularity in [V/tick]


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
    
#define SWITCHING_FREQUENCY         500e+3      // Power Supply Switching Frequency in [Hz]
#define NO_OF_PHASES                2U          // Number of phases of the converter
//------ macros
#define SWITCHING_PERIOD            (1.0/SWITCHING_FREQUENCY) // Power Supply Switching Period in [sec]
#define PWM_RES                     (1.0/PWM_CLK_FREQUENCY) // PWM Resolution
#define PWM_PERIOD                  (uint16_t)(SWITCHING_PERIOD / PWM_RES) // Measured in [tick = 1ns]
//------ 

#define MAXIMUM_DUTY_RATIO          0.85    // Maximum Duty Ratio in [%]
#define LEB_PERIOD                  200e-9  // Leading Edge Blanking period in [sec]
#define VOUT_ADC_TRIGGER_DELAY      ((0.85 * SWITCHING_PERIOD) - 800e-9) // ADC trigger delay in [sec] used to sample output voltage
#define PWM_MAIN_PHASE_SHIFT        (SWITCHING_PERIOD/NO_OF_PHASES)   // Switching frequency phase shift in [sec]
    
//------ macros
#define MAX_DUTY_CYCLE              (uint16_t)(PWM_PERIOD * MAXIMUM_DUTY_RATIO)     // This sets the maximum duty cycle
#define PWM_LEB_PERIOD              (uint16_t)(LEB_PERIOD / PWM_RES)  // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
#define PWM_PHASE_SHIFT             (uint16_t)(PWM_MAIN_PHASE_SHIFT / PWM_RES)   // Leading Edge Blanking = n x PWM resolution (here: 50 x 2ns = 100ns)
    
#define VOUT_ADCTRIG                (uint16_t)(VOUT_ADC_TRIGGER_DELAY / PWM_RES)    // ADC trigger delay in [ticks] used to sample output voltage

#define PWM_DEAD_TIME_RISING        50 // Rising edge dead time [2ns]
#define PWM_DEAD_TIME_FALLING       70 // Falling edge dead time [2ns]

    
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
    
#define VOUT_NOMINAL          15.0    // Nominal output voltage in [V]
#define VOUT_MAXIMUM          24.0    // Maximum output voltage in [V]
#define VOUT_HYSTERESIS       1.0     // Output voltage protection hysteresis in [V]
#define VOUT_UPPER_DEVIATION  2.0     // Upper output voltage deviation from reference in [V]
#define VOUT_LOWER_DEVIATION  0.8     // Lower output voltage deviation from reference in [V]

#define VOUT_R1           (2.0 * 2.87) // Upper voltage divider resistor in kOhm
#define VOUT_R2           1.0          // Lower voltage divider resistor in kOhm

//~~~~~~~~~~~~~~~~~
#define VOUT_FB_GAIN      (float)((SEPIC_VOUT_R2) / (SEPIC_VOUT_R1 + SEPIC_VOUT_R2))
#define VOUT_REF          (uint16_t)(SEPIC_VOUT_NOMINAL * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_OVP          (uint16_t)(SEPIC_VOUT_MAXIMUM * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_HYST         (uint16_t)(SEPIC_VOUT_HYSTERESIS * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_UDEVI        (uint16_t)(SEPIC_VOUT_UPPER_DEVIATION * SEPIC_VOUT_FB_GAIN / ADC_GRAN)
#define VOUT_LDEVI        (uint16_t)(SEPIC_VOUT_LOWER_DEVIATION * SEPIC_VOUT_FB_GAIN / ADC_GRAN)

//~~~~~~~~~~~~~~~~~
#define VIN_MINIMUM       7.0             // Minimum input voltage in [V]
#define VIN_MAXIMUM       20.0            // Maximum input voltage in [V]
#define VIN_HYSTERESIS    1.0             // Input voltage protection hysteresis in [V]
    
#define VIN_R1            15.8            // Upper voltage divider resistor in kOhm
#define VIN_R2            1.0             // Lower voltage divider resistor in kOhm

#define VIN_FB_GAIN       (float)((SEPIC_VIN_R2) / (SEPIC_VIN_R1 + SEPIC_VIN_R2))
#define VIN_UVLO          (uint16_t)(SEPIC_VIN_MINIMUM * SEPIC_VIN_FB_GAIN / ADC_GRAN)
#define VIN_OVLO          (uint16_t)(SEPIC_VIN_MAXIMUM * SEPIC_VIN_FB_GAIN / ADC_GRAN)
#define VIN_HYST          (uint16_t)(SEPIC_VIN_HYSTERESIS * SEPIC_VIN_FB_GAIN / ADC_GRAN)

    
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

#define POWER_ON_DELAY    500e-3      // power on delay in [sec]
#define RAMP_PERIOD       50e-3         // ramp period in [sec]
#define POWER_GOOD_DELAY  100e-3        // power good in [sec]

#define PODLY     (uint16_t)((POWER_ON_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define RPER      (uint16_t)((RAMP_PERIOD / MAIN_EXECUTION_PERIOD)-1.0)
#define PGDLY     (uint16_t)((POWER_GOOD_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define REF_STEP  (uint16_t)((VOUT_REF / (RPER + 1.0)))

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

#define FAULT_SHUT_DOWN_DELAY   1e-3          // shut down delay in [sec]
#define FAULT_RECOVERY_DELAY    300e-3       // recovery delay in [sec]

#define FLTTRP_DLY (uint16_t)((FAULT_SHUT_DOWN_DELAY / MAIN_EXECUTION_PERIOD)-1.0)
#define RCVRY_DLY  (uint16_t)((FAULT_RECOVERY_DELAY / MAIN_EXECUTION_PERIOD)-1.0)

    
/*!SEPIC_POWER_CONTROLLER_t data structure sepic
 * *************************************************************************************************
 * Summary:
 * Global SEPIC Signal Mapping
 * 
 * Description:
 * The SEPIC needs one PWM output, one ADC input to sample output voltage and one analog
 * feedback signal for the peak current feedback signal. In addition, the following internal 
 * peripheral instances need to be defined:
 * 
 *     - Main PWM Generator Instance
 *     - Auxiliary PWM Generator Instance
 *     - ADC trigger register
 *     - ADC input number
 *     - Comparator/DAC Instance
 *     - Comparator Input Selection
 *  
 * *************************************************************************************************/

#define _VOUT_ADCInterrupt        _ADCAN16Interrupt   
#define VIN_ADCBUF                ADCBUF12
#define VOUT_ADCBUF               ADCBUF16
#define VOUT_ADCTRIG              PG2TRIGA
#define VOUT_FEEDBACK_OFFSET      0

/*!POWER_CONTROLLER_t data structure
 * *************************************************************************************************
 * Summary:
 * Global data object for the converter 
 * 
 * Description:
 * this data object holds all status, control and monitoring values of the power 
 * controller. The POWER_CONTROLLER_t data structure is defined in app_PowerControl.h.
 * Please refer to the comments on top of this file for further information.
 *  
 * *************************************************************************************************/

extern volatile BUCK_POWER_CONTROLLER_t buck;


/*!Global Functions
 * *************************************************************************************************
 * Summary:
 * Declaration of global funciton prototypes 
 * 
 * Description:
 * All functions which should be globally accessible should be listed here
 *  
 * *************************************************************************************************/
extern volatile uint16_t Oscillator_Initialize(void);
extern volatile uint16_t SystemTimer_Initialize(void);
extern volatile uint16_t ConfigureGPIOs(void);
extern volatile uint16_t ConfigureUART(void);
extern volatile uint16_t ConfigureADC(void);
extern volatile uint16_t ConfigurePWM(void);
extern volatile uint16_t ConfigureControlLoops(void);
extern volatile uint16_t PowerControlStateMachine(void);

extern void EnableAndCalibrateADC(void);
extern void SendData(void);


    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* APPLICATION_GLOBALS_HEADER_H */

