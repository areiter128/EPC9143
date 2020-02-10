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
 * File:   drv_BuckConverter.h
 * Author: M91406
 * Comments: 
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef _BUCK_CONVERTER_ACMC_STATE_MACHINE_H_
#define	_BUCK_CONVERTER_ACMC_STATE_MACHINE_H_

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>

#include "drivers/drv_MPhaseBuck_pconfig.h"
#include "drivers/npnz16b.h"


/*!MPHBUCK_MODE_STATUS_t
 * ***************************************************************************************************
 * Summary:
 * Generic power controller status word
 * 
 * Description:
 * The power controller status/control word contains status (low-byte) and control bits (high-byte). 
 * 1) Status Bits:
 *      - ADC_ACTIVE: ADC is active and running (read only)
 *      - PWM_STARTED: PWM is active and running generating ADC triggers (read only)
 *      - POWERSOURCE_DETECTED: A valid power source has been detected allowing the converter to run (read only)
 *      - CS_READ: Current sense feedback ready (read only)
 *      - FORCED_SHUT_DOWN: Control(Status bit for external sowftware components forcing the converter to stay off
 *      - BUSY: Converter is currently going through an internal process (e.g. ramp up/down) (read only)
 * 
 * 2) Control Bits
 *      - ENABLE: Enables/Disables the power converter
 *      - AUTORUN: When set, the power converter will automatically start up once all status bits are set accordingly
 *      - GO: Control bit to manually start the power converter if (AUTOSTART=0)
 *  
 * *************************************************************************************************** */

// Controller Status Bits
#define MPHBUCK_STAT_ADC_ACTIVE             0b0000000000000001
#define MPHBUCK_STAT_PWM_STARTED            0b0000000000000010
#define MPHBUCK_STAT_POWERSOURCE_DETECTED   0b0000000000000100
#define MPHBUCK_STAT_CS_SENSE_READY         0b0000000000001000
#define MPHBUCK_STAT_FORCED_SHUT_DOWN       0b0000000000100000

#define MPHBUCK_STAT_BUSY                   0b0000000100000000

// Controller Control Bits
#define MPHBUCK_STAT_GO                     0b0010000000000000
#define MPHBUCK_STAT_AUTORUN                0b0100000000000000
#define MPHBUCK_STAT_NO_AUTORUN             0b0000000000000000

#define MPHBUCK_STAT_ENABLED                0b1000000000000000
#define MPHBUCK_STAT_DISABLED               0b0000000000000000

typedef union 
{
    struct{
        volatile bool adc_active:1;	    // Bit #0:  indicating that ADC has been started and samples are taken
        volatile bool pwm_started:1;	// Bit #1:  indicating that PWM has been started and ADC triggers are generated
        volatile bool power_source_detected:1;	// Bit #2:  indicating that a valid power source was detected
        volatile bool cs_calib_ready :1; // Bit #3:  indicating that current sensor is ready
        volatile unsigned :1;       // Bit #4: (reserved)
        volatile unsigned :1;       // Bit #5: (reserved)
        volatile bool forced_shut_down :1;  // Bit #6: Flag bit indicating system is in enforced shut down mode (usually due to a fault condition)
        volatile bool enable_switch :1; // Bit #7: Flag bit indicating that ENABLE state has changed

        volatile bool busy :1;      // Bit #8:  Flag bit indicating that the state machine is executing a process (e.g. startup-ramp)
        volatile unsigned :1;       // Bit #9: (reserved)
        volatile unsigned :1;       // Bit #10: (reserved)
        volatile unsigned :1;       // Bit #11: (reserved)
        volatile unsigned :1;       // Bit #12: (reserved)
        volatile bool GO :1;        // Bit #13: When set, the GO-bit fires up the power supply
        volatile bool autorun :1;   // Bit #14: Control bit determining if charger is starting automatically or on command (using the GO bit)
        volatile bool enable :1;    // Bit #15: Control bit enabling/disabling the charger port
    } __attribute__((packed)) bits; // data structure for single bit addressing operations

	volatile uint16_t value; // buffer for 16-bit word read/write operations
    
} MPHBUCK_MODE_STATUS_t;

/*!MPHBUCK_MODE_STATE_t
 * ***************************************************************************************************
 * Summary:
 * Generic power controller state-machine states
 * 
 * Description:
 * This enumeration is listing all defined states supported by the power controller state-machine.
 * The state machine handles the initialization of the power controller, stand-by, start up procedure
 * including Power-On-Delay, Ramp-Up and Power Good Delay until it ends up in a continuous operating
 * state. When reference values are changed while running, the state machine will tune into the new
 * reference values using the slew rates defined for the startup phase. 
 *
 * For more information on each state, please read the related sections.
 *  
 * *************************************************************************************************** */
typedef enum {
    BUCK_STATE_INITIALIZE     = 0,  // power converter control state #0: initialize variables and hijack controller reference
    BUCK_STATE_RESUME         = 1,  // power converter control state #1: Initializing variable but bypassing delays
    BUCK_STATE_STANDBY        = 2,  // power converter control state #2: standing by, ready to launch, waiting for GO (no action)
    BUCK_STATE_POWER_ON_DELAY = 3,  // power converter control state #3: power on delay (no action)
    BUCK_STATE_LAUNCH_V_RAMP  = 4,  // power converter control state #4: turn on PWM outputs and enable controller
    BUCK_STATE_V_RAMP_UP      = 5,  // power converter control state #5: perform output voltage ramp up based on parameters and system response 
    BUCK_STATE_I_RAMP_UP      = 6,  // power converter control state #6: perform output current ramp up based on parameters and system response (average current mode only)
    BUCK_STATE_PWRGOOD_DELAY  = 7,  // power converter control state #7: Output reached regulation point but waits until things have settled
    BUCK_STATE_ONLINE         = 8,  // power converter control state #8: Output in regulation and power is OK (normal continuous operation)
    BUCK_STATE_SUSPEND        = 9   // power converter control state #9: state machine will be reset with POD and PG counters bypassing delays
} MPHBUCK_MODE_STATE_t;

/*!MPHBUCK_DATA_t
 * ***************************************************************************************************
 * Summary:
 * Generic power controller runtime data object
 * 
 * Description:
 * This data structure is used as data buffer runtime data of the power controller. Each 
 * power controller offers a set of default data points such as input voltage, output voltage,
 * phase currents, total output current and board/system temperature.
 * The data will be stored as bare ADC values and might include signal offsets and specific scaling.
 * If scaled data is needed, e.g. to send via communication interfaces, the user software would
 * have to apply user-defined scalers to transform raw ADC data into physical values.
 *  
 * *************************************************************************************************** */
typedef struct {
    volatile uint16_t v_in;  // Real-time operating data input voltage
    volatile uint16_t v_out; // Real-time operating data output voltage
    volatile uint16_t i_out; // Real-time operating data output current
    volatile uint16_t i_ph[MPHBUCK_NO_OF_PHASES]; // Real-time operating data phase current #1-n
    volatile uint16_t temp;  // Real-time operating data temperature
} MPHBUCK_DATA_t;

/*!MPHBUCK_CONTROL_t
 * ***************************************************************************************************
 * Summary:
 * Generic power controller control settings
 * 
 * Description:
 * This data structure is used to set the overall settings to allow external software instances 
 * to control the power control object, such as voltage and current references.
 *  
 * *************************************************************************************************** */
typedef struct {
    volatile uint16_t v_ref; // User reference setting used to control the power converter controller
    volatile uint16_t i_ref; // User reference setting used to control the power converter controller
} MPHBUCK_CONTROL_t;

/*!MPHBUCK_STARTUP_SETTINGS_t
 * ***************************************************************************************************
 * Summary:
 * Generic power controller startup settings
 * 
 * Description:
 * This data structure is used to set the startup settings such as power on delay, power good delay
 * and ramp up time. It further covers private values like startup counters and reference values
 * for voltage and current, which are used internally by the controller (read only) but are still
 * accessible for external code modules for monitoring, diagnostics and fault handling purposes.
 * 
 * *************************************************************************************************** */
typedef struct {
    volatile uint16_t counter; // Soft-Start Execution Counter (read only)
    volatile uint16_t period;  // Soft-Start Period (POD, RAMP PERIOD, PGD, etc.)
    volatile uint16_t reference; // Internal dummy reference used to increment/decrement controller reference
    volatile uint16_t ref_inc_step; // Size/value of one reference increment/decrement or this period
} BATPORT_STARTUP_PERIOD_HANDLER_t; // Power converter soft-start auxiliary variables

typedef struct {
    volatile BATPORT_STARTUP_PERIOD_HANDLER_t power_on_delay;
    volatile BATPORT_STARTUP_PERIOD_HANDLER_t power_good_delay;
    volatile BATPORT_STARTUP_PERIOD_HANDLER_t i_ramp;
    volatile BATPORT_STARTUP_PERIOD_HANDLER_t v_ramp;
} MPHBUCK_STARTUP_SETTINGS_t; // Power converter start-up settings and variables

/*!MPHBUCK_LOOP_SETTINGS_t
 * ***************************************************************************************************
 * Summary:
 * Generic power control loop settings
 * 
 * Description:
 * This data structure is used to set the control loop settings such as pointers to controller 
 * objects and its function calls as well as basic user settings such as reference, feedback
 * signal offsets, trigger delays and minimum/maximum output clamping values.
 * 
 * *************************************************************************************************** */
typedef struct {
    // Properties (user settings)
    volatile uint16_t reference; // Control loop reference variable
    volatile uint16_t feedback_offset; // Feedback offset value for calibration or bi-direction feedback signals
    volatile uint16_t trigger_offset; // ADC trigger offset value for trigger fine-tuning
    volatile int16_t  minimum; // output clamping value (minimum)
    volatile uint16_t maximum; // output clamping value (maximum)
    // Control Loop Object
    volatile cNPNZ16b_t* controller; // pointer to control loop object data structure
    // Function pointers
    volatile uint16_t (*ctrl_Init)(volatile cNPNZ16b_t*); // Function pointer to INIT routine
    void (*ctrl_Reset)(volatile cNPNZ16b_t*); // Function pointer to RESET routine
    void (*ctrl_Update)(volatile cNPNZ16b_t*); // Function pointer to UPDATE routine
    void (*ctrl_Precharge)(volatile cNPNZ16b_t*, volatile fractional, volatile fractional); // Function pointer to PRECHARGE routine
} MPHBUCK_LOOP_SETTINGS_t; // User defined settings for control loops; 

/*!MPHBUCK_SWITCH_NODE_SETTINGS_t
 * ***************************************************************************************************
 * Summary:
 * Generic power converter switch-node specifications
 * 
 * Description:
 * This data structure is used to set the converter switch-node specifications declaring which
 * PWM channel is used as well as its switching frequency, phase-shift, dead times and duty ratio
 * limits.
 * 
 * *************************************************************************************************** */
typedef struct {
    volatile uint16_t pwm_instance[MPHBUCK_NO_OF_PHASES]; // number of the PWM channel used
    volatile uint16_t number_of_phases; // Number of phases used
    volatile uint16_t period; // Switching period
    volatile uint16_t phase[MPHBUCK_NO_OF_PHASES]; // Switching signal phase-shift
    volatile uint16_t duty_ratio_init[MPHBUCK_NO_OF_PHASES]; // Initial duty cycle when the PWM module is being turned on
    volatile uint16_t duty_ratio_min[MPHBUCK_NO_OF_PHASES]; // Absolute duty cycle minimum during normal operation
    volatile uint16_t duty_ratio_max[MPHBUCK_NO_OF_PHASES]; // Absolute duty cycle maximum during normal operation
    volatile uint16_t dead_time_rising[MPHBUCK_NO_OF_PHASES]; // Dead time setting at rising edge of a half-bridge drive
    volatile uint16_t dead_time_falling[MPHBUCK_NO_OF_PHASES]; // Dead time setting at falling edge of a half-bridge drive
    volatile uint16_t trigger_scaler[MPHBUCK_NO_OF_PHASES]; // PWM triggers for ADC will be generated every n-th cycle
    volatile uint16_t trigger_offset[MPHBUCK_NO_OF_PHASES];  // PWM triggers for ADC will be offset by n cycles
} MPHBUCK_SWITCH_NODE_SETTINGS_t; // Switching signal timing settings



/*!MPHBUCK_FEEDBACK_SETTINGS_t
 * ***************************************************************************************************
 * Summary:
 * Generic power converter switch-node specifications
 * 
 * Description:
 * This data structure is used to set the converter switch-node specifications declaring which
 * PWM channel is used as well as its switching frequency, phase-shift, dead times and duty ratio
 * limits.
 * 
 * *************************************************************************************************** */
typedef struct {
    volatile bool enable; // input channel enable bit
    volatile uint8_t adc_input; // number of the ADC input channel used
    volatile uint8_t adc_core; // Number of the ADC core connected to the selected channel
    volatile uint8_t trigger_source; // input channel trigger source
    volatile bool interrupt_enable; // input channel interrupt enable bit
    volatile bool early_interrupt_enable; // input channel early interrupt enable bit
    volatile bool differential_input; // input channel differential mode enable bit
    volatile bool signed_result; // input channel singed result mode enable bit
    volatile bool level_trigger; // input channel level trigger mode enable bit
} MPHBUCK_ADC_INPUT_SETTINGS_t; // ADC input channel configuration

typedef struct {
    volatile MPHBUCK_ADC_INPUT_SETTINGS_t ad_vin;   // ADC input sampling input voltage
    volatile MPHBUCK_ADC_INPUT_SETTINGS_t ad_vout;  // ADC input sampling output voltage
    volatile MPHBUCK_ADC_INPUT_SETTINGS_t ad_iphs[MPHBUCK_NO_OF_PHASES]; // ADC input sampling phase current
    volatile MPHBUCK_ADC_INPUT_SETTINGS_t ad_temp;  // ADC input sampling temperature
} MPHBUCK_FEEDBACK_SETTINGS_t;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*!MPHBUCK_POWER_CONTROLLER_t
 * ***************************************************************************************************
 * Summary:
 * Generic power converter switch-node specifications
 * 
 * Description:
 * This data structure is used to set the converter switch-node specifications declaring which
 * PWM channel is used as well as its switching frequency, phase-shift, dead times and duty ratio
 * limits.
 * 
 * *************************************************************************************************** */

typedef struct {
    volatile MPHBUCK_MODE_STATUS_t status;             // Status word 
    volatile MPHBUCK_DATA_t data;                      // Operation real-time data buffers
    volatile MPHBUCK_MODE_STATE_t mode;                // buck converter state machine state
    volatile MPHBUCK_CONTROL_t set_values;             // Control field for global access to references
    volatile MPHBUCK_STARTUP_SETTINGS_t startup;        // buck converter startup settings
    volatile MPHBUCK_LOOP_SETTINGS_t v_loop;           // buck converter voltage loop settings
    volatile MPHBUCK_LOOP_SETTINGS_t i_loop[MPHBUCK_NO_OF_PHASES];        // buck converter current loop settings
    volatile MPHBUCK_SWITCH_NODE_SETTINGS_t swnode; // buck converter switch-node settings
    volatile MPHBUCK_FEEDBACK_SETTINGS_t feedback; // Feedback input channel configuration
} MPHBUCK_POWER_CONTROLLER_t; // Generic multi-phase buck converter controller object

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

extern volatile uint16_t MPhaseBuck_Execute(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_Reset(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_Suspend(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_Resume(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);

extern volatile uint16_t MPhaseBuck_PWM_Module_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_PWM_Channel_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_PWM_Enable(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_PWM_Disable(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_PWM_Hold(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_PWM_Release(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);

extern volatile uint16_t MPhaseBuck_ADC_Module_Initialize(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);
extern volatile uint16_t MPhaseBuck_ADC_Channel_Initialize(volatile MPHBUCK_ADC_INPUT_SETTINGS_t* pInstance);
extern volatile uint16_t MPhaseBuck_ADC_EnableAndCalibrateADC(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance);

#endif	/* _BUCK_CONVERTER_ACMC_STATE_MACHINE_H_ */

