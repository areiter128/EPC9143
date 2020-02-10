/* 
 * File:   app_PowerControl.c
 * Author: M91406
 * Comments:
 * This is the highest-level of the power converter state machine
 * Revision history: 
 * 1.0  initial release
 */

#include "globals.h"
#include "apps/app_PowerControl.h"


/* === Global Power Converter Controller Object(s) =================================================== */
volatile MPHBUCK_POWER_CONTROLLER_t mph_buck;

volatile uint16_t itrig1_max = 0;
volatile uint16_t itrig1_min = 0;
volatile uint16_t itrig2_max = 0;
volatile uint16_t itrig2_min = 0;
volatile uint16_t iphs1_max = 0;
volatile uint16_t iphs1_min = 4095;
volatile uint16_t iphs2_max = 0;
volatile uint16_t iphs2_min = 4095;

/*!PowerControl_Execute
 * ***************************************************************************************************
 * Summary:
 * Executes the power converter main state machine
 * 
 * Description:
 *  
 * *************************************************************************************************** */

volatile uint16_t PowerControl_Execute(void) {

    volatile uint16_t fres=0;
    volatile uint16_t i=0, i_sum=0;

    mph_buck.status.bits.power_source_detected = 
        (bool)((VIN_UVLO < mph_buck.data.v_in) && (mph_buck.data.v_in < VIN_OVLO));
    
#if __DPDevBoard
    mph_buck.status.bits.power_source_detected = true;
#endif
    mph_buck.status.bits.cs_calib_ready = true; // no current sense calibration required
    mph_buck.status.bits.pwm_started = PTCONbits.PTEN; // Check if PWM is enabled
    mph_buck.status.bits.forced_shut_down = false; // ToDo: Remove once fault handler is in place
    
    MPhaseBuck_Execute(&mph_buck);
    
    for(i=0; i<mph_buck.swnode.number_of_phases; i++) {
        i_sum += mph_buck.data.i_ph[i];
    }
    mph_buck.data.i_out = i_sum;
    
    return(fres);
}

/*!PowerControl_Initialize
 * ***************************************************************************************************
 * Summary:
 * Initializes the power converter driven by the main state machine
 * 
 * Description:
 * In this routine the power converter control object is configured and the respective
 * peripherals (PWM and ADC) are initialized.
 * 
 * *************************************************************************************************** */

volatile uint16_t PowerControl_Initialize(void) {

    // This routine initializes two phases of a multi-phase buck converter
    // In case this code is reused for a converter with more phases, paste
    // a message when project is built
    #if (BUCK_NO_OF_PHASES > 2)
        #pragma message "Please initialize all converter phases"
    #endif
    
    volatile uint16_t fres=0;
    volatile uint16_t i=0;

    // Initialize power converter switch node settings
    
    mph_buck.swnode.number_of_phases = BUCK_NO_OF_PHASES;   // Number of phases of the multiphase converter
    mph_buck.swnode.period = PWM_PERIOD;                    // Set common switching frequency

    mph_buck.swnode.pwm_instance[0] = BUCK_PWM_CHANNEL_PHA; // Assign PWM channel driving phase #1
    mph_buck.swnode.pwm_instance[1] = BUCK_PWM_CHANNEL_PHB; // Assign PWM channel driving phase #2
    
    // dsPIC33EP moves the PWM signal in when a phase shift value is applied
    // Thus he signal starts early. To make sure the ADC sequence is executed correctly
    // phase shift is therefore applied to phase A while phase B remains at zero
    mph_buck.swnode.phase[0] = PWM_PHASE_SHIFT;         // Set phase shift of phase A
    mph_buck.swnode.phase[1] = 0;                       // Set phase shift of phase B
    
    // Initialize timing settings of each phase with identical settings
    for(i=0; i<BUCK_NO_OF_PHASES; i++) { 
        mph_buck.swnode.duty_ratio_min[i] = PWM_DUTY_CYCLE_MIN;   // Set minimum duty cycle
        mph_buck.swnode.duty_ratio_max[i] = PWM_DUTY_CYCLE_MAX;   // Set maximum duty cycle
        mph_buck.swnode.duty_ratio_init[i] = PWM_DUTY_CYCLE_INIT; // Set initial duty cycle
        mph_buck.swnode.dead_time_rising[i] = PWM_DT_RISING;      // Set rising edge dead time
        mph_buck.swnode.dead_time_falling[i] = PWM_DT_FALLING;     // Set falling edge dead time

        mph_buck.swnode.trigger_scaler[i] = BUCK_PWM_TRG_SCALER;
    }
    
    mph_buck.swnode.trigger_offset[0] = BUCK_PWM_PHA_TRG_OFFSET;
    mph_buck.swnode.trigger_offset[1] = BUCK_PWM_PHB_TRG_OFFSET;
    
    // Initialize power converter feedback input settings
    mph_buck.feedback.ad_vin.enable = true;
    mph_buck.feedback.ad_vin.adc_input = VIN_FB_ADC_INPUT;
    mph_buck.feedback.ad_vin.adc_core = VIN_FB_ADC_CORE;
    mph_buck.feedback.ad_vin.trigger_source = VIN_FB_TRGSRC;
    mph_buck.feedback.ad_vin.differential_input = false;
    mph_buck.feedback.ad_vin.signed_result = false;
    mph_buck.feedback.ad_vin.level_trigger = false;
    mph_buck.feedback.ad_vin.early_interrupt_enable = false;
    mph_buck.feedback.ad_vin.interrupt_enable = false;

    VIN_FB_ADC_IF = 0;
    VIN_FB_ADC_IP = 0;
    VIN_FB_ADC_IE = mph_buck.feedback.ad_vin.interrupt_enable;

    mph_buck.feedback.ad_vout.enable = true;
    mph_buck.feedback.ad_vout.adc_input = VOUT_FB_ADC_INPUT;
    mph_buck.feedback.ad_vout.adc_core = VOUT_FB_ADC_CORE;
    mph_buck.feedback.ad_vout.trigger_source = VOUT_FB_TRGSRC;
    mph_buck.feedback.ad_vout.differential_input = false;
    mph_buck.feedback.ad_vout.signed_result = false;
    mph_buck.feedback.ad_vout.level_trigger = false;
    mph_buck.feedback.ad_vout.early_interrupt_enable = true;
    mph_buck.feedback.ad_vout.interrupt_enable = true;

    VOUT_FB_ADC_IF = 0;
    VOUT_FB_ADC_IP = CONTROL_LOOP_INTERRUPT_PRIORITY;
    VOUT_FB_ADC_IE = mph_buck.feedback.ad_vin.interrupt_enable;

    mph_buck.feedback.ad_iphs[0].enable = true;
    mph_buck.feedback.ad_iphs[0].adc_input = IOUT_A_ADC_INPUT;
    mph_buck.feedback.ad_iphs[0].adc_core = IOUT_A_FB_ADC_CORE;
    mph_buck.feedback.ad_iphs[0].trigger_source = IOUT_A_FB_TRGSRC;
    mph_buck.feedback.ad_iphs[0].differential_input = false;
    mph_buck.feedback.ad_iphs[0].signed_result = false;
    mph_buck.feedback.ad_iphs[0].level_trigger = false;
    mph_buck.feedback.ad_iphs[0].early_interrupt_enable = true;
    mph_buck.feedback.ad_iphs[0].interrupt_enable = true;

    IOUT_A_FB_ADC_IF = 0;
    IOUT_A_FB_ADC_IP = CONTROL_LOOP_INTERRUPT_PRIORITY;
    IOUT_A_FB_ADC_IE = mph_buck.feedback.ad_iphs[0].interrupt_enable;
    
    mph_buck.feedback.ad_iphs[1].enable = true;
    mph_buck.feedback.ad_iphs[1].adc_input = IOUT_B_ADC_INPUT;
    mph_buck.feedback.ad_iphs[1].adc_core = IOUT_B_FB_ADC_CORE;
    mph_buck.feedback.ad_iphs[1].trigger_source = IOUT_B_FB_TRGSRC;
    mph_buck.feedback.ad_iphs[1].differential_input = false;
    mph_buck.feedback.ad_iphs[1].signed_result = false;
    mph_buck.feedback.ad_iphs[1].level_trigger = false;
    mph_buck.feedback.ad_iphs[1].early_interrupt_enable = true;
    mph_buck.feedback.ad_iphs[1].interrupt_enable = true;

    IOUT_B_FB_ADC_IF = 0;
    IOUT_B_FB_ADC_IP = CONTROL_LOOP_INTERRUPT_PRIORITY;
    IOUT_B_FB_ADC_IE = mph_buck.feedback.ad_iphs[1].interrupt_enable;

    mph_buck.feedback.ad_temp.enable = false;
    
    // Power Converter Voltage Loop Initialization

    mph_buck.v_loop.controller = &v_loop;
    mph_buck.v_loop.feedback_offset = VOUT_OFFSET;
    mph_buck.v_loop.reference = VOUT_FB_REF;
    mph_buck.v_loop.trigger_offset = (VOUT_ADCTRIG_DLY + (PWM_PERIOD >> 1)); // Triggered at 50% off-time

    mph_buck.v_loop.controller->ptrSource = &VOUT_FB_ADCBUF;

    #if (BUCK_CONTROL_MODE == __ACMC)

    mph_buck.v_loop.controller->ptrTarget = &mph_buck.set_values.i_ref;
    mph_buck.v_loop.minimum = IOUTx_FB_MIN;
    mph_buck.v_loop.maximum = IOUTx_OCL_MAX;

    #elif (BUCK_CONTROL_MODE == __VMC)

    mph_buck.v_loop.controller->ptrTarget = &BUCK_PWM_PHA_DC;
    mph_buck.v_loop.minimum = PWM_DUTY_CYCLE_MIN;
    mph_buck.v_loop.maximum = PWM_DUTY_CYCLE_MAX;

    #endif

    mph_buck.v_loop.controller->ptrControlReference = &mph_buck.v_loop.reference;
    mph_buck.v_loop.controller->ptrDataProviderControlInput = &mph_buck.data.v_out;
    mph_buck.v_loop.controller->ptrADCTriggerARegister = &VOUT_FB_ADCTRIG;
    mph_buck.v_loop.controller->ADCTriggerAOffset = mph_buck.v_loop.trigger_offset;
    mph_buck.v_loop.controller->InputOffset = mph_buck.v_loop.feedback_offset;
    mph_buck.v_loop.controller->MinOutput = mph_buck.v_loop.minimum;
    mph_buck.v_loop.controller->MaxOutput = mph_buck.v_loop.maximum;

    mph_buck.v_loop.controller->status.bits.enable = false;

    // Assign control functions by loading function pointers into the data structure
    mph_buck.v_loop.ctrl_Init = &v_loop_Init;        // Function pointer to CONTROL INIT routine
    mph_buck.v_loop.ctrl_Update = &v_loop_Update;    // Function pointer to CONTROL UPDATE routine
    mph_buck.v_loop.ctrl_Precharge = &v_loop_Precharge; // Function pointer to CONTROL PRECHARGE routine
    mph_buck.v_loop.ctrl_Reset = &v_loop_Reset;     // Function pointer to CONTROL RESET routine
    
    mph_buck.v_loop.ctrl_Reset(mph_buck.v_loop.controller); // Call RESET routine of current loop controller
    
    v_loop_Init(&v_loop);

    // Power Converter Phase #1 Current Loop Initialization
    mph_buck.i_loop[0].feedback_offset = IOUTx_FB_OFFSET;
    mph_buck.i_loop[0].minimum = PWM_DUTY_CYCLE_MIN;
    mph_buck.i_loop[0].maximum = PWM_DUTY_CYCLE_MAX;
    mph_buck.i_loop[0].reference = IOUTx_FB_REF;
    mph_buck.i_loop[0].trigger_offset = IOUT_ADCTRIG_DLY; // Triggered at 50% on-time
    
    mph_buck.i_loop[0].controller = &i_loop_a;
    mph_buck.i_loop[0].controller->ptrSource = &IOUT_A_FB_ADCBUF;
    mph_buck.i_loop[0].controller->ptrTarget = &BUCK_PWM_PHA_DC;
    mph_buck.i_loop[0].controller->ptrControlReference = &mph_buck.set_values.i_ref;
    mph_buck.i_loop[0].controller->ptrDataProviderControlInput = &mph_buck.data.i_ph[0];
    mph_buck.i_loop[0].controller->ptrADCTriggerARegister = &IOUT_A_FB_ADCTRIG;
    mph_buck.i_loop[0].controller->ADCTriggerAOffset = mph_buck.i_loop[0].trigger_offset;
    mph_buck.i_loop[0].controller->InputOffset = mph_buck.i_loop[0].feedback_offset;
    mph_buck.i_loop[0].controller->MinOutput = mph_buck.i_loop[0].minimum;
    mph_buck.i_loop[0].controller->MaxOutput = mph_buck.i_loop[0].maximum;
    
    mph_buck.i_loop[0].controller->status.bits.enable = false; // Disable current loop controller

    // Initialize phase #1 current loop controller function pointers
    mph_buck.i_loop[0].ctrl_Init = &i_loop_a_Init; // Set function pointer to controller initialization routine
    mph_buck.i_loop[0].ctrl_Reset = &i_loop_a_Reset; // Set function pointer to controller history reset routine
    mph_buck.i_loop[0].ctrl_Precharge = &i_loop_a_Precharge; // Set function pointer to controller history pre-charge routine
    mph_buck.i_loop[0].ctrl_Update = &i_loop_a_Update; // Set function pointer to controller update routine

    mph_buck.i_loop[0].ctrl_Init(mph_buck.i_loop[0].controller); // Call controller initialization

    // Power Converter Phase #2 Current Loop Initialization
    mph_buck.i_loop[1].feedback_offset = IOUTx_FB_OFFSET;
    mph_buck.i_loop[1].minimum = PWM_DUTY_CYCLE_MIN;
    mph_buck.i_loop[1].maximum = PWM_DUTY_CYCLE_MAX;
    mph_buck.i_loop[1].reference = IOUTx_FB_REF;
    mph_buck.i_loop[1].trigger_offset = IOUT_ADCTRIG_DLY; // Triggered at 50% on-time
    
    mph_buck.i_loop[1].controller = &i_loop_b;
    mph_buck.i_loop[1].controller->ptrSource = &IOUT_B_FB_ADCBUF;
    mph_buck.i_loop[1].controller->ptrTarget = &BUCK_PWM_PHB_DC;
    mph_buck.i_loop[1].controller->ptrControlReference = &mph_buck.set_values.i_ref;
    mph_buck.i_loop[1].controller->ptrDataProviderControlInput = &mph_buck.data.i_ph[1];
    mph_buck.i_loop[1].controller->ptrADCTriggerARegister = &IOUT_B_FB_ADCTRIG;
    mph_buck.i_loop[1].controller->ADCTriggerAOffset = mph_buck.i_loop[1].trigger_offset;
    mph_buck.i_loop[1].controller->InputOffset = mph_buck.i_loop[1].feedback_offset;
    mph_buck.i_loop[1].controller->MinOutput = mph_buck.i_loop[1].minimum;
    mph_buck.i_loop[1].controller->MaxOutput = mph_buck.i_loop[1].maximum;
    
    mph_buck.i_loop[1].controller->status.bits.enable = false; // Disable current loop controller
    
    // Initialize phase #2 current loop controller function pointers
    mph_buck.i_loop[1].ctrl_Init = &i_loop_b_Init; // Set function pointer to controller initialization routine
    mph_buck.i_loop[1].ctrl_Reset = &i_loop_b_Reset; // Set function pointer to controller history reset routine
    mph_buck.i_loop[1].ctrl_Precharge = &i_loop_b_Precharge; // Set function pointer to controller history pre-charge routine
    mph_buck.i_loop[1].ctrl_Update = &i_loop_b_Update; // Set function pointer to controller update routine

    mph_buck.i_loop[1].ctrl_Init(mph_buck.i_loop[1].controller); // Call controller initialization

    // Initialize startup settings
    mph_buck.mode = BUCK_STATE_INITIALIZE;  // Reset power controller state machine
    
    mph_buck.startup.power_on_delay.period = PODLY;     // Set Power-On Delay
    mph_buck.startup.power_good_delay.period = PGDLY;   // Set Power-Good Delay
    mph_buck.startup.v_ramp.period = RPER;              // Set Ramp-Up Period
    mph_buck.startup.v_ramp.ref_inc_step = VREF_STEP;   // Set size of ramp-up reference increment
    if (mph_buck.startup.v_ramp.ref_inc_step == 0)      // In case macro calculation result is too small...
        mph_buck.startup.v_ramp.ref_inc_step = 1;       // ... set ramp increment to smallest feasible number
    mph_buck.startup.i_ramp.period = RPER;              // Set Ramp-Up Period
    mph_buck.startup.i_ramp.ref_inc_step = IREF_STEP;   // Set size of ramp-up reference increment
    if (mph_buck.startup.i_ramp.ref_inc_step == 0)      // In case macro calculation result is too small...
        mph_buck.startup.i_ramp.ref_inc_step = 1;       // ... set ramp increment to smallest feasible number
    
    // Initialize Default References
    mph_buck.set_values.v_ref = VOUT_FB_REF;    // Set default output voltage reference
    mph_buck.set_values.i_ref = IOUTx_FB_MAX;   // Set default output current reference (dynamic)
    
    // Initialize Runtime Data Buffers
    mph_buck.data.v_in = 0;     // Clear input voltage data buffer
    mph_buck.data.v_out = 0;    // Clear output voltage data buffer
    mph_buck.data.i_out = 0;    // Clear output current data buffer
    
    for(i=0; i<mph_buck.swnode.number_of_phases; i++)
    { mph_buck.data.i_ph[i] = 0; } // Clear phase current data buffer
    
    mph_buck.data.temp = 0;    // Clear temperature data buffer
    
    // Initialize Power Controller Status Bits
    mph_buck.status.bits.adc_active = false;            // Clear ADC_ACTIVE indicator flag
    mph_buck.status.bits.forced_shut_down = false;      // Clear FORCED_OFF indicator flag
    mph_buck.status.bits.pwm_started = false;           // Clear PWM_ACTIVE indicator flag
    mph_buck.status.bits.power_source_detected = false; // Clear POWER_SOURCE_DETECTED indicator flag
    mph_buck.status.bits.cs_calib_ready = false;        // Clear CURRENT_SENSE_CALIBRATED indicator flag

    // Initialize Power Controller Control Bits
    mph_buck.status.bits.enable = false;                // Clear ENABLE control bit
    mph_buck.status.bits.autorun = ALLOW_AUTORUN;       // Set/Clear AUTORUN control bit (converter starts automatically)
    mph_buck.status.bits.GO = false;                    // Clear GO bit (will be set by state machine)

    // Initialize ADC module and channels
    fres &= MPhaseBuck_ADC_Module_Initialize(&mph_buck);
    fres &= MPhaseBuck_ADC_Channel_Initialize(&mph_buck.feedback.ad_vin);
    fres &= MPhaseBuck_ADC_Channel_Initialize(&mph_buck.feedback.ad_vout);
    fres &= MPhaseBuck_ADC_Channel_Initialize(&mph_buck.feedback.ad_iphs[0]);
    fres &= MPhaseBuck_ADC_Channel_Initialize(&mph_buck.feedback.ad_iphs[1]);
    fres &= MPhaseBuck_ADC_Channel_Initialize(&mph_buck.feedback.ad_temp);
    fres &= MPhaseBuck_ADC_EnableAndCalibrateADC(&mph_buck);
        

    VOUT_FB_ADC_IF = 0;
    VOUT_FB_ADC_IP = 5;
    VOUT_FB_ADC_IE = 1;

    IOUT_A_FB_ADC_IF = 0;
    IOUT_A_FB_ADC_IP = 5;
    IOUT_A_FB_ADC_IE = 0;

    IOUT_B_FB_ADC_IF = 0;
    IOUT_B_FB_ADC_IP = 5;
    IOUT_B_FB_ADC_IE = 0;
    
    // Initialize PWM module and channels
    fres &= MPhaseBuck_PWM_Module_Initialize(&mph_buck);
    fres &= MPhaseBuck_PWM_Channel_Initialize(&mph_buck);
    fres &= MPhaseBuck_PWM_Enable(&mph_buck);
    
    return(fres);
}


/*!_VOUT_FB_ADC_Interrupt
 * ***********************************************************************************************
 * Voltage loop of controller is driven from the VBATA Feedback ISR
 * *********************************************************************************************** */

void __attribute__((__interrupt__, auto_psv, context)) _VOUT_FB_ADC_Interrupt() 
{	
 DBGPIN1_SET;
 
    // read most recent data
    mph_buck.status.bits.adc_active = true;     // set ISR monitoring flag bits
//    mph_buck.data.v_out = VOUT_FB_ADCBUF;       // publishing output voltage
    mph_buck.data.v_in = VIN_FB_ADCBUF;         // publishing input voltage
        
    // call voltage mode control loop
//    #if (FORCE_CONSTANT_CURRENT == false)
    v_loop_Update(&v_loop);
    BUCK_PWM_PHB_DC = BUCK_PWM_PHA_DC;
//    #elif (FORCE_CONSTANT_CURRENT == true)
//    mph_buck.set_values.i_ref = IBATx_OCL_STARTUP;
//    #endif
/*    
    if(BUCK_PWM_PHA_PTRIG < (PTPER-50))
    {
        if ( ADCBUF2 > iphs1_max ){
            iphs1_max = ADCBUF2; 
            itrig1_max = BUCK_PWM_PHA_PTRIG;
        }
        if ( ADCBUF2 < iphs1_min ){
            iphs1_min = ADCBUF2; 
            itrig1_min = BUCK_PWM_PHA_PTRIG;
        }
        BUCK_PWM_PHA_PTRIG+=8;
    }
    else    {
        BUCK_PWM_PHA_PTRIG = 0;
    }


    if(BUCK_PWM_PHA_PTRIG < (PTPER-50))
    {
        if ( ADCBUF3 > iphs2_max ){
            iphs2_max = ADCBUF3; 
            itrig2_max = BUCK_PWM_PHB_PTRIG;
        }
        if ( ADCBUF3 < iphs2_min ){
            iphs2_min = ADCBUF3; 
            itrig2_min = BUCK_PWM_PHB_PTRIG;
        }
        BUCK_PWM_PHB_PTRIG+=8;
    }
    else    {
        BUCK_PWM_PHB_PTRIG = 0;
    }
    
    #if (BUCK_CONTROL_MODE == __VMC)
//    BUCK_PWM_PWMUPD = 1;
    #endif
*/    
    // Clear ISR flag bits
	VOUT_FB_ADC_IF = 0;	// Clear interrupt flag bit

DBGPIN1_CLEAR;
    
    return;

}

/*!_IOUT_A_FB_ADC_Interrupt
 * ***********************************************************************************************
 * Current loop of Phase A
 * *********************************************************************************************** */
#if (BUCK_CONTROL_MODE == __ACMC)
void __attribute__((__interrupt__, auto_psv, context)) _IOUT_A_FB_ADC_Interrupt() 
{	

    DBGPIN1_SET; // Set debugging pin HIGH

    // read most recent data
    i_loop_a_Update(&i_loop_a);
    BUCK_PWM_PHB_DC = BUCK_PWM_PHA_DC;

//    mph_buck.status.bits.adc_active = true; // set ISR monitoring flag bits

    // PLACE USER CODE HERE

    // Clear ISR flag bits
	IOUT_A_FB_ADC_IF = 0;	// Clear interrupt flag bit

    DBGPIN1_CLEAR; // Set debugging pin HIGH
    
}


/*!_IOUT_B_FB_ADC_Interrupt
 * ***********************************************************************************************
 * Summary: Current loop of Phase B
 * 
 * Description:
 * 
 * Please note: 
 * this interrupt uses the attribute "context". This will prevent the compiler from creating 
 * additional code for context save and restore by switching to a defined, alternate set of
 * working registers. This, however, requires the correct device configuration or may result
 * in address errors during execution.
 * 
 * Please make sure you set one of the configuration bits CTXT1 to CTXTn to a specific
 * interrupt priority (e.g IPL5) and that this interrupt level matches the interrupt priority
 * set for the ADC input interrupt used below.
 * 
 * *********************************************************************************************** */

void __attribute__((__interrupt__, auto_psv, context)) _IOUT_B_FB_ADC_Interrupt() 
{	

    DBGPIN1_SET; // Set debugging pin HIGH

    // read most recent data
//    mph_buck.status.bits.adc_active = true; // set ISR monitoring flag bits

    // PLACE USER CODE HERE

    // Clear ISR flag bits
	IOUT_B_FB_ADC_IF = 0;	// Clear interrupt flag bit

    DBGPIN1_CLEAR; // Set debugging pin HIGH
    
	return;

}
#endif
/* ***********************************************************************************************
 * END OF POWER CONTROL STATE MACHINE
 * *********************************************************************************************** */


