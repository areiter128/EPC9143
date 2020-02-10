/*
 * File:   soft_start.c
 * Author: M91406
 *
 * Created on October 16, 2018, 1:07 PM
 */

#include "globals.h"
#include "drivers/drv_MPhaseBuck_control.h"
#include "init/init_pwm.h"


/* PRIVATE FUNCTION CALL PROTOTYPES */
// (none)

/*!BUCK_Execute
 * ***********************************************************************************************
 * This is the main state machine of the multiphase buck converter control object 
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_Execute(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {
    
    volatile uint16_t fres = 1;
    volatile uint16_t i=0;
    volatile float fdummy = 0.0;
    volatile uint16_t int_dummy = 0;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* DISABLE-RESET                                                                      */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    // When enable status has changed from ENABLED to DISABLED, reset the state machine
    if ((!pInstance->status.bits.enable) || 
        (!pInstance->status.bits.power_source_detected) ||
        (pInstance->status.bits.forced_shut_down))
    {
        pInstance->mode = BUCK_STATE_INITIALIZE;
    }    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* EXECUTE STATE MACHINE                                                              */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    switch (pInstance->mode) {

        /*!BUCK_STATE_INITIALIZE
         * ============================
         * In this step the soft-start procedure and control loops are reset. The soft-start ramp
         * is defined by a power on delay, pre-charge delay, ramp-up period and power good delay. 
         * The internal counters for each of these phases are reset. Then the controller reference
         * is hijacked and reset to zero. 
         * In the next function call the soft-start step POWER_ON_DELAY will be executed */
        case BUCK_STATE_INITIALIZE:

            // Set the BUSY bit indicating a delay/ramp period being executed
            pInstance->status.bits.busy = true;
            
            pInstance->startup.power_on_delay.counter = 0;   // Reset power on counter
            pInstance->startup.power_good_delay.counter = 0; // Reset power good counter

            // Disable PWM outputs & control loops (immediate power cut-off)
            MPhaseBuck_PWM_Hold(pInstance); // Disable PWM outputs

            // Reset all status bits and initiate current sensor calibration
            pInstance->status.bits.power_source_detected = false;
            pInstance->status.bits.pwm_started = false;
            pInstance->status.bits.adc_active = false;
            pInstance->status.bits.cs_calib_ready = false; 
            pInstance->status.bits.forced_shut_down = true;
            
            // switch to soft-start phase RESUME
            pInstance->mode = BUCK_STATE_RESUME;

            break;

        /*!BUCK_STATE_RESUME
         * ============================
         * After successful initialization or after an externally triggered state machine reset,
         * the state machine returns to this RESUME mode, re-initiating references and status bits.
         * before switching further into STANDBY mode
         *  */
        case BUCK_STATE_RESUME:

            // Set the BUSY bit indicating a delay/ramp period being executed
            pInstance->status.bits.busy = true;

            // Disable PWM outputs & control loops (immediate power cut-off)
            MPhaseBuck_PWM_Hold(pInstance); // Disable PWM outputs
            
            // Disable voltage loop controller and reset control loop histories
            pInstance->v_loop.controller->status.bits.enable = false; // disable voltage control loop
            pInstance->v_loop.ctrl_Reset(pInstance->v_loop.controller); // Reset control histories of outer voltage controller
            *pInstance->v_loop.controller->ptrTarget = pInstance->v_loop.controller->MinOutput;
                
            // Disable all current control loops and reset control loop histories
            for (i=0; i<pInstance->swnode.number_of_phases; i++){
                pInstance->i_loop[i].controller->status.bits.enable = false; 
                pInstance->i_loop[i].ctrl_Reset(pInstance->i_loop[i].controller); 
                *pInstance->i_loop[i].controller->ptrTarget = pInstance->i_loop[i].controller->MinOutput;
            }

            // Switch to STANDBY mode
            pInstance->mode = BUCK_STATE_STANDBY;  
            
            break;
            
        /*!BUCK_STATE_STANDBY
         * ============================
         * After a successful state machine reset, the state machine waits in STANDBY mode 
         * for being enabled again. 
         *  */
        case BUCK_STATE_STANDBY:
            
            // Set the BUSY bit indicating a delay/ramp period being executed
            pInstance->status.bits.busy = false;

            // if the 'autorun' option is set, automatically set the GO bit when the 
            // converter is enabled
            if ((pInstance->status.bits.enable) && (pInstance->status.bits.autorun))
            { pInstance->status.bits.GO = true; }
            
            // Wait for all startup conditions to be met
            if ((pInstance->status.bits.enable) &&           // state machine needs to be enabled
                (pInstance->status.bits.GO) &&               // GO-bit needs to be set
                (pInstance->status.bits.adc_active) &&       // ADC needs to be running
                (pInstance->status.bits.pwm_started) &&      // PWM needs to be running 
                (!pInstance->status.bits.forced_shut_down)   // No active fault is present
                )
            {
                // switch to soft-start phase POWER-ON DELAY
                pInstance->status.bits.GO = false;   // Clear the GO bit
                pInstance->mode = BUCK_STATE_POWER_ON_DELAY; // Step over of POD
            }
                
            break;
            
            
        /*!BUCK_STATE_POWER_ON_DELAY
         * ================================
         * In this step the soft-start procedure continues with counting up the power-on delay counter 
         * until the defined power-on delay period has expired. */
        case BUCK_STATE_POWER_ON_DELAY:

            pInstance->status.bits.busy = true;  // Set the BUSY bit

            // delay startup until POWER ON DELAY has expired
            
            if(pInstance->startup.power_on_delay.counter++ > pInstance->startup.power_on_delay.period)
            {
                // Clamp POD counter to EXPIRED
                pInstance->startup.power_on_delay.counter = 
                    (pInstance->startup.power_on_delay.period + 1); // Saturate power on counter

                if (pInstance->status.bits.cs_calib_ready) { // if current sensors is calibrated
                    pInstance->mode = BUCK_STATE_LAUNCH_V_RAMP; // ramp up output
                }

            }
                
            break;

        /*!BUCK_STATE_LAUNCH_V_RAMP
         * ================================
         * In this step the ramp up starting point is determined by measuring the input and output 
         * voltage and calculates the ideal duty ratio of the PWM. This value is then programmed into
         * the PWM module duty cycle register and is also used to pre-charge the control loop output
         * history. In addition the measured output voltage also set as reference to ensure the loop 
         * starts without jerks and jumps.
         * When voltage mode control is enabled, the voltage loop control history is charged, 
         * when average current mode control is enabled, the current loop control history is charged.
         *  */
        case BUCK_STATE_LAUNCH_V_RAMP:

            pInstance->status.bits.busy = true;  // Set the BUSY bit
            
            // Hijack voltage loop controller reference
            pInstance->startup.v_ramp.reference = 0; // Reset Soft-Start Voltage Reference
            pInstance->startup.i_ramp.reference = 0; // Reset Soft-Start Current Reference
            pInstance->v_loop.controller->ptrControlReference = &pInstance->startup.v_ramp.reference; // Voltage loop is pointing to Soft-Start Reference

            // Pre-charge reference
            // Never start above the pre-biased output voltage.
            // Always start at or slightly below the pre-biased output voltage
            pInstance->startup.v_ramp.reference = pInstance->data.v_out;
            
            // In average current mode, set current reference limit to max startup current level
            #if (BUCK_CONTROL_MODE == __ACMC)
//            pInstance->v_loop.maximum = pInstance->set_values.i_ref;
//            pInstance->v_loop.controller->MaxOutput = pInstance->v_loop.maximum;
            #endif
            
            // Pre-charge PWM and control loop history
            fdummy = (float)(pInstance->data.v_out) / (float)(pInstance->data.v_in << 1);
            int_dummy = (uint16_t)(fdummy * (float)pInstance->swnode.period);
            
            #if (BUCK_CONTROL_MODE == __VMC)
            
            if(int_dummy < pInstance->v_loop.minimum) 
            { int_dummy = pInstance->v_loop.minimum; }
            else if(int_dummy > pInstance->v_loop.maximum) 
            { int_dummy = pInstance->v_loop.maximum; }
            
            pInstance->v_loop.ctrl_Precharge(pInstance->v_loop.controller, 0, int_dummy);
            *pInstance->v_loop.controller->ptrTarget = int_dummy; // set initial PWM duty ratio

            #elif (BUCK_CONTROL_MODE == __ACMC)

            for(i=0; i<pInstance->swnode.number_of_phases; i++) {

                if(int_dummy < pInstance->i_loop[i].minimum) 
                { int_dummy = pInstance->i_loop[i].minimum; }
                else if(int_dummy > pInstance->i_loop[i].maximum) 
                { int_dummy = pInstance->i_loop[i].maximum; }

                pInstance->i_loop[i].ctrl_Precharge(
                            pInstance->i_loop[i].controller, 0, int_dummy
                        );

                *pInstance->i_loop[i].controller->ptrTarget = int_dummy; // set initial PWM duty ratio
                
            }
            
            #endif            

            Nop(); // For placing break points
            Nop();
            Nop();
            Nop();
            
            // switch to soft-start phase RAMP UP
            pInstance->mode = BUCK_STATE_V_RAMP_UP;
            
            break;
            
        /*!BUCK_STATE_V_RAMP_UP
         * ===========================
         * This is the essential step in which the output voltage is ramped up by incrementing the 
         * outer control loop reference. In voltage mode the output voltage will ramp up to the 
         * nominal regulation point. 
         * In average current mode the inner loop will limit the voltage as soon as the current 
         * reference limit is hit and the output is switched to constant current mode. */
        case BUCK_STATE_V_RAMP_UP:

            pInstance->status.bits.busy = true;  // Set the BUSY bit

            // enable control loop
            if(!pInstance->v_loop.controller->status.bits.enable) {

                // Enable input power source
                MPhaseBuck_PWM_Release(pInstance);  // Enable PWM outputs

                #if (BUCK_CONTROL_MODE == __VMC)
                pInstance->v_loop.controller->status.bits.enable = true; // enable voltage loop controller
                #elif (BUCK_CONTROL_MODE == __ACMC)
                pInstance->v_loop.controller->status.bits.enable = true; // enable voltage loop controller
                for(i=0; i<pInstance->swnode.number_of_phases; i++) {
                    pInstance->i_loop[i].controller->status.bits.enable = true; // enable phase current loop controller
                }
                #endif

            }
            
            // increment reference
            pInstance->startup.v_ramp.reference += pInstance->startup.v_ramp.ref_inc_step;
            
            // check if ramp is complete
            if (pInstance->startup.v_ramp.reference > pInstance->v_loop.reference) 
            {
                Nop(); // for breakpoint placement
                Nop();
                Nop();
                
                // Set reference to the desired level
                pInstance->startup.v_ramp.reference = pInstance->v_loop.reference;
                
                // Reconnect API reference to controller
                pInstance->v_loop.controller->ptrControlReference = &pInstance->v_loop.reference;
                
                #if (BUCK_CONTROL_MODE == __VMC)
                pInstance->mode = BUCK_STATE_PWRGOOD_DELAY;
                #elif (BUCK_CONTROL_MODE == __ACMC)
                pInstance->mode = BUCK_STATE_I_RAMP_UP;
                #endif
            }

            break;

        /*!BUCK_STATE_I_RAMP_UP
         * ===========================
         * This phase of the soft-start ramp is only executed in average current mode and will 
         * only take effect when the current limit is hit before the nominal voltage regulation 
         * point. In this case the constant output current is ramped up to from the startup current
         * to the nominal constant charging current. */
        case BUCK_STATE_I_RAMP_UP:

            pInstance->status.bits.busy = true;  // Set the BUSY bit

            // increment current limit
            pInstance->v_loop.controller->MaxOutput += pInstance->startup.i_ramp.ref_inc_step; // Increment maximum current limit

            // check if ramp is complete
            if (pInstance->v_loop.controller->MaxOutput >= pInstance->set_values.i_ref)
            {
                pInstance->v_loop.maximum = pInstance->set_values.i_ref;
                pInstance->v_loop.controller->MaxOutput = pInstance->v_loop.maximum;
                pInstance->mode = BUCK_STATE_PWRGOOD_DELAY;
            }
            break;

        /*!BUCK_STATE_PWRGOOD_DELAY
         * =============================
         * In this phase of the soft-start procedure a counter is incremented until the power good 
         * delay has expired before the soft-start process is marked as COMPLETEd */
        case BUCK_STATE_PWRGOOD_DELAY:
            // POWER GOOD Delay

            pInstance->status.bits.busy = true;  // Set the BUSY bit
            
            // increment delay counter until the GOWER GOOD delay has expired
            if(pInstance->startup.power_good_delay.counter++ > pInstance->startup.power_good_delay.period)
            {
                pInstance->mode = BUCK_STATE_ONLINE; // Set COMPLETE flag
                pInstance->startup.power_good_delay.counter = 
                    (pInstance->startup.power_good_delay.period + 1); // Clamp to PERIOD_EXPIRED for future startups
            }
            Nop();
            break;
            
        /*!BUCK_STATE_COMPLETE
         * =============================
         * When the soft-start step is set to BUCK_STATE_CHARGING, the soft-start state machine 
         * will not be executed any further and the converter has entered normal operation.  */
        case BUCK_STATE_ONLINE:
            
            /*!Runtime Reference Tuning
             * ==================================================================================
             * Description:
             * If the user reference setting has been changed and is different from the most recent 
             * controller reference/current clamping, the state machine will tune the controller 
             * reference/current clamping into the new user control reference level. 
             * While ramping the output voltage up or down, the BUSY bit will be set and any new 
             * changes to the reference will be ignored until the ramp up/down is complete.
             * =================================================================================*/

            if(pInstance->set_values.v_ref != pInstance->v_loop.reference) 
            {
                // Set the BUSY bit indicating a delay/ramp period being executed
                pInstance->status.bits.busy = true;
                
                // New reference value is less than controller reference value => ramp down
                if(pInstance->set_values.v_ref < pInstance->v_loop.reference){
                    // decrement reference until new reference level is reached
                    pInstance->v_loop.reference -= pInstance->startup.v_ramp.ref_inc_step; // decrement reference
                    if(pInstance->v_loop.reference < pInstance->set_values.v_ref) { // check if ramp is complete
                        pInstance->v_loop.reference = pInstance->set_values.v_ref; // clamp reference level to setting
                    }
                        
                }
                // New reference value is greater than controller reference value => ramp up
                else if(pInstance->set_values.v_ref > pInstance->v_loop.reference){
                    // increment reference until new reference level is reached
                    pInstance->v_loop.reference += pInstance->startup.v_ramp.ref_inc_step; // increment reference
                    if(pInstance->v_loop.reference > pInstance->set_values.v_ref) { // check if ramp is complete
                        pInstance->v_loop.reference = pInstance->set_values.v_ref; // clamp reference level to setting
                    }
                        
                }
                
            }
            else{
                // Clear the BUSY bit indicating "no state machine activity"
                pInstance->status.bits.busy = false;
            }
              
            Nop();
            Nop();
            break;

        /*!BUCK_STATE_SUSPEND
         * =============================
         * When the state machine step is set to BATCRG_STEP_SUSPEND, the PWM and control
         * loops are reset and the state machine is reset to RESUME mode WITH all
         * delay counters pre-charged. When re-enabled, the state machine will perform 
         * a soft-start without POWER ON and POWER GOOD delays.*/
        case BUCK_STATE_SUSPEND:

            // Disable PWM outputs & control loops (immediate power shut-down)
            MPhaseBuck_PWM_Hold(pInstance); // Disable PWM outputs
            
            pInstance->v_loop.controller->status.bits.enable = false;   // disable voltage control loop
            for(i=0; i<pInstance->swnode.number_of_phases; i++) {
                pInstance->i_loop[i].controller->status.bits.enable = false;  // disable current control loop
            }
            
            // Reset the state machine to repeat the soft-start without delays (ramp only)
            pInstance->startup.power_on_delay.counter = (pInstance->startup.power_on_delay.period + 1); // Clamp POD counter to PERIOD_EXPIRED for future startups
            pInstance->startup.power_good_delay.counter = (pInstance->startup.power_good_delay.period + 1); // Clamp PG counter to PERIOD_EXPIRED for future startups

            pInstance->mode = BUCK_STATE_RESUME; // Reset state machine to RESUME
            
            Nop();
            Nop();
            break;

        /*!BUCK_STATE_UNDEFINED
         * ===========================
         * When this state-machine step is executed, something went wrong in the master state machine.
         * To fix this issue the state-machine will be reset to INITIALIZATE.  */
        default:
            pInstance->mode = BUCK_STATE_INITIALIZE;
            break;
    }


    
    
    return(fres);
}
/*!MPhaseBuck_Reset
 * ***********************************************************************************************
 * Reset the state machine and all control loops to their initial defaults
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_Reset(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {

    volatile uint16_t fres = 1;
    
    // Shut down power supply
    pInstance->status.bits.forced_shut_down = true; // Set flag for FORCED_SHUT_DOWN
    pInstance->mode = BUCK_STATE_SUSPEND;           // Put state machine in SUSPEND mode
    fres &= MPhaseBuck_Execute(pInstance);          // Execute state machine once
    
    return(fres);
    
}

/*!MPhaseBuck_Suspend
 * ***********************************************************************************************
 * Resets control loops and prepares the state machine for a restart without delays 
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_Suspend(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {

    volatile uint16_t fres = 1;
    
    // Shut down power supply
    pInstance->mode = BUCK_STATE_SUSPEND;   // Set state machine to RESET
    fres &= MPhaseBuck_Execute(pInstance);  // Execute state machine once
    
    return(fres);
    
}

/*!MPhaseBuck_Resume
 * ***********************************************************************************************
 * Restarts the power control state machine without startup-delays (recovery after forced shut down)
 * *********************************************************************************************** */

volatile uint16_t MPhaseBuck_Resume(volatile MPHBUCK_POWER_CONTROLLER_t* pInstance) {

    volatile uint16_t fres=1;
    
    // Restart power supply
    pInstance->status.bits.forced_shut_down = false; // Clear flag for FORCED_SHUT_DOWN
    pInstance->status.bits.enable = true; // Enable the controller
    fres &= MPhaseBuck_Execute(pInstance); // Execute state machine once
    
    return(fres);
}


