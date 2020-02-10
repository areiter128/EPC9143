/*LICENSE ********************************************************************
 * Microchip Technology Inc. and its subsidiaries.  You may use this software 
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
 * ***************************************************************************/
/* @@p33SMPS_oscillator.h
 * ***************************************************************************
 *
 * File:   p33SMPS_oscillator.h
 * Author: M91406
 *
 * Created on October 27, 2017, 11:24 AM
 * ***************************************************************************/

#ifndef __MCAL_P33SMPS_OSCILLATOR_H__
#define __MCAL_P33SMPS_OSCILLATOR_H__

#include "mcal/p33SMPS_devices.h"

#if defined (__P33SMPS_EP__)
/* TLAL, TLAH and TLAY Devices */

#include <stdint.h>
#include <stdbool.h>

/*!System Frequencies
 * ************************************************************************************************
 * Summary:
 * Set of defines, data types and data structures for system frequency adaption
 *
 * Description:
 * This library offers default preset for CPU frequencies to simplify the oscillator 
 * configuration for standard applications using the internal Fast RC oscillator (FRC).
 * These parameters are supposed to be used with the function call init_FRCCLK_Defaults().
 * To change the CPU frequency at any point during runtime, clock switching must be enabled
 * by setting the configuration bit FCKSM:
 * 
 *      #pragma config FCKSM = CSECMD (Clock Switching Enabled/Clock Monitor Disabled) 
 *   or                        CSECME (Clock Switching Enabled/Clock Monitor Enabled)
 * 
 * Example:
 * The following code line configures the internal FRC oscillator for 80 MIPS operation:
 * 
 *      init_FRCCLK_Defaults(CPU_SPEED_80_MIPS);    // Configuring FRC for 80 MIPS operation
 * 
 * ***********************************************************************************************/

typedef enum 
{
    CPU_SPEED_20_MIPS = 20, // CPU Speed setting for 20 MIPS operation
    CPU_SPEED_30_MIPS = 30, // CPU Speed setting for 30 MIPS operation
    CPU_SPEED_40_MIPS = 40, // CPU Speed setting for 40 MIPS operation
    CPU_SPEED_50_MIPS = 50, // CPU Speed setting for 50 MIPS operation
    CPU_SPEED_60_MIPS = 60, // CPU Speed setting for 60 MIPS operation
    CPU_SPEED_70_MIPS = 70  // CPU Speed setting for 70 MIPS operation
} CPU_SPEED_DEFAULTS_e;  // Default CPU speed settings 


typedef enum 
{
    AFPLLO_100_MHZ  = 100, // Auxiliary PLL output frequency of 500 MHz
    AFPLLO_200_MHZ  = 200, // Auxiliary PLL output frequency of 500 MHz
    AFPLLO_300_MHZ  = 300, // Auxiliary PLL output frequency of 500 MHz
    AFPLLO_400_MHZ  = 400, // Auxiliary PLL output frequency of 500 MHz
    AFPLLO_500_MHZ  = 500, // Auxiliary PLL output frequency of 500 MHz (Default for high resolution PWM)
    AFPLLO_600_MHZ  = 600, // Auxiliary PLL output frequency of 600 MHz
    AFPLLO_700_MHZ  = 700, // Auxiliary PLL output frequency of 700 MHz
    AFPLLO_800_MHZ  = 800  // Auxiliary PLL output frequency of 800 MHz
} AUX_PLL_DEFAULTS_e;  // Default Auxiliary PLL output frequency settings 


/*!System OSCILLATOR_SYSTEM_FREQUENCIES_t
 * ************************************************************************************************
 * Summary:
 * Global data structure holding system frequencies of different clock domains
 *
 * Description:
 * The data structure "system_frequencies" of type OSCILLATOR_SYSTEM_FREQUENCIES_t is used
 * to broadcast most recent system frequencies of multiple clock domains. Contents of this data 
 * structure are NOT updated automatically. 
 * 
 * The function osc_get_frequencies() must be called from user code to update/refresh the 
 * contents of this data structure every time a oscillator configuration has been changed.
 *
 * Example:
 * The following code lines initialize the internal FRC oscillator for 100 MIPS operation and
 * the auxiliary PLL for 500 MHz to support 250ps resolution of the PWM module. After both 
 * configurations have been set, the function 'osc_get_frequencies()' is used to update the 
 * most recent frequencies of multiple clock domains.
 * 
 *      init_FRCCLK_Defaults(CPU_SPEED_100_MIPS);   // Initialize FRC for 100 MIPS operation
 *      init_AUXCLK_500MHz();                       // Initialize AuxPLL for 500 MHz clock output
 *      osc_get_frequencies(0);                     // Update system frequencies data structure
 * 
 * Please note: 
 * When an external oscillator is used, the function osc_get_frequencies() must be called 
 * to set the external frequency value in [Hz] and update all related frequencies accordingly.
 * If only the internal FRC oscillator is used, this parameter should be set = 0.
 * ***********************************************************************************************/

typedef struct {
    volatile uint32_t frc;      // Internal fast RC oscillator frequency incl. tuning
    volatile uint32_t fpri;     // External primary oscillator frequency 
    volatile uint32_t fclk;     // Clock frequency (external or internal oscillator frequency)
    volatile uint32_t fosc;     // Oscillator frequency
    volatile uint32_t fcy;      // CPU click frequency (instruction frequency = MIPS incl. DOZE divider)
    volatile uint32_t fp;       // Peripheral bus clock frequency
    volatile uint32_t fpllo;    // PLL output frequency
    volatile uint32_t fvco;     // PLL VCO frequency output incl. divider
    volatile float tp;          // Peripheral clock period 
    volatile float tcy;         // CPU clock period 
    volatile uint32_t afpllo;   // APLL output frequency
    volatile uint32_t afvco;    // APLL VCO frequency output incl. divider
}OSCILLATOR_SYSTEM_FREQUENCIES_t;


/* FRC oscillator settings */

#define FRCTUN_MIN      -32     // minimum tuning value
#define FRCTUN_MAX      31      // maximum tuning value

#define OSC_FRC_FREQ    7370000     // Frequency of the internal oscillator in [Hz]
#define OSC_FRC_TUN     31          // <OSCTUN> FRC Oscillator Tuning Rregister value
#define OSC_TUN_SCALER  0.00047     // Oscillator frequency step size of <OSCTUN>


#if (USE_EXTERNAL_OSC == 0)     // if device is running with internal FRC oscillator
    #define OSC_FREQ ((uint32_t)((float)OSC_FRC_FREQ + (float)((float)OSC_FRC_TUN * (OSC_TUN_SCALER * (float)OSC_FRC_FREQ))))
#else
    #define OSC_FREQ  EXT_OSC_FREQ	// External oscillator frequency in [Hz] as defined by user in device_config.h
#endif



/* ***************************************************************************************
 *	BASIC DEFINES
 * **************************************************************************************/

/* ===========================================================================
 * OSCCON: OSCILLATOR CONTROL REGISTER
 * ===========================================================================*/

#define REG_OSCCON_VALID_DATA_WRITE_MASK    0x0789
#define REG_OSCCON_VALID_DATA_READ_MASK     0x77A9

#define REG_OSCCON_OSWEN_REQUEST_SWITCH  0b0000000000001000
#define REG_OSCCON_OSWEN_SWITCH_COMPLETE 0b0000000000000000

typedef enum {
    OSCCON_OSWEN_REQUEST_SWITCH = 0b1, // Requests oscillator switch to the selection specified by the NOSC<2:0> bits
    OSCCON_OSWEN_SWITCH_COMPLETE = 0b0 // Oscillator switch is complete
} OSCCON_OSWEN_e; // Oscillator Switch Enable bit

#define REG_OSCCON_CF_CLKSTAT_FAIL 0b0000000000001000
#define REG_OSCCON_CF_CLKSTAT_OK   0b0000000000000000

typedef enum {
    OSCCON_CF_CLKSTAT_FAIL = 0b1, // FSCM has detected a clock failure
    OSCCON_CF_CLKSTAT_OK = 0b0 // FSCM has not detected a clock failure
} OSCCON_CF_e; // Clock Fail Detect bit

#define REG_OSCCON_LOCK_PLL_LOCKED   0b0000000000100000
#define REG_OSCCON_LOCK_PLL_UNLOCKED 0b0000000000000000

typedef enum {
    OSCCON_LOCK_PLL_LOCKED = 0b1, // Indicates that PLL is in lock or PLL start-up timer is satisfied
    OSCCON_LOCK_PLL_UNLOCKED = 0b0 // Indicates that PLL is out of lock, start-up timer is in progress or PLL is disabled
} OSCCON_LOCK_e; // PLL Lock Status bit (read-only)

#define REG_OSCCON_CLKLOCK_LOCKED   0b0000000010000000
#define REG_OSCCON_CLKLOCK_UNLOCKED 0b0000000000000000

typedef enum {
    OSCCON_CLKLOCK_LOCKED = 0b1, // If (FCKSM0 = 1), then clock and PLL configurations are locked; if (FCKSM0 = 0), then clock and PLL configurations may be modified
    OSCCON_CLKLOCK_UNLOCKED = 0b0 // Clock and PLL selections are not locked, configurations may be modified
} OSCCON_CLKLOCK_e; // Clock Lock Enable bit

#define REG_OSCCON_NOSC_FRCDIVN 0b0000011100000000
#define REG_OSCCON_NOSC_BFRC    0b0000011000000000
#define REG_OSCCON_NOSC_LPRC    0b0000010100000000
#define REG_OSCCON_NOSC_PRIPLL  0b0000001100000000
#define REG_OSCCON_NOSC_PRI     0b0000001000000000
#define REG_OSCCON_NOSC_FRCPLL  0b0000000100000000
#define REG_OSCCON_NOSC_FRC     0b0000000000000000

#define REG_OSCCON_COSC_FRCDIVN 0b0111000000000000
#define REG_OSCCON_COSC_BFRC    0b0110000000000000
#define REG_OSCCON_COSC_LPRC    0b0101000000000000
#define REG_OSCCON_COSC_PRIPLL  0b0011000000000000
#define REG_OSCCON_COSC_PRI     0b0010000000000000
#define REG_OSCCON_COSC_FRCPLL  0b0001000000000000
#define REG_OSCCON_COSC_FRC     0b0000000000000000

typedef enum {
    OSCCON_xOSC_FRC = 0b000, // Fast RC Oscillator, no PLL 
    OSCCON_xOSC_FRCPLL = 0b001, // Fast RC Oscillator with PLL
    OSCCON_xOSC_PRI = 0b010, // Primary Oscillator (EC, XT, HS), no PLL
    OSCCON_xOSC_PRIPLL = 0b011, // Primary Oscillator (EC, XT, HS) with PLL
    OSCCON_xOSC_LPRC = 0b101, // Low Power Oscillator for Idle/Sleep Mode
    OSCCON_xOSC_BFRC = 0b110, // Backup Fast RC Oscillator
    OSCCON_xOSC_FRCDIVN = 0b111 // Fast RC Oscillator with variable Divider
} OSCCON_xOSC_e; // Oscillator Type Selection bits

typedef union {
    struct {
        volatile OSCCON_OSWEN_e OSWEN : 1; // Oscillator Switch Enable bit
        volatile unsigned : 2; // reserved
        volatile OSCCON_CF_e CF : 1; // Clock Fail Detect bit
        volatile unsigned : 1; // reserved
        volatile OSCCON_LOCK_e LOCK : 1; // PLL Lock Status bit (read only)
        volatile unsigned : 1; // reserved
        volatile OSCCON_CLKLOCK_e CLKLOCK : 1; // Clock Lock Enable bit
        volatile OSCCON_xOSC_e NOSC : 3; // New Oscillator Selection bits
        volatile unsigned : 1; // reserved
        volatile OSCCON_xOSC_e COSC : 3; // Current Oscillator Selection bits (read only)

    } __attribute__((packed)) bits;
    volatile uint16_t value;
}OSCCON_t;
    
/* ===========================================================================
 * CLKDIV: CLOCK DIVIDER REGISTER
 * ===========================================================================*/

#define REG_CLKDIV_VALID_DATA_WRITE_MASK    0xFF1F
#define REG_CLKDIV_VALID_DATA_READ_MASK     0xFF1F

#define REG_CLKDIV_PLLPRE_DIV_MASK          0b0000000000011111
#define REG_CLKDIV_PLLPRE_DIVIDER_N1(x)     (x & REG_CLKDIV_PLLPRE_DIV_MASK)

typedef enum {
    CLKDIV_PLLDIV_N1_2 = 0b000000, // PLL Input Clock Divider Setting 1:2
    CLKDIV_PLLDIV_N1_3 = 0b000001, // PLL Input Clock Divider Setting 1:3
    CLKDIV_PLLDIV_N1_4 = 0b000010, // PLL Input Clock Divider Setting 1:4
    CLKDIV_PLLDIV_N1_5 = 0b000011, // PLL Input Clock Divider Setting 1:5
    CLKDIV_PLLDIV_N1_6 = 0b000100, // PLL Input Clock Divider Setting 1:6
    CLKDIV_PLLDIV_N1_7 = 0b000101, // PLL Input Clock Divider Setting 1:7
    CLKDIV_PLLDIV_N1_8 = 0b000110, // PLL Input Clock Divider Setting 1:8
    CLKDIV_PLLDIV_N1_9 = 0b000111, // PLL Input Clock Divider Setting 1:9
    CLKDIV_PLLDIV_N1_10 = 0b001000, // PLL Input Clock Divider Setting 1:10
    CLKDIV_PLLDIV_N1_11 = 0b001001, // PLL Input Clock Divider Setting 1:11
    CLKDIV_PLLDIV_N1_12 = 0b001010, // PLL Input Clock Divider Setting 1:12
    CLKDIV_PLLDIV_N1_13 = 0b001011, // PLL Input Clock Divider Setting 1:13
    CLKDIV_PLLDIV_N1_14 = 0b001100, // PLL Input Clock Divider Setting 1:14
    CLKDIV_PLLDIV_N1_15 = 0b001101, // PLL Input Clock Divider Setting 1:15
    CLKDIV_PLLDIV_N1_16 = 0b001110, // PLL Input Clock Divider Setting 1:16
    CLKDIV_PLLDIV_N1_17 = 0b001111, // PLL Input Clock Divider Setting 1:17
    CLKDIV_PLLDIV_N1_18 = 0b010000, // PLL Input Clock Divider Setting 1:18
    CLKDIV_PLLDIV_N1_19 = 0b010001, // PLL Input Clock Divider Setting 1:19
    CLKDIV_PLLDIV_N1_20 = 0b010010, // PLL Input Clock Divider Setting 1:20
    CLKDIV_PLLDIV_N1_21 = 0b010011, // PLL Input Clock Divider Setting 1:21
    CLKDIV_PLLDIV_N1_22 = 0b010100, // PLL Input Clock Divider Setting 1:22
    CLKDIV_PLLDIV_N1_23 = 0b010101, // PLL Input Clock Divider Setting 1:23
    CLKDIV_PLLDIV_N1_24 = 0b010110, // PLL Input Clock Divider Setting 1:24
    CLKDIV_PLLDIV_N1_25 = 0b010111, // PLL Input Clock Divider Setting 1:25
    CLKDIV_PLLDIV_N1_26 = 0b011000, // PLL Input Clock Divider Setting 1:26
    CLKDIV_PLLDIV_N1_27 = 0b011001, // PLL Input Clock Divider Setting 1:27
    CLKDIV_PLLDIV_N1_28 = 0b011010, // PLL Input Clock Divider Setting 1:28
    CLKDIV_PLLDIV_N1_29 = 0b011011, // PLL Input Clock Divider Setting 1:29
    CLKDIV_PLLDIV_N1_30 = 0b011100, // PLL Input Clock Divider Setting 1:30
    CLKDIV_PLLDIV_N1_31 = 0b011101, // PLL Input Clock Divider Setting 1:31
    CLKDIV_PLLDIV_N1_32 = 0b011110, // PLL Input Clock Divider Setting 1:32
    CLKDIV_PLLDIV_N1_33 = 0b011111, // PLL Input Clock Divider Setting 1:33
} CLKDIV_PLLPRE_e; // PLL Phase Detector Input Divider Select bits (also denoted as ?N1?, PLL prescaler)


#define REG_CLKDIV_DOZE_DIV_1    0b0000000000000000
#define REG_CLKDIV_DOZE_DIV_2    0b0001000000000000
#define REG_CLKDIV_DOZE_DIV_4    0b0010000000000000
#define REG_CLKDIV_DOZE_DIV_8    0b0011000000000000
#define REG_CLKDIV_DOZE_DIV_16   0b0100000000000000
#define REG_CLKDIV_DOZE_DIV_32   0b0101000000000000
#define REG_CLKDIV_DOZE_DIV_64   0b0110000000000000
#define REG_CLKDIV_DOZE_DIV_128  0b0111000000000000

typedef enum {
    CLKDIV_DOZE_DIV_1 = 0b000, // FCY Clock Divider Setting 1:1
    CLKDIV_DOZE_DIV_2 = 0b001, // FCY Clock Divider Setting 1:2
    CLKDIV_DOZE_DIV_4 = 0b010, // FCY Clock Divider Setting 1:4
    CLKDIV_DOZE_DIV_8 = 0b011, // FCY Clock Divider Setting 1:8
    CLKDIV_DOZE_DIV_16 = 0b100, // FCY Clock Divider Setting 1:16
    CLKDIV_DOZE_DIV_32 = 0b101, // FCY Clock Divider Setting 1:32
    CLKDIV_DOZE_DIV_64 = 0b110, // FCY Clock Divider Setting 1:64
    CLKDIV_DOZE_DIV_128 = 0b111 // FCY Clock Divider Setting 1:128
} CLKDIV_DOZE_e; // Processor Clock Reduction Select bits

#define REG_CLKDIV_DOZEN_ENABLED  0b0000100000000000
#define REG_CLKDIV_DOZEN_DISABLED 0b0000000000000000

typedef enum {
    CLKDIV_DOZEN_ENABLED = 0b1, // DOZE<2:0> field specifies the ratio between the peripheral clocks and the processor clocks
    CLKDIV_DOZEN_DISABLED = 0b0 // Processor clock and peripheral clock ratio is forced to 1:1
} CLKDIV_DOZEN_e; // Doze Mode Enable bit

#define REG_CLKDIV_FRCDIVN_256  0b0000011100000000
#define REG_CLKDIV_FRCDIVN_64   0b0000011000000000
#define REG_CLKDIV_FRCDIVN_32   0b0000010100000000
#define REG_CLKDIV_FRCDIVN_16   0b0000010000000000
#define REG_CLKDIV_FRCDIVN_8    0b0000001100000000
#define REG_CLKDIV_FRCDIVN_4    0b0000001000000000
#define REG_CLKDIV_FRCDIVN_2    0b0000000100000000
#define REG_CLKDIV_FRCDIVN_1    0b0000000000000000

typedef enum {
    CLKDIV_FRCDIVN_1 = 0b000, // Fast RC Oscillator Clock Divider Setting 1:1
    CLKDIV_FRCDIVN_2 = 0b001, // Fast RC Oscillator Clock Divider Setting 1:2
    CLKDIV_FRCDIVN_4 = 0b010, // Fast RC Oscillator Clock Divider Setting 1:4
    CLKDIV_FRCDIVN_8 = 0b011, // Fast RC Oscillator Clock Divider Setting 1:8
    CLKDIV_FRCDIVN_16 = 0b100, // Fast RC Oscillator Clock Divider Setting 1:16
    CLKDIV_FRCDIVN_32 = 0b101, // Fast RC Oscillator Clock Divider Setting 1:32
    CLKDIV_FRCDIVN_64 = 0b110, // Fast RC Oscillator Clock Divider Setting 1:64
    CLKDIV_FRCDIVN_256 = 0b111 // Fast RC Oscillator Clock Divider Setting 1:256
} CLKDIV_FRCDIVN_e; // Internal Fast RC Oscillator Post-Scaler bits

#define REG_CLKDIV_ROI_ENABLED  0b1000000000000000
#define REG_CLKDIV_ROI_DISABLED 0b0000000000000000

typedef enum {
    CLKDIV_ROI_ENABLED = 0b1, // Interrupts will clear the DOZEN bit and the processor clock, and the peripheral clock ratio is set to 1:1
    CLKDIV_ROI_DISABLED = 0b0 // Interrupts have no effect on the DOZEN bit
} CLKDIV_ROI_e; // Recover on Interrupt bit

typedef union {
    struct {
        volatile CLKDIV_PLLPRE_e PLLPRE : 6; // PLL Phase Detector Input Divider Select bits (also denoted as ?N1?, PLL prescaler)
        volatile unsigned : 2;
        volatile CLKDIV_FRCDIVN_e FRCDIV : 3; // Internal Fast RC Oscillator Postscaler bits
        volatile CLKDIV_DOZEN_e DOZEN : 1; // Doze Mode Enable bit
        volatile CLKDIV_DOZE_e DOZE : 3; // Processor Clock Reduction Select bits
        volatile CLKDIV_ROI_e ROI : 1; // Recover on Interrupt bit
    } __attribute__((packed)) bits;
    volatile uint16_t value;
} CLKDIV_t;


/* ===========================================================================
 * PLLFBD: PLL FEEDBACK DIVIDER REGISTER
 * ===========================================================================*/

#define REG_PLLFBD_VALID_DATA_WRITE_MASK    0x01FF
#define REG_PLLFBD_VALID_DATA_READ_MASK     0x01FF

#define REG_PLLFBD_PLLFBDIV_M_MASK           0b0000000111111111
#define REG_PLLFBD_MULTIPLIER_M(x)     (x & REG_PLLFBD_PLLFBDIV_M_MASK)

typedef enum {
    PLLFBD_PLLFBDIV_M_2   = 0b000000000, // PLL Input Clock Multiplier Setting x2
    PLLFBD_PLLFBDIV_M_3   = 0b000000001, // PLL Input Clock Multiplier Setting x3
    PLLFBD_PLLFBDIV_M_4   = 0b000000010, // PLL Input Clock Multiplier Setting x4
    PLLFBD_PLLFBDIV_M_5   = 0b000000011, // PLL Input Clock Multiplier Setting x5
    PLLFBD_PLLFBDIV_M_6   = 0b000000100, // PLL Input Clock Multiplier Setting x6
    PLLFBD_PLLFBDIV_M_7   = 0b000000101, // PLL Input Clock Multiplier Setting x7
    PLLFBD_PLLFBDIV_M_8   = 0b000000110, // PLL Input Clock Multiplier Setting x8
    PLLFBD_PLLFBDIV_M_9   = 0b000000111, // PLL Input Clock Multiplier Setting x9
    PLLFBD_PLLFBDIV_M_10  = 0b000001000, // PLL Input Clock Multiplier Setting x10
    PLLFBD_PLLFBDIV_M_11  = 0b000001001, // PLL Input Clock Multiplier Setting x11
    PLLFBD_PLLFBDIV_M_12  = 0b000001010, // PLL Input Clock Multiplier Setting x12
    PLLFBD_PLLFBDIV_M_13  = 0b000001011, // PLL Input Clock Multiplier Setting x13
    PLLFBD_PLLFBDIV_M_14  = 0b000001100, // PLL Input Clock Multiplier Setting x14
    PLLFBD_PLLFBDIV_M_15  = 0b000001101, // PLL Input Clock Multiplier Setting x15
    PLLFBD_PLLFBDIV_M_16  = 0b000001110, // PLL Input Clock Multiplier Setting x16
    PLLFBD_PLLFBDIV_M_17  = 0b000001111, // PLL Input Clock Multiplier Setting x17
    PLLFBD_PLLFBDIV_M_18  = 0b000010000, // PLL Input Clock Multiplier Setting x18
    PLLFBD_PLLFBDIV_M_19  = 0b000010001, // PLL Input Clock Multiplier Setting x19
    PLLFBD_PLLFBDIV_M_20  = 0b000010010, // PLL Input Clock Multiplier Setting x20
    PLLFBD_PLLFBDIV_M_21  = 0b000010011, // PLL Input Clock Multiplier Setting x21
    PLLFBD_PLLFBDIV_M_22  = 0b000010100, // PLL Input Clock Multiplier Setting x22
    PLLFBD_PLLFBDIV_M_23  = 0b000010101, // PLL Input Clock Multiplier Setting x23
    PLLFBD_PLLFBDIV_M_24  = 0b000010110, // PLL Input Clock Multiplier Setting x24
    PLLFBD_PLLFBDIV_M_25  = 0b000010111, // PLL Input Clock Multiplier Setting x25
    PLLFBD_PLLFBDIV_M_26  = 0b000011000, // PLL Input Clock Multiplier Setting x26
    PLLFBD_PLLFBDIV_M_27  = 0b000011001, // PLL Input Clock Multiplier Setting x27
    PLLFBD_PLLFBDIV_M_28  = 0b000011010, // PLL Input Clock Multiplier Setting x28
    PLLFBD_PLLFBDIV_M_29  = 0b000011011, // PLL Input Clock Multiplier Setting x29
    PLLFBD_PLLFBDIV_M_30  = 0b000011100, // PLL Input Clock Multiplier Setting x30
    PLLFBD_PLLFBDIV_M_31  = 0b000011101, // PLL Input Clock Multiplier Setting x31
    PLLFBD_PLLFBDIV_M_32  = 0b000011110, // PLL Input Clock Multiplier Setting x32
    PLLFBD_PLLFBDIV_M_33  = 0b000011111, // PLL Input Clock Multiplier Setting x33
    PLLFBD_PLLFBDIV_M_34  = 0b000100000, // PLL Input Clock Multiplier Setting x34
    PLLFBD_PLLFBDIV_M_35  = 0b000100001, // PLL Input Clock Multiplier Setting x35
    PLLFBD_PLLFBDIV_M_36  = 0b000100010, // PLL Input Clock Multiplier Setting x36
    PLLFBD_PLLFBDIV_M_37  = 0b000100011, // PLL Input Clock Multiplier Setting x37
    PLLFBD_PLLFBDIV_M_38  = 0b000100100, // PLL Input Clock Multiplier Setting x38
    PLLFBD_PLLFBDIV_M_39  = 0b000100101, // PLL Input Clock Multiplier Setting x39
    PLLFBD_PLLFBDIV_M_40  = 0b000100110, // PLL Input Clock Multiplier Setting x40
    PLLFBD_PLLFBDIV_M_41  = 0b000100111, // PLL Input Clock Multiplier Setting x41
    PLLFBD_PLLFBDIV_M_42  = 0b000101000, // PLL Input Clock Multiplier Setting x42
    PLLFBD_PLLFBDIV_M_43  = 0b000101001, // PLL Input Clock Multiplier Setting x43
    PLLFBD_PLLFBDIV_M_44  = 0b000101010, // PLL Input Clock Multiplier Setting x44
    PLLFBD_PLLFBDIV_M_45  = 0b000101011, // PLL Input Clock Multiplier Setting x45
    PLLFBD_PLLFBDIV_M_46  = 0b000101100, // PLL Input Clock Multiplier Setting x46
    PLLFBD_PLLFBDIV_M_47  = 0b000101101, // PLL Input Clock Multiplier Setting x47
    PLLFBD_PLLFBDIV_M_48  = 0b000101110, // PLL Input Clock Multiplier Setting x48
    PLLFBD_PLLFBDIV_M_49  = 0b000101111, // PLL Input Clock Multiplier Setting x49
    PLLFBD_PLLFBDIV_M_50  = 0b000110000, // PLL Input Clock Multiplier Setting x50
    PLLFBD_PLLFBDIV_M_51  = 0b000110001, // PLL Input Clock Multiplier Setting x51
    PLLFBD_PLLFBDIV_M_52  = 0b000110010, // PLL Input Clock Multiplier Setting x52
    PLLFBD_PLLFBDIV_M_53  = 0b000110011, // PLL Input Clock Multiplier Setting x53
    PLLFBD_PLLFBDIV_M_54  = 0b000110100, // PLL Input Clock Multiplier Setting x54
    PLLFBD_PLLFBDIV_M_55  = 0b000110101, // PLL Input Clock Multiplier Setting x55
    PLLFBD_PLLFBDIV_M_56  = 0b000110110, // PLL Input Clock Multiplier Setting x56
    PLLFBD_PLLFBDIV_M_57  = 0b000110111, // PLL Input Clock Multiplier Setting x57
    PLLFBD_PLLFBDIV_M_58  = 0b000111000, // PLL Input Clock Multiplier Setting x58
    PLLFBD_PLLFBDIV_M_59  = 0b000111001, // PLL Input Clock Multiplier Setting x59
    PLLFBD_PLLFBDIV_M_60  = 0b000111010, // PLL Input Clock Multiplier Setting x60
    PLLFBD_PLLFBDIV_M_61  = 0b000111011, // PLL Input Clock Multiplier Setting x61
    PLLFBD_PLLFBDIV_M_62  = 0b000111100, // PLL Input Clock Multiplier Setting x62
    PLLFBD_PLLFBDIV_M_63  = 0b000111101, // PLL Input Clock Multiplier Setting x63
    PLLFBD_PLLFBDIV_M_64  = 0b000111110, // PLL Input Clock Multiplier Setting x64
    PLLFBD_PLLFBDIV_M_65  = 0b000111111, // PLL Input Clock Multiplier Setting x65
    PLLFBD_PLLFBDIV_M_66  = 0b001000000, // PLL Input Clock Multiplier Setting x66
    PLLFBD_PLLFBDIV_M_67  = 0b001000001, // PLL Input Clock Multiplier Setting x67
    PLLFBD_PLLFBDIV_M_68  = 0b001000010, // PLL Input Clock Multiplier Setting x68
    PLLFBD_PLLFBDIV_M_69  = 0b001000011, // PLL Input Clock Multiplier Setting x69
    PLLFBD_PLLFBDIV_M_70  = 0b001000100, // PLL Input Clock Multiplier Setting x70
    PLLFBD_PLLFBDIV_M_71  = 0b001000101, // PLL Input Clock Multiplier Setting x71
    PLLFBD_PLLFBDIV_M_72  = 0b001000110, // PLL Input Clock Multiplier Setting x72
    PLLFBD_PLLFBDIV_M_73  = 0b001000111, // PLL Input Clock Multiplier Setting x73
    PLLFBD_PLLFBDIV_M_74  = 0b001001000, // PLL Input Clock Multiplier Setting x74
    PLLFBD_PLLFBDIV_M_75  = 0b001001001, // PLL Input Clock Multiplier Setting x75
    PLLFBD_PLLFBDIV_M_76  = 0b001001010, // PLL Input Clock Multiplier Setting x76
    PLLFBD_PLLFBDIV_M_77  = 0b001001011, // PLL Input Clock Multiplier Setting x77
    PLLFBD_PLLFBDIV_M_78  = 0b001001100, // PLL Input Clock Multiplier Setting x78
    PLLFBD_PLLFBDIV_M_79  = 0b001001101, // PLL Input Clock Multiplier Setting x79
    PLLFBD_PLLFBDIV_M_80  = 0b001001110, // PLL Input Clock Multiplier Setting x80
    PLLFBD_PLLFBDIV_M_81  = 0b001001111, // PLL Input Clock Multiplier Setting x81
    PLLFBD_PLLFBDIV_M_82  = 0b001010000, // PLL Input Clock Multiplier Setting x82
    PLLFBD_PLLFBDIV_M_83  = 0b001010001, // PLL Input Clock Multiplier Setting x83
    PLLFBD_PLLFBDIV_M_84  = 0b001010010, // PLL Input Clock Multiplier Setting x84
    PLLFBD_PLLFBDIV_M_85  = 0b001010011, // PLL Input Clock Multiplier Setting x85
    PLLFBD_PLLFBDIV_M_86  = 0b001010100, // PLL Input Clock Multiplier Setting x86
    PLLFBD_PLLFBDIV_M_87  = 0b001010101, // PLL Input Clock Multiplier Setting x87
    PLLFBD_PLLFBDIV_M_88  = 0b001010110, // PLL Input Clock Multiplier Setting x88
    PLLFBD_PLLFBDIV_M_89  = 0b001010111, // PLL Input Clock Multiplier Setting x89
    PLLFBD_PLLFBDIV_M_90  = 0b001011000, // PLL Input Clock Multiplier Setting x90
    PLLFBD_PLLFBDIV_M_91  = 0b001011001, // PLL Input Clock Multiplier Setting x91
    PLLFBD_PLLFBDIV_M_92  = 0b001011010, // PLL Input Clock Multiplier Setting x92
    PLLFBD_PLLFBDIV_M_93  = 0b001011011, // PLL Input Clock Multiplier Setting x93
    PLLFBD_PLLFBDIV_M_94  = 0b001011100, // PLL Input Clock Multiplier Setting x94
    PLLFBD_PLLFBDIV_M_95  = 0b001011101, // PLL Input Clock Multiplier Setting x95
    PLLFBD_PLLFBDIV_M_96  = 0b001011110, // PLL Input Clock Multiplier Setting x96
    PLLFBD_PLLFBDIV_M_97  = 0b001011111, // PLL Input Clock Multiplier Setting x97
    PLLFBD_PLLFBDIV_M_98  = 0b001100000, // PLL Input Clock Multiplier Setting x98
    PLLFBD_PLLFBDIV_M_99  = 0b001100001, // PLL Input Clock Multiplier Setting x99
    PLLFBD_PLLFBDIV_M_100 = 0b001100010, // PLL Input Clock Multiplier Setting x100
    PLLFBD_PLLFBDIV_M_101 = 0b001100011, // PLL Input Clock Multiplier Setting x101
    PLLFBD_PLLFBDIV_M_102 = 0b001100100, // PLL Input Clock Multiplier Setting x102
    PLLFBD_PLLFBDIV_M_103 = 0b001100101, // PLL Input Clock Multiplier Setting x103
    PLLFBD_PLLFBDIV_M_104 = 0b001100110, // PLL Input Clock Multiplier Setting x104
    PLLFBD_PLLFBDIV_M_105 = 0b001100111, // PLL Input Clock Multiplier Setting x105
    PLLFBD_PLLFBDIV_M_106 = 0b001101000, // PLL Input Clock Multiplier Setting x106
    PLLFBD_PLLFBDIV_M_107 = 0b001101001, // PLL Input Clock Multiplier Setting x107
    PLLFBD_PLLFBDIV_M_108 = 0b001101010, // PLL Input Clock Multiplier Setting x108
    PLLFBD_PLLFBDIV_M_109 = 0b001101011, // PLL Input Clock Multiplier Setting x109
    PLLFBD_PLLFBDIV_M_110 = 0b001101100, // PLL Input Clock Multiplier Setting x110
    PLLFBD_PLLFBDIV_M_111 = 0b001101101, // PLL Input Clock Multiplier Setting x111
    PLLFBD_PLLFBDIV_M_112 = 0b001101110, // PLL Input Clock Multiplier Setting x112
    PLLFBD_PLLFBDIV_M_113 = 0b001101111, // PLL Input Clock Multiplier Setting x113
    PLLFBD_PLLFBDIV_M_114 = 0b001110000, // PLL Input Clock Multiplier Setting x114
    PLLFBD_PLLFBDIV_M_115 = 0b001110001, // PLL Input Clock Multiplier Setting x115
    PLLFBD_PLLFBDIV_M_116 = 0b001110010, // PLL Input Clock Multiplier Setting x116
    PLLFBD_PLLFBDIV_M_117 = 0b001110011, // PLL Input Clock Multiplier Setting x117
    PLLFBD_PLLFBDIV_M_118 = 0b001110100, // PLL Input Clock Multiplier Setting x118
    PLLFBD_PLLFBDIV_M_119 = 0b001110101, // PLL Input Clock Multiplier Setting x119
    PLLFBD_PLLFBDIV_M_120 = 0b001110110, // PLL Input Clock Multiplier Setting x120
    PLLFBD_PLLFBDIV_M_121 = 0b001110111, // PLL Input Clock Multiplier Setting x121
    PLLFBD_PLLFBDIV_M_122 = 0b001111000, // PLL Input Clock Multiplier Setting x122
    PLLFBD_PLLFBDIV_M_123 = 0b001111001, // PLL Input Clock Multiplier Setting x123
    PLLFBD_PLLFBDIV_M_124 = 0b001111010, // PLL Input Clock Multiplier Setting x124
    PLLFBD_PLLFBDIV_M_125 = 0b001111011, // PLL Input Clock Multiplier Setting x125
    PLLFBD_PLLFBDIV_M_126 = 0b001111100, // PLL Input Clock Multiplier Setting x126
    PLLFBD_PLLFBDIV_M_127 = 0b001111101, // PLL Input Clock Multiplier Setting x127
    PLLFBD_PLLFBDIV_M_128 = 0b001111110, // PLL Input Clock Multiplier Setting x128
    PLLFBD_PLLFBDIV_M_129 = 0b001111111, // PLL Input Clock Multiplier Setting x129
    PLLFBD_PLLFBDIV_M_130 = 0b010000000, // PLL Input Clock Multiplier Setting x130
    PLLFBD_PLLFBDIV_M_131 = 0b010000001, // PLL Input Clock Multiplier Setting x131
    PLLFBD_PLLFBDIV_M_132 = 0b010000010, // PLL Input Clock Multiplier Setting x132
    PLLFBD_PLLFBDIV_M_133 = 0b010000011, // PLL Input Clock Multiplier Setting x133
    PLLFBD_PLLFBDIV_M_134 = 0b010000100, // PLL Input Clock Multiplier Setting x134
    PLLFBD_PLLFBDIV_M_135 = 0b010000101, // PLL Input Clock Multiplier Setting x135
    PLLFBD_PLLFBDIV_M_136 = 0b010000110, // PLL Input Clock Multiplier Setting x136
    PLLFBD_PLLFBDIV_M_137 = 0b010000111, // PLL Input Clock Multiplier Setting x137
    PLLFBD_PLLFBDIV_M_138 = 0b010001000, // PLL Input Clock Multiplier Setting x138
    PLLFBD_PLLFBDIV_M_139 = 0b010001001, // PLL Input Clock Multiplier Setting x139
    PLLFBD_PLLFBDIV_M_140 = 0b010001010, // PLL Input Clock Multiplier Setting x140
    PLLFBD_PLLFBDIV_M_141 = 0b010001011, // PLL Input Clock Multiplier Setting x141
    PLLFBD_PLLFBDIV_M_142 = 0b010001100, // PLL Input Clock Multiplier Setting x142
    PLLFBD_PLLFBDIV_M_143 = 0b010001101, // PLL Input Clock Multiplier Setting x143
    PLLFBD_PLLFBDIV_M_144 = 0b010001110, // PLL Input Clock Multiplier Setting x144
    PLLFBD_PLLFBDIV_M_145 = 0b010001111, // PLL Input Clock Multiplier Setting x145
    PLLFBD_PLLFBDIV_M_146 = 0b010010000, // PLL Input Clock Multiplier Setting x146
    PLLFBD_PLLFBDIV_M_147 = 0b010010001, // PLL Input Clock Multiplier Setting x147
    PLLFBD_PLLFBDIV_M_148 = 0b010010010, // PLL Input Clock Multiplier Setting x148
    PLLFBD_PLLFBDIV_M_149 = 0b010010011, // PLL Input Clock Multiplier Setting x149
    PLLFBD_PLLFBDIV_M_150 = 0b010010100, // PLL Input Clock Multiplier Setting x150
    PLLFBD_PLLFBDIV_M_151 = 0b010010101, // PLL Input Clock Multiplier Setting x151
    PLLFBD_PLLFBDIV_M_152 = 0b010010110, // PLL Input Clock Multiplier Setting x152
    PLLFBD_PLLFBDIV_M_153 = 0b010010111, // PLL Input Clock Multiplier Setting x153
    PLLFBD_PLLFBDIV_M_154 = 0b010011000, // PLL Input Clock Multiplier Setting x154
    PLLFBD_PLLFBDIV_M_155 = 0b010011001, // PLL Input Clock Multiplier Setting x155
    PLLFBD_PLLFBDIV_M_156 = 0b010011010, // PLL Input Clock Multiplier Setting x156
    PLLFBD_PLLFBDIV_M_157 = 0b010011011, // PLL Input Clock Multiplier Setting x157
    PLLFBD_PLLFBDIV_M_158 = 0b010011100, // PLL Input Clock Multiplier Setting x158
    PLLFBD_PLLFBDIV_M_159 = 0b010011101, // PLL Input Clock Multiplier Setting x159
    PLLFBD_PLLFBDIV_M_160 = 0b010011110, // PLL Input Clock Multiplier Setting x160
    PLLFBD_PLLFBDIV_M_161 = 0b010011111, // PLL Input Clock Multiplier Setting x161
    PLLFBD_PLLFBDIV_M_162 = 0b010100000, // PLL Input Clock Multiplier Setting x162
    PLLFBD_PLLFBDIV_M_163 = 0b010100001, // PLL Input Clock Multiplier Setting x163
    PLLFBD_PLLFBDIV_M_164 = 0b010100010, // PLL Input Clock Multiplier Setting x164
    PLLFBD_PLLFBDIV_M_165 = 0b010100011, // PLL Input Clock Multiplier Setting x165
    PLLFBD_PLLFBDIV_M_166 = 0b010100100, // PLL Input Clock Multiplier Setting x166
    PLLFBD_PLLFBDIV_M_167 = 0b010100101, // PLL Input Clock Multiplier Setting x167
    PLLFBD_PLLFBDIV_M_168 = 0b010100110, // PLL Input Clock Multiplier Setting x168
    PLLFBD_PLLFBDIV_M_169 = 0b010100111, // PLL Input Clock Multiplier Setting x169
    PLLFBD_PLLFBDIV_M_170 = 0b010101000, // PLL Input Clock Multiplier Setting x170
    PLLFBD_PLLFBDIV_M_171 = 0b010101001, // PLL Input Clock Multiplier Setting x171
    PLLFBD_PLLFBDIV_M_172 = 0b010101010, // PLL Input Clock Multiplier Setting x172
    PLLFBD_PLLFBDIV_M_173 = 0b010101011, // PLL Input Clock Multiplier Setting x173
    PLLFBD_PLLFBDIV_M_174 = 0b010101100, // PLL Input Clock Multiplier Setting x174
    PLLFBD_PLLFBDIV_M_175 = 0b010101101, // PLL Input Clock Multiplier Setting x175
    PLLFBD_PLLFBDIV_M_176 = 0b010101110, // PLL Input Clock Multiplier Setting x176
    PLLFBD_PLLFBDIV_M_177 = 0b010101111, // PLL Input Clock Multiplier Setting x177
    PLLFBD_PLLFBDIV_M_178 = 0b010110000, // PLL Input Clock Multiplier Setting x178
    PLLFBD_PLLFBDIV_M_179 = 0b010110001, // PLL Input Clock Multiplier Setting x179
    PLLFBD_PLLFBDIV_M_180 = 0b010110010, // PLL Input Clock Multiplier Setting x180
    PLLFBD_PLLFBDIV_M_181 = 0b010110011, // PLL Input Clock Multiplier Setting x181
    PLLFBD_PLLFBDIV_M_182 = 0b010110100, // PLL Input Clock Multiplier Setting x182
    PLLFBD_PLLFBDIV_M_183 = 0b010110101, // PLL Input Clock Multiplier Setting x183
    PLLFBD_PLLFBDIV_M_184 = 0b010110110, // PLL Input Clock Multiplier Setting x184
    PLLFBD_PLLFBDIV_M_185 = 0b010110111, // PLL Input Clock Multiplier Setting x185
    PLLFBD_PLLFBDIV_M_186 = 0b010111000, // PLL Input Clock Multiplier Setting x186
    PLLFBD_PLLFBDIV_M_187 = 0b010111001, // PLL Input Clock Multiplier Setting x187
    PLLFBD_PLLFBDIV_M_188 = 0b010111010, // PLL Input Clock Multiplier Setting x188
    PLLFBD_PLLFBDIV_M_189 = 0b010111011, // PLL Input Clock Multiplier Setting x189
    PLLFBD_PLLFBDIV_M_190 = 0b010111100, // PLL Input Clock Multiplier Setting x190
    PLLFBD_PLLFBDIV_M_191 = 0b010111101, // PLL Input Clock Multiplier Setting x191
    PLLFBD_PLLFBDIV_M_192 = 0b010111110, // PLL Input Clock Multiplier Setting x192
    PLLFBD_PLLFBDIV_M_193 = 0b010111111, // PLL Input Clock Multiplier Setting x193
    PLLFBD_PLLFBDIV_M_194 = 0b011000000, // PLL Input Clock Multiplier Setting x194
    PLLFBD_PLLFBDIV_M_195 = 0b011000001, // PLL Input Clock Multiplier Setting x195
    PLLFBD_PLLFBDIV_M_196 = 0b011000010, // PLL Input Clock Multiplier Setting x196
    PLLFBD_PLLFBDIV_M_197 = 0b011000011, // PLL Input Clock Multiplier Setting x197
    PLLFBD_PLLFBDIV_M_198 = 0b011000100, // PLL Input Clock Multiplier Setting x198
    PLLFBD_PLLFBDIV_M_199 = 0b011000101, // PLL Input Clock Multiplier Setting x199
    PLLFBD_PLLFBDIV_M_200 = 0b011000110, // PLL Input Clock Multiplier Setting x200
    PLLFBD_PLLFBDIV_M_201 = 0b011000111, // PLL Input Clock Multiplier Setting x201
    PLLFBD_PLLFBDIV_M_202 = 0b011001000, // PLL Input Clock Multiplier Setting x202
    PLLFBD_PLLFBDIV_M_203 = 0b011001001, // PLL Input Clock Multiplier Setting x203
    PLLFBD_PLLFBDIV_M_204 = 0b011001010, // PLL Input Clock Multiplier Setting x204
    PLLFBD_PLLFBDIV_M_205 = 0b011001011, // PLL Input Clock Multiplier Setting x205
    PLLFBD_PLLFBDIV_M_206 = 0b011001100, // PLL Input Clock Multiplier Setting x206
    PLLFBD_PLLFBDIV_M_207 = 0b011001101, // PLL Input Clock Multiplier Setting x207
    PLLFBD_PLLFBDIV_M_208 = 0b011001110, // PLL Input Clock Multiplier Setting x208
    PLLFBD_PLLFBDIV_M_209 = 0b011001111, // PLL Input Clock Multiplier Setting x209
    PLLFBD_PLLFBDIV_M_210 = 0b011010000, // PLL Input Clock Multiplier Setting x210
    PLLFBD_PLLFBDIV_M_211 = 0b011010001, // PLL Input Clock Multiplier Setting x211
    PLLFBD_PLLFBDIV_M_212 = 0b011010010, // PLL Input Clock Multiplier Setting x212
    PLLFBD_PLLFBDIV_M_213 = 0b011010011, // PLL Input Clock Multiplier Setting x213
    PLLFBD_PLLFBDIV_M_214 = 0b011010100, // PLL Input Clock Multiplier Setting x214
    PLLFBD_PLLFBDIV_M_215 = 0b011010101, // PLL Input Clock Multiplier Setting x215
    PLLFBD_PLLFBDIV_M_216 = 0b011010110, // PLL Input Clock Multiplier Setting x216
    PLLFBD_PLLFBDIV_M_217 = 0b011010111, // PLL Input Clock Multiplier Setting x217
    PLLFBD_PLLFBDIV_M_218 = 0b011011000, // PLL Input Clock Multiplier Setting x218
    PLLFBD_PLLFBDIV_M_219 = 0b011011001, // PLL Input Clock Multiplier Setting x219
    PLLFBD_PLLFBDIV_M_220 = 0b011011010, // PLL Input Clock Multiplier Setting x220
    PLLFBD_PLLFBDIV_M_221 = 0b011011011, // PLL Input Clock Multiplier Setting x221
    PLLFBD_PLLFBDIV_M_222 = 0b011011100, // PLL Input Clock Multiplier Setting x222
    PLLFBD_PLLFBDIV_M_223 = 0b011011101, // PLL Input Clock Multiplier Setting x223
    PLLFBD_PLLFBDIV_M_224 = 0b011011110, // PLL Input Clock Multiplier Setting x224
    PLLFBD_PLLFBDIV_M_225 = 0b011011111, // PLL Input Clock Multiplier Setting x225
    PLLFBD_PLLFBDIV_M_226 = 0b011100000, // PLL Input Clock Multiplier Setting x226
    PLLFBD_PLLFBDIV_M_227 = 0b011100001, // PLL Input Clock Multiplier Setting x227
    PLLFBD_PLLFBDIV_M_228 = 0b011100010, // PLL Input Clock Multiplier Setting x228
    PLLFBD_PLLFBDIV_M_229 = 0b011100011, // PLL Input Clock Multiplier Setting x229
    PLLFBD_PLLFBDIV_M_230 = 0b011100100, // PLL Input Clock Multiplier Setting x230
    PLLFBD_PLLFBDIV_M_231 = 0b011100101, // PLL Input Clock Multiplier Setting x231
    PLLFBD_PLLFBDIV_M_232 = 0b011100110, // PLL Input Clock Multiplier Setting x232
    PLLFBD_PLLFBDIV_M_233 = 0b011100111, // PLL Input Clock Multiplier Setting x233
    PLLFBD_PLLFBDIV_M_234 = 0b011101000, // PLL Input Clock Multiplier Setting x234
    PLLFBD_PLLFBDIV_M_235 = 0b011101001, // PLL Input Clock Multiplier Setting x235
    PLLFBD_PLLFBDIV_M_236 = 0b011101010, // PLL Input Clock Multiplier Setting x236
    PLLFBD_PLLFBDIV_M_237 = 0b011101011, // PLL Input Clock Multiplier Setting x237
    PLLFBD_PLLFBDIV_M_238 = 0b011101100, // PLL Input Clock Multiplier Setting x238
    PLLFBD_PLLFBDIV_M_239 = 0b011101101, // PLL Input Clock Multiplier Setting x239
    PLLFBD_PLLFBDIV_M_240 = 0b011101110, // PLL Input Clock Multiplier Setting x240
    PLLFBD_PLLFBDIV_M_241 = 0b011101111, // PLL Input Clock Multiplier Setting x241
    PLLFBD_PLLFBDIV_M_242 = 0b011110000, // PLL Input Clock Multiplier Setting x242
    PLLFBD_PLLFBDIV_M_243 = 0b011110001, // PLL Input Clock Multiplier Setting x243
    PLLFBD_PLLFBDIV_M_244 = 0b011110010, // PLL Input Clock Multiplier Setting x244
    PLLFBD_PLLFBDIV_M_245 = 0b011110011, // PLL Input Clock Multiplier Setting x245
    PLLFBD_PLLFBDIV_M_246 = 0b011110100, // PLL Input Clock Multiplier Setting x246
    PLLFBD_PLLFBDIV_M_247 = 0b011110101, // PLL Input Clock Multiplier Setting x247
    PLLFBD_PLLFBDIV_M_248 = 0b011110110, // PLL Input Clock Multiplier Setting x248
    PLLFBD_PLLFBDIV_M_249 = 0b011110111, // PLL Input Clock Multiplier Setting x249
    PLLFBD_PLLFBDIV_M_250 = 0b011111000, // PLL Input Clock Multiplier Setting x250
    PLLFBD_PLLFBDIV_M_251 = 0b011111001, // PLL Input Clock Multiplier Setting x251
    PLLFBD_PLLFBDIV_M_252 = 0b011111010, // PLL Input Clock Multiplier Setting x252
    PLLFBD_PLLFBDIV_M_253 = 0b011111011, // PLL Input Clock Multiplier Setting x253
    PLLFBD_PLLFBDIV_M_254 = 0b011111100, // PLL Input Clock Multiplier Setting x254
    PLLFBD_PLLFBDIV_M_255 = 0b011111101, // PLL Input Clock Multiplier Setting x255
    PLLFBD_PLLFBDIV_M_256 = 0b011111110, // PLL Input Clock Multiplier Setting x256
    PLLFBD_PLLFBDIV_M_257 = 0b011111111, // PLL Input Clock Multiplier Setting x257
    PLLFBD_PLLFBDIV_M_258 = 0b100000000, // PLL Input Clock Multiplier Setting x258
    PLLFBD_PLLFBDIV_M_259 = 0b100000001, // PLL Input Clock Multiplier Setting x259
    PLLFBD_PLLFBDIV_M_260 = 0b100000010, // PLL Input Clock Multiplier Setting x260
    PLLFBD_PLLFBDIV_M_261 = 0b100000011, // PLL Input Clock Multiplier Setting x261
    PLLFBD_PLLFBDIV_M_262 = 0b100000100, // PLL Input Clock Multiplier Setting x262
    PLLFBD_PLLFBDIV_M_263 = 0b100000101, // PLL Input Clock Multiplier Setting x263
    PLLFBD_PLLFBDIV_M_264 = 0b100000110, // PLL Input Clock Multiplier Setting x264
    PLLFBD_PLLFBDIV_M_265 = 0b100000111, // PLL Input Clock Multiplier Setting x265
    PLLFBD_PLLFBDIV_M_266 = 0b100001000, // PLL Input Clock Multiplier Setting x266
    PLLFBD_PLLFBDIV_M_267 = 0b100001001, // PLL Input Clock Multiplier Setting x267
    PLLFBD_PLLFBDIV_M_268 = 0b100001010, // PLL Input Clock Multiplier Setting x268
    PLLFBD_PLLFBDIV_M_269 = 0b100001011, // PLL Input Clock Multiplier Setting x269
    PLLFBD_PLLFBDIV_M_270 = 0b100001100, // PLL Input Clock Multiplier Setting x270
    PLLFBD_PLLFBDIV_M_271 = 0b100001101, // PLL Input Clock Multiplier Setting x271
    PLLFBD_PLLFBDIV_M_272 = 0b100001110, // PLL Input Clock Multiplier Setting x272
    PLLFBD_PLLFBDIV_M_273 = 0b100001111, // PLL Input Clock Multiplier Setting x273
    PLLFBD_PLLFBDIV_M_274 = 0b100010000, // PLL Input Clock Multiplier Setting x274
    PLLFBD_PLLFBDIV_M_275 = 0b100010001, // PLL Input Clock Multiplier Setting x275
    PLLFBD_PLLFBDIV_M_276 = 0b100010010, // PLL Input Clock Multiplier Setting x276
    PLLFBD_PLLFBDIV_M_277 = 0b100010011, // PLL Input Clock Multiplier Setting x277
    PLLFBD_PLLFBDIV_M_278 = 0b100010100, // PLL Input Clock Multiplier Setting x278
    PLLFBD_PLLFBDIV_M_279 = 0b100010101, // PLL Input Clock Multiplier Setting x279
    PLLFBD_PLLFBDIV_M_280 = 0b100010110, // PLL Input Clock Multiplier Setting x280
    PLLFBD_PLLFBDIV_M_281 = 0b100010111, // PLL Input Clock Multiplier Setting x281
    PLLFBD_PLLFBDIV_M_282 = 0b100011000, // PLL Input Clock Multiplier Setting x282
    PLLFBD_PLLFBDIV_M_283 = 0b100011001, // PLL Input Clock Multiplier Setting x283
    PLLFBD_PLLFBDIV_M_284 = 0b100011010, // PLL Input Clock Multiplier Setting x284
    PLLFBD_PLLFBDIV_M_285 = 0b100011011, // PLL Input Clock Multiplier Setting x285
    PLLFBD_PLLFBDIV_M_286 = 0b100011100, // PLL Input Clock Multiplier Setting x286
    PLLFBD_PLLFBDIV_M_287 = 0b100011101, // PLL Input Clock Multiplier Setting x287
    PLLFBD_PLLFBDIV_M_288 = 0b100011110, // PLL Input Clock Multiplier Setting x288
    PLLFBD_PLLFBDIV_M_289 = 0b100011111, // PLL Input Clock Multiplier Setting x289
    PLLFBD_PLLFBDIV_M_290 = 0b100100000, // PLL Input Clock Multiplier Setting x290
    PLLFBD_PLLFBDIV_M_291 = 0b100100001, // PLL Input Clock Multiplier Setting x291
    PLLFBD_PLLFBDIV_M_292 = 0b100100010, // PLL Input Clock Multiplier Setting x292
    PLLFBD_PLLFBDIV_M_293 = 0b100100011, // PLL Input Clock Multiplier Setting x293
    PLLFBD_PLLFBDIV_M_294 = 0b100100100, // PLL Input Clock Multiplier Setting x294
    PLLFBD_PLLFBDIV_M_295 = 0b100100101, // PLL Input Clock Multiplier Setting x295
    PLLFBD_PLLFBDIV_M_296 = 0b100100110, // PLL Input Clock Multiplier Setting x296
    PLLFBD_PLLFBDIV_M_297 = 0b100100111, // PLL Input Clock Multiplier Setting x297
    PLLFBD_PLLFBDIV_M_298 = 0b100101000, // PLL Input Clock Multiplier Setting x298
    PLLFBD_PLLFBDIV_M_299 = 0b100101001, // PLL Input Clock Multiplier Setting x299
    PLLFBD_PLLFBDIV_M_300 = 0b100101010, // PLL Input Clock Multiplier Setting x300
    PLLFBD_PLLFBDIV_M_301 = 0b100101011, // PLL Input Clock Multiplier Setting x301
    PLLFBD_PLLFBDIV_M_302 = 0b100101100, // PLL Input Clock Multiplier Setting x302
    PLLFBD_PLLFBDIV_M_303 = 0b100101101, // PLL Input Clock Multiplier Setting x303
    PLLFBD_PLLFBDIV_M_304 = 0b100101110, // PLL Input Clock Multiplier Setting x304
    PLLFBD_PLLFBDIV_M_305 = 0b100101111, // PLL Input Clock Multiplier Setting x305
    PLLFBD_PLLFBDIV_M_306 = 0b100110000, // PLL Input Clock Multiplier Setting x306
    PLLFBD_PLLFBDIV_M_307 = 0b100110001, // PLL Input Clock Multiplier Setting x307
    PLLFBD_PLLFBDIV_M_308 = 0b100110010, // PLL Input Clock Multiplier Setting x308
    PLLFBD_PLLFBDIV_M_309 = 0b100110011, // PLL Input Clock Multiplier Setting x309
    PLLFBD_PLLFBDIV_M_310 = 0b100110100, // PLL Input Clock Multiplier Setting x310
    PLLFBD_PLLFBDIV_M_311 = 0b100110101, // PLL Input Clock Multiplier Setting x311
    PLLFBD_PLLFBDIV_M_312 = 0b100110110, // PLL Input Clock Multiplier Setting x312
    PLLFBD_PLLFBDIV_M_313 = 0b100110111, // PLL Input Clock Multiplier Setting x313
    PLLFBD_PLLFBDIV_M_314 = 0b100111000, // PLL Input Clock Multiplier Setting x314
    PLLFBD_PLLFBDIV_M_315 = 0b100111001, // PLL Input Clock Multiplier Setting x315
    PLLFBD_PLLFBDIV_M_316 = 0b100111010, // PLL Input Clock Multiplier Setting x316
    PLLFBD_PLLFBDIV_M_317 = 0b100111011, // PLL Input Clock Multiplier Setting x317
    PLLFBD_PLLFBDIV_M_318 = 0b100111100, // PLL Input Clock Multiplier Setting x318
    PLLFBD_PLLFBDIV_M_319 = 0b100111101, // PLL Input Clock Multiplier Setting x319
    PLLFBD_PLLFBDIV_M_320 = 0b100111110, // PLL Input Clock Multiplier Setting x320
    PLLFBD_PLLFBDIV_M_321 = 0b100111111, // PLL Input Clock Multiplier Setting x321
    PLLFBD_PLLFBDIV_M_322 = 0b101000000, // PLL Input Clock Multiplier Setting x322
    PLLFBD_PLLFBDIV_M_323 = 0b101000001, // PLL Input Clock Multiplier Setting x323
    PLLFBD_PLLFBDIV_M_324 = 0b101000010, // PLL Input Clock Multiplier Setting x324
    PLLFBD_PLLFBDIV_M_325 = 0b101000011, // PLL Input Clock Multiplier Setting x325
    PLLFBD_PLLFBDIV_M_326 = 0b101000100, // PLL Input Clock Multiplier Setting x326
    PLLFBD_PLLFBDIV_M_327 = 0b101000101, // PLL Input Clock Multiplier Setting x327
    PLLFBD_PLLFBDIV_M_328 = 0b101000110, // PLL Input Clock Multiplier Setting x328
    PLLFBD_PLLFBDIV_M_329 = 0b101000111, // PLL Input Clock Multiplier Setting x329
    PLLFBD_PLLFBDIV_M_330 = 0b101001000, // PLL Input Clock Multiplier Setting x330
    PLLFBD_PLLFBDIV_M_331 = 0b101001001, // PLL Input Clock Multiplier Setting x331
    PLLFBD_PLLFBDIV_M_332 = 0b101001010, // PLL Input Clock Multiplier Setting x332
    PLLFBD_PLLFBDIV_M_333 = 0b101001011, // PLL Input Clock Multiplier Setting x333
    PLLFBD_PLLFBDIV_M_334 = 0b101001100, // PLL Input Clock Multiplier Setting x334
    PLLFBD_PLLFBDIV_M_335 = 0b101001101, // PLL Input Clock Multiplier Setting x335
    PLLFBD_PLLFBDIV_M_336 = 0b101001110, // PLL Input Clock Multiplier Setting x336
    PLLFBD_PLLFBDIV_M_337 = 0b101001111, // PLL Input Clock Multiplier Setting x337
    PLLFBD_PLLFBDIV_M_338 = 0b101010000, // PLL Input Clock Multiplier Setting x338
    PLLFBD_PLLFBDIV_M_339 = 0b101010001, // PLL Input Clock Multiplier Setting x339
    PLLFBD_PLLFBDIV_M_340 = 0b101010010, // PLL Input Clock Multiplier Setting x340
    PLLFBD_PLLFBDIV_M_341 = 0b101010011, // PLL Input Clock Multiplier Setting x341
    PLLFBD_PLLFBDIV_M_342 = 0b101010100, // PLL Input Clock Multiplier Setting x342
    PLLFBD_PLLFBDIV_M_343 = 0b101010101, // PLL Input Clock Multiplier Setting x343
    PLLFBD_PLLFBDIV_M_344 = 0b101010110, // PLL Input Clock Multiplier Setting x344
    PLLFBD_PLLFBDIV_M_345 = 0b101010111, // PLL Input Clock Multiplier Setting x345
    PLLFBD_PLLFBDIV_M_346 = 0b101011000, // PLL Input Clock Multiplier Setting x346
    PLLFBD_PLLFBDIV_M_347 = 0b101011001, // PLL Input Clock Multiplier Setting x347
    PLLFBD_PLLFBDIV_M_348 = 0b101011010, // PLL Input Clock Multiplier Setting x348
    PLLFBD_PLLFBDIV_M_349 = 0b101011011, // PLL Input Clock Multiplier Setting x349
    PLLFBD_PLLFBDIV_M_350 = 0b101011100, // PLL Input Clock Multiplier Setting x350
    PLLFBD_PLLFBDIV_M_351 = 0b101011101, // PLL Input Clock Multiplier Setting x351
    PLLFBD_PLLFBDIV_M_352 = 0b101011110, // PLL Input Clock Multiplier Setting x352
    PLLFBD_PLLFBDIV_M_353 = 0b101011111, // PLL Input Clock Multiplier Setting x353
    PLLFBD_PLLFBDIV_M_354 = 0b101100000, // PLL Input Clock Multiplier Setting x354
    PLLFBD_PLLFBDIV_M_355 = 0b101100001, // PLL Input Clock Multiplier Setting x355
    PLLFBD_PLLFBDIV_M_356 = 0b101100010, // PLL Input Clock Multiplier Setting x356
    PLLFBD_PLLFBDIV_M_357 = 0b101100011, // PLL Input Clock Multiplier Setting x357
    PLLFBD_PLLFBDIV_M_358 = 0b101100100, // PLL Input Clock Multiplier Setting x358
    PLLFBD_PLLFBDIV_M_359 = 0b101100101, // PLL Input Clock Multiplier Setting x359
    PLLFBD_PLLFBDIV_M_360 = 0b101100110, // PLL Input Clock Multiplier Setting x360
    PLLFBD_PLLFBDIV_M_361 = 0b101100111, // PLL Input Clock Multiplier Setting x361
    PLLFBD_PLLFBDIV_M_362 = 0b101101000, // PLL Input Clock Multiplier Setting x362
    PLLFBD_PLLFBDIV_M_363 = 0b101101001, // PLL Input Clock Multiplier Setting x363
    PLLFBD_PLLFBDIV_M_364 = 0b101101010, // PLL Input Clock Multiplier Setting x364
    PLLFBD_PLLFBDIV_M_365 = 0b101101011, // PLL Input Clock Multiplier Setting x365
    PLLFBD_PLLFBDIV_M_366 = 0b101101100, // PLL Input Clock Multiplier Setting x366
    PLLFBD_PLLFBDIV_M_367 = 0b101101101, // PLL Input Clock Multiplier Setting x367
    PLLFBD_PLLFBDIV_M_368 = 0b101101110, // PLL Input Clock Multiplier Setting x368
    PLLFBD_PLLFBDIV_M_369 = 0b101101111, // PLL Input Clock Multiplier Setting x369
    PLLFBD_PLLFBDIV_M_370 = 0b101110000, // PLL Input Clock Multiplier Setting x370
    PLLFBD_PLLFBDIV_M_371 = 0b101110001, // PLL Input Clock Multiplier Setting x371
    PLLFBD_PLLFBDIV_M_372 = 0b101110010, // PLL Input Clock Multiplier Setting x372
    PLLFBD_PLLFBDIV_M_373 = 0b101110011, // PLL Input Clock Multiplier Setting x373
    PLLFBD_PLLFBDIV_M_374 = 0b101110100, // PLL Input Clock Multiplier Setting x374
    PLLFBD_PLLFBDIV_M_375 = 0b101110101, // PLL Input Clock Multiplier Setting x375
    PLLFBD_PLLFBDIV_M_376 = 0b101110110, // PLL Input Clock Multiplier Setting x376
    PLLFBD_PLLFBDIV_M_377 = 0b101110111, // PLL Input Clock Multiplier Setting x377
    PLLFBD_PLLFBDIV_M_378 = 0b101111000, // PLL Input Clock Multiplier Setting x378
    PLLFBD_PLLFBDIV_M_379 = 0b101111001, // PLL Input Clock Multiplier Setting x379
    PLLFBD_PLLFBDIV_M_380 = 0b101111010, // PLL Input Clock Multiplier Setting x380
    PLLFBD_PLLFBDIV_M_381 = 0b101111011, // PLL Input Clock Multiplier Setting x381
    PLLFBD_PLLFBDIV_M_382 = 0b101111100, // PLL Input Clock Multiplier Setting x382
    PLLFBD_PLLFBDIV_M_383 = 0b101111101, // PLL Input Clock Multiplier Setting x383
    PLLFBD_PLLFBDIV_M_384 = 0b101111110, // PLL Input Clock Multiplier Setting x384
    PLLFBD_PLLFBDIV_M_385 = 0b101111111, // PLL Input Clock Multiplier Setting x385
    PLLFBD_PLLFBDIV_M_386 = 0b110000000, // PLL Input Clock Multiplier Setting x386
    PLLFBD_PLLFBDIV_M_387 = 0b110000001, // PLL Input Clock Multiplier Setting x387
    PLLFBD_PLLFBDIV_M_388 = 0b110000010, // PLL Input Clock Multiplier Setting x388
    PLLFBD_PLLFBDIV_M_389 = 0b110000011, // PLL Input Clock Multiplier Setting x389
    PLLFBD_PLLFBDIV_M_390 = 0b110000100, // PLL Input Clock Multiplier Setting x390
    PLLFBD_PLLFBDIV_M_391 = 0b110000101, // PLL Input Clock Multiplier Setting x391
    PLLFBD_PLLFBDIV_M_392 = 0b110000110, // PLL Input Clock Multiplier Setting x392
    PLLFBD_PLLFBDIV_M_393 = 0b110000111, // PLL Input Clock Multiplier Setting x393
    PLLFBD_PLLFBDIV_M_394 = 0b110001000, // PLL Input Clock Multiplier Setting x394
    PLLFBD_PLLFBDIV_M_395 = 0b110001001, // PLL Input Clock Multiplier Setting x395
    PLLFBD_PLLFBDIV_M_396 = 0b110001010, // PLL Input Clock Multiplier Setting x396
    PLLFBD_PLLFBDIV_M_397 = 0b110001011, // PLL Input Clock Multiplier Setting x397
    PLLFBD_PLLFBDIV_M_398 = 0b110001100, // PLL Input Clock Multiplier Setting x398
    PLLFBD_PLLFBDIV_M_399 = 0b110001101, // PLL Input Clock Multiplier Setting x399
    PLLFBD_PLLFBDIV_M_400 = 0b110001110, // PLL Input Clock Multiplier Setting x400
    PLLFBD_PLLFBDIV_M_401 = 0b110001111, // PLL Input Clock Multiplier Setting x401
    PLLFBD_PLLFBDIV_M_402 = 0b110010000, // PLL Input Clock Multiplier Setting x402
    PLLFBD_PLLFBDIV_M_403 = 0b110010001, // PLL Input Clock Multiplier Setting x403
    PLLFBD_PLLFBDIV_M_404 = 0b110010010, // PLL Input Clock Multiplier Setting x404
    PLLFBD_PLLFBDIV_M_405 = 0b110010011, // PLL Input Clock Multiplier Setting x405
    PLLFBD_PLLFBDIV_M_406 = 0b110010100, // PLL Input Clock Multiplier Setting x406
    PLLFBD_PLLFBDIV_M_407 = 0b110010101, // PLL Input Clock Multiplier Setting x407
    PLLFBD_PLLFBDIV_M_408 = 0b110010110, // PLL Input Clock Multiplier Setting x408
    PLLFBD_PLLFBDIV_M_409 = 0b110010111, // PLL Input Clock Multiplier Setting x409
    PLLFBD_PLLFBDIV_M_410 = 0b110011000, // PLL Input Clock Multiplier Setting x410
    PLLFBD_PLLFBDIV_M_411 = 0b110011001, // PLL Input Clock Multiplier Setting x411
    PLLFBD_PLLFBDIV_M_412 = 0b110011010, // PLL Input Clock Multiplier Setting x412
    PLLFBD_PLLFBDIV_M_413 = 0b110011011, // PLL Input Clock Multiplier Setting x413
    PLLFBD_PLLFBDIV_M_414 = 0b110011100, // PLL Input Clock Multiplier Setting x414
    PLLFBD_PLLFBDIV_M_415 = 0b110011101, // PLL Input Clock Multiplier Setting x415
    PLLFBD_PLLFBDIV_M_416 = 0b110011110, // PLL Input Clock Multiplier Setting x416
    PLLFBD_PLLFBDIV_M_417 = 0b110011111, // PLL Input Clock Multiplier Setting x417
    PLLFBD_PLLFBDIV_M_418 = 0b110100000, // PLL Input Clock Multiplier Setting x418
    PLLFBD_PLLFBDIV_M_419 = 0b110100001, // PLL Input Clock Multiplier Setting x419
    PLLFBD_PLLFBDIV_M_420 = 0b110100010, // PLL Input Clock Multiplier Setting x420
    PLLFBD_PLLFBDIV_M_421 = 0b110100011, // PLL Input Clock Multiplier Setting x421
    PLLFBD_PLLFBDIV_M_422 = 0b110100100, // PLL Input Clock Multiplier Setting x422
    PLLFBD_PLLFBDIV_M_423 = 0b110100101, // PLL Input Clock Multiplier Setting x423
    PLLFBD_PLLFBDIV_M_424 = 0b110100110, // PLL Input Clock Multiplier Setting x424
    PLLFBD_PLLFBDIV_M_425 = 0b110100111, // PLL Input Clock Multiplier Setting x425
    PLLFBD_PLLFBDIV_M_426 = 0b110101000, // PLL Input Clock Multiplier Setting x426
    PLLFBD_PLLFBDIV_M_427 = 0b110101001, // PLL Input Clock Multiplier Setting x427
    PLLFBD_PLLFBDIV_M_428 = 0b110101010, // PLL Input Clock Multiplier Setting x428
    PLLFBD_PLLFBDIV_M_429 = 0b110101011, // PLL Input Clock Multiplier Setting x429
    PLLFBD_PLLFBDIV_M_430 = 0b110101100, // PLL Input Clock Multiplier Setting x430
    PLLFBD_PLLFBDIV_M_431 = 0b110101101, // PLL Input Clock Multiplier Setting x431
    PLLFBD_PLLFBDIV_M_432 = 0b110101110, // PLL Input Clock Multiplier Setting x432
    PLLFBD_PLLFBDIV_M_433 = 0b110101111, // PLL Input Clock Multiplier Setting x433
    PLLFBD_PLLFBDIV_M_434 = 0b110110000, // PLL Input Clock Multiplier Setting x434
    PLLFBD_PLLFBDIV_M_435 = 0b110110001, // PLL Input Clock Multiplier Setting x435
    PLLFBD_PLLFBDIV_M_436 = 0b110110010, // PLL Input Clock Multiplier Setting x436
    PLLFBD_PLLFBDIV_M_437 = 0b110110011, // PLL Input Clock Multiplier Setting x437
    PLLFBD_PLLFBDIV_M_438 = 0b110110100, // PLL Input Clock Multiplier Setting x438
    PLLFBD_PLLFBDIV_M_439 = 0b110110101, // PLL Input Clock Multiplier Setting x439
    PLLFBD_PLLFBDIV_M_440 = 0b110110110, // PLL Input Clock Multiplier Setting x440
    PLLFBD_PLLFBDIV_M_441 = 0b110110111, // PLL Input Clock Multiplier Setting x441
    PLLFBD_PLLFBDIV_M_442 = 0b110111000, // PLL Input Clock Multiplier Setting x442
    PLLFBD_PLLFBDIV_M_443 = 0b110111001, // PLL Input Clock Multiplier Setting x443
    PLLFBD_PLLFBDIV_M_444 = 0b110111010, // PLL Input Clock Multiplier Setting x444
    PLLFBD_PLLFBDIV_M_445 = 0b110111011, // PLL Input Clock Multiplier Setting x445
    PLLFBD_PLLFBDIV_M_446 = 0b110111100, // PLL Input Clock Multiplier Setting x446
    PLLFBD_PLLFBDIV_M_447 = 0b110111101, // PLL Input Clock Multiplier Setting x447
    PLLFBD_PLLFBDIV_M_448 = 0b110111110, // PLL Input Clock Multiplier Setting x448
    PLLFBD_PLLFBDIV_M_449 = 0b110111111, // PLL Input Clock Multiplier Setting x449
    PLLFBD_PLLFBDIV_M_450 = 0b111000000, // PLL Input Clock Multiplier Setting x450
    PLLFBD_PLLFBDIV_M_451 = 0b111000001, // PLL Input Clock Multiplier Setting x451
    PLLFBD_PLLFBDIV_M_452 = 0b111000010, // PLL Input Clock Multiplier Setting x452
    PLLFBD_PLLFBDIV_M_453 = 0b111000011, // PLL Input Clock Multiplier Setting x453
    PLLFBD_PLLFBDIV_M_454 = 0b111000100, // PLL Input Clock Multiplier Setting x454
    PLLFBD_PLLFBDIV_M_455 = 0b111000101, // PLL Input Clock Multiplier Setting x455
    PLLFBD_PLLFBDIV_M_456 = 0b111000110, // PLL Input Clock Multiplier Setting x456
    PLLFBD_PLLFBDIV_M_457 = 0b111000111, // PLL Input Clock Multiplier Setting x457
    PLLFBD_PLLFBDIV_M_458 = 0b111001000, // PLL Input Clock Multiplier Setting x458
    PLLFBD_PLLFBDIV_M_459 = 0b111001001, // PLL Input Clock Multiplier Setting x459
    PLLFBD_PLLFBDIV_M_460 = 0b111001010, // PLL Input Clock Multiplier Setting x460
    PLLFBD_PLLFBDIV_M_461 = 0b111001011, // PLL Input Clock Multiplier Setting x461
    PLLFBD_PLLFBDIV_M_462 = 0b111001100, // PLL Input Clock Multiplier Setting x462
    PLLFBD_PLLFBDIV_M_463 = 0b111001101, // PLL Input Clock Multiplier Setting x463
    PLLFBD_PLLFBDIV_M_464 = 0b111001110, // PLL Input Clock Multiplier Setting x464
    PLLFBD_PLLFBDIV_M_465 = 0b111001111, // PLL Input Clock Multiplier Setting x465
    PLLFBD_PLLFBDIV_M_466 = 0b111010000, // PLL Input Clock Multiplier Setting x466
    PLLFBD_PLLFBDIV_M_467 = 0b111010001, // PLL Input Clock Multiplier Setting x467
    PLLFBD_PLLFBDIV_M_468 = 0b111010010, // PLL Input Clock Multiplier Setting x468
    PLLFBD_PLLFBDIV_M_469 = 0b111010011, // PLL Input Clock Multiplier Setting x469
    PLLFBD_PLLFBDIV_M_470 = 0b111010100, // PLL Input Clock Multiplier Setting x470
    PLLFBD_PLLFBDIV_M_471 = 0b111010101, // PLL Input Clock Multiplier Setting x471
    PLLFBD_PLLFBDIV_M_472 = 0b111010110, // PLL Input Clock Multiplier Setting x472
    PLLFBD_PLLFBDIV_M_473 = 0b111010111, // PLL Input Clock Multiplier Setting x473
    PLLFBD_PLLFBDIV_M_474 = 0b111011000, // PLL Input Clock Multiplier Setting x474
    PLLFBD_PLLFBDIV_M_475 = 0b111011001, // PLL Input Clock Multiplier Setting x475
    PLLFBD_PLLFBDIV_M_476 = 0b111011010, // PLL Input Clock Multiplier Setting x476
    PLLFBD_PLLFBDIV_M_477 = 0b111011011, // PLL Input Clock Multiplier Setting x477
    PLLFBD_PLLFBDIV_M_478 = 0b111011100, // PLL Input Clock Multiplier Setting x478
    PLLFBD_PLLFBDIV_M_479 = 0b111011101, // PLL Input Clock Multiplier Setting x479
    PLLFBD_PLLFBDIV_M_480 = 0b111011110, // PLL Input Clock Multiplier Setting x480
    PLLFBD_PLLFBDIV_M_481 = 0b111011111, // PLL Input Clock Multiplier Setting x481
    PLLFBD_PLLFBDIV_M_482 = 0b111100000, // PLL Input Clock Multiplier Setting x482
    PLLFBD_PLLFBDIV_M_483 = 0b111100001, // PLL Input Clock Multiplier Setting x483
    PLLFBD_PLLFBDIV_M_484 = 0b111100010, // PLL Input Clock Multiplier Setting x484
    PLLFBD_PLLFBDIV_M_485 = 0b111100011, // PLL Input Clock Multiplier Setting x485
    PLLFBD_PLLFBDIV_M_486 = 0b111100100, // PLL Input Clock Multiplier Setting x486
    PLLFBD_PLLFBDIV_M_487 = 0b111100101, // PLL Input Clock Multiplier Setting x487
    PLLFBD_PLLFBDIV_M_488 = 0b111100110, // PLL Input Clock Multiplier Setting x488
    PLLFBD_PLLFBDIV_M_489 = 0b111100111, // PLL Input Clock Multiplier Setting x489
    PLLFBD_PLLFBDIV_M_490 = 0b111101000, // PLL Input Clock Multiplier Setting x490
    PLLFBD_PLLFBDIV_M_491 = 0b111101001, // PLL Input Clock Multiplier Setting x491
    PLLFBD_PLLFBDIV_M_492 = 0b111101010, // PLL Input Clock Multiplier Setting x492
    PLLFBD_PLLFBDIV_M_493 = 0b111101011, // PLL Input Clock Multiplier Setting x493
    PLLFBD_PLLFBDIV_M_494 = 0b111101100, // PLL Input Clock Multiplier Setting x494
    PLLFBD_PLLFBDIV_M_495 = 0b111101101, // PLL Input Clock Multiplier Setting x495
    PLLFBD_PLLFBDIV_M_496 = 0b111101110, // PLL Input Clock Multiplier Setting x496
    PLLFBD_PLLFBDIV_M_497 = 0b111101111, // PLL Input Clock Multiplier Setting x497
    PLLFBD_PLLFBDIV_M_498 = 0b111110000, // PLL Input Clock Multiplier Setting x498
    PLLFBD_PLLFBDIV_M_499 = 0b111110001, // PLL Input Clock Multiplier Setting x499
    PLLFBD_PLLFBDIV_M_500 = 0b111110010, // PLL Input Clock Multiplier Setting x500
    PLLFBD_PLLFBDIV_M_501 = 0b111110011, // PLL Input Clock Multiplier Setting x501
    PLLFBD_PLLFBDIV_M_502 = 0b111110100, // PLL Input Clock Multiplier Setting x502
    PLLFBD_PLLFBDIV_M_503 = 0b111110101, // PLL Input Clock Multiplier Setting x503
    PLLFBD_PLLFBDIV_M_504 = 0b111110110, // PLL Input Clock Multiplier Setting x504
    PLLFBD_PLLFBDIV_M_505 = 0b111110111, // PLL Input Clock Multiplier Setting x505
    PLLFBD_PLLFBDIV_M_506 = 0b111111000, // PLL Input Clock Multiplier Setting x506
    PLLFBD_PLLFBDIV_M_507 = 0b111111001, // PLL Input Clock Multiplier Setting x507
    PLLFBD_PLLFBDIV_M_508 = 0b111111010, // PLL Input Clock Multiplier Setting x508
    PLLFBD_PLLFBDIV_M_509 = 0b111111011, // PLL Input Clock Multiplier Setting x509
    PLLFBD_PLLFBDIV_M_510 = 0b111111100, // PLL Input Clock Multiplier Setting x510
    PLLFBD_PLLFBDIV_M_511 = 0b111111101, // PLL Input Clock Multiplier Setting x511
    PLLFBD_PLLFBDIV_M_512 = 0b111111110, // PLL Input Clock Multiplier Setting x512
    PLLFBD_PLLFBDIV_M_513 = 0b111111111 // PLL Input Clock Multiplier Setting x513
} PLLFBD_PLLFBDIV_e; // PLL Feedback Divider bits (also denoted as ?M?, PLL multiplier)

typedef union {
    struct {
        volatile PLLFBD_PLLFBDIV_e PLLFBDIV : 9; // PLL Feedback Divider bits (also denoted as ?M?, PLL multiplier)
        volatile unsigned : 7; // reserved
    } __attribute__((packed)) bits;
    volatile uint16_t value;
} PLLFBD_t;


/* ===========================================================================
 * OSCTUN: FRC OSCILLATOR TUNING REGISTER
 * ===========================================================================*/

#define REG_OSCTUN_VALID_DATA_WRITE_MASK    0x003F
#define REG_OSCTUN_VALID_DATA_READ_MASK     0x003F

#define REG_OSCTUN_TUNE_VALUE_MASK           0b0000000000111111
#define REG_OSCTUN_TUNE_VALUE(x)     (x & REG_OSCTUN_TUNE_VALUE_MASK)


typedef enum {
    OSCTUN_TUN_MINUS_31 = 0b100001, // Center frequency -1.457% (=7.2626191 MHz)
    OSCTUN_TUN_MINUS_30 = 0b100010, // Center frequency -1.41% (=7.266083 MHz)
    OSCTUN_TUN_MINUS_29 = 0b100011, // Center frequency -1.363% (=7.2695469 MHz)
    OSCTUN_TUN_MINUS_28 = 0b100100, // Center frequency -1.316% (=7.2730108 MHz)
    OSCTUN_TUN_MINUS_27 = 0b100101, // Center frequency -1.269% (=7.2764747 MHz)
    OSCTUN_TUN_MINUS_26 = 0b100110, // Center frequency -1.222% (=7.2799386 MHz)
    OSCTUN_TUN_MINUS_25 = 0b100111, // Center frequency -1.175% (=7.2834025 MHz)
    OSCTUN_TUN_MINUS_24 = 0b101000, // Center frequency -1.128% (=7.2868664 MHz)
    OSCTUN_TUN_MINUS_23 = 0b101001, // Center frequency -1.081% (=7.2903303 MHz)
    OSCTUN_TUN_MINUS_22 = 0b101010, // Center frequency -1.034% (=7.2937942 MHz)
    OSCTUN_TUN_MINUS_21 = 0b101011, // Center frequency -0.987% (=7.2972581 MHz)
    OSCTUN_TUN_MINUS_20 = 0b101100, // Center frequency -0.94% (=7.300722 MHz)
    OSCTUN_TUN_MINUS_19 = 0b101101, // Center frequency -0.893% (=7.3041859 MHz)
    OSCTUN_TUN_MINUS_18 = 0b101110, // Center frequency -0.846% (=7.3076498 MHz)
    OSCTUN_TUN_MINUS_17 = 0b101111, // Center frequency -0.799% (=7.3111137 MHz)
    OSCTUN_TUN_MINUS_16 = 0b110000, // Center frequency -0.752% (=7.3145776 MHz)
    OSCTUN_TUN_MINUS_15 = 0b110001, // Center frequency -0.705% (=7.3180415 MHz)
    OSCTUN_TUN_MINUS_14 = 0b110010, // Center frequency -0.658% (=7.3215054 MHz)
    OSCTUN_TUN_MINUS_13 = 0b110011, // Center frequency -0.611% (=7.3249693 MHz)
    OSCTUN_TUN_MINUS_12 = 0b110100, // Center frequency -0.564% (=7.3284332 MHz)
    OSCTUN_TUN_MINUS_11 = 0b110101, // Center frequency -0.517% (=7.3318971 MHz)
    OSCTUN_TUN_MINUS_10 = 0b110110, // Center frequency -0.47% (=7.335361 MHz)
    OSCTUN_TUN_MINUS_9 = 0b110111, // Center frequency -0.423% (=7.3388249 MHz)
    OSCTUN_TUN_MINUS_8 = 0b111000, // Center frequency -0.376% (=7.3422888 MHz)
    OSCTUN_TUN_MINUS_7 = 0b111001, // Center frequency -0.329% (=7.3457527 MHz)
    OSCTUN_TUN_MINUS_6 = 0b111010, // Center frequency -0.282% (=7.3492166 MHz)
    OSCTUN_TUN_MINUS_5 = 0b111011, // Center frequency -0.235% (=7.3526805 MHz)
    OSCTUN_TUN_MINUS_4 = 0b111100, // Center frequency -0.188% (=7.3561444 MHz)
    OSCTUN_TUN_MINUS_3 = 0b111101, // Center frequency -0.141% (=7.3596083 MHz)
    OSCTUN_TUN_MINUS_2 = 0b111110, // Center frequency -0.094% (=7.3630722 MHz)
    OSCTUN_TUN_MINUS_1 = 0b111111, // Center frequency -0.047% (=7.3665361 MHz)
    OSCTUN_TUN_NOMINAL = 0b000000, // Center frequency 0% (=7.37 MHz)
    OSCTUN_TUN_PLUS_1 = 0b000001, // Center frequency 0.047% (=7.3734639 MHz)
    OSCTUN_TUN_PLUS_2 = 0b000010, // Center frequency 0.094% (=7.3769278 MHz)
    OSCTUN_TUN_PLUS_3 = 0b000011, // Center frequency 0.141% (=7.3803917 MHz)
    OSCTUN_TUN_PLUS_4 = 0b000100, // Center frequency 0.188% (=7.3838556 MHz)
    OSCTUN_TUN_PLUS_5 = 0b000101, // Center frequency 0.235% (=7.3873195 MHz)
    OSCTUN_TUN_PLUS_6 = 0b000110, // Center frequency 0.282% (=7.3907834 MHz)
    OSCTUN_TUN_PLUS_7 = 0b000111, // Center frequency 0.329% (=7.3942473 MHz)
    OSCTUN_TUN_PLUS_8 = 0b001000, // Center frequency 0.376% (=7.3977112 MHz)
    OSCTUN_TUN_PLUS_9 = 0b001001, // Center frequency 0.423% (=7.4011751 MHz)
    OSCTUN_TUN_PLUS_10 = 0b001010, // Center frequency 0.47% (=7.404639 MHz)
    OSCTUN_TUN_PLUS_11 = 0b001011, // Center frequency 0.517% (=7.4081029 MHz)
    OSCTUN_TUN_PLUS_12 = 0b001100, // Center frequency 0.564% (=7.4115668 MHz)
    OSCTUN_TUN_PLUS_13 = 0b001101, // Center frequency 0.611% (=7.4150307 MHz)
    OSCTUN_TUN_PLUS_14 = 0b001110, // Center frequency 0.658% (=7.4184946 MHz)
    OSCTUN_TUN_PLUS_15 = 0b001111, // Center frequency 0.705% (=7.4219585 MHz)
    OSCTUN_TUN_PLUS_16 = 0b010000, // Center frequency 0.752% (=7.4254224 MHz)
    OSCTUN_TUN_PLUS_17 = 0b010001, // Center frequency 0.799% (=7.4288863 MHz)
    OSCTUN_TUN_PLUS_18 = 0b010010, // Center frequency 0.846% (=7.4323502 MHz)
    OSCTUN_TUN_PLUS_19 = 0b010011, // Center frequency 0.893% (=7.4358141 MHz)
    OSCTUN_TUN_PLUS_20 = 0b010100, // Center frequency 0.94% (=7.439278 MHz)
    OSCTUN_TUN_PLUS_21 = 0b010101, // Center frequency 0.987% (=7.4427419 MHz)
    OSCTUN_TUN_PLUS_22 = 0b010110, // Center frequency 1.034% (=7.4462058 MHz)
    OSCTUN_TUN_PLUS_23 = 0b010111, // Center frequency 1.081% (=7.4496697 MHz)
    OSCTUN_TUN_PLUS_24 = 0b011000, // Center frequency 1.128% (=7.4531336 MHz)
    OSCTUN_TUN_PLUS_25 = 0b011001, // Center frequency 1.175% (=7.4565975 MHz)
    OSCTUN_TUN_PLUS_26 = 0b011010, // Center frequency 1.222% (=7.4600614 MHz)
    OSCTUN_TUN_PLUS_27 = 0b011011, // Center frequency 1.269% (=7.4635253 MHz)
    OSCTUN_TUN_PLUS_28 = 0b011100, // Center frequency 1.316% (=7.4669892 MHz)
    OSCTUN_TUN_PLUS_29 = 0b011101, // Center frequency 1.363% (=7.4704531 MHz)
    OSCTUN_TUN_PLUS_30 = 0b011110, // Center frequency 1.41% (=7.473917 MHz)
    OSCTUN_TUN_PLUS_31 = 0b011111 // Center frequency 1.457% (=7.4773809 MHz)
} OSCTUN_TUN_e; // FRC Oscillator Tuning bits



typedef union {
    struct {
        volatile OSCTUN_TUN_e TUN : 6; // FRC Oscillator Tuning bits
        volatile unsigned : 10; // reserved
    } __attribute__((packed)) bits;
    volatile uint16_t value;
} OSCTUN_t;


/* ===========================================================================
 * REFOCON: REFERENCE OSCILLATOR CONTROL REGISTER
 * ===========================================================================*/

#define REG_REFOCON_ROON_ENABLED    0b1000000000000000
#define REG_REFOCON_ROON_DISABLED   0b0000000000000000

typedef enum {
    REFOCON_ROON_ENABLED  = 0b1, // Reference oscillator output is enabled on the RPn pin
    REFOCON_ROON_DISABLED = 0b0  // Reference oscillator output is disabled
}ROON_e; // Reference Oscillator Output Enable bit

#define REG_REFOCON_ROSSLP_RUN     0b0010000000000000
#define REG_REFOCON_ROSSLP_STOP    0b0000000000000000

typedef enum {
    REFOCON_ROSSLP_RUN  = 0b1, // Reference oscillator output continues to run in Sleep
    REFOCON_ROSSLP_STOP = 0b0  // Reference oscillator output is disabled in Sleep
}ROSSLP_e; // Reference Oscillator Run in Sleep bit

#define REG_REFOCON_ROSEL_EXTCLK   0b0001000000000000
#define REG_REFOCON_ROSEL_SYSCLK   0b0000000000000000

typedef enum {
    REFOCON_ROSEL_EXTCLK  = 0b1, // Oscillator crystal is used as the reference clock
    REFOCON_ROSEL_SYSCLK = 0b0   // System clock is used as the reference clock
}ROSEL_e; // Reference Oscillator Source Select bit


#define REG_REFOCON_RODIV_32768    0b0000111100000000
#define REG_REFOCON_RODIV_16384    0b0000111000000000
#define REG_REFOCON_RODIV_8192     0b0000110100000000
#define REG_REFOCON_RODIV_4096     0b0000110000000000
#define REG_REFOCON_RODIV_2048     0b0000101100000000
#define REG_REFOCON_RODIV_1024     0b0000101000000000
#define REG_REFOCON_RODIV_512      0b0000100100000000
#define REG_REFOCON_RODIV_256      0b0000100000000000
#define REG_REFOCON_RODIV_128      0b0000011100000000
#define REG_REFOCON_RODIV_64       0b0000011000000000
#define REG_REFOCON_RODIV_32       0b0000010100000000
#define REG_REFOCON_RODIV_16       0b0000010000000000
#define REG_REFOCON_RODIV_8        0b0000001100000000
#define REG_REFOCON_RODIV_4        0b0000001000000000
#define REG_REFOCON_RODIV_2        0b0000000100000000
#define REG_REFOCON_RODIV_1        0b0000000000000000

typedef enum {
    REFOCON_RODIV_32768 = 0b1111, // Reference clock divided by 32,768
    REFOCON_RODIV_16384 = 0b1110, // Reference clock divided by 16,384
    REFOCON_RODIV_8192  = 0b1101, // Reference clock divided by 8,192
    REFOCON_RODIV_4096  = 0b1100, // Reference clock divided by 4,096
    REFOCON_RODIV_2048  = 0b1011, // Reference clock divided by 2,048
    REFOCON_RODIV_1024  = 0b1010, // Reference clock divided by 1,024
    REFOCON_RODIV_512   = 0b1001, // Reference clock divided by 512
    REFOCON_RODIV_256   = 0b1000, // Reference clock divided by 256
    REFOCON_RODIV_128   = 0b0111, // Reference clock divided by 128
    REFOCON_RODIV_64    = 0b0110, // Reference clock divided by 64
    REFOCON_RODIV_32    = 0b0101, // Reference clock divided by 32
    REFOCON_RODIV_16    = 0b0100, // Reference clock divided by 16
    REFOCON_RODIV_8     = 0b0011, // Reference clock divided by 8
    REFOCON_RODIV_4     = 0b0010, // Reference clock divided by 4
    REFOCON_RODIV_2     = 0b0001, // Reference clock divided by 2
    REFOCON_RODIV_1     = 0b0000  // Reference clock
}RODIV_e; // Reference Oscillator Divider bits



typedef union {
    struct {
        volatile unsigned : 8;          // Bit [7:0]: (unimplemented)
        volatile RODIV_e rodiv : 4;     // Bit [11:8]: Reference Oscillator Divider bits
        volatile ROSEL_e rosel : 1;     // Bit 12: Reference Oscillator Source Select bit
        volatile ROSSLP_e rosslp : 1;   // Bit 13: Reference Oscillator Run in Sleep bit
        volatile unsigned : 1;          // Bit 14: (unimplemented)
        volatile ROON_e roon : 1;       // Bit 15: Reference Oscillator Output Enable bit
    } __attribute__((packed)) bits;     // REFERENCE OSCILLATOR CONTROL REGISTER BIT FIELD
    volatile uint16_t value; // REFERENCE OSCILLATOR CONTROL REGISTER VALUE
}REFOCON_t; // REFERENCE OSCILLATOR CONTROL REGISTER

/* ===========================================================================
 * ACLKCON: AUXILIARY CLOCK DIVISOR CONTROL REGISTER
 * ===========================================================================*/

#define REG_ACLKCON_ENAPLL_ENABLED  0b1000000000000000 // APLL is enabled
#define REG_ACLKCON_ENAPLL_DISABLED 0b0000000000000000 // APLL is disabled

typedef enum {
    ACLKCON_ENAPLL_ENABLED  = 0b1, // APLL is enabled
    ACLKCON_ENAPLL_DISABLED = 0b0  // APLL is disabled
}ENAPLL_e; // Auxiliary PLL Enable bit

#define REG_ACLKCON_APLLCK_LOCKED   0b0100000000000000 // Indicates that Auxiliary PLL is in lock
#define REG_ACLKCON_APLLCK_UNLOCKED 0b0000000000000000 // Indicates that Auxiliary PLL is not in lock

typedef enum {
    ACLKCON_APLLCK_LOCKED   = 0b1, // Indicates that Auxiliary PLL is in lock
    ACLKCON_APLLCK_UNLOCKED = 0b0  // Indicates that Auxiliary PLL is not in lock
}APLLCK_e; // APLL Locked Status bit (read-only)

#define REG_ACLKCON_SELACLK_ACLK 0b0010000000000000 // Auxiliary oscillators provide the source clock for the auxiliary clock divider
#define REG_ACLKCON_SELACLK_FVCO 0b0000000000000000 // Primary PLL (FVCO) provides the source clock for the auxiliary clock divider

typedef enum {
    ACLKCON_SELACLK_ACLK = 0b1, // Auxiliary oscillators provide the source clock for the auxiliary clock divider
    ACLKCON_SELACLK_FVCO = 0b0  // Primary PLL (FVCO) provides the source clock for the auxiliary clock divider
}SELACLK_e; // Select Auxiliary Clock Source for Auxiliary Clock Divider bit

#define REG_ACLKCON_APSTSCLR_1   0b0000011100000000 // Divided by 1
#define REG_ACLKCON_APSTSCLR_2   0b0000011000000000 // Divided by 2
#define REG_ACLKCON_APSTSCLR_4   0b0000010100000000 // Divided by 4
#define REG_ACLKCON_APSTSCLR_8   0b0000010000000000 // Divided by 8
#define REG_ACLKCON_APSTSCLR_16  0b0000001100000000 // Divided by 16
#define REG_ACLKCON_APSTSCLR_32  0b0000001000000000 // Divided by 32
#define REG_ACLKCON_APSTSCLR_64  0b0000000100000000 // Divided by 64
#define REG_ACLKCON_APSTSCLR_128 0b0000000000000000 // Divided by 256

typedef enum {
    ACLKCON_APSTSCLR_1   = 0b111, // Divided by 1
    ACLKCON_APSTSCLR_2   = 0b110, // Divided by 2
    ACLKCON_APSTSCLR_4   = 0b101, // Divided by 4
    ACLKCON_APSTSCLR_8   = 0b100, // Divided by 8
    ACLKCON_APSTSCLR_16  = 0b011, // Divided by 16
    ACLKCON_APSTSCLR_32  = 0b010, // Divided by 32
    ACLKCON_APSTSCLR_64  = 0b001, // Divided by 64
    ACLKCON_APSTSCLR_128 = 0b000  // Divided by 256
}APSTSCLR_e; // Auxiliary Clock Output Divider bits
 
#define REG_ACLKCON_ASRCSEL_PCLK    0b0000000010000000 // Primary oscillator is the clock source
#define REG_ACLKCON_ASRCSEL_NONE    0b0000000000000000 // No clock input is selected

typedef enum {
    ACLKCON_ASRCSEL_PCLK = 0b1, // Primary oscillator is the clock source
    ACLKCON_ASRCSEL_NONE = 0b0  // No clock input is selected
}ASRCSEL_e; // Select Reference Clock Source for Auxiliary Clock bit
    
#define REG_ACLKCON_FRCSEL_FRC      0b0000000001000000 // Selects the FRC clock for Auxiliary PLL
#define REG_ACLKCON_FRCSEL_ASRCSEL  0b0000000000000000 // Input clock source is determined by the ASRCSEL bit setting

typedef enum {
    ACLKCON_FRCSEL_FRC     = 0b1, // Selects the FRC clock for Auxiliary PLL
    ACLKCON_FRCSEL_ASRCSEL = 0b0  // Input clock source is determined by the ASRCSEL bit setting
}FRCSEL_e; // Select Reference Clock Source for Auxiliary PLL bit
    

typedef union {
    struct {
        volatile unsigned : 6;              // Bit [5:0]: (unimplemented)
        volatile FRCSEL_e frcsel : 1;       // Bit 6: Select Reference Clock Source for Auxiliary PLL bit
        volatile ASRCSEL_e asrcsel : 1;     // Bit 7: Select Reference Clock Source for Auxiliary Clock bit
        volatile APSTSCLR_e apstsclr : 3;   // Bit {10:8]: Auxiliary Clock Output Divider bits
        volatile unsigned : 2;              // Bit [12:11]: (unimplemented)
        volatile SELACLK_e selaclk : 1;     // Bit 13: Select Auxiliary Clock Source for Auxiliary Clock Divider bit
        volatile APLLCK_e apllck : 1;       // Bit 14: APLL Locked Status bit (read-only)
        volatile ENAPLL_e enapll : 1;       // Bit 15: Auxiliary PLL Enable bit
    } __attribute__ ((packed)) bits; // AUXILIARY CLOCK DIVISOR CONTROL REGISTER BIT FIELD
    volatile uint16_t value; // AUXILIARY CLOCK DIVISOR CONTROL REGISTER VALUE
}ACLKCON_t; // AUXILIARY CLOCK DIVISOR CONTROL REGISTER

/* ***************************************************************************************
 *	ERROR CODES
 * **************************************************************************************/

typedef enum {
    OSCERR_SUCCESS = 0x0001, // Clock initialization was successfully performed
    OSCERR_FAILURE = 0x0000, // Global Clock Error
    // (reserved)
    OSCERR_CSF = 0x0002, // Clock switch-over failed
    OSCERR_PLL_LCK = 0x0008, // Primary PLL does not lock in
    OSCERR_APLL_LCK = 0x0010, // Auxiliary PLL does not lock in
    OSCERR_CSD = 0x0004, // Clock switching is disabled but desired clock differs 
    // from current clock
    OSCERR_FRCTUN = 0x0020 // FRC tuning failed
} OSC_CFG_ERR_RESULT_t;

typedef struct {
    volatile OSCCON_t osccon; // OSCILLATOR CONTROL REGISTER
    volatile CLKDIV_t clkdiv; // CLOCK DIVISOR REGISTER
    volatile PLLFBD_t pllfbd; // PLL FEEDBACK DIVISOR REGISTER
    volatile OSCTUN_t osctun; // FRC OSCILLATOR TUNING REGISTER
    volatile REFOCON_t refocon; // REFERENCE OSCILLATOR CONTROL REGISTER
    volatile ACLKCON_t aclkcon; // ACLKCON: AUXILIARY CLOCK DIVISOR CONTROL REGISTER
}OSC_CONFIG_t;

/* ***************************************************************************************
 *	Prototypes
 * **************************************************************************************/

extern int16_t init_FOSC(OSC_CONFIG_t osc_config);
extern int16_t init_AUXCLK(AUXOSC_CONFIG_t aux_clock_config);

extern volatile uint16_t smpsOSC_Initialize(volatile OSC_CONFIG_t osc_config);
extern volatile uint16_t smpsOSC_FRC_Initialize(volatile CLKDIV_FRCDIVN_e frc_div, volatile OSCTUN_TUN_e frc_tun);
extern volatile uint16_t smpsOSC_AUXCLK_Initialize(volatile AUXOSC_CONFIG_t aux_clock_config);

extern volatile uint16_t smpsOSC_FRC_DefaultInitialize(volatile CPU_SPEED_DEFAULTS_e cpu_speed);
extern volatile uint16_t smpsOSC_AUXCLK_DefaultInitialize(volatile AUX_PLL_DEFAULTS_e afpllo_frequency);
extern volatile uint16_t smpsOSC_GetFrequencies(volatile uint32_t main_osc_frequency);


#endif  /* __P33SMPS_EP__
#endif  /* __MCAL_P33SMPS_OSCILLATOR_H__ */
