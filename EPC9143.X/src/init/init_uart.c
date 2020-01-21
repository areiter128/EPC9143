/*
 * File:   init_uart.c
 * Author: M91406.LA
 *
 * Created on November 2, 2019, 2:23 PM
 */


#include <xc.h>
#include <stdint.h>
#include "user.h"

volatile uint16_t ConfigureUART(void) {
    
    U1MODEbits.STSEL = 0; // 1-Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled
    U1MODEbits.BRGH = 0; // Standard-Speed mode
    U1BRG = BRGVAL; // Baud Rate setting for 9600
    U1STAbits.UTXISEL0 = 1; // Interrupt when 4 deep FIFO TX buffer is empty
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL1 = 0; // Interrupt after one RX character is received;

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX
    
    return (8);
}

