/*
 * main.c
 *
 *  Created on: Feb 2, 2020
 *  Author: Gavin
 */


#include <msp430.h>

int main(void){

    WDTCTL = WDTPW + WDTHOLD;   // stop watchdog timer

    // Clear P1
    P1OUT = 0;
    // Set P1 for GPIO
    P1DIR = P1DIR | BIT0;
    // Toggle P1 to flash LED
    while(1){
        P1OUT ^= BIT0;
        __delay_cycles(500000); // Increase cycles to make flash slower, decrease to make flash faster
        //__delay_cycles(1000000); // Slow flash
        //__delay_cycles(100000); // Fast flash
    }
}
