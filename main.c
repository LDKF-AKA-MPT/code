#include <msp430.h> 

/*============================================================================
*   Title: Initalization of the MSP430
*
*   Description:Initialize all values and perefeials that will be used throughout the
*   program
*
*
*
*
*
*
*
============================================================================*/
void init_msp430(){
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    __bis_SR_register( GIE ); // Let interupts be global

    P1DIR |= BIT0; // Set P1.0 as output
    P1OUT &= ~BIT0; // Initally led off

    P1REN |= BIT1; // Enable P1.1 internal resistor
    P1OUT |= BIT1; // Set P1.1 as pull-Up resistor
    P1IES &= ~BIT1; // P1.1 Lo/Hi edge
    P1IFG &= ~BIT1; // P1.1 IFG cleared
    P1IE |= BIT1; // P1.1 interrupt enabled

    P2REN |= BIT1; // Enable P2.1 internal resistor
    P2OUT |= BIT1; // Set P2.1 as pull-Up resistor
    P2IES &= ~BIT1; // P2.1 Lo/Hi edge
    P2IFG &= ~BIT1; // P2.1 IFG cleared
    P2IE |= BIT1; // P2.1 interrupt enabled
}

/*===============================================================================
* main.c
*
*
*
================================================================================*/

void main (void)
{



    while(1) {
        // Infinite loop
    }
}

// P1.1 interrupt
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG1:
        P1OUT^=BIT0;
        break;
    default:   _never_executed();
    }
}

// P2.1 interrupt
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    switch( __even_in_range( P2IV, P2IV_P2IFG7 )) {
    case P2IV_P2IFG1:
        // DO STUFF
    default:   _never_executed();
    }
}
