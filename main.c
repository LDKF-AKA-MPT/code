#include <msp430.h> 
int button1Flag = 0;
int button2Flag = 0;
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

    TA0CCR0 = 512-1;                          // PWM Period
    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA0CCR1 = 256;                            // CCR1 PWM duty cycle
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR

    TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
    TA1CCR0 = 65536-1;
    TA1CTL = TASSEL_2 + MC_1 + TACLR;

    __bis_SR_register(LPM0_bits+GIE);             // Enter LPM0
    __no_operation();                         // For debugger
}

/*===============================================================================
* main.c
*
*
*
================================================================================*/

void main (void)
{
    while(1){
        if (button1Flag){
            distance -= 1;
            if (distance < 1){
                distance = 12;
            }
            P1OUT ^= BIT0;
        }
        if (button2Flag){

        }
    }
}

// P1.1 interrupt
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG1:
        button1Flag = 1;
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
        button2Flag = 1;
        break;
    default:   _never_executed();
    }
}

// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (counter >= 9) {
        if (P1DIR & BIT2) {
            P1DIR &= 0xfb;      //turn off output
        }
        else {
            P1DIR |= BIT2;      //turn on output
        }
        counter = 0;
        if (distance > 10) {        //change frequency according to distance
            TA1CCR0 = 65536-1;
        }
        else if (distance > 8) {
            TA1CCR0 = 32768-1;
        }
        else if (distance > 6){
            TA1CCR0 = 16384-1;
        }
        else if (distance > 4){
            TA1CCR0 = 8192-1;
        }
        else if (distance > 2){
            TA1CCR0 = 4096-1;
        }
        else {
            P1DIR |= BIT2;
        }
    }
    counter ++;
}
