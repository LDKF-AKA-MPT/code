#include <msp430.h>
int distance = 12;
int counter = 0;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  P1DIR |= BIT2;                            // P1.2 output
  P1SEL |= BIT2;                            // P1.2 options select

  P1DIR |= BIT0; // Set P1.0 as output
  P1OUT &= ~BIT0; // Initally led off

  P1REN |= BIT1; // Enable P1.1 internal resistor
  P1OUT |= BIT1; // Set P1.1 as pull-Up resistor
  P1IES &= ~BIT1; // P1.1 Lo/Hi edge
  P1IFG &= ~BIT1; // P1.1 IFG cleared
  P1IE |= BIT1; // P1.1 interrupt enabled

  TA0CCR0 = 512-1;                          // PWM Period
  TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
  TA0CCR1 = 256;                            // CCR1 PWM duty cycle
  TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR

  TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
  TA1CCR0 = 65536-1;
  TA1CTL = TASSEL_2 + MC_1 + TACLR;
/*  while (1) {
      scanf("%d", &distance);
  }*/
  __bis_SR_register(LPM0_bits+GIE);             // Enter LPM0
  __no_operation();                         // For debugger
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

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG1:
        distance -= 1;
        if (distance < 1) {
            distance = 12;
        }
        P1OUT^=BIT0;
        break;
    default:   _never_executed();
    }
}


