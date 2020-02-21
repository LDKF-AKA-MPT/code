/*
 * main.c
 *
 *  Created on: Feb 2, 2020
 *  Author: Gavin
 */

#include <msp430.h>

/*
 * SCL: Serial Clock (pin 14) (p4.3)
 * SS: Slave Select (p1.3)
 * SI: Serial Data (pin 13) (p4.1)
 * http://www.newhavendisplay.com/specs/NHD-0216K3Z-FSRGB-FBW-V3.pdf
 */

void data_wr(unsigned char data){
    P1OUT &= ~BIT3; //SS = 0;
    P4OUT &= ~BIT3; //SCL = 0;
    while(!(UCB1IFG&UCTXIFG)); // Wait for TX buffer ready
    UCB1TXBUF = data;
    __delay_cycles(100);
    P4OUT |= BIT3; //SCL = 1;
    P1OUT |= BIT3; //SS = 1;
}

void str_wr(char* string){
    while(string){
        data_wr(string[0]);
        string++;
    }
}

void init_msp430(){
    WDTCTL = WDTPW + WDTHOLD;

    P4SEL |= BIT1 + BIT3; // SPI pins 4.1 (UCB1SIMO), 4.3 (UCB1CLK)
    P1DIR |= BIT3; // P1.3 for Slave Select

    UCB1CTL1 |= UCSWRST; // Reset state machine
    UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // UCMST (Master), UCSYNC (SPI), UCCKPL (Clock Polarity), UCMSB (MSB)
    UCB1CTL1 |= UCSSEL_2; // SMCLK
    UCB1BR0 = 0x02; // Set baud rate for UCB1
    UCB1BR1 = 0;
    UCA0MCTL = 0; // No modulation
    UCB1CTL1 &= ~UCSWRST; // Init state machine
    UCA0IE |= UCRXIE; // Enable RX interrupt

    __delay_cycles(1000); // Wait for slave to init
}

void clear_lcd(){
    data_wr(0xFE); // Command Prompt
    data_wr(0x51); // Clear Screen
    data_wr(0xFE); // Command Prompt
    data_wr(0x46); // Move Cursor to Start
}

void init_lcd(){
    data_wr(0xFE); // Command Prompt
    data_wr(0x42); // Display Off
    data_wr(0xFE); // Command Prompt
    data_wr(0x52); // Set Contrast
    data_wr(40); // contrast 1 - 50
    data_wr(0xFE); // Command Prompt
    data_wr(0x53); // Set Brightness
    data_wr(15); // backlight 1 - 15
    data_wr(0xFE); // Command Prompt
    data_wr(0x48); // Underline Cursor Off
    data_wr(0xFE); // Command Prompt
    data_wr(0x4B); // Blinking Cursor On
    data_wr(0xFE); // Command Prompt
    data_wr(0x46); // Move Cursor to Start
    data_wr(0xFE); // Command Prompt
    data_wr(0x41); // Display On
}

int main(void) {
    init_msp430();
    init_lcd();

    str_wr("Working");

    while(1){
        // Do Nothing
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  volatile unsigned int i;

  switch(__even_in_range(UCB1IV,4))
  {
    case 0: break;                          // Vector 0 - no interrupt
    case 2: break;                          // Vector 2 - RXIFG
    case 4: break;                          // Vector 4 - TXIFG
    default: break;
  }
}
