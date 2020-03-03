#include <msp430.h>
#include <string.h>
#include <stdio.h>

void init_msp430(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    P3SEL |= BIT3 + BIT4; // Select P3.3 for USCI_A0 TXD and P3.4 for USCI_A0 RXD
    UCA0CTL1 |= UCSWRST; // Reset state machine
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 0x6D; // Baud Rate to 9600
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_2 + UCBRF_0; // UCBRSx is 1 and UCBRFx is 0 for modulation pattern 01000000
    UCA0CTL1 &= ~UCSWRST; // Init state machine
    UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts


}

void initGPS(){
    char* initCommand = "$PSRF100,1,9600,8,1,0*5D\r\n"; // Set Serial PORT ID: 100 Set PORTA parameters and protocol
    int i;
    int cLen = strlen(initCommand);
    for(i = 0; i < cLen; i++){
        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
        UCA0TXBUF = initCommand[i]; // set TX buffer with current command char
        __delay_cycles(100);
    }
    //char* queryCommand = "$PSRF103,01,01,00,01*24"; // Query command to send one set of coordinants

}

void sendQuery(){
    char* queryCommand = "$PSRF103,01,01,01,01*25\r\n"; // Query command to send one set of coordinants
    int qLen = strlen(queryCommand);
    int i;
    for(i = 0; i < qLen; i++){
        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
        UCA0TXBUF = queryCommand[i]; // set TX buffer with current command char
        __delay_cycles(10);
    }
}

int main(void)
{
    init_msp430();
    initGPS();
//    while(1){
//        if (P2IN & BIT0) {
            sendQuery();
            __delay_cycles(1000);
//        }
//    }
    __bis_SR_register(LPM0_bits + GIE); // Enter low power mode, interrupts enabled
    __no_operation(); // For debugger
}


// UART interrupt for GPS
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    char temp;
    switch(__even_in_range(UCA0IV,4))
    {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
        temp = UCA0RXBUF;                  // TX -> RXed character
        break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
  }
}

