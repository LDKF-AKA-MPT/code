#include <msp430.h>
#include <string.h>
#include <stdio.h>

char buff[1000] = {0};
int counter = 0;     // buffer counter
int receiveflag = 0; // start to receive message flag
int storeflag = 0;   // start to store into buffer flag


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
    char* initCommand = "$PSRF100,1,9600,8,1,0*0D\r\n"; // Set Serial PORT ID: 100 Set PORTA parameters and protocol
    int i;
    int cLen = strlen(initCommand);
    for(i = 0; i < cLen; i++){
        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
        UCA0TXBUF = initCommand[i]; // set TX buffer with current command char
        __delay_cycles(100);
    }
}

//void enableDebug() {
//    char* debugCommand = "$PSRF105,1*3E\r\n"; // Query command to enable debugging
//    int qLen = strlen(debugCommand);
//    int i;
//    for(i = 0; i < qLen; i++){
//        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
//        UCA0TXBUF = debugCommand[i]; // set TX buffer with current command char
//    }
//}

void setRate(){
    char* rateCommand = "$PSRF103,01,00,01,01*24\r\n"; // Query command to set rate
    int qLen = strlen(rateCommand);
    int i;
    for(i = 0; i < qLen; i++){
        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
        UCA0TXBUF = rateCommand[i]; // set TX buffer with current command char
    }
}

void disableUnused(){
    // Disable other messages (GGA, GSA, etc)
    char* queryCommand[] = {"$PSRF103,00,00,00,01*24\r\n","$PSRF103,02,00,00,01*24\r\n","$PSRF103,03,00,00,01*24\r\n","$PSRF103,04,00,00,01*24\r\n","$PSRF103,05,00,00,01*24\r\n","$PSRF103,06,00,00,01*24\r\n","$PSRF103,08,00,00,01*24\r\n"};
    int qLen = strlen(queryCommand[0]);
    int i;
    int j;
    for (i = 0; i < 7; i++){
        for(j = 0; j < qLen; j++){
            while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
            UCA0TXBUF = queryCommand[i][j]; // set TX buffer with current command char
            __delay_cycles(10);
        }
        __delay_cycles(100);
    }

}

void sendQuery(){
    char* queryCommand = "$PSRF103,01,01,00,01*24\r\n"; // Query command for GLL message (once/sec)
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
//    enableDebug();
//    __delay_cycles(100);
    initGPS();
    __delay_cycles(100);
    setRate();
    __delay_cycles(100);
    disableUnused();
    sendQuery();
    receiveflag = 1;

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
        if (receiveflag){                     // if receiving
            temp = UCA0RXBUF;
            if (temp == '*') {               // stop storing if useful info ends
                storeflag = 0;
                buff[counter] = '\n';        // store a newline
                counter++;
            }
            if (storeflag){
                buff[counter] = temp;
                counter++;
            }
            if (temp=='$'){                   // start storing from next char if line starts
                storeflag = 1;
            }

        }
        break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
  }
}

