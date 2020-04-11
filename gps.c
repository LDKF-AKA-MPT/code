#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define Pi 3.14159265359

char buff[50] = {0};
int counter = 0;     // buffer counter
int receiveflag = 0; // start to receive message flag
int storeflag = 0;   // start to store into buffer flag
char* coords[4];
float latitude;
float longitude;

void init_msp430(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    P3SEL |= BIT3 + BIT4; // Select P3.3 for USCI_A0 TXD and P3.4 for USCI_A0 RXD
    UCA0CTL1 |= UCSWRST; // Reset state machine
    UCSCTL4 &= !SELS_4; // Use XT1 oscillator as SMCLK source
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 0x06; // Baud Rate to 4800
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_7 + UCBRF_0; // UCBRSx is 7 and UCBRFx is 0
    UCA0CTL1 &= ~UCSWRST; // Init state machine
    UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts
}

void initGPS(){
    char* initCommand = "$PSRF100,1,4800,8,1,0*0E\r\n"; // Set Serial PORT ID: 100 Set PORTA parameters and protocol
    int i;
    int cLen = strlen(initCommand);
    for(i = 0; i < cLen; i++){
        while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
        UCA0TXBUF = initCommand[i]; // set TX buffer with current command char
        __delay_cycles(100);
    }
}


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
    char* queryCommand[] = {"$PSRF103,00,00,00,01*24\r\n","$PSRF103,02,00,00,01*26\r\n","$PSRF103,03,00,00,01*27\r\n","$PSRF103,04,00,00,01*20\r\n","$PSRF103,05,00,00,01*21\r\n","$PSRF103,06,00,00,01*22\r\n","$PSRF103,08,00,00,01*2C\r\n"};
    int qLen = strlen(queryCommand[0]);
    int i;
    int j;
    for (i = 0; i < 7; i++){
        for(j = 0; j < qLen; j++){
            while (!(UCA0IFG&UCTXIFG)); // Wait for USCI_A0 TX buffer ready
            UCA0TXBUF = queryCommand[i][j]; // set TX buffer with current command char
            __delay_cycles(10);
        }
        __delay_cycles(1000);
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

float convertCoord(char* coordD, char* coordM) {
    int degree =  strtol(coordD, NULL, 10);
    float minute = strtof(coordM, NULL)/60.0;
    return degree+minute;
}

void getCoords() {
    receiveflag = 0;    // end receiving

    char* latD = (char*)malloc(sizeof(char)*4);   // strings for lat degrees and minute
    char* latM = (char*)malloc(sizeof(char)*9);
    char* lonD = (char*)malloc(sizeof(char)*5);    // strings for lon degrees and minute
    char* lonM = (char*)malloc(sizeof(char)*9);

    unsigned int lati = 1;   // counters
    unsigned int loni = 1;
    unsigned int i;

    latD[3] = '\0';
    latM[8] = '\0';
    lonD[4] = '\0';
    lonM[8] = '\0';
    
    latD[1] = buff[6];      // store latD
    latD[2] = buff[7];

    for (i = 8; i < 15; i++) {   // store latM
        latM[lati] = buff[i];
        lati++;
    }
    if (buff[16] == 'S') {  //determine sign
        latD[0] = '-';
        latM[0] = '-';
    }
    else {
        latD[0] = '+';
        latM[0] = '+';
    }
    coords[0] = latD;
    coords[1] = latM;

    lonD[1] = buff[18];     // store lonD
    lonD[2] = buff[19];
    lonD[3] = buff[20];

    for (i = 21; i < 28; i++) {   // store lonM
        lonM[loni] = buff[i];
        loni++;
    }
    if (buff[29] == 'W') {  // determine sign
        lonD[0] = '-';
        lonM[0] = '-';
    }
    else {
        lonD[0] = '+';
        lonM[0] = '+';
    }
    coords[2] = lonD;
    coords[3] = lonM;

    latitude = convertCoord(coords[0], coords[1]);
    longitude = convertCoord(coords[2], coords[3]);
}

int main(void)
{
    init_msp430();

    initGPS();
    __delay_cycles(100);
    setRate();
    __delay_cycles(100);
    disableUnused();
    sendQuery();
    while (!(P2IN & 0x01)); // wait for gps to fix coords
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
                counter = 0;
                getCoords();
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

