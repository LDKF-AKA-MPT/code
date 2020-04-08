#include <msp430.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
// http://www.ti.com/lit/zip/slaa208
#include "I2Croutines.h"

unsigned char read_val;
unsigned char write_val;
int button1Flag = 0;
int button2Flag = 0;
int button3Flag = 0;
char buff[1000] = {0};
int counter = 0;     // buffer counter
int receiveflag = 0; // start to receive message flag
int storeflag = 0;   // start to store into buffer flag

void init_msp430(){
    WDTCTL = WDTPW + WDTHOLD;
    __bis_SR_register( GIE ); // Let interupts be global

    //UCSCTL2 |= FLLD1 + FLLD0; // Set clock divided by 8
    // Set Clocks
    UCSCTL5 |= DIVS0 + DIVS2; // SMCLK = fSMCLK/32
    UCSCTL5 |= DIVM0 + DIVM2; // MLCK = fMCLK/32

	// LCD init
    P4SEL |= BIT1 + BIT2 + BIT3; // SPI pins 4.1 (UCB1SIMO), 4.2 (UCB1SOMI) 4.3 (UCB1CLK)
    P1DIR |= BIT3; // P1.3 for Slave Select
    UCB1CTL1 |= UCSWRST; // Reset state machine
    UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // UCMST (Master), UCSYNC (SPI), UCCKPL (Clock Polarity), UCMSB (Active State High)
    UCB1CTL0 &= ~(BIT1 + BIT2); // 3-pin SPI mode for UCSYNC
    UCB1CTL1 |= UCSSEL_2; // SMCLK
    UCB1BR0 = 0x02; // Set baud rate for UCB1
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST; // Init state machine
	
	// GPS init
	P3SEL |= BIT3 + BIT4; // Select P3.3 for USCI_A0 TXD and P3.4 for USCI_A0 RXD
    UCA0CTL1 |= UCSWRST; // Reset state machine
    UCSCTL4 &= !SELS_4; // Use XT1 oscillator as SMCLK source
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 0x06; // Baud Rate to 4800
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_7 + UCBRF_0; // UCBRSx is 7 and UCBRFx is 0
    UCA0CTL1 &= ~UCSWRST; // Init state machine
    UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts
	
	// Button 1 init
    P1REN |= BIT1; // Enable P1.1 internal resistor
    P1OUT |= BIT1; // Set P1.1 as pull-Up resistor
    P1IES &= ~BIT1; // P1.1 Lo/Hi edge
    P1IFG &= ~BIT1; // P1.1 IFG cleared
    P1IE |= BIT1; // P1.1 interrupt enabled

	// Button 2 init
    P2REN |= BIT1; // Enable P2.1 internal resistor
    P2OUT |= BIT1; // Set P2.1 as pull-Up resistor
    P2IES &= ~BIT1; // P2.1 Lo/Hi edge
    P2IFG &= ~BIT1; // P2.1 IFG cleared
    P2IE |= BIT1; // P2.1 interrupt enabled
	
	// Speaker init
	TA0CCR0 = 512-1;                          // PWM Period
    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA0CCR1 = 256;                            // CCR1 PWM duty cycle
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR
    TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
    TA1CCR0 = 65536-1;
    TA1CTL = TASSEL_2 + MC_1 + TACLR;

    //__bis_SR_register(LPM0_bits+GIE);             // Enter LPM0
    //__no_operation();                         // For debugger
}

double to_radians(double degrees) {
    // Convert degrees to radians
    return degrees/180.0*3.14159265359;
}

double calculateDistance(double lat1, double lat2, double deltaLat, double deltaLon){
    // Calculate distance between current location and destination using Haversine formula
//    double lat1 = to_radians(currLat);            // move these 4 lines to caller function
//    double lat2 = to_radians(destLat);
//    double deltaLat = to_radians(destLat-currLat);
//    double deltaLon = to_radians(destLon-currLon);
    double a = pow(sin(deltaLat/2), 2.0) + cos(lat1)*cos(lat2)*pow(sin(deltaLon/2), 2.0);
    return 2*atan2(sqrt(a), sqrt(1-a))*6371000.0;
}

double calculateBearing(double lat1, double lat2, double deltaLon) {
    // Calculate bearing, result is radians from north to east
    // Remember to convert returned value to degrees in caller function
    return atan2(sin(deltaLon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(deltaLon));
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

void data_wr(unsigned char data){
    P1OUT &= ~BIT3; //SS = 0;
    while(!(UCB1IFG&UCTXIFG)); // Wait for TX buffer ready
    UCB1TXBUF = data; // Fill and send buffer
    __delay_cycles(100); // Wait for buffer to send
    // Pulse clock, Data is sent on rising edge
    P4OUT &= ~BIT3; //SCL = 0;
    __delay_cycles(500);
    P4OUT |= BIT3; //SCL = 1;
    P1OUT |= BIT3; //SS = 1;
}

void str_wr(char* data){
    int i;
    int strLen = strlen(data);
    for(i = 0; i < strLen; i++){
        data_wr(data[i]);
    }
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
    data_wr(0x28); // contrast 1 - 50
    //data_wr(0xFE); // Command Prompt
    //data_wr(0x53); // Set Brightness
    //data_wr(0x0F); // backlight 1 - 15
    data_wr(0xFE); // Command Prompt
    data_wr(0x48); // Underline Cursor Off
    data_wr(0xFE); // Command Prompt
    data_wr(0x4B); // Blinking Cursor On
    data_wr(0xFE); // Command Prompt
    data_wr(0x46); // Move Cursor to Start
    data_wr(0xFE); // Command Prompt
    data_wr(0x41); // Display On
}

void new_line(){
    data_wr(0xFE);
    data_wr(0x45);
    data_wr(0x40);
}

void main (void){
    while(1){
		// Save Button - Saves current coodinates
        if (button1Flag){
            
			button1Flag = 0;
        }
		// Track Button - Loads saved location and tracks back to it from current location
        if (button2Flag){
			
			button2Flag = 0;
        }
		// Battery Reset Button - Resets battery state
		if (button3Flag) {
			
			button3Flag = 0;
		}
    }
}

// P1.1 interrupt
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG1:
        button1Flag = 1;
        break;
    default:   _never_executed();
    }
}

// P2.1 interrupt
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void){
    switch( __even_in_range( P2IV, P2IV_P2IFG7 )) {
    case P2IV_P2IFG1:
        button2Flag = 1;
        break;
    default:   _never_executed();
    }
}

// Timer0 A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)

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

// UART interrupt for GPS
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void){
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