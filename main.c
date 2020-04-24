#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "I2Croutines.h"
#include "hdq_comm.h"
#define Pi 3.14159265359
#define charge_reset "2500.00000"

char buff[50] = {0};
int counter = 0;     // buffer counter
int receiveflag = 0; // start to receive message flag
int storeflag = 0;   // start to store into buffer flag
int firstFlag = 0; // Help with prof meyers shaky ass hands
int secondFlag = 0; // Help with prof meyers shaky ass hands
int thirdFlag = 0; // Help with prof meyers shaky ass hands
int idleFlag = 0;
int speakerFlag = 0; //flag that sets the output of the speaker on
int brightness = 0;
int count = 0; //count for speaker output
char* coords[4];
char* saved_coords[4]; //coordinates read from eeprom
float prev_latitude;
float prev_longitude;
float curr_latitude;
float curr_longitude;
float distance;
float bearing;
char* direction;
char dist[11];
int filled = 0;
int address_c = 0;
float charge =0;
float percent = 0;
char percent_str[4];
//char* latD;
//char* latM;
//char* lonD;
//char* lonM;


unsigned char read_val;
unsigned char write_val;
int button1Flag = 0;
int button2Flag = 0;



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
    //data_wr(0x01); // backlight 1 - 15
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

void init_msp430(){
    //GPS INIT
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    P3SEL |= BIT3 + BIT4; // Select P3.3 for USCI_A0 TXD and P3.4 for USCI_A0 RXD
    UCA0CTL1 |= UCSWRST; // Reset state machine
    UCSCTL4 &= ~SELS_4; // Use XT1 oscillator as SMCLK source
    //UCSCTL4 &= ~ SELM_4;
    UCSCTL4 |= SELA_4;
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 0x06; // Baud Rate to 4800
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_7 + UCBRF_0; // UCBRSx is 7 and UCBRFx is 0
    UCA0CTL1 &= ~UCSWRST; // Init state machine
    //UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts


    //LCD INIT
   // UCSCTL5 |= DIVS0 + DIVS2; // SMCLK = fSMCLK/32
   // UCSCTL5 |= DIVM0 + DIVM2; // MLCK = fMCLK/32

    P4SEL |= BIT1 + BIT2 + BIT3; // SPI pins 4.1 (UCB1SIMO), 4.2 (UCB1SOMI) 4.3 (UCB1CLK)
    P1DIR |= BIT3; // P1.3 for Slave Select

    UCB1CTL1 |= UCSWRST; // Reset state machine
    UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // UCMST (Master), UCSYNC (SPI), UCCKPL (Clock Polarity), UCMSB (Active State High)
    UCB1CTL0 &= ~(BIT1 + BIT2); // 3-pin SPI mode for UCSYNC
    UCB1CTL1 |= UCSSEL_2; // SMCLK
    UCB1BR0 = 0x02; // Set baud rate for UCB1
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST; // Init state machine

    //Button 1 Setup
    P1REN |= BIT5; // Enable P1.4/5 internal resistor
    P1OUT |= BIT5; // Set P1.4/5 as pull-Up resistor
    P1IES &= ~BIT5; // P1.4/5 Lo/Hi edge
    P1IFG &= ~BIT5; // P1.4/5 IFG cleared
    P1IE |= BIT5; // P1.4/5 interrupt enabled

    //Button 2 Setup
    P2REN |= BIT0; // Enable P2.0 internal resistor
    P2OUT |= BIT0; // Set P2.0 as pull-Up resistor
    P2IES &= ~BIT0; // P2.0 Lo/Hi edge
    P2IFG &= ~BIT0; // P2.0 IFG cleared
    P2IE |= BIT0; // P2.0 interrupt enabled

    //Button 4 setup
    P1REN |= BIT6; // Enable P2.0 internal resistor
    P1OUT |= BIT6; // Set P2.0 as pull-Up resistor
    P1IES &= ~BIT6; // P2.0 Lo/Hi edge
    P1IFG &= ~BIT6; // P2.0 IFG cleared
    P1IE |= BIT6; // P2.0 interrupt enabled

    //Button 4 setup
    P2REN |= BIT2; // Enable P2.0 internal resistor
    P2OUT |= BIT2; // Set P2.0 as pull-Up resistor
    P2IES &= ~BIT2; // P2.0 Lo/Hi edge
    P2IFG &= ~BIT2; // P2.0 IFG cleared
    P2IE |= BIT2; // P2.0 interrupt enabled

    //Directive Input
    P2REN |= BIT4;
    P2OUT |= BIT4; // Set P2.0 as pull-Up resistor
    P2IES &= ~BIT4; // P2.0 Lo/Hi edge


    //TIMER INIT
    TA1CCR0 = 65536-1;
    TA1CTL = TASSEL_2 + MC_1 + TACLR;
    //TA1CTL |= BIT7;

    //Piezo Buzzer INIT
    P1DIR &= ~BIT2;                            // P1.2 output
    P1SEL |= BIT2;                            // P1.2 options select

    //PWM OUTPUT
    TA0CCR0 = 32-1;                          // PWM Period
    TA0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TA0CCR1 = 16-1;                            // CCR1 PWM duty cycle
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, up mode, clear TAR

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

float calculateBearing(float lat1, float lat2, float deltaLon) {
    // Calculate bearing, result is radians from north to east
    // Remember to convert returned value to degrees in caller function
    return atan2(sin(deltaLon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(deltaLon));
}

float calculateDistance(float lat1, float lat2, float deltaLat, float deltaLon){
    // Calculate distance between current location and destination using Haversine formula
//    float lat1 = to_radians(currLat);            // move these 4 lines to caller function
//    float lat2 = to_radians(destLat);
//    float deltaLat = to_radians(destLat-currLat);
//    float deltaLon = to_radians(destLon-currLon);
    float a = pow(sin(deltaLat/2), 2.0) + cos(lat1)*cos(lat2)*pow(sin(deltaLon/2), 2.0);
    return 2*atan2(sqrt(a), sqrt(1-a))*6371000.0;
}

float to_radians(float degrees) {
    // Convert degrees to radians
    return degrees/180.0*Pi;
}

float convertCoord(char* coordD, char* coordM) {
    int degree =  strtol(coordD, NULL, 10);
    float minute = strtof(coordM, NULL)/60.0;
    return to_radians(degree+minute);
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
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
    //latitude = convertCoord(coords[0], coords[1]);
    //longitude = convertCoord(coords[2], coords[3]);
    filled = 1;
}
char* calc_direction(){
    char* dir = (char*)malloc(sizeof(char)*2);
    float deg;
    int degree;
    deg = bearing * 180 / Pi;
    degree = ((int)(deg) + 360) % 360;
    if (degree > 330 || degree <= 30){
        dir[0] = ' ';
        dir[1] = 'N';
    }
    else if (degree > 30 && degree <= 60){
        dir[0] = 'N';
        dir[1] = 'E';
    }
    else if (degree > 60 && degree <= 120){
        dir[0] = ' ';
        dir[1] = 'E';
    }
    else if (degree > 120 && degree <= 150){
        dir[0] = 'S';
        dir[1] = 'E';
    }
    else if (degree > 150 && degree <= 210){
        dir[0] = ' ';
        dir[1] = 'S';
    }
    else if (degree > 210 && degree <= 240){
        dir[0] = 'S';
        dir[1] = 'W';
    }
    else if (degree > 240 && degree <= 300){
        dir[0] = ' ';
        dir[1] = 'W';
    }
    else if (degree > 300 && degree <= 330){
        dir[0] = 'N';
        dir[1] = 'W';
    }
    return dir;
}
void store_coords()
{

    int i;
    int address = 0;
    for(i = 0; i < strlen(coords[0]) ; i++)
    {
        EEPROM_ByteWrite(address,coords[0][i]);
        EEPROM_AckPolling();
        address += 1;
    }
    for(i = 0; i < strlen(coords[1]) ; i++)
        {
            EEPROM_ByteWrite(address,coords[1][i]);
            EEPROM_AckPolling();
            address += 1;
        }
    for(i = 0; i < strlen(coords[2]) ; i++)
        {
            EEPROM_ByteWrite(address,coords[2][i]);
            EEPROM_AckPolling();
            address += 1;
        }
    for(i = 0; i < strlen(coords[3]) ; i++)
        {
            EEPROM_ByteWrite(address,coords[3][i]);
            EEPROM_AckPolling();
            address += 1;
        }
    free(coords[0]);
    free(coords[1]);
    free(coords[2]);
    free(coords[3]);
    TA1CCTL0 &= ~CCIE; //disable timer interrupt after storing to eeprom
}

void disable_timer()
{
    TA1CCTL0 &= ~CCIE; //disable timer interrupt after reading from the eeprom
}

void read_coords()
{
    int address = 0;
    int i;
    saved_coords[0] = (char*)malloc(sizeof(char)*4);
    for(i = 0; i < 4; i++)
       {
           read_val = EEPROM_RandomRead(address);
           saved_coords[0][i] = read_val;
           address += 1;
       }
    saved_coords[0][3] = '\0';
    saved_coords[1] = (char*)malloc(sizeof(char)*9);
    address -= 1;
    for(i = 0; i < 9; i++)
       {
           read_val = EEPROM_RandomRead(address);
           saved_coords[1][i] = read_val;
           address += 1;
       }
    saved_coords[1][8] = '\0';
    address -= 1;
    saved_coords[2] = (char*)malloc(sizeof(char)*5);
    for(i = 0; i < 5; i++)
       {
           read_val = EEPROM_RandomRead(address);
           saved_coords[2][i] = read_val;
           address += 1;
       }
    saved_coords[2][4] = '\0';
    address -= 1;
    saved_coords[3] = (char*)malloc(sizeof(char)*9);
    for(i = 0; i < 9; i++)
       {
           read_val = EEPROM_RandomRead(address);
           saved_coords[3][i] = read_val;
           address += 1;
       }
    saved_coords[3][8] = '\0';

}



int main(void)
{
    InitI2C(0x50); // Initialize I2C module
    init_msp430();
    //_BIS_SR(GIE);
    hdq_init();
    //__delay_cycles(10000000);
    hdq_send(0x74,0x09);
    init_lcd();
    clear_lcd();
    str_wr("   Welcome to   ");
    new_line();
    str_wr("      MPT      ");
    __delay_cycles(1000000);
    initGPS();
    __delay_cycles(1000);
    setRate();
    __delay_cycles(1000);
    disableUnused();
    sendQuery();
    clear_lcd();
    str_wr("Wait for fixed");
    new_line();
    str_wr("GPS location!");
    _delay_cycles(1000000);
    while(!(P2IN & BIT4)); // wait for gps to fix coords
    clear_lcd();
    str_wr("GPS FIXED!");
    __delay_cycles(1000000);
    clear_lcd();
    str_wr("   Welcome to   ");
    new_line();
    str_wr("      MPT      ");
    //uint8_t count_high = hdq_rec(0x79);
    //uint8_t count_low = hdq_rec(0x78);
    //uint8_t data = hdq_rec(0x7E);
    //receiveflag = 1;

/*
    //Store To EEPROM Coord Arrays
    if(button1Flag)
    {
        store_coords();
        button1Flag = 0;
    }
    //Read and Display Cooord Data
    if(button2Flag)
    {
        read_disp_coords();
        button2Flag = 0;
    }
*/
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
    TA1CCTL0 &= ~CCIE;                //disable timer interrupt - CCR0
    P1IE &= ~BIT5;                    // P1.5 interrupt disabled
    P2IE &= ~BIT0;                    //P2.0 interrupt disabled
    P1IFG &= ~BIT5;                     //clear flag
    P2IFG &= ~BIT0;                     //clear flag

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
                P1IE |= BIT5;                    // P1.5 interrupt enabled
                P2IE |= BIT0;                    //P2.0 interrupt enabled
                UCA0IE &= ~UCRXIE;           // disable USCI_A0 RX interrupts
                TA1CCTL0 = CCIE; // CCR0 interrupt enabled

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

// PB1.5 interrupt PB1
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void){
    disable_timer();
    counter = 0;
    switch( __even_in_range( P1IV, P1IV_P1IFG7 )) {
    case P1IV_P1IFG5:
        idleFlag = 0;
        secondFlag = 0;
        thirdFlag = 0;
        speakerFlag = 0;
        P1DIR &= ~BIT2;
        while(!(P1IN & BIT5)); // Wait for button to be unpressed
        if (firstFlag){
            firstFlag = 0;
            clear_lcd();
            str_wr("Saving Coords...");
            counter = 0; //set buffer counter to 0
            UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts
            receiveflag = 1;
            button1Flag = 1; //write to eeprom
        } else if(firstFlag == 0 && button1Flag == 0) {
            clear_lcd();
            str_wr("Save current");
            new_line();
            str_wr("coordinates?");
            firstFlag = 1;
        }
        break;
    case P1IV_P1IFG6:
       //reset battery
        P1IFG &= ~BIT5; // P2.0 IFG cleared
        P1IE &= ~BIT5;
        idleFlag = 0;
        secondFlag = 0;
        firstFlag = 0;
        speakerFlag = 0;
        P1DIR &= ~BIT2;
        disable_timer();
        while(!(P1IN & BIT6)); // Wait for button to be unpressed
        if(thirdFlag)
        {
            thirdFlag = 0;
            P1IE &= ~BIT6;
            address_c = 69;
            int i = 0;
            for(i = 0; i < strlen(charge_reset); i++){
               EEPROM_ByteWrite(address_c,charge_reset[i]);
               EEPROM_AckPolling();
               address_c += 1;
            }
            EEPROM_ByteWrite(0x420, 0);
            EEPROM_AckPolling();
            EEPROM_ByteWrite(0x421, 0);
            EEPROM_AckPolling();
            EEPROM_ByteWrite(0x422, 0);
            EEPROM_AckPolling();
            EEPROM_ByteWrite(0x423, 0);
            EEPROM_AckPolling();
            P1IE |= BIT6;
            clear_lcd();
            str_wr("Battery reset!");
        }
        else if(thirdFlag == 0){
            clear_lcd();
            str_wr("Reset battery?");
            thirdFlag = 1;
        }
        break;
    default:   _never_executed();
    }
    __delay_cycles(1000000);
    P1IFG &= ~BIT5; // P2.0 IFG cleared
    P1IFG &= ~BIT6; // P2.0 IFG cleared
    P1IE |= BIT5;
}

// P2.0 interrupt PB2
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void){
    disable_timer();
    counter = 0;
    switch( __even_in_range( P2IV, P2IV_P2IFG7 )) {
    case P2IV_P2IFG0:
        P2IE &= ~BIT0;
        P2IFG &= ~BIT0;
        P2IE &= ~BIT2;
        P2IFG &= ~BIT2;
        idleFlag = 0;
        firstFlag = 0;
        thirdFlag = 0;
        speakerFlag = 0;
        P1DIR &= ~BIT2;
        while(!(P2IN & BIT0)); // Wait for button to be unpressed
        if (secondFlag){
            secondFlag = 0;
            button2Flag = 1; //read from eeprom
            clear_lcd();
            str_wr("Tracking...");
            read_coords();
            counter = 0; //set buffer counter to 0
            UCA0IE |= UCRXIE; // Enable USCI_A0 RX interrupts
            receiveflag = 1; //start recieving coordinates from GPS
        } else if(secondFlag == 0 && button2Flag == 0) {
            clear_lcd();
            str_wr("Track previous");
            new_line();
            str_wr("location?");
            secondFlag = 1;
        }
        break;
    case P2IV_P2IFG2:
        while(!(P2IN & BIT2)); // Wait for button to be unpressed
        counter = 0;
        P2IE &= ~BIT2;
        P2IFG &= ~BIT2;
        P2IE &= ~BIT0;
        P2IFG &= ~BIT0;
        disable_timer();
        P2IFG &= ~BIT2;
        clear_lcd();
        if(speakerFlag){
            P1DIR &= ~BIT2;
            speakerFlag = 0;
            str_wr("Tracking ended!");
            __delay_cycles(1000000);
        }

        idleFlag = 1;
        __delay_cycles(1000000);
        charge = cc_update();
        TA1CCR0 = 65536-1;
        TA1CCTL0 |= CCIE;
        count = 0;
        speakerFlag = 0;
        secondFlag = 0;
        firstFlag = 0;
        thirdFlag = 0;
        button1Flag = 0;
        button2Flag = 0;
        clear_lcd();
        str_wr("   Welcome to   ");
        new_line();
        str_wr("      MPT      ");

        break;
    default:   _never_executed();
    }
    __delay_cycles(1000000);
    //P2IFG &= ~BIT0; // P2.0 IFG cleared
    //P2IFG &= ~BIT2; // P2.0 IFG cleared
    P2IFG &= ~BIT0;
    P2IE |= BIT0;
    P2IE |= BIT2;
    P2IFG &= ~BIT2;
}

//TIMER INTERUPT
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  counter = 0;

  if(filled && button1Flag){
    button1Flag = 0;
    speakerFlag = 0;
    filled = 0;
    if(buff[0] == '\x00'){
        clear_lcd();
        str_wr("Please try");
        new_line();
        str_wr("saving again!");
    }
    else{
        store_coords();
        clear_lcd();
        str_wr("Location Saved!");

    }

  }
  else if(filled && button2Flag)
  {
     disable_timer();
     button2Flag = 0;
     speakerFlag = 0;
     filled = 0;
     prev_latitude = convertCoord(saved_coords[0], saved_coords[1]);
     prev_longitude = convertCoord(saved_coords[2], saved_coords[3]);
     curr_latitude = convertCoord(coords[0], coords[1]);
     curr_longitude = convertCoord(coords[2], coords[3]);
     distance = calculateDistance( curr_latitude, prev_latitude, (prev_latitude - curr_latitude),(prev_longitude - curr_longitude));
     bearing = calculateBearing(curr_latitude, prev_latitude, (prev_longitude - curr_longitude));
     clear_lcd();
     str_wr("DIST:");
     ftoa(distance, dist, 2);
     str_wr(dist);
     str_wr(" m");
     new_line();
     str_wr("DIR:");
     direction = calc_direction();
     str_wr(direction);
     //str_wr(" BatLife:H");
     free(saved_coords[0]);
     free(saved_coords[1]);
     free(saved_coords[2]);
     free(saved_coords[3]);
     free(coords[0]);
     free(coords[1]);
     free(coords[2]);
     free(coords[3]);
     free(direction);
     //disable_timer();
     button2Flag = 0;
     speakerFlag = 1;
     count = 9;
     TA1CCTL0 = CCIE; // CCR0 interrupt enabled


  }
  else if(speakerFlag){
      //distance = 60; //need to remove later
      if (count >= 9) {
              if (P1DIR & BIT2) {
                  P1DIR &= ~BIT2;      //turn off output
              }
              else {
                  P1DIR |= BIT2;      //turn on output
              }
              count = 0;
              if (distance <= 5000) {        //change frequency according to distance
                  TA1CCR0 = 65536 - 1;
              }
              if (distance <= 1000) {
                  TA1CCR0 = 32768 - 1;
              }
              if (distance <= 500){
                  TA1CCR0 = 16384 - 1;
              }
              if(distance <= 250){
                  TA1CCR0 = 8192 - 1;
              }
              if (distance <= 150){
                  TA1CCR0 = 4096 - 1;
              }
              if (distance <= 100){
                  TA1CCR0 = 2048 - 1;
              }
              if (distance <= 50){
                  TA1CCR0 = 1024- 1;
              }
              if(distance <= 10){
                  TA1CCR0 = 512 - 1;
              }
              if(distance <= 5){
                  P1DIR |= BIT2;
                  disable_timer();
              }
          }
          count++;
      }
  else if(idleFlag){
      if(count >= 40){
          count = 0;
          charge = cc_update();
          percent = charge / 2500 * 100;
          ftoa(percent, percent_str, 1);
          if(charge > 1250){
              clear_lcd();
              str_wr("BatteryLife:");
              new_line();
              str_wr("HIGH ");

          }
          if(charge <= 1250){
              clear_lcd();
              str_wr("BatteryLife:");
              new_line();
              str_wr("MEDIUM ");

            }
          if(charge <=500){
              clear_lcd();
              str_wr("BatteryLife:");
              new_line();
              str_wr("LOW ");
          }
          str_wr(percent_str);
          str_wr("%");
      }
      count++;
  }

}
