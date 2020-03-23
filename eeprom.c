#include <msp430.h>
#include "I2Croutines.h"

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

void init_msp430(){
    WDTCTL = WDTPW + WDTHOLD;
    __bis_SR_register( GIE ); // Let interupts be global

    //UCSCTL2 |= FLLD1 + FLLD0; // Set clock divided by 8
    // Set Clocks
    UCSCTL5 |= DIVS0 + DIVS2; // SMCLK = fSMCLK/32
    UCSCTL5 |= DIVM0 + DIVM2; // MLCK = fMCLK/32

    P4SEL |= BIT1 + BIT2 + BIT3; // SPI pins 4.1 (UCB1SIMO), 4.2 (UCB1SOMI) 4.3 (UCB1CLK)
    P1DIR |= BIT3; // P1.3 for Slave Select

    UCB1CTL1 |= UCSWRST; // Reset state machine
    UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // UCMST (Master), UCSYNC (SPI), UCCKPL (Clock Polarity), UCMSB (Active State High)
    UCB1CTL0 &= ~(BIT1 + BIT2); // 3-pin SPI mode for UCSYNC
    UCB1CTL1 |= UCSSEL_2; // SMCLK
    UCB1BR0 = 0x02; // Set baud rate for UCB1
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST; // Init state machine

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

    //__bis_SR_register(LPM0_bits+GIE);             // Enter LPM0
    //__no_operation();                         // For debugger
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

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    InitI2C(0x50); // Initialize I2C module
    init_msp430();
    init_lcd();
    clear_lcd();
    //data_wr(0xFE);
    //data_wr(0x71); // Baud rate display
    str_wr("Start Of Program");
    write_val = 0x30;

    while(1){
        if (button1Flag){
            EEPROM_ByteWrite(0x0000,write_val);
            EEPROM_AckPolling();
            clear_lcd();
            str_wr("Data written to");
            new_line();
            str_wr("EEPROM: ");
            data_wr(write_val);
            write_val++;
            if (write_val == ':'){
                write_val = 0x30;
            }
            button1Flag = 0;
        }
        if (button2Flag){
            read_val = EEPROM_RandomRead(0x0000);
            clear_lcd();
            str_wr("Data read from");
            new_line();
            str_wr("EEPROM: ");
            data_wr(read_val);
            button2Flag = 0;
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
        //while(P1OUT & BIT1); // Wait for button to be unpressed
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
