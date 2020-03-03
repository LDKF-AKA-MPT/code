#include <msp430.h> 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned char I2C_READ_BYTE;
unsigned char I2C_WRITE_BUFFER[8];
int I2C_WRITE_COUNT;

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

void init_msp430(){
    WDTCTL = WDTPW + WDTHOLD;

    // SPI Init for LCD
    P4SEL |= BIT1 + BIT2 + BIT3; // SPI pins 4.1 (UCB1SIMO), 4.2 (UCB1SOMI) 4.3 (UCB1CLK)
    P1DIR |= BIT3; // P1.3 for Slave Select

    UCSCTL2 |= FLLD1 + FLLD0; // Set clock divided by 8

    UCB1CTL1 |= UCSWRST; // Reset state machine
    UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // UCMST (Master), UCSYNC (SPI), UCCKPL (Clock Polarity), UCMSB (Active State High)
    UCB1CTL0 &= ~(BIT1 + BIT2); // 3-pin SPI mode for UCSYNC
    UCB1CTL1 |= UCSSEL_2; // SMCLK
    UCB1BR0 = 0x02; // Set baud rate for UCB1
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST; // Init state machine

    // I2C Init for EEPROM
    P3DIR |= BIT0 + BIT1; // Assign I2C pins to USCI_B0
    P3SEL |= BIT0 + BIT1; // I2C Pins p3.0 (SDA), p3.1 (SCL)

    UCB0CTL1 |= UCSWRST; // Reset State Machine
    UCB0CTL0 = UCMST + UCSYNC + UCMODE_3; // UCMST (Master), UCSYNC (I2C), UCMODE_3 (Synchronous mode)
    UCB1CTL1 = UCSSEL_2 + UCTR + UCSWRST; // SMCLK and stay in reset
    UCB0BR0 = 12; // fSCL = SMCLK/12 = 100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = 0x48; // Slave Address
    UCB0CTL1 &= ~UCSWRST; // Init State Machine

    UCB0IE |= UCRXIE + UCTXIE; // RX and TX interrupts enabled
}

void EEPROM_acknownledge(){
    while (UCBUSY & UCB0STAT){} // Wait for I2C to be ready

    do{
        UCB0STAT = 0; // Clear I2C interrupts
        UCB0CTL1 |= UCTR; // Set transmission flag
        UCB0CTL1 &= ~UCTXSTT; // Make sure the transmission start flag is not set
        UCB0CTL1 |= UCTXSTT; // Set the transmit start flag
        while (UCB0CTL1 & UCTXSTT){
            if (!(UCNACKIFG & UCB0STAT)){
                break; // ACK is received from EEPROM
            }
        }
        UCB0CTL1 |= UCTXSTP; // Send transmit stop condition
        __delay_cycles(100); // Wait and try for ACK again
    } while(UCB0STAT & UCNACKIFG); // Continue while NAK flag is set
}

unsigned char EEPROM_read(unsigned int data_addr){
    while (UCBUSY & UCB0STAT){} // Wait for I2C to be ready
    I2C_WRITE_BUFFER[0] = data_addr & 0xFF; // Store low byte of data address
    I2C_WRITE_BUFFER[1] = data_addr >> 8; // Store high byte of data address
    I2C_WRITE_COUNT = 2;

    // Write address to read from to EEPROM
    UCB0CTL1 |= UCTR; // I2C Transmission mode
    UCB0IFG &= ~UCTXIFG; // Clear transmission interrupt flag
    UCB0IE &= ~UCRXIE; // Disable read interrupts
    UCB0IE |= UCTXIE; // Enable write interrupts
    UCB0CTL1 &= ~UCTXSTP & ~UCTXSTT; // Clear stop and start bits
    UCB0CTL1 |= UCTR + UCTXSTT; // Send start condition and set transmit flag
    while(UCB0CTL1 & UCTXSTP){} // Wait for stop condition

    // Send read command
    UCB0CTL1 &= ~UCTR; // I2C receive mode
    UCB0IFG &= ~UCRXIFG & ~UCTXIFG; // Clear receive and transmit interrupt flag
    UCB0IE &= ~UCTXIE; // Disable write interrupts
    UCB0IE |= UCRXIE; // Enable read interrupts

    UCB0CTL1 &= ~UCTXSTP & ~UCTXSTT; // Clear stop and start bits
    UCB0CTL1 |= UCTXSTT; // Send start condition and set transmit flag
    while(UCB0CTL1 & UCTXSTT){} // Wait for start condition to be sent
    UCB0CTL1 |= UCTXSTP; // Send stop condition
    while(UCB0CTL1 & UCTXSTP){} // Wait for stop condition

    return I2C_READ_BYTE; // return byte read from the EEPROM as a char
}

void EEPROM_write(unsigned char data, unsigned int data_addr){
    while (UCBUSY & UCB0STAT){} // Wait for I2C to be ready

    I2C_WRITE_BUFFER[0] = data; // Store data to be sent
    I2C_WRITE_BUFFER[1] = data_addr & 0xFF; // Store low byte of data address
    I2C_WRITE_BUFFER[2] = data_addr >> 8; // Store high byte of data address
    I2C_WRITE_COUNT = 3;

    // Write address and byte of information to EEPROM
    UCB0CTL1 |= UCTR; // I2C Transmission mode
    UCB0IFG &= ~UCTXIFG; // Clear transmission interrupt flag
    UCB0IE &= ~UCRXIE; // Disable read interrupts
    UCB0IE |= UCTXIE; // Enable write interrupts
    UCB0CTL1 &= ~UCTXSTP & ~UCTXSTT; // Clear stop and start bits
    UCB0CTL1 |= UCTR + UCTXSTT; // Send start condition and set transmit flag
    while(UCB0CTL1 & UCTXSTP){} // Wait for stop condition
}

int main(void){
    // Declare variables
	//unsigned int curr_addr = 0x0000;
	uint32_t x_coord = 32;
	uint32_t y_coord = 12;
	uint32_t new_x_coord;
    uint32_t new_y_coord;
    char x_coord_str[10];
    char y_coord_str[10];
    unsigned char curr_byte = 0;

    // Initalize
    init_msp430();
    init_lcd();
    clear_lcd();

	// Send x coordinant to be stored in EEPROM
    curr_byte = x_coord >> 24;
    EEPROM_write(curr_byte, 0x0000);
    EEPROM_acknownledge();
    curr_byte = x_coord >> 16;
    EEPROM_write(curr_byte, 0x0001);
    EEPROM_acknownledge();
    curr_byte = x_coord >> 8;
    EEPROM_write(curr_byte, 0x0002);
    EEPROM_acknownledge();
    curr_byte = x_coord >> 0;
    EEPROM_write(curr_byte, 0x0003);
    EEPROM_acknownledge();

	// Send y coordinant to be stored in EEPROM
    curr_byte = y_coord >> 24;
    EEPROM_write(curr_byte, 0x0004);
    EEPROM_acknownledge();
    curr_byte = y_coord >> 16;
    EEPROM_write(curr_byte, 0x0005);
    EEPROM_acknownledge();
    curr_byte = y_coord >> 8;
    EEPROM_write(curr_byte, 0x0006);
    EEPROM_acknownledge();
    curr_byte = y_coord >> 0;
    EEPROM_write(curr_byte, 0x0007);
    EEPROM_acknownledge();

	// Read x and y coordinants from the EEPROM
	//new_x_coord = (EEPROM_read(0x0000) << 24) | (EEPROM_read(0x0001) << 16) | (EEPROM_read(0x0002) << 8) | EEPROM_read(0x0003);
	new_x_coord = (EEPROM_read(0x0000) << 24);
	new_x_coord |= (EEPROM_read(0x0001) << 16);
	new_x_coord |= (EEPROM_read(0x0002) << 8);
	new_x_coord |= EEPROM_read(0x0003);

	//new_y_coord = (EEPROM_read(0x0004) << 24) | (EEPROM_read(0x0005) << 16) | (EEPROM_read(0x0006) << 8) | EEPROM_read(0x0007);
	new_y_coord = (EEPROM_read(0x0004) << 24);
    new_y_coord |= (EEPROM_read(0x0005) << 16);
    new_y_coord |= (EEPROM_read(0x0006) << 8);
    new_y_coord |= EEPROM_read(0x0007);


	sprintf(x_coord_str, "&#37;u", new_x_coord);
	sprintf(y_coord_str, "&#37;u", new_y_coord);
	str_wr(x_coord_str);
	str_wr("+");
	str_wr(y_coord_str);

	return 0;
}

// USCI_B0 Data ISR
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void){
    // Decide if transmit or receive interrupt flag is set
    if (UCB0IFG & UCTXIFG){
        // Transmit data
        UCB0TXBUF = I2C_WRITE_BUFFER[I2C_WRITE_COUNT-1];
        I2C_WRITE_COUNT--;
        if (I2C_WRITE_COUNT <= 0){
            while (!(UCB0IFG & UCTXIFG)){} // Wait for transmission to be complete
            UCB0CTL1 |= UCTXSTP; // Send stop condition
            UCB0CTL1 &= ~UCTXSTT; // Clear start bit
            UCB0IE &= ~UCTXIE; // Disable transmission interrupt
            UCB0IFG &= ~UCTXIFG; // Clear transmit interrupt flag
        }
    } else if (UCB0IFG & UCRXIFG){
        // Receive data
        I2C_READ_BYTE = UCB0RXBUF; // Recieve byte from EEPROM and store it
        //UCB0CTL1 |= UCTXSTP;
        UCB0CTL1 &= ~UCTXSTT;
        UCB0IE &= ~UCRXIE; // Disable transmission interrupt
        UCB0IFG &= ~UCRXIFG; // Clear transmit interrupt flag
    }
}

// USCI_B1 Data ISR
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void){
  volatile unsigned int i;

  switch(__even_in_range(UCB1IV,4))
  {
    case 0: break;                          // Vector 0 - no interrupt
    case 2: break;                          // Vector 2 - RXIFG
    case 4: break;                          // Vector 4 - TXIFG
    default: break;
  }
}
