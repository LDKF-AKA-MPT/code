#include <msp430.h>
#include <stdint.h>
#include "hdq_comm.h"
#include "I2Croutines.h"
#include "stdlib.h"
#include <string.h>
#include <math.h>
/*
 * hdq_comm.c
 *
 *  Created on: Feb 25, 2020
 *      Author: Daniel Meulbroek
 */
float curr_dcr_fl=0;
float curr_dtr_fl=0;
int curr_dcr=0;
int curr_dtr=0;
float charge_diss=0;
float diss_hours=0;
float total_scurrent=0;
float scurrent_div=0;
float scurrent=0;
float data_dcr_old_fl=0;
float data_dtr_old_fl=0;
float charge_old_fl=0;
int data_dcr_old=0;
int data_dtr_old=0;
int charge_old=0;
char charge_old_str[10];
float charge_new_fl=0;
int charge_new=0;
int address_charge = 0;
char charge_str[10];
char dcr_str[3];

void clear_lcd1(){
    data_wr1(0xFE); // Command Prompt
    data_wr1(0x51); // Clear Screen
    data_wr1(0xFE); // Command Prompt
    data_wr1(0x46); // Move Cursor to Start
}

void data_wr1(unsigned char data){
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

void str_wr1(char* data){
    int i;
    int strLen = strlen(data);
    for(i = 0; i < strLen; i++){
        data_wr1(data[i]);
    }
}

void hdq_init(void){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Init Timer B
    TBCTL = 0;                  // Zero Out
    TB0CTL = BIT4+BIT8;         // Up Mode+ACLK Source

    // Init P1.0 Output
    P2DIR &= ~BIT3;             // INPUT
    P2OUT |= BIT3;              // PULL UP

    return;
}
//CITE THIS - taken from https://www.geeksforgeeks.org/convert-floating-point-number-string/
void reverse1(char* str, int len)
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
//CITE THIS - taken from https://www.geeksforgeeks.org/convert-floating-point-number-string/
int intToStr1(int x, char str[], int d)
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

    reverse1(str, i);
    str[i] = '\0';
    return i;
}
//CITE THIS - taken from https://www.geeksforgeeks.org/convert-floating-point-number-string/
void ftoa1(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr1(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr1((int)fpart, res + i + 1, afterpoint);
    }
}

float cc_update(void){
    int i=0;

    // Get new DCR and DTR values
    curr_dcr=cc_dataread(0x7E);
    curr_dtr=cc_dataread(0x78);

    // Get old values
    data_dcr_old = EEPROM_RandomRead(0x0421);
    data_dcr_old = data_dcr_old << 8;
    data_dcr_old |= EEPROM_RandomRead(0x0420);
    data_dtr_old = EEPROM_RandomRead(0x0423);
    data_dtr_old = data_dtr_old << 8;
    data_dtr_old |= EEPROM_RandomRead(0x0422);
    address_charge = 69;
    for(i = 0; i < 10; i++){
        charge_old_str[i] = EEPROM_RandomRead(address_charge);
        address_charge++;
    }

    charge_old = atof(charge_old_str);
    data_dcr_old_fl=(float)data_dcr_old;
    data_dtr_old_fl=(float)data_dtr_old;
    charge_old_fl=(float)charge_old;
    curr_dcr_fl=(float)curr_dcr;
    curr_dtr_fl=(float)curr_dtr;

    if(curr_dcr == 0){
            data_dcr_old_fl = 0-(data_dcr_old_fl);
        }
    if(curr_dtr == 0){
        data_dtr_old_fl = 0-(data_dtr_old_fl);
    }

    // Calculations
    scurrent = (curr_dcr_fl - data_dcr_old_fl) * 0.0125 / 10;    // Find total mA through sense
    if(scurrent < 0){ // if it rolled over reset and recalc
           scurrent += 65535; // 2^16
           curr_dcr_fl = 0;
           hdq_send(0x74,0x01);
    }

    diss_hours   = (curr_dtr_fl-data_dtr_old_fl)/4096;    // Find number of hours
    if(diss_hours < 0){ // if it rolled over reset and recalc
        diss_hours += 65535; // 2^16
        curr_dtr_fl = 0;
        hdq_send(0x74,0x08);
    }
    charge_diss=scurrent*diss_hours;            // Calc charge diss in mAh

    charge_new_fl = charge_old_fl - charge_diss;          // Update charge

    ftoa1(charge_new_fl, charge_str, 5);
    // Change to writeable data
    curr_dcr=(int)curr_dcr_fl;
    curr_dtr=(int)curr_dtr_fl;
    charge_new=(int)charge_new_fl;

    // write new values
    EEPROM_ByteWrite(0x0420,((unsigned char) (curr_dcr & 0xFF)));
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0421,((unsigned char) ((curr_dcr>>8) & 0xFF)));
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0422,((unsigned char) (curr_dtr & 0xFF)));
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0423,((unsigned char) ((curr_dtr>>8) & 0xFF)));
    EEPROM_AckPolling();
    address_charge = 69;
    for(i = 0; i < strlen(charge_str); i++){
       EEPROM_ByteWrite(address_charge,charge_str[i]);
       EEPROM_AckPolling();
       address_charge += 1;
    }
    /*EEPROM_ByteWrite(0x0069,((unsigned char) (charge_new & 0xFF)));
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0070,((unsigned char) ((charge_new>>8) & 0xFF)));
    EEPROM_AckPolling();*/

    return charge_new_fl;
}

uint16_t cc_dataread(uint8_t addr){
    uint16_t data = 0x0;
    _delay_cycles(50);
    hdq_init();
    data |= hdq_rec(addr+1);
    data = data<<8;
    _delay_cycles(50);
    hdq_init();
    data |= hdq_rec(addr);

    return data;
}

// Useless rn
void cc_clear(uint8_t addr) {
    if(addr==0x7E){
        hdq_send(0x74,0x01);
    } else if(addr==0x7C){
        hdq_send(0x74,0x02);
    } else if(addr==0x78){
        hdq_send(0x74,0x08);
    } else if(addr==0x76){
        hdq_send(0x74,0x10);
    }

    return;
}

uint8_t hdq_rec(uint8_t addr){
    // Decs
    uint8_t mask = 0x01;
    uint8_t i    = 0;       // index
    uint8_t bts  = 0x00;    // bit to send
    uint8_t data = 0x00;    // return data
    uint8_t pack_size = 0x8;// Packet size is 8
    uint8_t addr_send=addr; // address for use
    int     cc_resp = 0;    // coulomb counter resp flag
    int     try_count = 0;  // Keep track of attemps

    // Timing parameters
    static uint16_t tb  = 200;   // break time
    static uint16_t t1  = 30;    // one time
    static uint16_t t0  = 150;   // zero time
    static uint16_t tc  = 210;   // bit period
    TB0CCR0 = 220-1;             // Timer Rollover


    // Send Addr.
    while(!cc_resp && (try_count != 7)){
        ////////// Start /////////

        // Break to start transmission
        TB0CTL |= TBCLR;
        P2OUT &= ~BIT3;
        while(TB0R <= tb){
            P2DIR |= BIT3;              // Start break (low)
        }
        P2DIR &= ~BIT3;                 // end break (high)
        __delay_cycles(40);
        TB0CTL |= TBCLR;
        for(i=0;i<pack_size;i++){
            bts = mask & addr_send;          // Set send bit
            P2DIR |= BIT3;
            if(bts == 0x01){
                while(TB0R < t1);
                P2DIR &= ~BIT3;         // Set to input (high)
                while(TB0R < tc);
            } else {
                while(TB0R < t0);
                P2DIR &= ~BIT3;         // Set to input (high)
                if(i==7){
                    TB0CCR0 =1600-1;
                    TB0CTL |= TBCLR;
                    break;
                }
                while(TB0R < tc);
            }
            addr_send = addr >> (i+1);
        }
        P2DIR &= ~BIT3;             // Set to input

        // Wait for response
        while(TB0R < 1500-1){
            if((P2IN&BIT3)==0){     // Response received
                TB0CTL |= TBCLR;    // Reset timer
                TB0CCR0 = 220-1;    // set bit period
                cc_resp = 1;        // flag for no-timeout
                break;
            }
        }
        if(cc_resp){
            break;  // Need to leave try loop
        }
        TB0CCR0=250-1;
        try_count++;    // didn't work
    }

    // Packet recieving loop
    for(i=0;i<pack_size;i++){
        while(TB0R < 50-1);   // Wait to get data
        data |= (P2IN & BIT3)<<4;    //
        data = (i==7) ? data : data >> 1;
        while(TB0R < 215-1);
    }

    /*if(try_count == 7){
        return 0xFF;
    }*/
    return data;
}



void hdq_send(uint8_t addr, uint8_t data){
        // Decs
       uint8_t mask = 0x01;
       uint8_t i    = 0;       // index
       uint8_t bts  = 0x00;    // bit to send
       uint8_t pack_size = 0x8;// Packet size is 8
       addr = addr | 0x80;

       // Timing parameters
       static uint16_t tb  = 200;   // break time
       static uint16_t t1  = 30;    // one time
       static uint16_t t0  = 150;   // zero time
       static uint16_t tc  = 210;   // bit period
       TB0CCR0 = 220-1;             // Timer Rollover


        // Break to start transmission
        P2OUT &= ~BIT3;
        TB0CTL |= TBCLR;
        while(TB0R <= tb){
            P2DIR |= BIT3;              // Start break (low)
            P2OUT &= ~BIT3;
        }
        P2DIR &= ~BIT3;                 // end break (high)
        __delay_cycles(40);

        // Send Addr.
        TB0CTL |= TBCLR;
        for(i=0;i<pack_size;i++){
           bts = mask & addr;          // Set send bit
            P2DIR |= BIT3;
            if(bts == 0x01){
                while(TB0R < t1);
                P2DIR &= ~BIT3;         // Set to input (high)
                while(TB0R < tc);
            } else {
                while(TB0R < t0);
                P2DIR &= ~BIT3;         // Set to input (high)
                while(TB0R < tc);
            }
            addr = addr >> 1;
        }
        _delay_cycles(50);  // Delay for better transmission
        TB0CTL |= TBCLR;    // Clear to start new cycle
        for(i=0;i<pack_size;i++){
               bts = mask & data;          // Set send bit
               P2DIR |= BIT3;
               if(bts == 0x01){
                   while(TB0R < t1);
                   P2DIR &= ~BIT3;         // Set to input (high)
                   while(TB0R < tc);
               } else {
                   while(TB0R < t0);
                   P2DIR &= ~BIT3;         // Set to input (high)
                   while(TB0R < tc);
               }
               data = data >> 1;
         }

       return;
}


