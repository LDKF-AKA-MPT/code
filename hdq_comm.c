#include <msp430.h>
#include <stdint.h>
#include "hdq_comm.h"
#include "I2Croutines.h"

/*
 * hdq_comm.c
 *
 *  Created on: Feb 25, 2020
 *      Author: Daniel Meulbroek
 */


void hdq_init(void){

    // Init Timer A
    TBCTL = 0;                  // Zero Out
    TB0CTL = BIT4+BIT8;         // Up Mode+ACLK Source

    // Init P1.0 Output
    P2DIR &= ~BIT3;             // INPUT
    P2OUT |= BIT3;              // PULL UP

    return;
}

uint16_t cc_update(void){
    // Get new DCR and DTR values
    uint16_t data_dcr=cc_dataread(0x7E);
    uint16_t data_dtr=cc_dataread(0x78);


    // Get old values
    uint16_t data_dcr_old = (uint16_t)EEPROM_RandomRead(0x0001);
    data_dcr_old = data_dcr_old << 8;
    data_dcr_old |= (uint16_t)EEPROM_RandomRead(0x0000);
    uint16_t data_dtr_old = (uint16_t)EEPROM_RandomRead(0x0003);
    data_dtr_old = data_dtr_old << 8;
    data_dtr_old |= (uint16_t)EEPROM_RandomRead(0x0002);
    uint16_t charge_old = (uint16_t)EEPROM_RandomRead(0x0005);
    charge_old = charge_old << 8;
    charge_old |= (uint16_t)EEPROM_RandomRead(0x0004);

    // Calculations
    float total_scurrent = ((float)data_dcr - (float)data_dcr_old);
    if(total_scurrent < 0){ // if it rolled over reset and recalc
           total_scurrent += 65536; // 2^16
           data_dcr = 0;
           hdq_send(0x74,0x01);
    }
    total_scurrent =total_scurrent*12.5/0.75; //mA
    float diss_hours   = ((float)data_dtr-(float)data_dtr_old);
    if(diss_hours < 0){ // if it rolled over reset and recalc
        diss_hours += 65536; // 2^16
        data_dtr = 0;
        hdq_send(0x74,0x08);
    }
    diss_hours = (diss_hours/4096);
    float charge_diss = total_scurrent * diss_hours;
    uint16_t charge_new = charge_old - (uint16_t)charge_diss;

    // Make them writeable to EEPROM
    unsigned char data_dcr_low  = (unsigned char) (data_dcr & 0xFF);
    unsigned char data_dcr_high = (unsigned char) ((data_dcr>>8) & 0xFF);
    unsigned char data_dtr_low  = (unsigned char) (data_dtr & 0xFF);
    unsigned char data_dtr_high = (unsigned char) ((data_dtr>>8) & 0xFF);
    unsigned char charge_new_low  = (unsigned char) (charge_new & 0xFF);
    unsigned char charge_new_high = (unsigned char) ((charge_new>>8) & 0xFF);

    // write new values
    EEPROM_ByteWrite(0x0000,data_dcr_low);
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0001,data_dcr_high);
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0002,data_dtr_low);
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0003,data_dtr_high);
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0004,charge_new_low);
    EEPROM_AckPolling();
    EEPROM_ByteWrite(0x0005,charge_new_high);
    EEPROM_AckPolling();

    return charge_new;
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



