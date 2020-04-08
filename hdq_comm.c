#include <msp430.h>
#include <stdint.h>
#include "hdq_comm.h"
/*
 * hdq_comm.c
 *
 *  Created on: Feb 25, 2020
 *      Author: Daniel Mulbroek
 */

void hdq_init(void){

    // Init Timer A
    TA0CTL = 0;             // Zero Out
    TA0CTL = BIT4 + BIT9;// + (BIT6|BIT7);   // Up Mode
    TA0CCR0 = 250-1;//2500-1;       // 200 us

    // Init P1.0 Output
    P2DIR &= ~BIT3;     // INPUT
    P2OUT |= BIT3;      // PULL UP

    TA0CCTL0 |= CCIE;   // Interrupt enable
    return;
}

uint8_t hdq_rec(uint8_t addr){
    // Decs
    uint8_t mask = 0x01;
    uint8_t i    = 0;       // index
    uint8_t bts  = 0x00;    // bit to send
    uint8_t data = 0x00;    // return data
    uint8_t pack_size = 0x8;// Packet size is 8
    int     cc_resp = 0;    // coulomb counter resp flag

    // Timing parameters
    static uint16_t tb  = 234;   // break time
    static uint16_t t1  = 55;    // one time
    static uint16_t t0  = 162;   // zero time

    ////////// Start /////////
    TA0CCR0 = 250-1;//2500-1;       // 200 us
    TA0CTL |= TACLR;        // Clear timer

    // Break to start transmission
    P2OUT &= ~BIT3;
    while(TA0R <= tb){
        P2DIR |= BIT3;  // Start break (low)
    }
    P2DIR &= ~BIT3;     // end break (high)
    __delay_cycles(10);

    // Send Addr.
    for(i=0;i<pack_size;i++){
        bts = mask & addr;          // Set send bit
        if(bts == 0x01){
            while(TA0R < t1);
            P2DIR &= ~BIT3;         // Set to input (high)
            __delay_cycles(200);    // Need interrupt
        } else {
            while(TA0R < t0);
            P2DIR &= ~BIT3;         // Set to input (high)
            if(i==7){
                TA0CCR0 =1600-1;
                TA0CCTL0 &= ~CCIE;
            }
            __delay_cycles(90);    // Need interrupt
        }
        addr = addr >> 1;
    }
    P2DIR &= ~BIT3;
    while(TA0R < 800-1){
        if((P2IN&BIT3)==0){
            TA0CTL |= TACLR;    // Reset timer
            TA0CCR0 = 220-1;    // set bit period
            cc_resp = 1;        // flag for no-timeout
            break;
        }
    }

    if(!cc_resp){
        return 0x00; // no response from bq
    }

    // Packet recieving loop

    for(i=0;i<pack_size;i++){
        while(TA0R < 30-1);//0-1);   // Wait to get data
        //P1OUT |= BIT6;
        data |= (P2IN & BIT3)<<4;    //
        data = (i==7) ? data : data >> 1;
        //P1OUT &= ~BIT6;
        while(TA0R < 215-1);
    }


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
       static uint16_t tb  = 234;   // break time
       static uint16_t t1  = 55;    // one time
       static uint16_t t0  = 162;   // zero time

       ////////// Start /////////
       TA0CTL |= TACLR;        // Clear timer

       // Break to start transmission
       P2OUT &= ~BIT3;
       while(TA0R <= tb){
           P2DIR |= BIT3;  // Start break (low)
       }
       P2DIR &= ~BIT3;     // end break (high)
       __delay_cycles(10);

       // Send Addr.
       for(i=0;i<pack_size;i++){
           bts = mask & addr;          // Set send bit
           if(bts == 0x01){
               while(TA0R < t1);
               P2DIR &= ~BIT3;         // Set to input (high)
               __delay_cycles(200);    // Need interrupt
           } else {
               while(TA0R < t0);
               P2DIR &= ~BIT3;         // Set to input (high)
               __delay_cycles(90);    // Need interrupt
           }
           addr = addr >> 1;
       }

       for(i=0;i<pack_size;i++){
           bts = mask & data;          // Set send bit
           if(bts == 0x01){
               while(TA0R < t1);
               P2DIR &= ~BIT3;         // Set to input (high)
               __delay_cycles(200);    // Need interrupt
           } else {
               while(TA0R < t0);
               P2DIR &= ~BIT3;         // Set to input (high)
               if(i==7){
                   TA0CCR0 =1600-1;
                   TA0CCTL0 &= ~CCIE;
               }
               __delay_cycles(90);    // Need interrupt
           }
           data = data >> 1;
       }

       return;
}



