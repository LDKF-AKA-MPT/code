#include <msp430.h> 


/**
 * main.c
 */

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Initialization =============================================

    // Set up UART for P1.1 (input) and P1.2 (output)
    //P1SEL |= BIT1 + BIT2;
    //P1SEL2 |= BIT1 + BIT2;

    UCMST = 1;
    // Set up I2C
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC; // Make micro the master for I2C
    UCMODE1 = 11;
    UCSYNC = 1;

    //=============================================================



    // Set P1 for GPIO
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1DIR |= BIT0;
    // Toggle P1 to flash LED
    while(1){
        if((P1IN & BIT1) != BIT1){ // Button pressed
            __delay_cycles(55000); // Debounce
            P1OUT ^= BIT0;
            // Initiate data transmission on I2C
            UCTR = 1;
            UCT1STT = 1; // I2C Start Condition
            // UCSLA10 = "slave address size"; // Slave address size
            // UCGCEN = "general call response"; // General call response
            // UCB1TXBUF = "data"; // TX buffer
            UCT1IFG = 1;
            // Once acknowledged by slave UCTxSTT & UCTxIFG are cleared and UCTxIFG is set when data is sent
            if (UCT1SST && UCT1IFG){
                UCT1STP = 1; // Stop condition if UCTxSTT & UCTxIFG are both set
                // DEBUG FLASHERS
                while(1){

                }
            }

        }
    }

    return 0;
}
