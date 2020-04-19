#include <stdint.h>
/*
 * hdq_comm.h
 *
 *  Created on: Feb 25, 2020
 *      Author: root
 */

#ifndef HDQ_COMM_H_
#define HDQ_COMM_H_



// Init
void hdq_init(void);

// Receive
uint16_t cc_dataread(uint8_t addr);
uint8_t hdq_rec(uint8_t addr);

// Send
void hdq_send(uint8_t addr, uint8_t data);
void cc_clear(uint8_t addr);

// Update
uint16_t cc_update(void);


#endif /* HDQ_COMM_H_ */
