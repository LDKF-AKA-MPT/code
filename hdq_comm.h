#include <stdint.h>
/*
 * hdq_comm.h
 *
 *  Created on: Feb 25, 2020
 *      Author: root
 */

#ifndef HDQ_COMM_H_
#define HDQ_COMM_H_

void hdq_init(void);

uint8_t hdq_rec(uint8_t addr);
void hdq_send(uint8_t addr, uint8_t data);




#endif /* HDQ_COMM_H_ */
