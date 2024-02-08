/*
 * sinewave.h
 *
 *  Created on: 12 Mar 2020
 *      Author: 20889356
 */

#ifndef SINEWAVE_H_
#define SINEWAVE_H_

#include <stdint.h>

void wave_init(void);
void wave_fillbuffer(uint16_t* buffer, uint8_t type, uint16_t len);


#endif /* SINEWAVE_H_ */
