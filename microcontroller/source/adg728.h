/*
 * adg728.h
 *
 *  Created on: 17.03.2017
 *      Author: jakob
 */

#ifndef SOURCE_ADG728_H_
#define SOURCE_ADG728_H_

#include "fsl_common.h"

typedef enum ADG728Address {
    adg728_00   = 0x4C,
	adg728_01	= 0x4D,
	adg728_10   = 0x4E,
	adg728_11   = 0x4F
} ADG728Address;

void adg728_set(ADG728Address address, uint8_t data);

#endif /* SOURCE_ADG728_H_ */
