/*
 * adg728.c
 *
 *  Created on: 17.03.2017
 *      Author: jakob
 */

#include "adg728.h"
#include "i2c_hal.h"

void adg728_set(ADG728Address address, uint8_t data) {
	uint8_t tmp[1] = {data};
	i2c_write(address, tmp, 1);
}
