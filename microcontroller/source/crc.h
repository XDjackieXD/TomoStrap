/*
 * crc.h
 *
 *  https://github.com/pebble/ArduinoPebbleSerial
 */

#ifndef SOURCE_CRC_H_
#define SOURCE_CRC_H_

#include <stdint.h>

void crc8_calculate_byte_streaming(const uint8_t data, uint8_t *crc);

#endif /* SOURCE_CRC_H_ */
