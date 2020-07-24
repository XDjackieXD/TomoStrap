/*
 * i2c_hal.h
 *
 *  Created on: 03.02.2017
 *      Author: jakob
 */

#ifndef SOURCE_I2C_HAL_H_
#define SOURCE_I2C_HAL_H_

#include "fsl_common.h"

#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_BASEADDR I2C0

#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 32U

#define I2C_WRITE false
#define I2C_READ true

void i2c_init(void);
void i2c_write(uint8_t slave_address, uint8_t data[], uint8_t data_length);
void i2c_read(uint8_t slave_address, uint8_t *data, uint8_t data_length);
void i2c_write_reg(uint8_t slave_address, uint8_t address, uint8_t data);
void i2c_read_reg(uint8_t slave_address, uint8_t address, uint8_t data[], uint8_t data_length);

void i2c_low_start(uint8_t slave_address, bool direction);
void i2c_low_restart(uint8_t slave_address, bool direction);
void i2c_low_stop(void);
void i2c_low_write(uint8_t* data, uint8_t length);
void i2c_low_read(uint8_t* data, uint8_t length, bool is_last);

#endif /* SOURCE_I2C_HAL_H_ */
