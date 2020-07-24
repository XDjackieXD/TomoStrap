/*
 * ad5933.h
 *
 *  Created on: 17.03.2017
 *      Author: jakob
 */

#include "ad5933.h"
#include "i2c_hal.h"

void ad5933_write_register(uint8_t register_address, uint8_t data) {
	i2c_write_reg(AD5933_ADDRESS, register_address, data);
}

void ad5933_write_address_ptr(uint8_t register_address) {
	i2c_write_reg(AD5933_ADDRESS, AD5933_COMMAND_ADDRESS_PTR, register_address);
}

void ad5933_write_block(uint8_t register_address, uint8_t *data, uint8_t data_length) {
	ad5933_write_address_ptr(register_address);
	i2c_low_start(AD5933_ADDRESS, I2C_WRITE);
	uint8_t tmp[2] = { AD5933_COMMAND_BLOCK_WRITE, data_length };
	i2c_low_write(tmp, 2);
	i2c_low_write(data, data_length);
	i2c_low_stop();
}

uint8_t ad5933_read_register(uint8_t register_address) {
	ad5933_write_address_ptr(register_address);
	uint8_t tmp[1];
	i2c_read(AD5933_ADDRESS, tmp, 1);
	return tmp[0];
}

void ad5933_read_block(uint8_t register_address, uint8_t *data, uint8_t data_length) {
	ad5933_write_address_ptr(register_address);
	i2c_low_start(AD5933_ADDRESS, I2C_WRITE);
	uint8_t tmp[2] = { AD5933_COMMAND_BLOCK_READ, data_length };
	i2c_low_write(tmp, 2);
	i2c_low_write(data, data_length);
	i2c_low_restart(AD5933_ADDRESS, I2C_READ);
	i2c_low_read(data, data_length, true);
	i2c_low_stop();
}


void ad5933_init(operation_mode_t op_mode, output_voltage_t voltage, pga_gain_t pga_gain, clock_source_t clock_source, uint32_t start_frequency, uint32_t frequency_increment, uint16_t number_increments, uint16_t settling_time_cycles) {
	i2c_low_start(AD5933_ADDRESS, I2C_WRITE);
	uint8_t tmp[3] = { AD5933_COMMAND_BLOCK_WRITE, 12, 0 };
	i2c_low_write(tmp, 2);
	tmp[0] = ((op_mode << 4) | (voltage << 1) | pga_gain) & 0xFF;
	tmp[1] = (clock_source << 3) & 0xFF;
	i2c_low_write(tmp, 2);
	tmp[0] = (start_frequency >> 16) & 0xFF;
	tmp[1] = (start_frequency >> 8) & 0xFF;
	tmp[2] = (start_frequency) & 0xFF;
	i2c_low_write(tmp, 3);
	tmp[0] = (frequency_increment >> 16) & 0xFF;
	tmp[1] = (frequency_increment >> 8) & 0xFF;
	tmp[2] = (frequency_increment) & 0xFF;
	i2c_low_write(tmp, 3);
	tmp[0] = (number_increments >> 8) & 0xFF;
	tmp[1] = (number_increments) & 0xFF;
	i2c_low_write(tmp, 2);
	tmp[0] = (settling_time_cycles >> 8) & 0xFF;
	tmp[1] = (settling_time_cycles) & 0xFF;
	i2c_low_write(tmp, 2);
	i2c_low_stop();
}

void ad5933_set_op_mode(operation_mode_t op_mode) {
	ad5933_write_register(AD5933_REGISTER_CONTROL_1, (ad5933_read_register(AD5933_REGISTER_CONTROL_1) & 0x0F) | (op_mode << 4));
}

void ad5933_set_output_voltage(output_voltage_t output_voltage) {
	ad5933_write_register(AD5933_REGISTER_CONTROL_1, (ad5933_read_register(AD5933_REGISTER_CONTROL_1) & 0xF9) | (output_voltage << 1));
}

void ad5933_set_pga_gain(pga_gain_t pga_gain) {
	ad5933_write_register(AD5933_REGISTER_CONTROL_1, (ad5933_read_register(AD5933_REGISTER_CONTROL_1) & 0xFE) | pga_gain);
}

void ad5933_set_clock_source(clock_source_t clock_source) {
	ad5933_write_register(AD5933_REGISTER_CONTROL_2, (ad5933_read_register(AD5933_REGISTER_CONTROL_2) & 0xF7) | (clock_source << 3));
}

void ad5933_set_start_frequency(uint32_t frequency) {
	uint8_t tmp[3] = { (frequency >> 16) & 0xFF, (frequency >> 8) & 0xFF, frequency & 0xFF };
	ad5933_write_block(AD5933_REGISTER_START_1, tmp, 3);
}

void ad5933_set_frequency_increment(uint32_t frequency) {
	uint8_t tmp[3] = { (frequency >> 16) & 0xFF, (frequency >> 8) & 0xFF, frequency & 0xFF };
	ad5933_write_block(AD5933_REGISTER_INCREMENT_1, tmp, 3);
}

void ad5933_set_increment_number(uint16_t number) {
	uint8_t tmp[2] = { (number >> 8) & 0xFF, number & 0xFF };
	ad5933_write_block(AD5933_REGISTER_NUMBER_1, tmp, 2);
}

void ad5933_set_settling_cycles_number(uint16_t number) {
	uint8_t tmp[2] = { (number >> 8) & 0xFF, number & 0xFF };
	ad5933_write_block(AD5933_REGISTER_SETTLING_1, tmp, 2);
}

void ad5933_reset(void) {
	ad5933_write_register(AD5933_REGISTER_CONTROL_2, ad5933_read_register(AD5933_REGISTER_CONTROL_2) | 0x10);
}

uint8_t ad5933_get_status(void) {
	return ad5933_read_register(AD5933_REGISTER_STATUS);
}

uint16_t ad5933_get_temperature(void) {
	uint8_t tmp[2];
	ad5933_read_block(AD5933_REGISTER_TEMP_1, tmp, 2);
	return (tmp[0] << 8) | tmp[1];
}

impedance_data_t ad5933_get_impedance(void) {
	uint8_t tmp[4];
	impedance_data_t data;
	ad5933_read_block(AD5933_REGISTER_REAL_1, tmp, 4);
	data.real = (tmp[0] << 8) | tmp[1];
	data.imaginary = (tmp[2] <<8) | tmp[3];
	return data;
}
