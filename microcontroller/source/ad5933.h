/*
 * ad5933.h
 *
 *  Created on: 17.03.2017
 *      Author: jakob
 */

#ifndef SOURCE_AD5933_H_
#define SOURCE_AD5933_H_

#include "fsl_common.h"

#define AD5933_ADDRESS 0x0D

#define AD5933_REGISTER_CONTROL_1	0x80
#define AD5933_REGISTER_CONTROL_2	0x81
#define AD5933_REGISTER_START_1		0x82
#define AD5933_REGISTER_START_2		0x83
#define AD5933_REGISTER_START_3		0x84
#define AD5933_REGISTER_INCREMENT_1	0x85
#define AD5933_REGISTER_INCREMENT_2 0x86
#define AD5933_REGISTER_INCREMENT_3 0x87
#define AD5933_REGISTER_NUMBER_1	0x88
#define AD5933_REGISTER_NUMBER_2	0x89
#define AD5933_REGISTER_SETTLING_1	0x8A
#define AD5933_REGISTER_SETTLING_2	0x8B
#define AD5933_REGISTER_STATUS		0x8F
#define AD5933_REGISTER_TEMP_1		0x92
#define AD5933_REGISTER_TEMP_2		0x93
#define AD5933_REGISTER_REAL_1		0x94
#define AD5933_REGISTER_REAL_2		0x95
#define AD5933_REGISTER_IMAGINARY_1	0x96
#define AD5933_REGISTER_IMAGINARY_2	0x97

#define AD5933_COMMAND_BLOCK_WRITE	0xA0
#define AD5933_COMMAND_BLOCK_READ	0xA1
#define AD5933_COMMAND_ADDRESS_PTR	0xB0

typedef enum operation_mode_t {
	no_operation		= 0x0,
	init_start_freq		= 0x1,
	start_sweep			= 0x2,
	increment_freq		= 0x3,
	repeat_freq			= 0x4,
	measure_temp		= 0x9,
	power_down_mode		= 0xA,
	standby_mode		= 0xB
} operation_mode_t;

typedef enum output_voltage_t {
	Vpp_2V				= 0x0,
	Vpp_200mV			= 0x1,
	Vpp_400mV			= 0x2,
	Vpp_1V				= 0x3
} output_voltage_t;

typedef enum pga_gain_t {
	gain_x1				= 0x1,
	gain_x5				= 0x0
} pga_gain_t;

typedef enum clock_source_t {
	clock_external		= 0x1,
	clock_internal		= 0x0
} clock_source_t;

typedef enum status_mask_t {
	valid_temp_measurement	= 0x01,
	valid_impedance_data	= 0x02,
	freq_sweep_completed	= 0x04,
} status_mask_t;

typedef struct impedance_data_t {
	uint16_t real;
	uint16_t imaginary;
} impedance_data_t;

void ad5933_init(operation_mode_t op_mode, output_voltage_t voltage, pga_gain_t pga_gain, clock_source_t clock_source, uint32_t start_frequency, uint32_t frequency_increment, uint16_t number_increments, uint16_t settling_time_cycles);
void ad5933_set_op_mode(operation_mode_t op_mode);
void ad5933_set_output_voltage(output_voltage_t output_voltage);
void ad5933_set_pga_gain(pga_gain_t pga_gain);
void ad5933_set_clock_source(clock_source_t clock_source);
void ad5933_set_start_frequency(uint32_t frequency);
void ad5933_set_frequency_increment(uint32_t frequency);
void ad5933_set_increment_number(uint16_t number);
void ad5933_set_settling_cycles_number(uint16_t number);
void ad5933_reset(void);
uint8_t ad5933_get_status(void);
uint16_t ad5933_get_temperature(void);
impedance_data_t ad5933_get_impedance(void);

#endif /* SOURCE_AD5933_H_ */
