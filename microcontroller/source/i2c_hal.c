/*
 * i2c_hal.c
 *
 *  Created on: 03.02.2017
 *      Author: jakob
 */

#include "i2c_hal.h"
#include "fsl_i2c.h"
#include "fsl_common.h"

i2c_master_config_t masterConfig;
i2c_master_transfer_t masterXfer;

void i2c_init(void) {
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    I2C_MasterInit(I2C_MASTER_BASEADDR, &masterConfig, CLOCK_GetFreq(I2C_MASTER_CLK_SRC));

    memset(&masterXfer, 0, sizeof(masterXfer));
}

void i2c_write(uint8_t slave_address, uint8_t data[], uint8_t data_length) {
    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)NULL;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = data_length;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
}

void i2c_read(uint8_t slave_address, uint8_t *data, uint8_t data_length) {
    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = (uint32_t)NULL;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = data_length;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
}

void i2c_write_reg(uint8_t slave_address, uint8_t address, uint8_t data) {
	uint8_t tmp[2] = {address, data};
    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)NULL;
    masterXfer.subaddressSize = 0;
    masterXfer.data = tmp;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
}

void i2c_read_reg(uint8_t slave_address, uint8_t address, uint8_t data[], uint8_t data_length) {
	uint8_t tmp[1] = {address};
    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)NULL;
    masterXfer.subaddressSize = 0;
    masterXfer.data = tmp;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag | kI2C_TransferNoStopFlag;

    I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);

    masterXfer.direction = kI2C_Read;
    masterXfer.data = data;
    masterXfer.dataSize = data_length;
    masterXfer.flags = kI2C_TransferDefaultFlag | kI2C_TransferRepeatedStartFlag;

    I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &masterXfer);
}

void i2c_low_start(uint8_t slave_address, bool direction) {
	I2C_MasterClearStatusFlags(I2C_MASTER_BASEADDR, kI2C_ArbitrationLostFlag | kI2C_IntPendingFlag | kI2C_StartDetectFlag | kI2C_StopDetectFlag);
	while (!(I2C_MASTER_BASEADDR->S & kI2C_TransferCompleteFlag)){}
	I2C_MasterStart(I2C_MASTER_BASEADDR, slave_address, (i2c_direction_t)direction);
	while (!(I2C_MASTER_BASEADDR->S & kI2C_IntPendingFlag)){}
}

void i2c_low_restart(uint8_t slave_address, bool direction) {
	I2C_MasterClearStatusFlags(I2C_MASTER_BASEADDR, kI2C_ArbitrationLostFlag | kI2C_IntPendingFlag | kI2C_StartDetectFlag | kI2C_StopDetectFlag);
	while (!(I2C_MASTER_BASEADDR->S & kI2C_TransferCompleteFlag)){}
	I2C_MasterRepeatedStart(I2C_MASTER_BASEADDR, slave_address, (i2c_direction_t)direction);
	while (!(I2C_MASTER_BASEADDR->S & kI2C_IntPendingFlag)){}
}

void i2c_low_stop(void) {
	I2C_MASTER_BASEADDR->S = kI2C_IntPendingFlag;
	I2C_MasterStop(I2C_MASTER_BASEADDR);
}

void i2c_low_write(uint8_t* data, uint8_t length) {
	//while (!(I2C_MASTER_BASEADDR->S & kI2C_IntPendingFlag)){}
	I2C_MasterWriteBlocking(I2C_MASTER_BASEADDR, data, length);
}

void i2c_low_read(uint8_t* data, uint8_t length, bool is_last) {
	volatile uint8_t dummy = 0;
	while (!(I2C_MASTER_BASEADDR->S & kI2C_TransferCompleteFlag)){}
	I2C_MASTER_BASEADDR->S = kI2C_IntPendingFlag;
	I2C_MASTER_BASEADDR->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
	if (length == 1 && is_last) {
		I2C_MASTER_BASEADDR->C1 |= I2C_C1_TXAK_MASK;
	}
	dummy = I2C_MASTER_BASEADDR->D;

    while ((length--)) {
        while (!(I2C_MASTER_BASEADDR->S & kI2C_IntPendingFlag)){}
        I2C_MASTER_BASEADDR->S = kI2C_IntPendingFlag;

        if (length == 1 && is_last) {
        	I2C_MASTER_BASEADDR->C1 |= I2C_C1_TXAK_MASK;
        }

        *data++ = I2C_MASTER_BASEADDR->D;
    }
}
