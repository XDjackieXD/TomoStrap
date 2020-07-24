/*
 * max30102.c
 *
 *  Created on: 15.02.2017
 *      Author: jakob
 *
 */


#include "max30102.h"

/**** Interrupt Configuration ****/
void max30102_setInterruptConfiguration(bool almostFull, bool newDataReady, bool ambientLightCancelationOverflow, bool proximityTriggered, bool tempReady) {
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_INT_ENABLE_1, (almostFull << 7) | (newDataReady << 6) | (ambientLightCancelationOverflow << 5) | (proximityTriggered << 4));
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_INT_ENABLE_2, (tempReady << 1));
}

/**** Interrupt Functions ****/
uint8_t max30102_getInterruptFlags(void) {
	uint8_t data_read[2];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_INT_STATUS_1, data_read, 2);
	return (data_read[0] | data_read[0]);
}

/**** FiFo Config ****/
void max30102_setFifoAverage(fifoAverage avg) {
    uint8_t data_read[1];
    uint8_t data_write;
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_read, 1);
    data_write = (data_read[0] & 0x1F) | (avg << 5);
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_write);
}

void max30102_setFifoRollover(bool enabled) {
    uint8_t data_read[1];
    uint8_t data_write;
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_read, 1);
    data_write = (data_read[0] & 0xEF) | (enabled << 4);
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_write);
}
void max30102_setFifoAlmostFull(uint8_t freeSamples) {

    uint8_t data_read[1];
    uint8_t data_write;
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_read, 1);
    data_write = (data_read[0] & 0xF0) | (freeSamples & 0x0F);
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_write);
}

/**** Mode Config ****/
void max30102_shutdown(void) {
	uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read, 1);
    data_read[0] = data_read[0] | 0x80;
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read[0]);
}

void max30102_startup(void) {
	uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read, 1);
    data_read[0] = data_read[0] & 0x7F;
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read[0]);
}

void max30102_reset(void) {
	uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read, 1);
    data_read[0] = data_read[0] | 0x40;
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read[0]);
}

bool max30102_getResetFlag(void) {
	uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read, 1);
    data_read[0] = data_read[0] | 0x40;
    return (data_read[0] & 0x40) >> 6;
}

void max30102_setLedMode(ledMode mode) {
    uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read, 1);
	switch(mode) {
	case modeRed:
        data_read[0] = (data_read[0] & 0xF8) | 0x02;
		break;
	case modeSpO2:
        data_read[0] = (data_read[0] & 0xF8) | 0x03;
		break;
	case modeMulti:
        data_read[0] = (data_read[0] & 0xF8) | 0x07;
		break;
	}
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_MODE_CONFIG, data_read[0]);
}

/**** SpO2 Configuration ****/
void max30102_setAdcRange(adcRange range) {
    uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
    data_read[0] = (data_read[0] & 0x9F) | (range << 5);
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read[0]);
}

void max30102_setSampleRate(sampleRate rate) {
    uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
    data_read[0] = (data_read[0] & 0xE3) | (rate << 2);
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read[0]);
}

void max30102_setPulseWidth(pulseWidth width) {
    uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
    data_read[0] = (data_read[0] & 0xFC) | width;
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read[0]);
}

/**** LED Power Configuration ****/
void max30102_setRedLedCurrent(uint8_t red) {
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_LED_CONFIG_1, red);
}

void max30102_setIrLedCurrent(uint8_t ir) {
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_LED_CONFIG_2, ir);
}

void max30102_setProximityLedCurrent(uint8_t pilot) {
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_LED_CONFIG_PROX, pilot);
}

/**** Multi-LED time slot configuration ****/
void max30102_setMultiLedTimeSlotConfig(slotConfig slot1, slotConfig slot2, slotConfig slot3, slotConfig slot4) {
    uint8_t data_write;
	data_write = ((slot2 << 4) & 0x70) | (slot1 & 0x07);
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_SLOT_CONFIG_1, data_write);
    data_write = ((slot4 << 4) & 0x70) | (slot3 & 0x07);
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_SLOT_CONFIG_2, data_write);
};

/**** Temperature Data ****/
int max30102_readTemp(void) {
	uint8_t data_read[1];
	uint8_t temp_int, temp_fract;
    int temp_measured;
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_TEMP_INTEGER, data_read, 1);
    temp_int = data_read[0];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_TEMP_FRACTION, data_read, 1);
    temp_fract = data_read[0] & 0x0F;
    temp_measured = ((int)temp_int)+(((int)temp_fract) >> 4);
    return temp_measured;
}

void max30102_initTempRead(void) {
	uint8_t data_read[1];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_read, 1);
    data_read[0] = data_read[0] | 0x01;    // Enable temperature
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_read[0]);
}

/**** Proximity Interrupt Threshold ****/
// Set IR ADC count that will trigger beginning of HR or SpO2 mode.
// 8 MSBs of ADC count
void max30102_setProximityIntThreshold(uint8_t threshold) {
    i2c_write_reg(MAX30102_ADDRESS, MAX30102_PROX_INT_THRES, threshold);
}

/**** Part ID and Revision ****/
uint8_t max30102_getRevisionId(void) {
	uint8_t data_read[1];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_REVISION_ID, data_read, 1);
	return data_read[0];
}

uint8_t max30102_getPartId(void) {
	uint8_t data_read[1];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_PART_ID, data_read, 1);
	return data_read[0];
}

/**** FiFo functions ****/
void max30102_clearFifo(void) {
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_FIFO_R_POINTER, 0);
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_FIFO_W_POINTER, 0);
	i2c_write_reg(MAX30102_ADDRESS, MAX30102_OVR_COUNTER, 0);
}

uint8_t max30102_getFifoReadPointer(void) {
	uint8_t data_read[1];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_R_POINTER, data_read, 1);
	return data_read[0];
}

uint8_t max30102_getFifoWritePointer(void) {
	uint8_t data_read[1];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_W_POINTER, data_read, 1);
	return data_read[0];
}

uint8_t max30102_getFifoOverflowCounter(void) {
	uint8_t data_read[1];
	i2c_read_reg(MAX30102_ADDRESS, MAX30102_OVR_COUNTER, data_read, 1);
	return data_read[0];
}

uint8_t max30102_getAvailableSamples(void) {
	int16_t data = max30102_getFifoWritePointer() - max30102_getFifoReadPointer();
	if (data < 0) data += 32;
	return data;
}

void max30102_readFifoSamplesSpO2_32(uint32_t *data_red, uint32_t *data_ir, uint8_t data_offset, uint8_t samples, pulseWidth resolution) {
	uint8_t tmp[1] = {MAX30102_FIFO_DATA_REG};
	uint8_t data_read[6*samples], loop;
    i2c_low_start(MAX30102_ADDRESS, I2C_WRITE);
    i2c_low_write(tmp, 1);
    i2c_low_restart(MAX30102_ADDRESS, I2C_READ);
    for(loop=0; loop<samples; loop++){
    	if(loop < samples-1)
    		i2c_low_read(data_read, 6, false);
    	else
    		i2c_low_read(data_read, 6, true);
    	data_red[loop+data_offset] = ((data_read[0]<<16) | (data_read[1]<<8) | data_read[2]) >> (3-resolution);
    	data_ir[loop+data_offset] = ((data_read[3]<<16) | (data_read[4]<<8) | data_read[5]) >> (3-resolution);
    }
    i2c_low_stop();
}

void max30102_readFifoSamplesSpO2_16(uint16_t *data_red, uint16_t *data_ir, uint8_t data_offset, uint8_t samples, pulseWidth resolution) {
	uint8_t tmp[1] = {MAX30102_FIFO_DATA_REG};
	uint8_t data_read[6*samples], loop;
    i2c_low_start(MAX30102_ADDRESS, I2C_WRITE);
    i2c_low_write(tmp, 1);
    i2c_low_restart(MAX30102_ADDRESS, I2C_READ);
    for(loop=0; loop<samples; loop++){
    	if(loop < samples-1)
    		i2c_low_read(data_read, 6, false);
    	else
    		i2c_low_read(data_read, 6, true);
    	data_red[loop+data_offset] = ((data_read[0]<<16) | (data_read[1]<<8) | data_read[2]) >> (3-resolution);
    	data_ir[loop+data_offset] = ((data_read[3]<<16) | (data_read[4]<<8) | data_read[5]) >> (3-resolution);
    }
    i2c_low_stop();
}

void max30102_readFifoSampleSpO2_32(uint32_t *data) {
	uint8_t data_read[6];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_DATA_REG, data_read, 6);
    data[0] = ((data_read[0]<<16) | (data_read[1]<<8) | data_read[2]);
    data[1] = ((data_read[3]<<16) | (data_read[4]<<8) | data_read[5]);
}

uint32_t max30102_readFifoSampleHR(pulseWidth resolution) {
	uint8_t data_read[3];
    i2c_read_reg(MAX30102_ADDRESS, MAX30102_FIFO_DATA_REG, data_read, 3);
    return ((data_read[0]<<16) | (data_read[1]<<8) | data_read[2]) >> 2;
}

/**** workflow simplifications ****/
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

void max30102_setup(fifoAverage avg, ledMode mode, adcRange range, sampleRate rate, pulseWidth width, uint8_t redCurrent, uint8_t irCurrent) {
	/*max30102_reset();
	uint8_t timeout = 100;
	while(timeout > 0) {
		if(max30102_getResetFlag() == 0)
			break;
		delay(30); // wait about half a millisecond
	}*/

	max30102_setFifoAverage(avg);
	max30102_setFifoRollover(true);
	max30102_setLedMode(mode);
	max30102_setAdcRange(range);
	max30102_setSampleRate(rate);
	max30102_setPulseWidth(width);
	max30102_setRedLedCurrent(redCurrent);
	max30102_setIrLedCurrent(irCurrent);

	max30102_clearFifo();
}
