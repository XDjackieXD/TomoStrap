/*
 * max30102.h
 *
 *  Created on: 15.02.2017
 *      Author: jakob
 */

#ifndef SOURCE_MAX30102_H_
#define SOURCE_MAX30102_H_

#include "i2c_hal.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/******************************************************************************/
/*********** PULSE OXIMETER AND HEART RATE REGISTER MAPPING  **************/
/******************************************************************************/

// status registers
#define MAX30102_INT_STATUS_1          0x00
#define MAX30102_INT_STATUS_2          0x01
#define MAX30102_INT_ENABLE_1          0x02
#define MAX30102_INT_ENABLE_2          0x03

// FIFO registers
#define MAX30102_FIFO_W_POINTER      0x04
#define MAX30102_OVR_COUNTER         0x05
#define MAX30102_FIFO_R_POINTER      0x06
#define MAX30102_FIFO_DATA_REG       0x07

// configuration registers
#define MAX30102_FIFO_CONFIG         0x08
#define MAX30102_MODE_CONFIG         0x09
#define MAX30102_SPO2_CONFIG         0x0A
#define MAX30102_LED_CONFIG_1        0x0C
#define MAX30102_LED_CONFIG_2        0x0D
#define MAX30102_LED_CONFIG_PROX     0x10
#define MAX30102_SLOT_CONFIG_1       0x11
#define MAX30102_SLOT_CONFIG_2       0x12

// temperature registers
#define MAX30102_TEMP_INTEGER        0x1F
#define MAX30102_TEMP_FRACTION       0x20
#define MAX30102_TEMP_CONFIG         0x21

// Proximity function
#define MAX30102_PROX_INT_THRES      0x30

// PART ID registers
#define MAX30102_REVISION_ID         0xFE
#define MAX30102_PART_ID             0xFF

/************************************** REGISTERS VALUE *******************************************/

// I2C address
#define MAX30102_ADDRESS             0x57

// Part ID
#define I_AM_MAX30102                0x15

//Enable interrupts
#define MAX30102_INT_ENB_A_FULL      ((uint8_t)0x80)
#define MAX30102_INT_ENB_TEMP_RDY    ((uint8_t)0x40)
#define MAX30102_INT_ENB_HR_RDY      ((uint8_t)0x20)
#define MAX30102_INT_ENB_SO2_RDY     ((uint8_t)0x10)

//Mode configuration
#define MAX30102_MODE_SHDN           ((uint8_t)0x80)      // Bit 7 high
#define MAX30102_MODE_RESET          ((uint8_t)0x40)      // Bit 6 high
#define MAX30102_MODE_TEMP_EN        ((uint8_t)0x01)
#define MAX30102_MODE_HR             ((uint8_t)0x02)
#define MAX30102_MODE_SPO2           ((uint8_t)0x03)

//SPO2 configuration
#define MAX30102_SPO2_HI_RES_EN           ((uint8_t)0x40)

typedef enum{ // This is the same for both LEDs
    pw68,     // 68us pulse, ADC 15
    pw118,    // 118us pulse, ADC 16
    pw215,    // 215us pulse, ADC 17
    pw411     // 411us pulse, ADC 18
}pulseWidth;

typedef enum{
    avg1,
    avg2,
    avg4,
    avg8,
    avg16,
    avg32
}fifoAverage;

typedef enum{
    modeRed,
    modeSpO2,
    modeMulti
}ledMode;

typedef enum{
    ar2048,  //  7.81pA LSB size
    ar4096,   // 15.63pA LSB size
    ar8192,  // 31.25pA LSB size
    ar16384  // 62.50pA LSB size
}adcRange;

typedef enum{
    sr50,
    sr100,
    sr200,
    sr400,
    sr800,
    sr1000,
    sr1600,
    sr3200
}sampleRate;

typedef enum{
    none,       // No LED active
    red,        // Red LED active with power set in LED1 config
    ir,         // IR LED active with power set in LED2 config
    none1,
    none2,
    red_pilot,  // Red LED active with power set in proximity pilot config
    ir_pilot    // IR LED active with power set in proximity pilot config
}slotConfig;

/**** Interrupt Configuration ****/
void max30102_setInterruptConfiguration(bool almostFull, bool newDataReady, bool ambientLightCancelationOverflow, bool proximityTriggered, bool tempReady);
/**** Interrupt Functions ****/
uint8_t max30102_getInterruptFlags(void);
/**** FiFo Config ****/
void max30102_setFifoAverage(fifoAverage avg);
void max30102_setFifoRollover(bool enabled);
void max30102_setFifoAlmostFull(uint8_t freeSamples);
/**** Mode Config ****/
void max30102_shutdown(void);
void max30102_startup(void);
void max30102_reset(void);
bool max30102_getResetFlag(void);
void max30102_setLedMode(ledMode mode);
/**** SpO2 Configuration ****/
void max30102_setAdcRange(adcRange range);
void max30102_setSampleRate(sampleRate rate);
void max30102_setPulseWidth(pulseWidth width);
/**** LED Power Configuration ****/
void max30102_setRedLedCurrent(uint8_t red);
void max30102_setIrLedCurrent(uint8_t ir);
void max30102_setProximityLedCurrent(uint8_t pilot);
/**** Multi-LED time slot configuration ****/
void max30102_setMultiLedTimeSlotConfig(slotConfig slot1, slotConfig slot2, slotConfig slot3, slotConfig slot4);
/**** Temperature Data ****/
int max30102_readTemp(void);
void max30102_initTempRead(void);
/**** Proximity Interrupt Threshold ****/
// Set IR ADC count that will trigger beginning of HR or SpO2 mode.
// 8 MSBs of ADC count
void max30102_setProximityIntThreshold(uint8_t threshold);
/**** Part ID and Revision ****/
uint8_t max30102_getRevisionId(void);
uint8_t max30102_getPartId(void);
/**** FiFo functions ****/
void max30102_clearFifo(void);
uint8_t max30102_getFifoReadPointer(void);
uint8_t max30102_getFifoWritePointer(void);
uint8_t max30102_getFifoOverflowCounter(void);
uint8_t max30102_getAvailableSamples(void);
void max30102_readFifoSamplesSpO2_32(uint32_t *data_red, uint32_t *data_ir, uint8_t data_offset, uint8_t samples, pulseWidth resolution);
void max30102_readFifoSamplesSpO2_16(uint16_t *data_red, uint16_t *data_ir, uint8_t data_offset, uint8_t samples, pulseWidth resolution);
void max30102_readFifoSampleSpO2_32(uint32_t *data);
uint32_t max30102_readFifoSampleHR(pulseWidth resolution);

/**** workflow simplifications ****/
void max30102_setup(fifoAverage avg, ledMode mode, adcRange range, sampleRate rate, pulseWidth width, uint8_t redCurrent, uint8_t irCurrent);

#endif /* SOURCE_MAX30102_H_ */
