/*
 * max_spo2.h
 *
 * modified version of
 * https://github.com/xcoder123/MAX30100/
 * to work with the max30102
 */

#ifndef SOURCE_MAX_SPO2_H_
#define SOURCE_MAX_SPO2_H_

#include "fsl_common.h"
#include "max30102.h"
#include <math.h>

/*----------------------------------------------*/
/* Config defines, you can tailor to your needs */
/*----------------------------------------------*/

/* MAX30100 parameters */
#define DEFAULT_OPERATING_MODE            MAX30100_MODE_SPO2_HR
/*!!!IMPORTANT
 * You can't just throw these two values at random. Check Check table 8 in datasheet on page 19.
 * 100hz + 1600us is max for that resolution
 */
#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            0x66
#define STARTING_RED_LED_CURRENT          0x66


/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500

/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     4

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1

#define PULSE_BPM_SAMPLE_SIZE       20 //Moving average size





/* Enums, data structures and typdefs. DO NOT EDIT */
typedef struct{
  bool pulseDetected;
  float heartBPM;

  float irCardiogram;

  float irDcValue;
  float redDcValue;

  float SaO2;

  uint32_t lastBeatThreshold;

  float dcFilteredIR;
  float dcFilteredRed;

  uint32_t rawIr;
  uint32_t rawRed;
} pulseoxymeter_t;

typedef enum PulseStateMachine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;

typedef struct {
  uint32_t rawIR;
  uint32_t rawRed;
} fifo_t;

typedef struct {
  float w;
  float result;
} dcFilter_t;

typedef struct {
  float v[2];
  float result;
} butterworthFilter_t;

typedef struct {
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} meanDiffFilter_t;

/* MAX30100 register and bit defines, DO NOT EDIT */
#define MAX30100_DEVICE                   0x57

//Part ID Registers
#define MAX30100_REV_ID                   0xFE
#define MAX30100_PART_ID                  0xFF

//status registers
#define MAX30100_INT_STATUS               0x00
#define MAX30100_INT_ENABLE               0x01

//Fifo registers
#define MAX30100_FIFO_WRITE               0x02
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x03
#define MAX30100_FIFO_READ                0x04
#define MAX30100_FIFO_DATA                0x05

//Config registers
#define MAX30100_MODE_CONF                0x06
#define MAX30100_SPO2_CONF                0x07
#define MAX30100_LED_CONF                 0x09

//Temperature registers
#define MAX30100_TEMP_INT                 0x16
#define MAX30100_TEMP_FRACTION            0x17


//Bit defines MODE Regsiter
#define MAX30100_MODE_SHDN                (1<<7)
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<3)

//Bit defines SpO2 register
#define MAX30100_SPO2_HI_RES_EN           (1 << 6)

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

    void spo2( ledMode mode,
    		sampleRate samplingRate,
    		pulseWidth pulseWidth,
            LEDCurrent irCurrent);

    pulseoxymeter_t spo2_update(uint32_t samples, uint16_t samples_per_second);

    void spo2_setLEDCurrents( uint8_t redLedCurrent, uint8_t IRLedCurrent );
    fifo_t spo2_readFIFO();

    dcFilter_t spo2_dcRemoval(float x, float prev_w, float alpha);
    void spo2_lowPassButterworthFilter( float x, butterworthFilter_t * filterResult );
    float spo2_meanDiff(float M, meanDiffFilter_t* filterValues);

    bool spo2_detectPulse(float sensor_value);
    void spo2_balanceIntesities( float redLedDC, float IRLedDC );



#endif /* SOURCE_MAX_SPO2_H_ */
