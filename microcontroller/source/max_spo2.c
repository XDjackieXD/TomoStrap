/*
 * max_spo2.c
 *
 * modified version of
 * https://github.com/xcoder123/MAX30100/
 * to work with the max30102
 */


#include "max_spo2.h"

uint8_t redLEDCurrent;
float lastREDLedCurrentCheck;

uint8_t currentPulseDetectorState;
float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
float valuesBPMSum;
uint8_t valuesBPMCount;
uint8_t bpmIndex;
uint32_t lastBeatThreshold;

fifo_t prevFifo;

dcFilter_t dcFilterIR;
dcFilter_t dcFilterRed;
butterworthFilter_t lpbFilterIR;
meanDiffFilter_t meanDiffIR;

float irACValueSqSum;
float redACValueSqSum;
uint16_t samplesRecorded;
uint16_t pulsesDetected;
float currentSaO2Value;

LEDCurrent IrLedCurrent;

uint32_t sample_count = 0;
float sps = 100;

uint32_t millis() {
	return (sample_count/sps)*1000.0;
}

void spo2(
		ledMode mode,
		sampleRate samplingRate,
		pulseWidth pulseWidth,
        LEDCurrent irCurrent)
{
  currentPulseDetectorState = PULSE_IDLE;

  redLEDCurrent = (uint8_t)STARTING_RED_LED_CURRENT;
  lastREDLedCurrentCheck = 0;

  IrLedCurrent = irCurrent;

  max30102_setup(avg2, mode, ar16384, samplingRate, pulseWidth, redLEDCurrent, IrLedCurrent);


  dcFilterIR.w = 0;
  dcFilterIR.result = 0;

  dcFilterRed.w = 0;
  dcFilterRed.result = 0;


  lpbFilterIR.v[0] = 0;
  lpbFilterIR.v[1] = 0;
  lpbFilterIR.result = 0;

  meanDiffIR.index = 0;
  meanDiffIR.sum = 0;
  meanDiffIR.count = 0;


  valuesBPM[0] = 0;
  valuesBPMSum = 0;
  valuesBPMCount = 0;
  bpmIndex = 0;


  irACValueSqSum = 0;
  redACValueSqSum = 0;
  samplesRecorded = 0;
  pulsesDetected = 0;
  currentSaO2Value = 0;

}

pulseoxymeter_t spo2_update(uint32_t samples, uint16_t samples_per_second)
{
  sample_count = samples;
  sps = samples_per_second;

  pulseoxymeter_t result = {
    /*bool pulseDetected*/ false,
    /*float heartBPM*/ 0.0,
    /*float irCardiogram*/ 0.0,
    /*float irDcValue*/ 0.0,
    /*float redDcValue*/ 0.0,
    /*float SaO2*/ currentSaO2Value,
    /*uint32_t lastBeatThreshold*/ 0,
    /*float dcFilteredIR*/ 0.0,
    /*float dcFilteredRed*/ 0.0,
	0,
	0
  };


  fifo_t rawData = spo2_readFIFO();

  dcFilterIR = spo2_dcRemoval( (float)rawData.rawIR, dcFilterIR.w, ALPHA );
  dcFilterRed = spo2_dcRemoval( (float)rawData.rawRed, dcFilterRed.w, ALPHA );

  float meanDiffResIR = spo2_meanDiff( dcFilterIR.result, &meanDiffIR);
  spo2_lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

  irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
  redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
  samplesRecorded++;

  if( spo2_detectPulse( lpbFilterIR.result ) )
  {
    result.pulseDetected=true;
    pulsesDetected++;

    float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

    //This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
    currentSaO2Value = 110.0 - 13.6 * ratioRMS;
    result.SaO2 = currentSaO2Value;

    if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
    {
      irACValueSqSum = 0;
      redACValueSqSum = 0;
      samplesRecorded = 0;
    }
  }

  spo2_balanceIntesities( dcFilterRed.w, dcFilterIR.w );


  result.heartBPM = currentBPM;
  result.irCardiogram = lpbFilterIR.result;
  result.irDcValue = dcFilterIR.w;
  result.redDcValue = dcFilterRed.w;
  result.lastBeatThreshold = lastBeatThreshold;
  result.dcFilteredIR = dcFilterIR.result;
  result.dcFilteredRed = dcFilterRed.result;
  result.rawIr = rawData.rawIR;
  result.rawRed = rawData.rawRed;


  return result;
}

bool spo2_detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {
        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;

        //This method sometimes glitches, it's better to go through whole moving average everytime
        //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
        //valuesBPMSum -= valuesBPM[bpmIndex];
        //valuesBPM[bpmIndex] = rawBPM;
        //valuesBPMSum += valuesBPM[bpmIndex];

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;
        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;

        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void spo2_balanceIntesities( float redLedDC, float IRLedDC )
{

  if( millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
  {
    //Serial.println( redLedDC - IRLedDC );
    if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent <= 0xEF)
    {
      redLEDCurrent += 0x08;
      spo2_setLEDCurrents( redLEDCurrent, IrLedCurrent );
    }
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent >= 0x10)
    {
      redLEDCurrent -= 0x08;
      spo2_setLEDCurrents( redLEDCurrent, IrLedCurrent );
    }

    lastREDLedCurrentCheck = millis();
  }
}

void spo2_setLEDCurrents( uint8_t redCurrent, uint8_t irCurrent )
{
	max30102_setRedLedCurrent(redCurrent);
	max30102_setIrLedCurrent(irCurrent);
}

fifo_t spo2_readFIFO()
{
  fifo_t result;

  uint32_t buffer[2];

  max30102_readFifoSampleSpO2_32(buffer);

  result.rawIR = buffer[1] & 0x0003FFFF;
  result.rawRed = buffer[0] & 0x0003FFFF;

  return result;
}

dcFilter_t spo2_dcRemoval(float x, float prev_w, float alpha)
{
  dcFilter_t filtered;
  filtered.w = x + alpha * prev_w;
  filtered.result = filtered.w - prev_w;

  return filtered;
}

void spo2_lowPassButterworthFilter( float x, butterworthFilter_t * filterResult )
{
  filterResult->v[0] = filterResult->v[1];

  //Fs = 100Hz and Fc = 10Hz
  filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

  //Fs = 100Hz and Fc = 4Hz
  //filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter

  filterResult->result = filterResult->v[0] + filterResult->v[1];
}

float spo2_meanDiff(float M, meanDiffFilter_t* filterValues)
{
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}
