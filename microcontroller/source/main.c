#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "pebble_smartstrap.h"
#include "i2c_hal.h"
#include "max30102.h"
#include "max_spo2.h"
#include "adg728.h"
#include "ad5933.h"

#include "dstring.h"
#include "fsl_lpuart.h"

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}



//------------------------------ SpO2 & Pulse ------------------------------
#define MAX30102_ENABLE_GPIO	GPIOB
#define MAX30102_ENABLE_PORT	PORTB
#define MAX30102_ENABLE_PIN		1U
#define MAX30102_IRQ_GPIO		GPIOB
#define MAX30102_IRQ_PORT		PORTB
#define MAX30102_IRQ_PIN		2U
#define MAX30102_IRQ_HANDLER	PORTB_IRQHandler
#define MAX30102_IRQ			PORTB_IRQn


uint32_t samples = 0;
pulseoxymeter_t result;

//char buffer[60];

void max30102_update() {
	if(max30102_getAvailableSamples() > 0) {
		result = spo2_update(samples, 100);
		samples++;


		//FlexiPlot output for fancy plotting. requires dstring lib to be included
		//if(samples % 10 == 0) {
		//dsprintf_P(buffer, "{P0|IR|0,0,255|%d|RED|255,0,0|%d}\r\n", (int)(result.dcFilteredIR), (int)(result.dcFilteredRed));
		//LPUART_WriteBlocking(LPUART0, buffer, strlen(buffer));
		//dsprintf_P(buffer, "{P1|RED|255,0,255|%d|BEAT|0,0,255|%d}\r\n", (int)(result.irCardiogram*10.0), (int)(result.lastBeatThreshold*10.0));
		//LPUART_WriteBlocking(LPUART0, buffer, strlen(buffer));
		//dsprintf_P(buffer, "{P2|BPM|255,40,0|%d|SaO2|0,0,255|%d}\r\n", (int)(result.heartBPM*10.0), (int)(result.SaO2*10.0));
		//LPUART_WriteBlocking(LPUART0, buffer, strlen(buffer));
		//}

		if(samples % 100 == 0 /*&& pebble_status == PEBBLE_STATUS_CONNECTED*/) {
			set_spo2_send_data((int)(result.heartBPM * 10.0), (int)(result.SaO2 * 10.0), result.pulseDetected);

			pebble_frame_t frame;

			frame.version = 1;
			frame.flags = 4;
			frame.profile = PEBBLE_PROFILE_RAW;
			frame.payload_length = 0;
			pebble_send_frame(frame);
		}
	}
}

// For whatever reason the MAX30102 did not send an interrupt (probably not configured correctly) so polling it is...
/*void MAX30102_IRQ_HANDLER(void) {
	if(GPIO_GetPinsInterruptFlags(MAX30102_IRQ_GPIO) & (1<<MAX30102_IRQ_PIN)){
		GPIO_ClearPinsInterruptFlags(MAX30102_IRQ_GPIO, 1U << MAX30102_IRQ_PIN);

		uint8_t flags = max30102_getInterruptFlags();
		if(flags & (1<<1)) {
			//die temp ready
		}
		if(flags & (1<<7)) {
			uint8_t av_samples = max30102_getAvailableSamples();

			if(pebble_status == PEBBLE_STATUS_CONNECTED) {

			}
		}
	}
}
void max30102_init_irq(void) {
    gpio_pin_config_t max_config = {
        kGPIO_DigitalInput, 0,
    };
    PORT_SetPinInterruptConfig(MAX30102_IRQ_PORT, MAX30102_ENABLE_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(MAX30102_IRQ);
    GPIO_PinInit(MAX30102_IRQ_GPIO, MAX30102_IRQ_PIN, &max_config);
}*/

//power max30102 1.8V supply on or off
void max30102_power(bool state) {
	if(state){
		GPIO_SetPinsOutput(MAX30102_ENABLE_GPIO, 1<<MAX30102_ENABLE_PIN); /* PTB1 on */
	}else{
		GPIO_ClearPinsOutput(MAX30102_ENABLE_GPIO, 1<<MAX30102_ENABLE_PIN); /* PTB1 off */
	}
}


//------------------------------ Impedance ------------------------------

#define IMPEDANCE_FREQUENCY 1280093 //40kHz
#define IMPEDANCE_SETTING_TIME 128

#define IMPEDANCE_STATE_STANDBY 0x00
#define IMPEDANCE_STATE_STARTED 0x01
#define IMPEDANCE_STATE_AWAITING_COMPLETION 0x02

uint8_t impedance_state = IMPEDANCE_STATE_STANDBY;
uint8_t ele1 = 0;
uint8_t ele2 = 0;
bool impedance_should_measure = false;

void impedance_start_measure() {
	ad5933_reset();
	ad5933_set_op_mode(standby_mode);
	ad5933_set_op_mode(init_start_freq);
	ad5933_set_op_mode(start_sweep);
	impedance_state = IMPEDANCE_STATE_STARTED;
}

void impedance_start_sweep(uint8_t electrode1, uint8_t electrode2) {
	adg728_set(adg728_00, electrode1);
	adg728_set(adg728_01, electrode2);
	delay(100);
	ad5933_set_op_mode(repeat_freq);
	impedance_state = IMPEDANCE_STATE_AWAITING_COMPLETION;
}

void impedance_stop_measure() {
	ad5933_reset();
	ad5933_set_op_mode(standby_mode);
	impedance_state = IMPEDANCE_STATE_STANDBY;
}

bool impedance_get_data() {
	uint8_t tmp = ad5933_get_status();

	if (sent_impedance_data()&& ((tmp & valid_impedance_data) != 0)) {
		impedance_data_t impedance = ad5933_get_impedance();

		set_impedance_send_data(impedance.real, impedance.imaginary, ele1, ele2);

		pebble_frame_t frame;

		frame.version = 1;
		frame.flags = 4;
		frame.profile = PEBBLE_PROFILE_RAW;
		frame.payload_length = 0;
		pebble_send_frame(frame);

		impedance_state = IMPEDANCE_STATE_STARTED;
		return true;
	} else {
		return false;
	}
}

void impedance_loop_handler() {
	/*if(get_impedance_measure()){
		set_impedance_send_data(0, 1, 0, 1);

		pebble_frame_t frame;

		frame.version = 1;
		frame.flags = 4;
		frame.profile = PEBBLE_PROFILE_RAW;
		frame.payload_length = 0;
		pebble_send_frame(frame);
	}*/
	if(impedance_should_measure == false && get_impedance_measure())
		impedance_should_measure = true;

	if (impedance_should_measure == true && impedance_state == IMPEDANCE_STATE_STANDBY) {
		impedance_should_measure = false;
		impedance_start_measure();
	} else if (impedance_state == IMPEDANCE_STATE_STARTED) {
		ele1++;
		if (ele1 >= 8) {
			ele2++;
			ele1 = ele2 + 1;
		}
		impedance_start_sweep(1 << ele1, 1 << ele2);
	} else if (impedance_state == IMPEDANCE_STATE_AWAITING_COMPLETION) {
		if (impedance_get_data()) {
			if (ele2 >= 6) {
				ele1 = 0;
				ele2 = 0;
				impedance_stop_measure();
			}
		}
	}
}




/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  smartstrap_init();
  i2c_init();

  // In case of "fancy printing" using flexiplot, manually init the serial connectionand DON'T init the smartstrap connection
  /*lpuart_config_t config;
  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200;
  config.parityMode = kLPUART_ParityDisabled;
  config.stopBitCount = kLPUART_OneStopBit;
  config.enableTx = true;
  config.enableRx = false;
  LPUART_Init(LPUART0, &config, CLOCK_GetFreq(SYS_CLK));*/

  delay(100);

  
  //init network analyzer for impedance measurement
  ad5933_init(standby_mode, Vpp_2V, gain_x5, clock_internal, IMPEDANCE_FREQUENCY, 1, 1, IMPEDANCE_SETTING_TIME);

  //init max30102 for continous pulse & SpO2 measurement
  max30102_power(true);
  delay(50);
  spo2(modeSpO2, sr200, pw411, 0x66);

  //poll everything because interrupts of the max30102 aren't working corrently :(
  for(;;) {
    delay(120);
    //smartstrap_loop_handler(1);
    impedance_loop_handler();
    max30102_update();
  }

  for(;;) { /* Infinite loop to avoid leaving the main function */
    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}
