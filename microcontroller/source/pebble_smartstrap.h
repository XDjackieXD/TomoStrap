/*
 * pebble_smartstrap.h
 *
 *  Created on: 15.12.2016
 *      Author: jakob
 */

#ifndef SOURCE_PEBBLE_SMARTSTRAP_H_
#define SOURCE_PEBBLE_SMARTSTRAP_H_

#define PEBBLE_LPUART				LPUART0
#define PEBBLE_LPUART_CLKSRC		SYS_CLK
#define PEBBLE_LPUART_IRQn			LPUART0_IRQn
#define PEBBLE_LPUART_IRQHandler	LPUART0_IRQHandler

#define PEBBLE_TIMEOUT				60000

#define PEBBLE_BUFFER_SIZE			100

#define PEBBLE_FRAME_DELIMITER		0x7e
#define PEBBLE_ESCAPE_CHAR			0x7d
#define PEBBLE_ESCAPE_MASK			0x20
#define PEBBLE_BREAK_CHARACTER		0x00



typedef struct {
	uint8_t version;
	uint32_t flags;
	uint16_t profile;
	uint8_t *payload;
	uint8_t payload_length;
	uint8_t checksum;
} pebble_frame_t;

#define PEBBLE_PROFILE_LINK		0x0001
#define PEBBLE_PROFILE_RAW		0x0002
#define PEBBLE_PROFILE_GENERIC	0x0003

#define PEBBLE_PROFILE_LINK_TYPE_STATUS		0x01
#define PEBBLE_PROFILE_LINK_TYPE_PROFILES	0x02
#define PEBBLE_PROFILE_LINK_TYPE_BAUDRATES	0x03

#define PEBBLE_LINK_STATUS_OK			0x00
#define PEBBLE_LINK_STATUS_BAUDRATE		0x01
#define PEBBLE_LINK_STATUS_DISCONNECT	0x02


void smartstrap_init();
void smartstrap_loop_handler(uint16_t passed_time);
bool pebble_is_ready_to_send();
void pebble_send_data(size_t length, const uint8_t *data);
void pebble_send_frame(pebble_frame_t);

static const uint16_t pebble_baud_rates[] = {96, 144, 192, 288, 384, 576, 625, 1152, 125, 2304, 2500, 4608};
uint16_t pebble_target_baud_rate = 0x04;
volatile uint16_t pebble_current_baud_rate = 0x00;

#define PEBBLE_STATUS_CONNECTED		0x00
#define PEBBLE_STATUS_DISCONNECTED	0x01
#define PEBBLE_STATUS_WAITING_FOR_STATUS_MSG 0x02
#define PEBBLE_STATUS_CONNECTING	0x03

volatile uint8_t __attribute__((used))	pebble_status = PEBBLE_STATUS_DISCONNECTED;

#define DATA_TYPE_SPO2 0x00
#define DATA_TYPE_IMPEDANCE 'i'
void set_spo2_send_data(uint32_t rate, uint32_t spo2, bool detected);
void set_impedance_send_data(uint16_t real, uint16_t imaginary, uint8_t electrode1, uint8_t electrode2);
bool get_impedance_measure();
bool sent_impedance_data();

#endif /* SOURCE_PEBBLE_SMARTSTRAP_H_ */
