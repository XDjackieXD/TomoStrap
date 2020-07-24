/*
 * pebble_smartstrap.c
 *
 *  Created on: 15.12.2016
 *      Author: jakob
 */

#include "fsl_lpuart.h"
#include "fsl_common.h"
#include "pebble_smartstrap.h"
#include "crc.h"

uint8_t pebbleBuffer[PEBBLE_BUFFER_SIZE];
uint8_t txBuffer[4];
volatile uint16_t bufferIndex = 0;
bool bufferWasEscape = false;
uint8_t new_rx_crc = 0;
uint8_t current_rx_crc = 0;
uint8_t current_tx_crc = 0;

uint16_t time_until_disconnect = 0;

void pebble_handle_data(uint8_t buffer[], uint16_t length, uint8_t checksum);
void pebble_handle_frame(pebble_frame_t frame);
void pebble_handle_link_status();
void pebble_handle_link_profiles();
void pebble_handle_link_baudrates();
void pebble_handle_raw(pebble_frame_t frame);

void PEBBLE_LPUART_IRQHandler(void)
{
    uint8_t data;

    //debugled(true);
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(PEBBLE_LPUART))
    {
        data = LPUART_ReadByte(PEBBLE_LPUART);

        if (bufferWasEscape) { // get char after escape char and write it to buffer
        	bufferWasEscape = false;
        	current_rx_crc = new_rx_crc;
        	crc8_calculate_byte_streaming(data ^ PEBBLE_ESCAPE_MASK, &new_rx_crc);
        	pebbleBuffer[bufferIndex] = data ^ PEBBLE_ESCAPE_MASK;
        	bufferIndex++;
        } else if (data == PEBBLE_FRAME_DELIMITER && bufferIndex != 0) { // handle end of frame
        	pebble_handle_data(pebbleBuffer, bufferIndex, current_rx_crc);
        	bufferIndex = 0;
        } else if (data == PEBBLE_FRAME_DELIMITER) {
        	// start of frame. no action needed here.
        } else if (data == PEBBLE_ESCAPE_CHAR) { // handle escaping
        	bufferWasEscape = true;
        } else if (bufferIndex < PEBBLE_BUFFER_SIZE) { // write data to buffer
        	current_rx_crc = new_rx_crc;
        	crc8_calculate_byte_streaming(data, &new_rx_crc);
            pebbleBuffer[bufferIndex] = data;
            bufferIndex++;
        }
    }
    //debugled(false);
}

void smartstrap_loop_handler(uint16_t passed_time) {
	if(time_until_disconnect >= passed_time)
		time_until_disconnect -= passed_time;
	else
		smartstrap_init();
}

void smartstrap_init() {
	lpuart_config_t config;

	pebble_current_baud_rate = 0x00;

	LPUART_GetDefaultConfig(&config);

    config.baudRate_Bps = pebble_baud_rates[pebble_current_baud_rate]*100;
    config.parityMode = kLPUART_ParityDisabled;
    config.stopBitCount = kLPUART_OneStopBit;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(PEBBLE_LPUART, &config, CLOCK_GetFreq(SYS_CLK));

    LPUART_EnableInterrupts(PEBBLE_LPUART, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(PEBBLE_LPUART_IRQn);

    pebble_status = PEBBLE_STATUS_CONNECTING;
    time_until_disconnect = PEBBLE_TIMEOUT;
}


// we calculate the needed register settings for the requested baud rate before actually setting it as this calculation takes longer than the pebble takes to respond after a baud rate switch when usign a rather low clock speed.
volatile uint16_t sbr;
volatile uint32_t osr;
void pebble_prepare_baud_rate(uint32_t baudRate_Bps) {
    if(pebble_status == PEBBLE_STATUS_CONNECTED)
    	__asm("NOP");

	uint32_t srcClock_Hz = CLOCK_GetFreq(PEBBLE_LPUART_CLKSRC);
    uint16_t sbrTemp;
    uint32_t osrTemp, tempDiff, calculatedBaud, baudDiff;

    /* This LPUART instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, OSR is typically hard-set to 16 in other LPUART instantiations
     * loop to find the best OSR value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of OSR */

    baudDiff = baudRate_Bps;
    osr = 0;
    sbr = 0;
    for (osrTemp = 4; osrTemp <= 32; osrTemp++)
    {
        /* calculate the temporary sbr value   */
        sbrTemp = (srcClock_Hz / (baudRate_Bps * osrTemp));
        /*set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate*/
        if (sbrTemp == 0)
        {
            sbrTemp = 1;
        }
        /* Calculate the baud rate based on the temporary OSR and SBR values */
        calculatedBaud = (srcClock_Hz / (osrTemp * sbrTemp));

        tempDiff = calculatedBaud - baudRate_Bps;

        /* Select the better value between srb and (sbr + 1) */
        if (tempDiff > (baudRate_Bps - (srcClock_Hz / (osrTemp * (sbrTemp + 1)))))
        {
            tempDiff = baudRate_Bps - (srcClock_Hz / (osrTemp * (sbrTemp + 1)));
            sbrTemp++;
        }

        if (tempDiff <= baudDiff)
        {
            baudDiff = tempDiff;
            osr = osrTemp; /* update and store the best OSR value calculated */
            sbr = sbrTemp; /* update store the best SBR value calculated */
        }
    }

    /* Check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate OSR value */
    if (baudDiff < ((baudRate_Bps / 100) * 3))
    {

    }
}

void pebble_set_baud_rate() {
	uint32_t temp, oldCtrl;

    /* Store CTRL before disable Tx and Rx */
    oldCtrl = PEBBLE_LPUART->CTRL;

    /* Disable LPUART TX RX before setting. */
    PEBBLE_LPUART->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);

    temp = PEBBLE_LPUART->BAUD;

    /* Acceptable baud rate, check if OSR is between 4x and 7x oversampling.
     * If so, then "BOTHEDGE" sampling must be turned on */
    if ((osr > 3) && (osr < 8))
    {
        temp |= LPUART_BAUD_BOTHEDGE_MASK;
    }

    /* program the osr value (bit value is one less than actual value) */
    temp &= ~LPUART_BAUD_OSR_MASK;
    temp |= LPUART_BAUD_OSR(osr - 1);

    /* write the sbr value to the BAUD registers */
    temp &= ~LPUART_BAUD_SBR_MASK;
    PEBBLE_LPUART->BAUD = temp | LPUART_BAUD_SBR(sbr);

    /* Restore CTRL. */
    PEBBLE_LPUART->CTRL = oldCtrl;
	//LPUART_SetBaudRate(PEBBLE_LPUART, baud_rate, CLOCK_GetFreq(PEBBLE_LPUART_CLKSRC));
}

void pebble_enable_rx(bool enabled) {
	LPUART_EnableRx(PEBBLE_LPUART, enabled);
}

bool pebble_is_ready_to_send() {
	return (kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(PEBBLE_LPUART));
}

bool pebble_transmission_complete() {
	return (kLPUART_TransmissionCompleteFlag & LPUART_GetStatusFlags(PEBBLE_LPUART));
}

void pebble_send_byte(uint8_t data) {
	while(!pebble_is_ready_to_send()){}
	LPUART_WriteByte(PEBBLE_LPUART, data);
}

void pebble_send_break() {
	PEBBLE_LPUART->CTRL = PEBBLE_LPUART->CTRL | LPUART_CTRL_M_MASK | kLPUART_ParityEven;
	while(!pebble_is_ready_to_send()){}
	pebble_send_byte(PEBBLE_BREAK_CHARACTER);
	while(!pebble_is_ready_to_send()){}
	PEBBLE_LPUART->CTRL = (PEBBLE_LPUART->CTRL & (~(LPUART_CTRL_M_MASK | LPUART_CTRL_PT_MASK | LPUART_CTRL_PE_MASK))) | kLPUART_ParityDisabled;
	while(!pebble_is_ready_to_send()){}
}

void pebble_send_data(size_t length, const uint8_t *data) {
	uint8_t tmp = 0;
	while (tmp < length) {
		if(data[tmp] == PEBBLE_FRAME_DELIMITER || data[tmp] == PEBBLE_ESCAPE_CHAR) {
			pebble_send_byte(PEBBLE_ESCAPE_CHAR);
			pebble_send_byte(data[tmp] ^ PEBBLE_ESCAPE_MASK);
		} else {
			pebble_send_byte(data[tmp]);
		}
		crc8_calculate_byte_streaming(data[tmp], &current_tx_crc);
		tmp++;
	}
}

void pebble_send_frame(pebble_frame_t frame) {
	current_tx_crc = 0;

	pebble_enable_rx(false);
	if(frame.flags & 0x04) // is notification
		pebble_send_break();
	pebble_send_byte(PEBBLE_FRAME_DELIMITER);
	txBuffer[0] = frame.version;
	pebble_send_data(1, txBuffer);
	txBuffer[0] = frame.flags & 0xFF;
	txBuffer[1] = (frame.flags >> 8) & 0xFF;
	txBuffer[2] = (frame.flags >> 16) & 0xFF;
	txBuffer[3] = (frame.flags >> 24) & 0xFF;
	pebble_send_data(4, txBuffer);
	txBuffer[0] = frame.profile & 0xFF;
	txBuffer[1] = (frame.profile >> 8) & 0xFF;
	pebble_send_data(2, txBuffer);
	pebble_send_data(frame.payload_length, frame.payload);
	txBuffer[0] = current_tx_crc;
	pebble_send_data(1, txBuffer);
	pebble_send_byte(PEBBLE_FRAME_DELIMITER);
	while(!pebble_transmission_complete()) {}
	pebble_enable_rx(true);
}

void pebble_handle_data(uint8_t buffer[], uint16_t length, uint8_t checksum) {
	pebble_frame_t frame;
	if(length < 8)	// frame is shorter than minimum length
		return;
	if(buffer[length-1] != checksum)	// invalid checksum
		return;

	frame.version = buffer[0];
	frame.flags = buffer[1];
	frame.flags |= (buffer[2] << 8);
	frame.flags |= (buffer[3] << 16);
	frame.flags |= (buffer[4] << 24);
	frame.profile = buffer[5];
	frame.profile |= (buffer[6] << 8);

	uint8_t payload[length-8];
	uint8_t tmp;
	for(tmp=0; tmp < length-8; tmp++)
		payload[tmp] = buffer[tmp+7];
	frame.payload = payload;
	frame.payload_length = length-8;

	//pebble_data = frame;
	//pebble_data_available = 1;
	pebble_handle_frame(frame);
}

void pebble_handle_frame(pebble_frame_t frame) {
	if(frame.version != 0x01)	// version has to be 1
		return;

	if(((frame.flags >> 1) & 1) != 1)	// the isMaster flag has to be set in frames sent from the watch
		return;

	if(((frame.flags >> 2) & 1) != 0)	// the isNotification flag has to be cleared in frames sent from the watch
		return;

	switch(frame.profile) {
	case PEBBLE_PROFILE_LINK:
		if(frame.payload_length >= 2 && frame.payload_length <= 8){	// Link profile messages have to be at least 2 bytes and at max 8 bytes long
			if(frame.payload[0] != 1)	// version has to be 1
				return;

			switch(frame.payload[1]) {
			case PEBBLE_PROFILE_LINK_TYPE_STATUS:
				pebble_handle_link_status();
				break;
			case PEBBLE_PROFILE_LINK_TYPE_PROFILES:
				pebble_handle_link_profiles();
				break;
			case PEBBLE_PROFILE_LINK_TYPE_BAUDRATES:
				pebble_handle_link_baudrates();
				break;
			default:
				break;
			}
		}
		break;
	case PEBBLE_PROFILE_RAW:
		pebble_handle_raw(frame);
		break;

	//We don't need the generic profile
	case PEBBLE_PROFILE_GENERIC:
	default:
		break;
	}
}

void pebble_handle_link_status() {
	pebble_frame_t frame;
	uint8_t data[3] = {0x01, PEBBLE_PROFILE_LINK_TYPE_STATUS, PEBBLE_LINK_STATUS_OK};

	frame.version = 1;
	frame.flags = 0;
	frame.profile = PEBBLE_PROFILE_LINK;

	if(pebble_current_baud_rate != pebble_target_baud_rate)
		data[2] = PEBBLE_LINK_STATUS_BAUDRATE;

	if(pebble_status == PEBBLE_STATUS_DISCONNECTED)
		data[2] = PEBBLE_LINK_STATUS_DISCONNECT;

	if(pebble_status == PEBBLE_STATUS_WAITING_FOR_STATUS_MSG)
		pebble_status = PEBBLE_STATUS_CONNECTED;

	if(pebble_status == PEBBLE_STATUS_CONNECTING)
		pebble_status = PEBBLE_STATUS_CONNECTED;

	frame.payload = data;
	frame.payload_length = 3;

	time_until_disconnect = PEBBLE_TIMEOUT;

	pebble_send_frame(frame);
}

void pebble_handle_link_profiles() {
	pebble_frame_t frame;
	frame.version = 1;
	frame.flags = 0;
	frame.profile = PEBBLE_PROFILE_LINK;

	//TODO: Implement flexible system for profiles
	uint8_t data[] = {0x01, PEBBLE_PROFILE_LINK_TYPE_PROFILES, PEBBLE_PROFILE_RAW & 0xFF, (PEBBLE_PROFILE_RAW >> 2) & 0xFF};
	frame.payload = data;
	frame.payload_length = 4;

	pebble_send_frame(frame);
}

void pebble_handle_link_baudrates() {
	pebble_frame_t frame;
	frame.version = 1;
	frame.flags = 0;
	frame.profile = PEBBLE_PROFILE_LINK;

	uint8_t data[] = {0x01, PEBBLE_PROFILE_LINK_TYPE_BAUDRATES, pebble_target_baud_rate};
	frame.payload = data;
	frame.payload_length = 4;

	pebble_prepare_baud_rate(pebble_baud_rates[pebble_target_baud_rate]*100);

	pebble_send_frame(frame);

	pebble_status = PEBBLE_STATUS_WAITING_FOR_STATUS_MSG;

	pebble_set_baud_rate();
	pebble_current_baud_rate = pebble_target_baud_rate;
}

bool got_spo2_data = false;
uint32_t rate_data = 0;
uint32_t spo2_data = 0;
bool pulse_detected = false;

void set_spo2_send_data(uint32_t rate, uint32_t spo2, bool detected) {
	rate_data = rate;
	spo2_data = spo2;
	pulse_detected = detected;
	got_spo2_data = true;
}

bool got_impedance_data = false;
uint16_t real_data = 0;
uint16_t imaginary_data = 0;
uint8_t electrode1_data = 0;
uint8_t electrode2_data = 0;

bool impedance_should_measure = false;

void set_impedance_send_data(uint16_t real, uint16_t imaginary, uint8_t electrode1, uint8_t electrode2) {
	real_data = real;
	imaginary_data = imaginary;
	electrode1_data = electrode1;
	electrode2_data = electrode2;
	got_impedance_data = true;
}

bool get_impedance_measure() {
	if(impedance_should_measure == true) {
		impedance_should_measure = false;
		return true;
	} else
		return false;
}

bool sent_impedance_data() {
	return !got_impedance_data;
}

void pebble_handle_raw(pebble_frame_t frame) {
	if(frame.flags & 0x01) {
		if (got_impedance_data == true) {
			got_impedance_data = false;
			frame.version = 1;
			frame.flags = 0;
			frame.profile = PEBBLE_PROFILE_RAW;
			uint8_t payload[7] = {	DATA_TYPE_IMPEDANCE, (real_data>>8)&0xFF, (real_data)&0xFF, (imaginary_data>>8)&0xFF, (imaginary_data)&0xFF,
									electrode1_data, electrode2_data};
			frame.payload = payload;
			frame.payload_length = 7;
			pebble_send_frame(frame);
		} else if (got_spo2_data == true) {
			got_spo2_data = false;
			frame.version = 1;
			frame.flags = 0;
			frame.profile = PEBBLE_PROFILE_RAW;
			uint8_t payload[10] = {	DATA_TYPE_SPO2, pulse_detected, (rate_data>>24)&0xFF, (rate_data>>16)&0xFF, (rate_data>>8)&0xFF, (rate_data)&0xFF,
									(spo2_data>>24)&0xFF, (spo2_data>>16)&0xFF, (spo2_data>>8)&0xFF, (spo2_data)&0xFF};
			frame.payload = payload;
			frame.payload_length = 10;
			pebble_send_frame(frame);
		}
	} else if (frame.payload_length >= 1 && frame.payload[0] == DATA_TYPE_IMPEDANCE) {
		impedance_should_measure = true;
	}
}
