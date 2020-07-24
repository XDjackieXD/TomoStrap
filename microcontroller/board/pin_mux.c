/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This is a template file for pins configuration created by New Kinetis SDK 2.x Project Wizard. Enjoy! */

#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define SOPT5_LPUART0RXSRC_LPUART_RX  0x00u   /*!< LPUART0 Receive Data Source Select: LPUART_RX pin */

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initialize all pins used in this example
 */
void BOARD_InitPins(void)
{
	  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
	  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */

	  PORT_SetPinMux(PORTA, PIN0_IDX, kPORT_MuxAlt3);            /* PORTA0 (pin 14) is configured as SWD_CLK */
	  PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt3);            /* PORTA1 (pin 15) is configured as RESET_b */
	  PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt3);            /* PORTA2 (pin 16) is configured as SWD_DIO */
	  PORT_SetPinMux(PORTA, PIN3_IDX, kPORT_MuxAlt4);            /* PORTA3 (pin 3) is configured as LPUART0_TX */
	  PORT_SetPinMux(PORTA, PIN4_IDX, kPORT_MuxAlt4);            /* PORTA4 (pin 4) is configured as LPUART0_RX */
	  PORT_SetPinMux(PORTA, PIN5_IDX, kPORT_PinDisabledOrAnalog); /* PORTA5 (pin 5) is disabled */
	  PORT_SetPinMux(PORTA, PIN6_IDX, kPORT_PinDisabledOrAnalog); /* PORTA6 (pin 6) is disabled */
	  PORT_SetPinMux(PORTA, PIN7_IDX, kPORT_PinDisabledOrAnalog); /* PORTA7 (pin 7) is disabled */
	  PORT_SetPinMux(PORTB, PIN0_IDX, kPORT_MuxAsGpio);          /* PORTB0 (pin 8) is configured as PTB0 */
	  PORT_SetPinMux(PORTB, PIN1_IDX, kPORT_MuxAsGpio);          /* PORTB1 (pin 9) is configured as PTB1 */
	  PORT_SetPinMux(PORTB, PIN2_IDX, kPORT_MuxAsGpio);          /* PORTB2 (pin 10) is configured as PTB2 */
	  PORT_SetPinMux(PORTB, PIN3_IDX, kPORT_MuxAlt2);            /* PORTB3 (pin 11) is configured as I2C0_SCL */
	  PORT_SetPinMux(PORTB, PIN4_IDX, kPORT_MuxAlt2);            /* PORTB4 (pin 12) is configured as I2C0_SDA */
	  PORT_SetPinMux(PORTB, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTB5 (pin 13) is configured as PTB5 */
	  SIM->SOPT5 = ((SIM->SOPT5 &
	    (~(SIM_SOPT5_LPUART0RXSRC_MASK)))                        /* Mask bits to zero which are setting */
	      | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX) /* LPUART0 Receive Data Source Select: LPUART_RX pin */
	    );

	  GPIO_PinInit(GPIOB, 5u, &EXT_ENABLE_configOutput); /* mux PTB5 as output */
	  GPIO_PinInit(GPIOB, 1u, &EXT_ENABLE_configOutput); /* mux PTB1 as output */
}
