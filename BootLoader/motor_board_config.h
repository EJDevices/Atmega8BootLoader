/*
 * motor_board_config.h
 *
 *	Created: 7/05/2014 9:05:51 PM
 *  Author: EJ Devices
 *  Copyright: EJ Devices 2014
 */


#ifndef BOARDCONFIG_H_
#define BOARDCONFIG_H_


// CPU frequency of 8MHz
#define F_CPU				8000000UL

// Leds
// PB0, 3, 4, 5
#define LED_PORT			B
#define LED_ORANGE_PIN		PB4
#define LED_GREEN_PIN		PB3
#define LED_YELLOW_PIN		PB0
#define LED_RED_PIN			PB5

// Buttons
// PD2, PD3
// off is the one below the crystal
// on is the one closest to the green header
#define BUTTON_PORT			D
#define BUTTON_ON_PIN		PD3
#define BUTTON_OFF_PIN		PD2

// Serial Baud
// Set the serial baud at 250Kbaud
// 1 - 250K, 25 = 19.2K
#define	BAUD_UBRRL			1

// RS485 
// PD0, PD1 is RX, TX
// PD4 low to receive, high to transmit
#define RS485_PORT			D
#define RS485_RX_PIN		PD0
#define RS485_TX_PIN		PD1
#define RS485_TX_ENABLE_PIN PD4

#if		BUTTON_PORT == RS485_PORT
#define BUTTON_RS485_PORT  RS485_PORT
#endif

#endif /* BOARDCONFIG_H_ */