/*
 * main.h
 *
 *  Created on: Feb 8, 2022
 *      Author: jcaf
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "system.h"
#include "types.h"

	struct _mainflag
	{
		unsigned sysTickMs :1;
		unsigned __a:7;
	};
	extern struct _mainflag mainflag;

	#define SYSTICK_MS 10//10ms

	#define PORTWxMOTOR_DIR_B 		PORTB
	#define PORTRxMOTOR_DIR_B 		PINB
	#define CONFIGIOxMOTOR_DIR_B 	DDRB
	#define PINxMOTOR_DIR_B		1

	#define PORTWxMOTOR_DIR_A 		PORTB
	#define PORTRxMOTOR_DIR_A 		PINB
	#define CONFIGIOxMOTOR_DIR_A 	DDRB
	#define PINxMOTOR_DIR_A		2

	#define PORTWxRV_MOTOR 		PORTC
	#define PORTRxRV_MOTOR 		PINC
	#define CONFIGIOxRV_MOTOR 	DDRC
	#define PINxRV_MOTOR		0

	#define PORTWxRELAY_TK_12V 		PORTC
	#define PORTRxRELAY_TK_12V 		PINC
	#define CONFIGIOxRELAY_TK_12V 	DDRC
	#define PINxRELAY_TK_12V		3


	#define PORTWxOUT_X 	PORTC
	#define PORTRxOUT_X 	PINC
	#define CONFIGIOxOUT_X 	DDRC
	#define PINxOUT_X		4

	#define PORTWxOUT_Y 	PORTC
	#define PORTRxOUT_Y 	PINC
	#define CONFIGIOxOUT_Y 	DDRC
	#define PINxOUT_Y		5

	#define PORTWxRS485_DIR 	PORTD
	#define PORTRxRS485_DIR 	PIND
	#define CONFIGIOxRS485_DIR 	DDRD
	#define PINxRS485_DIR		2

	#define PORTWxMOTOR_PWM 	PORTD
	#define PORTRxMOTOR_PWM 	PIND
	#define CONFIGIOxMOTOR_PWM 	DDRD
	#define PINxMOTOR_PWM		5

	#define PORTWxLIGHT_PWM 	PORTD
	#define PORTRxLIGHT_PWM 	PIND
	#define CONFIGIOxLIGHT_PWM 	DDRD
	#define PINxLIGHT_PWM		6

#endif /* MAIN_H_ */
