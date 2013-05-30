/*-----------------------------------------------------------------Benito7g.h
 
 Based on Benito7Serial CopyLeft 2008 Donald Delmar Davis (don@digithink.com)
 which is based on
 based on USBtoSerial.[c-h]
 MyUSB Library Copyright (C) Dean Camera, 2008.
  dean [at] fourwalledcubicle [dot] com www.fourwalledcubicle.com
 Released under the LGPL Licence, Version 3
-------------------------------------------------------------------------------*/

#ifndef _Benito7g_H_
#define _Benito7g_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/interrupt.h>

		#include "Descriptors.h"
		#include "RingBuff.h"

		#include <MyUSB/Version.h>                         // Library Version Information
		#include <MyUSB/Drivers/USB/USB.h>                 // USB Functionality
		#include <MyUSB/Drivers/AT90USBXXX/Serial.h>       // USART driver (defines mostly)
		#include <MyUSB/Scheduler/Scheduler.h>             // Simple scheduler for task management

	/* Tweekables
	   POV_MILISECONDS -- The amount of time an LED needs to be on 
						  to be visibly bright.
	   DTR_MILISECONDS -- The (minimum) length of the pulse 
						  sent on DTR pin when DTR is requested
	   DTR_PIN 
	   RTS_PIN
	  */
	    #define POV_MILISECONDS	60
		#define DTR_MILISECONDS 2

#undef Benito7r0

#ifdef Benito7r0	  
//		#define DTR_PIN_PORT	PORTB
//		#define DTR_PIN_DDR		DDRB
//		#define DTR_PIN_PINS	PINB
//		#define DTR_PIN			PINB0
#else	  
//		#define DTR_PIN_PORT	PORTD
//		#define DTR_PIN_DDR		DDRD
//		#define DTR_PIN_PINS	PIND
//		#define DTR_PIN			PIND4
#endif

//		#define RTS_PIN_PORT	PORTD
//		#define RTS_PIN_DDR		DDRD
//		#define RTS_PIN_PINS	PIND
//		#define RTS_PIN			PIND6

//#ifdef __AT90USB647__
//		#define LEDS_PORT PORTE
//		#define LEDS_DDR DDRE
//		#define TXD_LED_PIN      PINE0
//		#define RXD_LED_PIN      PINE1
//#else
//		#define LEDS_PORT PORTC
//		#define LEDS_DDR DDRC
//		#define TXD_LED_PIN      PINC6
//		#define RXD_LED_PIN      PINC7
//#endif


			


	/* Macros: */
		#define GET_LINE_CODING              0x21
		#define SET_LINE_CODING              0x20
		#define SET_CONTROL_LINE_STATE       0x22

	/* Event Handlers: */
		HANDLES_EVENT(USB_Connect);
		HANDLES_EVENT(USB_Disconnect);
		HANDLES_EVENT(USB_ConfigurationChanged);
		HANDLES_EVENT(USB_UnhandledControlPacket);
		
		
	/* Type Defines: */
		typedef struct
		{
			uint32_t BaudRateBPS;
			uint8_t  CharFormat;
			uint8_t  ParityType;
			uint8_t  DataBits;
		} CDC_Line_Coding_t;
		
	/* Enums: */
		enum CDC_Line_Coding_Format_t
		{
			OneStopBit          = 0,
			OneAndAHalfStopBits = 1,
			TwoStopBits         = 2,
		};
		
		enum CDC_Line_Codeing_Parity_t
		{
			Parity_None         = 0,
			Parity_Odd          = 1,
			Parity_Even         = 2,
			Parity_Mark         = 3,
			Parity_Space        = 4,
		};

	/* Function Prototypes: */
		void ReconfigureUSART(void);
		void Flash_TXD(void);
		void Flash_RXD(void);

	/* Tasks: */
		TASK(CDC_Task);
        TASK(CDC_DTR_PULSE_Task);
		TASK(CDC_TXD_POV_Task);
		TASK(CDC_RXD_POV_Task);
#endif
