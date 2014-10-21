/*---------------------------------------------------------------Benito7g.c
 The benito7g is an AT90USB162 based board produced by tempusdictum.
 This firmware turns the benito7 into a usb to serial converter with a
 2ms pulse instead of the DTR signal. This is designed to reset 
 microcontrollers with serial bootloaders. 
 -------------------------------------------------------------------------------
 Based on BenitoSerial.c CopyLeft 2008 Donald Delmar Davis (don@digithink.com)
  based on USBtoSerial.[c-h]
 MyUSB Library Copyright (C) Dean Camera, 2008.
  dean [at] fourwalledcubicle [dot] com www.fourwalledcubicle.com
 Released under the LGPL Licence, Version 3
-------------------------------------------------------------------------------*/
/*	WINDOWS USERS,
    (windows has a driver for this but it cant figure it out.) 
    Before running, you will need to install the INF file that
	is located in the source project directory. This will enable
	Windows to use its inbuilt CDC drivers, negating the need
	for special Windows drivers for the device. To install,
	right-click the .INF file and choose the Install option.
*/

/*
	USB Mode:           Device
	USB Class:          Communications Device Class (CDC)
	USB Subclass:       Abstract Control Model (ACM)
	Relevant Standards: USBIF CDC Class Standard
	Usable Speeds:      Full Speed Mode
*/

/*-----------------------------------------------------------------------------*
 ************************************ GLOBALS **********************************
 *-----------------------------------------------------------------------------*/	

unsigned char do_store = 1;

#include "Benito7g.h"
#include "dmacode.c"

void setup(void);
TASK(LOOP_Task);

/* Scheduler Task List */
TASK_LIST
{
	{ Task: USB_USBTask          , TaskStatus: TASK_STOP },
	{ Task: CDC_Task             , TaskStatus: TASK_STOP },
	{ Task: CDC_DTR_PULSE_Task	 , TaskStatus: TASK_STOP },
	{ Task: CDC_TXD_POV_Task	 , TaskStatus: TASK_STOP },
	{ Task: CDC_RXD_POV_Task	 , TaskStatus: TASK_STOP },	
	{ Task: LOOP_Task             , TaskStatus: TASK_STOP },
};

/* Globals: */
CDC_Line_Coding_t LineCoding = { BaudRateBPS: 19200,
                                 CharFormat:  OneStopBit,
                                 ParityType:  Parity_None,
                                 DataBits:    8            };

RingBuff_t        Rx_Buffer;
RingBuff_t        Tx_Buffer;
RingBuff_t        ConsTx_Buffer;

volatile bool     Transmitting = false;
volatile bool     RXD_Flag = false;
volatile bool     TXD_Flag = false;
volatile bool     DTR_Flag = false;
volatile bool     usb_waiting = false;

volatile unsigned char uart_state=0;
void set_no_pr(void);
void set_pr(void);
void set_led(int led, int v);
unsigned char dma_running=0;

/*-----------------------------------------------------------------------------*
 **************************** INITIALIZATION AND MAIN **************************
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------int main(void)
 Set up the timer,serial,iopins and start scheduler.
-------------------------------------------------------------------------------*/

int main(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable Clock Division */
	SetSystemClockPrescaler(0);

    /* Millisecond timer initialization, with output compare interrupt enabled */
	OCR0A  = 0x7D;
	TCCR0A = (1 << WGM01);
	TCCR0B = ((1 << CS01) | (1 << CS00));
	TIMSK0 = (1 << OCIE0A);
	

	/* Hardware Initialization */
//	LEDS_DDR  |=  _BV(TXD_LED_PIN)|_BV(RXD_LED_PIN);
//	LEDS_PORT |=  _BV(TXD_LED_PIN)|_BV(RXD_LED_PIN);
//	DTR_PIN_DDR  |=  _BV(DTR_PIN);
//	DTR_PIN_PORT |= _BV(DTR_PIN); // PULL DTR HIGH
	//RTS_PIN_DDR |= _BV(RTS_PIN);
	//RTS_PIN_PORT |= _BV(RTS_PIN);
	ReconfigureUSART();
	
	
	/* Ringbuffer Initialization */
	Buffer_Initialize(&Rx_Buffer);
	Buffer_Initialize(&Tx_Buffer);
	Buffer_Initialize(&ConsTx_Buffer);

	
	/* Indicate USB not ready */
//	LEDS_PORT &= ~(_BV(TXD_LED_PIN)|_BV(RXD_LED_PIN));  
	
	/* Initialize Scheduler so that it can be used */
	Scheduler_Init();

	/* Initialize USB Subsystem */
	USB_Init();

	/* Scheduling - routine never returns, so put this last in the main function */
	Scheduler_Start();
}

/*----------------------------------------------ISR(TIMER0_COMPA_vect, ISR_BLOCK)
 * Increment scheduler tick counter once each millisecond 
 * Gleaned from MyUsb test demo
 *-----------------------------------------------------------------------------*/
 
ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
	/* Scheduler test - increment scheduler tick counter once each millisecond */
	Scheduler_TickCounter++;
}


/*-----------------------------------------------------------------------------*
 *********************************** USB SECTION *******************************
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------EVENT_HANDLER(USB_Connect)
 * start USB_USBTask, update status
 *-----------------------------------------------------------------------------*/

EVENT_HANDLER(USB_Connect)
{
	Buffer_Initialize(&ConsTx_Buffer);
	uart_state = 0xff;
	setup();
	/* Start USB management task */
	Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);
	Scheduler_SetTaskMode(LOOP_Task, TASK_RUN);

	/* Indicate USB enumerating */
//	LEDS_PORT &= ~(_BV(RXD_LED_PIN));  //rxd on
//	LEDS_PORT |= _BV(TXD_LED_PIN);  //txd off
}

/*--------------------------------------------------EVENT_HANDLER(USB_Disconnect)
 *  stop CDC_Task and USB_USBTask, update status
 *-----------------------------------------------------------------------------*/

EVENT_HANDLER(USB_Disconnect)
{
	/* Stop running CDC and USB management tasks */
	Scheduler_SetTaskMode(CDC_Task, TASK_STOP);
	Scheduler_SetTaskMode(LOOP_Task, TASK_STOP);
	Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);

	/* Indicate USB not ready */
	//LEDS_PORT &= ~(_BV(TXD_LED_PIN));  //txd on
	//LEDS_PORT |= _BV(RXD_LED_PIN);  //rxd off
}

/*---------------------------------------EVENT_HANDLER(USB_ConfigurationChanged)
 * setup 3 endpoints, start USB_USBTask, update status
 *-----------------------------------------------------------------------------*/

EVENT_HANDLER(USB_ConfigurationChanged)
{
	/* Setup CDC Notification, Rx and Tx Endpoints */
	Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPNUM, EP_TYPE_INTERRUPT,
		                       ENDPOINT_DIR_IN, CDC_NOTIFICATION_EPSIZE,
	                           ENDPOINT_BANK_SINGLE);

	Endpoint_ConfigureEndpoint(CDC_TX_EPNUM, EP_TYPE_BULK,
		                       ENDPOINT_DIR_IN, CDC_TXRX_EPSIZE,
	                           ENDPOINT_BANK_DOUBLE);

	Endpoint_ConfigureEndpoint(CDC_RX_EPNUM, EP_TYPE_BULK,
		                       ENDPOINT_DIR_OUT, CDC_TXRX_EPSIZE,
	                           ENDPOINT_BANK_DOUBLE);

	/* Indicate USB connected and ready */
	//LEDS_PORT |= _BV(TXD_LED_PIN)|_BV(RXD_LED_PIN);  //both off
	

	/* Start CDC task */
	Scheduler_SetTaskMode(CDC_Task, TASK_RUN);
}

/*--------------------------------------EVENT_HANDLER(USB_UnhandledControlPacket)
 * 
 * This handler deals with requests specific to CDC device class,
 *
 * In particular it handles serial setup and control (DTR/RTS) requests. 
 *
 * note: Request and RequestType are declared by the EVENT_HANDLER macro
 *-----------------------------------------------------------------------------*/

#define SET_CONTROL_LINE_STATE_RTS_MASK 0x0002
#define SET_CONTROL_LINE_STATE_DTR_MASK 0x0001

EVENT_HANDLER(USB_UnhandledControlPacket)
{
	uint8_t* LineCodingData = (uint8_t*)&LineCoding;
    uint16_t wValue;
	
	wValue = Endpoint_Read_Word_LE();
//	Endpoint_Ignore_Word();

	/* Process CDC specific control requests */
	switch (bRequest)
	{
		case GET_LINE_CODING:
			if (bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSetupReceived();

				for (uint8_t i = 0; i < sizeof(LineCoding); i++)
				  Endpoint_Write_Byte(*(LineCodingData++));	
				
				Endpoint_ClearSetupIN();
				while (!(Endpoint_IsSetupINReady()));
				
				while (!(Endpoint_IsSetupOUTReceived()));
				Endpoint_ClearSetupOUT();
			}
			
			break;
		case SET_LINE_CODING:
			if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSetupReceived();

				while (!(Endpoint_IsSetupOUTReceived()));

				for (uint8_t i = 0; i < sizeof(LineCoding); i++)
				  *(LineCodingData++) = Endpoint_Read_Byte();

				Endpoint_ClearSetupOUT();

				ReconfigureUSART();

				Endpoint_ClearSetupIN();
				while (!(Endpoint_IsSetupINReady()));
			}
	
			break;
		case SET_CONTROL_LINE_STATE:
			if (bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				if (wValue & SET_CONTROL_LINE_STATE_DTR_MASK) {
//				     DTR_PIN_PORT &= ~(_BV(DTR_PIN));
					 DTR_Flag = true;
//				     Scheduler_SetTaskMode(CDC_DTR_PULSE_Task, TASK_RUN);
					 //LEDs_TurnOffLEDs(STATUS_RED_LED);
			    } //else {
				//	 Handle Unset DTR request
				//}
				//if (wValue & SET_CONTROL_LINE_STATE_RTS_MASK) {
				//     Handle Set RTS
			    //} else {
				//     Handle UnSet RTS
				//}
				
				Endpoint_ClearSetupReceived();
				
				Endpoint_ClearSetupIN();
				while (!(Endpoint_IsSetupINReady()));
			}
	
			break;
	}
}

/*-----------------------------------------------------------------------------
 **************************** SERIAL HANDLING SECTION **************************
 *-----------------------------------------------------------------------------*/

/*----------------------------------------------------------------TASK(CDC_Task)
 * Moves the bits from USB to serial and back.
 *-----------------------------------------------------------------------------*/

TASK(CDC_Task)
{
  set_led(2, 1);
	if (USB_IsConnected)
	{
		/* Select the Serial Rx Endpoint */
		Endpoint_SelectEndpoint(CDC_RX_EPNUM);
		
		if (Endpoint_ReadWriteAllowed())
		{
			/* Read the recieved data endpoint into the transmission buffer */
			while (Endpoint_BytesInEndpoint())
			{
				/* Wait until the buffer has space for a new character */
				if (Rx_Buffer.Elements >= BUFF_STATICSIZE) {
					usb_waiting = 1;
					break;
				}
			
				/* Store each character from the endpoint */
				Buffer_StoreElement(&Rx_Buffer, Endpoint_Read_Byte());
				Flash_RXD(); // do the pov thing
			}
			
			/* Clear the endpoint buffer */
			Endpoint_ClearCurrentBank();
		}
		
		/* Check if Rx buffer contains data */
		if (Rx_Buffer.Elements)
		{
			/* Initiate the transmission of the buffer contents if USART idle */
			//if (UCSR1A & (1 << TXC1))
			//  Serial_TxByte(Buffer_GetElement(&Rx_Buffer));

			/* Initiate the transmission of the buffer contents if USART idle */
//			if (!(Transmitting))
//			{
//				Transmitting = true;
//				Serial_TxByte(Buffer_GetElement(&Rx_Buffer));
//			}
	 	 	Scheduler_SetTaskMode(LOOP_Task, TASK_RUN);
		}

		if (Tx_Buffer.Elements || ConsTx_Buffer.Elements) {
			/* Select the Serial Tx Endpoint */
			Endpoint_SelectEndpoint(CDC_TX_EPNUM);
	
			/* Check if the Tx buffer contains anything to be sent to the host */
			if (Endpoint_ReadWriteAllowed()) {
				if (Tx_Buffer.Elements) {
					/* Wait until Serial Tx Endpoint Ready for Read/Write */
					while (!(Endpoint_ReadWriteAllowed()));
			
			
					/* Write the transmission buffer contents to the recieved data endpoint */
					while (Tx_Buffer.Elements && (Endpoint_BytesInEndpoint() < CDC_TXRX_EPSIZE))
			  			Endpoint_Write_Byte(Buffer_GetElement(&Tx_Buffer));
				
					/* Check before sending the data if the endpoint is completely full */
					bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);
					/* Send the data */
					Endpoint_ClearCurrentBank();

					/* If a full endpoint was sent, we need to send an empty packet afterwards to terminate the transfer */
					if (IsFull) {
						/* Wait until Serial Tx Endpoint Ready for Read/Write */
						while (!(Endpoint_ReadWriteAllowed()));

						/* Send an empty packet to terminate the transfer */
						Endpoint_ClearCurrentBank();
					}
				} else
				if (ConsTx_Buffer.Elements && Endpoint_BytesInEndpoint() < (CDC_TXRX_EPSIZE-3)) {
			
					//while (!(Endpoint_ReadWriteAllowed()));
					/* Check before sending the data if the endpoint is completely full */
				
					if (uart_state == 0xff)
						Endpoint_Write_Byte(0xfc);	// start text flag
					/* Write the transmission buffer contents to the recieved data endpoint */
					while (ConsTx_Buffer.Elements && (Endpoint_BytesInEndpoint() < (CDC_TXRX_EPSIZE-1)))
			  			Endpoint_Write_Byte(Buffer_GetElement(&ConsTx_Buffer));
					if (uart_state == 0xff)
						Endpoint_Write_Byte(0xfd);	// end text flag
				
					bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);
					/* Send the data */
					Endpoint_ClearCurrentBank();
	
					/* If a full endpoint was sent, we need to send an empty packet afterwards to terminate the transfer */
					if (IsFull) {
					/* Wait until Serial Tx Endpoint Ready for Read/Write */
						while (!(Endpoint_ReadWriteAllowed()));
	
						/* Send an empty packet to terminate the transfer */
						Endpoint_ClearCurrentBank();
					}
				}
			}
		}
	}
  set_led(2, 0);
}

/*------------------------------------------------------------ISR(USART1_TX_vect)
 * stuff the next outgoing byte into UDR
 *-----------------------------------------------------------------------------*/
ISR(USART1_TX_vect)
{
	/* Send next character if avaliable */
//	if (Rx_Buffer.Elements)
//	  UDR1 = Buffer_GetElement(&Rx_Buffer);
//	else
	  if (Rx_Buffer.Elements) 
	 	 Scheduler_SetTaskMode(LOOP_Task, TASK_RUN);
	  Transmitting = false;
}

/*------------------------------------------------------------ISR(USART1_RX_vect)
 * put the byte into the buffer for CDC_Taks to send. 
 *-----------------------------------------------------------------------------*/
ISR(USART1_RX_vect)
{
	/* Character recieved, store it into the buffer */
	char s = UCSR1A;
	char c = UDR1;
	if (s&0x80 && !(s&0x14) && do_store && USB_IsConnected)
		Buffer_StoreElement(&ConsTx_Buffer, c);
	Flash_TXD(); // do the pov thing
}
		
/*-------------------------------------------------------TASK(CDC_DTR_PULSE_Task)
 * Insure that DTR pulse is at least DTR_MILISECONDS long
 *-----------------------------------------------------------------------------*/

TASK(CDC_DTR_PULSE_Task)
{
	static SchedulerDelayCounter_t DelayCounter; 

    if (DTR_Flag) { 
		Scheduler_ResetDelay(&DelayCounter);
		DTR_Flag=false;
	} else if (Scheduler_HasDelayElapsed(DTR_MILISECONDS, &DelayCounter)) {
//		DTR_PIN_PORT |= _BV(DTR_PIN); // PULL DTR HIGH
		Scheduler_ResetDelay(&DelayCounter);
		Scheduler_SetTaskMode(CDC_DTR_PULSE_Task, TASK_STOP);
	}	
}
		
/*----------------------------------------------------void ReconfigureUSART(void)
 * handle the reconfiguration request 
 *-----------------------------------------------------------------------------*/

void ReconfigureUSART(void)
{
	uint8_t ConfigMask = 0;

	/* Determine parity - non odd/even parity mode defaults to no parity */
	if (LineCoding.ParityType == Parity_Odd)
	  ConfigMask = ((1 << UPM11) | (1 << UPM10));
	else if (LineCoding.ParityType == Parity_Even)
	  ConfigMask = (1 << UPM11);

	/* Determine stop bits - 1.5 stop bits is set as 1 stop bit due to hardware limitations */
	if (LineCoding.CharFormat == TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	/* Determine data size - 5, 6, 7, or 8 bits are supported */
	if (LineCoding.DataBits == 6)
	  ConfigMask |= (1 << UCSZ10);
	else if (LineCoding.DataBits == 7)
	  ConfigMask |= (1 << UCSZ11);
	else if (LineCoding.DataBits == 8)
	  ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
	
	/* Enable double speed, gives better error percentages at 8MHz */
	UCSR1A = (1 << U2X1);
	
	/* Enable transmit and receive modules and interrupts */
	UCSR1B = (1 << TXCIE1) | (1 << RXCIE1) | (uart_state!=0xff?(1 << TXEN1):0) | (1 << RXEN1);

	/* Set the USART mode to the mask generated by the Line Coding options */
	UCSR1C = ConfigMask;
	
	/* Set the USART baud rate register to the desired baud rate value */
	UBRR1  = SERIAL_2X_UBBRVAL((uint16_t)LineCoding.BaudRateBPS);
}

/*-----------------------------------------------------------------------------
 **************************** BLINKIN LIGHTS SECTION **************************
 *-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------Flash_TXD()
 * Turn the TXD on and let the schedular keep in on
 *-----------------------------------------------------------------------------*/

void Flash_TXD(void)
{
	//LEDS_PORT &= ~(_BV(TXD_LED_PIN));  //txd on
	TXD_Flag = true;
	Scheduler_SetTaskMode(CDC_TXD_POV_Task, TASK_RUN); // do the pov thing
}

/*-------------------------------------------------------------------Flash_RXD()
 * Turn the RXD on and let the schedular keep in on
 *-----------------------------------------------------------------------------*/

void Flash_RXD(void)
{
//	LEDS_PORT &= ~(_BV(RXD_LED_PIN));  //rxd on
	RXD_Flag = true;
	Scheduler_SetTaskMode(CDC_RXD_POV_Task, TASK_RUN);
}	

/*---------------------------------------------------------TASK(CDC_TXD_POV_Task)
 * This task makes sure that the TXD light stays on long enough to be visible
 *-----------------------------------------------------------------------------*/
TASK(CDC_TXD_POV_Task)
{
	static SchedulerDelayCounter_t DelayCounter; 
	
    if (TXD_Flag) { 
		Scheduler_ResetDelay(&DelayCounter);
		TXD_Flag=false;
	} else if (Scheduler_HasDelayElapsed(POV_MILISECONDS, &DelayCounter)) {
		
//		LEDS_PORT |= _BV(TXD_LED_PIN);  //txd off
		Scheduler_ResetDelay(&DelayCounter);
		Scheduler_SetTaskMode(CDC_TXD_POV_Task, TASK_STOP);
	}	
}
/*---------------------------------------------------------TASK(CDC_RXD_POV_Task)
 * This task makes sure that the TXD light stays on long enough to be visible
 *-----------------------------------------------------------------------------*/

TASK(CDC_RXD_POV_Task)
{
	static SchedulerDelayCounter_t DelayCounter; 
	
    if (RXD_Flag) { 
		Scheduler_ResetDelay(&DelayCounter);
		RXD_Flag=false;
	} else if (Scheduler_HasDelayElapsed(POV_MILISECONDS, &DelayCounter)) {
//		LEDS_PORT |= _BV(RXD_LED_PIN);  //rxd off
		Scheduler_ResetDelay(&DelayCounter);
		Scheduler_SetTaskMode(CDC_RXD_POV_Task, TASK_STOP);
	}	
}

//
//	
// CC2533 programmer code
//
//	(c) Copyright 2012 Paul Campbell paul@taniwha.com
//
// Released under the LGPL Licence, Version 3
//

#include <ctype.h>

const int LED1      = 1<<1;		// PD1
#define DDR_LED1  DDRD
#define PORT_LED1  PORTD
#define PIN_LED1  PIND
const int LED2      = 1<<1;		// PB1
#define DDR_LED2  DDRB
#define PORT_LED2  PORTB
#define PIN_LED2  PINB
const int LED3      = 1<<5;		// PB5
#define DDR_LED3  DDRB
#define PORT_LED3  PORTB
#define PIN_LED3  PINB

const int CC_DC    = 1<<2;		// PB2
#define DDR_DC DDRB
#define PORT_DC  PORTB
const int CC_DD    = 1<<3;		// PB3
#define DDR_DD  DDRB
#define PORT_DD  PORTB
#define PIN_DD  PINB
const int CC_RST   = 1<<4;		// PB4
#define DDR_RST  DDRB
#define PORT_RST  PORTB

const int CC_DC2    = 1<<4;		// PD4
#define DDR_DC2 DDRD
#define PORT_DC2  PORTD
const int CC_DD2    = 1<<5;		// PD5
#define DDR_DD2  DDRD
#define PORT_DD2  PORTD
#define PIN_DD2  PIND
const int CC_RST2   = 1<<6;		// PD6
#define DDR_RST2  DDRD
#define PORT_RST2  PORTD

unsigned char prod = 1;	

void set_led(int led, int v) 
{
	switch (led) {
	case 1:
		DDR_LED1 |= LED1;
		if (v) {
			PORT_LED1 |= LED1;
		} else {
			PORT_LED1 &= ~LED1;
		}
		break;
	case 2:
		DDR_LED2 |= LED2;
		if (v) {
			PORT_LED2 |= LED2;
		} else {
			PORT_LED2 &= ~LED2;
		}
		break;
	case 3:
		DDR_LED3 |= LED3;
		if (v) {
			PORT_LED3 |= LED3;
		} else {
			PORT_LED3 &= ~LED3;
		}
		break;
	}
}

void set_DC(int v)
{
	if (prod) {
		if (v) {
			PORT_DC |= CC_DC;
		} else {
			PORT_DC &= ~CC_DC;
		}
	} else {
		if (v) {
			PORT_DC2 |= CC_DC2;
		} else {
			PORT_DC2 &= ~CC_DC2;
		}
	}
}
#define set_DC_prod(v)  if (v) { PORT_DC |= CC_DC; } else { PORT_DC &= ~CC_DC; }
#define set_DC_debug(v)  if (v) { PORT_DC2 |= CC_DC2; } else { PORT_DC2 &= ~CC_DC2; }

void dir_DD(int v)
{
	if (prod) {
		if (v) {
			DDR_DD |= CC_DD;
		} else {
			DDR_DD &= ~CC_DD;
		}
	} else {
		if (v) {
			DDR_DD2 |= CC_DD2;
		} else {
			DDR_DD2 &= ~CC_DD2;
		}
	}
}
#define dir_DD_prod(v)  if (v) { DDR_DD |= CC_DD; } else { DDR_DD &= ~CC_DD; }
#define dir_DD_debug(v)  if (v) { DDR_DD2 |= CC_DD2; } else { DDR_DD2 &= ~CC_DD2; }

void set_DD(int v)
{
	if (prod) {
		if (v) {
			PORT_DD |= CC_DD;
		} else {
			PORT_DD &= ~CC_DD;
		}
	} else {
		if (v) {
			PORT_DD2 |= CC_DD2;
		} else {
			PORT_DD2 &= ~CC_DD2;
		}
	}
}

#define set_DD_prod(v)  if (v) { PORT_DD |= CC_DD; } else { PORT_DD &= ~CC_DD; }
#define set_DD_debug(v)  if (v) { PORT_DD2 |= CC_DD2; } else { PORT_DD2 &= ~CC_DD2; }


inline void set_RST(int v)
{
	if (prod) {
		if (v) {
			DDR_RST |= CC_RST;
			PORT_RST |= CC_RST;
		} else {
			DDR_RST |= CC_RST;
			PORT_RST &= ~CC_RST;
		}
	} else {
		if (v) {
			DDR_RST2 |= CC_RST2;
			PORT_RST2 |= CC_RST2;
		} else {
			DDR_RST2 |= CC_RST2;
			PORT_RST2 &= ~CC_RST2;
		}
	}
}


#define CMD_SERIAL       'S'
#define CMD_XDATA        'M'
#define CMD_READ_CODE    'C'
#define CMD_LED          'L'
#define CMD_ENTER_DEBUG  'D'
#define CMD_EXTENDED     'X'
#define CMD_RESET        'R'
#define CMD_EXIT         'E'

#define BUFFER_SIZE      190

unsigned char inByte;
unsigned char inBuffer[BUFFER_SIZE];
unsigned char idx;
unsigned char data0;
unsigned char data1;
unsigned char data2;

unsigned char dbg_read(void);
void dbg_write(unsigned char data);
void printHex(unsigned char data);
void printHexln(unsigned char data);
unsigned char dbg_instr3(unsigned char in0, unsigned char in1, unsigned char in2);
unsigned char dbg_instr2(unsigned char in0, unsigned char in1);
unsigned char dbg_instr1(unsigned char in0);
unsigned char wr_config(unsigned char in0);
unsigned char resume(void);
unsigned char halt(void);
unsigned char step(void);
unsigned int get_pc(void);
unsigned char read_status(void);
unsigned char end_transaction(void);
unsigned int lend_transaction(void);
void sendOK(void);
void sendERROR(void);
unsigned char checkChecksum(void);
unsigned char getHexByte(unsigned char index);
unsigned char isHexByte(unsigned char index);
void dbg_enter(void);
void dbg_reset(unsigned char state);
void cc_delay( unsigned char d );
void Serial_print(PGM_P cp);

void
Serial_print(PGM_P cp)
{
	for (;;) {
		char c = pgm_read_byte_near(cp);
		cp++;
		if (c == 0)
			break;
		Buffer_StoreElement(&Tx_Buffer, c);
	}
}
void
Serial_println(void)
{
	Serial_print(PSTR("\r\n"));
}

void
Serial_printdigit(int v)
{
	if (v == 0)
		return;
	if (v >= 10) {
		Serial_printdigit(v/10);
		v = v%10;
	}
	Buffer_StoreElement(&Tx_Buffer, v+'0');
}

void
Serial_printdec(int v)
{
	if (v == 0) {
		Serial_print(PSTR("0"));
		return;
	}
	if (v < 0) {
		Serial_print(PSTR("-"));
		v=-v;
	}
	Serial_printdigit(v);
}
 
void
Serial_printhex(int v)
{
	int v2 = (v>>4)&0xf;
	v &= 0xf;
	if (v2 < 10) {
		v2 += '0';
	} else {
		v2 -= 10;
		v2 += 'A';
	}
	Buffer_StoreElement(&Tx_Buffer, v2);
	if (v < 10) {
		v += '0';
	} else {
		v -= 10;
		v += 'A';
	}
	Buffer_StoreElement(&Tx_Buffer, v);
}

void setup(void)
{
  Buffer_Initialize(&Rx_Buffer);
  Buffer_Initialize(&Tx_Buffer);
  CLKPR = 0x80;
  CLKPR = 0;
  uart_state = 0;
  set_no_pr();
}

void
set_no_pr()
{
  DDR_DC &= ~CC_DC;
  DDR_DC2 &= ~CC_DC2;
  prod = 0;
  dir_DD(0);
  prod = 1;
  dir_DD(0);
  UCSR1B = ((1 << TXCIE1) | (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}
void
set_pr()
{
  //pinMode(LED, OUTPUT);

  UCSR1B = ((1 << TXCIE1) | (1 << RXCIE1) |  (1 << RXEN1));
  DDRD &= ~(1<<3);	// dont enable TX
  DDR_DC |= CC_DC;
  DDR_DC2 |= CC_DC2;
  set_led(1, 0);
  set_led(3, 1);
  set_led(2, 0);
  prod = 0;
  set_RST(1);
  set_DD(0);
  dir_DD(1);
  set_DC(1);
  prod = 1;
  set_RST(1);
  set_DD(0);
  dir_DD(1);
  set_DC(1);
  //pinMode(CC_DC, OUTPUT);
  //digitalWrite(CC_DC, LOW);
  //pinMode(CC_DD, OUTPUT);
  //digitalWrite(CC_DD, LOW);
  //pinMode(CC_RST, OUTPUT);
  //digitalWrite(CC_RST, HIGH);  // tristate

  Serial_print(PSTR("\r\n:"));
  data1 = 0;
  idx = 0;
}


TASK(LOOP_Task)
{
  //if (Serial_available() <= 0)
    //return;
  if (!Rx_Buffer.Elements)
	return;
  if (uart_state != 0xff) {
	if (Transmitting)
		return;
  	inByte = Buffer_GetElement(&Rx_Buffer);
	if (usb_waiting) {
		usb_waiting = 0;
		Scheduler_SetTaskMode(CDC_Task, TASK_RUN);
	}
	switch (uart_state) {
	case 0:	uart_state = (inByte=='+'?1:0); break;
	case 1:	uart_state = (inByte=='+'?2:0); break;
	case 2:	uart_state = (inByte=='+'?3:0); break;
	case 3:	uart_state = (inByte=='A'?4:0); break;
	case 4:	uart_state = (inByte=='T'?5:0); break;
	case 5:	uart_state = (inByte=='R'?6:0); break;
	case 6:	if (inByte=='F') {
			uart_state = 0xff;
			set_pr();
			return;
		} else {
		 	uart_state = 0;
		}
		break;
	default: uart_state = 0; break;
	}
	Transmitting = true;
	Serial_TxByte(inByte);
	return;
  }
  inByte = Buffer_GetElement(&Rx_Buffer);
  if (usb_waiting) {
	usb_waiting = 0;
	Scheduler_SetTaskMode(CDC_Task, TASK_RUN);
  }
  //inByte = Serial_read();
  if (inByte != '\r') {
    inByte = toupper(inByte);
    if (inByte >= ' ') {
      //Serial.print(inByte);
      if (data1)
        return;
      if (idx < BUFFER_SIZE) {
        inBuffer[idx] = inByte;
        ++idx;
        return;
      }
      data1 = 1;
      return;
    }
    data1 = 2;
    return;
  }
  set_led(2, 1);
  Serial_println();
  if (!idx) {
    sendOK();
    return;
  }
  if (data1 || idx <= 2) {
    Serial_print(PSTR("BAD PACKET:"));
    printHex(idx);Serial_print(PSTR(" "));
    printHex(inByte);Serial_print(PSTR(" "));
    printHexln(data1);
    sendERROR();
    return;
  }

  inByte = checkChecksum();
  if (inByte) {
    Serial_print(PSTR("BAD CHECKSUM:"));
    printHexln(0-inByte);
    sendERROR();
    return;
  }
  // Remove checksum from length
  idx -= 2;

  switch(inBuffer[0]) {
  default:
    Serial_print(PSTR("BAD COMMAND:"));
    Serial_printdec(inBuffer[0]);
    Serial_println();
    break;
  case CMD_EXIT:
    uart_state = 0;
    set_no_pr();
    break;	
  case CMD_SERIAL:
    if (idx == 2)
      switch (inBuffer[1]){
      case '0':
        do_store = 0;
        sendOK();
        return;
      case '1':
        do_store = 1;
        sendOK();
        return;
      }
    break;
  case CMD_ENTER_DEBUG:
    if(idx == 1) {
      dbg_enter();
      sendOK();
      return;
    }
    break;
  case CMD_LED:
    if (idx == 2)
      switch (inBuffer[1]){
      case '0':
        //LED_OFF();
        sendOK();
      	set_led(2, 0);
        return;
      case '1':
        //LED_ON();
        sendOK();
        return;
      case '3':
	set_DC(1);
        //digitalWrite(CC_DC, HIGH);
        sendOK();
        return;
      case '4':
	set_DC(0);
        //digitalWrite(CC_DC, LOW);
        sendOK();
        return;
      case '5':
	dir_DD(0);
	set_DD(1);
        //pinMode(CC_DD, INPUT);
        //digitalWrite(CC_DD, HIGH);
        sendOK();
        return;
      case '6':
	dir_DD(1);
	set_DD(0);
        //pinMode(CC_DD, OUTPUT);
        //digitalWrite(CC_DD, LOW);
        sendOK();
        return;
      case '7':
	set_RST(1);
        //pinMode(CC_RST, INPUT);
        //digitalWrite(CC_RST, HIGH);
        sendOK();
        return;
      case '8':
	set_RST(0);
        //pinMode(CC_RST, OUTPUT);
        //digitalWrite(CC_RST, LOW);
        sendOK();
        return;
      }
    break;
  case CMD_RESET:
    if (idx == 2)
      switch (inBuffer[1]){
      case '0':
        dbg_reset(0);
        sendOK();
        return;
      case '1':
        dbg_reset(1);
        sendOK();
        return;
      case '2':	// switch to production interface
	prod = 1;
  	set_led(1, 0);
  	set_led(3, 1);
        sendOK();
	return;
      case '3':	// switch to debug interface
	prod = 0;
  	set_led(1, 1);
  	set_led(3, 0);
        sendOK();
	return;
      }
    break;
  case CMD_XDATA:
    if (inBuffer[1] == 'Q') {
	halt();
	dma_running = 0;
        dbg_instr3(0x90, 0x00, 00);            // MOV DPTR #0
	for (unsigned char i = 0; i < dma_size; i++) {
              dbg_instr2(0x74, pgm_read_byte(&dma_code[i]));	// MOV A, #byte
              dbg_instr1(0xF0);			// MOV @DPTR, A
              dbg_instr1(0xA3);			// INC DPTR
	}
//	dbg_instr3(0x75, 0x9f, 0x00);	// map flash to 0x8000
//	dbg_instr3(0x75, 0xc7, 0x08);	// map sram to 0x8000
	sendOK();
	return;
    }
    if (idx >= 8) {
      unsigned char i;
      if(isHexByte(2) && isHexByte(4) && isHexByte(6)) {
	if (inBuffer[1] == 'J') {
		if (dma_running) {
			end_transaction();
			dma_running = 0;
			for (i = 0; ; i++) {
				if (read_status()&0x20)	// halted?
					break;
				if (i == 200) {
					halt();
              				Serial_print(PSTR("TIMEOUT"));
              				Serial_println();
              				sendERROR();
					return;
				}
			}
		} else {
			halt();
		}
          	sendOK();
          	return;
	}
        unsigned char cnt = getHexByte(6);
        if(!cnt) {
          sendOK();
          return;
        }
        i = 8;
	if (inBuffer[1] == 'N') {
		unsigned addr = (getHexByte(2)<<8)|getHexByte(4);	// addr/4
		if ((addr&((512>>2)-1)) == 0) {
//			if (dma_running) {
//				end_transaction();
//				dma_running = 0;
//				for (i = 0; ; i++) {
//					if (read_status()&0x20)	// halted?
//						break;
//					if (i == 200) {
//						halt();
//              					Serial_print(PSTR("TIMEOUT"));
//              					Serial_println();
//              					sendERROR();
//						return;
//					}
//				}
//				i = 8;
//			} else {
//				halt();
//			}
			
			wr_config(0x22);			// enable dma

dbg_instr3(0x75, 0xD5, 0x00);  // mov     DMA0CFGH, #inp>>8               
dbg_instr3(0x75, 0xD4, 0x00);  // mov     DMA0CFGL, #inp                  
dbg_instr3(0x75, 0xD6, 0x01);  // mov     DMAARM, #1                      




#ifdef NOTDEF
//Serial_print(PSTR("START"));
//        		dbg_instr2(0x7f, addr>>8);           	// MOV r7 #high
//        		dbg_instr2(0x7e, addr);            	// MOV r6 #lo
//			wr_config(0x22);			// enable dma
//			dbg_instr3(0x02, 0x80, 0x10);	      // ljmp 16
//			resume();				// resume
//for (i=0;i<200;i++) ;
//for (i=0;i<200;i++) step();
//Serial_print(PSTR(" step pc="));
//for (i=0;i<10;i++) {
//step();
//#ifdef NOTDEF
//halt();
//{unsigned int pc=get_pc();
//Serial_printhex(pc>>8);
//Serial_printhex(pc);
//Serial_print(PSTR(" "));
//}
//Serial_println();
//sendERROR();
//return;
//#endif
//}
//Serial_print(PSTR("R"));
dbg_write(0x80);			// burst write 512
dbg_write(0x02);			
dbg_write(0x34);			
dbg_write(0x56);			
end_transaction();
halt();
dbg_instr3(0x90, 0x00, 0x63);            // MOV DPTR #high #low

i = dbg_instr1(0xe0);	// MOV A, @DPTR
Serial_printhex(i);
Serial_print(PSTR(" "));
dbg_instr1(0xA3);			// INC DPTR

i = dbg_instr1(0xe0);	// MOV A, @DPTR
Serial_printhex(i);
Serial_print(PSTR(" "));
dbg_instr1(0xA3);			// INC DPTR
//
//Serial_print(PSTR("B"));
//dbg_instr3(0x90, 0x00, 0x02);            // MOV DPTR #high #low
//
//i = dbg_instr1(0xe0);	// MOV A, @DPTR
//Serial_printhex(i);
//Serial_print(PSTR(" "));
//dbg_instr1(0xA3);			// INC DPTR
//
//i = dbg_instr1(0xe0);	// MOV A, @DPTR
//Serial_printhex(i);
//Serial_print(PSTR(" "));
//dbg_instr1(0xA3);			// INC DPTR
//
//i = dbg_instr1(0xe0);	// MOV A, @DPTR
//Serial_printhex(i);
//Serial_print(PSTR(" "));
//dbg_instr1(0xA3);			// INC DPTR
//
//i = dbg_instr1(0xe0);	// MOV A, @DPTR
//Serial_printhex(i);
//Serial_print(PSTR(" "));
//dbg_instr1(0xA3);			// INC DPTR
//
Serial_println();
sendERROR();
return;
#endif
			dbg_write(0x82);			// burst write 512
			dbg_write(0x00);			
		}
		cnt = 128;
          	while (cnt > 0) {
            		if (i + 3 >= idx) {
              			Serial_print(PSTR("NO DATA"));
              			Serial_println();
              			sendERROR();
              			return;
            		}
			unsigned char b = inBuffer[i++]-0x20;
			if (b >= 0x40) {
nohex:
              			Serial_print(PSTR("NO HEX"));
              			Serial_println();
              			sendERROR();
              			return;
            		}
			unsigned char c = b<<2;
			b = inBuffer[i++]-0x20;
			if (b >= 0x40) 
				goto nohex;
			c |= b>>4;
			dbg_write(c);			
			c = b<<4;
			b = inBuffer[i++]-0x20;
			if (b >= 0x40) 
				goto nohex;
			c |= b>>2;
			dbg_write(c);			
			if (cnt >= 3) {
				c = b<<6;
				b = inBuffer[i++]-0x20;
				if (b >= 0x40) 
					goto nohex;
				c |= b;
				dbg_write(c);			
			} else {
				break;
			}
			cnt -= 3;
          	}
		if ((addr&((512>>2)-1)) == ((512-128)>>2)) {	// last of series
			end_transaction();
			dbg_instr3(0x75, 0xD3, 0x00);	// mov     DMA1CFGH, #outp>>8              
			dbg_instr3(0x75, 0xD2, 0x08);	// mov     DMA1CFGL, #outp                 
			dbg_instr3(0x90, 0x62, 0x71);	// mov     dptr, #FADDRL                   
              		dbg_instr2(0x74, addr&0x80);	// MOV A, #addr&0x80
			dbg_instr1(0xF0);		// movx    @dptr, a
			dbg_instr1(0xA3);		// inc     dptr
              		dbg_instr2(0x74, addr>>8);	// MOV A, #addr>>8
			dbg_instr1(0xF0);		// movx    @dptr, a
			dbg_instr3(0x90, 0x62, 0x70);	// mov     dptr, #FCTL                     
			dbg_instr3(0x75, 0xD6, 0x02);	// mov     DMAARM, #2
			dbg_instr2(0x74, 0x02);		// mov     a, #FWRITE
			dbg_instr1(0xF0);		// movx    @dptr, a
		}
		sendOK();
		return;
	}
        if(cnt > 64) {
          Serial_print(PSTR("NO MORE 64 BYTES"));
          Serial_println();
          sendERROR();
          return;
        }
        dbg_instr3(0x90, getHexByte(2), getHexByte(4));            // MOV DPTR #high #low
        data1 = 1;
        //LED_TOGGLE();
        if (inBuffer[1] == 'W') {
          while (cnt-- > 0) {
            if (i + 1 >= idx) {
              Serial_print(PSTR("NO DATA"));
              Serial_println();
              sendERROR();
              return;
            }
            if (isHexByte(i)) {
              dbg_instr2(0x74, getHexByte(i));	// MOV A, #byte
              dbg_instr1(0xF0);			// MOV @DPTR, A
              dbg_instr1(0xA3);			// INC DPTR
              i += 2;
            } 
            else {
              Serial_print(PSTR("NO HEX"));
              Serial_println();
              sendERROR();
              return;
            }
          }
        } 
        else if (inBuffer[1] == 'R' || inBuffer[1] == 'C') {
          data2 = 1;
          if (inBuffer[1] == 'R')
            data2 = 0;
          Serial_print(PSTR("READ:"));
          unsigned char csum = 0;
          while (cnt > 0) {
            --cnt;
            if(data2) {
	      dbg_instr1(0xe4);		// CLR A
              data0 = dbg_instr1(0x93);	// MOVC A, @A+DPTR
            } else {
              data0 = dbg_instr1(0xe0);	// MOV A, @DPTR
            }
            dbg_instr1(0xA3);		// INC DPTR
            csum += data0;
            printHex(data0);
          }
          csum = (0 - (~csum));
          printHexln(csum);
        }
        else
          break;
        sendOK();
        return;
      }
    }
    break;
  case CMD_EXTENDED:
    //LED_TOGGLE();
    {
    unsigned char i = 1;
    unsigned char cnt;
    data1 = 1;
    while (data1 && i < idx) {
      switch (inBuffer[i++]) {
      case 'W':
        if (i < idx && isdigit(inBuffer[i])) {
          cnt = inBuffer[i++] - '0';
          if (!cnt)
            continue;
          while (cnt > 0) {
            if (i + 1 >= idx) {
              data1 = 0;
              break;
            }
            --cnt;
            if (isHexByte(i)) {
              dbg_write(getHexByte(i));
              i += 2;
            } 
            else {
              data1 = 0;
              break;
            }
          }
          if(data1)
          {
            cc_delay(10);
            continue;
          }
        }
        data1 = 0;
        break;
      case 'R':
        if (i < idx && isdigit(inBuffer[i])) {
	  int x=0;
          cnt = inBuffer[i++] - '0';
          Serial_print(PSTR("READ:"));
          unsigned char csum = 0;
  	  dir_DD(0);
  	  set_DD(0);
  	  cc_delay(20);
    	  while (prod?PIN_DD&CC_DD:PIN_DD2&CC_DD2) {
		if (x == 1000) {
			break;
		}
	        (void)dbg_read();
  		cc_delay(6);
		x++;
  	  }
          while (cnt > 0) {
            --cnt;
            data0 = dbg_read();
            csum += data0;
            printHex(data0);
          }
          csum = (0 - (~csum));
          printHexln(csum);
          continue;
        }
        data1 = 0;
        break;
      default:
        data1 = 0;
        break;
      }
    }
    //LED_TOGGLE();
    if (data1) {
      sendOK();
      return;
    }
    }
    break;
  }

  sendERROR();
}

unsigned char isHexDigit(unsigned char c) {
  if (c >= '0' && c <= '9')
    return 1;
  else if (c >= 'A' && c <= 'F')
    return 1;
  return 0;
}

unsigned char isHexByte(unsigned char index) {
  return isHexDigit(inBuffer[index]) & isHexDigit(inBuffer[index + 1]);
}

unsigned char getHexDigit(unsigned char c) {
  if (c >= '0' && c <= '9')
    c -= '0';
  else if (c >= 'A' && c <= 'F')
    c -= ('A' - 10);
  return c;
}

unsigned char getHexByte(unsigned char index) {
  return ((getHexDigit(inBuffer[index]) << 4) | getHexDigit(inBuffer[index + 1]));
}


unsigned char checkChecksum(void) {
  if (idx <= 2)
    return 0;

  unsigned char csum = 0;
  unsigned char imax = idx - 2;
  unsigned char i = 0;
  for(; i < imax; ++i)
    csum += inBuffer[i];
  csum = ~csum;
  return (csum + getHexByte(i));
}

#ifdef NOTDEF
void LED_OFF()
{
  digitalWrite(LED, LOW);
}

void LED_ON()
{
  digitalWrite(LED, HIGH);
}

void LED_TOGGLE()
{
  digitalWrite(LED, !digitalRead(LED));
}

void BlinkLED(unsigned char blinks)
{
  while(blinks-- > 0)
  {
    LED_ON();
    delay(500);
    LED_OFF();
    delay(500);
  }
  delay(1000);
}
#endif

void cc_delay( unsigned char d )
{
  volatile unsigned char i = d;
  while( i-- );
}

void dbg_clock_high(void) {
  set_DC(1);
  //digitalWrite(CC_DC, HIGH);
}

void dbg_clock_low(void) {
  set_DC(0);
  //digitalWrite(CC_DC, LOW);
}

// 1 - activate RESET (low)
// 0 - deactivate RESET (high)
void dbg_reset(unsigned char state) {
  if (state) {
    set_RST(0);
    //pinMode(CC_RST, OUTPUT);
    //digitalWrite(CC_RST, LOW);
  } 
  else {
    set_RST(1);
    //pinMode(CC_RST, INPUT);
    //digitalWrite(CC_RST, HIGH);
  }
  cc_delay(200);
}
void dbg_enter(void) {
  dbg_reset(1);
  dbg_clock_high();
  cc_delay(1);
  dbg_clock_low();
  cc_delay(1);
  dbg_clock_high();
  cc_delay(1);
  dbg_clock_low();
  cc_delay(200);
  dbg_reset(0);
}

unsigned char dbg_read(void) {
  unsigned char cnt, data=0;

  if (prod) {
  	dir_DD_prod(0);
  	set_DD_prod(0);
  	//pinMode(CC_DD, INPUT);
  	//digitalWrite(CC_DD, LOW); // no pullup
	
  	for (cnt = 8; cnt; cnt--) {
    		data <<= 1;
		set_DC_prod(1);
    		cc_delay(3);
    		if (PIN_DD&CC_DD) 
      			data |= 0x01;
		set_DC_prod(0);
    	cc_delay(3);
  	}
  } else {
  	dir_DD_debug(0);
  	set_DD_debug(0);
  	//pinMode(CC_DD, INPUT);
  	//digitalWrite(CC_DD, LOW); // no pullup
	
  	for (cnt = 8; cnt; cnt--) {
    		data <<= 1;
		set_DC_debug(1);
    		//cc_delay(3);
    		if (PIN_DD2&CC_DD2) 
      			data |= 0x01;
		set_DC_debug(0);
    	//cc_delay(3);
  	}
  }
  return data;
}

void dbg_write(unsigned char data) {
  unsigned char cnt;
  
  if (prod) {
  	dir_DD_prod(1);
  	for (cnt = 8; cnt; cnt--) {
    		if (data & 0x80) {
      			set_DD_prod(1);
      	//digitalWrite(CC_DD, HIGH);
    		} else {
      			set_DD_prod(0);
      	//digitalWrite(CC_DD, LOW);
    		}
    		data <<= 1;
    		set_DC_prod(1);
    	cc_delay(3);
	
    		set_DC_prod(0);
    	cc_delay(3);
	}
  	dir_DD_prod(1);
  	set_DD_prod(0);
  } else {
  	dir_DD_debug(1);
  	for (cnt = 8; cnt; cnt--) {
    		if (data & 0x80) {
      			set_DD_debug(1);
      	//digitalWrite(CC_DD, HIGH);
    		} else {
      			set_DD_debug(0);
      	//digitalWrite(CC_DD, LOW);
    		}
    		data <<= 1;
    		set_DC_debug(1);
    	//cc_delay(3);
	
    		set_DC_debug(0);
    	//cc_delay(3);
	}
  	dir_DD_debug(1);
  	set_DD_debug(0);
  }

  //pinMode(CC_DD, OUTPUT);
  //digitalWrite(CC_DD, LOW);
}

void printHex(unsigned char data) {
  Serial_printhex(data);
}

void printHexln(unsigned char data) {
  Serial_printhex(data);
  Serial_println();
}

unsigned int lend_transaction() 
{
  unsigned int l;
  dir_DD(0);
  set_DD(0);
  cc_delay(20);
  while (prod?PIN_DD&CC_DD:PIN_DD2&CC_DD2) {
	(void)dbg_read();
  	cc_delay(6);
  }
  l = dbg_read()<<8;
  while (prod?PIN_DD&CC_DD:PIN_DD2&CC_DD2) {
	(void)dbg_read();
  	cc_delay(6);
  }
  l |= dbg_read();
  return l;
}
unsigned char end_transaction() 
{
  dir_DD(0);
  set_DD(0);
  cc_delay(20);
  while (prod?PIN_DD&CC_DD:PIN_DD2&CC_DD2) {
	(void)dbg_read();
  	cc_delay(6);
  }
  return dbg_read();
}
unsigned char dbg_instr3(unsigned char in0, unsigned char in1, unsigned char in2) {
  dbg_write(0x57);
  dbg_write(in0);
  dbg_write(in1);
  dbg_write(in2);
  return end_transaction();
}
unsigned char dbg_instr2(unsigned char in0, unsigned char in1) {
  dbg_write(0x56);
  dbg_write(in0);
  dbg_write(in1);
  return end_transaction();
}
unsigned char dbg_instr1(unsigned char in0) {
  dbg_write(0x55);
  dbg_write(in0);
  return end_transaction();
}
unsigned char wr_config(unsigned char in0) {
  dbg_write(0x18);
  dbg_write(in0);
  return end_transaction();
}
unsigned char halt() {
  dbg_write(0x40);
  return end_transaction();
}
unsigned char resume() {
  dbg_write(0x48);
  return end_transaction();
}
unsigned char step() {
  dbg_write(0x58);
  return end_transaction();
}
unsigned int get_pc() {
  dbg_write(0x28);
  return lend_transaction();
}
unsigned char read_status() {
  dbg_write(0x30);
  return end_transaction();
}

void sendERROR(void) {
  Serial_print(PSTR("ERROR\r\n:"));
  data1 = 0;
  idx = 0;
  set_led(2, 0);
}

void sendOK(void) {
  Serial_print(PSTR("OK\r\n:"));
  data1 = 0;
  idx = 0;
  set_led(2, 0);
}
