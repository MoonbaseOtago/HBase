/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  This file contains macros which are common to all library elements, and which may be useful in user code. It
 *  also includes other common headees, such as Atomic.h, FunctionAttributes.h and BoardTypes.h.
 */

#ifndef __COMMON_H__
#define __COMMON_H__

	/* Includes: */
		#include <avr/io.h>
		#include <stdio.h>
		#include <avr/version.h>
		
		#include "FunctionAttributes.h"
		
		#include <alloca.h>

	/* Public Interface - May be used in end-application: */
		/* Macros: */		
			/** Macro for encasing other multi-statment macros. This should be used along with an opening brace
			 *  before the start of any multi-statement macro, so that the macros contents as a whole are treated
			 *  as a discreete block and not as a list of seperate statements which may cause problems when used as
			 *  a block (such as inline IF statments).
			 */
			#define MACROS                  do

			/** Macro for encasing other multi-statment macros. This should be used along with a preceeding closing
			 *  brace at the end of any multi-statement macro, so that the macros contents as a whole are treated
			 *  as a discreete block and not as a list of seperate statements which may cause problems when used as
			 *  a block (such as inline IF statments).
			 */
			#define MACROE                  while (0)
			
			/** Defines a volatile NOP statment which cannot be optimized out by the compiler, and thus can always
			 *  be set as a breakpoint in the resulting code. Useful for debugging purposes, where the optimizer
			 *  removes/reorders code to the point where break points cannot reliably be set.
			 */
			#define JTAG_DEBUG_POINT()      asm volatile ("NOP" ::)

			/** Defines an explicit JTAG break point in the resulting binary via the ASM BREAK statment. When
			 *  a JTAG is used, this causes the program execution to halt when reached until manually resumed. */
			#define JTAG_DEBUG_BREAK()      asm volatile ("BREAK" ::)
			
			/** Macro for testing condition "x" and breaking via JTAG_DEBUG_BREAK() if the condition is false. */
			#define JTAG_DEBUG_ASSERT(x)    MACROS{ if (!(x)) { JTAG_DEBUG_BREAK(); } }MACROE

			/** Macro for testing condition "x" and writing debug data to the serial stream if false. As a
			 *  prerequisite for this macro, the serial stream should be configured via the Serial_Stream driver.
			 *
			 *  The serial output takes the form "{FILENAME}: Function {FUNCTION NAME}, Line {LINE NUMBER}: Assertion
			 *  {x} failed."
			 */
			#define SERIAL_STREAM_ASSERT(x) MACROS{ if (!(x)) { printf_P(PSTR("%s: Function \"%s\", Line %d: "   \
																"Assertion \"%s\" failed.\r\n"),   \
																__FILE__, __func__, __LINE__, #x); \
			                                } }MACROE

		/* Inline Functions: */
			/** Function for reliably setting the AVR's system clock prescaler, using inline assembly. This function
			 *  is guaranteed to operate reliably regardless of optimization setting or other compile time options. 
			 *
			 *  \param PrescalerMask   The mask of the new prescaler setting for CLKPR
			 */
			static inline void SetSystemClockPrescaler(uint8_t PrescalerMask)
			{
					uint8_t tmp = (1 << CLKPCE);
					__asm__ __volatile__ (
							"in __tmp_reg__,__SREG__" "\n\t"
							"cli" "\n\t"
							"sts %1, %0" "\n\t"
							"sts %1, %2" "\n\t"
							"out __SREG__, __tmp_reg__"
							: /* no outputs */
							: "d" (tmp),
							  "M" (_SFR_MEM_ADDR(CLKPR)),
							  "d" (PrescalerMask)
							: "r0");
			}

			/** Function to reverse the individual bits in a byte - i.e. bit 7 is moved to bit 0, bit 6 to bit 1,
			 *  etc.
			 *
			 *  \param Byte   Byte of data whose bits are to be reversed
			 */
			static inline uint8_t BitReverse(uint8_t Byte)
			{
				Byte = (((Byte & 0b11110000) >> 4) | ((Byte & 0b00001111) << 4));
				Byte = (((Byte & 0b11001100) >> 2) | ((Byte & 0b00110011) << 2));
				Byte = (((Byte & 0b10101010) >> 1) | ((Byte & 0b01010101) << 1));

				return Byte;
			}
			
			/** Function to reverse the byte ordering of the individual bytes in a 16 bit number.
			 *
			 *  \param Word   Word of data whose bytes are to be swapped
			 */
			static inline uint16_t SwapEndian_16(uint16_t Word)
			{
				uint8_t* Bytes = (uint8_t*)&Word;
			
				return (((uint16_t)Bytes[1] >> 8) | ((uint16_t)Bytes[0] << 8));
			}

			/** Function to reverse the byte ordering of the individual bytes in a 32 bit number.
			 *
			 *  \param DWord   Double word of data whose bytes are to be swapped
			 */
			static inline uint16_t SwapEndian_32(uint16_t DWord)
			{
				uint8_t* Bytes = (uint8_t*)&DWord;
			
				return (((uint32_t)Bytes[3] >> 24) | ((uint32_t)Bytes[2] >> 16) |
				        ((uint32_t)Bytes[1] << 16) | ((uint32_t)Bytes[0] << 24));
			}

			/** Function to reverse the byte ordering of the individual bytes in a n byte number.
			 *
			 *  \param Data   Pointer to a number containing an even number of bytes to be reversed
			 */
			static inline void SwapEndian_n(uint8_t* Data, uint8_t Bytes)
			{
				uint8_t Temp;
				
				while (Bytes)
				{
					Temp = *Data;
					*Data = *(Data + Bytes - 1);
					*(Data + Bytes) = Temp;

					Data++;
					Bytes -= 2;
				}
			}

#endif
