/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef __PIPE_H__
#define __PIPE_H__

	/* Includes: */
		#include <avr/io.h>
		#include <stdbool.h>

		#include "../../../Common/Common.h"
		#include "../HighLevel/USBTask.h"

	/* Enable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			extern "C" {
		#endif

	/* Public Interface - May be used in end-application: */
		/* Macros: */
			/** Mask for Pipe_GetErrorFlags(), indicating that a CRC error occurred in the pipe on the received data. */
			#define PIPE_ERRORFLAG_CRC16                   (1 << 4)

			/** Mask for Pipe_GetErrorFlags(), indicating that a hardware timeout error occurred in the pipe. */
			#define PIPE_ERRORFLAG_TIMEOUT                 (1 << 3)

			/** Mask for Pipe_GetErrorFlags(), indicating that a hardware PID error occurred in the pipe. */
			#define PIPE_ERRORFLAG_PID                     (1 << 2)

			/** Mask for Pipe_GetErrorFlags(), indicating that a hardware data PID error occurred in the pipe. */
			#define PIPE_ERRORFLAG_DATAPID                 (1 << 1)

			/** Mask for Pipe_GetErrorFlags(), indicating that a hardware data toggle error occurred in the pipe. */
			#define PIPE_ERRORFLAG_DATATGL                 (1 << 0)

			/** Token mask for Pipe_ConfigurePipe(). This sets the pipe as a SETUP token (for CONTROL type pipes),
			 *  which will trigger a control request on the attached device when data is written to the pipe.
			 */
			#define PIPE_TOKEN_SETUP                       (0b00 << PTOKEN0)

			/** Token mask for Pipe_ConfigurePipe(). This sets the pipe as a IN token (for non-CONTROL type pipes),
			 *  indicating that the pipe data will flow from device to host.
			 */
			#define PIPE_TOKEN_IN                          (0b01 << PTOKEN0)

			/** Token mask for Pipe_ConfigurePipe(). This sets the pipe as a IN token (for non-CONTROL type pipes),
			 *  indicating that the pipe data will flow from host to device.
			 */
			#define PIPE_TOKEN_OUT                         (0b10 << PTOKEN0)

			/** Mask for the bank mode selection for the Pipe_ConfigurePipe() macro. This indicates that the pipe
			 *  should have one single bank, which requires less USB FIFO memory but results in slower transfers as
			 *  only one USB device (the AVR or the attached device) can access the pipe's bank at the one time.
			 */
			#define PIPE_BANK_SINGLE                       0

			/** Mask for the bank mode selection for the Pipe_ConfigurePipe() macro. This indicates that the pipe
			 *  should have two banks, which requires more USB FIFO memory but results in faster transfers as one
			 *  USB device (the AVR or the attached device) can access one bank while the other accesses the second
			 *  bank.
			 */
			#define PIPE_BANK_DOUBLE                       (1 << EPBK0)
			
			/** Pipe address for the default control pipe, which always resides in address 0. This is
			 *  defined for convenience to give more readable code when used with the pipe macros.
			 */
			#define PIPE_CONTROLPIPE                       0

			/** Default size of the default control pipe's bank, until altered by the Endpoint0Size value 
			 *  in the device descriptor of the attached device.
			 */
			#define PIPE_CONTROLPIPE_DEFAULT_SIZE          8
			
			/** Pipe number mask, for masking against pipe addresses to retrieve the pipe's numerical address
			 *  in the device.
			 */
			#define PIPE_PIPENUM_MASK                      0b111

			/** Total number of pipes (including the default control pipe at address 0) which may be used in
			 *  the device. Different USB AVR models support different amounts of pipes, this value reflects
			 *  the maximum number of pipes for the currently selected AVR model.
			 */
			#define PIPE_MAX_PIPES                         7

			/** Size in bytes of the largest pipe bank size possible in the device. Not all banks on each AVR
			 *  model supports the largest bank size possible on the device; different pipe numbers support
			 *  different maximum bank sizes. This value reflects the largest possible bank of any pipe on the
			 *  currently selected USB AVR model.
			 */
			#define PIPE_MAX_SIZE                          256

			/** Endpoint number mask, for masking against endpoint addresses to retrieve the endpoint's
			 *  numerical address in the attached device.
			 */
			#define PIPE_EPNUM_MASK                        0b111

			/** Endpoint bank size mask, for masking against endpoint addresses to retrieve the endpoint's
			 *  bank size in the attached device.
			 */
			#define PIPE_EPSIZE_MASK                       0x7FF

			/** Interrupt definition for the pipe IN interrupt (for INTERRUPT type pipes). Should be used with
			 *  the USB_INT_* macros located in USBInterrupt.h.
			 *
			 *  This interrupt will fire if enabled on an INTERRUPT type pipe if a the pipe interrupt period has
			 *  elapsed and the pipe is ready for the next packet from the attached device to be read out from its
			 *  FIFO buffer (if received).
			 */
			#define PIPE_INT_IN                            UPIENX, (1 << RXINE) , UPINTX, (1 << RXINI)

			/** Interrupt definition for the pipe OUT interrupt (for INTERRUPT type pipes). Should be used with
			 *  the USB_INT_* macros located in USBInterrupt.h.
			 *
			 *  This interrupt will fire if enabled on an INTERRUPT type endpoint if a the pipe interrupt period
			 *  has elapsed and the pipe is ready for a packet to be written to the pipe's FIFO buffer and sent
			 *  to the attached device (if required).
			 */
			#define PIPE_INT_OUT                           UPIENX, (1 << TXOUTE), UPINTX, (1 << TXOUTI)

			/** Indicates the number of bytes currently stored in the current pipe's selected bank. */
			#define Pipe_BytesInPipe()                     UPBCX

			/** Resets the desired pipe, including the pipe banks and flags. */
			#define Pipe_ResetPipe(pipenum)        MACROS{ UPRST    =  (1 << pipenum); UPRST = 0;                  }MACROE

			/** Selects the given pipe number. Any pipe operations which do not require the pipe number to be
			 *  indicated will operate on the currently selected pipe.
			 */
			#define Pipe_SelectPipe(pipenum)       MACROS{ UPNUM    =  pipenum;                                    }MACROE

			/** Returns the pipe address of the currently selected pipe. This is typically used to save the
			 *  currently selected pipe number so that it can be restored after another pipe has been manipulated.
			 */
			#define Pipe_GetCurrentPipe()                 (UPNUM   &   PIPE_PIPENUM_MASK)

			/** Enables the currently selected pipe so that data can be sent and received through it to and from
			 *  an attached device.
			 *
			 *  \note Pipes must first be configured properly rather than just being enabled via the
			 *        Pipe_ConfigurePipe() macro, which calls Pipe_EnablePipe() automatically.
			 */
			#define Pipe_EnablePipe()              MACROS{ UPCONX  |=  (1 << PEN);                                 }MACROE

			/** Disables the currently selected pipe so that data cannot be sent and received through it to and
			 *  from an attached device.
			 */
			#define Pipe_DisablePipe()             MACROS{ UPCONX  &= ~(1 << PEN);                                 }MACROE

			/** Returns true if the currently selected pipe is enabled, false otherwise. */
			#define Pipe_IsEnabled()                     ((UPCONX  &   (1 << PEN)) ? true : false)

			/** Sets the token for the currently selected endpoint to one of the tokens specified by the PIPE_TOKEN_*
			 *  masks. This should only be used on CONTROL type endpoints, to allow for bidirectional transfer of
			 *  data during control requests.
			 */
			#define Pipe_SetToken(token)           MACROS{ UPCFG0X  = ((UPCFG0X & ~PIPE_TOKEN_MASK) | token);      }MACROE
			
			/** Configures the currently selected pipe to allow for an unlimited number of IN requests. */
			#define Pipe_SetInfiniteINRequests()   MACROS{ UPCONX  |=  (1 << INMODE);                              }MACROE

			/** Configures the currently selected pipe to only allow the specified number of IN requests to be
			 *  accepted by the pipe before it is automatically frozen.
			 */
			#define Pipe_SetFiniteINRequests(n)    MACROS{ UPCONX  &= ~(1 << INMODE); UPINRQX = n;                 }MACROE
			
			/** Configures the specified pipe number with the given pipe type, token, target endpoint number in the
			 *  attached device, bank size and banking mode. Pipes should be allocated in ascending order by their
			 *  address in the device (i.e. pipe 1 should be configured before pipe 2 and so on).
			 *
			 *  The pipe type may be one of the EP_TYPE_* macros listed in LowLevel.h, the token may be one of the
			 *  PIPE_TOKEN_* masks.
			 *
			 *  The bank size must indicate the maximum packet size that the pipe can handle. Different pipe
			 *  numbers can handle different maximum packet sizes - refer to the chosen USB AVR's datasheet to
			 *  determine the maximum bank size for each pipe.
			 *
			 *  The banking mode may be either PIPE_BANK_SINGLE or PIPE_BANK_DOUBLE.
			 *
			 *  The success of this routine can be determined via the Pipe_IsConfigured() macro. A newly configured
			 *  pipe is frozen by default, and must be unfrozen before use via the Pipe_Unfreeze() macro.
			 *
			 *  \note This routine will select the specified pipe, and the pipe will remain selected once the
			 *        routine completes regardless of if the pipe configuration succeeds.
			 */			
			#define Pipe_ConfigurePipe(num, type, token, epnum, size, banks)                 \
												   MACROS{ Pipe_ConfigurePipe_P(num, size,   \
																			  ((type << PTYPE0) | token | ((epnum & PIPE_EPNUM_MASK) << PEPNUM0)),   \
																			  banks); }MACROE

			/** Returns true if the currently selected pipe is configured, false otherwise. */
			#define Pipe_IsConfigured()                  ((UPSTAX  & (1 << CFGOK)) ? true : false)

			/** Sets the period between interrupts for an INTERRUPT type pipe to a specified number of milliseconds. */
			#define Pipe_SetInterruptPeriod(ms)    MACROS{ UPCFG2X  = ms;                                          }MACROE

			/** Returns a mask indicating which pipe's interrupt periods have elapsed, indicating that the pipe should
			 *  be serviced.
			 */
			#define Pipe_GetPipeInterrupts()               UPINT

			/** Clears the interrupt flag for the specified pipe number. */
			#define Pipe_ClearPipeInterrupt(n)     MACROS{ UPINT   &= ~(1 << n);                                   }MACROE

			/** Returns true if the specified pipe's interrupt period has elapsed, false otherwise. */
			#define Pipe_HasPipeInterrupted(n)           ((UPINT   &   (1 << n)) ? true : false)
			
			/** Clears the pipe bank, and switches to the alternate bank if the currently selected pipe is
			 *  dual-banked. When cleared, this either frees the bank up for the next packet from the host
			 *  (if the endpoint is of the OUT direction) or sends the packet contents to the host (if the
			 *  pipe is of the IN direction).
			 */
			#define Pipe_ClearCurrentBank()        MACROS{ UPINTX  &= ~(1 << FIFOCON);                             }MACROE

			/** Unfreezes the pipe, allowing it to communicate with an attached device. */
			#define Pipe_Unfreeze()                MACROS{ UPCONX  &= ~(1 << PFREEZE);                             }MACROE

			/** Freezes the pipe, preventing it from communicating with an attached device. */
			#define Pipe_Freeze()                  MACROS{ UPCONX  |=  (1 << PFREEZE);                             }MACROE

			/** Clears the master pipe error flag. */
			#define Pipe_ClearError()              MACROS{ UPINTX  &= ~(1 << PERRI);                               }MACROE

			/** Returns true if the master pipe error flag is set for the currently selected pipe, indicating that
			 *  some sort of hardware error has occurred on the pipe.
			 *
			 *  \see Pipe_GetErrorFlags() macro for information on retreiving the exact error flag.
			 */
			#define Pipe_IsError()                       ((UPINTX  &   (1 << PERRI)) ? true : false)
			
			/** Clears all the currently selected pipe's hardware error flags, but does not clear the master error
			 *  flag for the pipe. */
			#define Pipe_ClearErrorFlags()         MACROS{ UPERRX   = 0;                                           }MACROE

			/** Returns a mask of the hardware error flags which have occured on the currently selected pipe. This
			 *  value can then be masked against the PIPE_ERRORFLAG_* masks to determine what error has occurred.
			 */
			#define Pipe_GetErrorFlags()                   UPERRX

			/** Returns true if the currently selected pipe may be read from (if data is waiting in the pipe
			 *  bank and the pipe is an IN direction, or if the bank is not yet full if the pipe is an OUT
			 *  direction). This function will return false if an error has occured in the pipe, or if the pipe
			 *  is an IN direction and no packet has been received, or if the pipe is an OUT direction and the
			 *  pipe bank is full.
			 */
			#define Pipe_ReadWriteAllowed()              ((UPINTX  & (1 << RWAL)) ? true : false)

			/** Clears the flag indicating that a SETUP request has been sent to the attached device from the
			 *  currently selected CONTROL type pipe.
			 */
			#define Pipe_ClearSetupSent()          MACROS{ UPINTX  &= ~(1 << TXSTPI);                              }MACROE

			/** Returns true if no SETUP request is currently being sent to the attached device, false otherwise. */
			#define Pipe_IsSetupSent()                   ((UPINTX  &   (1 << TXSTPI)) ? true : false)

			/** Returns true if the currently selected pipe has been stalled by the attached device, false otherwise. */
			#define Pipe_IsStalled()                     ((UPINTX  &   (1 << RXSTALLI)) ? true : false)

			/** Clears the stall condition on the currently selected pipe. */
			#define Pipe_ClearStall()              MACROS{ UPINTX  &= ~(1 << RXSTALLI);                            }MACROE             

			/** Returns true if an IN request has been received on the currently selected CONTROL type pipe, false
			 *  otherwise. */
			#define Pipe_IsSetupINReceived()             ((UPINTX  &   (1 << RXINI)) ? true : false)

			/** Returns true if the currently selected CONTROL type pipe is ready to send an OUT request, false
			 *  otherwise. */
			#define Pipe_IsSetupOUTReady()               ((UPINTX  &   (1 << TXOUTI)) ? true : false)

			/** Acknowedges the reception of a setup IN request from the attached device on the currently selected
			 *  CONTROL type endpoint, allowing for the transmission of a setup OUT packet, or the reception of
			 *  another setup IN packet.
			 */
			#define Pipe_ClearSetupIN()            MACROS{ UPINTX  &= ~(1 << RXINI); UPINTX &= ~(1 << FIFOCON);    }MACROE

			/** Sends the currently selected CONTROL type pipe's contents to the device as a setup OUT packet. */
			#define Pipe_ClearSetupOUT()           MACROS{ UPINTX  &= ~(1 << TXOUTI); UPINTX &= ~(1 << FIFOCON);   }MACROE

		/* Enums: */
			/** Enum for the possible error return codes of the Pipe_*_Stream_* functions. */
			enum Pipe_Stream_RW_ErrorCodes_t
			{
				PIPE_RWSTREAM_ERROR_NoError            = 0, /**< Command completed successfully, no error. */
				PIPE_RWSTREAM_ERROR_PipeStalled        = 1, /**< The device stalled the pipe during the transfer. */		
				PIPE_RWSTREAM_ERROR_DeviceDisconnected = 2, /**< Device was disconnected from the host during
			                                                 *   the transfer.
			                                                 */
			};

		/* Inline Functions: */
			/** Reads one byte from the currently selected pipe's bank, for OUT direction pipes. */
			static inline uint8_t Pipe_Read_Byte(void) ATTR_WARN_UNUSED_RESULT;
			static inline uint8_t Pipe_Read_Byte(void)
			{
				return UPDATX;
			}

			/** Writes one byte from the currently selected pipe's bank, for IN direction pipes. */
			static inline void Pipe_Write_Byte(const uint8_t Byte)
			{
				UPDATX = Byte;
			}

			/** Discards one byte from the currently selected pipe's bank, for OUT direction pipes. */
			static inline void Pipe_Discard_Byte(void)
			{
				uint8_t Dummy;
				
				Dummy = UPDATX;
			}
			
			/** Reads two bytes from the currently selected pipe's bank in little endian format, for OUT
			 *  direction pipes.
			 */
			static inline uint16_t Pipe_Read_Word_LE(void) ATTR_WARN_UNUSED_RESULT;
			static inline uint16_t Pipe_Read_Word_LE(void)
			{
				uint16_t Data;
				
				Data  = UPDATX;
				Data |= (((uint16_t)UPDATX) << 8);
			
				return Data;
			}

			/** Reads two bytes from the currently selected pipe's bank in big endian format, for OUT
			 *  direction pipes.
			 */
			static inline uint16_t Pipe_Read_Word_BE(void) ATTR_WARN_UNUSED_RESULT;
			static inline uint16_t Pipe_Read_Word_BE(void)
			{
				uint16_t Data;
				
				Data  = (((uint16_t)UPDATX) << 8);
				Data |= UPDATX;
			
				return Data;
			}
			
			/** Writes two bytes to the currently selected pipe's bank in little endian format, for IN
			 *  direction pipes.
			 */
			static inline void Pipe_Write_Word_LE(const uint16_t Word)
			{
				UPDATX = (Word & 0xFF);
				UPDATX = (Word >> 8);
			}
			
			/** Writes two bytes to the currently selected pipe's bank in big endian format, for IN
			 *  direction pipes.
			 */
			static inline void Pipe_Write_Word_BE(const uint16_t Word)
			{
				UPDATX = (Word >> 8);
				UPDATX = (Word & 0xFF);
			}

			/** Discards two bytes from the currently selected pipe's bank, for OUT direction pipes. */
			static inline void Pipe_Ignore_Word(void)
			{
				uint8_t Dummy;
				
				Dummy = UPDATX;
				Dummy = UPDATX;
			}

			/** Reads four bytes from the currently selected pipe's bank in little endian format, for OUT
			 *  direction pipes.
			 */
			static inline uint32_t Pipe_Read_DWord_LE(void) ATTR_WARN_UNUSED_RESULT;
			static inline uint32_t Pipe_Read_DWord_LE(void)
			{
				union
				{
					uint32_t DWord;
					uint8_t  Bytes[4];
				} Data;
				
				Data.Bytes[0] = UPDATX;
				Data.Bytes[1] = UPDATX;
				Data.Bytes[2] = UPDATX;
				Data.Bytes[3] = UPDATX;
			
				return Data.DWord;
			}

			/** Reads four bytes from the currently selected pipe's bank in big endian format, for OUT
			 *  direction pipes.
			 */
			static inline uint32_t Pipe_Read_DWord_BE(void) ATTR_WARN_UNUSED_RESULT;
			static inline uint32_t Pipe_Read_DWord_BE(void)
			{
				union
				{
					uint32_t DWord;
					uint8_t  Bytes[4];
				} Data;
				
				Data.Bytes[3] = UPDATX;
				Data.Bytes[2] = UPDATX;
				Data.Bytes[1] = UPDATX;
				Data.Bytes[0] = UPDATX;
			
				return Data.DWord;
			}

			/** Writes four bytes to the currently selected pipe's bank in little endian format, for IN
			 *  direction pipes.
			 */
			static inline void Pipe_Write_DWord_LE(const uint32_t DWord)
			{
				Pipe_Write_Word_LE(DWord);
				Pipe_Write_Word_LE(DWord >> 16);
			}
			
			/** Writes four bytes to the currently selected pipe's bank in big endian format, for IN
			 *  direction pipes.
			 */			
			static inline void Pipe_Write_DWord_BE(const uint32_t DWord)
			{
				Pipe_Write_Word_BE(DWord >> 16);
				Pipe_Write_Word_BE(DWord);
			}			
			
			/** Discards four bytes from the currently selected pipe's bank, for OUT direction pipes. */
			static inline void Pipe_Ignore_DWord(void)
			{
				uint8_t Dummy;
				
				Dummy = UPDATX;
				Dummy = UPDATX;
				Dummy = UPDATX;
				Dummy = UPDATX;
			}

		/* External Variables: */
			/** Global indicating the maximum packet size of the default control pipe located at address
			 *  0 in the device. This value is set to the value indicated in the attached device's device
		     *  descriptor once the USB interface is initialized into host mode and a device is attached
			 *  to the USB bus.
			 *
			 *  \note This variable should be treated as read-only in the user application, and never manually
			 *        changed in value.
			 */
			extern uint8_t USB_ControlPipeSize;

		/* Function Prototypes: */
			/** Writes the given number of bytes to the pipe from the given buffer in little endian,
			 *  sending full packets to the device as needed. The last packet filled is not automatically sent;
			 *  the user is responsible for manually sending the last written packet to the host via the
			 *  Pipe_ClearCurrentBank() macro.
			 *
			 *  \param Buffer  Pointer to the buffer to write the received bytes to.
			 *  \param Length  Number of bytes to read for the currently selected pipe into the buffer.
			 *
			 *  \return A value from the Pipe_Stream_RW_ErrorCodes_t enum.
			 */
			uint8_t Pipe_Write_Stream_LE(void* Data, uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);

			/** Writes the given number of bytes to the pipe from the given buffer in big endian,
			 *  sending full packets to the device as needed. The last packet filled is not automatically sent;
			 *  the user is responsible for manually sending the last written packet to the host via the
			 *  Pipe_ClearCurrentBank() macro.
			 *
			 *  \param Buffer  Pointer to the buffer to write the received bytes to.
			 *  \param Length  Number of bytes to read for the currently selected pipe into the buffer.
			 *
			 *  \return A value from the Pipe_Stream_RW_ErrorCodes_t enum.
			 */
			uint8_t Pipe_Write_Stream_BE(void* Data, uint16_t Length) ATTR_NON_NULL_PTR_ARG(1);
			
			/** Reads the given number of bytes from the pipe from the given buffer in little endian,
			 *  discarding fully read packets from the host as needed. The last packet is not automatically
			 *  discarded once the remaining bytes has been read; the user is responsible for manually
			 *  discarding the last packet from the host via the Pipe_ClearCurrentBank() macro.
			 *
			 *  \param Buffer  Pointer to the buffer to read the bytes to send from.
			 *  \param Length  Number of bytes to send via the currently selected pipe.
			 *
			 *  \return A value from the Pipe_Stream_RW_ErrorCodes_t enum.
			 */
			uint8_t Pipe_Read_Stream_LE(void* Data, uint16_t Length)  ATTR_NON_NULL_PTR_ARG(1);

			/** Reads the given number of bytes from the pipe from the given buffer in big endian,
			 *  discarding fully read packets from the host as needed. The last packet is not automatically
			 *  discarded once the remaining bytes has been read; the user is responsible for manually
			 *  discarding the last packet from the host via the Pipe_ClearCurrentBank() macro.
			 *
			 *  \param Buffer  Pointer to the buffer to read the bytes to send from.
			 *  \param Length  Number of bytes to send via the currently selected pipe.
			 *
			 *  \return A value from the Pipe_Stream_RW_ErrorCodes_t enum.
			 */
			uint8_t Pipe_Read_Stream_BE(void* Data, uint16_t Length)  ATTR_NON_NULL_PTR_ARG(1);
		
		/* Function Aliases: */
			/** Alias for Pipe_Discard_Byte().
			 */
			#define Pipe_Ignore_Byte()                 Pipe_Discard_Byte()

			/** Alias for Pipe_Discard_Word().
			 */
			#define Pipe_Ignore_Word()                 Pipe_Discard_Word()		

			/** Alias for Pipe_Discard_DWord().
			 */
			#define Pipe_Ignore_DWord()                Pipe_Discard_DWord()

			/** Alias for Pipe_Read_Word_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Read_Word()                   Pipe_Read_Word_LE()

			/** Alias for Pipe_Write_Word_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Write_Word(Word)              Pipe_Write_Word_LE(Word)

			/** Alias for Pipe_Read_DWord_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Read_DWord()                  Pipe_Read_DWord_LE()

			/** Alias for Pipe_Write_DWord_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Write_DWord(DWord)            Pipe_Write_DWord_LE(DWord)

			/** Alias for Pipe_Read_Stream_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Read_Stream(Buffer, Length)   Pipe_Read_Stream_LE(Buffer, Length)

			/** Alias for Pipe_Write_Stream_LE(). By default USB transfers use little endian format, thus
			 *  the command with no endianness specifier indicates little endian mode.
			 */
			#define Pipe_Write_Stream(Data, Length)    Pipe_Write_Stream_LE(Data, Length)

	/* Private Interface - For use in library only: */
	#if !defined(__DOXYGEN__)
		/* Macros: */
			#define PIPE_TOKEN_MASK                    (0b11 << PTOKEN0)

			#define Pipe_AllocateMemory()          MACROS{ UPCFG1X |=  (1 << ALLOC);                               }MACROE
			#define Pipe_DeallocateMemory()        MACROS{ UPCFG1X &= ~(1 << ALLOC);                               }MACROE

		/* Inline Functions: */
			static inline uint8_t Pipe_BytesToEPSizeMask(uint16_t Bytes)
			                                             ATTR_WARN_UNUSED_RESULT ATTR_CONST;
			static inline uint8_t Pipe_BytesToEPSizeMask(uint16_t Bytes)
			{
				Bytes &= PIPE_EPSIZE_MASK;
			
				if (Bytes <= 8)
				  return (0 << EPSIZE0);
				else if (Bytes <= 16)
				  return (1 << EPSIZE0);
				else if (Bytes <= 32)
				  return (2 << EPSIZE0);
				else if (Bytes <= 64)
				  return (3 << EPSIZE0);
				else if (Bytes <= (8 << 4))
				  return (4 << EPSIZE0);
				else
				  return (5 << EPSIZE0);
			};
		
		/* Function Prototypes: */
			void Pipe_ClearPipes(void);
			void Pipe_ConfigurePipe_P(const uint8_t  PipeNum,
			                          const uint16_t PipeSize,
			                          const uint8_t  UPCFG0Xdata,
			                          const uint8_t  UPCFG1Xdata);
	#endif

	/* Disable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			}
		#endif
		
#endif
