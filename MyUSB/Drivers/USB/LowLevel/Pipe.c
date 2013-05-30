/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#include "USBMode.h"
#if defined(USB_CAN_BE_HOST)

#include "Pipe.h"

uint8_t USB_ControlPipeSize = PIPE_CONTROLPIPE_DEFAULT_SIZE;

void Pipe_ConfigurePipe_P(const uint8_t  PipeNum,
                          const uint16_t PipeSize,
                          const uint8_t  UPCFG0Xdata,
                          const uint8_t  UPCFG1Xdata)
{
	Pipe_SelectPipe(PipeNum);
	Pipe_EnablePipe();
	
	UPCFG0X = UPCFG0Xdata;
	UPCFG1X = ((UPCFG1X & (1 << ALLOC)) | UPCFG1Xdata | Pipe_BytesToEPSizeMask(PipeSize));
	UPCFG2X = 0;
	
	Pipe_AllocateMemory();
}

void Pipe_ClearPipes(void)
{
	UPINT = 0;

	for (uint8_t PNum = 0; PNum < PIPE_MAX_PIPES; PNum++)
	{
		Pipe_ResetPipe(PNum);
		Pipe_SelectPipe(PNum);
		UPIENX = 0;
		UPINTX = 0;
		Pipe_ClearError();
		Pipe_ClearErrorFlags();
		Pipe_DeallocateMemory();
		Pipe_DisablePipe();
	}
}

uint8_t Pipe_Write_Stream_LE(void* Data, uint16_t Length)
{
	uint8_t* DataStream = (uint8_t*)Data;
	
	while (Length)
	{
		if (!(Pipe_ReadWriteAllowed()))
		{
			Pipe_ClearCurrentBank();
				
			while (!(Pipe_ReadWriteAllowed()))
			{
				if (!(USB_IsConnected))
				  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
				else if (Pipe_IsStalled())
				  return PIPE_RWSTREAM_ERROR_PipeStalled;
			}
		}

		Pipe_Write_Byte(*(DataStream++));
		Length--;
		
		if (!(USB_IsConnected))
		  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
		else if (Pipe_IsStalled())
		  return PIPE_RWSTREAM_ERROR_PipeStalled;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t Pipe_Write_Stream_BE(void* Data, uint16_t Length)
{
	uint8_t* DataStream = (uint8_t*)(Data + Length - 1);
	
	while (Length)
	{
		if (!(Pipe_ReadWriteAllowed()))
		{
			Pipe_ClearCurrentBank();
				
			while (!(Pipe_ReadWriteAllowed()))
			{
				if (!(USB_IsConnected))
				  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
				else if (Pipe_IsStalled())
				  return PIPE_RWSTREAM_ERROR_PipeStalled;
			}
		}

		Pipe_Write_Byte(*(DataStream--));
		Length--;
		
		if (!(USB_IsConnected))
		  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
		else if (Pipe_IsStalled())
		  return PIPE_RWSTREAM_ERROR_PipeStalled;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t Pipe_Read_Stream_LE(void* Buffer, uint16_t Length)
{
	uint8_t* DataStream = (uint8_t*)Buffer;
	
	while (Length)
	{
		if (!(Pipe_ReadWriteAllowed()))
		{
			Pipe_ClearCurrentBank();
				
			while (!(Pipe_ReadWriteAllowed()))
			{
				if (!(USB_IsConnected))
				  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
				else if (Pipe_IsStalled())
				  return PIPE_RWSTREAM_ERROR_PipeStalled;
			}
		}

		*(DataStream++) = Pipe_Read_Byte();
		Length--;
		
		if (!(USB_IsConnected))
		  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
		else if (Pipe_IsStalled())
		  return PIPE_RWSTREAM_ERROR_PipeStalled;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

uint8_t Pipe_Read_Stream_BE(void* Buffer, uint16_t Length)
{
	uint8_t* DataStream = (uint8_t*)(Buffer + Length - 1);
	
	while (Length)
	{
		if (!(Pipe_ReadWriteAllowed()))
		{
			Pipe_ClearCurrentBank();
				
			while (!(Pipe_ReadWriteAllowed()))
			{
				if (!(USB_IsConnected))
				  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
				else if (Pipe_IsStalled())
				  return PIPE_RWSTREAM_ERROR_PipeStalled;
			}
		}

		*(DataStream--) = Pipe_Read_Byte();
		Length--;
		
		if (!(USB_IsConnected))
		  return PIPE_RWSTREAM_ERROR_DeviceDisconnected;
		else if (Pipe_IsStalled())
		  return PIPE_RWSTREAM_ERROR_PipeStalled;
	}
	
	return PIPE_RWSTREAM_ERROR_NoError;
}

#endif
