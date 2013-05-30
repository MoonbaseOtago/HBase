/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  Module for device mode request processing. This module allows for the processing of standard control
 *  requests to the default control endpoint while in device mode.
 *
 *  \see Chapter 9 of the USB 2.0 specification.
 */

#ifndef __DEVCHAPTER9_H__
#define __DEVCHAPTER9_H__

	/* Includes: */
		#include <avr/io.h>
		#include <avr/pgmspace.h>
		#include <avr/eeprom.h>
		
		#include "../HighLevel/StdDescriptors.h"
		#include "../HighLevel/Events.h"
		#include "LowLevel.h"
		#include "StdRequestType.h"

	/* Enable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			extern "C" {
		#endif

	/* Public Interface - May be used in end-application: */
		/* Global Variables: */
			/** Currently set configuration number of the device. USB devices may have several different
			 *  configurations which the host can select between; this indicates the currently selected
			 *  value, or 0 if no configuration has been selected.
			 *
			 *  \note This variable should be treated as read-only in the user application, and never manually
			 *        changed in value.
			 */
			extern uint8_t USB_ConfigurationNumber;

		/* Throwable Events: */
			/** This module raises the USB_UnhandledControlPacket event when a request to the default control
			 *  endpoint has been received, but the library does not implement an internal handler for it.
			 *
			 *  \see Events.h for more information on this event.
			 */
			RAISES_EVENT(USB_UnhandledControlPacket);

			/** This module raises the USB_ConfigurationChanged event when the host issues a REQ_SetConfiguration
			 *  device request, to change the currently selected configuration number.
			 *
			 *  \see Events.h for more information on this event.
			 */
			RAISES_EVENT(USB_ConfigurationChanged);

			/** This module raises the USB_DeviceEnumerationComplete event when the host has completed its
			 *  enumeration of the device (i.e. when a REQ_SetConfiguration request changes the current configuration
			 *  number from 0 to a non-zero value).
			 *
			 *  \see Events.h for more information on this event.
			 */
			RAISES_EVENT(USB_DeviceEnumerationComplete);
	
	/* Private Interface - For use in library only: */
	#if !defined(__DOXYGEN__)
		/* Function Prototypes: */
			void USB_Device_ProcessControlPacket(void);
			
			#if defined(INCLUDE_FROM_DEVCHAPTER9_C)
				static void USB_Device_SetAddress(void);
				static void USB_Device_SetConfiguration(void);
				static void USB_Device_GetConfiguration(void);
				static void USB_Device_GetDescriptor(void);
				static void USB_Device_GetStatus(const uint8_t bmRequestType);
				static void USB_Device_ClearSetFeature(const uint8_t bRequest, const uint8_t bmRequestType);
			#endif
	#endif

	/* Disable C linkage for C++ Compilers: */
		#if defined(__cplusplus)
			}
		#endif
		
#endif
