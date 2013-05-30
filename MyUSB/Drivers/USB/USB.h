/*
             MyUSB Library
     Copyright (C) Dean Camera, 2007.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  Master include file for the library USB functionality. This file should be included in all user projects making
 *  use of the USB portions of the library, instead of including any headers in the USB/LowLevel or USB/HighLevel
 *  directories.
 *
 *  Class specific utility files in USB/Class/ must still be included manually, as they are not normally part of
 *  the USB library unless desired by the library user.
 */

#ifndef __USB_H__
#define __USB_H__

	/* Preprocessor Checks: */
		#if (!(defined(__AVR_AT90USB1287__) || defined(__AVR_AT90USB647__)) && defined(USB_HOST_ONLY))
			#error USB_HOST_ONLY is not avaliable for the currently selected USB AVR model.
		#endif
		
		#if (!(defined(__AVR_AT90USB1287__) || defined(__AVR_AT90USB647__) ||  \
		       defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB646__) ||  \
			   defined(__AVR_AT90USB162__)  || defined(__AVR_AT90USB82__)  ||  \
			   defined(__AVR_ATmega32U4__)))
			#error The currently selected AVR model is not supported under the USB component of the MyUSB library.
		#endif
		
	/* Includes: */
		#include "LowLevel/USBMode.h"
	
		#if defined(USB_CAN_BE_HOST) || defined(__DOXYGEN__)
			#include "LowLevel/Host.h"
			#include "LowLevel/HostChapter9.h"
			#include "LowLevel/Pipe.h"
		#endif
		
		#if defined(USB_CAN_BE_DEVICE) || defined(__DOXYGEN__)
			#include "LowLevel/Device.h"
			#include "LowLevel/DevChapter9.h"
			#include "LowLevel/Endpoint.h"
		#endif
		
		#if defined(USB_CAN_BE_BOTH) || defined(__DOXYGEN__)
			#include "LowLevel/OTG.h"
		#endif

		#include "LowLevel/LowLevel.h"
		#include "HighLevel/USBTask.h"
		#include "HighLevel/USBInterrupt.h"
		#include "HighLevel/Events.h"
		#include "HighLevel/StdDescriptors.h"
		
#endif

