/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

#ifndef __USBMODE_H__
#define __USBMODE_H__

	/* Private Interface - For use in library only: */
	#if !defined(__DOXYGEN__)
		/* Macros: */
			#if ((defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB646__) ||   \
			      defined(__AVR_AT90USB162__)  || defined(__AVR_AT90USB82__)  ||   \
				  defined(__AVR_ATmega32U4__)) && !defined(USB_DEVICE_ONLY))
				#define USB_DEVICE_ONLY
			#endif
			
			#if (defined(__AVR_AT90USB162__)  || defined(__AVR_AT90USB82__))
				#define USB_LIMITED_CONTROLLER
			#else
				#define USB_FULL_CONTROLLER
			#endif			

			#if (!defined(USB_DEVICE_ONLY) && !defined(USB_HOST_ONLY))
				#define USB_CAN_BE_BOTH
				#define USB_CAN_BE_HOST
				#define USB_CAN_BE_DEVICE
			#elif defined(USB_HOST_ONLY)
				#define USB_CAN_BE_HOST
				#define USB_CurrentMode USB_MODE_HOST
			#elif defined(USB_DEVICE_ONLY)
				#define USB_CAN_BE_DEVICE
				#define USB_CurrentMode USB_MODE_DEVICE
			#endif
			
			#if (defined(USB_HOST_ONLY) && defined(USB_DEVICE_ONLY))
				#error USB_HOST_ONLY and USB_DEVICE_ONLY are mutually exclusive.
			#endif

			#if (defined(USB_RAM_DESCRIPTORS) && defined(USE_EEPROM_DESCRIPTORS))
				#error USB_RAM_DESCRIPTORS and USE_EEPROM_DESCRIPTORS are mutually exclusive.
			#endif

			#if defined(USE_STATIC_OPTIONS)
				#define USB_Options USE_STATIC_OPTIONS
			#endif
	#endif
	
#endif
