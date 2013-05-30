/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  This file contains constants which can be passed to the compiler (via setting the macro BOARD) in the
 *  user project makefile using the -D option to configure the library board-specific drivers.
 *
 *  \note Do not include this file directly, rather include the Common.h header file instead to gain this file's
 *        functionality.
 */
	
#ifndef __BOARDTYPES_H__
#define __BOARDTYPES_H__

	/* Preprocessor Checks: */
		#if !defined(__COMMON_H__)
			#error Do not include this file directly. Include MyUSB/Common/Common.h instead to gain this functionality.
		#endif

	/* Public Interface - May be used in end-application: */
		/* Macros: */
			/** Selects the USBKEY specific board drivers, including Dataflash, Joystick and LED drivers. */
			#define BOARD_USBKEY        0

			/** Selects the STK525 specific board drivers, including Dataflash, Joystick and LED drivers. */
			#define BOARD_STK525        1

			/** Selects the STK526 specific board drivers, including Dataflash, Joystick and LED drivers. */
			#define BOARD_STK526        2

			/** Selects the RZUSBSTICK specific board drivers, including the driver for the boards LEDs. */
			#define BOARD_RZUSBSTICK    3

			/** Selects the user-defined board drivers, which should be placed in the user project's folder
			 *  under a directory named /Board/. Each board driver should be named identically to the MyUSB
			 *  master board driver (i.e., driver in the MyUSB/Drivers/Board director) so that the library
			 *  can correctly identify it.
			 */
			#define BOARD_USER          4

#endif
