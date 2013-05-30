HBase
=====

Firmware for CC2533 programmer - based on Benito serial firmware - driven by CCLoad program

// CC2533 programmer code
//
//	(c) Copyright 2012 Paul Campbell paul@taniwha.com
//
// Released under the LGPL Licence, Version 3
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


To load code into a programmer:

- 'make' here to build a binary
- erase the programmer (short the 'D7' pins, then short the 'RESET' pins, remove the shorts)
- run the "load" script here (you will need the dfu-programmer tools)
- unplug the programmer and plug it back in
