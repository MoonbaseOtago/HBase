/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  Contains definitions for the various control request parameters, so that the request details (such as data
 *  direction, request recipient, etc.) can be extracted via masking.
 */
 
#ifndef __STDREQTYPE_H__
#define __STDREQTYPE_H__

	/* Public Interface - May be used in end-application: */
		/* Macros: */
			/** Mask for the request type parameter, to indicate the direction of the request data (Host to Device
			 *  or Device to Host). The result of this mask should then be compared to the request direction masks.
			 *
			 *  \see REQDIR_* macros for masks indicating the request data direction.
			 */
			#define CONTROL_REQTYPE_DIRECTION  0b10000000

			/** Mask for the request type parameter, to indicate the type of request (Device, Class or Vendor
			 *  Specific). The result of this mask should then be compared to the request type masks.
			 *
			 *  \see REQTYPE_* macros for masks indicating the request type.
			 */
			#define CONTROL_REQTYPE_TYPE       0b01100000

			/** Mask for the request type parameter, to indicate the recipient of the request (Standard, Class
			 *  or Vendor Specific). The result of this mask should then be compared to the request recipient
			 *  masks.
			 *
			 *  \see REQREC_* macros for masks indicating the request recipient.
			 */
			#define CONTROL_REQTYPE_RECIPIENT  0b00011111

			/** Request data direction mask, indicating that the request data will flow from host to device.
			 *
			 *  \see CONTROL_REQTYPE_DIRECTION macro.
			 */
			#define REQDIR_HOSTTODEVICE        (0 << 7)

			/** Request data direction mask, indicating that the request data will flow from device to host.
			 *
			 *  \see CONTROL_REQTYPE_DIRECTION macro.
			 */
			#define REQDIR_DEVICETOHOST        (1 << 7)

			/** Request type mask, indicating that the request is a standard request.
			 *
			 *  \see CONTROL_REQTYPE_TYPE macro.
			 */
			#define REQTYPE_STANDARD           (0 << 5)

			/** Request type mask, indicating that the request is a class-specific request.
			 *
			 *  \see CONTROL_REQTYPE_TYPE macro.
			 */
			#define REQTYPE_CLASS              (1 << 5)

			/** Request type mask, indicating that the request is a vendor specific request.
			 *
			 *  \see CONTROL_REQTYPE_TYPE macro.
			 */
			#define REQTYPE_VENDOR             (2 << 5)

			/** Request recipient mask, indicating that the request is to be issued to the device as a whole.
			 *
			 *  \see CONTROL_REQTYPE_RECIPIENT macro.
			 */
			#define REQREC_DEVICE              (0 << 0)

			/** Request recipient mask, indicating that the request is to be issued to an interface in the
			 *  currently selected configuration.
			 *
			 *  \see CONTROL_REQTYPE_RECIPIENT macro.
			 */
			#define REQREC_INTERFACE           (1 << 0)

			/** Request recipient mask, indicating that the request is to be issued to an endpoint in the
			 *  currently selected configuration.
			 *
			 *  \see CONTROL_REQTYPE_RECIPIENT macro.
			 */
			#define REQREC_ENDPOINT            (2 << 0)

			/** Request recipient mask, indicating that the request is to be issued to an unspecified element
			 *  in the currently selected configuration.
			 *
			 *  \see CONTROL_REQTYPE_RECIPIENT macro.
			 */
			#define REQREC_OTHER               (3 << 0)

		/* Enums: */
			/** Enumeration for the various standard request commands. These commands are applicable when the
			 *  request type is REQTYPE_STANDARD (with the exception of REQ_GetDescriptor, which is always
			 *  handled regardless of the request type value).
			 *
			 *  \see Chapter 9 of the USB 2.0 Specification.
			 */
			enum Control_Request_t
			{
				REQ_GetStatus           = 0, /**< Implemented in the library for device, endpoint and interface
				                              *   recipients. Passed to the user application for other recipients
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_ClearFeature        = 1, /**< Implemented in the library for device, endpoint and interface
				                              *   recipients. Passed to the user application for other recipients
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_SetFeature          = 3, /**< Implemented in the library for device, endpoint and interface
				                              *   recipients. Passed to the user application for other recipients
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_SetAddress          = 5, /**< Implemented in the library for the device recipient. Passed
				                              *   to the user application for other recipients via the
				                              *   USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_GetDescriptor       = 6, /**< Implemented in the library for all recipients and all request
				                              *   types. */
				REQ_SetDescriptor       = 7, /**< Not implemented in the library, passed to the user application
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_GetConfiguration    = 8, /**< Implemented in the library for the device recipient. Passed
				                              *   to the user application for other recipients via the
				                              *   USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_SetConfiguration    = 9, /**< Implemented in the library for the device recipient. Passed
				                              *   to the user application for other recipients via the
				                              *   USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_GetInterface        = 10, /**< Not implemented in the library, passed to the user application
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_SetInterface        = 11, /**< Not implemented in the library, passed to the user application
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
				REQ_SynchFrame          = 12, /**< Not implemented in the library, passed to the user application
				                              *   via the USB_UnhandledControlPacket() event when received in
				                              *   device mode. */
			};

/* Private Interface - For use in library only: */
	#if !defined(__DOXYGEN__)
		/* Macros: */
			#define FEATURE_ENDPOINT           0b00000000
			#define FEATURE_REMOTE_WAKEUP      0b00000001		
			#define FEATURE_SELFPOWERED        0b00000010
	#endif
	
#endif
