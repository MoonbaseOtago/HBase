/*
             MyUSB Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

 Released under the LGPL Licence, Version 3
*/

/** \file
 *
 *  This file contains macros for applying GCC specific attributes to functions to control various optimizer
 *  and code generation features of the compiler. Attributes may be placed in the function prototype in any
 *  order, and multiple attributes can be specified for a single function via a space seperated list.
 *
 *  \note Do not include this file directly, rather include the Common.h header file instead to gain this file's
 *        functionality.
 */

#ifndef __FUNCATTR_H__
#define __FUNCATTR_H__

	/* Preprocessor Checks: */
		#if !defined(__COMMON_H__)
			#error Do not include this file directly. Include MyUSB/Common/Common.h instead to gain this functionality.
		#endif

	/* Public Interface - May be used in end-application: */
		/* Macros: */
			/** Indicates to the compiler that the function can not ever return, so that any stack restoring or
			 *  return code may be ommited by the compiler in the resulting binary.
			 */
			#define ATTR_NO_RETURN              __attribute__ ((noreturn))

			/** Places the function in one of the initilization sections, which execute before the main function
			 *  of the application. The init function number can be specified as "x", as an integer. Refer to the
			 *  avr-libc manual for more information on the initialization sections.
			 */
			#define ATTR_INIT_SECTION(x)        __attribute__ ((naked, section (".init" #x )))
			
			/** Indicates that the function returns a value which should not be ignored by the user code. When
			 *  applied, any ignored return value from calling the function will produce a compiler warning.
			 */
			#define ATTR_WARN_UNUSED_RESULT     __attribute__ ((warn_unused_result))

			/** Indicates that the specified parameters of the function are pointers which should never be NULL.
			 *  When applied as a 1-based comma seperated list the compiler will emmit a warning if the specified
			 *  parameters are known at compiler time to be NULL at the point of calling the function.
			 */
			#define ATTR_NON_NULL_PTR_ARG(...)  __attribute__ ((nonnull (__VA_ARGS__)))

			/** Removes any preample or postample from the function. When used, the function will not have any
			 *  register or stack saving code. This should be used with caution, and when used the programmer
			 *  is responsible for maintaining stack and register integrity.
			 */
			#define ATTR_NAKED                  __attribute__ ((naked))
			
			/** Prevents the compiler from considering a specified function for inlining. When applied, the given
			 *  function will not be inlined under any circumstances.
			 */
			#define ATTR_NOINLINE               __attribute__ ((noinline))
			
			/** Indicates that the specified function is pure, in that it has no side-effects other than global
			 *  or parameter variable access.
			 */
			#define ATTR_PURE                   __attribute__ ((pure))
			
			/** Indicates that the specified function is constant, in that it has no side effects other than
			 *  parameter access.
			 */
			#define ATTR_CONST                  __attribute__ ((const))
			
			/** Marks a given function as deprecated, which produces a warning if the function is called. */
			#define ATTR_DEPRECATED             __attribute__ ((deprecated))
			
			/** Marks a function as a weak reference, which can be overridden by other functions with an
			 *  identical name (in which case the weak reference is discarded at link time).
			 */
			#define ATTR_WEAK                   __attribute__ ((weak))
			
			/** Marks a function as an alias for another function of name "x". */
			#define ATTR_ALIAS(x)               __attribute__ ((alias( #x )))
		
#endif
