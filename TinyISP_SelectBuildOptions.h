/*==============================================================================

  TinyISP 1.0 - Turn an ATmega328[P] Arduino compatible board (like an Uno) or
  a Teensy into an In System Programmer (ISP).  Based on MegaISP and 
  ArduinoISP.

  ----------------------------------------------------------------------------

  Copyright (c) 2012 Rowdy Dog Software
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer. 
    
  * Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution. 
    
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
  POSSIBILITY OF SUCH DAMAGE.

==============================================================================*/

#ifndef TinyISP_SelectBuildOptions_h
#define TinyISP_SelectBuildOptions_h

#include <Arduino.h>


/*============================================================================*/
/* 
  DO NOT MODIFY THIS FILE.  Options should be changed in 
  _TinyISP_BuildOptions.h
*/
/*============================================================================*/


/*----------------------------------------------------------------------------*/
/* 
  Take a guess at the board based on the processor.
*/

#if defined( __AVR_ATmega328__ ) || defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega168__ )

  #define BOARD_ARDUINO_UNO                 1
  #define __AVR_ATmega328X__                1

#elif defined( __AVR_AT90USB646__ )
// fix? Check for CORE_TEENSY ?

  #define BOARD_HAS_UART                    1
  #define BOARD_TEENSY_1_PLUS               1

#elif defined( __AVR_AT90USB162__ )
// fix? Check for CORE_TEENSY ?

  #define BOARD_HAS_UART                    1
  #define BOARD_TEENSY_1                    1

#elif defined( __AVR_ATmega32U4__ )

  #if defined( CORE_TEENSY )

    #define BOARD_HAS_UART                    1
    #define BOARD_TEENSY_2                    1

    #if ! defined( ANALOG_READ_CHANNEL )
      #define ANALOG_READ_CHANNEL             11
    #endif

  #else

    // Sadly, there is no way to know if the board is a Micro or a Leonardo.  We'll assume it's a Micro.  It may make no difference.

    #define BOARD_HAS_SERIAL1                 1
    #define BOARD_ARDUINO_MICRO               1

    #define RESET                             A5
    #define RESET_NOT_SS_DDR                  DDRB
    #define RESET_NOT_SS_BIT                  0

  #endif

#elif defined( __AVR_ATmega2560__ )

  #define BOARD_ARDUINO_MEGA                1

#else

  #error No definition for the selected board.

#endif


/*----------------------------------------------------------------------------*/
/*
  SPI clock options for PROGRAMMER_SPI_CLOCK.

  SCK rules from the datasheet...

  Low:  > 2 CPU clock cycles for fck < 12 MHz, 3 CPU clock cycles for fck >= 12 MHz
  High: > 2 CPU clock cycles for fck < 12 MHz, 3 CPU clock cycles for fck >= 12 MHz

  fck      CPU time               SCK frequency             Option
  -------  ---------------------  ------------------------  ------
    8 MHz  4 / 1000000 = 0.5 us   1 / 0.5 us =       2 MHz  FAST
    1 MHz  4 / 1000000 = 4 us     1 / 4 us =       250 KHz  NORMAL
    1 MHz  4 / 1000000 = 4 us     (1 / 4 us) / 2 = 125 KHz  SAFE
  128 KHz  4 / 128000 = 31.25 us  1 / 31.25 us =    32 KHz  SLOW

  Note: fck is the clock speed of the target.
*/

#define SLOW                              4
#define SAFE                              3
#define NORMAL                            2
#define FAST                              1


/*============================================================================*/
/* Include the user's options. */
/*============================================================================*/

#include "_TinyISP_BuildOptions.h"


/*============================================================================*/
/* Provide defaults for the things the user did not specify... */
/*============================================================================*/


/*----------------------------------------------------------------------------*/
/* 
  Baud rate for communications between the host computer and this "programmer".
  19200 makes this sketch a drop-in replacement for ArduinoISP.
*/

#if ! defined( PROGRAMMER_BAUD_RATE )

  #define PROGRAMMER_BAUD_RATE  19200

#endif


/*----------------------------------------------------------------------------*/
/*
  LED provides feedback for the human.  The states are...
  
  Heartbeat - LED slowly fades brighter then darker.  Programmer is running.
  
  Error - LED blinks solid once a second.  Currently only STK500 protocol 
      problems are checked so an error indicates a problem between the host
      and the programmer.

  Flash - LED flashes full brightness and then slowly fades darker.  
      Programmer is communicating with the target.

  LED on pin 9 makes this sketch a drop-in-replacement for ArduinoISP but may 
  interfere with other optional features.
*/

#if ! defined( LED_PIN )

  #if BOARD_ARDUINO_UNO

    #define LED_PIN  9

  #elif BOARD_TEENSY_1 || BOARD_TEENSY_1_PLUS

    #define LED_PIN  0

  #elif BOARD_TEENSY_2

    #define LED_PIN  5

  #elif BOARD_ARDUINO_MICRO

    #define LED_PIN  3

  #elif BOARD_ARDUINO_MEGA

    #define LED_PIN  13

  #endif

#endif


/*----------------------------------------------------------------------------*/
/*
  SPI clock option.
*/

#if ! defined( PROGRAMMER_SPI_CLOCK )

  #define PROGRAMMER_SPI_CLOCK  SAFE

#endif


/*----------------------------------------------------------------------------*/
/*
  Knock-Bang Relay and Serial Relay options.
*/

#if ! defined( RELAY_KNOCK_BANG_ENABLED )

  #define RELAY_KNOCK_BANG_ENABLED  0

#endif

#if RELAY_KNOCK_BANG_ENABLED && RELAY_KNOCK_BANG_USE_ALTERNATE_PIN

  #if defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega328__ )

    /* The alternate pin is 7 / PD7 */

    #define KBR_DDR     DDRD
    #define KBR_PORT    PORTD
    #define KBR_PIN     PIND
    #define KBR_BIT     7

    #define KBR_PCICR   PCICR
    #define KBR_PCIFR   PCIFR
    #define KBR_PCR     2

  #elif defined( BOARD_TEENSY_2 )

    /* The alternate pin is 13 / PB4 */

    #define KBR_DDR     DDRB
    #define KBR_PORT    PORTB
    #define KBR_PIN     PINB
    #define KBR_BIT     4

    #define KBR_PCICR   PCICR
    #define KBR_PCIFR   PCIFR
    #define KBR_PCR     0

  #else
    #warning Alternate pin not defined for the selected processor.  KnockBang receive functions are not available.
    #define KNOCKBANG_RECEIVER_AVAILABLE 0
  #endif

#endif

/* 
  If this option is enabled for a board that has only one Hardware Serial port,
  "#include <SoftwareSerial.h>" must be present in the sketch.  If this option
  is not enabled, "#include <SoftwareSerial.h>" cannot be present in the 
  sketch.
*/

#if ! defined( RELAY_SERIAL_ENABLED )

  #define RELAY_SERIAL_ENABLED  0

#endif

#if ! defined( RELAY_BAUD_RATE )

  #define RELAY_BAUD_RATE  9600

#endif


/*----------------------------------------------------------------------------*/
/*
  By default should the target be held in reset after programming?
*/

#if ! defined( HOLD_TARGET_IN_RESET_BY_DEFAULT )

  #define HOLD_TARGET_IN_RESET_BY_DEFAULT  0

#endif


/*----------------------------------------------------------------------------*/
/*
  Output a "tuning signal"?  The tuning signal is a 16 millisecond high pulse.
*/

#if ! defined( TUNING_SIGNAL_ENABLED )

  #define TUNING_SIGNAL_ENABLED  0

#endif

#if ! defined( TUNING_SIGNAL_OUTPUT_LONG_PULSE )

  #define TUNING_SIGNAL_OUTPUT_LONG_PULSE  1

#endif


/*----------------------------------------------------------------------------*/
/*
  If TICK_TOCK_ENABLED is true, TinyISP provides a simple stopwatch.  When the
  pin is toggled by the target, the pin value and millis() are output.
*/

#if ! defined( TICK_TOCK_ENABLED )

  #define TICK_TOCK_ENABLED  0

#endif

#if ! defined( TICK_TOCK_PIN )

  #if BOARD_ARDUINO_UNO

    #define TICK_TOCK_PIN  8

  #elif BOARD_TEENSY_1_PLUS

    #define TICK_TOCK_PIN  17

  #elif BOARD_TEENSY_2

    #define TICK_TOCK_PIN  10

  #elif BOARD_ARDUINO_MICRO

    #define TICK_TOCK_PIN  13

  #elif BOARD_ARDUINO_MEGA

    #define TICK_TOCK_PIN  47

  #endif

#endif


/*----------------------------------------------------------------------------*/
/*
  Internal debugging.  Only available on the Teensy.  Output is sent to 
  HardwareSerial.
*/

#if ! defined( INTERNAL_DEBUGGING )

  #define INTERNAL_DEBUGGING  0

#endif


/*----------------------------------------------------------------------------*/
/*
  Aggregate options based on the individual options selected above
*/

#if (RELAY_SERIAL_ENABLED) && (BOARD_ARDUINO_UNO)
  #define RELAY_SOFT_SERIAL_ENABLED  1
#endif

#if (RELAY_SERIAL_ENABLED) && (BOARD_HAS_UART)
  #define RELAY_UART_SERIAL_ENABLED  1
#endif

#if (INTERNAL_DEBUGGING) && (BOARD_HAS_UART) && (!RELAY_SERIAL_ENABLED)
  #define INTERNAL_DEBUGGING_TO_HARDWARESERIAL  1
#endif

#if ! defined( RESET )
  #define RESET SS
#endif


#endif  // TinyISP_SelectBuildOptions_h
