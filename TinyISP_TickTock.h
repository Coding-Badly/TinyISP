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

#ifndef TinyISP_TickTock_h
#define TinyISP_TickTock_h

#include "TinyISP_SelectBuildOptions.h"


#if (TICK_TOCK_ENABLED)

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

class TickTockClass
{
public:

  void begin( uint8_t pin )
  {
    _pin = pin;
    #if ARDUINO >= 101
      pinMode( _pin, INPUT_PULLUP );
      _previous = digitalRead( _pin );
    #else
      pinMode( _pin, INPUT );
      digitalWrite( _pin, HIGH );
      _previous = digitalRead( _pin );
    #endif
  }

  bool toggled( void )
  {
    bool v;

    v = digitalRead( _pin );

    if ( v != _previous )
    {
      _previous = v;
      return( true );
    }
    return( false );
  }

  bool value( void )
  {
    return( _previous );
  }

  void end( void )
  {
    #if ARDUINO >= 101
      pinMode( _pin, INPUT );
    #else
      pinMode( _pin, INPUT );
      digitalWrite( _pin, LOW );
    #endif
  }

private:
  uint8_t _pin;
  bool _previous;
};

#else // TICK_TOCK_ENABLED

class TickTockClass
{
public:

  static inline void begin( uint8_t pin )
  {
  }

  static inline bool toggled( void )
  {
    return( false );
  }

  static inline bool value( void )
  {
    return( false );
  }

  static inline void end( void )
  {
  }
};

#endif // TICK_TOCK_ENABLED


#endif
