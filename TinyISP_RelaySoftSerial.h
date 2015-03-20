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

#ifndef TinyISP_RelaySoftSerial_h
#define TinyISP_RelaySoftSerial_h

#include "TinyISP_SelectBuildOptions.h"


#if RELAY_SOFT_SERIAL_ENABLED

#if ARDUINO >= 100
  #include <SoftwareSerial.h>
  typedef SoftwareSerial SerialRelay;
#else
  #include <NewSoftSerial.h>
  typedef NewSoftSerial SerialRelay;
#endif

static SerialRelay RelaySerial( 12, 14 );

#endif


#if ! RELAY_SERIAL_ENABLED

class EmptySerialRelayClass
{
public:
  void begin( unsigned long ) {}
  void end( void ) {}
  int available( void ) { return( 0 ); }
  int read( void ) { return( -1 ); }
  void write( uint8_t ) { }
};

typedef EmptySerialRelayClass SerialRelayClass;

static SerialRelayClass RelaySerial; 

#endif


#endif
