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

#ifndef TinyISP_RelayKnockBang_h
#define TinyISP_RelayKnockBang_h

#include "TinyISP_SelectBuildOptions.h"


#if RELAY_KNOCK_BANG_ENABLED

#include "KnockBangParser.h"
#include "KnockBangReceiver.h"

class KnockBangRelayClass
{
public:

  static void begin( void ) 
  {
    TinyDebugParser.reset();
    KnockBangReceive_begin();
  }
  
  static void end( void ) 
  {
    KnockBangReceive_end();
  }

  static bool process( void ) 
  {
    return( TinyDebugParser.process() );
  }

  static uint8_t lastFault( void )
  {
    return( KnockBangReceive_getLastFault() );
  }
};

#else // RELAY_KNOCK_BANG_ENABLED

class EmptyKnockBangRelayClass
{
public:
  static void begin( void ) {}
  static void end( void ) {}
  static bool process( void ) { return( false ); }
  static uint8_t lastFault( void ) { return( 0 ); }
};

typedef EmptyKnockBangRelayClass KnockBangRelayClass;

#endif // RELAY_KNOCK_BANG_ENABLED


#endif
