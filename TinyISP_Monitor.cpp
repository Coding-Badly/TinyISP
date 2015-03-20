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

#include "TinyISP_SelectBuildOptions.h"

#include "TinyISP_AispLED.h"
#include "TinyISP_Monitor.h"
#include "TinyISP_RelayKnockBang.h"
#include "TinyISP_RelaySoftSerial.h"
#include "TinyISP_RelayUart.h"
#include "TinyISP_Programmer.h"
#include "TinyISP_TickTock.h"
#include "KnockBangParser.h"

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #define F(s) s
  #include <WProgram.h>
#endif


/*----------------------------------------------------------------------------*/

static KnockBangRelayClass RelayKnockBang; 

/*----------------------------------------------------------------------------*/

static TickTockClass TickTock;

/*----------------------------------------------------------------------------*/

static void monitor_output_help( void )
{
  Serial.println();
  Serial.println( F( "TinyISP Monitor commands..." ) );
  Serial.println();
  Serial.println( F( " !  Toggle between Monitor mode and Programmer mode" ) );
  Serial.println( F( " @  Toggle between holding the target in reset after programming and allowing the target to run immediately" ) );
  Serial.println( F( " #  Reset the target or allow the target to run if it is held in reset" ) );
  Serial.println( F( " $  Hold the target in reset" ) );
  Serial.println();
}

/*----------------------------------------------------------------------------*/

// rmv? extern uint32_t debug_programmer_1;

static void monitor_output_debug( void )
{
  Serial.println();
  Serial.println( F( "TinyISP Monitor debug..." ) );
  Serial.println();
// rmv?   Serial.print( F( "  debug_programmer_1: " ) );
// rmv?   Serial.println( debug_programmer_1, HEX );
  Serial.println();
}

/*----------------------------------------------------------------------------*/

static void purge_serial( void )
{
  static const uint32_t MicrosecondsForFourBytes =
      (
        ( 1 /*start*/ + 8 /*data*/ + 1 /*stop*/ )       // bits per byte
        * 4                                             // bytes
        * 1000000UL                                     // scale to microseconds
      );
  uint32_t WaitTime;
  uint32_t Start;

  // Microseconds needed to send 4 bytes at the programmer's baud rate plus three milliseconds for the host computer and USB channel
  WaitTime = 
    ( ( MicrosecondsForFourBytes + (PROGRAMMER_BAUD_RATE - 1) ) / PROGRAMMER_BAUD_RATE )
    + 3000;

  // Wait for 4 bytes of quiet time before returning
  Start = micros();
  while ( micros() - Start < WaitTime )
  {
    if ( Serial.available() )
    {
      Serial.read();
      Start = micros();
    }
  }
}

/*----------------------------------------------------------------------------*/

bool monitor_run( uint8_t& b )
{
  bool RelayActive;
  bool SomethingRelayed;
  bool rv;
  uint8_t f;

  TickTock.begin( TICK_TOCK_PIN );
  RelaySerial.begin( RELAY_BAUD_RATE );

  StatusIndicator.setMode( AispLEDClass::mRelay );
  
  Serial.println( F( "\r\n--- Monitor starting ---" ) );

  if ( programmer_release_target_from_reset() )
  {
    /* rmv? To minimize the potential overrun in KnockBang
    Serial.println( F( "\r\n--- Target released from reset ---" ) ); */
  }

  // This must be done after the target is released from reset.  While in reset, the target does something with MISO that interferes with KnockBang. */
  RelayKnockBang.begin();
  
  rv = false;
  RelayActive = true;
  SomethingRelayed = false;

  while ( RelayActive )
  {
    while ( RelaySerial.available() )
    {
      Serial.write( RelaySerial.read() );
      SomethingRelayed = true;
    }

    if ( RelayKnockBang.process() )
    {
      SomethingRelayed = true;
    }

/* rmv 
    if ( KnockBangReceive_available() )
    {
      while ( KnockBangReceive_available() )
      {
        Serial.print( KnockBangReceive_read(), HEX );
        Serial.write( ' ' );
      }
      Serial.println();
      SomethingRelayed = true;
    }
  */

    f = RelayKnockBang.lastFault();
    if ( f != 0 )
    {
      Serial.print( F( "\r\n--- Knock-Bang fault: " ) );
      Serial.print( f, HEX );
      Serial.println( F( " ---" ) );
      // fix? 
      TinyDebugParser.reset();
    }

    if ( SomethingRelayed )
    {
      StatusIndicator.flash();
      SomethingRelayed = false;
    }

    if ( TickTock.toggled() )
    {
      Serial.write( '.' );
      Serial.write( '\t' );
      Serial.print( TickTock.value(), DEC );
      Serial.write( '\t' );
      Serial.print( millis(), DEC );
      Serial.println();
    }

    if ( Serial.available() )
    {
      uint8_t ch = Serial.read();
      
      switch ( ch )
      {
        case '!':
          RelayActive = false;
          break;

        case '@':
          Serial.print( F( "\r\n--- Target " ) );

          if ( programmer_toggle_hold_in_reset() )
          {
            Serial.print( F( "WILL" ) );
          }
          else
          {
            Serial.print( F( "will NOT" ) );
          }
          Serial.println( F( " be held in reset after programming ---" ) );
          break;

        case '#':
          programmer_reset_target();
          Serial.println( F( "\r\n--- Target was reset and is now running ---" ) );
          break;

        case '$':
          programmer_hold_target_in_reset();
          Serial.println( F( "\r\n--- Target is stopped (held in reset) ---" ) );
          break;

        case '?':
          monitor_output_help();
          break;

        case '\\':
          monitor_output_debug();
          break;

        case '0':  // Cmnd_STK_GET_SYNC
          RelayActive = false;
          rv = true;
          b = ch;
          break;

        default:
          RelaySerial.write( ch );
          break;
      }
    }
    
    StatusIndicator.update();
  }

  if ( ! rv )
  {
    purge_serial();
  }

  TickTock.end();
  RelaySerial.end();

  RelayKnockBang.end();

  StatusIndicator.setMode( AispLEDClass::mProgrammer );

  if ( ! rv )
  {
    Serial.println( "\r\n--- Monitor stopped ---" );
  }
  return( rv );
}

/*----------------------------------------------------------------------------*/
