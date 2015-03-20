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

#include "KnockBangReceiver.h"
#include "KnockBangParser.h"
#include "TinyISP_TuningSignal.h"

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #define F(s) s
  #include <WProgram.h>
#endif


#if KNOCKBANG_RECEIVER_AVAILABLE

/*----------------------------------------------------------------------------*/

bool TinyDebugKnockBangParserClass::process( void )
{
  #if defined( ANALOG_READ_CHANNEL )
    static bool iar = true;  // Initialize Analog Read?
  #endif
  uint8_t by;
  bool rv;

  rv = false;

goagain:

/* rmv... 
  if ( KnockBangReceive_available() )
  {
    static uint8_t sample_point_histogram[256];
    
    kbr_stats_t kbr_stats;

    KnockBangReceive_getStats( kbr_stats );
    
    if ( ++sample_point_histogram[kbr_stats.sample_point] == 255 )
    {
      uint8_t i;
      uint8_t j;

      Serial.println();
      Serial.println();

      i = 0;
      do
      {
        if ( sample_point_histogram[i] != 0 )
        {
          Serial.print( i );
          Serial.write( '\t' );
          Serial.print( sample_point_histogram[i] );
          Serial.println();
  
          sample_point_histogram[i] = 0;
        }
        ++i;
      }
      while ( i != 0 );

      Serial.println();
      Serial.println();
    }
  }
 ...rmv */

  switch ( _state )
  {
    case sInit:
      if ( KnockBangReceive_available() )
      {
        _command = KnockBangReceive_read();
        _state = sDispatch;
        goto goagain;
      }
      break;
      
    case sDispatch:
      switch ( _command )
      {
        case CMD_WRITE_CRLF:
          Serial.println();
          rv = true;
          _state = sInit;
          goto goagain;
          break;

        case CMD_WRITE_BYTE:
          _state = sCollectByte;
          goto goagain;
          break;

        case CMD_WRITE_BLOCK:
          _state = sCollectSize;
          goto goagain;
          break;

        case CMD_PRINT_STRING:
          _state = sPrintToNull;
          goto goagain;
          break;
          
        case CMD_PRINT_UINT8:
          _value.i = 0;
          _count = 1;
          _state = sCollectValue;
          goto goagain;
          break;
          
        case CMD_PRINT_SINT16:
        case CMD_PRINT_UINT16:
          _value.i = 0;
          _count = 2;
          _state = sCollectValue;
          goto goagain;
          break;
          
        case CMD_PRINT_SINT32:
        case CMD_PRINT_UINT32:
        case CMD_PRINT_DOUBLE:
          _value.i = 0;
          _count = 4;
          _state = sCollectValue;
          goto goagain;
          break;

        case CMD_PRINT_ANALOG_READ:
          #if defined( ANALOG_READ_CHANNEL )
            if ( iar )
            {
              analogReference( EXTERNAL );
              analogRead( ANALOG_READ_CHANNEL );
              iar = false;
            }
            Serial.print( analogRead( ANALOG_READ_CHANNEL ) );
          #else
            Serial.print( F( "n/a" ) );
          #endif
          rv = true;
          _state = sInit;
          goto goagain;
          break;

        case CMD_CONTROL_TUNING_SIGNAL:
          _state = sCollectByte;
          goto goagain;
          break;

        case CMD_PRINT_MILLIS:
          Serial.print( millis() );
          rv = true;
          _state = sInit;
          goto goagain;
          break;
      }
      break;
      
    case sPrintToNull:
      while ( KnockBangReceive_available() )
      {
        by = KnockBangReceive_read();
        if ( by == 0 )
        {
          _state = sInit;
          goto goagain;
        }
        Serial.write( by );
        rv = true;
      }
      break;
      
    case sCollectValue:
      while ( KnockBangReceive_available() )
      {
        by = KnockBangReceive_read();

        _value.i = (_value.i << 8) | by;

        --_count;

        if ( _count == 0 )
        {
          _state = sCollectBase;
          goto goagain;
        }
      }
      break;
      
    case sCollectBase:
      if ( KnockBangReceive_available() )
      {
        _base = KnockBangReceive_read();
        
        if ( (_command == CMD_PRINT_UINT8) || (_command == CMD_PRINT_UINT16)  || (_command == CMD_PRINT_UINT32) )
        {
          Serial.print( (unsigned long)(_value.i), _base );
          rv = true;
        }
        else if ( _command == CMD_PRINT_SINT16 )
        {
          Serial.print( (signed long)((int16_t)(_value.i)), _base );
          rv = true;
        }
        else if ( _command == CMD_PRINT_SINT32 )
        {
          Serial.print( (signed long)(_value.i), _base );
          rv = true;
        }
        else if ( _command == CMD_PRINT_DOUBLE )
        {
          // _base is overloaded; it is "decimals" in this call...
          Serial.print( _value.f, _base );
          rv = true;
        }

        _state = sInit;
        goto goagain;
      }
      break;
      
    case sCollectByte:
      if ( KnockBangReceive_available() )
      {
        by = KnockBangReceive_read();

        if ( _command == CMD_WRITE_BYTE )
        {
          Serial.write( by );
          rv = true;
        }
        else if ( _command == CMD_CONTROL_TUNING_SIGNAL )
        {
          tuning_signal_control( tuning_signal_control_t(by) );
          rv = true;
        }

        _state = sInit;
        goto goagain;
      }
      break;
      
    case sCollectSize:
      if ( KnockBangReceive_available() )
      {
        // WARNING: _count cannot be zero and is not validated
        _count = KnockBangReceive_read();
        _state = sWriteToCount;
        goto goagain;
      }
      break;
      
    case sWriteToCount:
      while ( KnockBangReceive_available() )
      {
        by = KnockBangReceive_read();
        Serial.write( by );
        rv = true;
        --_count;
        if ( _count == 0 )
        {
          _state = sInit;
          goto goagain;
        }
      }
      break;
  }
  return( rv );
}

/*----------------------------------------------------------------------------*/

TinyDebugKnockBangParserClass TinyDebugParser;

/*----------------------------------------------------------------------------*/

#endif
