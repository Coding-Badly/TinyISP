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

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


/*----------------------------------------------------------------------------*/

AispLEDClass StatusIndicator;

/*----------------------------------------------------------------------------*/

void AispLEDClass::begin( uint8_t pin )
{
  _pin = pin;
  pinMode( _pin, OUTPUT );
  _state = sLampTest;
  _next = sHeartbeat;
  _mode = sHeartbeat;
  _previousTick = millis();
  _heartbeat = +1;
}

/*----------------------------------------------------------------------------*/

void AispLEDClass::error( void )
{
  if ( ! ( (_state >= sError0) && (_state <= sErrorN) ) )
  {
    _state = sError;
    update();
  }
}

/*----------------------------------------------------------------------------*/

void AispLEDClass::flash( void )
{
  {
    if ( (_state >= sFlash0) && (_state <= sFlashN) )
    {
      _next = sFlash2;
    }
    else
    {
      _state = sFlash;
      update();
    }
  }
}

/*----------------------------------------------------------------------------*/

void AispLEDClass::setMode( mode_t mode )
{
  if ( mode == mRelay )
  {
    _mode = sSilent;
  }
  else
  {
    _mode = sHeartbeat;
  }
}

/*----------------------------------------------------------------------------*/

void AispLEDClass::update( void )
{
  state_t CurrentState;
  tick_t currentTick;
  tick_t delta;
  
  currentTick = millis();

  do
  {
    CurrentState = _state;
  
    switch ( _state )
    {
      case sFlash:
        analogWrite( _pin, 255 );
        _previousTick = currentTick;
        _state = sFlash1;
        break;

      case sFlash1:
        if ( currentTick - _previousTick >= 50 )
        {
          if ( _next != sFlash2 )
          {
            _fade = 128;
            _heartbeat = -1;
            analogWrite( _pin, _fade );
          }
          else
          {
            analogWrite( _pin, 0 );
          }
          _previousTick = currentTick;
          _state = _next;
          _next = _mode;
        }
        break;

      case sFlash2:
        if ( currentTick - _previousTick >= 50 )
        {
          analogWrite( _pin, 255 );
          _previousTick = currentTick;
          _state = sFlash1;
        }
        break;

      case sError:
        analogWrite( _pin, 255 );
        _previousTick = currentTick;
        _state = sError1;
        break;
        
      case sError1:
        if ( currentTick - _previousTick >= 900 )
        {
          analogWrite( _pin, 0 );
          _previousTick = currentTick;
          _state = sError2;
        }
        break;

      case sError2:
        if ( currentTick - _previousTick >= 100 )
        {
          analogWrite( _pin, 255 );
          _previousTick = currentTick;
          _state = sError1;
        }
        break;

      case sHeartbeat:
        _previousTick = currentTick;
        _state = sHeartbeat1;
        break;

      case sHeartbeat1:
        if ( currentTick - _previousTick >= 48 /*16*/ )
        {
          if ( _fade >= 48 /*64*/ )
          {
            _heartbeat = -1;
          }
          else if ( _fade <= 8 )
          {
            _heartbeat = +1;
            
            if ( _mode == sSilent )
            {
              _state = sSilent;
            }
          }
          _fade = _fade + _heartbeat;
          analogWrite( _pin, _fade );
          _previousTick = currentTick;
        }
        break;
        
      case sSilent:
        analogWrite( _pin, 0 );
        _state = sSilent1;
        break;

      case sSilent1:
          if ( _mode == sHeartbeat )
          {
            _state = sHeartbeat;
          }
        break;

      case sLampTest:
        _fade = 0;
        analogWrite( _pin, 255 );
        _previousTick = currentTick;
        _state = sLampTest1;
        break;

      case sLampTest1:
        delta = currentTick - _previousTick;
        if ( delta >= 50 )
        {
          ++_fade;
          if ( _fade <= 4 )
          {
            analogWrite( _pin, 255 );
            _previousTick = currentTick;
          }
          else
          {
            _state = _mode;
          }
        }
        else
          analogWrite( _pin, 255 - (5*delta) );
        break;
    }
  }
  while ( CurrentState != _state );
}

/*----------------------------------------------------------------------------*/
