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

#ifndef TinyISP_AispLED_h
#define TinyISP_AispLED_h

#include <inttypes.h>


class AispLEDClass
{
public:

  void begin( uint8_t pin );

  void error( void );

  void flash( void );

  typedef enum 
  { 
    mProgrammer, mRelay
  } 
  mode_t;

  void setMode( mode_t mode );

  void update( void );

private:

  typedef enum 
  { 
    sFlash0, sFlash, sFlash1, sFlash2, sFlashN,
    sError0, sError, sError1, sError2, sErrorN,
    sHeartbeat0, sHeartbeat, sHeartbeat1, sHeartbeatN,
    sSilent0, sSilent, sSilent1, sSilentN,
    sLampTest, sLampTest1,
    sFini 
  } 
  state_t;
  
  typedef unsigned short tick_t;
  
  uint8_t _pin;
  tick_t _previousTick;
  state_t _state;
  state_t _next;
  state_t _mode;
  int16_t _fade;
  int16_t _heartbeat;
};


extern AispLEDClass StatusIndicator;


#endif
