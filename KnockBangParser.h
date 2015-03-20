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

#ifndef KnockBangParser_h
#define KnockBangParser_h

#include "KnockBangReceiver.h"

#include <inttypes.h>


#if KNOCKBANG_RECEIVER_AVAILABLE

/*----------------------------------------------------------------------------*/

// fix: Move to a common header file

static const uint8_t CMD_WRITE_CRLF             =  1;
static const uint8_t CMD_WRITE_BYTE             =  2;
static const uint8_t CMD_WRITE_BLOCK            =  3;

static const uint8_t CMD_PRINT_STRING           =  4;
static const uint8_t CMD_PRINT_UINT8            =  5;
static const uint8_t CMD_PRINT_SINT16           =  6;
static const uint8_t CMD_PRINT_UINT16           =  7;
static const uint8_t CMD_PRINT_SINT32           =  8;
static const uint8_t CMD_PRINT_UINT32           =  9;
static const uint8_t CMD_PRINT_DOUBLE           = 10;

static const uint8_t CMD_PRINT_ANALOG_READ      = 11;
static const uint8_t CMD_CONTROL_TUNING_SIGNAL  = 12;
static const uint8_t CMD_PRINT_MILLIS           = 13;

/*----------------------------------------------------------------------------*/

class TinyDebugKnockBangParserClass
{
public:

  void reset( void )
  {
    _state = sInit;
  }

  bool process( void );
  
private:

  typedef enum { sInit, sDispatch, sPrintToNull, sCollectValue, sCollectBase, sCollectByte, sCollectSize, sWriteToCount } state_t;
  uint8_t _state;
  uint8_t _command;
  // WARNING: double is assumed to be the same size as uint32_t
  union { uint32_t i; double f; } _value;
  uint8_t _base;
  uint8_t _count;
};

/*----------------------------------------------------------------------------*/

extern TinyDebugKnockBangParserClass TinyDebugParser;

/*----------------------------------------------------------------------------*/

#endif


#endif
