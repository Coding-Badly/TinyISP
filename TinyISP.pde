/*==============================================================================

  TinyISP 1.0 - Turn an ATmega328[P] Arduino compatible board (like an Uno) or
  a Teensy into an In System Programmer (ISP).  Based on MegaISP and 
  ArduinoISP.

  ----------------------------------------------------------------------------

  Copyright (c) 2012 Rowdy Dog Software
  Copyright (c) 2008-2011 Randall Bohn
  Copyright (c) 2009 David A. Mellis
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

//#include <SoftwareSerial.h>

#include <avr/pgmspace.h>

#include "TinyISP_SelectBuildOptions.h"
#include "TinyISP_SanityChecks.h"

#include "TinyISP_AispLED.h"
#include "TinyISP_InternalDebugging.h"
#include "TinyISP_Monitor.h"
#include "TinyISP_Programmer.h"

static const uint8_t Anchor = 13;

#if ARDUINO >= 100
#else
  #define F(s) s
#endif


/*----------------------------------------------------------------------------*/

void setup() 
{
/* fix? rmv?
  When a Teensy 2.0 is first connected, the following causes it to wait until 
  the first "normal" connection.  The end result is the monitor has to be 
  activated once before the Teensy begins functioning as a programmer.

  while ( ! Serial );
*/

  #if INTERNAL_DEBUGGING_TO_HARDWARESERIAL
    InternalDebugging.begin( 115200 );
  #endif

  Serial.begin( PROGRAMMER_BAUD_RATE );
  
  StatusIndicator.begin( LED_PIN );
}

/*----------------------------------------------------------------------------*/

void loop( void ) 
{
  StatusIndicator.update();

  if ( Serial.available() ) 
  {
    uint8_t ch;

    ch = Serial.read();

    if ( ! programmer_is_active() 
        && ((ch == '!') || (ch == '@') || (ch == '#') || (ch == '$')) )
    {
      if ( monitor_run( ch ) )
      {
        goto quick_switch;
      }
    }
    else
    {
quick_switch:
      programmer_process_command( ch );
    }
  }
}

/*----------------------------------------------------------------------------*/

 __attribute__ ((used)) char const Copyright[] PROGMEM = "Copyright (c) 2012 Rowdy Dog Software.  Copyright (c) 2008-2011 Randall Bohn.  Copyright (c) 2009 David A. Mellis.  All rights reserved.";

 /*----------------------------------------------------------------------------*/
