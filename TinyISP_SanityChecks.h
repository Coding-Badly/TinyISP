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

#ifndef TinyISP_SanityChecks_h
#define TinyISP_SanityChecks_h

#include "TinyISP_SelectBuildOptions.h"


#if defined( __AVR_ATmega328X__ ) 

  #if (RELAY_KNOCK_BANG_ENABLED) && (RELAY_SERIAL_ENABLED)
    #error The Knock-Bang Relay and Serial Relay are mutually exclusive for this processor.  Enable one or the other but not both.  See the <_TinyISP_BuildOptions.h> file.
  #endif

  #if (RELAY_SERIAL_ENABLED)
    #if (RELAY_SOFT_SERIAL_ENABLED)
      #if ! defined( SoftwareSerial_h )
        #error Ensure the "#include <SoftwareSerial.h>" line in the sketch is not commented-out or set RELAY_SERIAL_ENABLED to 0 in the <_TinyISP_BuildOptions.h> file.
      #endif
    #else
#error
    #endif
  #else
    #if defined( SoftwareSerial_h )
      #error Comment-out the "#include <SoftwareSerial.h>" line in the sketch or set RELAY_SERIAL_ENABLED to 1 in the <_TinyISP_BuildOptions.h> file.
    #endif
  #endif

/* rmv
  #if (RELAY_KNOCK_BANG_ENABLED)
    #if ! defined( KnockBangReceiver_h )
      #error Ensure the "#include <KnockBangReceiver.h>" line in the sketch is not commented-out or set RELAY_KNOCK_BANG_ENABLED to 0 in the <_TinyISP_BuildOptions.h> file.
    #endif
  #else
    #if defined( KnockBangReceiver_h )
      #error Comment-out the "#include <KnockBangReceiver.h>" line in the sketch or set RELAY_KNOCK_BANG_ENABLED to 1 in the <_TinyISP_BuildOptions.h> file.
    #endif
  #endif
*/

#endif


#endif
