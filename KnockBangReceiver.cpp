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
#include "KnockBangReceiver.h"

#include <util/atomic.h>

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


#if KNOCKBANG_RECEIVER_AVAILABLE && RELAY_KNOCK_BANG_ENABLED


/*==============================================================================
  Calculate a loop and nop count for use by DELAY_FINE.  Closest match less 
  than or equal to the target is returned (floor).
==============================================================================*/

static uint16_t MicrosecondsToLoopsAndNops( long us, long cc ) __attribute__ (( const, always_inline ));

static uint16_t MicrosecondsToLoopsAndNops( long us, long cc )
{
  long goal_cycles;
  long loops_raw;
  long loops_cooked;
  long loops_cycles;
  long leftover;
  long nops_cooked;
  
  goal_cycles   = us * F_CPU;
  
  loops_raw     = (goal_cycles - (1000000L * (0L + cc))) / (1000000L * 3L);
  
  loops_cooked  = loops_raw < 0 ? 0 : loops_raw;
  
  loops_cycles  = (loops_cooked * 3L) + 0L + cc;
  
  leftover      = (goal_cycles - (loops_cycles * 1000000L)) / 1000000L;
  
  nops_cooked   = leftover < 0 ? 0 : leftover;

  return( (loops_cooked << 2) | nops_cooked );
}


/*==============================================================================
  Assembly macros
==============================================================================*/

void TinyDebugKnockBangClass_AssemblyMacros( void )
{
  // Macro definitions

  asm volatile 
  (
    ".macro DELAY_ROUGH Rx, LoopsOrNops"              "\n\t"

    ".if \\LoopsOrNops != 0"                          "\n\t"

    ".if \\LoopsOrNops < 128"                         "\n\t"

    "ldi   \\Rx, \\LoopsOrNops"                       "\n\t"
  "1:"                                                "\n\t"
    "dec   \\Rx"                                      "\n\t"
    "brne  1b"                                        "\n\t"

    ".else"                                           "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 1"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 2"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 3"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".endif"                                          "\n\t"

    ".endif"                                          "\n\t"

    ".endm"                                           "\n\t"
		: 
		: 
    :
  );

  asm volatile 
  (
    ".macro DELAY_FINE Rx, LoopsAndNops"              "\n\t"

    ".if ( \\LoopsAndNops >> 2 ) != 0"                "\n\t"
      "ldi   \\Rx, ( \\LoopsAndNops >> 2 )"           "\n\t"
    "1:"                                              "\n\t"
      "dec   \\Rx"                                    "\n\t"
      "brne  1b"                                      "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 1"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 2"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 3"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".endm"                                           "\n\t"
		: 
		: 
    :
  );
}


/*==============================================================================
  Stuff for debugging the the ring buffer.
==============================================================================*/

static volatile bool knb_dirty asm("knb_dirty") __attribute__ ((used));

static volatile uint8_t knb_knock_continue asm("knb_knock_continue") __attribute__ ((used));
static volatile uint8_t knb_hello_float asm("knb_hello_float") __attribute__ ((used));
static volatile uint8_t knb_sample_pad asm("knb_sample_pad") __attribute__ ((used));
static volatile uint8_t knb_sample_point asm("knb_sample_point") __attribute__ ((used));
static volatile uint8_t knb_init_byte asm("knb_init_byte") __attribute__ ((used));
static volatile uint8_t knb_overrun asm("knb_overrun") __attribute__ ((used));

static const uint8_t knb_ring_buffer_size = 128; // 64;
/*static*/ volatile uint8_t knb_ring_space asm("knb_ring_space") __attribute__ ((used));
/*static*/ volatile uint8_t knb_ring_head asm("knb_ring_head") __attribute__ ((used));
static uint8_t knb_ring_tail;
static volatile uint8_t knb_ring_buffer[knb_ring_buffer_size] asm("knb_ring_buffer") __attribute__ ((used));
/*static*/ volatile uint8_t kbr_fault_code asm("kbr_fault_code") __attribute__ ((used));


/*==============================================================================
  Initialize the library
==============================================================================*/

void KnockBangReceive_begin( void )
{
  ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
  {
    // Flush any pending data
    knb_ring_space = knb_ring_buffer_size;
    knb_ring_head = 0;
    knb_ring_tail = 0;

    // Input / pullup enabled
    KBR_DDR &= ~ (1 << KBR_BIT);
    KBR_PORT |= (1 << KBR_BIT);

    // Enable the pin-change interrupt for the port
    KBR_PCICR |= (1 << KBR_PCIE);

    // Enable the pin-change interrupt for the pin
    // Warning: It is assumed that the bit in PCMSK is the same bit in DDR, PORT, and PIN.
    KBR_PCMSK |= (1 << KBR_BIT);
  }
}


/*==============================================================================
  Shut down the library
==============================================================================*/

void KnockBangReceive_end( void )
{
  ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
  {
    // Disable the pin-change interrupt for the pin
    // Warning: It is assumed that the bit in PCMSK is the same bit in DDR, PORT, and PIN.
    KBR_PCMSK &= ~ (1 << KBR_BIT);

    // Disable the pin-change interrupt for the port
    KBR_PCICR &= ~ (1 << KBR_PCIE);

    // Input / pullup disabled
    KBR_DDR &= ~ (1 << KBR_BIT);
    KBR_PORT &= ~ (1 << KBR_BIT);

    // Flush any pending data
    knb_ring_space = knb_ring_buffer_size;
    knb_ring_head = 0;
    knb_ring_tail = 0;
  }
}


/*==============================================================================
  Return the number of bytes available in the ring buffer
==============================================================================*/

uint8_t KnockBangReceive_available( void )
{
  return( knb_ring_buffer_size - knb_ring_space );
}


/*==============================================================================
  Return the next byte from the ring buffer or -1 if nothing is available
==============================================================================*/

int KnockBangReceive_read( void )
{
  int rv;

  if ( (knb_ring_buffer_size - knb_ring_space) == 0 )
  {
    return( -1 );
  }
  else
  {
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
      rv = knb_ring_buffer[knb_ring_tail];
      knb_ring_tail = (knb_ring_tail + 1) & (knb_ring_buffer_size - 1);
      ++knb_ring_space;
    }
  }
  return( rv );
}


/*==============================================================================
  On pin change interrupt, attempt to collect any received data
==============================================================================*/

ISR(KBR_PCINT_VECT)
{
  uint8_t state;
  uint8_t status;
  uint8_t i;
  uint8_t j;  // Extended timeout
  uint8_t sp;
  uint8_t by;
  uint8_t bi;
  uint8_t ring_space;
  uint8_t ring_head;
  uint8_t* ring_buffer;
  uint8_t frame_count;

  asm volatile
  (
    "state_init                   = 0x00   \n\t"
    "state_by_count               = 0x01   \n\t"
    "state_by_null                = 0x02   \n\t"

    "status_ok                    = 0x00   \n\t"
    "status_overrun               = 0x01   \n\t"

    "no_fault                     = 0x00   \n\t"
    "fault_no_knock               = 0x04   \n\t"
    "fault_no_idle                = 0x05   \n\t"
    "fault_timeout_knock          = 0x11   \n\t"
    "fault_timeout_hello          = 0x12   \n\t"
    "fault_timeout_sample_0       = 0x13   \n\t"
    "fault_timeout_sample_N       = 0x14   \n\t"
    "fault_timeout_toggle_0       = 0x15   \n\t"
    "fault_timeout_toggle_1       = 0x16   \n\t"

    "sbis  %[mosipin], %[mosibit]"                    "\n\t"   
    "rjmp  L%=bpossibleknock"                         "\n\t"

    // fault: No knock!
    "ldi   %[i], fault_no_knock"                      "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"

  "L%=bpossibleknock:"  "\n\t"

    "ldi   %[i], 0"                                   "\n\t"  
  "L%=bwaitforknock:"  "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"  
    "rjmp  L%=bgotknock"                              "\n\t"  
    "dec   %[i]"                                      "\n\t"  
    "brne  L%=bwaitforknock"                          "\n\t"   
    "ldi   %[i], fault_timeout_knock"                 "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bgotknock:"  "\n\t"
    "sts   knb_knock_continue, %[i]"                  "\n\t"

  "L%=bsayhello:"  "\n\t"
/* Skip the Hello
    // Pull MISO low (sinking output)
    "cbi   %[mosiport], %[mosibit]"                   "\n\t"
    "sbi   %[mosiddr], %[mosibit]"                    "\n\t"
*/
    // This is a good time to initialize a few things
    "lds   %[ring_space], knb_ring_space"             "\n\t"
    "lds   %[ring_head], knb_ring_head"               "\n\t"
    "ldi   %[state], state_init"                      "\n\t"
    "ldi   %[status], status_ok"                      "\n\t"
/* Skip the Hello
    // delay 6 us (29 includes the stuff above)
    "ldi   %[i], 29"                                  "\n\t"  
  "L%=bdelay6us:"  "\n\t"
    "dec   %[i]"                                      "\n\t"  
    "brne  L%=bdelay6us"                              "\n\t"  
*/

//  "ldi   %[i], 0"                                   "\n\t"  // %[i] is assumed to be and is zero
/* Skip the Hello
    // MOSI back high (input pullup)
    "cbi   %[mosiddr], %[mosibit]"                    "\n\t"
    "sbi   %[mosiport], %[mosibit]"                   "\n\t"
*/
    // Wait for MISO to float high
  "L%=bwaitforhigh:"                                  "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"
    "rjmp  L%=bsaidhello"                             "\n\t"
    "dec   %[i]"                                      "\n\t"
    "brne  L%=bwaitforhigh"                           "\n\t"
    "ldi   %[i], fault_timeout_hello"                 "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bsaidhello:"                                    "\n\t"
    "sts   knb_hello_float, %[i]"                     "\n\t"

    "ldi   %[i], 0"                                   "\n\t"
  "L%=bwaitforsample0:"                               "\n\t"
    "sbis  %[mosipin], %[mosibit]"                    "\n\t"  // 2 1
    "rjmp  L%=bgotsample0"                            "\n\t"  // - 2
    "dec   %[i]"                                      "\n\t"  // 1
    "brne  L%=bwaitforsample0"                        "\n\t"  // 2
    "ldi   %[i], fault_timeout_sample_0"              "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bgotsample0:"                                   "\n\t"
    "nop"                                             "\n\t"  //   1
    "nop"                                             "\n\t"  //   1
    "ldi   %[sp], 0"                                  "\n\t"  //   1    c = 11 or 6
  "L%=bwaitforsampleN:"                               "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"  // 2
    "rjmp  L%=bgotsampleN"                            "\n\t"  // -
//rmv    "dec   %[sp]"                                     "\n\t"  // 1
    "inc   %[sp]"                                     "\n\t"  // 1
    "brne  L%=bwaitforsampleN"                        "\n\t"  // 2      m = 5
    "ldi   %[i], fault_timeout_sample_N"              "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bgotsampleN:"                                   "\n\t"

  "L%=bgotsamplepoint:"  "\n\t"
    "sts   knb_sample_pad, %[i]"                      "\n\t"  // 2      +8
//rmv    "com   %[sp]"                                     "\n\t"  // 1
    "sts   knb_sample_point, %[sp]"                   "\n\t"  // 2

    "ldi   %[by], 0x00"                               "\n\t"  // 1
    "ldi   %[bi], 8"                                  "\n\t"  // 1

  "L%=bwaitfortoggle0:"                               "\n\t"
    "ldi   %[i], 0"                                   "\n\t"  // 1
    "ldi   %[j], 56"                                  "\n\t"  // Extended timeout: 256*56/16 = 896us
  "L%=bwaitfortoggle0loop:"                           "\n\t"
    "sbis  %[mosipin], %[mosibit]"                    "\n\t"  // 2 1
    "rjmp  L%=bgottoggle0"                            "\n\t"  // - 2
    "dec   %[i]"                                      "\n\t"  // 1
    "brne  L%=bwaitfortoggle0loop"                    "\n\t"  // 2
    "dec   %[j]"                                      "\n\t"  // Extended timeout
    "brne  L%=bwaitfortoggle0loop"                    "\n\t"
    "ldi   %[i], fault_timeout_toggle_0"              "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bgottoggle0:"                                   "\n\t"
    "nop"                                             "\n\t"  //   1
    "nop"                                             "\n\t"  //   1
    
/*rmv...
  "L%=bwaitfordata:"                                  "\n\t"
    "DELAY_FINE %[i], %[tog_to_bit]"                  "\n\t"
...rmv*/

/*
  "L%=bwaitfordata:"                                  "\n\t"
    "mov   %[i], %[sp]"                               "\n\t"  //   1    c = 11 or 6
  "L%=bwaitfordatasample:"                            "\n\t"
    "dec   %[i]"                                      "\n\t"  // 1
    "breq  L%=bsamplenow"                             "\n\t"  // 1
    "nop"                                             "\n\t"  // 1
    "rjmp  L%=bwaitfordatasample"                     "\n\t"  // 2      m = 5
*/
  "L%=bwaitfordata:"                                  "\n\t"
    "mov   %[i], %[sp]"                               "\n\t"
  "L%=bwaitfordatasample:"                            "\n\t"
    "nop"                                             "\n\t"
    "nop"                                             "\n\t"
    "dec   %[i]"                                      "\n\t"
    "brne  L%=bwaitfordatasample"                     "\n\t"

  "L%=bsamplenow:"  "\n\t"
    "sbis  %[mosipin], %[mosibit]"                    "\n\t"  // 1
    "rjmp  L%=bgotdata0"                              "\n\t"  // 2
  "L%=bgotdata1:"  "\n\t"
    "lsr   %[by]"                                     "\n\t"
    "ori   %[by], 0x80"                               "\n\t"
    "dec   %[bi]"                                     "\n\t"
    "brne  L%=bwaitfortoggle0"                        "\n\t"
    "clt"                                             "\n\t"
    "rjmp  L%=bgotbyte"                               "\n\t"  
  "L%=bgotdata0:"  "\n\t"
    "lsr   %[by]"                                     "\n\t"  // 1
    "nop"                                             "\n\t"  // 1
    "dec   %[bi]"                                     "\n\t"  // 1
    "brne  L%=bwaitfortoggle1"                        "\n\t"  // 2
    "set"                                             "\n\t"
    "rjmp  L%=bgotbyte"                               "\n\t"  

  "L%=bwaitfortoggle1:"                               "\n\t"
    "ldi   %[i], 0"                                   "\n\t"  // 1
    "ldi   %[j], 56"                                  "\n\t"  // Extended timeout: 256*56/16 = 896us
  "L%=bwaitfortoggle1loop:"                           "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"
    "rjmp  L%=bgottoggle1"                            "\n\t"
    "dec   %[i]"                                      "\n\t"
    "brne  L%=bwaitfortoggle1loop"                    "\n\t"
    "dec   %[j]"                                      "\n\t"  // Extended timeout
    "brne  L%=bwaitfortoggle1loop"                    "\n\t"
    "ldi   %[i], fault_timeout_toggle_1"              "\n\t"
    "rjmp  L%=bfault"                                 "\n\t"
  "L%=bgottoggle1:"                                   "\n\t"
    "rjmp  L%=bwaitfordata"                           "\n\t"

  "L%=bgotbyte:"  "\n\t"

    "cpi   %[state], state_init"                      "\n\t"
    "breq  L%=bstateinit"                             "\n\t"

  "L%=bstateframebycount:"  "\n\t"
  "L%=bstateframebynull:"  "\n\t"

  "L%=bsavethebyte:"  "\n\t"
    "and   %[ring_space], %[ring_space]"              "\n\t"
    "breq  L%=bringfull"                              "\n\t"
    "ldi   %A[ring_buffer], lo8(knb_ring_buffer)"     "\n\t"
    "ldi   %B[ring_buffer], hi8(knb_ring_buffer)"     "\n\t"
    "add   %A[ring_buffer], %[ring_head]"             "\n\t"
    "adc   %B[ring_buffer], __zero_reg__"             "\n\t"
    "st    %a[ring_buffer], %[by]"                    "\n\t"
    "inc   %[ring_head]"                              "\n\t"
    "andi  %[ring_head], %[ring_buffer_mask]"         "\n\t"
    "dec   %[ring_space]"                             "\n\t"
    "rjmp  L%=bringend"                               "\n\t"
  "L%=bringfull:"  "\n\t"
    "ori   %[status], status_overrun"                 "\n\t"
  "L%=bringend:"  "\n\t"

    "cpi   %[state], state_by_null"                   "\n\t"
    "breq  L%=bfinishbynull"                          "\n\t"
    "dec   %[frame_count]"                            "\n\t"
    "brne  L%=bresetformore"                          "\n\t"
    "rjmp  L%=bfinishsavethebyte"                     "\n\t"
  "L%=bfinishbynull:"  "\n\t"
    "and   %[by], %[by]"                              "\n\t"
    "brne  L%=bresetformore"                          "\n\t"
//  "rjmp  L%=bfinishsavethebyte"                     "\n\t"
  "L%=bfinishsavethebyte:"  "\n\t"
    "ldi   %[state], state_init"                      "\n\t"
    "rjmp  L%=bresetformore"                          "\n\t"

  "L%=bstateinit:"  "\n\t"

    "sts   knb_init_byte, %[by]"                      "\n\t"  // rmv

    // if %[by] == 0 then we're finished
    "cpi   %[by], 0"                                  "\n\t"
    "breq  L%=bnomoreframes"                          "\n\t"

    // if %[by] == 255 then it's a null terminated string
    "cpi   %[by], 255"                                "\n\t"
    "brne  L%=bcheckby2"                              "\n\t"
    "ldi   %[state], state_by_null"                   "\n\t"
    "rjmp  L%=bresetformore"                          "\n\t"

    // otherwise, %[by] is the number of bytes to expect
  "L%=bcheckby2:"  "\n\t"
    "ldi   %[state], state_by_count"                  "\n\t"
    "mov   %[frame_count], %[by]"                     "\n\t"

  "L%=bresetformore:"  "\n\t"
    "ldi   %[by], 0x00"                               "\n\t"
    "ldi   %[bi], 8"                                  "\n\t"
    "brts  L%=bwaitfortoggle1"                        "\n\t"
//  "brtc  L%=bwaitfortoggle0"                        "\n\t"
    "rjmp  L%=bwaitfortoggle0"                        "\n\t"

  "L%=bnomoreframes:"                                 "\n\t"
    "ldi   %[i], 1"                                   "\n\t"
    "sts   knb_dirty, %[i]"                           "\n\t"
    "rjmp  L%=bwaitforidle"                           "\n\t"

  "L%=bfault:"                                        "\n\t"
    "sts   kbr_debug_1, %[bi]"                        "\n\t"
    "lds   %[bi], kbr_fault_code"                     "\n\t"
    "and   %[bi], %[bi]"                              "\n\t"
    "brne  L%=bfaultalreadyset1"                      "\n\t"
    "sts   kbr_fault_code, %[i]"                      "\n\t"
  "L%=bfaultalreadyset1:"                             "\n\t"

/* rmv
  "L%=bwaitforidle:"  "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"
    "rjmp  L%=breturn"                                "\n\t"
  "L%=bwaitfor1:"  "\n\t"
    "ldi   %[i], 0"                                   "\n\t"
  "L%=bwaitfor1loop:"  "\n\t"
    "dec   %[i]"                                      "\n\t"
    "breq  L%=breturn"                                "\n\t"
    "sbis  %[mosipin], %[mosibit]"                    "\n\t"
    "rjmp  L%=bwaitfor1loop"                          "\n\t"
*/
  "L%=bwaitforidle:"                                  "\n\t"
    "ldi   %[i], 0"                                   "\n\t"
    "ldi   %[j], 56"                                  "\n\t"  // Extended timeout: 256*56/16 = 896us
  "L%=bwaitfor1loop:"                                 "\n\t"
    "sbic  %[mosipin], %[mosibit]"                    "\n\t"
    "rjmp  L%=breturn"                                "\n\t"
    "dec   %[i]"                                      "\n\t"
    "brne  L%=bwaitfor1loop"                          "\n\t"
    "dec   %[j]"                                      "\n\t"
    "brne  L%=bwaitfor1loop"                          "\n\t"
  "L%=bfaultnoidle:"                                  "\n\t"
    "ldi   %[i], fault_no_idle"                       "\n\t"
    "lds   %[bi], kbr_fault_code"                     "\n\t"
    "and   %[bi], %[bi]"                              "\n\t"
    "brne  L%=bfaultalreadyset2"                      "\n\t"
    "sts   kbr_fault_code, %[i]"                      "\n\t"
  "L%=bfaultalreadyset2:"                             "\n\t"

  "L%=breturn:"  "\n\t"
    "cpi   %[status], status_ok"                      "\n\t"
    "brne  L%=brollback"                              "\n\t"
  "L%=bcommit:"  "\n\t"
    "sts   knb_ring_space, %[ring_space]"             "\n\t"
    "sts   knb_ring_head, %[ring_head]"               "\n\t"
    "ldi   %[i], 0"                                   "\n\t"
    "rjmp  L%=bfini"                                  "\n\t"
  "L%=brollback:"  "\n\t"
    "ldi   %[i], 1"                                   "\n\t"
  "L%=bfini:"  "\n\t"
    "sts   knb_overrun, %[i]"                         "\n\t"

    : 
      // Outputs
      [state] "=r" ( state ),
      [status] "=r" ( status ),
      [i] "=r" ( i ),
      [j] "=r" ( j ),  // Extended timeout
      [sp] "=r" ( sp ),
      [by] "=r" ( by ),
      [bi] "=r" ( bi ),
      [frame_count] "=r" ( frame_count ), 
      [ring_space] "=r" ( ring_space ),
      [ring_head] "=r" ( ring_head ),
      [ring_buffer] "=e" ( ring_buffer )
    : 
      // Inputs
      [mosiddr] "I" ( _SFR_IO_ADDR( KBR_DDR ) ),
      [mosiport] "I" ( _SFR_IO_ADDR( KBR_PORT ) ),
      [mosipin] "I" ( _SFR_IO_ADDR( KBR_PIN ) ),
      [mosibit] "M" ( KBR_BIT ),
      [ring_buffer_mask] "M" ( knb_ring_buffer_size-1 ),
      [tog_to_bit] "M" ( MicrosecondsToLoopsAndNops( 2, 7 /*5 or 10*/ ) )
    :
      // Clobbers
  );
  KBR_PCIFR |= (1 << KBR_PCIF);
}


/*==============================================================================
  Return debugging statistics
==============================================================================*/

bool KnockBangReceive_getStats( kbr_stats_t & stats )
{
  bool rv;

  rv = false;

  if ( knb_dirty )
  {
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
      if ( knb_dirty )
      {
        rv = true;
        knb_dirty = false;

        stats.knock_continue = knb_knock_continue;
        stats.hello_float    = knb_hello_float;
        stats.sample_pad     = knb_sample_pad;
        stats.sample_point   = knb_sample_point;
        stats.init_byte      = knb_init_byte;
        stats.ring_space     = knb_ring_space;
        stats.ring_head      = knb_ring_head;
        stats.overrun        = knb_overrun;
      }
    }
  }
  return( rv );
}


/*==============================================================================
  Return the last fault code or zero if nothing bad has happened
==============================================================================*/

uint8_t KnockBangReceive_getLastFault( void )
{
  uint8_t rv;

  ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
  {
    rv = kbr_fault_code;
    kbr_fault_code = 0;
  }

  return( rv );
}


/*==============================================================================
  Return a generic debug value
==============================================================================*/

uint8_t KnockBangReceive_getLastDebug1( void )
{
  static volatile uint8_t d1 asm("kbr_debug_1");
  uint8_t rv;

  ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
  {
    rv = d1;
    d1 = 0;
  }

  return( rv );
}


#endif // KNOCKBANG_RECEIVER_AVAILABLE
