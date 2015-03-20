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

#include "TinyISP_TuningSignal.h"

#include <avr/io.h>


/*----------------------------------------------------------------------------*/

#if BOARD_ARDUINO_UNO // defined( __AVR_ATmega328X__ )

void start_tuning_signal( tuning_signal_control_t tuning_signal )
{
  // Generate a stream of 2.000 (or 16.000) millisecond pulses on OC2B (PD3, digital pin 3)
  // If the target processor is perfectly tuned, TimeOnePulse returns 3200 counts (*5 = 16000 clocks) from a 2.000 millsecond pulse

  // Turn the timer off while changes are made
  TCCR2B =
      (0 << FOC2A) | (0 << FOC2B) 
        | 
      (0 << WGM22) 
        | 
      (0 << CS22) | (0 << CS21) | (0 << CS20);

  // Configure the Compare Match Output Mode and the Waveform Generation Mode
  // COM2A1 COM2A0 = 0 0 = Normal port operation, OC2A disconnected.
  // COM2B1 COM2B0 = 1 0 = Clear OC2B on Compare Match, set OC2B at TOP
  // Mode WGM22 WGM21 WGM20 = 3 0 1 1 Fast PWM 0xFF TOP MAX
  TCCR2A =
      (0 << COM2A1) | (0 << COM2A0)
        |
      (1 << COM2B1) | (0 << COM2B0)
        |
      (1 << WGM21) | (1 << WGM20);

  TCCR2B =
      TCCR2B
        |
      (0 << WGM22);

  // No interrupts
  TIMSK2 =
      (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);
  TIFR2 =
      (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);

  // Ensure the first pulse is correct (fix? Should this be set to TOP on the Teensy?)
  // fix: The prescaler should also be reset.
  TCNT2 = 0;

  // Enable the output driver
  DDRD |= (1 << DDD3);

  if ( tuning_signal == tsLongPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (1024 / 16000000) * (249 + 1)
    // 16 milliseconds
    OCR2B = 249;

    // Start the timer
    // CS22 CS21 CS20 = 1 1 1 clkT2S/1024 (From prescaler)
    TCCR2B =
        TCCR2B
          |
        ((1 << CS22) | (1 << CS21) | (1 << CS20));
  }
  else // if ( tuning_signal == tsShortPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (256 / 16000000) * (124 + 1)
    // 2 milliseconds
    OCR2B = 124;

    // Start the timer
    // CS22 CS21 CS20 = 1 1 0 clkT2S/256 (From prescaler)
    TCCR2B =
        TCCR2B
          |
        ((1 << CS22) | (1 << CS21) | (0 << CS20));
  }
}

void stop_tuning_signal( void )
{
  // Stop the timer
  // CS22 CS21 CS20 = 0 0 0 = No clock source (Timer/Counter stopped).
  TCCR2B =
      TCCR2B
        &
      ~ ((0 << CS22) | (0 << CS21) | (0 << CS20));

  // Disable the output driver
  DDRD &= ~ (1 << DDD3);
  // And ensure the pullup is disabled
  PORTD &= ~ (1 << PORTD3);
}

#endif

/*----------------------------------------------------------------------------*/

#if BOARD_TEENSY_2 || BOARD_ARDUINO_MICRO // defined( __AVR_ATmega32U4__ )

void start_tuning_signal( tuning_signal_control_t tuning_signal )
{
  // Generate a stream of 2.000 (or 16.000) millisecond pulses on OC1B (PB6, digital pin 15)
  // If the target processor is perfectly tuned, TimeOnePulse returns 3200 counts (*5 = 16000 clocks) from a 2.000 millsecond pulse

  // Turn the timer off while changes are made
  TCCR1B =
      (0 << ICNC1) | (0 << ICES1)
        |
      (0 << WGM13) | (0 << WGM12)
        |
      (0 << CS12) | (0 << CS11) | (0 << CS10);

  // Configure the Compare Match Output Mode and the Waveform Generation Mode
  // COM1A1 COM1A0 = 0 0 = Normal port operation, OC1A disconnected.
  // COM1B1 COM1B0 = 1 0 = Clear OC1B on Compare Match, set OC1B at TOP
  // Mode WGMn3 WGMn2 WGMn1 WGMn0
  //   5    0     1     0     1    Fast PWM, 8-bit 0x00FF TOP TOP
  TCCR1A =
      (0 << COM1A1) | (0 << COM1A0)
        |
      (1 << COM1B1) | (0 << COM1B0)
        |
      (0 << COM1C1) | (0 << COM1C0)
        |
      (0 << WGM11) | (1 << WGM10);

  TCCR1B =
      TCCR1B
        |
      (0 << WGM13) | (1 << WGM12);

  // No interrupts
  TIMSK1 =
      (0 << ICIE1) | (0 << OCIE1C) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);
  TIFR1 =
      (1 << ICF1) | (1 << OCF1C) | (1 << OCF1B) | (1 << OCF1A) | (1 << TOV1);

  // Ensure the first pulse is correct (fix? Should this be set to TOP on the Teensy?)
  // fix: The prescaler should also be reset.
  TCNT1 = 0;

  // Enable the output driver
  DDRB |= (1 << DDB6);

  if ( tuning_signal == tsLongPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (1024 / 16000000) * (249 + 1)
    // 16 milliseconds
    OCR1B = 249;

    // Start the timer
    // CSn2 CSn1 CSn0 = 1 0 1 clkI/O/1024 (From prescaler)
    TCCR1B =
        TCCR1B
          |
        ((1 << CS12) | (0 << CS11) | (1 << CS10));
  }
  else // if ( tuning_signal == tsShortPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (256 / 16000000) * (124 + 1)
    // 2 milliseconds
    OCR1B = 124;

    // Start the timer
    // CSn2 CSn1 CSn0 = 1 0 0 clkI/O/256 (From prescaler)
    TCCR1B =
        TCCR1B
          |
        ((1 << CS12) | (0 << CS11) | (0 << CS10));
  }
}

void stop_tuning_signal( void )
{
  // Stop the timer
  // CSn2 CSn1 CSn0 = 0 0 0 = No clock source. (Timer/Counter stopped)
  TCCR1B =
      TCCR1B
        &
      ~ ((0 << CS12) | (0 << CS11) | (0 << CS10));

  // Disable the output driver
  DDRB &= ~ (1 << DDB6);
  // And ensure the pullup is disabled
  PORTB &= ~ (1 << PORTB6);
}

#endif

/*----------------------------------------------------------------------------*/

#if BOARD_TEENSY_1_PLUS // defined( __AVR_AT90USB646__ )

void start_tuning_signal( tuning_signal_control_t tuning_signal )
{
  // Generate a stream of 2.000 (or 16.000) millisecond pulses on OC2B (PD1, digital pin 1)
  // If the target processor is perfectly tuned, TimeOnePulse returns 3200 counts (*5 = 16000 clocks) from a 2.000 millsecond pulse

  // Turn the timer off while changes are made
  TCCR2B =
      (0 << FOC2A) | (0 << FOC2B) 
        | 
      (0 << WGM22) 
        | 
      (0 << CS22) | (0 << CS21) | (0 << CS20);

  // Configure the Compare Match Output Mode and the Waveform Generation Mode
  // COM2A1 COM2A0 = 0 0 = Normal port operation, OC2A disconnected.
  // COM2B1 COM2B0 = 1 0 = Clear OC2B on Compare Match, set OC2B at TOP
  // Mode WGM22 WGM21 WGM20 = 3 0 1 1 Fast PWM 0xFF TOP MAX
  TCCR2A =
      (0 << COM2A1) | (0 << COM2A0)
        |
      (1 << COM2B1) | (0 << COM2B0)
        |
      (1 << WGM21) | (1 << WGM20);

  TCCR2B =
      TCCR2B
        |
      (0 << WGM22);

  // No interrupts
  TIMSK2 =
      (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);
  TIFR2 =
      (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);

  // Ensure the first pulse is correct (fix? Should this be set to TOP on the Teensy?)
  // fix: The prescaler should also be reset.
  TCNT2 = 0;

  // Enable the output driver
  DDRD |= (1 << DDD1);

  if ( tuning_signal == tsLongPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (1024 / 16000000) * (249 + 1)
    // 16 milliseconds
    OCR2B = 249;

    // Start the timer
    // CS22 CS21 CS20 = 1 1 1 clkT2S/1024 (From prescaler)
    TCCR2B =
        TCCR2B
          |
        ((1 << CS22) | (1 << CS21) | (1 << CS20));
  }
  else // if ( tuning_signal == tsShortPulse )
  {
    // (Prescaler / F_CPU) * (OCR + 1)
    // (256 / 16000000) * (124 + 1)
    // 2 milliseconds
    OCR2B = 124;

    // Start the timer
    // CS22 CS21 CS20 = 1 1 0 clkT2S/256 (From prescaler)
    TCCR2B =
        TCCR2B
          |
        ((1 << CS22) | (1 << CS21) | (0 << CS20));
  }
}

void stop_tuning_signal( void )
{
  // Stop the timer
  // CS22 CS21 CS20 = 0 0 0 = No clock source (Timer/Counter stopped).
  TCCR2B =
      TCCR2B
        &
      ~ ((0 << CS22) | (0 << CS21) | (0 << CS20));

  // Disable the output driver
  DDRD &= ~ (1 << DDD1);
  // And ensure the pullup is disabled
  PORTD &= ~ (1 << PORTD1);
}

/* fix: Make this the Recovery Clock
void start_tuning_signal( tuning_signal_control_t tuning_signal )
{
  // Generate a 1.0 MHz clock on OC2B (PD1, digital pin )

  // Using a 1.0 MHz clock requires that the target runs faster than 2.5 MHz so that the clock can reliably drive timer 0.  (F_CPU / 2.5)
  // The target decides what to do with the clock.

  // The clock can be used to recover a processor that does not have an external crystal with the fuses set to use an external crystal

  // Turn the timer off while changes are made
  TCCR2B =
      (0 << FOC2A) | (0 << FOC2B) 
      | 
      (0 << WGM22) 
      | 
      (0 << CS22) | (0 << CS21) | (0 << CS20);

  // Configure the Compare Match Output Mode and the Waveform Generation Mode
  // COM2A1 COM2A0 = 0 0 = Normal port operation, OC0A disconnected.
  // COM2B1 COM2B0 = 0 1 = Toggle OC2B on Compare Match
  // WGM22 WGM21 WGM20 = 0 1 0 = CTC OCRA Immediate MAX
  TCCR2A =
      (0 << COM2A1) | (0 << COM2A0)
      |
      (0 << COM2B1) | (1 << COM2B0)
      |
      (1 << WGM21) | (0 << WGM20);

  TCCR2B =
      TCCR2B
        |
      (0 << WGM22);

  // No interrupts
  TIMSK2 =
      (0 << OCIE2B) | (0 << OCIE2A) | (0 << TOIE2);
  TIFR2 =
      (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);

  // Ensure the first pulse is correct (fix? Should this be set to TOP on the Teensy?)
  // fix: The prescaler should also be reset.
  TCNT2 = 0;

  // F_CPU / (2 * Prescaler * (1 + OCR))
  // 16000000 / (2 * 1 * (1 + 7))
  // 1 MHz
  OCR2A = 7;

  // Enable the output driver
  // rmv DDRD |= (1 << DDD3);
  DDRD |= (1 << DDD1);

  // Start the timer
  // CS22 CS21 CS20 = 0 0 1 = clkT2S/(No prescaling)
  TCCR2B =
      TCCR2B
        |
      ((0 << CS12) | (0 << CS11) | (1 << CS10));
}

void stop_tuning_signal( void )
{
  // Stop the timer
  // CS22 CS21 CS20 = 0 0 0 = No clock source (Timer/Counter stopped).
  TCCR2B =
      TCCR2B
        &
      ~ ((0 << CS22) | (0 << CS21) | (0 << CS20));

  // Disable the output driver
  // rmv DDRD &= ~ (1 << DDD3);
  DDRD &= ~ (1 << DDD1);
  PORTD &= ~ (1 << PORTD1);
}
*/

#endif

/*----------------------------------------------------------------------------*/

void tuning_signal_control( tuning_signal_control_t const control )
{
  switch ( control )
  {
    default:
    case tsStop:
      stop_tuning_signal();
      break;

    case tsShortPulse:
      start_tuning_signal( control );
      break;

    case tsLongPulse:
      start_tuning_signal( control );
      break;
  }
}

/*----------------------------------------------------------------------------*/
