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

#include "TinyISP_SelectBuildOptions.h"

#include "TinyISP_SPI.h"

#include <inttypes.h>
#include <avr/io.h>

#if PROGRAMMER_SPI_CLOCK == SLOW
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif


/*----------------------------------------------------------------------------*/

#if PROGRAMMER_SPI_CLOCK != SLOW

void spi_begin( void ) 
{
  uint8_t x;

#if defined( RESET_NOT_SS_DDR ) && defined( RESET_NOT_SS_BIT )
  RESET_NOT_SS_DDR |= (1 << RESET_NOT_SS_BIT);
#endif

#if PROGRAMMER_SPI_CLOCK == SAFE
  // SPE: SPI Enable
  // MSTR: Master/Slave Select
  // SPI2X SPR1 SPR0 = 0 1 1 = SCK Frequency is fosc/128 = 125 K
  // 125 K * 2 * 2 = 500 K
  SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) |  (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (1 << SPR0);
#endif

#if PROGRAMMER_SPI_CLOCK == NORMAL
  // SPE: SPI Enable
  // MSTR: Master/Slave Select
  // SPI2X SPR1 SPR0 = 0 1 0 = SCK Frequency is fosc/64 = 250 K
  // 250 K * 2 * 2 = 1 M
  SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) |  (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (0 << SPR0);
#endif

#if PROGRAMMER_SPI_CLOCK == FAST
  // SPE: SPI Enable
  // MSTR: Master/Slave Select
  // SPI2X SPR1 SPR0 = 1 0 1 = SCK Frequency is fosc/8 = 2 M
  // 2 M * 2 * 2 = 8 M
  SPCR = (0 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) |  (0 << CPOL) | (0 << CPHA) | (0 << SPR1) | (1 << SPR0);
  SPSR = SPSR | (1 << SPI2X);
#endif

  x = SPSR;
  x = SPDR;
}

#else
void spi_begin( void ) 
{
}
#endif

/*----------------------------------------------------------------------------*/

#if PROGRAMMER_SPI_CLOCK != SLOW

void spi_end( void )
{
  SPCR &= ~ (1 << SPE);

  #if defined( RESET_NOT_SS_DDR ) && defined( RESET_NOT_SS_BIT )
    RESET_NOT_SS_DDR &= ~ (1 << RESET_NOT_SS_BIT);
  #endif
}

#else
void spi_end( void )
{
}
#endif

/*----------------------------------------------------------------------------*/

#if PROGRAMMER_SPI_CLOCK != SLOW

static void spi_wait( void ) 
{
  do {
  } 
  while ( ! (SPSR & (1 << SPIF)) );
}

static uint8_t spi_send( uint8_t b ) 
{
  uint8_t reply;
  SPDR = b;
  spi_wait();
  reply = SPDR;
  return( reply );
}

#else // PROGRAMMER_SPI_CLOCK == SLOW

static uint8_t spi_send( uint8_t b ) 
{
  uint8_t rv;
  
  rv = 0;

  for ( char i=7; i >= 0; --i )
  {
    rv = rv << 1;

    if ( b & 0x80 )
    {
      digitalWrite( MOSI, HIGH );
    }
    else
    {
      digitalWrite( MOSI, LOW );
    }

    // Note: The ATtiny13A datasheet indicates the pulses must be > 2 CPU cycles making 3 CPU cycles the minimum
    
    // 3 cycles / 128000 cycles per second = 23.4375 us
    digitalWrite( SCK, HIGH );
    delayMicroseconds( 24 );
    
    if ( digitalRead( MISO ) )
    {
      rv = rv | 0x01;
    }

    // 3 cycles / 128000 cycles per second = 23.4375 us
    digitalWrite( SCK, LOW );
    delayMicroseconds( 24 );

    b = b << 1;
  }
  
  return( rv );
}

#endif


/*----------------------------------------------------------------------------*/

uint8_t spi_transaction( uint8_t a, uint8_t b, uint8_t c, uint8_t d ) 
{
  uint8_t n;
  spi_send( a ); 
  n = spi_send( b );
  // rmv or fix ... if (n != a) error_count = -1;
  n = spi_send( c );
  return( spi_send( d ) );
}

/*----------------------------------------------------------------------------*/

uint32_t spi_transaction2( uint32_t c ) 
{
  uint32_t rv;

  rv =      spi_send( c >> 24 );
  rv = rv << 8;
  rv = rv | spi_send( c >> 16 );
  rv = rv << 8;
  rv = rv | spi_send( c >>  8 );
  rv = rv << 8;
  rv = rv | spi_send( c >>  0 );

  return( rv );
}

/*----------------------------------------------------------------------------*/
