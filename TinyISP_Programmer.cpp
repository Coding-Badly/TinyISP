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

#include "TinyISP_AispLED.h"
#include "TinyISP_InternalDebugging.h"
#include "TinyISP_Programmer.h"
#include "TinyISP_SPI.h"
#include "TinyISP_TuningSignal.h"

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #define F(s) s
  #include <WProgram.h>
  #include <pins_arduino.h>
#endif


/*----------------------------------------------------------------------------*/

static const uint8_t Resp_STK_OK                = 0x10;  // DLE
static const uint8_t Resp_STK_FAILED            = 0x11;  // DC1
static const uint8_t Resp_STK_UNKNOWN           = 0x12;  // DC2
static const uint8_t Resp_STK_NODEVICE          = 0x13;  // DC3
static const uint8_t Resp_STK_INSYNC            = 0x14;  // DC4
static const uint8_t Resp_STK_NOSYNC            = 0x15;  // NAK

static const uint8_t Sync_CRC_EOP               = 0x20;  // SPACE

static const uint8_t Cmnd_STK_GET_SYNC          = 0x30;  // '0'
static const uint8_t Cmnd_STK_GET_SIGN_ON       = 0x31;  // '1'

static const uint8_t Cmnd_STK_SET_PARAMETER     = 0x40;  // '@'
static const uint8_t Cmnd_STK_GET_PARAMETER     = 0x41;  // 'A'
static const uint8_t Cmnd_STK_SET_DEVICE        = 0x42;  // 'B'
static const uint8_t Cmnd_STK_SET_DEVICE_EXT    = 0x45;  // 'E'

static const uint8_t Cmnd_STK_ENTER_PROGMODE    = 0x50;  // 'P'
static const uint8_t Cmnd_STK_LEAVE_PROGMODE    = 0x51;  // 'Q'
static const uint8_t Cmnd_STK_LOAD_ADDRESS      = 0x55;  // 'U'
static const uint8_t Cmnd_STK_UNIVERSAL         = 0x56;  // 'V'

static const uint8_t Cmnd_STK_PROG_FLASH        = 0x60;  // '`'
static const uint8_t Cmnd_STK_PROG_DATA         = 0x61;  // 'a'
static const uint8_t Cmnd_STK_PROG_PAGE         = 0x64;  // 'd'

static const uint8_t Cmnd_STK_READ_PAGE         = 0x74;  // 't'
static const uint8_t Cmnd_STK_READ_SIGN         = 0x75;  // 'u'

static const uint8_t Parm_STK_HW_VER            = 0x80;  // ' ' - R
static const uint8_t Parm_STK_SW_MAJOR          = 0x81;  // ' ' - R
static const uint8_t Parm_STK_SW_MINOR          = 0x82;  // ' ' - R
static const uint8_t Parm_STK_PROGMODE          = 0x93;  // ' ' - 'P' or 'S'

/*----------------------------------------------------------------------------*/

static const uint8_t VERSION_HARDWARE         = 0;  // Reported as zero so it can be distinguished from the bootloader or a "real" programmer.

static const uint8_t VERSION_SOFTWARE_MAJOR   = 1;  // Bump one of these when a new version is published.
static const uint8_t VERSION_SOFTWARE_MINOR   = 0;

/*----------------------------------------------------------------------------*/

bool programmer_active;

static uint8_t buff[256];

typedef struct param 
{
  uint8_t       devicecode;
  uint8_t       revision;
  uint8_t       progtype;
  uint8_t       parmode;  
  uint8_t       polling;
  uint8_t       selftimed;
  uint8_t       lockbytes;
  uint8_t       fusebytes;
  uint8_t       flashpoll;
  uint16_t      eeprompoll;
  uint16_t      pagesize;
  uint16_t      eepromsize;
  uint32_t      flashsize;
} 
parameter;

parameter param;

#if (HOLD_TARGET_IN_RESET_BY_DEFAULT)
static bool HoldInResetAfterProgramming = true;
#else
static bool HoldInResetAfterProgramming;
#endif

static bool HeldInReset;

static int here;  // fix? Change this to uint16_t?

/*----------------------------------------------------------------------------*/

// rmv? uint32_t debug_programmer_1;

/*----------------------------------------------------------------------------*/

static unsigned error_count;
static uint8_t error_first_mark;
static uint8_t error_extra;

static void set_error( uint8_t _mark, uint8_t _extra )
{
  ++error_count;

  if ( error_first_mark == 0 )
  {
    error_first_mark = _mark;
    error_extra = _extra;
  }

  StatusIndicator.error();
}

/*----------------------------------------------------------------------------*/

static uint8_t wait_for_byte( void )  // fix: rename to wait_for_byte
{
  while( ! Serial.available() );
  return( Serial.read() );
}

/*----------------------------------------------------------------------------*/

static void wait_for_buffer( int n ) // fix: rename to wait_for_buffer
{
  for ( int x = 0; x < n; x++ ) 
  {
    buff[x] = wait_for_byte();
  }
}

/*----------------------------------------------------------------------------*/

static void empty_reply( void ) 
{
  if ( Sync_CRC_EOP == wait_for_byte() ) 
  {
    Serial.write( Resp_STK_INSYNC );
    Serial.write( Resp_STK_OK );
  } 
  else 
  {
    set_error( 1, 0 );
    Serial.write( Resp_STK_NOSYNC );
  }
}

/*----------------------------------------------------------------------------*/

static void byte_reply( uint8_t b )
{
  if ( Sync_CRC_EOP == wait_for_byte() ) 
  {
    Serial.write( Resp_STK_INSYNC );
    Serial.write( b );
    Serial.write( Resp_STK_OK );
  } 
  else 
  {
    set_error( 2, 0 );
    Serial.write( Resp_STK_NOSYNC );
  }
}

/*----------------------------------------------------------------------------*/

static void get_parameter( uint8_t c ) 
{
  switch( c ) 
  {
    case Parm_STK_HW_VER:
      byte_reply( VERSION_HARDWARE );
      break;

    case Parm_STK_SW_MAJOR:
      byte_reply( VERSION_SOFTWARE_MAJOR );
      break;

    case Parm_STK_SW_MINOR:
      byte_reply( VERSION_SOFTWARE_MINOR );
      break;

    case Parm_STK_PROGMODE:
      byte_reply( 'S' ); // serial programmer
      break;

    default:
      byte_reply( 0 );
      break;
  }
}

/*----------------------------------------------------------------------------*/

#define beget16(addr) (*addr * 256 + *(addr+1) )

static void set_parameters( void ) 
{
  // call this after reading paramter packet into buff[]
  param.devicecode  = buff[0];
  param.revision    = buff[1];
  param.progtype    = buff[2];
  param.parmode     = buff[3];
  param.polling     = buff[4];
  param.selftimed   = buff[5];
  param.lockbytes   = buff[6];
  param.fusebytes   = buff[7];
  param.flashpoll   = buff[8];
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll  = beget16( &buff[10] );
  param.pagesize    = beget16( &buff[12] );
  param.eepromsize  = beget16( &buff[14] );

  // 32 bits flashsize (big endian)
  param.flashsize = 
      buff[16] * 0x01000000
      + buff[17] * 0x00010000
      + buff[18] * 0x00000100
      + buff[19];
}

/*----------------------------------------------------------------------------*/

static void Handle_SET_DEVICE_EXT( void )
{
  uint8_t n_extparms_plus_one;
  uint8_t i;

  // meaning of the data in the format <offset>: <description>
  // 0: number of following parameters + 1
  // 1: EEPROM page size or 0
  // 2: pagel from config file (only needed for parallel programming)
  // 3: bs2 from config file (only needed for parallel programming)
  // 4: 0=(reset_disposition=RESET_DEDICATED); 1=other

  n_extparms_plus_one = wait_for_byte();

  for ( i=1; i < n_extparms_plus_one; ++i )
  {
    wait_for_byte();
  }
  empty_reply();
}

/*----------------------------------------------------------------------------*/

static void reset_toggle( void )
{
  noInterrupts();
  digitalWrite( RESET, HIGH );

//    delayMicroseconds (2);  // pulse for at least 2 clock cycles

  #if (PROGRAMMER_SPI_CLOCK == SLOW)
    // The pulse duration must be at least tRST (< 2500ns) plus two CPU clock cycles.
    // 2.5 us + 2 cycles / 128000 cycles per second = 2.5 us + 15.625 us
    delayMicroseconds( 3 + 16 );
  #elif (PROGRAMMER_SPI_CLOCK == SAFE) || (PROGRAMMER_SPI_CLOCK == NORMAL)
    // The pulse duration must be at least tRST (< 2500ns) plus two CPU clock cycles.
    // 2.5 us + 2 cycles / 1000000 cycles per second = 2.5 us + 2 us
    delayMicroseconds( 3 + 2 );
  #elif (PROGRAMMER_SPI_CLOCK == FAST)
    // The pulse duration must be at least tRST (< 2500ns) plus two CPU clock cycles.
    // 2.5 us + 2 cycles / 16000000 cycles per second = 2.5 us + 0.125 us
    delayMicroseconds( 3 );
  #else
    #error Unsupported SPI clock in reset_toggle; no way to determine the RESET pulse duration.
  #endif

  digitalWrite( RESET, LOW );
  interrupts();
}

/*----------------------------------------------------------------------------*/

static void reset_target( void )
{
  if ( ! HeldInReset )
  {
    // Tuning signal should be turned off before progamming the target in case the pins overlap
    tuning_signal_control( tsStop );

    // Hold the target in reset
    pinMode( RESET, OUTPUT );
    digitalWrite( RESET, LOW );  // rmv?  This should not be necessary.

    // Ensure SCK is in a known state
    pinMode( SCK, OUTPUT );
    digitalWrite( SCK, LOW );

    // Give RESET a quick toggle to ensure the target is in the desired state
    reset_toggle();

    HeldInReset = true;
  }
}

/*----------------------------------------------------------------------------*/

static void programmer_start_tuning_signal( void )
{
  #if TUNING_SIGNAL_ENABLED
    #if TUNING_SIGNAL_OUTPUT_LONG_PULSE
      tuning_signal_control( tsLongPulse );
    #else
      tuning_signal_control( tsShortPulse );
    #endif
  #endif
}

/*----------------------------------------------------------------------------*/

static bool release_target( void )
{
  if ( HeldInReset )
  {
    // Input / pullup disabled
    pinMode( SCK, INPUT );

    // Generate a tuning signal in case the target wants to tune
    programmer_start_tuning_signal();

    // Input / pullup disabled
    pinMode( RESET, INPUT );

    HeldInReset = false;

    return( true );
  }
  return( false );
}

/*----------------------------------------------------------------------------*/

static void programmer_activate( void ) 
{
  uint32_t rv;
  int8_t tries;

  reset_target();

  // For Master mode, enabling SPI should be done after RESET (SS) is turned into an output.  For this application, it is a difference without a distinction because RESET (SS) is held high by a pullup resistor.
  spi_begin();

  // rmv: ATmega328 datasheet calls for at least 20 ms.  50 ms is excessive.
  //delay( 50 ); 
  delay( 20 );

  // Prepare the data lines
  pinMode( MISO, INPUT );
  pinMode( MOSI, OUTPUT );

  // Send Programming Enable to the target

  tries = 0;
  do
  {
    rv = spi_transaction2( 0xAC530000 );

    #if INTERNAL_DEBUGGING_TO_HARDWARESERIAL
      InternalDebugging.print( F( "Programming Enable: " ) );
      InternalDebugging.print( rv, HEX );
      InternalDebugging.println();
    #endif

    if ( (rv & 0x0000FF00) == 0x00005300 )
    {
      break;
    }
    else
    {
      #if INTERNAL_DEBUGGING_TO_HARDWARESERIAL
        InternalDebugging.println( F( "  Toggling RESET and trying again." ) );
      #endif
      reset_toggle();
      delay( 20 );
    }

    ++tries;
  }
  while ( tries < 3 );

  programmer_active = true;
}

/*----------------------------------------------------------------------------*/

static void programmer_deactivate( void ) 
{
  pinMode( MISO, INPUT );
  pinMode( MOSI, INPUT );

  spi_end();

  if ( HoldInResetAfterProgramming )
  {
/* rmv: Not necessary.  HeldInReset has to be true at this point.
    HeldInReset = true; */
  }
  else
  {
    release_target();
  }

  programmer_active = false;
}

/*----------------------------------------------------------------------------*/

static void universal( void ) 
{
  uint8_t ch;

  wait_for_buffer( 4 );
  ch = spi_transaction( buff[0], buff[1], buff[2], buff[3] );
  byte_reply( ch );
}

/*----------------------------------------------------------------------------*/

static void flash( uint8_t hilo, int addr, uint8_t data ) 
{
  spi_transaction( 0x40+8*hilo, addr>>8 & 0xFF, addr & 0xFF, data );
}

/*----------------------------------------------------------------------------*/

static void commit( int addr ) 
{
  uint8_t RdyBsy;

  spi_transaction( 0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0 );

  StatusIndicator.flash();

  unsigned long Start;

  Start = millis();

  while ( millis() - Start < 30 )
  {
    StatusIndicator.update();

    if ( param.polling )
    {
      RdyBsy = spi_transaction( 0xF0, 0x00, 0x00, 0x00 );

      if ( (RdyBsy & 0x01) == 0x00 )
        break;
    }
  }
}

/*----------------------------------------------------------------------------*/

// rmv #define _current_page(x) (here & 0xFFFFE0)

static int current_page( int addr ) 
{
  if ( param.pagesize == 32 ) return here & 0xFFFFFFF0;
  if ( param.pagesize == 64 ) return here & 0xFFFFFFE0;
  if ( param.pagesize == 128 ) return here & 0xFFFFFFC0;
  if ( param.pagesize == 256 ) return here & 0xFFFFFF80;
  return here;
}

/*----------------------------------------------------------------------------*/

static uint8_t write_flash_pages( int length ) 
{
  int x = 0;

  int page = current_page( here );

  while ( x < length ) 
  {
    if ( page != current_page( here ) ) 
    {
      commit( page );
      page = current_page( here );
    }
    flash( LOW, here, buff[x++] );
    flash( HIGH, here, buff[x++] );
    here++;
  }

  commit( page );

  return( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

static void write_flash( int length ) 
{
  wait_for_buffer( length );

  if ( Sync_CRC_EOP == wait_for_byte() ) 
  {
    Serial.write( Resp_STK_INSYNC );
    Serial.write( write_flash_pages( length ) );
  } 
  else 
  {
    set_error( 3, 0 );
    Serial.write( Resp_STK_NOSYNC );
  }
}

/*----------------------------------------------------------------------------*/

// write (length) bytes, (start) is a byte address

static uint8_t write_eeprom_chunk( int start, int length ) 
{
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)
  wait_for_buffer( length );

  for ( int x = 0; x < length; x++ ) 
  {
    int addr = start+x;
    spi_transaction( 0xC0, (addr>>8) & 0xFF, addr & 0xFF, buff[x] );
    delay( 45 );
  }

  StatusIndicator.flash();

  return( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

#define EECHUNK (32)

static uint8_t write_eeprom( int length ) 
{
  // here is a word address, get the byte address
  int start = here * 2;

  int remaining = length;

  if ( length > param.eepromsize ) 
  {
    set_error( 4, 0 );
    return( Resp_STK_FAILED );
  }
  while ( remaining > EECHUNK ) 
  {
    write_eeprom_chunk( start, EECHUNK );
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk( start, remaining );
  
  return( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

static uint8_t flash_read( uint8_t hilo, int addr ) 
{
  return spi_transaction( 0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0 );
}

/*----------------------------------------------------------------------------*/

static uint8_t flash_read_page( int length ) 
{
  for ( int x = 0; x < length; x+=2 ) 
  {
    uint8_t low = flash_read( LOW, here );
    Serial.write( low );

    uint8_t high = flash_read( HIGH, here );
    Serial.write( high );

    here++;
  }
  return( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

static uint8_t eeprom_read_page( int length ) 
{
  // here again we have a word address
  int start = here * 2;

  for ( int x = 0; x < length; x++ ) 
  {
    int addr = start + x;

    uint8_t ee = spi_transaction( 0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF );

    Serial.write( ee );
  }
  return( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

static void read_page( void ) 
{
  uint8_t result = Resp_STK_FAILED;
  int length;
  char memtype;
  
  length = 256 * wait_for_byte();
  length = length | wait_for_byte();
  memtype = wait_for_byte();
  
  if ( Sync_CRC_EOP != wait_for_byte() ) 
  {
    set_error( 6, 0 );
    Serial.write( Resp_STK_NOSYNC );
    return;
  }
  Serial.write( Resp_STK_INSYNC );
  if ( memtype == 'F' ) result = flash_read_page( length );
  if ( memtype == 'E' ) result = eeprom_read_page( length );
  Serial.write( result );
  
  StatusIndicator.flash();
}

/*----------------------------------------------------------------------------*/

static void read_signature( void ) 
{
  if ( Sync_CRC_EOP != wait_for_byte() ) 
  {
    set_error( 7, 0 );
    Serial.write( Resp_STK_NOSYNC );
    return;
  }
  Serial.write( Resp_STK_INSYNC );

  uint8_t high = spi_transaction( 0x30, 0x00, 0x00, 0x00 );
  Serial.write( high );

  uint8_t middle = spi_transaction( 0x30, 0x00, 0x01, 0x00 );
  Serial.write( middle );

  uint8_t low = spi_transaction( 0x30, 0x00, 0x02, 0x00 );
  Serial.write( low );

  Serial.write( Resp_STK_OK );
}

/*----------------------------------------------------------------------------*/

static void program_page( void ) 
{
  uint8_t result = Resp_STK_FAILED;
  int length;
  char memtype;
  
  length = 256 * wait_for_byte();
  length = length | wait_for_byte();
  memtype = wait_for_byte();
  
  // flash memory @here, (length) bytes
  if ( memtype == 'F' ) 
  {
    write_flash( length );
    return;
  }

  if ( memtype == 'E' ) 
  {
    result = write_eeprom( length );

    if ( Sync_CRC_EOP == wait_for_byte() ) 
    {
      Serial.write( Resp_STK_INSYNC );
      Serial.write( result );
    } 
    else 
    {
      set_error( 5, 0 );
      Serial.write( Resp_STK_NOSYNC );
    }
    return;
  }
  Serial.write( Resp_STK_FAILED );
  return;
}

/*----------------------------------------------------------------------------*/

static unsigned command_count;
static unsigned signons;

void programmer_process_command( uint8_t ch ) 
{ 
  static uint8_t last_command; // rmv
  uint8_t data;
  uint8_t low;
  uint8_t high;
  
  switch ( ch ) 
  {
    case Cmnd_STK_GET_SYNC: // rmv '0': // signon
      ++command_count;
//fix        error_count = 0;
//fix        error_first_mark = 0;
//fix        reset the StatusIndicator?
      empty_reply();
      ++signons;
      break;

    case Cmnd_STK_GET_SIGN_ON: // rmv '1':
      ++command_count;
      if ( wait_for_byte() == Sync_CRC_EOP ) 
      {
        Serial.write( Resp_STK_INSYNC );
        Serial.print( F( "AVR ISP" ) );
        Serial.write( Resp_STK_OK );
      }
      break;

    case Cmnd_STK_GET_PARAMETER: // rmv 'A':
      ++command_count;
      get_parameter( wait_for_byte() );
      break;

    case Cmnd_STK_SET_DEVICE: // rmv 'B':
      ++command_count;
      wait_for_buffer( 20 );
      set_parameters();
      empty_reply();
      break;

    case Cmnd_STK_SET_DEVICE_EXT: // rmv 'E': // extended parameters - ignore for now
      ++command_count;
      Handle_SET_DEVICE_EXT();
      break;
  
    case Cmnd_STK_ENTER_PROGMODE: // rmv 'P':
      ++command_count;
      programmer_activate();
      empty_reply();
      break;

    case Cmnd_STK_LOAD_ADDRESS: // rmv 'U': // set address (word)
      ++command_count;
      here = wait_for_byte();
      here = here | (256 * wait_for_byte());
      empty_reply();
      break;
  
    case Cmnd_STK_PROG_FLASH: // rmv 0x60: //STK_PROG_FLASH
      ++command_count;
      low = wait_for_byte();
      high = wait_for_byte();
      empty_reply();
      break;

    case Cmnd_STK_PROG_DATA: // rmv 0x61: //STK_PROG_DATA
      ++command_count;
      data = wait_for_byte();
      empty_reply();
      break;
  
    case Cmnd_STK_PROG_PAGE: // rmv 0x64: //STK_PROG_PAGE 'd'
      ++command_count;
      program_page();
      break;
      
    case Cmnd_STK_READ_PAGE: // rmv 0x74: //STK_READ_PAGE 't'
      ++command_count;
      read_page();    
      break;
  
    case Cmnd_STK_UNIVERSAL: // rmv 'V': //0x56
      ++command_count;
      universal();
      break;

    case Cmnd_STK_LEAVE_PROGMODE: // rmv 'Q': //0x51
      ++command_count;
//fix        error_count = 0;
//fix        error_first_mark = 0;
//fix        reset the StatusIndicator?
      programmer_deactivate();
      empty_reply();
      break;
      
    case Cmnd_STK_READ_SIGN: // rmv 0x75: //STK_READ_SIGN 'u'
      ++command_count;
      read_signature();
      break;
  
    // expecting a command, not Sync_CRC_EOP
    // this is how we can get back in sync
    case Sync_CRC_EOP:
      if ( programmer_active )
        set_error( 8, last_command );
      Serial.write( Resp_STK_NOSYNC );
      break;
      
    // anything else we will return Resp_STK_UNKNOWN
    default:
      if ( programmer_active )
        set_error( 10, ch );

      if ( (ch == 0x0A) || (ch == 0x0D) )
        set_error( 13, ch );

      // fix? If we're not in "program mode" should we return anything?
      if ( Sync_CRC_EOP == wait_for_byte() ) 
        Serial.write( Resp_STK_UNKNOWN );
      else
        Serial.write( Resp_STK_NOSYNC );
      break;
  }

  last_command = ch;
}

/*----------------------------------------------------------------------------*/

bool programmer_toggle_hold_in_reset( void )
{
  HoldInResetAfterProgramming = ! HoldInResetAfterProgramming;

  return( HoldInResetAfterProgramming );
}

/*----------------------------------------------------------------------------*/

bool programmer_release_target_from_reset( void )
{
  return( release_target() );
}

/*----------------------------------------------------------------------------*/

void programmer_reset_target( void )
{
  if ( ! release_target() )
  {
    unsigned long Start;

    pinMode( RESET, OUTPUT );
    Start = millis();
    while ( millis() - Start < 50 );
    programmer_start_tuning_signal();
    pinMode( RESET, INPUT );
  }
}

/*----------------------------------------------------------------------------*/

void programmer_hold_target_in_reset( void )
{
  reset_target();
}

/*----------------------------------------------------------------------------*/

bool programmer_error_occurred( void )
{
  return( error_count > 0 );
}

/*----------------------------------------------------------------------------*/
