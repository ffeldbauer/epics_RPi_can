//******************************************************************************
//! Copyright (C) 2012 Florian Feldbauer
//!
//! This program is free software; you can redistribute it and/or modify
//! it under the terms of the GNU General Public License as published by
//! the Free Software Foundation; either version 3 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//! GNU General Public License for more details.
//!
//! You should have received a copy of the GNU General Public License
//! along with this program; if not, write to the Free Software
//! Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//!
//! This source code is based on peak-linux-driver-7.6
//!   linux@peak-system.com
//!   www.peak-system.com
//!
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   read input parser and write output formatter
//!
//! @version 2.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <linux/errno.h>       // error codes
#include <linux/kernel.h>      // DPRINTK()

#include "can_parse.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//----------------------------------------------------------------------------
// helper for use in read..., makes a line of formatted output
int can_make_output( char* buffer, struct can_frame* m ) {
  char* ptr = buffer;
  int i;
  char r_or_m, s_or_e;

  *buffer = 0;

  if( m->can_id & CAN_ERR_FLAG ) {
    // any status frames are x-ed
    r_or_m = 'x';
    s_or_e = 'x';
  } else {
    r_or_m = ( m->can_id & CAN_RTR_FLAG ) ? 'r' : 'm';
    s_or_e = ( m->can_id & CAN_EFF_FLAG ) ? 'e' : 's';
  }

  // print RTR, 11 or 29, CAN-Id and datalength
  sprintf(ptr, "%c %c 0x%08x %1d  ", r_or_m, s_or_e, ( m->can_id & CAN_EFF_MASK ), m->can_dlc);

  // search the end
  while( *ptr )
    ptr++;

  // "Vorsicht ist die Mutter der Porzellankiste!"
  // or "Better safe than sorry"
  if ( m->can_dlc > 8 ) {
    for( i = 0; i < 8; i++ ) {
      sprintf(ptr, "---- ");
      ptr += 3;
    }
  } else {
    // print data
    u8 ucLen;

    // don't print any data if it is a RTR message
    if( m->can_id & CAN_RTR_FLAG )
      ucLen = 0;
    else
      ucLen =  m->can_dlc;

    for( i = 0; i < ucLen; i++ ) {
      sprintf( ptr, "0x%02x ", m->data[i] );
      ptr += 5;
    }
  }

  sprintf( ptr, "\n" );

  // search the end
  while (*ptr)
    ptr++;

  return (int)(ptr - buffer);
}

//----------------------------------------------------------------------------
// skip blanks and tabs
static inline void skip_blanks(char **ptr) {
  // remove blanks or tabs
  while ((**ptr == ' ') || (**ptr == '\t'))
    (*ptr)++;
}

//----------------------------------------------------------------------------
// skip blanks, return 0 if the 1st non-blank char is not '\n'
static int skip_blanks_and_test_for_CR(char **ptr) {
  // remove blanks or tabs
  skip_blanks(ptr);

  if (**ptr == '\n')
    return -1;
  else
    return 0;
}

//----------------------------------------------------------------------------
// extract a number, either hex or decimal from a string
static int scan_unsigned_number(char **ptr, u32 *dwResult) {
  char *p = *ptr;

  *dwResult = simple_strtoul(p, ptr, 0);

  if (p != *ptr)
    return 0;

  return -ERANGE;
}

//----------------------------------------------------------------------------
// extract a char from a string
static inline char scan_char(char **ptr) {
  return *(*ptr)++;
}

//----------------------------------------------------------------------------
// lengthy helper for use in write..., reject empty and comment lines
int can_parse_input_idle(char *buffer) {
  char *ptr = buffer;

  // remove leading blanks
  skip_blanks(&ptr);

  // search for 'm' or 'r' to distinguish between message or init strings
  switch (scan_char(&ptr)) {
  case '#':  // comment
  case '\n': return 0;

  default:   return -EINVAL;
  }
}

//----------------------------------------------------------------------------
// lengthy helper for use in write..., parses a message command
int can_parse_input_message(char *buffer, struct can_frame *Message) {
  char *ptr = buffer;
  u32 dwLen;
  u32 dwDat;
  u32 ID;
  int i = 0;
  int err = -EINVAL;

  // remove leading blanks
  skip_blanks(&ptr);

  // search for 'm' or 'r' to distinguish between message or init strings
  Message->can_id = 0;
  switch (scan_char(&ptr)) {
  case 'm':  break; // normal message
  case 'r':  Message->can_id |= CAN_RTR_FLAG; break; // rtr message

  default:   goto reject;
  }
  if ( skip_blanks_and_test_for_CR( &ptr ) ) // no CR allowed here
    goto reject;

  // read message type
  switch (scan_char(&ptr)) {
  case 's': break;
  case 'e': Message->can_id |= CAN_EFF_FLAG; break;

  default:  goto reject;
  }
  if (skip_blanks_and_test_for_CR(&ptr))
    goto reject;

  // read CAN-ID
  if ( ( err = scan_unsigned_number(&ptr, &ID) ) )
    goto reject;
  if ( Message->can_id & CAN_EFF_FLAG ) {
    if( ID > CAN_EFF_MASK )
      goto reject;
    else
      Message->can_id |= ID;
  } else {
    if( Message->can_id > CAN_SFF_MASK )
      goto reject;
    else
      Message->can_id |= ID;
  }
  if ( skip_blanks_and_test_for_CR( &ptr ) )
    goto reject;

  //read datalength
  if ( ( err = scan_unsigned_number( &ptr, &dwLen ) ) )
    goto reject;
  if ( dwLen > 8 )
    goto reject;
  Message->can_dlc = (u8)dwLen;

  if ( !(Message->can_id & CAN_RTR_FLAG) ) {
    // read data elements up to message len
    // Only if data message
    while (i < dwLen) {
      if ( skip_blanks_and_test_for_CR( &ptr ) )
        goto reject;

      if ( ( err = scan_unsigned_number( &ptr, &dwDat ) ) )
        goto reject;
      if (dwDat > 255)
        goto reject;
      Message->data[i] = (u8)dwDat;

      i++;
    }
  }
  return 0;

 reject:
  return err;
}

//----------------------------------------------------------------------------
// lengthy helper for use in write..., parses a init command
int can_parse_input_init(char *buffer, TPBTR0BTR1 *Init) {
  char *ptr = buffer;
  u32 dwDummy;
  int err = -EINVAL;

  // remove leading blanks
  skip_blanks( &ptr );

  // is it really a init string
  if ( scan_char( &ptr ) != 'i')
    goto reject;

  // parse init string, a CR is not allowed here
  if ( skip_blanks_and_test_for_CR( &ptr ) )
    goto reject;

  // get BTR0BTR1
  if ( ( err = scan_unsigned_number(&ptr, &dwDummy) ) )
    goto reject;
  if ( 1000000 < dwDummy )
    goto reject;

  Init->dwBitRate  = dwDummy;

  return 0;

 reject:
  return err;
}


