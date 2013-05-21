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
//! @brief   Test application for transmitting CAN Frames
//!
//! @version 1.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>

#include <libpican.h>

#include "CanTest.h"
#include "Exceptions.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________
CANBUS* can;

//_____ L O C A L S ____________________________________________________________
const std::string DEFAULT_FILE = "msg";
const std::string DEFAULT_DEV = "/dev/rpi_can0";
const unsigned int DEFAULT_BITRATE = 125000;

//_____ F U N C T I O N S ______________________________________________________

void print_diag() {
  TPDIAG diag;
  int err = CAN_Statistics( can, &diag );
  
  if( err ) {
    std::cerr << "Can't read diagnostics!\n"
              << "Error " << err << ": " << strerror( err )
              << std::endl;
  } else {
    double runtime = CanTest::getInstance()->runtime();
    std::cout << "Test results:\n"
              << "  count of reads    = " << diag.dwReadCounter << "\n"
              << "  count of writes   = " << diag.dwWriteCounter << "\n"
              << "  count of writes/s = " << (double)diag.dwWriteCounter / runtime << "\n"
              << "  count of errors   = " << diag.dwErrorCounter << "\n"
              << "  count of irqs     = " << diag.dwIRQcounter << "\n"
              << "  last CAN status   = 0x" << std::hex << diag.wErrorFlag << std::dec << "\n"
              << "  last error        = " << diag.nLastError << "\n"
              << "  driver version    = " << diag.szVersionString << "\n"
              << std::endl;
  }
}

//------------------------------------------------------------------------------

void do_exit() {
  if ( can ) {
    print_diag();
    CAN_Close( can );
  }
}

//------------------------------------------------------------------------------

static void signal_handler( int signal ) {
  do_exit();
}

//------------------------------------------------------------------------------

static void helpmsg() {
  std::cout << "rpi_can_test - a small test program which sends/receives CAN messages.\n"
            << "usage:   rpi_can_test [-f=filename] [-d=device] [-b=bitrate] [-n=count] [-r=msec] [-?] mode\n"
            << "options: mode  - mandatory modus {receive|transmit}.\n"
            << "         -f    - name of message description file, default = " << DEFAULT_FILE << "\n"
            << "         -d    - path to device file, default=" << DEFAULT_DEV << "\n"
            << "         -b    - Bitrate, default = " << DEFAULT_BITRATE << "\n"
            << "         -n    - maximum number of messages to send/receive, default=INF\n"
            << "         -r    - messages are send at random times with maximum time in milliseconds (msec).\n"
            << "         -?/-h - show this screen.\n"
            << std::endl;
}

//------------------------------------------------------------------------------

int main( int argc, char* argv[] ) {
  std::string filename;
  std::string devicename;
  std::string mode;
  unsigned int bitrate = 0;
  unsigned int count = 0;
  unsigned int speed = 0;

  if ( argc < 2 ) {
    std::cout << "Missing argument" << std::endl;
    return -1;
  }

  // parse command line options
  for( int i = 0; i < argc; i++ ) {
    
    if ( strcmp( argv[i], "-?" ) == 0 || strcmp( argv[i], "-h" ) == 0 ) {
      helpmsg();
      return 0;
    }

    char* ptr = argv[i];
    
    if( '-' == *ptr ) {
      ptr++;
      char c = *ptr;
      ptr++;
      if ( '=' != *ptr ) {
        std::cerr << "Missing argument to option '-" << c << "'\n" << std::endl;
        helpmsg();
        return -1;
      }
      ptr++;  
      
      std::istringstream dummy( ptr );
      
      switch( tolower(c) ) {
      case 'f':
        dummy >> filename;
        break;
      case 'd':
        dummy >> devicename;
        break;
      case 'b':
        dummy >> bitrate;
        if ( 0 == bitrate ) {
          std::cerr << "Invalid argument for option 'b'\n" << std::endl;
          helpmsg();
          return -1;
        }
        break;
      case 'n':
        dummy >> count;
        break;
      case 'r':
        dummy >> speed;
        break;
      default:
        std::cerr << "Unrecognized option '" << c << "'\n" << std::endl;
        helpmsg();
        return -1;
      }
    } else {
      mode = argv[i];
    }
  }

  if ( filename.empty() ) filename = DEFAULT_FILE;

  CanTest::create( filename );

  if ( devicename.empty() ) devicename = DEFAULT_DEV;
  can = CAN_Open( devicename.c_str(), O_RDWR );
  if( !can ) {
    std::cerr << "Cannot open device: '" << devicename << "'!\n'";
    return -1;
  }
  
  if( 0 == bitrate ) bitrate = DEFAULT_BITRATE;
  if( CAN_Bitrate( can, bitrate ) ) {
    std::cerr << "Cannot set bitrate: " << bitrate << "\n"
              << "Error " << errno << " " << strerror( errno )
              << std::endl;
    return -1;
  }
  
  signal( SIGTERM, signal_handler );
  signal( SIGINT, signal_handler );

  try {  
    if ( mode.compare( "transmit" ) == 0 )
      CanTest::getInstance()->transmitTest( speed, count );
    if ( mode.compare( "receive" ) == 0 )
      CanTest::getInstance()->receiveTest();
  } catch( const CanFailure& e ) {
    std::cerr << e.what() << std::endl;
  }
  do_exit();

  return 0;
}
