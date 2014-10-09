//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Ruhr-Universitaet Bochum, Lehrstuhl fuer Experimentalphysik I
//
// This file is part of rpi_can kernel module
//
// rpi_can is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// rpi_can is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// brief   Test application for transmitting/receiving CAN Frames
//
// version 1.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

#include "CanTest.h"
#include "Exceptions.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const std::string  DEFAULT_DEV     = "can0";
static const unsigned int DEFAULT_BITRATE = 125000;

//_____ F U N C T I O N S ______________________________________________________

static void signal_handler( int signal ) {
  CanTest::getInstance()->CanClose();
}

//------------------------------------------------------------------------------

static void helpmsg() {
  std::cout << "rpi_can_test - a small test program which sends/receives CAN messages.\n"
            << "usage:   rpi_can_test [-f=filename] [-d=device] [-b=bitrate] [-n=count] [-r=msec] [-?] mode\n"
            << "options: mode  - mandatory modus {receive|transmit}.\n"
            << "         -f    - name of message description file\n"
            << "                 (if no filename is given, 20 random messages are generated)\n"
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

  if ( devicename.empty() )  devicename = DEFAULT_DEV;
  if ( 0 == bitrate )        bitrate    = DEFAULT_BITRATE;

  CanTest::create( filename, devicename );
  CanTest::getInstance()->setBitrate( bitrate );
  CanTest::getInstance()->CanOpen();
    
  signal( SIGTERM, signal_handler );
  signal( SIGINT, signal_handler );

  try {  
    if ( mode.compare( "transmit" ) == 0 )
      CanTest::getInstance()->transmitTest( speed, count );
    if ( mode.compare( "receive" ) == 0 )
      CanTest::getInstance()->receiveTest( count );
  } catch( const CanFailure& e ) {
    std::cerr << e.what() << std::endl;
  }

  CanTest::getInstance()->CanClose();

  return 0;
}
