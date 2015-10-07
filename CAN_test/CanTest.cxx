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

// ANSI C++ headers
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <libsocketcan.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <unistd.h>

// local headers
#include "CanTest.h"
#include "Exceptions.h"

//_____ D E F I N I T I O N S __________________________________________________
typedef  struct can_frame     can_frame_t;
typedef  struct sockaddr_can  sockaddr_can_t;
typedef  struct sockaddr      sockaddr_t;
typedef  struct ifreq         ifreq_t; 

//_____ G L O B A L S __________________________________________________________
CanTest* CanTest::_pinstance = NULL;

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________
bool operator==( const can_frame_t& lhs, const can_frame_t& rhs ) {
  if ( ( lhs.can_id != rhs.can_id ) || ( lhs.can_dlc != rhs.can_dlc ) )
    return false;

  for ( epicsUInt8 i = 0; i < lhs.can_dlc; i++ ) {
    if( lhs.data[i] != rhs.data[i] ) return false;
  }
  return true;
}

//------------------------------------------------------------------------------

CanTest::CanTest()
  : _devName( "can0" ),
    _start(0),
    _stop(0),
    _counter(0),
    _open( false ),
    _mismatched(0)
{ 
  ParseMessages();
}

//------------------------------------------------------------------------------

CanTest::CanTest( std::string& filename, std::string& devicename )
  : _filename( filename ),
    _devName( devicename ),
    _start( 0 ),
    _stop( 0 ),
    _counter( 0 ),
    _open( false ),
    _mismatched( 0 )
{ 
  ParseMessages();
}

//------------------------------------------------------------------------------

CanTest::CanTest( const CanTest& rother ) {
}

//------------------------------------------------------------------------------

CanTest::~CanTest() {
  _msgList.clear();
}

//------------------------------------------------------------------------------

bool CanTest::exists() {
  return ( NULL != _pinstance );
}

//------------------------------------------------------------------------------

CanTest* CanTest::getInstance () {  
  if ( NULL == _pinstance ) throw BadConfig("CanTest base class has not been created");
  return _pinstance;
}

//------------------------------------------------------------------------------

void CanTest::create( std::string& file, std::string& device ) {
  if ( NULL != _pinstance )
    std::cerr << "CanTest Singleton has already been created" << std::endl;
  else
    _pinstance = new CanTest( file, device );
  return;
}

//------------------------------------------------------------------------------

bool CanTest::CanOpen() {
  sockaddr_can_t addr;
  ifreq_t ifr;

  // open socket
  _socket = socket( PF_CAN, SOCK_RAW, CAN_RAW );
  if( _socket < 0 ) {
    perror( "Error while opening socket" );
    return false;
  }
  
  strcpy( ifr.ifr_name, _devName.c_str() );
  ioctl( _socket, SIOCGIFINDEX, &ifr );
 
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
 
  if( bind( _socket, (sockaddr_t*)&addr, sizeof( addr ) ) < 0 ) {
    perror( "Error in socket bind" );
    close( _socket ); 
    return false;
  }
  std::cout << "Opened CAN socket " << _socket << std::endl;
  _open = true;
  return true;
}

//------------------------------------------------------------------------------

void CanTest::CanClose() {
  if ( _open ){
    printDiag();
    close( _socket );
    _open = false;
    std::cout << "Closed CAN socket " << _socket << std::endl;
  }
}

//------------------------------------------------------------------------------

void CanTest::printDiag() {
  struct can_device_stats* cds = new struct can_device_stats;
  struct can_berr_counter* bc = new struct can_berr_counter;
  int err = 0;
  
  err = can_get_device_stats( _devName.c_str(), cds );
  if( err ) {
    std::cerr << "Can't read diagnostics!" << std::endl;
    return;
  }
  
  err = can_get_berr_counter( _devName.c_str(), bc );
  if( err ) {
    std::cerr << "Can't read error counter!" << std::endl;
    return;
  }

  if ( 0 == _stop ) time( &_stop );
  double runtime = difftime( _stop, _start );
  double fps = _counter / runtime;
  
  std::cout << "Test results:\n"
            << "  Bus errors                      " << cds->bus_error << "\n"
            << "  Changes to error warning state  " << cds->error_warning << "\n"
            << "  Changes to error passive state  " << cds->error_passive << "\n"
            << "  Changes to bus off state        " << cds->bus_off << "\n"
            << "  Arbitration lost errors         " << cds->arbitration_lost << "\n"
            << "  CAN controller re-starts        " << cds->restarts << "\n"
            << "  CAN Tx errors                   " << bc->txerr << "\n"
            << "  CAN Rx errors                   " << bc->rxerr << "\n\n"
            << "  Bitrate                         " << _bitrate << "\n"
            << "  Frames received/transmitted     " << _counter << "\n"
            << "  Frames/s                        " << fps << "\n"
            << "  Mismatched counted              " << _mismatched << "\n"
            << std::endl;
}

//------------------------------------------------------------------------------

void CanTest::setBitrate( unsigned int bitrate ){
  int err = 0;
  bool wasOpen = false;

  if( _open ) { CanClose(); wasOpen = true; }

  err = can_do_stop( _devName.c_str() );
  if( err ) {
    perror( "Could not bring interface down" );
    exit(1);
  }
  err = can_set_bitrate( _devName.c_str(), bitrate );
  if( err ) {
    perror( "Could not set bitrate" );
    exit(1);
  }
  err = can_do_start( _devName.c_str() );
  if( err ) {
    perror( "Could not bring interface up" );
    exit(1);
  }

  _bitrate = bitrate;

  if( wasOpen ) CanOpen();
}

//------------------------------------------------------------------------------

void CanTest::ParseMessages() {
  if( !_filename.empty ) {
    std::ifstream input( _filename.c_str(), std::ios_base::in );
    std::string line;
  
    while( std::getline( input, line ) ) {
      can_frame_t *pframe = new can_frame_t;

      // handle comments in config file
      size_t comment;
      if ( ( comment = line.find('#') ) != std::string::npos )
        line.erase( comment ); // erase everthing after first '#' from string
      
      if ( 0 == line.length() ) continue; // string is empty, no need to parse it
        
      std::istringstream parse( line );
      char rtr;
      char ide;
      unsigned int dummydlc = 0;
      parse >> rtr >> ide >> std::hex >> pframe->can_id >> dummydlc;
      switch( rtr ) {
      case 'm': break;
      case 'r': pframe->can_id |= CAN_RTR_FLAG; break;
      default:
        std::cerr << "Invalid format: '"<< rtr << "' in '" << line << "'" << std::endl;
        continue;
      }
      switch( ide ) {
      case 's': break;
      case 'e': pframe->can_id |= CAN_EFF_FLAG; break;
      default:
        std::cerr << "Invalid type: '"<< ide << "' in '" << line << "'" << std::endl;
        continue;
      }

      if ( 8 < dummydlc ) {
        std::cerr << "Invalid length: '"<< dummydlc << "' in '" << line << "'" << std::endl;
        continue;
      }
      pframe->can_dlc = (unsigned char)(dummydlc & 8);
      for ( unsigned char i = 0; i < pframe->can_dlc; i++ )
        parse >> std::hex >> pframe->data[i];
  
      _msgList.push_back( pframe );
  
    }
    input.close();

  } else {

  // create 20 random CAN frames for transmit tests
//  for( int i = 0; i < 20; i++ ) {
//    can_frame_t *pframe = new can_frame_t;
//    time_t t;
//
//    time(&t);
//    srand( (unsigned int)t );
//
//    pframe->can_id = rand() % CAN_EFF_MASK;
//    if ( pframe->can_id > CAN_SFF_MASK ) pframe->can_id |= CAN_EFF_FLAG;
//    pframe->can_dlc = rand() % 9;
//    for( unsigned int j = 0; j < pframe->can_dlc; j++ )  pframe->data[i] = rand() % 255;
//
//    _msgList.push_back( pframe );
//  }
  _msgList.push_back( new can_frame_t( 0x97fb, 8, 0x58, 0x93, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff ) );
  _msgList.push_back( new can_frame_t( 0x9f2e, 8, 0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a, 0x69, 0x78 ) );
}

//------------------------------------------------------------------------------

void CanTest::transmitTest( unsigned int dwMaxTimeInterval, unsigned int dwMaxLoop ) {
  
  double scale = ( dwMaxTimeInterval * 1000.0 ) / ( RAND_MAX + 1.0 );
  std::list<can_frame_t*>::iterator iter;

  // Set size of CAN socket send buffer to minimum
  int sndBufNew = 0;
  if( setsockopt( _socket, SOL_SOCKET, SO_SNDBUF, (void *)&sndBufNew, sizeof( sndBufNew ) ) < 0 ){
    perror( "Error while set socket option SNDBUF" );
    return;
  }

  time( &_start );
  if( 0 != dwMaxLoop ) {
    for ( unsigned int count = 0; count < dwMaxLoop; count++ ) {
      for ( iter = _msgList.begin(); iter != _msgList.end(); iter++ ) {
        
        // send the message
        int nbytes = write( _socket, (*iter), sizeof(can_frame_t) );
        if ( 0 > nbytes ) {
          std::stringstream errmsg;
          errmsg << "transmitest: write()\n"
                 << "Error " << errno << ": " << strerror( errno )
                 << std::endl;
          throw CanFailure( errmsg );
        }
        _counter++;
        // wait some time before the invocation
        if( dwMaxTimeInterval )
          usleep( (__useconds_t)( scale * rand() ) );
      }
    }

  } else {

    while( true ) {
      for ( iter = _msgList.begin(); iter != _msgList.end(); iter++ ) {
        
        // send the message
        int nbytes = write( _socket, (*iter), sizeof(can_frame_t) );
        if ( 0 > nbytes ) {
          std::stringstream errmsg;
          errmsg << "transmitest: write()\n"
                 << "Error " << errno << ": " << strerror( errno )
                 << std::endl;
          throw CanFailure( errmsg );
        }
        _counter++;

        // wait some time before the invocation
        if( dwMaxTimeInterval )
          usleep( (__useconds_t)( scale * rand() ) );
      }
    }
  }
  time( &_stop );
}

//------------------------------------------------------------------------------

void CanTest::receiveTest( unsigned int dwMaxLoop ) { 
  can_frame_t *pframe = new can_frame_t;
  std::list<can_frame_t*>::const_iterator it = _msgList.cbegin();
  time( &_start );

  if( 0 != dwMaxLoop ) {

    for ( unsigned int count = 0; count < dwMaxLoop; count++ ) {
      int nbytes = read( _socket, pframe, sizeof(can_frame_t) );
      if ( 0 > nbytes ) {
        std::stringstream errmsg;
        errmsg << "receivetest: read()\n"
               << "Error " << errno << ": " << strerror( errno )
               << std::endl;
        throw CanFailure( errmsg );
      } else  {
        if ( *pframe != *(*it) ) _mismatched++;
        it++;
        _counter++;
      }
    }  
       
  } else {     
  
    while( true ) {
      int nbytes = read( _socket, pframe, sizeof(can_frame_t) );
      if ( 0 > nbytes ) {
        std::stringstream errmsg;
        errmsg << "receivetest: read()\n"
               << "Error " << errno << ": " << strerror( errno )
               << std::endl;
        throw CanFailure( errmsg );
      } else  {
        if ( *pframe != *(*it) ) _mismatched++;
        it++;
        _counter++;
      }
    }

  }
  time( &_stop );
}
