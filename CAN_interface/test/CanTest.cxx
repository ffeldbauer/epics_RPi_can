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

// ANSI C++ headers
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <unistd.h>

// local headers
#include "CanTest.h"
#include "Exceptions.h"

//_____ D E F I N I T I O N S __________________________________________________
CanTest* CanTest::pinstance_ = NULL;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

CanTest::CanTest() {
}

//------------------------------------------------------------------------------

CanTest::CanTest( std::string& filename )
  : filename_(filename)
{ 
  ParseMessages();
}

//------------------------------------------------------------------------------

CanTest::CanTest( const CanTest& rother ) {
}

//------------------------------------------------------------------------------

CanTest::~CanTest() {
  msgList_.clear();
}

//------------------------------------------------------------------------------

bool CanTest::exists() {
  return ( NULL != pinstance_ );
}

//------------------------------------------------------------------------------

CanTest* CanTest::getInstance () {  
  if ( NULL == pinstance_ ) throw BadConfig("CanTest base class has not been created");
  return pinstance_; 
}

//------------------------------------------------------------------------------

void CanTest::create( std::string& file ) {
  if ( NULL != pinstance_ )
    std::cerr << "CanTest Singleton has already been created" << std::endl;
  else
    pinstance_ = new CanTest( file );
  return;
}

//------------------------------------------------------------------------------

void CanTest::ParseMessages() {
  
  std::ifstream input( filename_.c_str(), std::ios_base::in );
  std::string line;
  
  while( std::getline( input, line ) ) {
    can_frame_t frame;

    // handle comments in config file
    size_t comment;
    if ( ( comment = line.find('#') ) != std::string::npos )
      line.erase( comment ); // erase everthing after first '#' from string
    
    if ( 0 == line.length() ) continue; // string is empty, no need to parse it
      
    std::istringstream parse( line );
    char rtr;
    char ide;
    unsigned int dummydlc = 0;
    parse >> rtr >> ide >> std::hex >> frame.can_id >> dummydlc;
    switch( rtr ) {
    case 'm': break;
    case 'r': frame.can_id |= CAN_RTR_FLAG; break;
    default:
      std::cerr << "Invalid format: '"<< rtr << "' in '" << line << "'" << std::endl;
      continue;
    }
    switch( ide ) {
    case 's': break;
    case 'e': frame.can_id |= CAN_EFF_FLAG; break;
    default:
      std::cerr << "Invalid type: '"<< ide << "' in '" << line << "'" << std::endl;
      continue;
    }

    if ( 8 < dummydlc ) {
      std::cerr << "Invalid length: '"<< dummydlc << "' in '" << line << "'" << std::endl;
      continue;
    }
    frame.can_dlc = (uint8_t)(dummydlc & 8);
    for ( unsigned char i = 0; i < frame.can_dlc; i++ )
      parse >> std::hex >> frame.data[i];

    msgList_.push_back( frame );

  }
  input.close();
}

//------------------------------------------------------------------------------

void CanTest::transmitTest( unsigned int dwMaxTimeInterval, unsigned int dwMaxLoop ) {
  
  double scale = ( dwMaxTimeInterval * 1000.0 ) / ( RAND_MAX + 1.0 );
  std::list<can_frame_t>::iterator iter;
  
  if( 0 != dwMaxLoop ) {

    time( &start_ );
    for ( unsigned int count = 0; count < dwMaxLoop; count++ ) {
      for ( iter = msgList_.begin(); iter != msgList_.end(); iter++ ) {
        
        // send the message
        if( CAN_Write( can, &(*iter) ) ) {
          std::stringstream errmsg;
          errmsg << "transmitest: CAN_Write()\n"
                 << "Error " << errno << ": " << strerror( errno )
                 << std::endl;
          throw CanFailure( errmsg );
        }
        
        // wait some time before the invocation
        if( dwMaxTimeInterval )
          usleep( (__useconds_t)( scale * rand() ) );
      }
    }
    time( &stop_ );

  } else {

    while( true ) {
      for ( iter = msgList_.begin(); iter != msgList_.end(); iter++ ) {
        
        // send the message
        if( CAN_Write( can, &(*iter) ) ) {
          std::stringstream errmsg;
          errmsg << "transmitest: CAN_Write()\n"
                 << "Error " << errno << ": " << strerror( errno )
                 << std::endl;
          throw CanFailure( errmsg );
        }
        
        // wait some time before the invocation
        if( dwMaxTimeInterval )
          usleep( (__useconds_t)( scale * rand() ) );
      }
    }

  }
}

//------------------------------------------------------------------------------

void CanTest::receiveTest( ) {
  while( true ) {
    can_frame_t frame;
    if( CAN_Read( can, &frame ) )  {
      std::stringstream errmsg;
      errmsg << "receivetest: CAN_Read()\n"
             << "Error " << errno << ": " << strerror( errno )
             << std::endl;
      throw CanFailure( errmsg );
    } else  {
      msgList_.push_back( frame );
    }
  }  
}
