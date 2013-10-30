//******************************************************************************
// Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//                    - Helmholtz-Institut Mainz
//
// This file is part of dallas1wire
//
// dallas1wire is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// dallas1wire is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// brief   Read Temperature/Humidity from Dallas 1-wire Sensors
//
// version 2.0.0; Feb. 28, 2013
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

// ANSI C includes 
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include "dallas1wire.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Read contents of file
//!
//! @param   [in]  pinfo  Address of device support info structure
//! @param   [out] value  Address of value
//!
//! @return  0:  Evertyhing OK
//!          -1: Cannot open file
//!          -2: CRC did not match
//!          -3: Parsing failed
//------------------------------------------------------------------------------
long readValue( d1w_info_t *pinfo, double* value ) {

  // open the file and set position indicator at end
  std::ifstream myRead( pinfo->filename, std::ios_base::in );
  std::string tmp;
  std::string crcMatch;
  int dummy = 0;
  int num = 0;

  // Check if file is really opened
  if( !myRead.is_open() ) return -1;
  
  if ( 0x10 == pinfo->fcode || 0x28 == pinfo->fcode ) {
    // DS18S20 Temperatur sensor
    std::getline( myRead, tmp );
    std::istringstream parse( tmp );
    do {
      parse >> crcMatch;
    } while (parse);
    if ( crcMatch.compare( "YES" ) ) {
      myRead.close();
      return -2;
    }
    
    std::getline( myRead, tmp );
    num = sscanf( tmp.c_str(), "%*x %*x %*x %*x %*x %*x %*x %*x %*x t=%u", &dummy );
    if ( 1 != num ) return -3;
    *value = dummy / 1000.;

  } else {
    // Humidity Sensor
    std::getline( myRead, tmp );
    std::getline( myRead, tmp );
    std::getline( myRead, tmp );
    std::getline( myRead, tmp );
    std::istringstream parse( tmp );
    std::string skip;
    parse >> skip >> skip >> dummy;
    *value = dummy;
  }
  myRead.close();
  
  return 0;
}
