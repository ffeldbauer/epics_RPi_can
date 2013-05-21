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

#ifndef _CAN_TEST_H_
#define _CAN_TEST_H_

//_____ I N C L U D E S _______________________________________________________

// ANSI C++ headers
#include <ctime>
#include <linux/can.h>
#include <list>
#include <string>

#include "libpican.h"

//_____ D E F I N I T I O N S __________________________________________________

typedef struct can_frame can_frame_t;

extern CANBUS* can;

class CanTest {

 public:
  static bool exists();
  static CanTest* getInstance();
  static void create( std::string& );

  void transmitTest( unsigned int, unsigned int );
  void receiveTest();

  inline double runtime() { return difftime( stop_, start_ ); }


 private:
  CanTest();
  CanTest( std::string& );
  CanTest( const CanTest& );
  ~CanTest();

  void ParseMessages();

  std::string filename_;
  std::list<can_frame_t> msgList_;
  time_t start_;
  time_t stop_;
  static CanTest* pinstance_;

};

#endif
