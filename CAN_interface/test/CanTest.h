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

#ifndef _CAN_TEST_H_
#define _CAN_TEST_H_

//_____ I N C L U D E S _______________________________________________________

// ANSI C++ headers
#include <ctime>
#include <linux/can.h>
#include <list>
#include <string>

//_____ D E F I N I T I O N S __________________________________________________

typedef struct can_frame can_frame_t;

class CanTest {

 public:
  static bool exists();
  static CanTest* getInstance();
  static void create( std::string&, std::string& );

  void transmitTest( unsigned int, unsigned int );
  void receiveTest( unsigned int );
  void printDiag();
  void setBitrate( unsigned int );
  bool CanOpen();
  void CanClose();

 private:
  CanTest();
  CanTest( std::string&, std::string& );
  CanTest( const CanTest& );
  ~CanTest();

  void ParseMessages();

  int                      _socket;
  std::string              _filename;
  std::string              _devName;
  std::list<can_frame_t*>  _msgList;
  time_t                   _start;
  time_t                   _stop;
  static CanTest*          _pinstance;
  int                      _counter;
  unsigned int             _bitrate;
  bool                     _open;
};

#endif
