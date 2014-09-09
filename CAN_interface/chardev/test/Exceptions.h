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

#ifndef EXCEPTION_H
#define EXCEPTION_H

//_____ I N C L U D E S ________________________________________________________
#include <exception>
#include <sstream>
#include <string>

//_____ D E F I N I T I O N S __________________________________________________

//------------------------------------------------------------------------------
//! @class   Exception
//!
//! @brief   Abstract base class of all exception types within the EnvDB
//------------------------------------------------------------------------------
class Exception : public std::exception {
 public:
  Exception(const Exception& e) throw() :
  std::exception(e),
    what_(e.what_)
      {}

  Exception& operator=(const Exception& rhs) throw() {
    what_ = rhs.what_;
    return *this;
  }
  ~Exception() throw() { }
  virtual const char* what() const throw() {
    return what_.c_str();
  }

 protected:
  Exception(const char* w = "") throw() :
  what_(w)
  {}

  Exception(const std::string& w) throw() :
  what_(w)
  {}

  std::string what_;
};

//------------------------------------------------------------------------------

class BadConfig : public Exception {
 public:
  BadConfig ( const std::string& error ) :
  Exception(error)
  {}
  BadConfig ( const char *error ) :
  Exception(error)
  {}
  ~BadConfig () throw() {}
};

//------------------------------------------------------------------------------

class CanFailure : public Exception {
 public:
  CanFailure ( const std::string& error ) :
  Exception(error)
  {}
  CanFailure ( const std::stringstream& error ) :
  Exception( error.str() )
  {}
  CanFailure ( const char *error ) :
  Exception(error)
  {}
  ~CanFailure () throw() {}
};

#endif
