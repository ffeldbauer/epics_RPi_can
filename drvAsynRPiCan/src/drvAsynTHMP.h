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
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   Asyn driver for THMP using the RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

#ifndef __ASYN_THMP_H__
#define __ASYN_THMP_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_THMP_ADCBUFFER_STRING          "THMP_RAWVALUE"          /* asynInt32,          r   */
#define P_THMP_IOBUFFER_STRING           "THMP_IOBOARD"           /* asynUInt32Digital,  r/w */
#define P_THMP_SERIALS_STRING            "THMP_SERIALS"           /* asynUInt32Digital,  r   */
#define P_THMP_FIRMWARE_STRING           "THMP_FIRMWARE"          /* asynUInt32Digital,  r   */
#define P_THMP_ERROR_STRING              "THMP_ERROR"             /* asynUInt32Digital,  r   */

#define P_THMP_CONFIGIO_STRING           "THMP_CONFIG_IO"         /* asynInt32,          r/w */
#define P_THMP_TRG_ADCBUFFER_STRING      "THMP_TRG_ADC"           /* asynInt32,          w   */
#define P_THMP_TRG_IOBUFFER_STRING       "THMP_TRG_IO"            /* asynInt32,          w   */
#define P_THMP_TRG_SERIALS_STRING        "THMP_TRG_SERIALS"       /* asynInt32,          w   */

class drvAsynTHMP : public asynPortDriver {
 public:
  drvAsynTHMP( const char *portName, const char *RPiCanPort, const int can_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );

  void readPoller( void );

 protected:
  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_RawValue;
#define FIRST_THMP_COMMAND P_RawValue
  int P_IoBoard;
  int P_Serials;
  int P_ConfigIO;
  int P_Firmware;
  int P_Error;
  int P_Trg_ADC;
  int P_Trg_IO;
  int P_Trg_Serials;
#define LAST_THMP_COMMAND P_Trg_Serials

 private:
  epicsEventId    eventId_;
  char           *deviceName_;
  epicsUInt32     can_id_;
  asynUser       *pAsynUserGenericPointer_;
};

#define NUM_THMP_PARAMS (&LAST_THMP_COMMAND - &FIRST_THMP_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
