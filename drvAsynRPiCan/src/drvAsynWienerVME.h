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
//! @brief   Asyn driver for Wiener VME crate remote control
//!          using the RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

#ifndef __ASYN_WIENER_VME_H__
#define __ASYN_WIENER_VME_H__

//_____ I N C L U D E S _______________________________________________________
#include "asynPortDriver.h"

//_____ D E F I N I T I O N S __________________________________________________

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_WIENERVME_STATUS0_STRING         "WIENER_VME_STATUS0"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS1_STRING         "WIENER_VME_STATUS1"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS2_STRING         "WIENER_VME_STATUS2"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS3_STRING         "WIENER_VME_STATUS3"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS4_STRING         "WIENER_VME_STATUS4"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS5_STRING         "WIENER_VME_STATUS5"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS6_STRING         "WIENER_VME_STATUS6"         /* asynUInt32Digital,  r   */
#define P_WIENERVME_STATUS7_STRING         "WIENER_VME_STATUS7"         /* asynUInt32Digital,  r   */

#define P_WIENERVME_V0_STRING              "WIENER_VME_VMOM0"           /* asynInt32,          r   */
#define P_WIENERVME_V1_STRING              "WIENER_VME_VMOM1"           /* asynInt32,          r   */
#define P_WIENERVME_V2_STRING              "WIENER_VME_VMOM2"           /* asynInt32,          r   */
#define P_WIENERVME_V3_STRING              "WIENER_VME_VMOM3"           /* asynInt32,          r   */
#define P_WIENERVME_V4_STRING              "WIENER_VME_VMOM4"           /* asynInt32,          r   */
#define P_WIENERVME_V5_STRING              "WIENER_VME_VMOM5"           /* asynInt32,          r   */
#define P_WIENERVME_V6_STRING              "WIENER_VME_VMOM6"           /* asynInt32,          r   */
#define P_WIENERVME_V7_STRING              "WIENER_VME_VMOM7"           /* asynInt32,          r   */

#define P_WIENERVME_I0_STRING              "WIENER_VME_IMOM0"           /* asynInt32,          r   */
#define P_WIENERVME_I1_STRING              "WIENER_VME_IMOM1"           /* asynInt32,          r   */
#define P_WIENERVME_I2_STRING              "WIENER_VME_IMOM2"           /* asynInt32,          r   */
#define P_WIENERVME_I3_STRING              "WIENER_VME_IMOM3"           /* asynInt32,          r   */
#define P_WIENERVME_I4_STRING              "WIENER_VME_IMOM4"           /* asynInt32,          r   */
#define P_WIENERVME_I5_STRING              "WIENER_VME_IMOM5"           /* asynInt32,          r   */
#define P_WIENERVME_I6_STRING              "WIENER_VME_IMOM6"           /* asynInt32,          r   */
#define P_WIENERVME_I7_STRING              "WIENER_VME_IMOM7"           /* asynInt32,          r   */

#define P_WIENERVME_FANMIDDLE_STRING       "WIENER_VME_FANMIDDLE"       /* asynInt32,          r   */
#define P_WIENERVME_FANNOMINAL_STRING      "WIENER_VME_FANNOMINAL"      /* asynInt32,          r   */
#define P_WIENERVME_FAN1_STRING            "WIENER_VME_FAN1"            /* asynInt32,          r   */
#define P_WIENERVME_FAN2_STRING            "WIENER_VME_FAN2"            /* asynInt32,          r   */
#define P_WIENERVME_FAN3_STRING            "WIENER_VME_FAN3"            /* asynInt32,          r   */
#define P_WIENERVME_FAN4_STRING            "WIENER_VME_FAN4"            /* asynInt32,          r   */
#define P_WIENERVME_FAN5_STRING            "WIENER_VME_FAN5"            /* asynInt32,          r   */
#define P_WIENERVME_FAN6_STRING            "WIENER_VME_FAN6"            /* asynInt32,          r   */

#define P_WIENERVME_TEMP1_STRING           "WIENER_VME_TEMP1"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP2_STRING           "WIENER_VME_TEMP2"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP3_STRING           "WIENER_VME_TEMP3"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP4_STRING           "WIENER_VME_TEMP4"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP5_STRING           "WIENER_VME_TEMP5"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP6_STRING           "WIENER_VME_TEMP6"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP7_STRING           "WIENER_VME_TEMP7"           /* asynInt32,          r   */
#define P_WIENERVME_TEMP8_STRING           "WIENER_VME_TEMP8"           /* asynInt32,          r   */

#define P_WIENERVME_SWITCH_STRING          "WIENER_VME_SWITCHONOFF"     /* asynInt32,          r/w */
#define P_WIENERVME_SYSRESET_STRING        "WIENER_VME_SYSRESET"        /* asynInt32,          w   */
#define P_WIENERVME_CHANGEFAN_STRING       "WIENER_VME_FANSPEED"        /* asynInt32,          w   */

class drvAsynWienerVme : public asynPortDriver {
 public:
  drvAsynWienerVme( const char *portName, const char *RPiCanPort, const int crate_id );

  /* These are the methods that we override from asynPortDriver */
  virtual asynStatus readInt32( asynUser *pasynUser, epicsInt32 *value );
  virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value );
  virtual asynStatus readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask );
  virtual asynStatus writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask );

 protected:

  asynStatus readValues( const epicsUInt32 cmd, epicsUInt32 mask, asynUser *pasynUser );

  /** Values used for pasynUser->reason, and indexes into the parameter library. */
  int P_Status0;
#define FIRST_WIENERVME_COMMAND P_Status0
  int P_Status1;
  int P_Status2;
  int P_Status3;
  int P_Status4;
  int P_Status5;
  int P_Status6;
  int P_Status7;
  int P_Vmom0;
  int P_Vmom1;
  int P_Vmom2;
  int P_Vmom3;
  int P_Vmom4;
  int P_Vmom5;
  int P_Vmom6;
  int P_Vmom7;
  int P_Imom0;
  int P_Imom1;
  int P_Imom2;
  int P_Imom3;
  int P_Imom4;
  int P_Imom5;
  int P_Imom6;
  int P_Imom7;
  int P_FanMiddle;
  int P_FanNominal;
  int P_Fan1;
  int P_Fan2;
  int P_Fan3;
  int P_Fan4;
  int P_Fan5;
  int P_Fan6;
  int P_Temp1;
  int P_Temp2;
  int P_Temp3;
  int P_Temp4;
  int P_Temp5;
  int P_Temp6;
  int P_Temp7;
  int P_Temp8;
  int P_Switch;
  int P_Sysreset;
  int P_FanSpeed;
#define LAST_WIENERVME_COMMAND P_FanSpeed

 private:
  char           *deviceName_;
  epicsUInt32     crate_id_;
  asynUser       *pAsynUserGenericPointer_;
};

#define NUM_WIENERVME_PARAMS (&LAST_WIENERVME_COMMAND - &FIRST_WIENERVME_COMMAND + 1)

#endif

//******************************************************************************
//! EOF
//******************************************************************************
