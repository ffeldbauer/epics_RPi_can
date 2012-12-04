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
//! @brief   Asyn driver for ISEG EHS/EDS high voltage modules using the RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <linux/can.h>

/* EPICS includes */
#include <epicsEvent.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsTimer.h>
#include <epicsTypes.h>
#include <iocsh.h>

/* ASYN includes */
#include "asynDriver.h"
#include "asynGenericPointerSyncIO.h"
#include "asynStandardInterfaces.h"

#include "drvAsynIsegHv.h"

//_____ D E F I N I T I O N S __________________________________________________

typedef struct can_frame can_frame_t;

// convert 32-bit float (IEEE 754) to 4*8 bit unsigned int
typedef union{
  epicsFloat32 fval;
  //  epicsUInt32  ival;
  epicsUInt8   val[4];
} can_float_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynIsegHVDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!          Perform one of the following tasks: Switch all HV channels off w/o ramp (emergency off)
//!                                              Clear the event status of the module
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [in]  value      Value to write
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( 0 != addr ) return asynSuccess;
  
  /* Set the parameter in the parameter library. */
  status = (asynStatus) setIntegerParam( addr, function, value );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks( addr, addr );
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, deviceName_, functionName, function, value );

  can_frame_t *pframe = new can_frame_t;
  if ( function == P_ClearEvtStatus ) {
    pframe->can_id  = can_id_;
    pframe->can_dlc = 4;
    pframe->data[0] = 0x10;
    pframe->data[1] = 0x01;
    pframe->data[2] = 0x18;
    pframe->data[3] = 0x40;
  } else if ( function == P_EmergencyOff ) {
    pframe->can_id  = can_id_;
    pframe->can_dlc = 6;
    pframe->data[0] = 0x22;
    pframe->data[1] = 0x01;
    pframe->data[2] = 0x00;
    pframe->data[3] = 0x00;
    pframe->data[4] = 0xff;
    pframe->data[5] = 0xff;
  }
  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, Could not send can frame.\033[0m", 
                   driverName, deviceName_, functionName, status, function );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!          Read module (event) status or channel (event) status
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [out] value      Address of the value to read
//!          [in]  mask       Mask value to use when reading the value.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char *functionName = "readUInt32Digital";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;

  if ( function == P_SwitchOnOff ) {
    status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
    return status;
  }
  if ( ( function == P_Mod_status || function == P_Mod_Event_status ) && ( addr != 0 ) ) {
    status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
    return status;
  }
  can_frame_t *pframe = new can_frame_t;
  unsigned int offset;
  pframe->can_id = can_id_ | 1;
  if ( function == P_Chan_status ) {
    offset = 3;
    pframe->can_dlc = 3;
    pframe->data[0] = 0x40;
    pframe->data[1] = 0x00;
    pframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Chan_Event_status ) {
    offset = 3;
    pframe->can_dlc = 3;
    pframe->data[0] = 0x40;
    pframe->data[1] = 0x02;
    pframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Mod_status ) {
    offset = 2;
    pframe->can_dlc = 2;
    pframe->data[0] = 0x10;
    pframe->data[1] = 0x00;
  } else if ( function == P_Mod_Event_status ) {
    offset = 2;
    pframe->can_dlc = 2;
    pframe->data[0] = 0x10;
    pframe->data[1] = 0x02;
  } else {
    return asynError;
  }

  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, No reply from device within %f s\033[0m", 
                   driverName, deviceName_, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( pframe->can_id != can_id_ || pframe->can_dlc != ( offset + 2 ) ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d where %08x %d was expected\033[0m", 
                   driverName, deviceName_, functionName, function, pframe->can_id, pframe->can_dlc,
                   can_id_, offset + 2 );
    return asynError;
  }
  
  // update the parameter with the received value
  epicsUInt32 myValue  = ( pframe->data[offset] << 8 ) | ( pframe->data[offset + 1]);
  status = setUIntDigitalParam( addr, function, myValue, mask );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, myValue, mask );
  
  // read back parameter
  status = (asynStatus) getUIntDigitalParam( addr, function, value, mask );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, *value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%u, mask=%u\n", 
               driverName, deviceName_, functionName, function, *value, mask );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynUInt32Digital->write().
//!          Switch all HV channels on / off
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [in]  value      Value to write
//!          [in]  mask       Mask value to use when reading the value.
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char* functionName = "writeUInt32Digital";

  if ( function != P_SwitchOnOff ) return asynSuccess;

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( 0 != addr ) return asynSuccess;
  
  /* Set the parameter in the parameter library. */
  status = (asynStatus) setUIntDigitalParam( addr, function, value, mask );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks( addr, addr );
  
  if ( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s: status=%d, function=%d, value=%u, mask=%u\033[0m", 
                   driverName, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, functionName, function, value, mask );

  can_frame_t *pframe = new can_frame_t;
  epicsUInt8 dummy = ( value ? 0xff : 0x00 );
  pframe->can_id  = can_id_;
  pframe->can_dlc = 6;
  pframe->data[0] = 0x22;
  pframe->data[1] = 0x00;
  pframe->data[2] = 0x00;
  pframe->data[3] = 0x00;
  pframe->data[4] = dummy;
  pframe->data[5] = dummy;

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, pasynUser->timeout );
  if ( status ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Could not send can frame.\033[0m", 
                   driverName, deviceName_, functionName, function );
    return asynError;
  }
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynFloat64->read().
//!          Read board temperature, measured voltages/currents or setpoints for
//!          voltages and currents
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [out] value      Address of value to read
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::readFloat64( asynUser *pasynUser, epicsFloat64 *value ) {
  int function = pasynUser->reason;
  int addr = 0;
  asynStatus status = asynSuccess;
  const char *functionName = "readFloat64";
  
  status = getAddress( pasynUser, &addr ); if( status != asynSuccess ) return status;

  if ( ( function == P_Temperature || function == P_RampSpeed ) && ( addr != 0 ) ) {
    status = (asynStatus) getDoubleParam( addr, function, value );
    return status;
  }

  can_frame_t *pwriteframe = new can_frame_t;
  can_frame_t *preadframe = new can_frame_t;
  unsigned int offset;
  epicsFloat64 exp = 1;
  pwriteframe->can_id = can_id_ | 1;

  if ( function == P_Chan_Vset ) {
    offset = 3;
    pwriteframe->can_dlc = 3;
    pwriteframe->data[0] = 0x41;
    pwriteframe->data[1] = 0x00;
    pwriteframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Chan_Iset ) {
    offset = 3;
    exp = 1.e6;
    pwriteframe->can_dlc = 3;
    pwriteframe->data[0] = 0x41;
    pwriteframe->data[1] = 0x01;
    pwriteframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Chan_Vmom ) {
    offset = 3;
    pwriteframe->can_dlc = 3;
    pwriteframe->data[0] = 0x41;
    pwriteframe->data[1] = 0x02;
    pwriteframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Chan_Imom ) {
    offset = 3;
    exp = 1.e6;
    pwriteframe->can_dlc = 3;
    pwriteframe->data[0] = 0x41;
    pwriteframe->data[1] = 0x03;
    pwriteframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Temperature ) {
    offset = 2;
    pwriteframe->can_dlc = 2;
    pwriteframe->data[0] = 0x11;
    pwriteframe->data[1] = 0x06;
  } else if ( function == P_RampSpeed ) {
    offset = 2;
    pwriteframe->can_dlc = 2;
    pwriteframe->data[0] = 0x11;
    pwriteframe->data[1] = 0x00;
  } else {
    return asynError;
  }

  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pwriteframe, preadframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, No reply from device within %f s\033[0m", 
                   driverName, deviceName_, functionName, status, function, pasynUser->timeout );
    return asynTimeout;
  }
  if ( ( preadframe->can_id  != can_id_ ) ||
       ( preadframe->can_dlc != offset + 4  ) ||
       ( preadframe->data[0] != pwriteframe->data[0] ) ||
       ( preadframe->data[1] != pwriteframe->data[1] ) 
       ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: function=%d, Mismatch in reply.\nGot %08x %d %02x %02x where %08x %d %02x %02x was expected\033[0m", 
                   driverName, deviceName_, functionName, function,
                   preadframe->can_id, preadframe->can_dlc, preadframe->data[0], preadframe->data[1],
                   can_id_, offset + 4, pwriteframe->data[0], pwriteframe->data[1] );
    return asynError;
  }
  
  // update the parameter with the received value
  can_float_t myValue;
  //  myValue.ival = *( epicsUInt32*)&preadframe->data[ offset ];
  myValue.val[3] = preadframe->data[ offset ];
  myValue.val[2] = preadframe->data[ offset + 1 ];
  myValue.val[1] = preadframe->data[ offset + 2 ];
  myValue.val[0] = preadframe->data[ offset + 3 ];

  myValue.fval *= exp;
  status = setDoubleParam( addr, function, myValue.fval );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%f\033[0m", 
                   driverName, deviceName_, functionName, status, function, myValue.fval );

  // read back parameter
  status = (asynStatus) getDoubleParam( addr, function, value );
  if ( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%f\033[0m", 
                   driverName, deviceName_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%f\n", 
               driverName, deviceName_, functionName, function, *value );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynFloat64->write().
//!          Change setpoint of rampspeed, voltage or currentlimit
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [in]  value      Value to write
//------------------------------------------------------------------------------
asynStatus drvAsynIsegHv::writeFloat64( asynUser *pasynUser, epicsFloat64 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  int addr = 0;
  const char *functionName = "writeFloat64";

  status = getAddress( pasynUser, &addr ); if ( status != asynSuccess ) return status;
  if ( function == P_RampSpeed && addr != 0 ) return asynSuccess;

  can_frame_t *pframe = new can_frame_t;
  unsigned int offset;
  epicsFloat64 exp = 1;
  pframe->can_id = can_id_;

  if ( function == P_Chan_Vset ) {
    offset = 3;
    pframe->can_dlc = 7;
    pframe->data[0] = 0x41;
    pframe->data[1] = 0x00;
    pframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_Chan_Iset ) {
    offset = 3;
    exp = 1.e-6;
    pframe->can_dlc = 7;
    pframe->data[0] = 0x41;
    pframe->data[1] = 0x01;
    pframe->data[2] = (epicsUInt8)( addr & 0xff );
  } else if ( function == P_RampSpeed ) {
    offset = 2;
    pframe->can_dlc = 6;
    pframe->data[0] = 0x11;
    pframe->data[1] = 0x00;
  } else if ( function == P_Chan_Vmom ||
              function == P_Chan_Imom ||
              function == P_Temperature ) {
    return asynSuccess;
  } else {
    return asynError;
  }

  can_float_t myValue;
  myValue.fval = value * exp;
  //  *(epicsUInt32*)&pframe->data[ offset ] = (epicsUInt32)myValue.ival;
  pframe->data[ offset ]     = myValue.val[3];
  pframe->data[ offset + 1 ] = myValue.val[2];
  pframe->data[ offset + 2 ] = myValue.val[1];
  pframe->data[ offset + 3 ] = myValue.val[0];

  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, Could not send can frame\033[0m", 
                   driverName, deviceName_, functionName, status, function );
    return asynError;
  }
  
  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam( addr, function, value );
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks( addr, addr );
  if ( status ) 
    asynPrint( pasynUser, ASYN_TRACE_ERROR, 
               "%s:%s:%s: error, status=%d function=%d, value=%f\n", 
               driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%f\n", 
               driverName, deviceName_, functionName, function, value );
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynIsegHv class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//!          [in]  RPiCanPort  The name of the interface 
//!          [in]  crate_id    The id of the crate
//!          [in]  module_id   The id of the module inside the crate
//------------------------------------------------------------------------------
drvAsynIsegHv::drvAsynIsegHv( const char *portName, const char *RPiCanPort,
                              const int crate_id, const int module_id ) 
  : asynPortDriver( portName, 
                    16, /* maxAddr */ 
                    NUM_ISEGHV_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask,  /* Interrupt mask */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynIsegHv";
  asynStatus status = asynSuccess;

  deviceName_  = epicsStrDup( portName );
  can_id_      = ( 1 << 9 ) | ( crate_id << 6 ) | ( module_id << 3 );

  // Create parameters
  createParam( P_ISEGHV_CHANSTATUS_STRING,    asynParamUInt32Digital, &P_Chan_status );
  createParam( P_ISEGHV_CHANEVTSTATUS_STRING, asynParamUInt32Digital, &P_Chan_Event_status );
  createParam( P_ISEGHV_VMOM_STRING,          asynParamFloat64,       &P_Chan_Vmom );
  createParam( P_ISEGHV_IMOM_STRING,          asynParamFloat64,       &P_Chan_Imom );
  createParam( P_ISEGHV_VSET_STRING,          asynParamFloat64,       &P_Chan_Vset );
  createParam( P_ISEGHV_ISET_STRING,          asynParamFloat64,       &P_Chan_Iset );
  // These parameters are only used once per module:
  createParam( P_ISEGHV_MODSTATUS_STRING,      asynParamUInt32Digital, &P_Mod_status );
  createParam( P_ISEGHV_MODEVTSTATUS_STRING,   asynParamUInt32Digital, &P_Mod_Event_status );
  createParam( P_ISEGHV_TEMPERATURE_STRING,    asynParamFloat64,       &P_Temperature );
  createParam( P_ISEGHV_RAMPSPEED_STRING,      asynParamFloat64,       &P_RampSpeed );
  createParam( P_ISEGHV_CLEAREVTSTATUS_STRING, asynParamInt32,         &P_ClearEvtStatus );
  createParam( P_ISEGHV_EMO_STRING,            asynParamInt32,         &P_EmergencyOff );
  createParam( P_ISEGHV_SWITCH_STRING,         asynParamUInt32Digital, &P_SwitchOnOff );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( RPiCanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    printf( "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
            driverName, deviceName_, functionName, RPiCanPort );
    return;
  }
  
  can_frame_t *pframe = new can_frame_t;
  // Send "status connected" message
  pframe->can_id  = can_id_;
  pframe->can_dlc = 2;
  pframe->data[0] = 0xd8;
  pframe->data[1] = 0x01;
  status = pasynGenericPointerSyncIO->write( pAsynUserGenericPointer_, pframe, 1. );

  // Get inital value for Switch-Parameter
  pframe->can_id  = can_id_ | 1;
  pframe->can_dlc = 2;
  pframe->data[0] = 0x22;
  pframe->data[1] = 0x00;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s: No reply from device within 1 s! \033[0m \n",
             driverName, deviceName_, functionName, P_ISEGHV_SWITCH_STRING );
    return;
  }
  if ( pframe->data[4] & 0xff || pframe->data[5] & 0xff)
    setUIntDigitalParam( 0, P_SwitchOnOff, 1, 1 );
  else
    setUIntDigitalParam( 0, P_SwitchOnOff, 0, 1 );

  // get initial values for vset/iset parameters
  for ( unsigned int i = 0; i < 16; i++ ) {
    pframe->can_id  = can_id_ | 1;
    pframe->can_dlc = 3;
    pframe->data[0] = 0x41;
    pframe->data[1] = 0x00;
    pframe->data[2] = i;
    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
    if ( status != asynSuccess ) {
      fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s %d: No reply from device within 1 s! \033[0m \n",
               driverName, deviceName_, functionName, P_ISEGHV_VSET_STRING, i );
      return;
    }
    can_float_t myValue;
    //    myValue.ival = *( epicsUInt32*)&pframe->data[4];
    myValue.val[3] = pframe->data[4];
    myValue.val[2] = pframe->data[5];
    myValue.val[1] = pframe->data[6];
    myValue.val[0] = pframe->data[7];
    setDoubleParam( i, P_Chan_Vset, myValue.fval );

    pframe->can_id  = can_id_ | 1;
    pframe->can_dlc = 3;
    pframe->data[0] = 0x41;
    pframe->data[1] = 0x01;
    pframe->data[2] = i;
    status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
    if ( status != asynSuccess ) {
      fprintf( stderr, "\033[31;1m %s:%s:%s: Init %s %d: No reply from device within 1 s! \033[0m \n",
               driverName, deviceName_, functionName, P_ISEGHV_ISET_STRING, i );
      return;
    }
    //    myValue.ival = *( epicsUInt32*)&pframe->data[4];
    myValue.val[3] = pframe->data[4];
    myValue.val[2] = pframe->data[5];
    myValue.val[1] = pframe->data[6];
    myValue.val[0] = pframe->data[7];
    myValue.fval *= 1.e6;
    setDoubleParam( i, P_Chan_Iset, myValue.fval );
  }
}

//******************************************************************************
//! EOF
//******************************************************************************
