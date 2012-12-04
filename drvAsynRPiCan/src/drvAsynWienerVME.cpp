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

#include "drvAsynWienerVME.h"

//_____ D E F I N I T I O N S __________________________________________________

typedef struct can_frame can_frame_t;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynWienerVMEDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Ask the crate for given values and update the parameter list
//!
//! @param   [in]  can_id     CAN id for this message
//!          [in]  funcitons  Address of array containing all functions related to this command
//!          [in]  num        Number of elements related to this command
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::readValues( const epicsUInt32 cmd, epicsUInt32 mask, asynUser *pasynUser ){
  asynStatus status = asynSuccess;
  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = ( cmd << 7 ) | crate_id_ | CAN_RTR_FLAG;
  pframe->can_dlc = 8;
  
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, pasynUser->timeout );
  if ( status ){
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:readValues: No reply from device within %f s.\033[0m", 
                   driverName, deviceName_, pasynUser->timeout );
    return asynTimeout;
  }
  if ( pframe->can_id != ( ( cmd << 7 ) | crate_id_ ) || pframe->can_dlc != 8 ) {
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:readValues: cmd=%d, Mismatch in reply.\n Got '%08x %d...' where '%x08 8...' was expected.\033[0m", 
                   driverName, deviceName_, cmd, pframe->can_id, pframe->can_dlc, ( cmd << 7 )|crate_id_  );
    return asynError;
  }
  
  switch ( cmd ) {
  case 0:
    status = setUIntDigitalParam( P_Status0, pframe->data[0], mask );
    status = setUIntDigitalParam( P_Status1, pframe->data[1], mask );
    status = setUIntDigitalParam( P_Status2, pframe->data[2], mask );
    status = setUIntDigitalParam( P_Status3, pframe->data[3], mask );
    status = setUIntDigitalParam( P_Status4, pframe->data[4], mask );
    status = setUIntDigitalParam( P_Status5, pframe->data[5], mask );
    status = setUIntDigitalParam( P_Status6, pframe->data[6], mask );
    status = setUIntDigitalParam( P_Status7, pframe->data[7], mask );
    break;

  case 2:
    status = setIntegerParam( P_Vmom0, ( pframe->data[1] << 8 ) | pframe->data[0] );
    status = setIntegerParam( P_Imom0, ( pframe->data[3] << 8 ) | pframe->data[2] );
    status = setIntegerParam( P_Vmom4, ( pframe->data[5] << 8 ) | pframe->data[4] );
    status = setIntegerParam( P_Imom4, ( pframe->data[7] << 8 ) | pframe->data[6] );
    break;

  case 3:
    status = setIntegerParam( P_Vmom1, ( pframe->data[1] << 8 ) | pframe->data[0] );
    status = setIntegerParam( P_Imom1, ( pframe->data[3] << 8 ) | pframe->data[2] );
    status = setIntegerParam( P_Vmom5, ( pframe->data[5] << 8 ) | pframe->data[4] );
    status = setIntegerParam( P_Imom5, ( pframe->data[7] << 8 ) | pframe->data[6] );
    break;

  case 4:
    status = setIntegerParam( P_Vmom2, ( pframe->data[1] << 8 ) | pframe->data[0] );
    status = setIntegerParam( P_Imom2, ( pframe->data[3] << 8 ) | pframe->data[2] );
    status = setIntegerParam( P_Vmom6, ( pframe->data[5] << 8 ) | pframe->data[4] );
    status = setIntegerParam( P_Imom6, ( pframe->data[7] << 8 ) | pframe->data[6] );
    break;

  case 5:
    status = setIntegerParam( P_Vmom3, ( pframe->data[1] << 8 ) | pframe->data[0] );
    status = setIntegerParam( P_Imom3, ( pframe->data[3] << 8 ) | pframe->data[2] );
    status = setIntegerParam( P_Vmom7, ( pframe->data[5] << 8 ) | pframe->data[4] );
    status = setIntegerParam( P_Imom7, ( pframe->data[7] << 8 ) | pframe->data[6] );
    break;

  case 6:
    status = setIntegerParam( P_FanMiddle, pframe->data[0] );
    status = setIntegerParam( P_FanNominal, pframe->data[1] );
    status = setIntegerParam( P_Fan1, pframe->data[2] );
    status = setIntegerParam( P_Fan2, pframe->data[3] );
    status = setIntegerParam( P_Fan3, pframe->data[4] );
    status = setIntegerParam( P_Fan4, pframe->data[5] );
    status = setIntegerParam( P_Fan5, pframe->data[6] );
    status = setIntegerParam( P_Fan6, pframe->data[7] );
    break;

  case 7:
    status = setIntegerParam( P_Temp1, pframe->data[0] );
    status = setIntegerParam( P_Temp2, pframe->data[1] );
    status = setIntegerParam( P_Temp3, pframe->data[2] );
    status = setIntegerParam( P_Temp4, pframe->data[3] );
    status = setIntegerParam( P_Temp5, pframe->data[4] );
    status = setIntegerParam( P_Temp6, pframe->data[5] );
    status = setIntegerParam( P_Temp7, pframe->data[6] );
    status = setIntegerParam( P_Temp8, pframe->data[7] );
    break;
    
  default:
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:readValues: Invalid function.\033[0m", 
                   driverName, deviceName_ );
    
    return asynError;
    
  }  
  if ( status ) return status;

  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->read().
//!          
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [out] value      Address of the value to read
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::readInt32( asynUser *pasynUser, epicsInt32 *value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "readInt32";

  if ( function == P_Vmom0 )            status = readValues ( 2, 0, pasynUser );
  else if ( function == P_Vmom1 )       status = readValues ( 3, 0, pasynUser );
  else if ( function == P_Vmom2 )       status = readValues ( 4, 0, pasynUser );
  else if ( function == P_Vmom3 )       status = readValues ( 5, 0, pasynUser );
  else if ( function == P_FanMiddle )   status = readValues ( 6, 0, pasynUser );
  else if ( function == P_Temp1 )       status = readValues ( 7, 0, pasynUser );
  
  if ( status ) return status;

  // read back parameter
  status = (asynStatus) getIntegerParam( function, value );
  if ( status )
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, *value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%u\n", 
               driverName, deviceName_, functionName, function, *value );
  
  return status;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynInt32->write().
//!          Change nominal fan speed ( all other Int32Params: do nothing! )
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [in]  value      Value to write
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::writeInt32( asynUser *pasynUser, epicsInt32 value ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  if ( ( function != P_FanSpeed ) && ( function != P_Sysreset ) ) return asynSuccess;
  
  /* Set the parameter in the parameter library. */
  status = (asynStatus) setIntegerParam( function, value );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d\033[0m", 
                   driverName, deviceName_, functionName, status, function, value );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d\n", 
               driverName, deviceName_, functionName, function, value );
  
  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = ( 1 << 7 ) | crate_id_;
  if ( function == P_FanSpeed ) {
    pframe->can_dlc = 2;
    pframe->data[0] = 0xc0;
    pframe->data[1] = (epicsUInt8)( value & 0xff );
  } else if ( function == P_Sysreset ) {
    pframe->can_dlc = 1;
    pframe->data[0] = 0x44;
  }

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
//! @brief   Called when asyn clients call pasynUInt32Digital->read().
//!          Read module (event) status or channel (event) status
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [out] value      Address of the value to read
//!          [in]  mask       Mask value to use when reading the value.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::readUInt32Digital( asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask ) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "readUInt32Digital";

  if ( function == P_Status0 ) {
    status = readValues( 0, mask, pasynUser );
    if ( status ) return status;
  }

  // read back parameter
  status = (asynStatus) getUIntDigitalParam( function, value, mask );
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
//!          Switch crate on/off
//!
//! @param   [in]  pasynUser  pasynUser structure that encodes the reason and address
//!          [in]  value      Value to write
//!          [in]  mask       Mask value to use when reading the value.
//------------------------------------------------------------------------------
asynStatus drvAsynWienerVme::writeUInt32Digital( asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask ){
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char* functionName = "writeInt32";
  
  if ( function != P_Switch ) return asynSuccess;

  /* Set the parameter in the parameter library. */
  status = (asynStatus) setUIntDigitalParam( function, value, mask );
  
  /* Do callbacks so higher layers see any changes */
  status = (asynStatus) callParamCallbacks();
  
  if( status ) 
    epicsSnprintf( pasynUser->errorMessage, pasynUser->errorMessageSize, 
                   "\033[31;1m%s:%s:%s: status=%d, function=%d, value=%d, mask=%u\033[0m", 
                   driverName, deviceName_, functionName, status, function, value, mask );
  else        
    asynPrint( pasynUser, ASYN_TRACEIO_DRIVER, 
               "%s:%s:%s: function=%d, value=%d, mask=%u\n", 
               driverName, deviceName_, functionName, function, value, mask );

  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = ( 1 << 7 ) | crate_id_;
  pframe->can_dlc = 1;
  pframe->data[0] = value ? 0x67 : 0x65;

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
//! @brief   Constructor for the drvAsynWienerVme class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName    The name of the asyn port driver to be created.
//!          [in]  RPiCanPort  The name of the interface 
//!          [in]  crate_id    The id of the crate
//------------------------------------------------------------------------------
drvAsynWienerVme::drvAsynWienerVme( const char *portName, const char *RPiCanPort, const int crate_id ) 
  : asynPortDriver( portName, 
                    1, /* maxAddr */ 
                    NUM_WIENERVME_PARAMS,
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynInt32Mask | asynUInt32DigitalMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynWienerVme";
  asynStatus status = asynSuccess;

  deviceName_  = epicsStrDup( portName );
  crate_id_    = crate_id;
  
  // Create parameters
  createParam( P_WIENERVME_STATUS0_STRING,    asynParamUInt32Digital, &P_Status0 );
  createParam( P_WIENERVME_STATUS1_STRING,    asynParamUInt32Digital, &P_Status1 );
  createParam( P_WIENERVME_STATUS2_STRING,    asynParamUInt32Digital, &P_Status2 );
  createParam( P_WIENERVME_STATUS3_STRING,    asynParamUInt32Digital, &P_Status3 );
  createParam( P_WIENERVME_STATUS4_STRING,    asynParamUInt32Digital, &P_Status4 );
  createParam( P_WIENERVME_STATUS5_STRING,    asynParamUInt32Digital, &P_Status5 );
  createParam( P_WIENERVME_STATUS6_STRING,    asynParamUInt32Digital, &P_Status6 );
  createParam( P_WIENERVME_STATUS7_STRING,    asynParamUInt32Digital, &P_Status7 );
  createParam( P_WIENERVME_V0_STRING,         asynParamInt32,         &P_Vmom0 );
  createParam( P_WIENERVME_V1_STRING,         asynParamInt32,         &P_Vmom1 );
  createParam( P_WIENERVME_V2_STRING,         asynParamInt32,         &P_Vmom2 );
  createParam( P_WIENERVME_V3_STRING,         asynParamInt32,         &P_Vmom3 );
  createParam( P_WIENERVME_V4_STRING,         asynParamInt32,         &P_Vmom4 );
  createParam( P_WIENERVME_V5_STRING,         asynParamInt32,         &P_Vmom5 );
  createParam( P_WIENERVME_V6_STRING,         asynParamInt32,         &P_Vmom6 );
  createParam( P_WIENERVME_V7_STRING,         asynParamInt32,         &P_Vmom7 );
  createParam( P_WIENERVME_I0_STRING,         asynParamInt32,         &P_Imom0 );
  createParam( P_WIENERVME_I1_STRING,         asynParamInt32,         &P_Imom1 );
  createParam( P_WIENERVME_I2_STRING,         asynParamInt32,         &P_Imom2 );
  createParam( P_WIENERVME_I3_STRING,         asynParamInt32,         &P_Imom3 );
  createParam( P_WIENERVME_I4_STRING,         asynParamInt32,         &P_Imom4 );
  createParam( P_WIENERVME_I5_STRING,         asynParamInt32,         &P_Imom5 );
  createParam( P_WIENERVME_I6_STRING,         asynParamInt32,         &P_Imom6 );
  createParam( P_WIENERVME_I7_STRING,         asynParamInt32,         &P_Imom7 );
  createParam( P_WIENERVME_FANMIDDLE_STRING,  asynParamInt32,         &P_FanMiddle );
  createParam( P_WIENERVME_FANNOMINAL_STRING, asynParamInt32,         &P_FanNominal );
  createParam( P_WIENERVME_FAN1_STRING,       asynParamInt32,         &P_Fan1 );
  createParam( P_WIENERVME_FAN2_STRING,       asynParamInt32,         &P_Fan2 );
  createParam( P_WIENERVME_FAN3_STRING,       asynParamInt32,         &P_Fan3 );
  createParam( P_WIENERVME_FAN4_STRING,       asynParamInt32,         &P_Fan4 );
  createParam( P_WIENERVME_FAN5_STRING,       asynParamInt32,         &P_Fan5 );
  createParam( P_WIENERVME_FAN6_STRING,       asynParamInt32,         &P_Fan6 );
  createParam( P_WIENERVME_TEMP1_STRING,      asynParamInt32,         &P_Temp1 );
  createParam( P_WIENERVME_TEMP2_STRING,      asynParamInt32,         &P_Temp2 );
  createParam( P_WIENERVME_TEMP3_STRING,      asynParamInt32,         &P_Temp3 );
  createParam( P_WIENERVME_TEMP4_STRING,      asynParamInt32,         &P_Temp4 );
  createParam( P_WIENERVME_TEMP5_STRING,      asynParamInt32,         &P_Temp5 );
  createParam( P_WIENERVME_TEMP6_STRING,      asynParamInt32,         &P_Temp6 );
  createParam( P_WIENERVME_TEMP7_STRING,      asynParamInt32,         &P_Temp7 );
  createParam( P_WIENERVME_TEMP8_STRING,      asynParamInt32,         &P_Temp8 );
  createParam( P_WIENERVME_SWITCH_STRING,     asynParamUInt32Digital, &P_Switch );
  createParam( P_WIENERVME_SYSRESET_STRING,   asynParamInt32,         &P_Sysreset );
  createParam( P_WIENERVME_CHANGEFAN_STRING,  asynParamInt32,         &P_FanSpeed );

  /* Connect to asyn generic pointer port with asynGenericPointerSyncIO */
  status = pasynGenericPointerSyncIO->connect( RPiCanPort, 0, &pAsynUserGenericPointer_, 0 );
  if ( status != asynSuccess ) {
    printf( "%s:%s:%s: can't connect to asynGenericPointer on port '%s'\n", 
            driverName, deviceName_, functionName, RPiCanPort );
  }
  
  // Get inital value for Switch-Parameter
  can_frame_t *pframe = new can_frame_t;
  pframe->can_id  = crate_id_ | CAN_RTR_FLAG;
  pframe->can_dlc = 1;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: Init %s: No reply from device within 1 s!\033[0m\n",
             driverName, deviceName_, functionName, P_WIENERVME_SWITCH_STRING );
    return;
  }
  setUIntDigitalParam( P_Switch, pframe->data[0] & 0x01, 1 );

  // initial value for nominal fan speed
  pframe->can_id  = crate_id_ | ( 6 << 7 ) | CAN_RTR_FLAG;
  pframe->can_dlc = 2;
  status = pasynGenericPointerSyncIO->writeRead( pAsynUserGenericPointer_, pframe, pframe, 1. );
  if ( status != asynSuccess ) {
    fprintf( stderr, "\033[31;1m%s:%s:%s: Init %s: No reply from device within 1 s!\033[0m\n",
             driverName, deviceName_, functionName, P_WIENERVME_CHANGEFAN_STRING );
    return;
  }
  setIntegerParam( P_FanSpeed, pframe->data[1] );

}

//******************************************************************************
//! EOF
//******************************************************************************
