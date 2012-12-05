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
//! @brief   Asyn Port Driver for RPi Can interface
//!
//! @version 1.0.0; Nov. 27, 2012
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________

/* ANSI C includes  */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <rpi_can.h>
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

#include "drvAsynRPiCan.h"
#include "drvAsynRPiCanDebug.h"

//_____ D E F I N I T I O N S __________________________________________________
#define DEFAULT_BITRATE 125000U

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static const char *driverName = "drvAsynRPiCanDriver";

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynGenericPointer->read().
//!          Generic Pointer should match can_frame_t
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//!          [out] genericPointer  Pointer to the object to read.
//------------------------------------------------------------------------------
asynStatus drvAsynRPiCan::readGenericPointer( asynUser *pasynUser, void *genericPointer ) {
  const char* functionName = "readGenericPointer";
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
  can_frame_t* pframe = (can_frame_t *)genericPointer;
  int err = drvRPiCanRead( pframe, mytimeout );
  
  if ( CAN_ERR_QRCVEMPTY == err )  return asynTimeout;

  if ( 0 != err ) {
    fprintf( stderr, "\033[31;1m %s:%s: Error receiving message from device: %d %s \033[0m \n", 
             driverName, functionName, err, strerror( err ) );
    return asynError;
  }  
  if ( drvAsynRPiCanDebug > 0 ) {
    printf( "%s:%s: received frame '%08x %d %02x %02x %02x %02x %02x %02x %02x %02x'\n", 
            driverName, functionName, pframe->can_id, pframe->can_dlc,
            pframe->data[0], pframe->data[1], pframe->data[2], pframe->data[3],
            pframe->data[4], pframe->data[5], pframe->data[6], pframe->data[7] );
  }

  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   Called when asyn clients call pasynGenericPointer->write().
//!          Generic Pointer should match can_frame_t
//!
//! @param   [in]  pasynUser       pasynUser structure that encodes the reason and address.
//!          [in]  genericPointer  Pointer to the object to write.
//------------------------------------------------------------------------------
asynStatus drvAsynRPiCan::writeGenericPointer( asynUser *pasynUser, void *genericPointer ) {
  const char* functionName = "writeGenericPointer";
  can_frame_t *myFrame = (can_frame_t *)genericPointer;
  int mytimeout = (int)( pasynUser->timeout * 1.e6 );
 
  if ( drvAsynRPiCanDebug > 0 ) {
    printf( "%s:%s: sending frame '%08x %d %02x %02x %02x %02x %02x %02x %02x %02x'\n", 
            driverName, functionName, myFrame->can_id, myFrame->can_dlc,
            myFrame->data[0], myFrame->data[1], myFrame->data[2], myFrame->data[3],
            myFrame->data[4], myFrame->data[5], myFrame->data[6], myFrame->data[7] );
  }
  int err = drvRPiCanWrite( myFrame, mytimeout );
  if ( 0 != err ) {
    fprintf( stderr, "\033[31;1m %s:%s: Error sending message to device: %s \033[0m \n", 
             driverName, functionName, strerror( err ) );
    return asynError;
  }  
  return asynSuccess;
}

//------------------------------------------------------------------------------
//! @brief   write a frame to the interface
//!
//! @param   [in]  pframe     CAN frame to send
//!          [in]  timeout    timeout in microseconds
//------------------------------------------------------------------------------
int drvAsynRPiCan::drvRPiCanWrite( can_frame_t *pframe, int timeout ){

  if ( timeout < 0)
    return ioctl( fd_, CAN_WRITE_MSG, pframe );
  
  fd_set fdWrite;
  struct timeval t;
  
  // calculate timeout values
  t.tv_sec  = timeout / 1000000L;
  t.tv_usec = timeout % 1000000L;
  
  FD_ZERO( &fdWrite );
  FD_SET( fd_, &fdWrite );
  
  // wait until timeout or a message is ready to get written
  int err = select( fd_ + 1, NULL, &fdWrite, NULL, &t );
  
  // the only one file descriptor is ready for write
  if ( err  > 0 )
    return ioctl( fd_, CAN_WRITE_MSG, pframe );
  
  // nothing is ready, timeout occured
  if ( err == 0 )
    return CAN_ERR_QXMTFULL;
  return err;
}

//------------------------------------------------------------------------------
//! @brief   read a frame from the interface
//!
//! @param   [out] pframe     CAN frame read
//!          [in]  timeout    timeout in microseconds
//------------------------------------------------------------------------------
int drvAsynRPiCan::drvRPiCanRead( can_frame_t *pframe, int timeout ){
  if ( timeout < 0)
    return ioctl( fd_, CAN_READ_MSG, pframe );

  fd_set fdRead;
  struct timeval t;
  
  // calculate timeout values
  t.tv_sec  = timeout / 1000000L;
  t.tv_usec = timeout % 1000000L;
  
  FD_ZERO( &fdRead );
  FD_SET( fd_, &fdRead );
  
  // wait until timeout or a message is ready to get read
  int err = select( fd_ + 1, &fdRead, NULL, NULL, &t );
  
  // the only one file descriptor is ready for read
  if ( err  > 0 )
    return ioctl( fd_, CAN_READ_MSG, pframe );
  
  // nothing is ready, timeout occured
  if ( err == 0 )
    return CAN_ERR_QRCVEMPTY;
  return err;
}

//------------------------------------------------------------------------------
//! @brief   Constructor for the drvAsynRPiCan class.
//!          Calls constructor for the asynPortDriver base class.
//!
//! @param   [in]  portName The name of the asyn port driver to be created.
//!          [in]  ttyName  The name of the device
//!          [in]  bitrate  The bitrate of the CAN interface
//------------------------------------------------------------------------------
drvAsynRPiCan::drvAsynRPiCan( const char *portName, const char *ttyName, const int bitrate ) 
  : asynPortDriver( portName,
                    1, /* maxAddr */ 
                    NUM_RPICAN_PARAMS,
                    asynCommonMask | asynGenericPointerMask | asynDrvUserMask, /* Interface mask */
                    asynCommonMask | asynGenericPointerMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0 ) /* Default stack size*/    
{
  const char *functionName = "drvAsynRPiCan";
  
  createParam( P_GENERIC_String, asynParamGenericPointer, &P_GENERIC );
  
  deviceName_ = epicsStrDup( ttyName );

  // open interface
  fd_ = open( deviceName_, O_RDWR );
  if ( 0 > fd_ ){
    fprintf( stderr, "\033[31;1m %s:%s: Could not open interface '%s'. %s \033[0m \n",
             driverName, functionName, deviceName_, strerror( errno ) );
    return;
  }
  
  // set baudrate
  if ( bitrate != DEFAULT_BITRATE ) {
    int err;
    TPBTR0BTR1 ratix = { bitrate, 0 };
    err = ioctl( fd_, CAN_BITRATE, &ratix );
    if ( err ) {
      fprintf( stderr, "\033[31;1m %s:%s: Could not change bitrate for interface '%s'. %s \033[0m \n",
               driverName, functionName, deviceName_, strerror( err ) );
      return;
    }
  }
  
}

//******************************************************************************
//! EOF
//******************************************************************************
