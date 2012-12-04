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
//! This source code is based on peak-linux-driver-7.6
//!   linux@peak-system.com
//!   www.peak-system.com
//!
//! @brief   Library for using Raspberry Pi CAN interface
//!
//! @version 1.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "lib/libpican.h"

//_____ D E F I N I T I O N S __________________________________________________
#define LOCAL_STRING_LEN 64       // length of internal used strings
#define MAGICNUMBER 1963460443U   // Magic number crc("libpican")

//_____ G L O B A L S __________________________________________________________
struct can_descriptor {
  char szVersionString[LOCAL_STRING_LEN];
  char szDevicePath[LOCAL_STRING_LEN];
  int  nFileNo;
  uint32_t magic; // magic number
};

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//----------------------------------------------------------------------------
// do a unix like open of the device
CANBUS *CAN_Open(const char *szDeviceName, int nFlag) {
  CANBUS *desc = NULL;

  errno = 0;

  if( ( desc = (CANBUS *)malloc(sizeof(CANBUS))) == NULL )
    goto fail;

  desc->szVersionString[0] = 0;
  desc->szDevicePath[0]    = 0;
  desc->magic = MAGICNUMBER;

  if( ( desc->nFileNo = open( szDeviceName, nFlag ) ) == -1 )
    goto fail;

  strncpy(desc->szDevicePath, szDeviceName, LOCAL_STRING_LEN);

  return desc;

 fail:
  printf("Error in CAN_Open\n");
  if (desc) {
  printf("...allocating memory worked: %d\n", desc->nFileNo);
    if (desc->nFileNo > -1)
      close( desc->nFileNo );
    free( desc );
  }

  return NULL;
}

DWORD CAN_Close(CANBUS *hHandle) {

  if (hHandle) {
    if (hHandle->nFileNo > -1) {
      close(hHandle->nFileNo);
      hHandle->nFileNo = -1;
    }
    free(hHandle);
  }
  return 0;
}

DWORD CAN_Read(CANBUS *hHandle, struct can_frame* pMsgBuff) {
  int err = EBADF;

  errno = err;
  if (hHandle)
    if ( hHandle->magic != MAGICNUMBER ) return err;
    err = ioctl(hHandle->nFileNo, CAN_READ_MSG, pMsgBuff);

  return err;
}

DWORD CAN_Read_Timeout(CANBUS *hHandle, struct can_frame* pMsgBuff, int nMicroSeconds) {
  int err = EBADF;

  if (nMicroSeconds < 0)
    return CAN_Read(hHandle, pMsgBuff);

  if (hHandle) {
    if ( hHandle->magic != MAGICNUMBER ) return err;
    fd_set fdRead;
    struct timeval t;

    // calculate timeout values
    t.tv_sec  = nMicroSeconds / 1000000L;
    t.tv_usec = nMicroSeconds % 1000000L;

    FD_ZERO(&fdRead);
    FD_SET(hHandle->nFileNo, &fdRead);

    // wait until timeout or a message is ready to read
    err = select(hHandle->nFileNo + 1, &fdRead, NULL, NULL, &t);

    // the only one file hHandleriptor is ready for read
    if (err  > 0)
      return CAN_Read(hHandle, pMsgBuff);

    // nothing is ready, timeout occured
    if (err == 0)
      return CAN_ERR_QRCVEMPTY;
  }

  // any else error
  return err;
}

DWORD CAN_Write (CANBUS *hHandle, struct can_frame* pMsgBuff) {
  int err = EBADF;

  errno = err;
  if (hHandle) {
    if ( hHandle->magic != MAGICNUMBER ) return err;
    if ((err = ioctl(hHandle->nFileNo, CAN_WRITE_MSG, pMsgBuff)) < 0)
      return err;
  }

  return err;
}

DWORD CAN_Write_Timeout(CANBUS *hHandle, struct can_frame* pMsgBuff, int nMicroSeconds) {
  int err = EBADF;

  if (nMicroSeconds < 0)
    return CAN_Write(hHandle, pMsgBuff);

  if (hHandle) {
    if ( hHandle->magic != MAGICNUMBER ) return err;
    fd_set fdWrite;
    struct timeval t;

    // calculate timeout values
    t.tv_sec  = nMicroSeconds / 1000000L;
    t.tv_usec = nMicroSeconds % 1000000L;

    FD_ZERO(&fdWrite);
    FD_SET(hHandle->nFileNo, &fdWrite);

    // wait until timeout or a message is ready to get written
    err = select(hHandle->nFileNo + 1,  NULL, &fdWrite,NULL, &t);

    // the only one file hHandleriptor is ready for write
    if (err  > 0)
      return CAN_Write(hHandle, pMsgBuff);

    // nothing is ready, timeout occured
    if (err == 0)
      return CAN_ERR_QXMTFULL;
  }

  // any else error
  return err;
}

DWORD CAN_Filter (CANBUS *hHandle, TPMSGFILTER* filter) {
  int err = EBADF;

  errno = err;
  if (hHandle) {
    if ( hHandle->magic != MAGICNUMBER ) return err;
    if ((err = ioctl(hHandle->nFileNo, CAN_MSG_FILTER, filter)) < 0)
      return err;
  }

  return err;
}

DWORD CAN_Status(CANBUS *hHandle, TPSTATUS *status) {
  int err = EBADF;

  errno = err;
  if (hHandle) {
    err = ioctl( hHandle->nFileNo, CAN_GET_STATUS, status );
  }

  return err;
}

int nGetLastError(void) {
  return errno;
}

WORD CAN_Bitrate(CANBUS *hHandle, DWORD dwBitRate) {
  int err = EBADF;
  TPBTR0BTR1 ratix;

  ratix.dwBitRate = dwBitRate;
  ratix.wBTR0BTR1 = 0;

  errno = err;
  if (hHandle)
    if ( hHandle->magic != MAGICNUMBER ) return err;
    err = ioctl( hHandle->nFileNo, CAN_BITRATE, &ratix );

  return err;
}

DWORD CAN_Statistics(CANBUS *hHandle, TPDIAG *diag) {
  int err = EBADF;

  errno = err;
  if (hHandle)
    if ( hHandle->magic != MAGICNUMBER ) return err;
    err = ioctl(hHandle->nFileNo, CAN_DIAG, diag);

  return err;
}

DWORD CAN_VersionInfo(CANBUS *hHandle, char* szTextBuff) {
  int err;
  TPDIAG diag;

  *szTextBuff = 0;

  err = (int)CAN_Statistics(hHandle, &diag);
  if (err)
    return err;

  strncpy(szTextBuff, diag.szVersionString, VERSIONSTRING_LEN);

  return err;
}

bool operator==( const struct can_frame &rhs, const struct can_frame &lhs ) {
  return ( ( rhs.can_id  == lhs.can_id  ) &&
           ( rhs.can_dlc == lhs.can_dlc ) &&
           ( rhs.data[0] == lhs.data[0] ) &&
           ( rhs.data[1] == lhs.data[1] ) &&
           ( rhs.data[2] == lhs.data[2] ) &&
           ( rhs.data[3] == lhs.data[3] ) &&
           ( rhs.data[4] == lhs.data[4] ) &&
           ( rhs.data[5] == lhs.data[5] ) &&
           ( rhs.data[6] == lhs.data[6] ) &&
           ( rhs.data[7] == lhs.data[7] ) );
}

bool operator!=( const struct can_frame &rhs, const struct can_frame &lhs ){
  return ( ( rhs.can_id  != lhs.can_id  ) ||
           ( rhs.can_dlc != lhs.can_dlc ) ||
           ( rhs.data[0] != lhs.data[0] ) ||
           ( rhs.data[1] != lhs.data[1] ) ||
           ( rhs.data[2] != lhs.data[2] ) ||
           ( rhs.data[3] != lhs.data[3] ) ||
           ( rhs.data[4] != lhs.data[4] ) ||
           ( rhs.data[5] != lhs.data[5] ) ||
           ( rhs.data[6] != lhs.data[6] ) ||
           ( rhs.data[7] != lhs.data[7] ) );
}

