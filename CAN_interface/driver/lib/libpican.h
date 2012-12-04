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
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   common header to access the functions within libpican.so.x.x
//!
//! @version 1.0.0
//******************************************************************************

#ifndef __LIB_PI_CAN_H__
#define __LIB_PI_CAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <fcntl.h>
#include <rpi_can.h>

//_____ D E F I N I T I O N S __________________________________________________
#if defined(LPSTR) || defined(CANBUS)
#error "double define for LPSTR, CANBUS found"
#endif

typedef struct can_descriptor CANBUS;

//****************************************************************************
// error codes are defined in pcan.h
#define CAN_ERR_ANYBUSERR (CAN_ERR_BUSLIGHT | CAN_ERR_BUSHEAVY | CAN_ERR_BUSOFF)

//****************************************************************************
// PROTOTYPES
#ifdef __cplusplus
extern "C"
{
#endif

//****************************************************************************
//  CAN_Open() - open, LINUX like
//  creates a path to a CAN port
//
//  input: the path to the device node (e.g. /dev/pcan0)
//  returns NULL when open failes
//
CANBUS* CAN_Open(const char *szDeviceName, int nFlag);

//****************************************************************************
//  CAN_Close()
//  closes the path to the CAN hardware.
//  The last close on the hardware put the chip into passive state.
DWORD CAN_Close(CANBUS *hHandle);

//****************************************************************************
//  CAN_Status()
//  request the current (stored) status of the CAN hardware. After the read the
//  stored status is reset.
//  If the status is negative a system error is returned (e.g. -EBADF).
DWORD CAN_Status(CANBUS *hHandle, TPSTATUS *status);

//****************************************************************************
//  CAN_Read()
//  reads a message WITH TIMESTAMP from the CAN bus. If there is no message
//  to read the current request blocks until either a new message arrives
//  or a error occures.
DWORD CAN_Read(CANBUS *hHandle, struct can_frame *pMsgBuff);

//****************************************************************************
//  LINUX_CAN_Read_Timeout()
//  reads a message WITH TIMESTAMP from the CAN bus. If there is no message
//  to read the current request blocks until either a new message arrives
//  or a timeout or a error occures.
//  nMicroSeconds  > 0 -> Timeout in microseconds
//  nMicroSeconds == 0 -> polling
//  nMicroSeconds  < 0 -> blocking, same as LINUX_CAN_Read()
DWORD CAN_Read_Timeout(CANBUS *hHandle, struct can_frame *pMsgBuff, int nMicroSeconds);

//****************************************************************************
//  CAN_Write()
//  writes a message to the CAN bus. If the write queue is full the current
//  write blocks until either a message is sent or a error occured.
//
DWORD CAN_Write(CANBUS *hHandle, struct can_frame* pMsgBuff);

//****************************************************************************
//  CAN_Write_Timeout()
//  writes a message to the CAN bus. If the (software) message buffer is full
//  the current write request blocks until a write slot gets empty
//  or a timeout or a error occures.
//  nMicroSeconds  > 0 -> Timeout in microseconds
//  nMicroSeconds == 0 -> polling
//  nMicroSeconds  < 0 -> blocking, same as CAN_Write()
DWORD CAN_Write_Timeout(CANBUS *hHandle, struct can_frame* pMsgBuff, int nMicroSeconds);

//****************************************************************************
//  CAN_VersionInfo()
//  returns a text string with driver version info.
//
DWORD CAN_VersionInfo(CANBUS *hHandle, char *lpszTextBuff);

//****************************************************************************
//  nGetLastError()
//  returns the last stored error (errno of the shared library). The returend
//  error is independend of any path.
//
int nGetLastError(void);

//****************************************************************************
//  LINUX_CAN_Statistics() - get statistics about this devices
//
DWORD CAN_Statistics(CANBUS *hHandle, TPDIAG *diag);

//****************************************************************************
//  LINUX_CAN_Statistics() - set acceptance filter
//
DWORD CAN_Filter(CANBUS *hHandle, TPMSGFILTER *filter);

//****************************************************************************
//  LINUX_CAN_BTR0BTR1() - get the BTR0 and BTR1 from bitrate, LINUX like
//
//  input:  the handle to the device node
//          the bitrate in bits / second, e.g. 500000 bits/sec
//
//  returns 0 if not possible
//          BTR0BTR1 for the interface
//
WORD CAN_Bitrate(CANBUS *hHandle, DWORD dwBitRate);

#ifdef __cplusplus
}
#endif // __cplusplus

bool operator==( const struct can_frame &rhs, const struct can_frame &lhs );
bool operator!=( const struct can_frame &rhs, const struct can_frame &lhs );

#endif // __LIBPCAN_H__
