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
//! @brief   constants and definitions to access the drivers
//!
//! @version 2.0.0
//******************************************************************************

#ifndef __RASPPI_CAN_H__
#define __RASPPI_CAN_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/can.h>

//_____ D E F I N I T I O N S __________________________________________________
#define CURRENT_RELEASE "Release_20130328_n"
#define DEVICE_NAME "RaspPiCanInterfaceRev1"

//****************************************************************************
// compatibilty defines
#if defined(DWORD) || defined(WORD) || defined(BYTE)
#error "double define for DWORD, WORD, BYTE found"
#endif

#ifdef __KERNEL__
#define DWORD  u32
#define WORD   u16
#define BYTE   u8
#else
#define DWORD  __u32
#define WORD   __u16
#define BYTE   __u8
#endif

//****************************************************************************
// error codes
#define CAN_ERR_OK             0x0000  // no error
#define CAN_ERR_XMTFULL        0x0001  // transmit buffer full
#define CAN_ERR_OVERRUN        0x0002  // overrun in receive buffer
#define CAN_ERR_BUSLIGHT       0x0004  // bus error, errorcounter limit reached
#define CAN_ERR_BUSHEAVY       0x0008  // bus error, errorcounter limit reached
//#define CAN_ERR_BUSOFF         0x0010  // bus error, 'bus off' state entered
#define CAN_ERR_QRCVEMPTY      0x0020  // receive queue is empty
#define CAN_ERR_QOVERRUN       0x0040  // receive queue overrun
#define CAN_ERR_QXMTFULL       0x0080  // transmit queue full
#define CAN_ERR_REGTEST        0x0100  // test of controller registers failed
#define CAN_ERR_NOVXD          0x0200  // Win95/98/ME only
#define CAN_ERR_RESOURCE       0x2000  // can't create resource
#define CAN_ERR_ILLPARAMTYPE   0x4000  // illegal parameter
#define CAN_ERR_ILLPARAMVAL    0x8000  // value out of range
#define CAN_ERRMASK_ILLHANDLE  0x1C00  // wrong handle, handle error

//****************************************************************************
// MSGTYPES
#define MSGTYPE_EXTENDED 0x02
#define MSGTYPE_RTR      0x01
#define MSGTYPE_STANDARD 0x00

//****************************************************************************
// maximum length of the version string (attention: used in driver too)
#define VERSIONSTRING_LEN     64

//****************************************************************************
// structures to communicate via ioctls

typedef struct {
  DWORD dwBase;          // the base address or port of this device
  WORD  wIrqLevel;       // the irq level of this device
  DWORD dwReadCounter;   // counts all reads to this device from start
  DWORD dwWriteCounter;  // counts all writes
  DWORD dwIRQcounter;    // counts all interrupts
  DWORD dwErrorCounter;  // counts all errors
  WORD  wErrorFlag;      // gathers all errors
  int   nLastError;      // the last local error for this device
  char  szVersionString[VERSIONSTRING_LEN]; // driver version string
} TPDIAG;                // for CAN_DIAG, in opposition to CAN_GET_STATUS nothing is cleared

typedef struct {
  DWORD dwBitRate;       // in + out, bitrate in bits per second
  WORD  wBTR0BTR1;       // out only: the result
} TPBTR0BTR1;

typedef struct {
  WORD  wErrorFlag;      // same as in TPDIAG, is cleared in driver after access
  int   nLastError;      // is cleared in driver after access
  int   nPendingReads;   // count of unread telegrams
  int   nPendingWrites;  // count of unsent telegrams
} TPSTATUS;              // for CAN_GET_STATUS

typedef struct {
  DWORD FromID;          // First CAN ID to accept
  DWORD ToID;            // Last CAN ID to accept
  BYTE  MSGTYPE;         // Standard or extended
} TPMSGFILTER;

//****************************************************************************
// some predefines for ioctls
#define CAN_MAGIC_NUMBER  'z'
#define MYSEQ_START        0x80

//****************************************************************************
// ioctls control codes
#define CAN_WRITE_MSG      _IOW (CAN_MAGIC_NUMBER, MYSEQ_START + 0, struct can_frame)
#define CAN_READ_MSG       _IOR (CAN_MAGIC_NUMBER, MYSEQ_START + 1, struct can_frame)
#define CAN_GET_STATUS     _IOR (CAN_MAGIC_NUMBER, MYSEQ_START + 2, TPSTATUS)
#define CAN_DIAG           _IOR (CAN_MAGIC_NUMBER, MYSEQ_START + 3, TPDIAG)
#define CAN_BITRATE        _IOW (CAN_MAGIC_NUMBER, MYSEQ_START + 4, TPBTR0BTR1)
#define CAN_GET_BITRATE    _IOR (CAN_MAGIC_NUMBER, MYSEQ_START + 5, TPBTR0BTR1)
#define CAN_MSG_FILTER     _IOW (CAN_MAGIC_NUMBER, MYSEQ_START + 6, TPMSGFILTER)

#endif // __CAN_H__
