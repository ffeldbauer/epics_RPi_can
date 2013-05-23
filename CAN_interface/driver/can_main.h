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
// This source code is based on peak-linux-driver-7.6
//   linux@peak-system.com
//   www.peak-system.com
//
// brief   Main part of driver. Defines fops, init and release funcitons
//         global defines to include in all files this module is made of
//
// version 2.0.0
//******************************************************************************

#ifndef __CAN_MAIN_H__
#define __CAN_MAIN_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>
#include <linux/list.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>

#include "rpi_can.h"

//_____ D E F I N I T I O N S __________________________________________________
#define READBUFFER_SIZE      80                            // buffers used in read and write call
#define WRITEBUFFER_SIZE     80
#define READ_MESSAGE_COUNT  500                            // max read message count
#define WRITE_MESSAGE_COUNT  50                            // max write message count

// parameter wBTR0BTR1
// bitrate codes of BTR0/BTR1 registers
#define CAN_BAUD_1M     0x0014                             //   1 MBit/s
#define CAN_BAUD_500K   0x001C                             // 500 kBit/s
#define CAN_BAUD_250K   0x011C                             // 250 kBit/s
#define CAN_BAUD_125K   0x031C                             // 125 kBit/s
#define CAN_BAUD_100K   0x432F                             // 100 kBit/s
#define CAN_BAUD_50K    0x472F                             //  50 kBit/s
#define CAN_BAUD_20K    0x532F                             //  20 kBit/s
#define CAN_BAUD_10K    0x672F                             //  10 kBit/s
#define CAN_BAUD_5K     0x7F7F                             //   5 kBit/s

// Activity states
#define ACTIVITY_NONE        0          // LED off           - set when the channel is created or deleted
#define ACTIVITY_INITIALIZED 1          // LED on            - set when the channel is initialized
#define ACTIVITY_IDLE        2          // LED slow blinking - set when the channel is ready to receive or transmit
#define ACTIVITY_XMIT        3          // LED fast blinking - set when the channel has received or transmitted

#define CAN_ERROR_ACTIVE     0          // CAN-Bus error states for busStatus - initial and normal state
#define CAN_ERROR_PASSIVE    1          // receive only state
#define CAN_BUS_OFF          2          // switched off from Bus

// a helper for fast conversion between 'SJA1000' data ordering and host data order
typedef union {
  u8  uc[4];
  u32 ul;
} ULCONV;

typedef struct {
  u16        wStepSize;                                    // size of bytes to step to next entry
  u16        wCopySize;                                    // size of bytes to copy
  void       *bufferBegin;                                 // points to first element
  void       *bufferEnd;                                   // points to the last element
  u32        nCount;                                       // max count of elements in fifo
  u32        nStored;                                      // count of currently received and stored messages
  u32        dwTotal;                                      // received messages
  void       *r;                                           // points to the next Msg to read into the read buffer
  void       *w;                                           // points to the next Msg to write into read buffer
  spinlock_t lock;                                         // mutual exclusion lock
} FIFO_MANAGER;

typedef struct candev {
  wait_queue_head_t read_queue;                            // read process wait queue anchor
  wait_queue_head_t write_queue;                           // write process wait queue anchor
  int nOpenPaths;
  int wIrq;

  u16  bus_load;
  int  nLastError;                                         // last error written
  int  busStatus;                                          // follows error status of CAN-Bus
  u32  dwErrorCounter;                                     // counts all fatal errors
  u32  dwInterruptCounter;                                 // counts all interrupts
  u16  wCANStatus;                                         // status of CAN chip
  u16  wBTR0BTR1;                                          // the persistent storage for BTR0 and BTR1
  u8   ucPhysicallyInstalled;                              // the device is PhysicallyInstalled
  u8   ucActivityState;                                    // follow the state of a channel activity
  atomic_t DataSendReady;                                  // !=0 if all data are send

  FIFO_MANAGER readFifo;                                   // manages the read fifo
  FIFO_MANAGER writeFifo;                                  // manages the write fifo

  struct can_frame rMsg[READ_MESSAGE_COUNT];               // all read messages
  struct can_frame wMsg[WRITE_MESSAGE_COUNT];              // all write messages

  void* filter;

  spinlock_t wlock;                                        // mutual exclusion lock for write invocation
  spinlock_t isr_lock;                                     // in isr
} CANDEV;

typedef struct fileobj {
  u8     pcReadBuffer[READBUFFER_SIZE];                    // buffer used in read() call
  u8     *pcReadPointer;                                   // points into current read data rest
  int    nReadRest;                                        // rest of data left to read
  int    nTotalReadCount;                                  // for test only
  u8     pcWriteBuffer[WRITEBUFFER_SIZE];                  // buffer used in write() call
  u8     *pcWritePointer;                                  // work pointer into buffer
  int    nWriteCount;                                      // count of written data bytes
} FILEOBJ;

#endif

//******************************************************************************
//! EOF
//******************************************************************************
