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
// brief   all about fifo buffer management
//
// version 2.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>
#include <linux/errno.h>    // error codes
#include <linux/string.h>   // memcpy
#include <linux/sched.h>
#include <asm/system.h>     // cli(), save_flags(), restore_flags()
#include <linux/spinlock.h>

#include "can_fifo.h"

//_____ D E F I N I T I O N S __________________________________________________
#define PCAN_FIFO_FIX_NOT_FULL_TEST

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________
int can_fifo_reset( register FIFO_MANAGER* anchor ) {
  unsigned long flags;

  spin_lock_irqsave( &anchor->lock, flags );

  anchor->dwTotal       = 0;
  anchor->nStored       = 0;
  anchor->r = anchor->w = anchor->bufferBegin; // nothing to read

  spin_unlock_irqrestore( &anchor->lock, flags );

  // DPRINTK(KERN_DEBUG "%s: pcan_fifo_reset() %d %p %pd\n", DEVICE_NAME, anchor->nStored, anchor->r, anchor->w);

  return 0;
}

int can_fifo_init( register FIFO_MANAGER* anchor, void* bufferBegin, void* bufferEnd, int nCount, u16 wCopySize ) {
  anchor->wCopySize   = wCopySize;
  anchor->wStepSize   = (bufferBegin == bufferEnd) ? 0 : ((bufferEnd - bufferBegin) / (nCount - 1));
  anchor->nCount      = nCount;
  anchor->bufferBegin = bufferBegin;
  anchor->bufferEnd   = bufferEnd;

  // check for fatal program errors
  if ((anchor->wStepSize < anchor->wCopySize) || (anchor->bufferBegin > anchor->bufferEnd) || (nCount <= 1))
    return -EINVAL;

  spin_lock_init( &anchor->lock );

  return can_fifo_reset( anchor );
}

int can_fifo_put( register FIFO_MANAGER* anchor, void* pvPutData ) {
  int err = 0;
  unsigned long flags;

  // DPRINTK(KERN_DEBUG "%s: pcan_fifo_put() %d %p %p\n", DEVICE_NAME, anchor->nStored, anchor->r, anchor->w);

  spin_lock_irqsave( &anchor->lock, flags );

  if (anchor->nStored < anchor->nCount) {
    memcpy(anchor->w, pvPutData, anchor->wCopySize);

    anchor->nStored++;
    anchor->dwTotal++;

    if (anchor->w < anchor->bufferEnd)
      anchor->w += anchor->wStepSize;   // increment to next
    else
      anchor->w = anchor->bufferBegin;  // start from begin
  }
  else
    err = -ENOSPC;

  spin_unlock_irqrestore( &anchor->lock, flags );

  return err;
}


int can_fifo_get( register FIFO_MANAGER* anchor, void* pvGetData ) {
  int err = 0;
  unsigned long flags;

  // DPRINTK(KERN_DEBUG "%s: pcan_fifo_get() %d %p %p\n", DEVICE_NAME, anchor->nStored, anchor->r, anchor->w);

  spin_lock_irqsave( &anchor->lock, flags );

  if (anchor->nStored > 0) {
    memcpy(pvGetData, anchor->r, anchor->wCopySize);

    anchor->nStored--;
    if (anchor->r < anchor->bufferEnd)
      anchor->r += anchor->wStepSize;  // increment to next
    else
      anchor->r = anchor->bufferBegin; // start from begin
  }
  else
    err = -ENODATA;

  spin_unlock_irqrestore( &anchor->lock, flags );

  return err;
}


//----------------------------------------------------------------------------
// returns the current count of elements in fifo
int can_fifo_status( FIFO_MANAGER* anchor ) {
  return anchor->nStored;
}

//----------------------------------------------------------------------------
// returns 0 if the fifo is full
int can_fifo_not_full( FIFO_MANAGER* anchor ) {
#ifdef PCAN_FIFO_FIX_NOT_FULL_TEST
  int r;
  unsigned long flags;

  spin_lock_irqsave( &anchor->lock, flags );
  r = (anchor->nStored < anchor->nCount);
  spin_unlock_irqrestore( &anchor->lock, flags );

  return r;
#else
  return (anchor->nStored < (anchor->nCount - 1));
#endif
}

//----------------------------------------------------------------------------
// returns !=0 if the fifo is empty
int can_fifo_empty(FIFO_MANAGER* anchor) {
  return !(anchor->nStored);
}


