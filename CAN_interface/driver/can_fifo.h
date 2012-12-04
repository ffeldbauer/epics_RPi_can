//******************************************************************************
//! @file    can_fifo.h
//!
//! @author  F. Feldbauer
//!
//! @brief   all about fifo buffer management
//!          BASED ON peak-linux-driver-7.6 (http://www.peak-system.com/)
//!
//! @version 1.0.0
//******************************************************************************

#ifndef __CAN_FIFO_H__
#define __CAN_FIFO_H__

//_____ I N C L U D E S _______________________________________________________
#include "can_main.h"

//_____ D E F I N I T I O N S __________________________________________________
int can_fifo_reset( register FIFO_MANAGER* anchor);
int can_fifo_init( register FIFO_MANAGER* anchor, void* bufferBegin, void* bufferEnd, int nCount, u16 wCopySize );
int can_fifo_put( register FIFO_MANAGER* anchor, void* pvPutData);
int can_fifo_get( register FIFO_MANAGER* anchor, void* pvPutData);
int can_fifo_status( FIFO_MANAGER* anchor );
int can_fifo_not_full( FIFO_MANAGER* anchor );
int can_fifo_empty( FIFO_MANAGER* anchor );

#endif // __CAN_FIFO_H__

