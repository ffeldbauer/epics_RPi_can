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
//! @brief   all about sja1000 init and data handling
//!
//! @version 1.0.0
//******************************************************************************

#ifndef __CAN_SJA1000_H__
#define __CAN_SJA1000_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/interrupt.h> // 2.6. special

#include "can_main.h"

//_____ D E F I N I T I O N S __________________________________________________
int  sja1000_open( struct candev* dev, u16 btr0btr1 );
void sja1000_release( struct candev* dev );
u16  sja1000_bitrate( u32 dwBitRate );
int  sja1000_write( struct candev* dev );
irqreturn_t can_sja1000_irqhandler( int irq, void* dev_id );

#endif

//******************************************************************************
//! EOF
//******************************************************************************
