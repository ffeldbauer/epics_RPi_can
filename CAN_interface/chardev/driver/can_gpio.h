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
// brief   header for all parts of the GPIO hardware
//
// version 2.0.0
//******************************************************************************

#ifndef __CAN_GPIO_H__
#define __CAN_GPIO_H__

//_____ I N C L U D E S _______________________________________________________
#include "can_main.h"

//_____ D E F I N I T I O N S __________________________________________________
u8 can_gpio_readreg( u8 reg );
void can_gpio_writereg( u8 reg, u8 data );
int can_gpio_init( struct candev* dev );
int can_gpio_req_irq( struct candev* dev );
void can_gpio_free( void );
void can_gpio_free_irq( struct candev* dev );

#endif

//******************************************************************************
//! EOF
//******************************************************************************
