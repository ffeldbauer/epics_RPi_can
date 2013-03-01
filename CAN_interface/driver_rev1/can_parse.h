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
//! @brief   header for read input parser and write output formatter
//!
//! @version 2.0.0
//******************************************************************************

#ifndef __CAN_PARSE_H__
#define __CAN_PARSE_H__

//_____ I N C L U D E S _______________________________________________________
#include "can_main.h"

//_____ D E F I N I T I O N S __________________________________________________
int can_make_output(char *buffer, struct can_frame *m);
int can_parse_input_idle(char *buffer);
int can_parse_input_message(char *buffer, struct can_frame *Message);
int can_parse_input_init(char *buffer, TPBTR0BTR1 *Init);

#endif // __CAN_PARSE_H__

