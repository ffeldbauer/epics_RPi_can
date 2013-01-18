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
//! @brief   all about CAN message filtering
//!
//! @version 2.0.0
//******************************************************************************

#ifndef __CAN_FILTER_H__
#define __CAN_FILTER_H__

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>

//_____ D E F I N I T I O N S __________________________________________________
void *can_create_filter_chain(void); // returns a handle pointer
int   can_add_filter(void *handle, u32 FromID, u32 ToID, u8 MSGTYPE);
void  can_delete_filter_all(void *handle);
int   can_do_filter(void *handle, u32 can_id); // take the netdev can_id as input
void  can_delete_filter_chain(void *handle);

#endif // __CAN_FILTER_H__

//******************************************************************************
//! EOF
//******************************************************************************

