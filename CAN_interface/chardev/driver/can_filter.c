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
// brief   all about CAN message filtering
//
// version 2.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "can_filter.h"
#include "rpi_can.h"

//_____ D E F I N I T I O N S __________________________________________________
#define CAN_MAX_FILTER_PER_CHAIN 8  // take care since it is user allocated

typedef struct filter_element {
  struct list_head list;   // anchor for a linked list of filter_elements
  u32 FromID;              // all messages lower than FromID are rejected
  u32 ToID;                // all messages higher than ToID are rejected
  u8  MSGTYPE;             // MSGTYPE_STANDARD excludes MSGTYPE_EXTENDED
                           // MSGTYPE_RTR excludes all non RTR messages
} filter_element;

typedef struct filter_chain {
  struct list_head anchor; // anchor for a linked list of filter_elements
  int    count;            // counts the number of filters in this chain
  spinlock_t lock;         // mutual exclusion lock for this filter chain
} filter_chain;

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @fn      can_create_filter_chain
//!
//! @brief   create the base for a list of filters - returns a handle
//------------------------------------------------------------------------------
void *can_create_filter_chain( void ) {
  struct filter_chain *chain = NULL;

  // alloc a new filter_element
  chain = (struct filter_chain *)kmalloc(sizeof(struct filter_chain), GFP_KERNEL);

  if (!chain)
    printk(KERN_ERR "%s: Cant't create filter chain!\n", DEVICE_NAME);
  else {
    INIT_LIST_HEAD(&chain->anchor);
    chain->count = -1;       // initial no blocking of messages to provide compatibilty
    spin_lock_init( &chain->lock );
  }

  return (void*)chain;
}

//------------------------------------------------------------------------------
//! @fn      can_add_filter
//!
//! @brief   add a filter element to the filter chain pointed by handle
//!
//! @return: 0 if it is OK, else return error
//------------------------------------------------------------------------------
int can_add_filter( void *handle, u32 FromID, u32 ToID, u8 MSGTYPE ) {
  struct filter_chain   *chain = (struct filter_chain *)handle;
  struct filter_element *pfilter;
  struct list_head      *ptr, *tmp;
  unsigned long flags;

  // if chain isn't set ignore it
  if (chain) {

    // test for doubly set entries
    list_for_each_safe(ptr, tmp, &chain->anchor) {
      pfilter = list_entry(ptr, struct filter_element, list);
      if ((pfilter->FromID == FromID) && (pfilter->ToID == ToID) && (pfilter->MSGTYPE == MSGTYPE))
        return 0;
    }

    // limit count of filters since filters are user allocated
    if (chain->count >= CAN_MAX_FILTER_PER_CHAIN)
      return -ENOMEM;

    // alloc a new filter_element
    if ((pfilter = (struct filter_element *)kmalloc(sizeof(struct filter_element), GFP_KERNEL)) == NULL)
    {
      printk(KERN_ERR "%s: Cant't create filter element!\n", DEVICE_NAME);
      return -ENOMEM;
    }

    // init filter element
    pfilter->FromID  = FromID;
    pfilter->ToID    = ToID;
    pfilter->MSGTYPE = MSGTYPE;

    // add this entry to chain
    spin_lock_irqsave( &chain->lock, flags );
    list_add_tail( &pfilter->list, &chain->anchor );
    if (chain->count < 0) // get first start for compatibility mode
      chain->count = 1;
    else
      chain->count++;
    spin_unlock_irqrestore( &chain->lock, flags );
  }

  return 0;
}

//------------------------------------------------------------------------------
//! @fn      can_delete_filter_all
//!
//! @brief   delete all filter elements in the filter chain pointed by handle
//------------------------------------------------------------------------------
void can_delete_filter_all( void *handle ) {
  struct filter_chain   *chain = (struct filter_chain *)handle;
  struct filter_element *pfilter;
  struct list_head      *ptr, *tmp;
  unsigned long flags;

  if (chain) {

    spin_lock_irqsave( &chain->lock, flags );
    list_for_each_safe( ptr, tmp, &chain->anchor ) {
      pfilter = list_entry(ptr, struct filter_element, list);
      list_del(ptr);
      kfree(pfilter);
    }
    chain->count = 0;
    spin_unlock_irqrestore( &chain->lock, flags );
  }
}

//------------------------------------------------------------------------------
//! @fn      can_do_filter
//!
//! @brief   do the filtering with all filter elements pointed by handle
//!
//! @return: 0 when the message should be passed
//------------------------------------------------------------------------------
int can_do_filter(void *handle, u32 can_id) {
  struct filter_chain   *chain = (struct filter_chain *)handle;
  struct filter_element *pfilter;
  struct list_head      *ptr, *tmp;

  // pass always when no filter reset has been done before
  if ( ( !chain ) || ( chain->count <= 0 ) )
    return 0;

  // status is always passed
  if ( can_id & CAN_ERR_FLAG )
    return 0;

  {
    int throw;
    u32 rtr_message = can_id & CAN_RTR_FLAG;
    u32 ext_message = can_id & CAN_EFF_FLAG;

    if (ext_message)
      can_id &= CAN_EFF_MASK;
    else
      can_id &= CAN_SFF_MASK;

    list_for_each_safe( ptr, tmp, &chain->anchor ) {
      pfilter = list_entry(ptr, struct filter_element, list);

      #define RTR_FILTER (pfilter->MSGTYPE & MSGTYPE_RTR     )
      #define EXT_FILTER (pfilter->MSGTYPE & MSGTYPE_EXTENDED)
      #define RTR_IN     (rtr_message)
      #define EXT_IN     (ext_message)
      // truth table for throw
      //
      //             RTR_FILTER | /RTR_FILTER
      //            ------------|------------
      //  EXT_FILTER|  1  |  0  |  0  |  0  |/EXT_IN
      //            |-----------|--------------
      //            |  1  |  0  |  0  |  0  |
      //         ---------------------------| EXT_IN
      //            |  1  |  1  |  1  |  1  |
      //            |-----------|--------------
      // /EXT_FILTER|  1  |  0  |  0  |  0  |/EXT_IN
      //            |-----|-----------|------
      //           /RTR_IN|  RTR_IN   |/RTR_IN
      //
      throw = ((RTR_FILTER && !RTR_IN) || (!EXT_FILTER && EXT_IN));

      if ((!throw) && (can_id >= pfilter->FromID) && (can_id <= pfilter->ToID))
        return 0;
    }
  }

  // no pass criteria was found
  return 1;
}

//------------------------------------------------------------------------------
//! @fn      can_delete_filter_chain
//!
//! @brief   remove the whole filter chain (and potential filter elements)
//------------------------------------------------------------------------------
void can_delete_filter_chain(void *handle) {
  struct filter_chain *chain = (struct filter_chain *)handle;

  can_delete_filter_all(handle);

  if (chain)
    kfree(chain);
}

//******************************************************************************
//! EOF
//******************************************************************************
