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
//! @brief   Main part of driver. Defines fops, init and release funcitons
//!
//! @version 1.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#include "can_main.h"
#include "can_sja1000.h"
#include "can_gpio.h"
#include "can_fifo.h"
#include "can_parse.h"
#include "can_filter.h"

//_____ D E F I N I T I O N S __________________________________________________
MODULE_AUTHOR( "florian@ep1.ruhr-uni-bochum.de" );
MODULE_DESCRIPTION( "Driver for RaspberryPi CAN interface." );
MODULE_VERSION( CURRENT_RELEASE );
MODULE_SUPPORTED_DEVICE( "PANDA Raspberry Pi CAN Extension Board" );
MODULE_LICENSE( "GPL" );

#define MAX_WAIT_UNTIL_CLOSE 1000

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________
static struct candev *dev = NULL;
static struct class *can_drv_class = NULL;
static u8  can_drv_initStep = 0;
static int can_drv_major = 0;
static int can_drv_minor = 0;

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @fn      wait_until_fifo_empty
//!
//! @brief   wait until write fifo is empty, max time in msec
//------------------------------------------------------------------------------
static void wait_until_fifo_empty( u32 mTime ) {
  u32 dwStart = jiffies_to_msecs(jiffies);

  while( !atomic_read( &dev->DataSendReady ) && ( ( jiffies_to_msecs( jiffies ) - dwStart ) < mTime ) )
    schedule();

  // force it
  atomic_set(&dev->DataSendReady, 1);
}

//------------------------------------------------------------------------------
//! @fn      can_open
//!
//! @brief   File Operations: is called when the path is opened
//!
//! @return: 0 on success otherwise -ERRNO
//------------------------------------------------------------------------------
static int can_open( struct inode *inode, struct file *filep ) {
  int err = 0;
  struct fileobj* fobj = (struct fileobj*)NULL;

  if( !dev )
    return -ENODEV;

  // create file object
  fobj = kmalloc(sizeof(struct fileobj), GFP_KERNEL);
  if (!fobj) {
    printk( KERN_ERR "%s: can't allocate kernel memory!\n", DEVICE_NAME );
    return -ENOMEM;
  }

  // fill file object and init read and write method buffers
  if ( filep->f_mode & FMODE_READ ) {
    fobj->nReadRest = 0;
    fobj->nTotalReadCount = 0;
    fobj->pcReadPointer = fobj->pcReadBuffer;
  }

  if (filep->f_mode & FMODE_WRITE) {
    fobj->nWriteCount = 0;
    fobj->pcWritePointer = fobj->pcWriteBuffer;
  }

  filep->private_data = (void *)fobj;

  // only the frist open to this device makes a default init on this device
  if ( !dev->nOpenPaths ) {
    err = can_fifo_reset( &dev->writeFifo );
    if (err)
      return err;
    err = can_fifo_reset( &dev->readFifo );
    if (err)
      return err;

    err = can_gpio_req_irq( dev );
    if (err) {
      printk( KERN_ERR "%s: Cannot request irq %d! Error: %d\n", DEVICE_NAME, dev->wIrq, err );
      return err;
    }

    // open the device itself
    err = sja1000_open( dev, dev->wBTR0BTR1 );
    if (err) {
      printk( KERN_ERR "%s: can't open device hardware itself!\n", DEVICE_NAME );
      return err;
    }
  }
  dev->nOpenPaths++;

  if ( err && fobj ) /* oops */
    kfree(fobj);

  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_release
//!
//! @brief   File Operations: is called when the path is closed
//!
//! @return: 0
//------------------------------------------------------------------------------
static int can_release( struct inode *inode, struct file *filep ) {
  struct fileobj *fobj = (struct fileobj *)filep->private_data;

  // free the associated irq and allocated memory
  if ( fobj ){

    if ( dev->nOpenPaths > 1 )
      dev->nOpenPaths--;
    else {

      wait_until_fifo_empty( MAX_WAIT_UNTIL_CLOSE );
      // release the device itself
      sja1000_release( dev );

      // release the interface depended irq, after this 'dev' is not valid
      can_gpio_free_irq( dev );
    }

    kfree(fobj);
  }
  return 0;
}

//------------------------------------------------------------------------------
//! @fn      can_read
//!
//! @brief   File Operations: is called when read from path
//!
//! @return:
//------------------------------------------------------------------------------
static ssize_t can_read( struct file *filep, char *buf, size_t count, loff_t *f_pos ) {
  int    err;
  int    len = 0;
  struct can_frame msg;
  struct fileobj *fobj = (struct fileobj *)filep->private_data;

  // if the device is plugged out
  if ( !dev->ucPhysicallyInstalled )
    return -ENODEV;

  if ( fobj->nReadRest <= 0 ) {
    // support nonblocking read if requested
    if ( ( filep->f_flags & O_NONBLOCK ) && ( can_fifo_empty( &dev->readFifo ) ) )
      return -EAGAIN;

    // sleep until data are available
    err = wait_event_interruptible( dev->read_queue, ( !can_fifo_empty( &dev->readFifo ) ) );
    if (err)
      return err;

    // get read data out of FIFO
    err = can_fifo_get( &dev->readFifo, &msg );

    if (err)
      return err;

    fobj->nReadRest = can_make_output( fobj->pcReadBuffer, &msg );
    fobj->pcReadPointer = fobj->pcReadBuffer;
  }

  // give the data to the user
  if (count > fobj->nReadRest) {
    // put all data to user
    len = fobj->nReadRest;
    fobj->nReadRest = 0;
    if( copy_to_user( buf, fobj->pcReadPointer, len ) )
      return -EFAULT;
    fobj->pcReadPointer = fobj->pcReadBuffer;
  } else {
    // put only partial data to user
    len = count;
    fobj->nReadRest -= count;
    if( copy_to_user( buf, fobj->pcReadPointer, len ) )
      return -EFAULT;
    fobj->pcReadPointer = (u8 *)((u8*)fobj->pcReadPointer + len);
  }

  *f_pos += len;
  fobj->nTotalReadCount += len;

  return len;
}

//------------------------------------------------------------------------------
//! @fn      can_write
//!
//! @brief   File Operations: is called when write to path
//!
//! @return:
//------------------------------------------------------------------------------
static ssize_t can_write( struct file *filep, const char *buf, size_t count, loff_t *f_pos ) {
  int err = 0;
  u32 dwRest;
  u8 *ptr;

  struct fileobj *fobj = (struct fileobj *)filep->private_data;

  // if the device is plugged out
  if (!dev->ucPhysicallyInstalled)
    return -ENODEV;

  // calculate remaining buffer space
  dwRest = WRITEBUFFER_SIZE - (fobj->pcWritePointer - fobj->pcWriteBuffer); // nRest > 0!
  count  = (count > dwRest) ? dwRest : count;

  if (copy_from_user(fobj->pcWritePointer, buf, count))
    return -EFAULT;

  // adjust working pointer to end
  fobj->pcWritePointer += count;

  // iterate search blocks ending with '\n'
  while (1) {
    // search first '\n' from begin of buffer
    ptr = fobj->pcWriteBuffer;
    while ((*ptr != '\n') && (ptr < fobj->pcWritePointer))
      ptr++;

    // parse input when a CR was found
    if ((*ptr == '\n') && (ptr < fobj->pcWritePointer)) {
      u32 amount = (u32)(fobj->pcWritePointer - ptr - 1);
      u32 offset = (u32)(ptr - fobj->pcWriteBuffer + 1);

      if ((amount > WRITEBUFFER_SIZE) || (offset > WRITEBUFFER_SIZE)) {
        printk( KERN_ERR "%s: fault in can_write() %zu %u, %u: \n", DEVICE_NAME, count, amount, offset );
        return -EFAULT;
      }

      if ( can_parse_input_idle(fobj->pcWriteBuffer) ) {
        struct can_frame msg;

        if ( can_parse_input_message(fobj->pcWriteBuffer, &msg) ) {
          TPBTR0BTR1 Init;

          if( ( err = can_parse_input_init( fobj->pcWriteBuffer, &Init ) ) )
            return err;
          else {
            // init the associated chip and the fifos again with new parameters
            Init.wBTR0BTR1 = sja1000_bitrate( Init.dwBitRate );
            if (!Init.wBTR0BTR1) {
              err = -EFAULT;
              return err;
            }
            err = sja1000_open( dev, Init.wBTR0BTR1 );
            if (err)
              return err;
            else {
              dev->wBTR0BTR1 = Init.wBTR0BTR1;
            }

            err = can_fifo_reset( &dev->writeFifo );
            if (err)
              return err;

            err = can_fifo_reset( &dev->readFifo );
            if (err)
              return err;
          }
        } else {

          // support nonblocking write if requested
          if ((filep->f_flags & O_NONBLOCK) && (!can_fifo_not_full(&dev->writeFifo)) && (!atomic_read(&dev->DataSendReady)))
            return -EAGAIN;

          // sleep until space is available
          err = wait_event_interruptible(dev->write_queue,
                                         (can_fifo_not_full(&dev->writeFifo) || atomic_read(&dev->DataSendReady)));
          if (err)
            return err;

          // if the device is plugged out
          if (!dev->ucPhysicallyInstalled)
            return -ENODEV;

          err = can_fifo_put(&dev->writeFifo, &msg);
          if (err)
            return err;

          // push a new transmission trough ioctl() only if interrupt triggered push was stalled
          mb();
          if (atomic_read(&dev->DataSendReady)) {
            atomic_set(&dev->DataSendReady, 0);
            mb();
            if ( ( err = sja1000_write( dev ) ) ) {
              atomic_set(&dev->DataSendReady, 1);

              // ignore only if the fifo is already empty
              if (err != -ENODATA)
                return err;
            }
          } else {
            printk(KERN_DEBUG "%s: pushed %d items into Fifo\n", DEVICE_NAME, dev->writeFifo.nStored);
          }
        }
      }

      // move rest of amount data in buffer offset steps to left
      memmove(fobj->pcWriteBuffer, ptr + 1, amount);
      fobj->pcWritePointer -= offset;
    }
    else
      break; // no CR found
  }

  if (fobj->pcWritePointer >= (fobj->pcWriteBuffer + WRITEBUFFER_SIZE)) {
    fobj->pcWritePointer = fobj->pcWriteBuffer; // reject all
    return -EFAULT;
  }

  return count;
}

//------------------------------------------------------------------------------
//! @fn      can_poll
//!
//! @brief   File Operations: is called when poll or select path
//!
//! @return:
//------------------------------------------------------------------------------
static unsigned int can_poll( struct file *filep, poll_table *wait ) {
  struct fileobj *fobj = (struct fileobj *)filep->private_data;
  unsigned int mask = 0;
  if ( fobj ){

    poll_wait( filep, &dev->read_queue,  wait );
    poll_wait( filep, &dev->write_queue, wait );

    // return on ops that could be performed without blocking
    if ( !can_fifo_empty( &dev->readFifo ) )    mask |= ( POLLIN  | POLLRDNORM );
    if ( can_fifo_not_full( &dev->writeFifo ) ) mask |= ( POLLOUT | POLLWRNORM );
  }
  return mask;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_read
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_READ_MSG
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_read( struct file *filep, struct candev *dev, struct can_frame *usr ) {
  int err = 0;
  struct can_frame msg;

  // support nonblocking read if requested
  if( ( filep->f_flags & O_NONBLOCK ) && ( can_fifo_empty( &dev->readFifo ) ) )
    return -EAGAIN;

  // sleep until data are available
  err = wait_event_interruptible( dev->read_queue, ( !can_fifo_empty( &dev->readFifo ) ) );

  if (err)
    goto fail;

  // if the device is plugged out
  if( !dev->ucPhysicallyInstalled )
    return -ENODEV;

  // get data out of fifo
  err = can_fifo_get( &dev->readFifo, (void *)&msg );
  if( err )
    goto fail;

  if( copy_to_user( usr, &msg, sizeof(*usr) ) )
    err = -EFAULT;

 fail:
  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_write
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_WRITE_MSG
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_write( struct file *filep, struct candev *dev, struct can_frame *usr ) {
  int err = 0;
  struct can_frame msg;

  // support nonblocking write if requested
  if( ( filep->f_flags & O_NONBLOCK ) && ( !can_fifo_not_full( &dev->writeFifo ) ) && ( !atomic_read( &dev->DataSendReady ) ) )
    return -EAGAIN;

  // sleep until space is available
  err = wait_event_interruptible( dev->write_queue,
                                  ( can_fifo_not_full( &dev->writeFifo ) || atomic_read( &dev->DataSendReady ) ) );
  if( err )
    goto fail;

  // if the device is plugged out
  if( !dev->ucPhysicallyInstalled )
    return -ENODEV;

  // get from user space
  if( copy_from_user( &msg, usr, sizeof(msg) ) ) {
    err = -EFAULT;
    goto fail;
  }

  // put data into fifo
  err = can_fifo_put( &dev->writeFifo, &msg );
  if (!err) {
    // push a new transmission trough ioctl() only if interrupt triggered push was stalled
    mb();
    if (atomic_read(&dev->DataSendReady)) {
      atomic_set(&dev->DataSendReady, 0);
      mb();
      err = sja1000_write( dev );
      if (err) {
        atomic_set( &dev->DataSendReady, 1 );

        // ignore only if the fifo is already empty
        if (err == -ENODATA)
          err = 0;
      }
    } else {
      // printk(KERN_DEBUG "%s: pushed %d items into Fifo\n", DEVICE_NAME, dev->writeFifo.nStored);
    }
  }

 fail:
  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_status
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_GET_STATUS
//!
//! @return:
//------------------------------------------------------------------------------
int can_ioctl_status(struct candev *dev, TPSTATUS *status) {
  int err = 0;
  TPSTATUS local;

  local.wErrorFlag = dev->wCANStatus;
  local.nPendingReads = dev->readFifo.nStored;
  // get infos for friends of polling operation
  if ( can_fifo_empty(&dev->readFifo) )
    local.wErrorFlag |= CAN_ERR_QRCVEMPTY;

  local.nPendingWrites = (dev->writeFifo.nStored + ((atomic_read(&dev->DataSendReady)) ? 0 : 1));

  if (!can_fifo_not_full(&dev->writeFifo))
    local.wErrorFlag |= CAN_ERR_QXMTFULL;

  local.nLastError = dev->nLastError;

  if (copy_to_user(status, &local, sizeof(local))) {
    err = -EFAULT;
    goto fail;
  }

  dev->wCANStatus = 0;
  dev->nLastError = 0;

 fail:
  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_diag
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_DIAG
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_diag(struct candev *dev, TPDIAG *diag) {
  int err = 0;
  TPDIAG local;

  local.dwReadCounter   = dev->readFifo.dwTotal;
  local.dwWriteCounter  = dev->writeFifo.dwTotal;
  local.dwIRQcounter    = dev->dwInterruptCounter;
  local.dwErrorCounter  = dev->dwErrorCounter;
  local.wErrorFlag      = dev->wCANStatus;

  // get infos for friends of polling operation
  if (can_fifo_empty(&dev->readFifo))
    local.wErrorFlag |= CAN_ERR_QRCVEMPTY;

  if (!can_fifo_not_full(&dev->writeFifo))
    local.wErrorFlag |= CAN_ERR_QXMTFULL;

  local.nLastError      = dev->nLastError;

  strncpy(local.szVersionString, CURRENT_RELEASE, VERSIONSTRING_LEN);

  if (copy_to_user(diag, &local, sizeof(local)))
    err = -EFAULT;

  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_BTR0BTR1
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_BITRATE
//!          change current bitrate
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_BTR0BTR1(struct candev *dev, TPBTR0BTR1 *BTR0BTR1) {
  int err = 0;
  TPBTR0BTR1 local;

  if (copy_from_user(&local, BTR0BTR1, sizeof(local))) {
    err = -EFAULT;
    return err;
  }

  // this does not influence hardware settings, only BTR0BTR1 values are calculated
  local.wBTR0BTR1 = sja1000_bitrate(local.dwBitRate);
  if (!local.wBTR0BTR1) {
    err = -EFAULT;
    return err;
  }
  err = sja1000_open( dev, local.wBTR0BTR1 );

  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_get_BTR0BTR1
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_GET_BITRATE
//!          get current bitrate
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_get_BTR0BTR1( struct candev *dev, TPBTR0BTR1 *BTR0BTR1 ) {
  int err = 0;
  TPBTR0BTR1 local;

  local.dwBitRate = sja1000_read_bitrate();
  switch ( local.dwBitRate ) {
  case   CAN_BAUD_1M: local.wBTR0BTR1 = 1000000; break;
  case CAN_BAUD_500K: local.wBTR0BTR1 =  500000; break;
  case CAN_BAUD_250K: local.wBTR0BTR1 =  250000; break;
  case CAN_BAUD_125K: local.wBTR0BTR1 =  125000; break;
  case CAN_BAUD_100K: local.wBTR0BTR1 =  100000; break;
  case  CAN_BAUD_50K: local.wBTR0BTR1 =   50000; break;
  case  CAN_BAUD_20K: local.wBTR0BTR1 =   20000; break;
  case  CAN_BAUD_10K: local.wBTR0BTR1 =   10000; break;
  case   CAN_BAUD_5K: local.wBTR0BTR1 =    5000; break;
  }

  if (copy_to_user(BTR0BTR1, &local, sizeof(local))) {
    err = -EFAULT;
  }

  return err;
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl_msg_filter
//!
//! @brief   File Operations: is called on user ioctl() call
//!                           with cmd = CAN_MSG_FILTER
//!          add a message filter_element into the filter chain or delete
//!          all filter_elements
//!
//! @return:
//------------------------------------------------------------------------------
static int can_ioctl_msg_filter(struct candev *dev, TPMSGFILTER *filter) {
  TPMSGFILTER local_filter;

  // filter == NULL -> delete the filter_elements in the chain
  if (!filter) {
    can_delete_filter_all( dev->filter );
    return 0;
  }

  if (copy_from_user(&local_filter, filter, sizeof(local_filter)))
    return -EFAULT;

  return can_add_filter( dev->filter, local_filter.FromID, local_filter.ToID, local_filter.MSGTYPE );
}

//------------------------------------------------------------------------------
//! @fn      can_ioctl
//!
//! @brief   File Operations: is called on user ioctl() call
//!
//! @return:
//------------------------------------------------------------------------------
long can_ioctl( struct file *filep, unsigned int cmd, unsigned long arg ) {
  int err;
  struct fileobj *fobj = (struct fileobj *)filep->private_data;

  // if the device is plugged out
  if ( !fobj || !dev->ucPhysicallyInstalled )
    return -ENODEV;

  switch(cmd) {
  case CAN_READ_MSG:
    err = can_ioctl_read(filep, dev, (struct can_frame *)arg); // support blocking and nonblocking IO
    break;
  case CAN_WRITE_MSG:
    err = can_ioctl_write(filep, dev, (struct can_frame *)arg);  // support blocking and nonblocking IO
    break;
  case CAN_GET_STATUS:
    err = can_ioctl_status(dev, (TPSTATUS *)arg);
    break;
  case CAN_DIAG:
    err = can_ioctl_diag(dev, (TPDIAG *)arg);
    break;
  case CAN_BITRATE:
    err = can_ioctl_BTR0BTR1(dev, (TPBTR0BTR1 *)arg);
    break;
  case CAN_MSG_FILTER:
    err = can_ioctl_msg_filter(dev, (TPMSGFILTER *)arg);
    break;

  default:
    err = -ENOTTY;
    break;
  }

  return err;
}

//------------------------------------------------------------------------------
//! @var     can_fops
//!
//! @brief   Struct containing method pointer to file operations
//------------------------------------------------------------------------------
struct file_operations can_fops = {
 owner:          THIS_MODULE,
 open:           can_open,
 release:        can_release,
 read:           can_read,
 write:          can_write,
 unlocked_ioctl: can_ioctl,
 poll:           can_poll,
};

//------------------------------------------------------------------------------
//! @fn      init_module
//!
//! @brief   Initialize the module. is called when the device is installed
//           'insmod canbus.ko'
//!
//! @return: 0 on success otherwise -ERRNO
//------------------------------------------------------------------------------
int init_module( void ) {
  int result = 0;

  printk( KERN_INFO "%s: %s ", DEVICE_NAME, CURRENT_RELEASE );
#if defined(__BIG_ENDIAN)
  printk("(be)\n");
#elif defined(__LITTLE_ENDIAN)
  printk("(le)\n");
#else
#error Endian not set
#endif

  dev = (struct candev*)kmalloc( sizeof( struct candev ), GFP_KERNEL );
  if ( NULL == dev ) {
    result = -ENOMEM;
    goto fail;
  }

  // register the driver by the OS
  result = register_chrdev( can_drv_major, DEVICE_NAME, &can_fops );
  if (result < 0)
    goto fail;
  else
    if ( !can_drv_major )
      can_drv_major = result;

  can_drv_class = class_create( THIS_MODULE , DEVICE_NAME );

  can_drv_initStep = 1;

  dev->nLastError       = 0;
  dev->busStatus        = CAN_ERROR_ACTIVE;
  dev->dwErrorCounter   = 0;
  dev->dwInterruptCounter = 0;
  dev->wCANStatus       = 0;
  dev->wBTR0BTR1        = CAN_BAUD_125K;
  dev->ucPhysicallyInstalled = 0;  // assume the device is not installed
  dev->ucActivityState       = ACTIVITY_NONE;
  dev->filter           = can_create_filter_chain();

  atomic_set( &dev->DataSendReady, 1 );
  // init fifos

  can_fifo_init( &dev->readFifo,  &dev->rMsg[0], &dev->rMsg[READ_MESSAGE_COUNT - 1],  READ_MESSAGE_COUNT,  sizeof( struct can_frame ) );
  can_fifo_init( &dev->writeFifo, &dev->wMsg[0], &dev->wMsg[WRITE_MESSAGE_COUNT - 1], WRITE_MESSAGE_COUNT, sizeof( struct can_frame ) );

  spin_lock_init( &dev->wlock );
  spin_lock_init( &dev->isr_lock );

  result = can_gpio_init( dev );
  if ( result )
    goto fail;

  can_drv_initStep = 2;

  device_create( can_drv_class, NULL, MKDEV( can_drv_major, can_drv_minor ), NULL, "canbus%d", can_drv_minor);

  can_drv_initStep = 3;

  printk( KERN_INFO "%s: major %d.\n", DEVICE_NAME, can_drv_major );
  return 0; // succeed

 fail:
  cleanup_module();
  return result;
}

//------------------------------------------------------------------------------
//! @fn      cleanup_module
//!
//! @brief   Release the module. is called when the device is removed
//           'rmmod canbus.ko'
//------------------------------------------------------------------------------
void cleanup_module( void ) {
  switch( can_drv_initStep ) {
  case 3: device_destroy( can_drv_class, MKDEV( can_drv_major, can_drv_minor ) );
  case 2: can_gpio_free();
    can_delete_filter_chain( dev->filter );
    class_destroy( can_drv_class );
    unregister_chrdev( can_drv_major, DEVICE_NAME );
    kfree( dev );
  }

  printk( KERN_INFO "%s: removed.\n", DEVICE_NAME );
}

//******************************************************************************
//! EOF
//******************************************************************************
