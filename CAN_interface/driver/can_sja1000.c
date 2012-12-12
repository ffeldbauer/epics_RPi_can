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

//_____ I N C L U D E S _______________________________________________________
#include <asm/errno.h>
#include <asm/byteorder.h>  // because of little / big endian
#include <linux/sched.h>

#include "can_main.h"
#include "can_sja1000.h"
#include "can_gpio.h"
#include "can_fifo.h"
#include "can_filter.h"

//_____ D E F I N I T I O N S __________________________________________________
// sja1000 registers, only PELICAN mode - TUX like it
#define MODE                   0      // mode register
#define COMMAND                1
#define CHIPSTATUS             2
#define INTERRUPT_STATUS       3
#define INTERRUPT_ENABLE       4      // acceptance code
#define TIMING0                6      // bus timing 0
#define TIMING1                7      // bus timing 1
#define OUTPUT_CONTROL         8      // output control
#define TESTREG                9

#define ARBIT_LOST_CAPTURE    11      // transmit buffer: Identifier
#define ERROR_CODE_CAPTURE    12      // RTR bit und data length code
#define ERROR_WARNING_LIMIT   13      // start byte of data field
#define RX_ERROR_COUNTER      14
#define TX_ERROR_COUNTER      15

#define ACCEPTANCE_CODE_BASE  16
#define RECEIVE_FRAME_BASE    16
#define TRANSMIT_FRAME_BASE   16

#define ACCEPTANCE_MASK_BASE  20

#define RECEIVE_MSG_COUNTER   29
#define RECEIVE_START_ADDRESS 30

#define CLKDIVIDER            31      // set bit rate and pelican mode

// important sja1000 register contents, MODE register
#define SLEEP_MODE             0x10
#define ACCEPT_FILTER_MODE     0x08
#define SELF_TEST_MODE         0x04
#define LISTEN_ONLY_MODE       0x02
#define RESET_MODE             0x01
#define NORMAL_MODE            0x00

// COMMAND register
#define CLEAR_DATA_OVERRUN     0x08
#define RELEASE_RECEIVE_BUFFER 0x04
#define ABORT_TRANSMISSION     0x02
#define TRANSMISSION_REQUEST   0x01

// CHIPSTATUS register
#define BUS_STATUS             0x80
#define ERROR_STATUS           0x40
#define TRANSMIT_STATUS        0x20
#define RECEIVE_STATUS         0x10
#define TRANS_COMPLETE_STATUS  0x08
#define TRANS_BUFFER_STATUS    0x04
#define DATA_OVERRUN_STATUS    0x02
#define RECEIVE_BUFFER_STATUS  0x01

// INTERRUPT STATUS register
#define BUS_ERROR_INTERRUPT    0x80
#define ARBIT_LOST_INTERRUPT   0x40
#define ERROR_PASSIV_INTERRUPT 0x20
#define WAKE_UP_INTERRUPT      0x10
#define DATA_OVERRUN_INTERRUPT 0x08
#define ERROR_WARN_INTERRUPT   0x04
#define TRANSMIT_INTERRUPT     0x02
#define RECEIVE_INTERRUPT      0x01

// INTERRUPT ENABLE register
#define BUS_ERROR_INTERRUPT_ENABLE    0x80
#define ARBIT_LOST_INTERRUPT_ENABLE   0x40
#define ERROR_PASSIV_INTERRUPT_ENABLE 0x20
#define WAKE_UP_INTERRUPT_ENABLE      0x10
#define DATA_OVERRUN_INTERRUPT_ENABLE 0x08
#define ERROR_WARN_INTERRUPT_ENABLE   0x04
#define TRANSMIT_INTERRUPT_ENABLE     0x02
#define RECEIVE_INTERRUPT_ENABLE      0x01

// OUTPUT CONTROL register
#define OUTPUT_CONTROL_TRANSISTOR_P1  0x80
#define OUTPUT_CONTROL_TRANSISTOR_N1  0x40
#define OUTPUT_CONTROL_POLARITY_1     0x20
#define OUTPUT_CONTROL_TRANSISTOR_P0  0x10
#define OUTPUT_CONTROL_TRANSISTOR_N0  0x08
#define OUTPUT_CONTROL_POLARITY_0     0x04
#define OUTPUT_CONTROL_MODE_1         0x02
#define OUTPUT_CONTROL_MODE_0         0x01

// TRANSMIT or RECEIVE BUFFER
#define BUFFER_EFF                    0x80 // set for 29 bit identifier
#define BUFFER_RTR                    0x40 // set for RTR request
#define BUFFER_DLC_MASK               0x0f

// CLKDIVIDER register
#define CAN_MODE                      0x80
#define CAN_BYPASS                    0x40
#define RXINT_OUTPUT_ENABLE           0x20
#define CLOCK_OFF                     0x08
#define CLOCK_DIVIDER_MASK            0x07

// additional informations
#define CLOCK_HZ                  16000000 // crystal frequency

// time for mode register to change mode
#define MODE_REGISTER_SWITCH_TIME 100 // msec

// some CLKDIVIDER register contents, hardware architecture dependend
#define PELICAN_SINGLE  (CAN_MODE | CAN_BYPASS | 0x07 | CLOCK_OFF)

// hardware depended setup for OUTPUT_CONTROL register
#define OUTPUT_CONTROL_SETUP (OUTPUT_CONTROL_TRANSISTOR_P0 | OUTPUT_CONTROL_TRANSISTOR_N0 | OUTPUT_CONTROL_MODE_1)

// the interrupt enables
#define INTERRUPT_ENABLE_SETUP (RECEIVE_INTERRUPT_ENABLE | TRANSMIT_INTERRUPT_ENABLE | DATA_OVERRUN_INTERRUPT_ENABLE | BUS_ERROR_INTERRUPT_ENABLE | ERROR_PASSIV_INTERRUPT_ENABLE | ERROR_WARN_INTERRUPT_ENABLE)

// the maximum number of handled messages in one interrupt
#define MAX_MESSAGES_PER_INTERRUPT 8

// the maximum number of handled sja1000 interrupts in 1 handler entry
//#define MAX_INTERRUPTS_PER_ENTRY   4

// constants from Arnaud Westenberg email:arnaud@wanadoo.nl
#define MAX_TSEG1  15
#define MAX_TSEG2  7
#define BTR1_SAM   (1<<1)

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

//_____ F U N C T I O N S ______________________________________________________

//----------------------------------------------------------------------------
// put received CAN frame into chardev receive FIFO
static int can_rx( struct candev* dev, struct can_frame* cf ) {
  int result = 0;

  if ( !can_do_filter( dev->filter, cf->can_id ) ) {
    // step forward in fifo
    result = can_fifo_put( &dev->readFifo, cf );
    // flag to higher layers that a message was put into fifo or an error occurred
    result = (result) ? result : 1;
  }
  return result;
}

//------------------------------------------------------------------------------
//! @fn      guarded_write_command
//!
//! @brief   guards writing sja1000's command register in multicore environments
//------------------------------------------------------------------------------
static inline void guarded_write_command( struct candev* dev, u8 data ) {
  unsigned long flags;
  spin_lock_irqsave( &dev->wlock, flags );
  can_gpio_writereg( COMMAND, data );
  can_gpio_readreg( CHIPSTATUS );
  spin_unlock_irqrestore( &dev->wlock, flags );
}

//------------------------------------------------------------------------------
//! @fn      set_reset_mode
//!
//! @brief   switches the chip into reset mode
//!
//! @return: 0 on success otherwise -EIO
//------------------------------------------------------------------------------
static int set_reset_mode( void ) {
  u32 dwStart = jiffies_to_msecs(jiffies);
  u8  tmp;

  tmp = can_gpio_readreg( MODE );
  while ( !( tmp & RESET_MODE ) && ( ( jiffies_to_msecs(jiffies) - dwStart ) < MODE_REGISTER_SWITCH_TIME ) ) {
    can_gpio_writereg( MODE, RESET_MODE ); // force into reset mode
    wmb();
    udelay(1);
    tmp = can_gpio_readreg( MODE );
  }

  if ( !( tmp & RESET_MODE ) )
    return -EIO;
  else
    return 0;
}

//------------------------------------------------------------------------------
//! @fn      set_normal_mode
//!
//! @brief   switches the chip back from reset mode
//!
//! @return: 0 on success otherwise -EIO
//------------------------------------------------------------------------------
static int set_normal_mode( void ) {
  u32 dwStart = jiffies_to_msecs(jiffies);
  u8  tmp;

  tmp = can_gpio_readreg( MODE );
  while ( ( tmp != NORMAL_MODE ) && ( ( jiffies_to_msecs(jiffies) - dwStart ) < MODE_REGISTER_SWITCH_TIME ) ) {
    can_gpio_writereg( MODE, NORMAL_MODE ); // force into normal mode
    wmb();
    udelay(1);
    tmp = can_gpio_readreg( MODE );
  }

  if ( tmp != NORMAL_MODE )
    return -EIO;
  else
    return 0;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_open
//!
//! @brief   init CAN chip
//!
//! @return: 0 on success otherwise -ERRNO
//------------------------------------------------------------------------------
int sja1000_open( struct candev* dev, u16 btr0btr1 ) {
  int result = 0;

  // switch to reset
  result = set_reset_mode();
  if ( result )
    return result;

  // take a fresh status
  dev->wCANStatus = 0;

  // configure clock divider register, switch into pelican mode, depended of of type
  can_gpio_writereg( CLKDIVIDER, PELICAN_SINGLE );

  // configure acceptance code registers
  can_gpio_writereg( ACCEPTANCE_CODE_BASE,     0 );
  can_gpio_writereg( ACCEPTANCE_CODE_BASE + 1, 0 );
  can_gpio_writereg( ACCEPTANCE_CODE_BASE + 2, 0 );
  can_gpio_writereg( ACCEPTANCE_CODE_BASE + 3, 0 );

  // configure all acceptance mask registers to don't care
  can_gpio_writereg( ACCEPTANCE_MASK_BASE,     0xff );
  can_gpio_writereg( ACCEPTANCE_MASK_BASE + 1, 0xff );
  can_gpio_writereg( ACCEPTANCE_MASK_BASE + 2, 0xff );
  can_gpio_writereg( ACCEPTANCE_MASK_BASE + 3, 0xff );

  // configure bus timing registers
  can_gpio_writereg( TIMING0, (u8)( (btr0btr1 >> 8) & 0xff ) );
  can_gpio_writereg( TIMING1, (u8)( (btr0btr1     ) & 0xff ) );

  // configure output control registers
  can_gpio_writereg( OUTPUT_CONTROL, OUTPUT_CONTROL_SETUP );

  // clear any pending interrupt
  can_gpio_readreg( INTERRUPT_STATUS );

  // enter normal operating mode
  result = set_normal_mode();
  if (result)
    return result;

  // enable CAN interrupts
  can_gpio_writereg( INTERRUPT_ENABLE, INTERRUPT_ENABLE_SETUP );

  return result;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_release
//!
//! @brief   release CAN chip
//------------------------------------------------------------------------------
void sja1000_release( struct candev* dev ) {
  // abort pending transmissions
  guarded_write_command( dev, ABORT_TRANSMISSION );

  // disable CAN interrupts and set chip in reset mode
  can_gpio_writereg( INTERRUPT_ENABLE, 0 );
  set_reset_mode();
}

//------------------------------------------------------------------------------
//! @fn      sja1000_read_bitrate
//!
//! @brief   get current bitrate
//------------------------------------------------------------------------------
u16 sja1000_read_bitrate( void ) {
  u8 btr0 = can_gpio_readreg( TIMING0 );
  u8 btr1 = can_gpio_readreg( TIMING1 );

  u16 wBTR0BTR1 = ( btr0 << 8 ) | btr1;
  return wBTR0BTR1;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_bitrate
//!
//! @brief   get BTR0BTR1 init values
//------------------------------------------------------------------------------
u16 sja1000_bitrate( u32 dwBitRate ) {
  u16 wBTR0BTR1 = 0;

  // get default const values
  switch (dwBitRate) {
  case 1000000: wBTR0BTR1 = CAN_BAUD_1M;   break;
  case  500000: wBTR0BTR1 = CAN_BAUD_500K; break;
  case  250000: wBTR0BTR1 = CAN_BAUD_250K; break;
  case  125000: wBTR0BTR1 = CAN_BAUD_125K; break;
  case  100000: wBTR0BTR1 = CAN_BAUD_100K; break;
  case   50000: wBTR0BTR1 = CAN_BAUD_50K;  break;
  case   20000: wBTR0BTR1 = CAN_BAUD_20K;  break;
  case   10000: wBTR0BTR1 = CAN_BAUD_10K;  break;
  case    5000: wBTR0BTR1 = CAN_BAUD_5K;   break;
  case       0: wBTR0BTR1 = 0;             break;

  default:
    printk( KERN_ERR "%s: not supported bitrate: %d.\n", DEVICE_NAME, dwBitRate );
  }

  return wBTR0BTR1;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_read_frames
//!
//! @brief   read CAN-data from chip, supposed a message is available
//------------------------------------------------------------------------------
static int sja1000_read_frames( struct candev* dev ) {
  int msgs   = MAX_MESSAGES_PER_INTERRUPT;
  u8 fi;
  u8 dreg;
  u8 dlc;
  ULCONV localID;
  struct can_frame frame;

  int i;
  int result = 0;

  do {
    fi  = can_gpio_readreg( RECEIVE_FRAME_BASE );
    dlc = fi & BUFFER_DLC_MASK;

    if ( dlc > 8 )
      dlc = 8;

    if ( fi & BUFFER_EFF ) {
      /* extended frame format (EFF) */
      dreg = RECEIVE_FRAME_BASE + 5;

#if defined(__LITTLE_ENDIAN)
      localID.uc[3] = can_gpio_readreg( RECEIVE_FRAME_BASE + 1 );
      localID.uc[2] = can_gpio_readreg( RECEIVE_FRAME_BASE + 2 );
      localID.uc[1] = can_gpio_readreg( RECEIVE_FRAME_BASE + 3 );
      localID.uc[0] = can_gpio_readreg( RECEIVE_FRAME_BASE + 4 );
#elif defined(__BIG_ENDIAN)
      localID.uc[0] = can_gpio_readreg( RECEIVE_FRAME_BASE + 1 );
      localID.uc[1] = can_gpio_readreg( RECEIVE_FRAME_BASE + 2 );
      localID.uc[2] = can_gpio_readreg( RECEIVE_FRAME_BASE + 3 );
      localID.uc[3] = can_gpio_readreg( RECEIVE_FRAME_BASE + 4 );
#else
#error  "Please fix the endianness defines in <asm/byteorder.h>"
#endif

      frame.can_id = ( localID.ul >> 3 ) | CAN_EFF_FLAG;
    } else {
      /* standard frame format (SFF) */
      dreg = RECEIVE_FRAME_BASE + 3;

      localID.ul    = 0;
#if defined(__LITTLE_ENDIAN)
      localID.uc[3] = can_gpio_readreg( RECEIVE_FRAME_BASE + 1 );
      localID.uc[2] = can_gpio_readreg( RECEIVE_FRAME_BASE + 2 );
#elif defined(__BIG_ENDIAN)
      localID.uc[0] = can_gpio_readreg( RECEIVE_FRAME_BASE + 1 );
      localID.uc[1] = can_gpio_readreg( RECEIVE_FRAME_BASE + 2 );
#else
#error  "Please fix the endianness defines in <asm/byteorder.h>"
#endif

      frame.can_id = ( localID.ul >> 21 );

    }

    if (fi & BUFFER_RTR)
      frame.can_id |= CAN_RTR_FLAG;

    *(__u64 *) &frame.data[0] = (__u64) 0; /* clear aligned data section */

    for ( i = 0; i < dlc; i++ )
      frame.data[i] = can_gpio_readreg( dreg++ );

    frame.can_dlc = dlc;

    if ( ( i = can_rx( dev, &frame ) ) ) // put into specific data sink
      result = i;

    // Any error processing on result =! 0 here?
    // Indeed we have to read from the controller as long as we receive data to
    // unblock the controller. If we have problems to fill the CAN frames into
    // the receive queues, this cannot be handled inside the interrupt.

    // release the receive buffer
    guarded_write_command( dev, RELEASE_RECEIVE_BUFFER );

    // give time to settle
    udelay(1);

  } while ( can_gpio_readreg( CHIPSTATUS ) & RECEIVE_BUFFER_STATUS && ( msgs-- ) );

  return result;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_write_frame
//!
//! @brief   write CAN-data to chip
//------------------------------------------------------------------------------
static int sja1000_write_frame( struct candev* dev, struct can_frame* cf ) {
  u8 fi;
  u8 dlc;
  canid_t id;
  ULCONV localID;
  u8 dreg;
  int i;

  fi = dlc = cf->can_dlc;
  id = localID.ul = cf->can_id;

  if ( id & CAN_RTR_FLAG )
    fi |= BUFFER_RTR;

  if ( id & CAN_EFF_FLAG ) {
    dreg  = TRANSMIT_FRAME_BASE + 5;
    fi   |= BUFFER_EFF;

    localID.ul <<= 3;

    can_gpio_writereg( TRANSMIT_FRAME_BASE, fi );

#if defined(__LITTLE_ENDIAN)
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 1, localID.uc[3] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 2, localID.uc[2] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 3, localID.uc[1] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 4, localID.uc[0] );
#elif defined(__BIG_ENDIAN)
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 1, localID.uc[0] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 2, localID.uc[1] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 3, localID.uc[2] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 4, localID.uc[3] );
#else
#error  "Please fix the endianness defines in <asm/byteorder.h>"
#endif
  } else {
    dreg = TRANSMIT_FRAME_BASE + 3;

    localID.ul <<= 21;

    can_gpio_writereg( TRANSMIT_FRAME_BASE, fi );

#if defined(__LITTLE_ENDIAN)
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 1, localID.uc[3] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 2, localID.uc[2] );
#elif defined(__BIG_ENDIAN)
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 1, localID.uc[0] );
    can_gpio_writereg( TRANSMIT_FRAME_BASE + 2, localID.uc[1] );
#else
#error  "Please fix the endianness defines in <asm/byteorder.h>"
#endif
  }

  for ( i = 0; i < dlc; i++ ) {
    can_gpio_writereg( dreg++, cf->data[i] );
  }

  // request a transmission
  guarded_write_command( dev, TRANSMISSION_REQUEST );

  return 0;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_write
//!
//! @brief   write CAN-data from FIFO to chip
//------------------------------------------------------------------------------
int sja1000_write( struct candev* dev ) {
  int err = 0;
  unsigned long flags;
  struct can_frame cf;

  spin_lock_irqsave( &dev->isr_lock, flags);

  /* Check if Tx buffer is empty before writing on */
  if ( can_gpio_readreg( CHIPSTATUS ) & TRANS_BUFFER_STATUS ) {
    err = can_fifo_get( &dev->writeFifo, &cf );
    if ( !err ) sja1000_write_frame( dev, &cf );
  }

  spin_unlock_irqrestore( &dev->isr_lock, flags );

  return err;
}

//------------------------------------------------------------------------------
//! @fn      sja1000_irqhandler
//!
//! @brief   handle a interrupt request
//------------------------------------------------------------------------------
irqreturn_t can_sja1000_irqhandler( int irq, void *dev_id ) {
  struct candev* dev = (struct candev*)dev_id;

  int ret = IRQ_NONE;
  u8 irqstatus;
  int err;
  u16 rwakeup = 0;
  u16 wwakeup = 0;

  unsigned long flags;
  struct can_frame ef;

  memset( &ef, 0, sizeof(ef) );

  spin_lock_irqsave( &dev->isr_lock, flags );

  irqstatus = can_gpio_readreg( INTERRUPT_STATUS );
  if( irqstatus ) {

    dev->dwInterruptCounter++;
    err = 0;

    // quick hack to badly workaround write stall
    if ( irqstatus & TRANSMIT_INTERRUPT ) {
      // handle transmission
      if ( ( err = sja1000_write( dev ) ) ) {
        if ( err == -ENODATA )
          wwakeup++;
        else {
          dev->nLastError = err;
          dev->dwErrorCounter++;
          dev->wCANStatus |= CAN_ERR_QXMTFULL; // fatal error!
        }
      }
      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if ( irqstatus & DATA_OVERRUN_INTERRUPT ) {

      // handle data overrun
      dev->wCANStatus |= CAN_ERR_OVERRUN;
      ef.can_id  |= CAN_ERR_CRTL;
      ef.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
      rwakeup++;
      dev->dwErrorCounter++;

      guarded_write_command( dev, CLEAR_DATA_OVERRUN );

      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if ( irqstatus & RECEIVE_INTERRUPT ) {
      // handle receiption
      if ( ( err = sja1000_read_frames( dev ) ) < 0 ) {
        dev->nLastError = err;
        dev->dwErrorCounter++;
        dev->wCANStatus |=  CAN_ERR_QOVERRUN;

        // throw away last message which was refused by fifo
        guarded_write_command( dev, RELEASE_RECEIVE_BUFFER );
      }

      if (err > 0) // successfully enqueued into chardev FIFO
        rwakeup++;

      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    if ( irqstatus & (BUS_ERROR_INTERRUPT | ERROR_PASSIV_INTERRUPT | ERROR_WARN_INTERRUPT) ) {

      if ( irqstatus & (ERROR_PASSIV_INTERRUPT | ERROR_WARN_INTERRUPT) ) {
        u8 chipstatus = can_gpio_readreg( CHIPSTATUS );

        switch (chipstatus & (BUS_STATUS | ERROR_STATUS)) {
        case 0x00:
          // error active, clear only local status
          dev->busStatus = CAN_ERROR_ACTIVE;
          dev->wCANStatus &=  ~(CAN_ERR_BUSOFF | CAN_ERR_BUSHEAVY | CAN_ERR_BUSLIGHT);
          break;
        case BUS_STATUS:
        case BUS_STATUS | ERROR_STATUS:
          // bus-off
          dev->busStatus = CAN_BUS_OFF;
          dev->wCANStatus |=  CAN_ERR_BUSOFF;
          ef.can_id |= CAN_ERR_BUSOFF;
          break;
        case ERROR_STATUS:
          if (irqstatus & ERROR_PASSIV_INTERRUPT) {
            // either enter or leave error passive status
            if (dev->busStatus == CAN_ERROR_PASSIVE) {
              // go back to error active
              dev->busStatus = CAN_ERROR_ACTIVE;
              dev->wCANStatus &= ~(CAN_ERR_BUSOFF | CAN_ERR_BUSHEAVY);
              dev->wCANStatus |= CAN_ERR_BUSLIGHT;
            } else {
              // enter error passive state
              dev->busStatus = CAN_ERROR_PASSIVE;
              dev->wCANStatus &= ~CAN_ERR_BUSOFF;
              dev->wCANStatus |=  CAN_ERR_BUSHEAVY;
              ef.can_id  |= CAN_ERR_CRTL;
              ef.data[1] |= (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE);
            }
          } else {
            // it was a warning limit reached event
            dev->busStatus = CAN_ERROR_ACTIVE;
            dev->wCANStatus |=  CAN_ERR_BUSLIGHT;
            ef.can_id  |= CAN_ERR_CRTL;
            ef.data[1] |= (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING);
          }
          break;
        }
      }

      // count each error signal even if it does not change any bus or error state
      dev->dwErrorCounter++;

      // wake up pending reads or writes
      rwakeup++;
      wwakeup++;
      dev->ucActivityState = ACTIVITY_XMIT; // reset to ACTIVITY_IDLE by cyclic timer
    }

    /* if an error condition occurred, send an error frame to the userspace */
    if ( ef.can_id ) {
      ef.can_id  |= CAN_ERR_FLAG;
      ef.can_dlc  = CAN_ERR_DLC;

      if ( can_rx( dev, &ef ) > 0 ) // put into specific data sink
        rwakeup++;

      memset( &ef, 0, sizeof(ef) ); // clear for next loop
    }

    ret = IRQ_HANDLED;
  }

  if ( wwakeup ) {
    atomic_set(&dev->DataSendReady, 1); // signal I'm ready to write
    //wmb();
    wake_up_interruptible(&dev->write_queue);
  }

  if (rwakeup) {
    wake_up_interruptible(&dev->read_queue);
  }

  spin_unlock_irqrestore( &dev->isr_lock, flags );

  return ret;
}

//******************************************************************************
//! EOF
//******************************************************************************
