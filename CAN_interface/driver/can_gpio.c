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
//! @author  F. Feldbauer <florian@ep1.ruhr-uni-bochum.de>
//!
//! @brief   all parts of the GPIO hardware to interface the SJA1000
//!
//! @version 2.0.0
//******************************************************************************

//_____ I N C L U D E S _______________________________________________________
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <mach/platform.h>

#include "can_gpio.h"
#include "can_sja1000.h"

//_____ D E F I N I T I O N S __________________________________________________
#define GPIOFSEL(x)  (0x00+(x)*4)
#define GPIOSET(x)   (0x1c+(x)*4)
#define GPIOCLR(x)   (0x28+(x)*4)
#define GPIOLEV(x)   (0x34+(x)*4)

//_____ G L O B A L S __________________________________________________________

//_____ L O C A L S ____________________________________________________________

// GPIOs used as address/data lines
static struct gpio ADi[8] = {
  {  7, GPIOF_OUT_INIT_LOW, "AD0" },
  {  8, GPIOF_OUT_INIT_LOW, "AD1" },
  {  9, GPIOF_OUT_INIT_LOW, "AD2" },
  { 27, GPIOF_OUT_INIT_LOW, "AD3" }, // was 21 on rev1
  { 22, GPIOF_OUT_INIT_LOW, "AD4" },
  { 23, GPIOF_OUT_INIT_LOW, "AD5" },
  { 24, GPIOF_OUT_INIT_LOW, "AD6" },
  { 25, GPIOF_OUT_INIT_LOW, "AD7" }
};

// GPIOs used as ctrl lines
static u8 ALE = 11;
static u8 nRD = 17;
static u8 nWR = 10;
static u8 nCS = 18;
static u8 nIRQ = 4;

// Masks for LEV/SET/CLR registers to get GPIOs used as A/D lines
static const u32 gpioMaskL = (  7 <<  7 );
static const u32 gpioMaskM = (  1 << 27 );  // was 21 on rev1
static const u32 gpioMaskH = ( 15 << 22 );
static const u32 gpioMask  = (  7 <<  7 ) | ( 15 << 22 ) | (  1 << 27 ); // last one was 21 on rev1

// Masks to switch direction of GPIOs used as A/D lines
// To switch pin g to input use GPIOFSEL( g/10 ) &= ~ ( 7 <<  (( g % 10 ) * 3 ) );
// To switch pin g to output use GPIOFSEL( g/10 ) |= ( 1 <<  (( g % 10 ) * 3 ) );
static const u32 gpioFSEL0_InpMask = ~( ( 7 << 21 ) | ( 7 << 24 ) | ( 7 << 27 ) );
static const u32 gpioFSEL0_OutMask = ( 1 << 21 ) | ( 1 << 24 ) | ( 1 << 27 );
static const u32 gpioFSEL2_InpMask = ~( ( 7 << 6 ) | ( 7 << 9 ) | ( 7 << 12 ) | ( 7 << 15 ) | ( 7 << 21 ) );
static const u32 gpioFSEL2_OutMask = ( 1 << 6 ) | ( 1 << 9 ) | ( 1 << 12 ) | ( 1 << 15 ) | ( 1 << 21 );

// Address base of GPIO registers
static void __iomem *base;

//_____ F U N C T I O N S ______________________________________________________

//------------------------------------------------------------------------------
//! @brief   read contents of a register
//!
//! @param   [in]  reg:   Address of register
//!
//! @return  contents of register
//------------------------------------------------------------------------------
u8 can_gpio_readreg( u8 reg ) {
  u8 data = 0;
  u32 buffer;

  // Set AD0..7 to outoput
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  buffer |= gpioFSEL0_OutMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  buffer |= gpioFSEL2_OutMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  // Write register address
  buffer  = ( reg & 0x07 ) <<  7;
  buffer |= ( reg & 0x08 ) << 24;
  buffer |= ( reg & 0xf0 ) << 18;
  iowrite32( ( 1 << ALE ), base + GPIOSET(0) );
  wmb();
  iowrite32( buffer, base + GPIOSET(0) );
  buffer = ~(buffer) & gpioMask;
  iowrite32( buffer, base + GPIOCLR(0) );
  wmb();

  iowrite32( ( 1 << ALE ), base + GPIOCLR(0) );
  iowrite32( ( 1 << nCS ) | ( 1 << nRD ), base + GPIOCLR(0) );
  wmb();

  // Set AD0..7 to input
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  udelay(1);
  // Read register contents
  buffer  = ioread32( base + GPIOLEV(0) );
  wmb();
  data  = (u8)(( buffer & gpioMaskL ) >>  7 );
  data |= (u8)(( buffer & gpioMaskM ) >> 24 );
  data |= (u8)(( buffer & gpioMaskH ) >> 18 );

  iowrite32( ( 1 << nCS ) | ( 1 << nRD ), base + GPIOSET(0) );
  wmb();

  return data;
}

//------------------------------------------------------------------------------
//! @brief   write content to register
//!
//! @param   [in]  reg:   Address of register
//! @param   [in]  data:  Data to write in register
//------------------------------------------------------------------------------
void can_gpio_writereg( u8 reg, u8 data ) {
  u32 buffer;

  // Set AD0..7 to outoput
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  buffer |= gpioFSEL0_OutMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  buffer |= gpioFSEL2_OutMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  // Write register address
  buffer  = ( reg & 0x07 ) <<  7;
  buffer |= ( reg & 0x08 ) << 24;
  buffer |= ( reg & 0xf0 ) << 18;
  iowrite32( ( 1 << ALE ), base + GPIOSET(0) );
  wmb();
  iowrite32( buffer, base + GPIOSET(0) );
  buffer = ~(buffer) & gpioMask;
  iowrite32( buffer, base + GPIOCLR(0) );
  wmb();

  iowrite32( ( 1 << ALE ), base + GPIOCLR(0) );
  iowrite32( ( 1 << nCS ) | ( 1 << nWR ), base + GPIOCLR(0) );
  wmb();

  // Write data to register
  buffer  = ( data & 0x07 ) <<  7;
  buffer |= ( data & 0x08 ) << 24;
  buffer |= ( data & 0xf0 ) << 18;
  iowrite32( buffer, base + GPIOSET(0) );
  buffer  = ~(buffer) & gpioMask;
  iowrite32( buffer & gpioMask, base + GPIOCLR(0) );
  wmb();

  iowrite32( ( 1 << nCS ) | ( 1 << nWR ), base + GPIOSET(0) );
  wmb();
}

//------------------------------------------------------------------------------
//! @brief   Initialize GPIOs
//!
//! @param   [out] dev:  address of can device descriptor
//!
//! @return  0 if no error occures, otherwise return errno
//------------------------------------------------------------------------------
int can_gpio_init( struct candev *dev ) {
  int err = 0;

  init_waitqueue_head( &dev->read_queue );
  init_waitqueue_head( &dev->write_queue );

  err = gpio_request_one( ALE, GPIOF_OUT_INIT_LOW, "ALE" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DEVICE_NAME, ALE, err );
    goto fail;
  }

  err = gpio_request_one( nRD, GPIOF_OUT_INIT_HIGH, "nRD" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DEVICE_NAME, nRD, err );
    goto fail;
  }
  err = gpio_request_one( nWR, GPIOF_OUT_INIT_HIGH, "nWR" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DEVICE_NAME, nWR, err );
    goto fail;
  }
  err = gpio_request_one( nCS, GPIOF_OUT_INIT_HIGH, "nCS" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DEVICE_NAME, nCS, err );
    goto fail;
  }
  err = gpio_request_array( ADi, ARRAY_SIZE(ADi) );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO array for AD ports! Error: %d\n", DEVICE_NAME, err );
    goto fail;
  }
  err = gpio_request_one( nIRQ, GPIOF_IN, "nINT" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DEVICE_NAME, nIRQ, err );
    goto fail;
  }

  dev->wIrq = gpio_to_irq( nIRQ );
  if ( ( err = dev->wIrq ) < 0 ) {
    printk( KERN_ERR "%s: Cannot map GPIO %d to IRQ! Error: %d\n", DEVICE_NAME, nIRQ, err );
    goto fail;
  }

  dev->ucPhysicallyInstalled = 1;

  base = __io_address( GPIO_BASE );

  return 0;

 fail:
  can_gpio_free();
  return err;
}

//------------------------------------------------------------------------------
//! @brief   Register IRQ Handler for device
//!
//! @param   [in]  dev:  address of can device descriptor
//!
//! @return  0 if no error occures, otherwise return errno
//------------------------------------------------------------------------------
int can_gpio_req_irq( struct candev *dev ) {
  return request_irq( dev->wIrq, can_sja1000_irqhandler, IRQF_TRIGGER_FALLING, "rpi_can", dev );
}

//------------------------------------------------------------------------------
//! @brief   Free registered IRQ Handler
//!
//! @param   [in]  dev:  address of can device descriptor
//------------------------------------------------------------------------------
void can_gpio_free_irq( struct candev* dev ) {
  free_irq( dev->wIrq, dev );
}

//------------------------------------------------------------------------------
//! @brief   Free GPIOs
//------------------------------------------------------------------------------
void can_gpio_free( void ) {
  gpio_free( ALE );
  gpio_free( nRD );
  gpio_free( nWR );
  gpio_free( nCS );
  gpio_free_array( ADi, ARRAY_SIZE(ADi) );
  gpio_free( nIRQ );
}

//******************************************************************************
//! EOF
//******************************************************************************

