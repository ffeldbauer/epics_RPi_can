/*
 * Based on sja1000_isa.c 
 * Copyright (C) 2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Copyright (C) 2014 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
 *                    - Ruhr-Universitaet Bochum, Lehrstuhl fuer Experimentalphysik I
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/****** I N C L U D E S ******************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <mach/platform.h>

#include "sja1000.h"

/****** D E F I N I T I O N S *************************************************/
MODULE_AUTHOR( "Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>" );
MODULE_DESCRIPTION( "Socket-CAN driver for SJA1000 on the PANDA Raspberry Pi CAN extension Board" );
MODULE_SUPPORTED_DEVICE("PANDA Raspberry Pi CAN extension Board");
MODULE_LICENSE( "GPL v2" );

/* define offset for used registers, refer to page 90 of the BCM2835 ARM Peripherals manual */
#define GPIOFSEL(x)  (0x00+(x)*4) /* GPIO Function Select  */
#define GPIOSET(x)   (0x1c+(x)*4) /* GPIO Pin Output Set   */
#define GPIOCLR(x)   (0x28+(x)*4) /* GPIO Pin Output Clear */
#define GPIOLEV(x)   (0x34+(x)*4) /* GPIO Pin Level        */

#define DRV_NAME     "sja1000_raspi"
#define CLK_DEFAULT  16000000                                     /* 16 MHz ext clock */
#define CDR_DEFAULT  ( CDR_CBP | CDR_CLKOUT_MASK | CDR_CLK_OFF )  /* 0x4f */
#define OCR_DEFAULT  ( OCR_TX0_PUSHPULL )                         /* 0x18 */

/* static unsigned gpio_ad[8] = { 7, 8, 9, 27, 22, 23, 24, 25 }; */
static int clk = 0;
static unsigned char cdr = 0xff;
static unsigned char ocr = 0xff;

/* module_param_array( gpio_ad, uint, NULL, S_IRUGO ); */
module_param( clk, int, S_IRUGO );
module_param( cdr, byte, S_IRUGO );
module_param( ocr, byte, S_IRUGO );

/* MODULE_PARM_DESC( gpio_ad, "GPIO numbers used as A/D lines (default 7, 8, 9, 27, 22, 23, 24, 25)" ); */
MODULE_PARM_DESC( clk, "External oscillator clock frequency (default=16000000 [16 MHz])" );
MODULE_PARM_DESC( cdr, "Clock divider register (default=0x4f [CDR_CBP | CDR_CLKOUT_MASK | CDR_CLK_OFF])" );
MODULE_PARM_DESC( ocr, "Output control register (default=0x18 [OCR_TX0_PUSHPULL])" );

/****** L O C A L S ***********************************************************/

/* GPIOs used as address/data lines */
static struct gpio ADi[8] = {
  {  7, GPIOF_OUT_INIT_LOW, "AD0" },
  {  8, GPIOF_OUT_INIT_LOW, "AD1" },
  {  9, GPIOF_OUT_INIT_LOW, "AD2" },
  { 27, GPIOF_OUT_INIT_LOW, "AD3" },
  { 22, GPIOF_OUT_INIT_LOW, "AD4" },
  { 23, GPIOF_OUT_INIT_LOW, "AD5" },
  { 24, GPIOF_OUT_INIT_LOW, "AD6" },
  { 25, GPIOF_OUT_INIT_LOW, "AD7" }
};

/* GPIOs used as ctrl lines */
static u8 ALE = 11;
static u8 nRD = 17;
static u8 nWR = 10;
static u8 nCS = 18;
static u8 nIRQ = 4;

/* Masks for LEV/SET/CLR registers to select GPIOs used as A/D lines */
static const u32 gpioMaskL = (  7 <<  7 );
static const u32 gpioMaskM = (  1 << 27 );
static const u32 gpioMaskH = ( 15 << 22 );
static const u32 gpioMask  = (  7 <<  7 ) | ( 15 << 22 ) | (  1 << 27 );

/*
 * Masks to switch direction of GPIOs used as A/D lines
 * To switch pin g to input  use GPIOFSEL( g/10 ) &= ~( 7 <<  (( g % 10 ) * 3 ) );
 * To switch pin g to output use GPIOFSEL( g/10 ) |=  ( 1 <<  (( g % 10 ) * 3 ) );
 */
static const u32 gpioFSEL0_InpMask = ~( ( 7 << 21 ) | ( 7 << 24 ) | ( 7 << 27 ) );
static const u32 gpioFSEL0_OutMask = ( 1 << 21 ) | ( 1 << 24 ) | ( 1 << 27 );
static const u32 gpioFSEL2_InpMask = ~( ( 7 << 6 ) | ( 7 << 9 ) | ( 7 << 12 ) | ( 7 << 15 ) | ( 7 << 21 ) );
static const u32 gpioFSEL2_OutMask = ( 1 << 6 ) | ( 1 << 9 ) | ( 1 << 12 ) | ( 1 << 15 ) | ( 1 << 21 );

static struct platform_device *sja1000_raspi_dev;

/****** F U N C T I O N S *****************************************************/

static u8 sja1000_raspi_read_reg( const struct sja1000_priv *priv, int reg ) {
  u8 data = 0;
  u32 buffer;
  void __iomem *base = priv->reg_base;

  /* Set AD0..7 to outoput */
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  buffer |= gpioFSEL0_OutMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  buffer |= gpioFSEL2_OutMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  /* Write register address */
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

  /* Set AD0..7 to input */
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  udelay(1);
  /* Read register contents */
  buffer  = ioread32( base + GPIOLEV(0) );
  wmb();
  data  = (u8)(( buffer & gpioMaskL ) >>  7 );
  data |= (u8)(( buffer & gpioMaskM ) >> 24 );
  data |= (u8)(( buffer & gpioMaskH ) >> 18 );

  iowrite32( ( 1 << nCS ) | ( 1 << nRD ), base + GPIOSET(0) );
  wmb();

  /*printk( KERN_INFO "%s: Read register %i: 0x%02x\n", DRV_NAME, reg, data );*/

  return data;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void sja1000_raspi_write_reg( const struct sja1000_priv *priv,
                                     int reg, u8 val ){
  u32 buffer;
  void __iomem *base = priv->reg_base;

  /*printk( KERN_INFO "%s: Write register %i: 0x%02x\n", DRV_NAME, reg, val );*/

  /* Set AD0..7 to outoput */
  buffer  = ioread32( base + GPIOFSEL(0) );
  buffer &= gpioFSEL0_InpMask;
  buffer |= gpioFSEL0_OutMask;
  iowrite32( buffer, base + GPIOFSEL(0) );
  buffer  = ioread32( base + GPIOFSEL(2) );
  buffer &= gpioFSEL2_InpMask;
  buffer |= gpioFSEL2_OutMask;
  iowrite32( buffer, base + GPIOFSEL(2) );
  wmb();

  /* Write register address */
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

  /* Write data to register */
  buffer  = ( val & 0x07 ) <<  7;
  buffer |= ( val & 0x08 ) << 24;
  buffer |= ( val & 0xf0 ) << 18;
  iowrite32( buffer, base + GPIOSET(0) );
  buffer  = ~(buffer) & gpioMask;
  iowrite32( buffer & gpioMask, base + GPIOCLR(0) );
  wmb();

  iowrite32( ( 1 << nCS ) | ( 1 << nWR ), base + GPIOSET(0) );
  wmb();
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static int sja1000_raspi_probe( struct platform_device *pdev ) {
  struct net_device *dev;
  struct sja1000_priv *priv;
  void __iomem *base = __io_address( GPIO_BASE );
  int err;

  /* request used GPIO lines */
  err = gpio_request_one( ALE, GPIOF_OUT_INIT_LOW, "ALE" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DRV_NAME, ALE, err );
    goto exit;
  }
  err = gpio_request_one( nRD, GPIOF_OUT_INIT_HIGH, "nRD" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DRV_NAME, nRD, err );
    goto exit;
  }
  err = gpio_request_one( nWR, GPIOF_OUT_INIT_HIGH, "nWR" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DRV_NAME, nWR, err );
    goto exit;
  }
  err = gpio_request_one( nCS, GPIOF_OUT_INIT_HIGH, "nCS" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DRV_NAME, nCS, err );
    goto exit;
  }
  err = gpio_request_array( ADi, ARRAY_SIZE(ADi) );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO array for AD ports! Error: %d\n", DRV_NAME, err );
    goto exit;
  }
  err = gpio_request_one( nIRQ, GPIOF_IN, "nINT" );
  if( err ) {
    printk( KERN_ERR "%s: Cannot request GPIO %d! Error: %d\n", DRV_NAME, nIRQ, err );
    goto exit;
  }

  dev = alloc_sja1000dev( 0 );
  if ( !dev ) {
    err = -ENOMEM;
    goto exit;
  }
  priv = netdev_priv( dev );
  dev->irq = gpio_to_irq( nIRQ );
  priv->irq_flags = IRQF_TRIGGER_FALLING | IRQF_SHARED;
  priv->reg_base = base;
  priv->read_reg = sja1000_raspi_read_reg;
  priv->write_reg = sja1000_raspi_write_reg;
  priv->can.clock.freq = ( clk != 0 ) ? ( clk / 2 ) : ( CLK_DEFAULT / 2 );
  priv->ocr = ( ocr != 0xff ) ? ocr : OCR_DEFAULT;
  priv->cdr = ( cdr != 0xff ) ? cdr : CDR_DEFAULT;

  platform_set_drvdata( pdev, dev );
  SET_NETDEV_DEV( dev, &pdev->dev );

  dev_info( &pdev->dev, "registering %s device (reg_base=0x%p, irq=%d)\n",
            DRV_NAME, priv->reg_base, dev->irq );

  err = register_sja1000dev( dev );
  if( err ) {
    dev_err( &pdev->dev, "registering %s failed (err=%d)\n",
             DRV_NAME, err);
    goto exit;
  }

  dev_info( &pdev->dev, "%s device registered (reg_base=0x%p, irq=%d)\n",
            DRV_NAME, priv->reg_base, dev->irq );

  return 0;

 exit:
  gpio_free( ALE );
  gpio_free( nRD );
  gpio_free( nWR );
  gpio_free( nCS );
  gpio_free_array( ADi, ARRAY_SIZE(ADi) );
  gpio_free( nIRQ );
  return err;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static int sja1000_raspi_remove( struct platform_device *pdev ) {
  struct net_device *dev = platform_get_drvdata( pdev );
  /*struct sja1000_priv *priv = netdev_priv( dev );*/

  unregister_sja1000dev( dev );
  free_sja1000dev( dev );

  gpio_free( ALE );
  gpio_free( nRD );
  gpio_free( nWR );
  gpio_free( nCS );
  gpio_free_array( ADi, ARRAY_SIZE(ADi) );
  gpio_free( nIRQ );

  return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static struct platform_driver sja1000_raspi_driver = {
  .probe = sja1000_raspi_probe,
  .remove = sja1000_raspi_remove,
  .driver = {
    .name = DRV_NAME,
    .owner = THIS_MODULE,
  },
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static int __init sja1000_raspi_init( void ) {
  int err = 0;

  sja1000_raspi_dev = platform_device_alloc( DRV_NAME, 0 );
  if( !sja1000_raspi_dev ) {
    err = -ENOMEM;
    goto fail;
  }
  err = platform_device_add( sja1000_raspi_dev );
  if( err ) {
    platform_device_put( sja1000_raspi_dev );
    goto fail;
  }

  err = platform_driver_register( &sja1000_raspi_driver );
  if( err )
    goto fail;

  return 0;

 fail:
  if( sja1000_raspi_dev )
    platform_device_unregister( sja1000_raspi_dev );

  return err;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static void __exit sja1000_raspi_exit( void ) {
  platform_driver_unregister( &sja1000_raspi_driver );
  if( sja1000_raspi_dev )
    platform_device_unregister( sja1000_raspi_dev );

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

module_init( sja1000_raspi_init );
module_exit( sja1000_raspi_exit );

