PANDA DCS epicsRPiCAN package
=============================

This package is used by the PANDA collaboration for EPICS device support.
The following devices controlled via CAN bus interfaces are supported:
-  CAN interface (as lower-level driver used by all other drivers)
-  [ISEG EHS/EDS high voltage modules](http://www.iseg-hv.com/en/products/product-details/product/21/)
-  [Wiener VME crate remote control](http://www.wiener-d.com/products/24/19.html)(via CANbus)
-  Temperature and Humidity Monitoring board for PANDA (THMP)
-  PANDA-EMC light pulser
-  [MCM142 1-axis stepper controller/driver](http://www.mocontronic.de/de/katalog/motorsteuerungen/TMCM-142-IF)(via CANbus)

Dependencies
============

CAN_interface:
-  For the kernel module to compile you need the kernel sources and headers
-  For the test program the libsocketcan is needed

drvAsynCan
-  [EPICS base 3.14.12.4 or later](http://www.aps.anl.gov/epics/)
-  [AsynDriver 4-22 or later](http://www.aps.anl.gov/epics/modules/soft/asyn/)
-  Optionally libsocketcan

dallas1wire
-  [EPICS base 3.14.12.4 or later](http://www.aps.anl.gov/epics/)

Install
=======

 CAN_interface:
  1. refer to CAN_interface/driver/README

 drvAsynCan:
  1. Edit configure/RELEASE and change the paths to ASYN and EPICS_BASE
  2. Optionally create configure/RELEASE.Common.&lt;arch&gt; and set LIBSOCKETCAN_INCLUDE and LIBSOCKETCAN_LIB
  3. Type `make` to compile the package.

 dallas1wire:
  1. Edit configure/RELEASE and change the paths to EPICS_BASE
  2. Type `make` to compile the package.

Usage
=====

Refer to the wiki page on github (https://github.com/ffeldbauer/epics_RPi_can/wiki)
