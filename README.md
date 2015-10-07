# PANDA DCS epicsRPiCAN package

This package is used by the PANDA collaboration for EPICS device support.
It is devided into three sub packages:

__UPDATE__ Kernel Module was removed from this repository.
It is now available in my fork of the raspberrypi/linux repository
[here](https://github.com/ffeldbauer/PandaRPiKernel)

#### CAN_test:
Test application to test the socket-CAN driver

#### drvAsynCan:
EPICS device support module based on the asynPortDriver class from AsynDriver.
This module adds device support for devices controlled via CANbus.
The following devices controlled via CAN bus interfaces are supported:
   -  CAN interface (as lower-level driver used by all other drivers)
   -  [ISEG EHS/EDS high voltage modules](http://www.iseg-hv.com/en/products/product-details/product/21/)
   -  [Wiener VME crate remote control](http://www.wiener-d.com/products/24/19.html)(via CANbus)
   -  Temperature and Humidity Monitoring board for PANDA (THMP)
   -  PANDA-EMC light pulser
   -  [TMCM142 1-axis stepper controller/driver](http://www.mocontronic.de/de/katalog/motorsteuerungen/TMCM-142-IF)(via CANbus)
   -  PANDA FSC HV from Protvino

#### dallas1wire:
EPICS device support module to read out DS18S20 digital temperature sensors connected via
dallas-1-wire to the Raspberry Pi Computer

## Dependencies

#### drvAsynCan
-  [EPICS base 3.14.12.4 or later](http://www.aps.anl.gov/epics/)
-  [AsynDriver 4-22 or later](http://www.aps.anl.gov/epics/modules/soft/asyn/)

#### dallas1wire
-  [EPICS base 3.14.12.4 or later](http://www.aps.anl.gov/epics/)

## Install

#### drvAsynCan
 1.  Edit configure/RELEASE and change the paths to ASYN and EPICS_BASE
 2.  Type `make` to compile the package.

#### dallas1wire
 1.  Edit configure/RELEASE and change the paths to EPICS_BASE
 2.  Type `make` to compile the package.

## Usage

Refer to the [wiki](https://github.com/ffeldbauer/epics_RPi_can/wiki) on github

