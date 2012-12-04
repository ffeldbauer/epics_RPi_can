TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += drvAsynRPiCan.dbd

LIBRARY_IOC += drvAsynRPiCanSupport
drvAsynRPiCanSupport_SRCS += drvAsynRPiCan.cpp
drvAsynRPiCanSupport_SRCS += drvAsynIsegHv.cpp
drvAsynRPiCanSupport_SRCS += drvAsynIsegHvGlobal.cpp
drvAsynRPiCanSupport_SRCS += drvAsynWienerVME.cpp
drvAsynRPiCanSupport_SRCS += drvAsynTHMP.cpp
drvAsynRPiCanSupport_SRCS += drvAsynLedPulser.cpp
drvAsynRPiCanSupport_SRCS += registerIOCfunctions.cpp
drvAsynRPiCanSupport_LIBS += asyn
drvAsynRPiCanSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

#PROD_IOC += drvAsynRPiCan

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
#drvAsynRPiCan_SRCS_DEFAULT += drvAsynRPiCan_registerRecordDeviceDriver.cpp AsynRPiCanMain.cpp

#drvAsynRPiCan_LIBS += drvAsynRPiCanSupport asyn
#drvAsynRPiCan_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
