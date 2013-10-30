TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += dallas1wire

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install 1wire.dbd into <top>/dbd
DBD += dallas1wire.dbd

# specify all source files to be compiled and added to the library
dallas1wire_SRCS += devAiRecord.c
dallas1wire_SRCS += readFile.cpp

dallas1wire_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

