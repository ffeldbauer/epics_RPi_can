/*******************************************************************************
 * Copyright (C) 2012 Florian Feldbauer <florian@ep1.ruhr-uni-bochum.de>
 *                    - Helmholtz-Institut Mainz
 *
 * This file is part of dallas1wire
 *
 * dallas1wire is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * dallas1wire is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * brief   Read Temperature from Dallas 1-wire Sensors
 *
 * version 2.0.0; May. 28, 2013
*******************************************************************************/

/*_____ I N C L U D E S ______________________________________________________*/

/* ANSI C includes  */
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* EPICS includes */
#include <aiRecord.h>
#include <alarm.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <devSup.h>
#include <errlog.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include <iocLog.h>
#include <iocsh.h>
#include <recGbl.h>

/*_____ D E F I N I T I O N S ________________________________________________*/

/* Forward declarations */
static long init_record( aiRecord *prec );
static long read_ai( aiRecord *prec );

/*
 * Device Support Entry Table
 */
struct {
  long   number;                /* number of device support routines */
  DEVSUPFUN   report;           /* print reports */
  DEVSUPFUN   init;             /* initialization of device support module */
  DEVSUPFUN   init_record;      /* initialization of single record instances */
  DEVSUPFUN   get_ioint_info;   /* get I/O interrupt info */
  DEVSUPFUN   process;          /* read/write value */
  DEVSUPFUN   special_linconv;  /* Calculate ESLO from EGUL and EGUF for ai/ao records */
} dev_ai_d1w = {
  6,
  NULL,
  NULL,
  init_record,
  NULL,
  read_ai,
  NULL
};

/*
 * Our Device Private data structure
 */
typedef struct {
  epicsUInt8  fcode;
  epicsUInt64 id;
  char        filename[256];
} d1w_info_t;

/*_____ G L O B A L S ________________________________________________________*/

/*_____ L O C A L S __________________________________________________________*/

/*_____ F U N C T I O N S ____________________________________________________*/

/*------------------------------------------------------------------------------
 * @brief   Print error message to IOCshell
 *
 * Prints an error message to STDERR of the IOCshell and sets the
 * status and severity of the record which caused the error.
 *
 * @param   [in]  prec    Address of record
 * @param   [in]  pinfo   Address of device support info structure
 * @param   [in]  status  Reason of error
 *----------------------------------------------------------------------------*/
static void errMsg( aiRecord *prec, d1w_info_t *pinfo, long status ) {
  epicsAlarmCondition stat = READ_ALARM;
  epicsAlarmSeverity sevr  = INVALID_ALARM;

  switch( status ) {
  case -1:
    stat = READ_ALARM;
    fprintf( stderr, "\033[31;1m%s: Could not open file '%s': %s\033[0m\n",
						 prec->name, pinfo->filename, strerror( errno ) );
    break;

  case -2:
    stat = CALC_ALARM;
    fprintf( stderr, "\033[31;1m%s: CRC did not match\033[0m\n", prec->name );
    break;

  case -3:
    stat = CALC_ALARM;
    fprintf( stderr, "\033[31;1m%s: Could not parse value\033[0m\n", prec->name );
    break;

  default:
    stat = READ_ALARM;
    fprintf( stderr, "\033[31;1m%s: Unknown error\033[0m\n", prec->name );
  }
  recGblSetSevr( prec, stat, sevr );
}

/*------------------------------------------------------------------------------
 * @brief   Read contents of file
 *
 * @param   [in]  pinfo  Address of device support info structure
 * @param   [out] value  Address of value
 *
 * @return   0: Evertyhing OK
 *          -1: Cannot open file
 *          -2: CRC did not match
 *          -3: Parsing failed
 *----------------------------------------------------------------------------*/
static long readValue( const d1w_info_t *pinfo, epicsFloat64* value ){
  /* In C all variables have to be declared before the first line of code! */
	FILE * pFile;
  epicsUInt32 dummy = 0;
  epicsUInt32 num = 0;
	char crcMatch[5];

  /* Open the dallas1wire device file for reading */
  pFile = fopen ( pinfo->filename, "r" );
  if( !pFile ) return -1;

  /*
   * Parse first line:
   * 2d 00 4b 46 ff ff 02 10 19 : crc=19 YES
   * The first 9 hex values are the actual message from the device which we ignore
   * we only want to know if the checksum did match or not.
   * num will be set to the number of parsed values, so it should be one.
   */
	num = fscanf( pFile, "%*x %*x %*x %*x %*x %*x %*x %*x %*x : crc=%*x %s", crcMatch );
  if ( 1 != num ) {
	  fclose( pFile );
		return -3;
	}

  /*
   * Compare the parsed cstring to the constant "YES"
   * if they do not match (return value of strcmp is not 0)
   * there was an error during the communication with the device
   */
	if( strcmp( crcMatch, "YES" ) != 0 ){
	  fclose( pFile );
		return -2;
	}

  /*
   * Parse second line:
   * 2d 00 4b 46 ff ff 02 10 19 t=22625
   * Again the first 9 hex values are of no interrest.
   * We only want the last number which is the measured temperature in millicentigrade
   */
	num = fscanf( pFile, "%*x %*x %*x %*x %*x %*x %*x %*x %*x t=%u", &dummy );
	fclose( pFile );
  if ( 1 != num ) return -3;

  /* "convert" the temperature to centigrade */
  *value = dummy / 1000.; 

  return 0;
}

/*------------------------------------------------------------------------------
 * @brief   Initialization of the record
 *
 * @param   [in]  precord  Address of the record calling this function
 *
 * @return  In case of error return -1, otherwise return 2 (no conversion)
 *----------------------------------------------------------------------------*/
static long init_record( aiRecord *prec ) {
  d1w_info_t *pinfo;
  d1w_info_t d1w_info;
  epicsFloat64 value = 0.;
  long status = 0;
  epicsUInt8  familycode = 0;
  epicsUInt64 chipid;

  /* disable record */
  prec->pact = (epicsUInt8)true; 

  /*
   * Parse the string within INP field (the leading @ is omitted)
   * We split here the Familycode (first 2 digit hex number)
   * from the rest of the unique id
   * This allows us to extend this device support module for other D1W chips
   * which may have a different output format
   */
  if ( sscanf( prec->inp.value.instio.string, "%x-%x", &familycode, chipid ) != 2 ) {
    fprintf( stderr, "\033[31;1m%s: Invalid value of INP field '%s'\033[0m\n",
             prec->name, prec->inp.value.instio.string );
    return -1;
  }
  /*
   * Up to now this Support module only support temperature sensors
   * e.g. DS18S20
   * So we check the familycode
   */
  if ( 0x28 != familycode && 0x10 != familycode ) {
    fprintf( stderr, "\033[31;1m%s: Invalid family code '%d'\033[0m\n",
             prec->name, familycode );
    return -1;
  }

  /* fill our temporary info structure */
  d1w_info.fcode = familycode;
  d1w_info.id    = chidpid;
  sprintf( d1w_info.filename, "/sys/devices/w1_bus_master1/%02x-%012x/w1_slave",
           familycode, chipid );

  status = readValue( &d1w_info, &value );
  /* if CRC mismatched, try again */
  if ( -2 == status ) status = readValue( &d1w_info, &value );
  if ( status ) {
    errMsg ( prec, &d1w_info, status );
    return status;
  }
  prec->val = value;

  /* allocate memory for the "persistent" info structure */
  pinfo = (d1w_info_t *) calloc( 1, sizeof( d1w_info_t ) );
  assert( NULL != pinfo );
  /* Copy the memory from temporary to persistent info structure */
  memcpy( pinfo, &d1w_info, sizeof( d1w_info ) );
  assert( NULL == prec->dpvt );

  /* Set values of record */
  prec->dpvt = pinfo;
  prec->linr = 0;
  prec->udf  = FALSE;
  /* enable record */
  prec->pact = (epicsUInt8)false; 

  return 2;
}

/*------------------------------------------------------------------------------
 * @brief   Read routine of the record
 *
 * @param   [in]  precord  Address of the record calling this function
 *
 * @return  In case of error return -1, otherwise return 2 (no conversion)
 *----------------------------------------------------------------------------*/
static long read_ai( aiRecord *prec ) {
  d1w_info_t *pinfo = (d1w_info_t *)prec->dpvt;
  epicsFloat64 value = 0.;
  long status = 0;

  status = readValue( pinfo, &value );
  if ( -2 == status ) status = readValue( pinfo, &value );
  if ( status ) {
    errMsg ( prec, pinfo, status );
    return status;
  }
  prec->val = value;

  return 2;
}

/*------------------------------------------------------------------------------
 * @brief   Export address of Device Support Entry Table to EPICS IOCshell
 *----------------------------------------------------------------------------*/
epicsExportAddress( dset, dev_ai_d1w );

/* EOF */
