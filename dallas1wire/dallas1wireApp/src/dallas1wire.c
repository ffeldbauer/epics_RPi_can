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

static long init_record( aiRecord *prec );
static long read_ai( aiRecord *prec );

typedef struct {
  long   number;
  DEVSUPFUN   report;
  DEVSUPFUN   init;
  DEVSUPFUN   init_record;
  DEVSUPFUN   get_ioint_info;
  DEVSUPFUN   read_ai;
  DEVSUPFUN   special_linconv;
} dev_dset_d1w_t;

typedef struct {
  int fcode;
  char id[20];
  char filename[256];
} d1w_info_t;

/*_____ G L O B A L S ________________________________________________________*/

/*_____ L O C A L S __________________________________________________________*/
dev_dset_d1w_t dev_ai_d1w = { 6, NULL, NULL, init_record, NULL, read_ai, NULL };
epicsExportAddress( dset, dev_ai_d1w );

/*_____ F U N C T I O N S ____________________________________________________*/

static void errMsg( aiRecord *prec, d1w_info_t *pinfo, long status ) {
  epicsAlarmCondition stat = READ_ALARM;
  epicsAlarmSeverity sevr = INVALID_ALARM;

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
static long readValue( const d1w_info_t *pinfo, double* value ){
	FILE * pFile;
  int dummy = 0;
  int num = 0;
	char crcMatch[5];
  pFile = fopen ( pinfo->filename, "r" );
	if( !pFile ) return -1;

	num = fscanf( pFile, "%*x %*x %*x %*x %*x %*x %*x %*x %*x : crc=%*x %s", crcMatch );
  if ( 1 != num ) {
	  fclose( pFile );
		return -3;
	}
	if( strcmp( crcMatch, "YES" ) != 0 ){
	  fclose( pFile );
		return -2;
	}

	num = fscanf( pFile, "%*x %*x %*x %*x %*x %*x %*x %*x %*x t=%u", &dummy );
	fclose( pFile );
  if ( 1 != num ) return -3;
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
  double value = 0.;
  long status = 0;
  int familycode = 0;
  char tmp[20];

  prec->pact = (epicsUInt8)true; /* disable record */

  if ( sscanf( prec->inp.value.instio.string, "%x-%20s", &familycode, tmp ) != 2 ) {
    fprintf( stderr, "\033[31;1m%s: Invalid value of INP field '%s'\033[0m\n",
             prec->name, prec->inp.value.instio.string );
    return -1;
  }
  if ( 0x28 != familycode && 0x10 != familycode ) {
    fprintf( stderr, "\033[31;1m%s: Invalid family code '%d'\033[0m\n",
             prec->name, familycode );
    return -1;
  }

  d1w_info.fcode = familycode;
  strcpy( d1w_info.id, tmp );
  sprintf( d1w_info.filename, "/sys/devices/w1_bus_master1/%x-%s/w1_slave",
           familycode, tmp );

  status = readValue( &d1w_info, &value );
  if ( -2 == status ) status = readValue( &d1w_info, &value );
  if ( status ) {
    errMsg ( prec, &d1w_info, status );
    return status;
  }
  prec->val = value;

  pinfo = (d1w_info_t *) calloc( 1, sizeof( d1w_info_t ) );
  assert( NULL != pinfo );
  memcpy( pinfo, &d1w_info, sizeof( d1w_info ) );
  assert( NULL == prec->dpvt );

  prec->dpvt = pinfo;
  prec->linr = 0;
  prec->udf  = FALSE;
  prec->pact = (epicsUInt8)false; /* enable record */

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
  double value = 0.;
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


/* EOF */
