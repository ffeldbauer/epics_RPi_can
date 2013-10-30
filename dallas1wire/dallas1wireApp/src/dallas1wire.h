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

#ifndef DALLAS_ONE_WIRE
#define DALLAS_ONE_WIRE

/*_____ I N C L U D E S ______________________________________________________*/

/*_____ D E F I N I T I O N S ________________________________________________*/
typedef struct {
  int fcode;
  char id[20];
  char filename[256];
} d1w_info_t;

#ifdef __cplusplus
extern "C"
{
#endif
  long readValue( d1w_info_t *pinfo, double* value );
#ifdef __cplusplus
}
#endif // __cplusplus

#endif
