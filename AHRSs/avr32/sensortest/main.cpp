/*
 *  Sensor test / display for AVR32 AHRS
 *  Works on the ATNGW100 reference board, see schematics on the hardware folder
 *
 *
 *  Copyright (c) by Carlos Becker	http://github.com/cbecker 
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../input/ads1256.h"
#include "../input/lis3lv02.h"
#include "../input/ad12.h"

#include "../avr32hw.h"
#include "../timer_this.h"

using namespace	input;

static void	fatalErr(const char *str) 
{
	printf("FATAL ERROR: %s\n", str );
	exit(-1);
}

int main( int argc, char **argv )
{
	ADS1256		ads1256( AVR32_SPIDEV_ADS1256, AVR32_ADS1256_VREF );
	if ( !ads1256.init( AVR32_ADS1256_SPICLK ) )
		fatalErr("Error init ADS1256\n");

	LIS3LV02	accel( AVR32_SPIDEV_LIS3LV02 );
	if ( !accel.init( AVR32_LIS3LV02_SPICLK ) )
		fatalErr("Error init accel\n");

	AD12		ad12( AVR32_SPIDEV_MCP3208, AVR32_MCP3208_VREF );
	if ( !ad12.init( AVR32_MCP3208_SPICLK ) )
		fatalErr("Error init MCP3208\n");
	
	float	a[3];
	float	mx,my,mz;
	float	wx,wy,wz;
	float	gVRef = 0;

	a[0] = a[1] = a[2] = mx = my = mz = wx = wy = wz = 0;

	float min,max;
	min = 3.3;
	max = 0.0;

	ads1256.test();

	for(;;) 
	{

	TIME_THIS("Mags:",

		if ( !ads1256.convert( CH_MAGX, &mx ) )
			fatalErr("Error get sample mx\n");
		if ( !ads1256.convert( CH_MAGY, &my ) )
			fatalErr("Error get sample mx\n");
		if ( !ads1256.convert( CH_MAGZ, &mz ) )
			fatalErr("Error get sample mx\n");
	);

	TIME_THIS("Accel:",
		if ( !accel.getAccel(a) )
			fatalErr("Error get accel\n");
	);

	TIME_THIS("Gyros:",
		if ( !ad12.getSample( CH_GYROX, &wx ) )
			fatalErr("Error get sample gyroX\n");
		if ( !ad12.getSample( CH_GYROY, &wy ) )
			fatalErr("Error get sample gyroY\n");
		if ( !ad12.getSample( CH_GYROZ, &wz ) )
			fatalErr("Error get sample gyroZ\n");
		
		if ( !ad12.getSample( CH_VREF, &gVRef ) )
			fatalErr("Error get sample vRef\n");
	);
		if ( gVRef > max )	max = gVRef;
		if ( gVRef < min )	min = gVRef;
		printf("\n");
		printf("Ax: %f\nAy:%lf\nAz:%f\n\n", a[0], a[1], a[2] );
		printf("Mx: %f\nMy:%lf\nMz:%f\n\n", mx, my, mz );
		printf("Wx: %f\nWy:%lf\nWz:%f\n\n", wx, wy, wz );
		printf("Gyro vRef: %f\n", gVRef );
		printf("Gyro vRef noise (max - min): %f\n",  max - min );

		usleep(100e3);
	}

	return 0;
}

