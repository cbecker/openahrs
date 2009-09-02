/*
 *  AHRS for AVR32.
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


#include "avr32hw.h"
#include <stdio.h>

bool	Sensing::flipMagns()
{
	dADS->setIO( AVR32_ADS1256_COILSET, 0 );
	dADS->setIO( AVR32_ADS1256_COILCLR, 1 );
	usleep(2e3);
	dADS->setIO( AVR32_ADS1256_COILSET, 1 );
	dADS->setIO( AVR32_ADS1256_COILCLR, 0 );
	usleep(2e3);

	return true;
}

bool	Sensing::init()
{
	d12	= new input::AD12( AVR32_SPIDEV_MCP3208, AVR32_MCP3208_VREF );
	if ( !d12->init(AVR32_MCP3208_SPICLK) ) {
		printf("Error init MCP3208\n");
		delete d12;
		return false;
	}

	dADS	= new input::ADS1256( AVR32_SPIDEV_ADS1256, AVR32_ADS1256_VREF );
	if ( !dADS->init(AVR32_ADS1256_SPICLK) ) {
		printf("Error init ADS1256\n");
		delete dADS; delete d12;
		return false;
	}
	

	//init
	dADS->test();

	dAccel	= new input::LIS3LV02( AVR32_SPIDEV_LIS3LV02 );
	if ( !dAccel->init(AVR32_LIS3LV02_SPICLK) ) {
		printf("Error init LIS3LV02\n");
		delete dADS; delete d12; delete dAccel;
		return false;
	}

	return true;
}

bool	Sensing::getAccels( Matrix<FT,3,1>	&a )
{
	float	f[3];
	if ( !dAccel->getAccel(f) )	
		return false;
	
	a << -f[1],-f[0],f[2];

	return true;
}

bool	Sensing::getGyros( Matrix<FT,3,1>	&omega )
{
	float gx,gy,gz;
	if ( !d12->getSample( CH_GYROX, &gx ) )
		return false;
	if ( !d12->getSample( CH_GYROY, &gy ) )
		return false;
	if ( !d12->getSample( CH_GYROZ, &gz ) )
		return false;

	omega	<<	gx*3.14/180/2.0e-3,
				gy*3.14/180/2.0e-3,
				gz*3.14/180/3.3e-3;

	return true;
}

bool	Sensing::getMagns( Matrix<FT,3,1>	&m )
{
	float	x,y,z;
	if ( !dADS->convert( CH_MAGX, &x ) )
		return false;
	if ( !dADS->convert( CH_MAGY, &y ) )
		return false;
	if ( !dADS->convert( CH_MAGZ, &z ) )
		return false;

	m	<< x,-y,z;

	return true;
}

