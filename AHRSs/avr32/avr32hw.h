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


#ifndef	_avr32hw_h_
#define	_avr32hw_h_

/**
 * AVR32 hardware information
 */


/**
 * SPI devices for A/Ds, accels, etc
 */

#define	AVR32_SPIDEV_ADS1256	"/dev/spidev0.2"
#define	AVR32_SPIDEV_LIS3LV02	"/dev/spidev0.1"
#define	AVR32_SPIDEV_MCP3208	"/dev/spidev0.3"

/* spi clocks ***/
#define	AVR32_ADS1256_SPICLK	400000
#define	AVR32_MCP3208_SPICLK	3000000
#define	AVR32_LIS3LV02_SPICLK	3000000

/** Mag set/reset coils, controlled through ADS1256 **/
#define	AVR32_ADS1256_COILSET	2
#define	AVR32_ADS1256_COILCLR	3

//Autozero for 2-axis gyro, controlled through ADS1256 **/
#define	AVR32_ADS1256_AUTOZERO	1

//reference voltages in volts
#define	AVR32_MCP3208_VREF		3.3
#define	AVR32_ADS1256_VREF		0.57

/**
 * A/D channels for MCP3208
 */
#define	CH_GYROZ	7

#if	1
	#define	CH_GYROY	5
	#define	CH_GYROX	1
#else
	#define	CH_GYROY	6
	#define	CH_GYROX	0
#endif

#define	CH_TEMP		3
#define	CH_VREF		4


/** ADS1256, differential **/
#define	CH_MAGX		1,0
#define	CH_MAGY		3,2
#define	CH_MAGZ		7,6

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

#include "input/lis3lv02.h"
#include "input/ad12.h"
#include "input/ads1256.h"

class	Sensing
{
private:
	input::LIS3LV02	*dAccel;
	input::AD12		*d12;
	input::ADS1256		*dADS;

public:
	Sensing() { 
		dAccel = 0;
		d12 = 0;
		dADS = 0; }

	~Sensing() { if ( dAccel ) 
					delete dAccel; 
				if ( d12 ) 
					delete d12; 
				if ( dADS ) 
					delete dADS; 
				}

	bool	init();
	bool	getAccels( Matrix<FT,3,1>	&a );
	bool	getMagns( Matrix<FT,3,1>	&m );
	bool	getGyros( Matrix<FT,3,1>	&g );

	bool	flipMagns();	//flip/reset magnetometer coils
};

#endif

