/*
 *  LIS3LV02 3-axis accelerometer 'driver'
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



#ifndef __lis3lv02_h_
#define	__lis3lv02_h_

#include <stdint.h>

namespace input
{
	/**
	 * Class for controlling a LIS3LV02 accelerometer
	 * through spidev
	 */
	class	LIS3LV02
	{
		private:
			uint8_t	awrite8( uint8_t addr, uint8_t val );
			uint8_t areadX( uint8_t addr, uint8_t *buf, uint16_t len);
			uint8_t	aread8( uint8_t addr );
 
		protected:
			const char	*spidevname;
			int	fd;	/* file descriptor for spi device */

		public:
			/**
			 *
			 * @param	_spidevname		spidev device name
			 */
			LIS3LV02( const char *_spidevname );

			/**
			 * Initialize SPI interface
			 *
			 * @param spiclk	spi clock to use with spidev
			 * @return	false on error
			 */
			bool	init(unsigned int spiclk);


			/**
			 * Get raw data, only for test purposes now
			 *
			 * @param iaccel	where to store the values
			 *
			 * @return	false if there was an error
			 */
			bool getRaw( int16_t iaccel[3] );

			/**
			 * Get accelerometer data for all axes
			 * Units are in 'g' (9.8m/s^2)
			 *
			 * @param accel	where to store the values
			 *
			 * @return	false if there was an error
			 */
			bool	getAccel( float accel[3] );
	};
};


#endif /** __list3lv02_h_ */

