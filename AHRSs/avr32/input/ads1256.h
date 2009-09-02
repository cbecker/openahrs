/*
 *  ADS1256 A/D 'driver' through spidev
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


#ifndef _ads1256_h_
#define	_ads1256_h_

#include <stdint.h>

namespace input {

	/**
	 * Class for controlling a MCP3208 A/D converter through
	 * spidev.
	 *
	 * Asumes 5V ARef
	 */
	class	ADS1256
	{
		protected:
			const char	*spidevname;
			int			fd;	/* file descriptor */
			float		vRef;

			/**
			 * Reads a register from the A/D.
			 * true if alright
			 */
			bool	readReg( unsigned char reg, unsigned char *val );

			bool setMux( int ch1, int ch2 ) ;
			/**
			 * Reads conversion data
			 */
			bool	readData( int *val );

			/**
			 * Writes register data
			 */
			bool	writeReg( unsigned char reg, unsigned char val, bool verify = false );
			
			/**
			 * Writes single-byte command
			 */
			bool	writeCmd( unsigned char cmd );
		public:
			/**
			 * 
			 * @param device	spidev to use for communication
			 * @param vref		reference voltage
			 */
			ADS1256( const char *device, float vref );

			bool test();

			bool	convert( int ch1, int ch2, float *val ) ;

			/**
			 * Try to initialize spidev and relevant functionality
			 *
			 * @param spiclk	spi clock
			 * @return	true if OK, false if there was an error
			 */
			bool init(unsigned int spiclk);

			//sets IO as output and sets pin value
			bool	setIO( unsigned int pin, int high );

			/**
			 * Gets sample
			 *
			 * @param channel	AD channel to acquire
			 * @param result	where to store data,
			 *					converted to Volts
			 */
			bool getSample( uint8_t channel, float *result );
	};
};

#endif

