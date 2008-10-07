/*
 *  MCP3208 12-bit A/D 'driver' through spidev
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


#ifndef _ad12_h_
#define	_ad12_h_

#include <stdint.h>

namespace input {

	/**
	 * Class for controlling a MCP3208 A/D converter through
	 * spidev.
	 *
	 * Asumes 5V ARef
	 */
	class	AD12
	{
		protected:
			const char	*spidevname;
			int			fd;	/* file descriptor */

		public:
			/**
			 * 
			 * @param device	spidev to use for communication
			 */
			AD12( const char *device );

			/**
			 * Try to initialize spidev and relevant functionality
			 *
			 * @return	true if OK, false if there was an error
			 */
			bool init(void);

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

