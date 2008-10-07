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




#include "ad12.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <asm/types.h>
#include <linux/spi/spidev.h>

#include <iostream>

#define	testbit(x,bit)	( (x) & (1<<(bit)) )

using namespace std;

namespace input 
{
	AD12::AD12( const char *spidev ) 
	{
		spidevname	= spidev;
	}

	bool AD12::init(void) 
	{
		int ret;

		uint8_t mode = 0;
		uint8_t bits = 8;
		uint32_t speed = 1000000;

		fd = open(spidevname, O_RDWR);
		if (fd < 0) {
			cout << "can't open device" << endl;
			return false;
		}

		/*
		 * spi mode
		 */
		ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (ret == -1) {
			cout << "can't set spi mode" << endl;
			return false;
		}

		ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
		if (ret == -1) {
			cout << "can't get spi mode" << endl;
			return false;
		}

		/*
		 * bits per word
		 */
		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
		if (ret == -1) {
			cout << "can't set bits per word" << endl;
			return false;
		}

		ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
		if (ret == -1) {
			cout << "can't get bits per word";
			return false;
		}

		/*
		 * max speed hz
		 */
		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
		if (ret == -1) {
			cout << "can't set max speed hz" << endl;
			return false;
		}

		ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
		if (ret == -1) {
			cout << "can't get max speed hz" << endl;
		}

		return true;	//ok!
	}

	bool AD12::getSample( uint8_t channel, float *result ) 
	{
		struct spi_ioc_transfer	xfer[2];
		unsigned char txb[5];	//1+2+2
		int status;

		uint16_t	ret = 0;
		int i;


		memset( xfer,0,sizeof(xfer));

		txb[0] = /*start*/ (1<<2) | /*single*/(1<<1) | (channel>>2);
		txb[1] = (channel & 0x3) << 6;
		txb[2] = 0;

		xfer[0].tx_buf = (__u32) txb;
		xfer[0].rx_buf = (__u32) txb;	//full duplex
		
		xfer[0].len = sizeof(txb);
		

		status = ioctl( fd, SPI_IOC_MESSAGE(1), xfer );

		/** now:
		 *	txb[1,2] => MSB a LSB
		 *	txb[3,4] => LSB a MSB (repeated, util for verification)
		 */

		if ( status < 0 )
		{
			cout << "Error at ioctl" << endl;
			return false;
		}

		txb[1] &= 0xFF >> 3;	//para sacar los MSB que no van
		
		//txb[3] <<= 1;	//el B0 no está en la retransmisión
		txb[4] &= 0xFF << 3;	//también.

		//check LSB
		for (i=1; i < 8; i++ )	//el primero no se chequea
		{
			if ( testbit( txb[2], i ) && ( !testbit( txb[3], 8-i ) ) )
				return false;
			if ( (!testbit( txb[2], i )) &&  testbit( txb[3], 8-i ) )
				return false;
		}


		//check MSB
		for (i=0; i < 12-8-1; i++ )
		{
			if ( testbit( txb[1], i+1 ) && ( !testbit( txb[4], 7-i ) ) )
				return false;
			if ( (!testbit( txb[1], i+1 )) &&  testbit( txb[4], 7-i ) )
				return false;
		}

		//queda 1 que es re loco..
		if ( testbit( txb[1], 0 ) !=  testbit( txb[3], 0 ) )
			return false;

		ret = txb[1]*256 + txb[2];

		if ( ret > 4095 )	//error!
		{
			cout << "Error on data from AD" << endl;
			return false;
		}

		if ( result != NULL ) {
			*result = 5.0*ret/4096;
		}

		return true;
	}

};

