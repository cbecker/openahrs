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




#include "lis3lv02.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <asm/types.h>
#include <linux/spi/spidev.h>

#include <iostream>
using namespace std;

#define	testbit(x,bit)	( (x) & (1<<(bit)) )

#define	OUTX_L	0x28
#define	OUTX_H	0x29
#define	CTRL_REG1	0x20
#define	CTRL_REG2	0x21
#define	OUTZ_L	0x2C
#define	OUTZ_H	0x2D



namespace input
{
	LIS3LV02::LIS3LV02( const char *_spidevname ) 
	{
		spidevname	= _spidevname;
	}


	uint8_t	LIS3LV02::awrite8( uint8_t addr, uint8_t val )
	{
		struct spi_ioc_transfer	xfer[2];
		unsigned char	buf[2];
		int status;

		memset( xfer,0,sizeof(xfer));


		buf[0] = /*write*/ (0<<7) | addr;
		buf[1] = val;

		xfer[0].tx_buf = (__u64) buf;
		xfer[0].len = 2;

		xfer[1].rx_buf = (__u64) buf;
		xfer[1].len = 0;
		
		status = ioctl( fd, SPI_IOC_MESSAGE(2), xfer );

		if ( status < 0 )
		{
			cout << "Error in ioctl" << endl;
			return 0;
		}

		return	buf[1];

	}

	uint8_t  LIS3LV02::areadX( uint8_t addr, uint8_t *buf, uint16_t len)
	{
		struct spi_ioc_transfer	xfer[2];
		unsigned char txb;
		int status;

		memset( xfer,0,sizeof(xfer));
		memset( buf,0, len);

		txb = /*read*/ (1<<7) | /*incr addr*/ (1<<6)  | addr;

		xfer[0].tx_buf = (__u64) &txb;
		xfer[0].len = 1;
		
		xfer[1].rx_buf = (__u64) buf;
		xfer[1].len = len;

		status = ioctl( fd, SPI_IOC_MESSAGE(2), xfer );

		if ( status < 0 )
		{
			cout << "Error in ioctl\n" << endl;
			return 0;
		}

		return 1;
	}


	uint8_t	 LIS3LV02::aread8( uint8_t addr )
	{
		struct spi_ioc_transfer	xfer[2];
		unsigned char	buf[2];
		int status;

		memset( xfer,0,sizeof(xfer));
		memset( buf,0,sizeof(buf));


		buf[0] = /*read*/ (1<<7) | addr;
		buf[1] = 0;

		xfer[0].tx_buf = (__u64) buf;
		xfer[0].len = 2;

		xfer[1].rx_buf = (__u64) buf;
		xfer[1].len = 2;

		status = ioctl( fd, SPI_IOC_MESSAGE(2), xfer );

		if ( status < 0 )
		{
			cout << "Error in ioctl" << endl;
			return 0;
		}

		return	buf[1];
	}



	bool LIS3LV02::init(void)
	{
		int ret;

		uint8_t mode = 0;
		uint8_t bits = 8;
		uint32_t speed = 1000000;
		//uint16_t delay = 0;

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


		/***
		 * Now try to find the device
		 */
		if ( aread8( 0x0F ) != 0x3A )
		{
			cout << "Couldn't find LIS3LV02!" << endl;
			return false;
		}

		//start
		awrite8( CTRL_REG1, (1<<7) | (1<<1) | (1<<2) | (1<<0) );	//all axes on

		//16 bit mode
		awrite8( CTRL_REG2, 1 	//16 bit mode
								| (1<<5)  //big endian
								| (1<<6)  //block update on
								| (0<<7)  //2g scale
											);



		return true;	//ok!
	}
	
	bool LIS3LV02::getAccel( float accel[3] )
	{
		int16_t	iaccel[3];
		areadX( OUTX_L, (uint8_t*)iaccel, sizeof(iaccel) );

		/* convert to 'g' */
		for (int i=0; i < 3; i++ ) {
			accel[i]	= iaccel[i]/16384.0;
		}
	
		return true;
	}
};

