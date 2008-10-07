/*
 *  mmap abstraction
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
#include <sys/mman.h>
#include <malloc.h>
#include <unistd.h>
#include <fcntl.h>

#include "mmap.h"

namespace input {

	
	void *DoMMap( unsigned int IOM_BASE, unsigned int IOM_SIZE )
	{
	/* Page size used by the kernel on the MMU
	 * It is needed since mmap asks for a multiple
	 * of PAGE_SIZE as the base address				*/
		int	PAGE_SIZE	= getpagesize();

		// /dev/mem file descriptor
		int mem_fd;

		// open /dev/mem
		mem_fd = open("/dev/mem", O_RDWR | O_SYNC );
		if ( mem_fd < 0 )
		{
			printf("Error opening /dev/mem\n");
			return NULL;
		}

		unsigned long *io_mem = NULL;

		unsigned long int	iom_base = IOM_BASE;
		unsigned long int	iom_offset = 0;

		//fix to fit a page start address
		iom_base -= IOM_BASE % PAGE_SIZE;

		/* offset of IOM_BASE with respect to
		 * iom_base							 */
		iom_offset = IOM_BASE % PAGE_SIZE;

		// print some debug info
		#if 0
			printf("Start address request: %lu\n", iom_base );
			printf("Offset %lu\n", iom_offset );
			printf("Page size is %d\n", PAGE_SIZE );
		#endif

		// call mmap()
		io_mem = (unsigned long*) mmap( NULL,
								IOM_SIZE + iom_offset ,
								PROT_READ | PROT_WRITE,
								MAP_SHARED,
								mem_fd,
								iom_base );


		// succeeded?
		if ( io_mem == MAP_FAILED )
		{
			printf("mmap couldn't success!\n");

			return NULL;
		}

		return ((void*)( ((unsigned int)io_mem) + iom_offset ) );
	}

};
