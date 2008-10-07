/*
 *  Coil flipping utility for magnetometers
 *  Uses AP32AP700's PORTA and some mosfets
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



#include "cflip.h"
#include "mmap.h"

#include <stdio.h>


/**
 * Flipping pins are on PORTA
 */
#define	PORTA_BASE	0xFFE02800

#define	PAREG(x)	*((volatile unsigned int *) ( ((unsigned int)mmaped)+(x)) )
#define	PORTA_PER	PAREG(0x0000)
#define	PORTA_OER	PAREG(0x0010)
#define	PORTA_SODR	PAREG(0x0030)
#define	PORTA_CODR	PAREG(0x0034)

/**
 * Flip bits for each magn axis
 */
#define	FLIPX	8
#define	FLIPY	9
#define	FLIPZ	23


namespace input {
	
	namespace CFlip {
		
		void *mmaped  = NULL;
		int	bits;

		bool	init()
		{
			mmaped = input::DoMMap( PORTA_BASE, 4096 );
			if ( mmaped == NULL )
				return false;

			/* PIO enable */
			PORTA_PER	= (1<<FLIPX) | (1<<FLIPY) | (1<<FLIPZ);

			/** as output **/
			PORTA_OER	= (1<<FLIPX) | (1<<FLIPY) | (1<<FLIPZ);

			bits	= (1<<FLIPX) | (1<<FLIPY) | (1<<FLIPZ) ;
			
			flipClear();

			return true;
		}

		void	flipSet(void)
		{
			PORTA_CODR	= bits;
		}

		void	flipClear(void)
		{
			PORTA_SODR	= bits;
		}
	};

};


