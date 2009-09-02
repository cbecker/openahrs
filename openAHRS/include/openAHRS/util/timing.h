/*
 *  Macros for loop timing
 *  Link with '-lrt'
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



#ifndef	_openahrs_timing_h_
#define	_openahrs_timing_h_

#include <time.h>

#define	TIME_THIS( que, X )				\
	{								\
		struct timespec ts1, ts2;	\
									\
		clock_gettime( CLOCK_REALTIME, &ts1 ); \
												\
		X;										\
												\
		clock_gettime( CLOCK_REALTIME, &ts2 );	\
												\
		printf( que " took: %f\n", (float) ( 1.0*(1.0*ts2.tv_nsec - ts1.tv_nsec*1.0)*1e-9 + 1.0*ts2.tv_sec - 1.0*ts1.tv_sec ) ); \
													\
	}

#endif

