#ifndef	_timer_this_h_
#define	_timer_this_h_

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
		printf( que " demora: %f\n", (float) ( 1.0*(1.0*ts2.tv_nsec - ts1.tv_nsec*1.0)*1e-9 + 1.0*ts2.tv_sec - 1.0*ts1.tv_sec ) ); \
													\
	}

#endif

