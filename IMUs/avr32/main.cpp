/*
 *  IMU for AVR32.
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





#include "input/lis3lv02.h"
#include "input/ad12.h"
#include "input/cflip.h"

#include "timer_this.h"

/**
 * If DEBUG_OCTAVE is not zero then N measurements are done and processed,
 * then saved to an octave file for analisys.
 *
 * Make sure the folder 'octave' exists on the same directory the application is run on
 *
 * Otherwise the program will never end and will keep sending data through UDP
 */
#define	DEBUG_OCTAVE	1

#if		DEBUG_OCTAVE
	#include <openIMU/util/octave.h>

	//number of points for octave debugging
	#define	N	1000
#endif


#include <openIMU/util/util.h>
#include <openIMU/kalman/kalman7.h>
#include <openIMU/util/net.h>

#include <iostream>

using namespace std;
using namespace input;
using namespace openIMU;
using namespace	openIMU::kalman7;

/**
 * A/D channels
 */
#define	CH_GYROZ	3
#define	CH_GYROY	2
#define	CH_GYROX	1

#define	CH_MAGX		5
#define	CH_MAGY		4
#define	CH_MAGZ		6


#if	DEBUG_OCTAVE
	/**
	 * Input data
	 */
	static struct
	{
		Matrix<FT,3,1>	accels[N];
		Matrix<FT,3,1>	gyros[N];
		Matrix<FT,3,1>	angles[N];
	} in;

	/**
	 * Output data
	 */
	static struct
	{
		Matrix<FT,7,1>	state[N];
		Matrix<FT,3,1>	angles[N];
		Matrix<FT,1,1>	dt[N];
	} out;
#endif


/**
 * Sampling time is not fixed,
 * depends on many things
 */
static float	dt;

/**
 * Time structures, used later
 */
static struct timespec t1,t2;


/** temp vectors, measured data **/
static Matrix<FT,3,1>		meas_accels,meas_gyros, meas_angles;
static Matrix<FT,3,1>		meas_magn;

/* filtered data */
static Matrix<FT,3,1>		angles;



/**
 * Gets elapsed time between t2 and t1,
 * in seconds
 */
static float	getElapsedTime( struct timespec *t1, struct timespec *t2 )
{
	return 1.0*(1.0*t2->tv_nsec - t1->tv_nsec*1.0)*1e-9 + 1.0*t2->tv_sec - 1.0*t1->tv_sec;
}

/**
 * Structure to hold calibration data
 */
struct sstat
{
	float min, max;

	sstat( float mn, float mx ) {
		min = mn;
		max = mx;
	}
};

/**
 * Init calibration struct
 */
void	init_stat( sstat *s )
{
	s->min = 9999999;
	s->max	= -9999999;
}

/**
 * Call to update limits on calibration struct
 * when calibrating
 */
void	do_stat( sstat *s, float val )
{
	if ( val < s->min )
		s->min = val;
	if ( val > s->max )
		s->max = val;

	printf("min: %.15f\n", s->min );
	printf("max: %.15f\n", s->max );
}

/**
 * Calculate value based on calibration data
 */
float	calc_val( const sstat *s, float val )
{
	float center = s->min + (s->max - s->min)/2;
	return	(val - center)/(s->max - center);
}


/** returns heading 
 * Angles need to have right pitch and roll **/
static float	getMagn( AD12 &ad, const Matrix<FT,3,1> &angles )
{
	float	xsample,ysample,zsample;

	/* Magnetometer and A/D calibration data **/
	static const sstat mx(
		2.490234375,		//min
		2.835034179680000	//max
	);

	static const sstat my(
		5.0-2.581787109375,	//min
		5.0-2.230224609375	//max
	);

	static const sstat mz(
		5.0 - 2.340087890625,	// min
		5.0 - 1.968994140625	// max
	);

	/**
	 * Magnetometers need resetting to avoid polarization
	 *
	 * Only once every some time
	 *
	 * TODO: this might not be the best place for this
	 */
	static int num = 0;
	num++;
	if ( num > 50 )
		num = 0;

	if ( num == 1 )
	{
		CFlip::flipClear();
		usleep(1000);

		CFlip::flipSet();
		usleep(1000);
	}


	if ( !ad.getSample( CH_MAGX, &xsample ) )
		cout	<< "Error get mX" << endl;
	if ( !ad.getSample( CH_MAGY, &ysample ) )
		cout	<< "Error get mY" << endl;
	if ( !ad.getSample( CH_MAGZ, &zsample ) )
		cout	<< "Error get mZ" << endl;

	ysample	= 5.0 - ysample;
	zsample = 5.0 - zsample;
	
	meas_magn[0]	= calc_val( &mx, xsample );
	meas_magn[1]	= calc_val( &my, ysample );
	meas_magn[2]	= calc_val( &mz, zsample );

	return util::calcHeading( meas_magn, angles );
}


/**
 * Gets accel and gyro data
 */
static void	getData( AD12 &ad, LIS3LV02 &accel, Matrix<FT,3,1> &accels, Matrix<FT,3,1> &gyros )
{
	float	faccels[3];
	float	xsample,ysample,zsample;

	if ( !accel.getAccel( faccels ) )
		cout << "Error get accels!" << endl;

	if ( !ad.getSample( CH_GYROX, &xsample ) )
		cout << "Error get X" << endl;
	if ( !ad.getSample( CH_GYROY, &ysample ) )
		cout << "Error get Y" << endl;
	if ( !ad.getSample( CH_GYROZ, &zsample ) )
		cout << "Error get Z" << endl;

	accels	<< faccels[2], -faccels[1], faccels[0];

	gyros	<<	-xsample*3.14/180/5e-3,	// ADXRS300	, this leads to negative bias, 
										// but won't matter, kalman will track it
										
										// ADXRS300
				ysample*3.14/180/5e-3,	
				-zsample*(3.41/180/15e-3);	// ADXRS150

/*	cout << "ACCELS: " << endl;
	cout << "\tX: " << faccels[2] << endl;
	cout << "\tY: " << faccels[1] << endl;
	cout << "\tZ: " << faccels[0] << endl;

	cout << "GYROS: " << endl;
	cout << "\tRoll : " << (5.0 - xsample) << endl;
	cout << "\tPitch: " << ysample << endl;
	cout << "\tYaw  : " << (5.0 - zsample) << endl <<endl;*/


}



int main( int argc, char *argv[] )
{
	Matrix<FT,3,1>	startBias;

	if ( argc != 2 ) {
		cout << "Wrong arguments" << endl << endl;
		cout << "Use: " << argv[0] << " " << "<host>" << endl;
		cout << "Packets will be sent through UDP, port 4444" << endl;
		return -1;
	}

	/* To output data through the network interface
	 * TODO: there is no host error checking */
	util::UDPConnection	conn( argv[1] ,4444);

	/**
	 * Try to initialize spi devices */
	AD12		dev_ad("/dev/spidev0.1");
	LIS3LV02	dev_accel("/dev/spidev0.2");
	if ( !dev_ad.init() ) {
		cout << "Error AD init" << endl;
		return -1;
	}

	if ( !dev_accel.init() ) {
		cout << "Error Accel init" << endl;
		return -1;
	}

	if ( !CFlip::init() ) {
		cout	<< "Error init cflip" << endl;
		return -1;
	}

/**
 * Uncomment to get calibration data for magnetometers
 */
#if 0
	sstat mx,my,mz;


		init_stat( &mx );
		init_stat( &my );
		init_stat( &mz );
	while(1)
	{
		CFlip::flipClear();
		usleep(1000);

		float xsample, ysample, zsample;

		if ( !ad.getSample( CH_MAGX, &xsample ) )
			cout	<< "Error get mX" << endl;
		if ( !ad.getSample( CH_MAGY, &ysample ) )
			cout	<< "Error get mY" << endl;
		if ( !ad.getSample( CH_MAGZ, &zsample ) )
			cout	<< "Error get mZ" << endl;

		CFlip::flipSet();



		cout << "X\n";
		do_stat( &mx, xsample );

		cout << "Y\n";
		do_stat( &my, ysample );
		cout << "Z\n";
		do_stat( &mz, zsample );

		cout << "X: " << xsample << endl;
		cout << "Y: " << ysample << endl;
		cout << "Z: " << zsample << endl;

		cout << endl << endl;

		usleep(10000);
	}
#endif



	/** Prepare initial estimate **/

	getData( dev_ad, dev_accel, meas_accels, meas_gyros );
	util::accelToPR( meas_accels, meas_angles );

	meas_angles(2)	= getMagn( dev_ad, meas_angles );

	/* lets provide an initial estimate */
	startBias	= meas_gyros;
	KalmanInit( meas_angles, startBias, 0.01,
					1e-4, 1e-7);



	clock_gettime( CLOCK_REALTIME, &t2 );
	#if	DEBUG_OCTAVE
		for (int i=0; i < N; i++ )
	#else
		for(int i=0;;i++)
	#endif
	{
		
		getData( dev_ad, dev_accel, meas_accels, meas_gyros );
		util::accelToPR( meas_accels, meas_angles );
		
		/** calculate yaw according to current magnetometer data and
		 *  previous angles */
		meas_angles(2)	= getMagn( dev_ad, angles );
	
		clock_gettime( CLOCK_REALTIME, &t1 );
		dt	= getElapsedTime( &t2, &t1 );
		t2 = t1;

		#if DEBUG_OCTAVE
			out.dt[i]	<< dt;
		#endif


		KalmanUpdate( i, meas_angles, dt );

		angles	= util::quatToEuler( X.start<4>() );

		#if	DEBUG_OCTAVE
			in.accels[i]	= meas_accels;
			in.angles[i]	= meas_angles;
			in.gyros[i]		= meas_gyros;

			out.state[i]	= X;
			out.angles[i]	= angles;
		#endif

		/**
		 * Send data through network.
		 * This format avoids little/big endian problems
		 */
		static char	netStr[512];
		sprintf(netStr,"Roll:%lf Pitch:%lf Yaw:%lf Bias1:%lf Bias2:%lf Bias3:%lf",
			angles(0), angles(1), angles(2),
			X(4), X(5), X(6) );

		conn.Send( netStr, strlen(netStr) );

		KalmanPredict( i, meas_gyros, true, dt );

		//show debug info once a while
		if ( i % 10 == 0 ) {
			cout << "dt: " << dt << endl;
			cout << "Roll : " << 180/3.14*angles(0) << endl;
			cout << "Pitch: " << 180/3.14*angles(1) << endl;
			cout << "Yaw:   " << 180/3.14*angles(2) << endl << endl;
		}


		usleep(10000);
	}


	#if DEBUG_OCTAVE
		/** Write original and filter data to Octave file **/
		ofstream	file("octave/indata");
		octave::writeVectors( file, "in_accels", in.accels, N );
		octave::writeVectors( file, "in_gyros", in.gyros, N );
		octave::writeVectors( file, "in_angles", in.angles, N );

		octave::writeVectors( file, "out_state", out.state, N );
		octave::writeVectors( file, "out_angles", out.angles, N  );
		octave::writeVectors( file, "dt", out.dt, N  );

		file.close();
	#endif

	return 0;
}

