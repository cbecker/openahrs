/*
 *  AHRS for AVR32.
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

#include <stdio.h>
#include "timer_this.h"
#include <iostream>

#include <openAHRS/util/util.h>
#include <openAHRS/kalman/UKFst7.h>
#include <openAHRS/util/net.h>
#include <openAHRS/util/matrixserializer.h>

using namespace std;
using namespace openAHRS;

#include "avr32hw.h"
#include "magcalib.h"

static MagCalib	calibM;
static Sensing	s;
static	util::UDPConnection	udp( "192.168.0.246", 4444 );

#define	USE_UKF	0
#if	USE_UKF
	#include <openAHRS/kalman/UKFst7.h>
#else
	#include <openAHRS/kalman/kalman7.h>
#endif


#if	USE_UKF
	static	openAHRS::UKFst7	K7;
#else
	static	openAHRS::kalman7	K7;
#endif

static struct timespec	t1,t2;

#include <Eigen/Geometry>

#define	MAGCALIB_FILENAME	"mag.cal"

/**
 * Gets elapsed time between t2 and t1,
 * in seconds
 */
static float	getElapsedTime( struct timespec *t1, struct timespec *t2 )
{
	return 1.0*(1.0*t2->tv_nsec - t1->tv_nsec*1.0)*1e-9 + 1.0*t2->tv_sec - 1.0*t1->tv_sec;
}

//takes quaternion, returns quaternion
static	Matrix<FT,4,1>	correct45Deg( Matrix<FT,4,1>	quat )
{
	static Eigen::Quaternion<FT>	qRot;	//quaternion used to rotate the measurements 45 degrees
	static bool	firstTime = true;
	if	(firstTime) {
		qRot.coeffs()	= util::eulerToQuat(  Matrix<FT,3,1>(0,0,-45*M_PI/180) );
		firstTime = false;
	}


	return (qRot * Eigen::Quaternion<FT>( quat ) ).coeffs();
}

static double	processMagn( const Matrix<FT,3,1>	&m, const Matrix<FT,3,1> &angles )
{
	Matrix<FT,3,1>	ang1;
	ang1 = angles;
	ang1(1) -= 32*M_PI/180;	//compensate for inclination (Argentina!)
							//TODO: get from GPS
	return	util::calcHeading( m, ang1 );
}

bool	testMagAccel()
{
	Matrix<FT,3,1>	a,mr,m,angles;
	
	while(1)
	{
		if ( !s.getMagns(mr) )	{ printf("Error get magn\n") ; return false; }
		if ( !s.getAccels(a) )	{ printf("Error get accel\n"); return false; }

		calibM.processInput(mr, m);

		util::accelToPR( a, angles );
		
		angles(2)	= processMagn( m, angles );

		angles	= util::quatToEuler( correct45Deg( util::eulerToQuat( angles ) ) );

		cout << "Mag: " << m << endl;
		cout << "Heading: " << angles(2)*180/3.14 << endl;
		cout << "Pitch & Roll: " << angles(1)*180/3.14 << " " << angles(0)*180/3.14 << endl;

		usleep(100e3);
		
		if ( util::kbhit() ) {
			getchar();
			break;
		}
	}

	return true;
}

bool	doFiltering()
{
	Matrix<FT,3,1>	a,g,mr,m, angles;
	Matrix<FT,3,1>	startBias;
	Matrix<FT,7,1>	X;

	startBias << 0,0,0;

	int i = 0;

	//init
		if ( !s.getGyros(g) )	{ printf("Error get gyros\n"); return -3; }
		if ( !s.getAccels(a) )	{ printf("Error get accel\n"); return -2; }
		if ( !s.getMagns(mr) )	{ printf("Error get magn\n"); return -4; }

		calibM.processInput(mr, m);

		util::accelToPR( a, angles );
		angles(2)	= processMagn( m, angles );

	#ifdef	KAL_DONT_USE_MAG
		g(2) = 0;
		angles(2) = 0;
	#endif

		startBias = g;

	#if	USE_UKF
		K7.KalmanInit( angles, startBias, 1e-1, 1e-8, 1e-12 );
	#else
		K7.KalmanInit( angles, startBias, 1e-2, 1e-4, 1e-7 );
	#endif

		clock_gettime( CLOCK_REALTIME, &t2 );


	while(1) {

		if ( !s.getGyros(g) )	{ printf("Error get gyros\n"); return -3; }
		if ( !s.getAccels(a) )	{ printf("Error get accel\n"); return -2; }
		if ( !s.getMagns(mr) )	{ printf("Error get magn\n"); return -4; }

		calibM.processInput(mr, m);

		util::accelToPR( a, angles );

		double rawHeading = angles(2)	= processMagn( m, angles );

		#ifdef	KAL_DONT_USE_MAG
			g(2) = 0;
			angles(2) = 0;
		#endif

		clock_gettime( CLOCK_REALTIME, &t1 );
		double	dt = getElapsedTime( &t2, &t1 );
		t2 = t1;

		K7.KalmanUpdate( i, angles, dt );
		K7.getStateVector(X);
		
		angles	= util::quatToEuler( correct45Deg( X.start<4>() ) );

		//cout << "angp: " << util::quatToEuler( X.start<4>() )*180/M_PI << endl;
		//cout << "angn: " << angles*180/M_PI << endl;

		/**
		 * Send data through network.
		 * This format avoids little/big endian problems
		 */
		static char	netStr[1024];
		sprintf(netStr,"Roll:%lf Pitch:%lf Yaw:%lf Bias1:%lf Bias2:%lf Bias3:%lf Ax:%lf Ay:%lf Az:%lf RH:%lf",
			angles(0), angles(1), angles(2),
			X(4), X(5), X(6),
			a(0), a(1), a(2),
			rawHeading );

		udp.Send( netStr, strlen(netStr) );
#if 1
		//show debug info once a while
		if ( i % 20 == 0 ) {
			cout << "dt: " << dt << endl;
			cout << "Roll : " << 180/3.14*angles(0) << endl;
			cout << "Pitch: " << 180/3.14*angles(1) << endl;
			cout << "Yaw:   " << 180/3.14*angles(2) << endl << endl;

			cout << "Raw yaw: " << 180/3.14*rawHeading << endl;
			cout << "Gyros: " << g << endl;
		}
#endif
		/////////////////
		K7.KalmanPredict( i, g, dt );

		if ( util::kbhit() )	{
			getchar();
			break;
		}
		usleep(1e3);
		i++;
	}

	return true;
}

bool	doMagCalibration()
{
	Matrix<FT,3,1>	mRaw,mCal;
	int i = 0;

	while(1)
	{
		i++;
		if ( !s.getMagns(mRaw) ) {
			printf("Err get mag\n");
			return false;
		}
		calibM.estimateParams(mRaw);
		calibM.processInput(mRaw, mCal);

		
		if ( i % 10 == 0 ) {
			s.flipMagns();
			Matrix<FT,9,1>	p; calibM.getParams(p);
			cout << "Params: \n" << p << "\n";
			cout << "Magn angle: " << 180/3.14*atan2(mCal(1),mCal(0)) << "\n\n\n";
		}

		if ( util::kbhit() ) {
			getchar();
			break;
		}

		usleep(5e3);
	}
	
	//save calibration
	if ( !calibM.saveParameters( MAGCALIB_FILENAME ) ) {
		printf("Error saving calibration parameters\n");
		return false;
	} else
		printf("Calibration parameters saved\n");

	return true;
}

int main(int argc, char **argv)
{

	if ( !s.init() ) {
		printf("Error init sensing\n"); return -1;
	}
	getchar();
	while(1)
	{
		printf("\n\n---=========== AVR32 AHRS =============----\n\n");
		printf("1:\tCalibrate magnetometers\n");
		printf("2:\tAlign accelerometer and magnetometers\n");
		printf("3:\tShow raw data\n");
		printf("4:\tStart filtering\n");
		printf("5:\tTest mag-accel relationship\n");
		printf("6:\tLoad mag calib data\n");
		printf("9:\tQuit\n");
		printf("\nYour choice: ");

		int c;
		c = getchar();
		printf("Char %d\n",c);
		getchar();	//enter
		switch(c) {
			case	'1':
				doMagCalibration();
				break;
			case	2:
				break;
			case	3:
				break;
			case	'5':
				testMagAccel();
				break;
			case	'4':
				doFiltering();
				break;
			case	'6':
				if ( !calibM.loadParameters( MAGCALIB_FILENAME ) ) {
					printf("Error loading calibration data\n"); break;
				} else
					printf("Calibration data loaded\n");
				break;
			case	'9':
				printf("--- Exiting....\n");
				return 0;
				break;
		}
	}

#if 0
	while(1)
	{
		if ( !s.getAccels(a) )	return -2;
		if ( !s.getGyros(g) )	return -3;
		if ( !s.getMagns(m) )	return -4;

		mcalib.estimateParams(m);
		mcalib.processInput(m, mc);

		if ( i % 10 == 0 ) {
			cout << "Accel: \n" << a << "\n\n";
			cout << "Gyro: \n" << g << "\n\n";
			cout << "Magn: \n" << m << "\n\n";

			Matrix<FT,9,1>	p; mcalib.getParams(p);
			cout << "Params: \n" << p << "\n";
			cout << "Magn angle: " << 180/3.14*atan2(mc(1),mc(0)) << "\n\n\n";
		}
		usleep(5e3);

		i++;
	}
#endif
	return 0;
}

