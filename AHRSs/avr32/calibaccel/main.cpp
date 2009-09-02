/*
 *  Accelerometer calibration test for AVR23 AHRS
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



#include <openAHRS/calib/Sphere.h>
#include <openAHRS/calib/Ellipsoid.h>
#include <openAHRS/calib/UKFEllipsoid.h>
//#include <openAHRS/calib/UKFEllipsoid2.h>

#include <openAHRS/util/util.h>
#include <openAHRS/util/octave.h>

#include <stdio.h>
#include <math.h>

#include <iostream>

#include "../input/lis3lv02.h"

#define	LOG			0
#define	USE_SPHERE	0

#if USE_SPHERE
	static openAHRS::calib::Sphere		SP;
#else
	static openAHRS::calib::UKFEllipsoid	SP;
	//static openAHRS::calib::Ellipsoid	SP;
#endif

using namespace openAHRS;
using namespace input;
using namespace std;

Matrix<FT,3,1>	raw;

#define	N	200
Matrix<FT,3,1>	rawData[N];


/** returns heading 
 * Angles need to have right pitch and roll **/
static void	getAccel( LIS3LV02 &ac, Matrix<FT,3,1> &out )
{
	static bool	firstTime = true;
	Matrix<FT,3,1>	meas;
	Matrix<FT,3,1> estBias;

	float	samples[3];
	if ( !ac.getAccel( samples ) )
		printf(" ERRR get accels\n" );

	meas << samples[0], samples[1], samples[2];

	raw = meas;

	/** process calibr **/
	if ( firstTime ) {
		estBias << 0, 0, 0;
		#if !USE_SPHERE
			SP.init( 1.0, estBias, 1.0, 1e-9, 1e-12);
			//SP.init( 1, estBias, 1, 1e-9,1e-12);
		#else
			SP.init( 1e-2, estBias);
		#endif
		firstTime = false;
	}

	SP.estimateParams( meas );

	SP.processInput( meas, out );
}



Matrix<FT,3,1> res;

#if !USE_SPHERE
	Matrix<FT,9,1>	st;
#else
	Matrix<FT,4,1>	st;
#endif

FT mmax,mmin;
int main()
{
	mmin = 4;
	mmax = 0;


	LIS3LV02	dev_accel("/dev/spidev0.1");
	if (!dev_accel.init()) {
		printf("Error accel init\n");
		exit(-1);
	}
	
	//printf(" hola\n" );
	//getchar();
#if LOG
	for (int i=0; i < N; i++)
	{
#else
	for (int i=0; ; i++)
	{
#endif
#if 1

	getAccel( dev_accel,res );

		if ( mmax < raw(0) )
			mmax = raw(0);
		if ( mmin > raw(0) )
			mmin = raw(0);
		
		if ( (i % 20) == 0 ) {
			printf("I:%d\n" , i );
			printf("Min: %.10f\n", mmin );
			printf("Max: %.10f\n", mmax );

			cout << "---------- Raw:\n" << raw << endl;
			cout << "---------- Res:\n" << res << endl;
			cout << "---------- State:\n";
			SP.getStateVector(st); 
			cout << st << std::endl << std::endl;

		}
		#if LOG
			rawData[i] = raw;
		#endif
		usleep(5000);
	}
	ofstream	file("octave/rawmag");
	octave::writeVectors( file, "rawmag", rawData, N );
	file.close();
#endif

	return 0;

}


