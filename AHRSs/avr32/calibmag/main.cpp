/*
 *  Magnetometer calibration test for AVR32
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

#include <openAHRS/util/util.h>
#include <openAHRS/util/octave.h>

#include <stdio.h>
#include <math.h>

#include <iostream>

#include "../input/ads1256.h"
#include "../avr32hw.h"

#define	USE_SPHERE	0
#if USE_SPHERE
	static openAHRS::calib::Sphere		SP;
#else
	//static openAHRS::calib::Ellipsoid	SP;
	static openAHRS::calib::UKFEllipsoid	SP;
#endif

using namespace openAHRS;
using namespace input;
using namespace std;

Matrix<FT,3,1>	raw;

#define	LOG	0
#define	N	2000
Matrix<FT,3,1>	rawData[N];


/** returns heading 
 * Angles need to have right pitch and roll **/
static void	getMagn( ADS1256 &ad, Matrix<FT,3,1> &out )
{
	Matrix<FT,3,1>	meas;
	Matrix<FT,3,1> estBias;
	
	float	xsample,ysample,zsample;

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
		/*CFlip::flipClear();
		usleep(1000);

		CFlip::flipSet();
		usleep(1000);*/
	}


	if ( !ad.convert( 1,0, &xsample ) )
		cout	<< "Error get mX" << endl;
	if ( !ad.convert( 3,2, &ysample ) )
		cout	<< "Error get mY" << endl;
	if ( !ad.convert( 7,6, &zsample ) )
		cout	<< "Error get mZ" << endl;

	meas << xsample, ysample, zsample;

	raw = meas;

	/** process calibr **/
	static bool	firstTime = true;
	if ( firstTime ) {
		estBias << 0,0,0;
		//estBias << -5.6e-5,-213e-6,-139e-6;
		//SP.init( 1e-2, estBias, 0.2, 1e-6, 1e-9);
		#if	USE_SPHERE
			SP.init( 1e-5, 0.1e-3, estBias);
		#else
			SP.init( 1e-5, estBias, 0.11e-3, 0,0,1e-12 );
		#endif
		firstTime = false;
	}

	SP.estimateParams( meas );

	return	SP.processInput( meas, out );
}



Matrix<FT,3,1> res;
#if	USE_SPHERE
	Matrix<FT,4,1>	st;
#else
	Matrix<FT,9,1>	st;
#endif

FT mmax[3],mmin[3];

int main()
{
	int i;
	for (i=0; i < 3; i++) {
		mmax[i] = -4;
		mmin[i] = 4;
	}

	ADS1256	devAD( AVR32_SPIDEV_ADS1256, AVR32_ADS1256_VREF );
	if ( !devAD.init(AVR32_ADS1256_SPICLK) ) {
		printf(" Error init AD\n" );
		return -1;
	}

	devAD.test();

	//set & clear IO
	if ( !devAD.setIO( AVR32_ADS1256_COILSET, 0 ) ) {
		printf("Error setio1\n"); return -1;
	}
	if ( !devAD.setIO( AVR32_ADS1256_COILCLR, 1 ) ) {
		printf("Error setio1\n"); return -1;
	}

	printf("Ahora\n");getchar();
	usleep(10e3);

	if ( !devAD.setIO( AVR32_ADS1256_COILSET, 1 ) ) {
		printf("Error setio1\n"); return -1;
	}
	if ( !devAD.setIO( AVR32_ADS1256_COILCLR, 0 ) ) {
		printf("Error setio1\n"); return -1;
	}
	printf("Despues\n");getchar();
	usleep(10e3);

	while(0)
	{
		float val;
//		usleep(10000);
		if  ( ! devAD.convert( CH_MAGX, &val ) )
			printf("Err\n");
		else
			printf("%f\t", val );

		if  ( ! devAD.convert( CH_MAGY, &val ) )
			printf("Err\n");
		else
			printf("%f\t", val );

		if  ( ! devAD.convert( CH_MAGZ, &val ) )
			printf("Err\n");
		else
			printf("%f\t\n", val );

	}


	/*if  (!CFlip::init()) {
		printf(" Error init cflip!\n");
		return -2;
	}*/

	
#if LOG
	for (int i=0; i < N; i++)
#else
	for (int i=0; ; i++)
#endif
	{
		int k;

		getMagn( devAD, res );
		for (k=0; k < 3; k++) {
			if ( mmax[k] < raw(k) )
				mmax[k] = raw(k);
			if ( mmin[k] > raw(k) )
				mmin[k] = raw(k);
		}
			
		if ( (i % 20) == 0 ) {
			for (k=0; k < 3; k++) {
				printf("Diff %d: %lf\n", k, (double)(mmax[k]-mmin[k]));
			}

			printf("Ang: %f\n", (float)180/3.14*atan2(res(1),res(0)));

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

	return 0;

}


