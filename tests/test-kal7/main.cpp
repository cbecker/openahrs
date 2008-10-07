/*
 *	Test for 7-state kalman filter
 *	Octave output
 *
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




#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

#include <openAHRS/util/util.h>
#include <openAHRS/kalman/kalman7.h>
#include <openAHRS/util/octave.h>
#include <openAHRS/util/net.h>

#include "timer_this.h"


using namespace std;
using namespace openAHRS;
using namespace openAHRS::kalman7;

FT	dt	= 1.0/50;

/* number of points for test */
#define	N	2000

static const FT meas_variance = 0.01;
static struct
{
	Matrix<FT,3,1>	accels[N];
	Matrix<FT,3,1>	angles[N];
	Matrix<FT,3,1>	gyros[N];
	Matrix<FT,3,1>	realAngles[N];
	
} input;

static struct
{
	Matrix<FT,7,1>	state[N];
	Matrix<FT,3,1>	angles[N];
} output;

FT	limitPI( FT x )
{
	if ( ( x > C_PI ) && ( x <= 2*C_PI ) )
		return	x - 2*C_PI;
	else if ( ( x < -C_PI ) && ( x > -2*C_PI ) )
		return	2*C_PI + x;
	else
		return x;
}

Matrix<FT,3,1>	gyroBias;

void	makeTempData( bool add_noise )
{
	gyroBias	<< 3,5,7;


	FT	roll,pitch,yaw;
	roll = pitch = yaw = 0.0;

	FT	p,q,r;

	for ( int i=0; i < N; i++ )
	{
		/* this is angular speed for roll/pitch/yaw */
		p	= 0.03*sin(2*0.02*C_PI*i*dt);
		q	= 0.5*cos(2*0.2*C_PI*i*dt+0.3);
		r	= 0.1*cos(2*0.07*C_PI*i*dt + 0.14 );

		roll	+= p*dt;
		pitch	+= q*dt;
		yaw		+= r*dt;

		roll	=	limitPI( roll );
		pitch	=	limitPI( pitch );
		yaw		=	limitPI( yaw );

		input.realAngles[i]	<< roll, pitch, yaw;

		input.accels[i]	<< -9.8*sin(pitch), 
						   -9.8*sin(roll)*cos(pitch),
						   9.8*cos(roll)*cos(pitch);

		if ( add_noise )
			input.accels[i]	+= util::randomVector3( 0, sqrt(meas_variance) );

		/* this simulates what the software would do to sensor data */
		util::accelToPR( input.accels[i], input.angles[i] );
		input.angles[i](2)	= yaw;

		if ( add_noise )
			input.angles[i](2)	+= util::randomNormal()*sqrt(meas_variance);

		FT	temp = -input.accels[i][0]/input.accels[i].norm();
		if ( temp > 1 )		temp = 1;
		if ( temp < -1 )	temp = -1;
		input.angles[i](1)	= asin(temp);

		input.gyros[i]	<< p,q,r;

		if ( add_noise )
			input.gyros[i]	+= util::randomVector3( 0, sqrt(meas_variance) );

		input.gyros[i]	+= gyroBias;
	}
}

int main()
{
	Matrix<FT,3,1>	angle;
	Matrix<FT,3,1>	startBias;
	Matrix<FT,3,1>	gyros;

	makeTempData(true);

	angle		=	input.angles[0];
	cout << "Start angle:\n" << angle << endl << endl;

	/* initial bias */
	startBias	=	input.gyros[0];

	KalmanInit( angle, startBias, meas_variance,
					1e-2, 1e-5 );
	cout << angle << endl;

	cout << "Q\n" << util::eulerToQuat( angle ) << endl;
	int i;

//TIME_THIS("aa",

	for (i=0; i < N; i++ )
	{
		KalmanUpdate( i, input.angles[i], dt );
		
		output.state[i]	= X;
		
		KalmanPredict( i, input.gyros[i], true, dt );

	}
//);

	for (i=0 ; i < N ; i++ )
	{
		output.angles[i]	= util::quatToEuler( output.state[i].start<4>() );
	}

	cout << "End angle: \n" << output.angles[N-1] << endl;

	ofstream	file("octave/kaltest");

	octave::writeVectors( file, "X", output.state,N);
	octave::writeVectors( file, "angles", output.angles, N );
	octave::writeVectors( file, "in_angles", input.angles, N );
	octave::writeVectors( file, "in_gyros", input.gyros, N );
	octave::writeVectors( file, "in_realAngles", input.gyros, N );
	octave::writeVectors( file, "gyro_bias", &gyroBias, 1 );

	file.close();


	return 0;
}
