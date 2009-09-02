/*
 *  Test program for Ellipsoid Calibration
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


#include <openAHRS/calib/Ellipsoid.h>
#include <openAHRS/util/util.h>
#include <openAHRS/util/octave.h>

#include <stdio.h>
#include <math.h>

using namespace openAHRS;

/** how many points for test data */
#define	N	20000

/** optimal sphere values */
#define	SPHERE_RADIUS	11
#define	SPHERE_X		7
#define	SPHERE_Y		3
#define	SPHERE_Z		5

/** if the sphere should be distorted when generating
 * sample input data */
#define	ADD_DISTORTION	1

/* if we should add noise to measurements */
#define	ADD_NOISE	1
FT	noiseStdDev	= 0.01;	//if adding noise, which is its std dev

Matrix<FT,3,1>	genMeas[N];	//generated measurements
Matrix<FT,9,1>	calibState[N];	//calibrator states


void	genInputData()
{
	FT	theta, phi;

	FT dist1,dist2,dist3;
	#if ADD_DISTORTION
		dist1 = 1.01; dist2 = 1.03; dist3 = 0.98;
	#else
		dist1 = dist2 = dist3 = 1.0;
	#endif

	for (int i=0; i < N; i++)
	{
		theta = -C_PI + 2*C_PI*util::randomNormal();
		phi	=	-C_PI + 2*C_PI*util::randomNormal();

		genMeas[i](0)	= SPHERE_X + dist1*SPHERE_RADIUS*cos(theta)*sin(phi);
		genMeas[i](1)	= SPHERE_Y + dist2*SPHERE_RADIUS*sin(theta)*sin(phi);
		genMeas[i](2)	= SPHERE_Z + dist3*SPHERE_RADIUS*cos(phi);

		#if ADD_NOISE
			genMeas[i](0) += noiseStdDev*util::randomNormal();
			genMeas[i](1) += noiseStdDev*util::randomNormal();
			genMeas[i](2) += noiseStdDev*util::randomNormal();
		#endif
	}
}

int main()
{
	genInputData();	//generate random data

	openAHRS::calib::Ellipsoid	EL;

	for (int i=0; i < N; i++)
	{
		if ( i == 0 )
			EL.init( 1, genMeas[i], SPHERE_RADIUS, 1e-9, 1e-9 );

		EL.estimateParams( genMeas[i] );

		EL.getStateVector(calibState[i]);
	}

	/** save octave file **/
	ofstream	file("octave/calibtest");

	octave::writeVectors( file, "meas", genMeas, N );
	octave::writeVectors( file, "state", calibState, N );

	Matrix<FT,4,1>	trueData;
	trueData(0) = SPHERE_RADIUS;
	trueData(1) = SPHERE_X;
	trueData(2) = SPHERE_Y;
	trueData(3) = SPHERE_Z;
	octave::writeVectors( file, "trueData", &trueData, 1 );

	file.close();

	return 0;
}

