/*
 *  Test program for Ellipsoid Calibration (UKF)
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


#include <openAHRS/calib/UKFEllipsoid.h>
#include <openAHRS/calib/Ellipsoid.h>
#include <openAHRS/util/util.h>
#include <openAHRS/util/octave.h>
#include <openAHRS/util/timing.h>

#include <stdio.h>
#include <math.h>

using namespace openAHRS;

/** how many points for test data */
#define	N	10000

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
Matrix<FT,4,1>	trueData;

bool	loadInputData( const char *file, const char *varname )
{
	ifstream	sfile(file);

	int nread = 0;
	MatrixXd *m = octave::readVectors( sfile, varname, &nread );
	if ( m == NULL )
		return false;
	if ( nread <=0 )
		return false;

	if ( nread > N )
		nread = N;
	
	for (int i=0; i < nread; i++) {
		genMeas[i](0) = m[i](0);
		genMeas[i](1) = m[i](1);
		genMeas[i](2) = m[i](2);
	}

	sfile.close();
	return true;
}

void	genInputData()
{
	FT	theta, phi;

	FT dist1,dist2,dist3;
	#if ADD_DISTORTION
		dist1 = 1.5; dist2 = 1.15; dist3 = 0.95;
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
#if 1
	cout << "--- Generating data..."  << endl;
	genInputData();	//generate random data
#else
	cout << "--- Loading data..." << endl;
	loadInputData("octave/mag","rawmag");
#endif

	cout << "---- Finished data, processing..." << endl;

	Matrix<FT,3,1>	offset;
	offset << 2.5, 2.5, 2.5;

	openAHRS::calib::UKFEllipsoid	EL;

TIME_THIS( "calib UKF",
	for (int i=0; i < N; i++)
	{
		if ( i == 0 )
			EL.init( 1e-6, offset, SPHERE_RADIUS, 1e-6, 1e-9, 1e-3 );

		EL.estimateParams( genMeas[i] );

		EL.getStateVector( calibState[i] );
	}
);

	cout << "---- Finished processing" << endl;
	cout << "State vector:\n" << calibState[N-1] << endl << endl;


	cout << "Write data to disk?? (y,n): ";
	if ( getchar() == 'y' ) {
		cout << "---- Writing to disk..." << endl;
		/** save octave file **/
		ofstream	file("octave/calibtest");
	
		octave::writeVectors( file, "meas", genMeas, N );
		octave::writeVectors( file, "state", calibState, N );
	
		trueData(0) = SPHERE_RADIUS;
		trueData(1) = SPHERE_X;
		trueData(2) = SPHERE_Y;
		trueData(3) = SPHERE_Z;
		octave::writeVectors( file, "trueData", &trueData, 1 );

		file.close();
	}

	return 0;
}

