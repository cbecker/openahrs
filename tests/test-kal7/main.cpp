#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

#include <openIMU/util/util.h>
#include <openIMU/kalman/kalman7.h>
#include <openIMU/util/octave.h>

#include "timer_this.h"


using namespace std;
using namespace openIMU;
using namespace openIMU::kalman7;

FT	dt	= 1.0/50;

#define	N	20000

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
		//p	= 0.03*sin(2*0.02*C_PI*i*dt);
		p	 = 0.1*sin(2*0.02*C_PI*i*dt);
		//p = 0.001;
		q	= 0.05*cos(2*0.2*C_PI*i*dt+0.3);
		//q = 0;
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
		input.angles[i]	<< atan2( -input.accels[i][1],
									input.accels[i][2] ),
							0,	
							yaw;

		if ( add_noise )
			input.angles[i](2)	+= util::randomNormal()*sqrt(meas_variance);

		FT	temp = -input.accels[i][0]/input.accels[i].norm();
		if ( temp > 1 )		temp = 1;
		if ( temp < -1 )	temp = -1;
		input.angles[i][1]	= asin(temp);

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

	makeTempData(false);

/*	ofstream	file("octest");

	octave::writeVectors( file, "varname", arr,2);
	octave::writeVectors( file, "var2", arr, 3 );
	file.close();*/

	angle		=	input.realAngles[0];

	/** let the filter 'guess' the biases **/
	startBias	<< 0,0,0;
	//startBias	=	gyroBias;

	KalmanInit( angle, startBias, meas_variance );
	cout << angle << endl;

	cout << "Q\n" << util::eulerToQuat( angle ) << endl;
	int i;

//TIME_THIS("aa",

	for (i=0; i < N; i++ )
	{
		KalmanUpdate( i, input.angles[i], dt );
		
		output.state[i]	= X;
		/*output.angles[i]	= util::quatToEuler( output.state[i].start<4>() );

		const FT	tolerance	= 0.2;
		if ( fabs( output.angles[i](1) - input.angles[i](1) ) > tolerance )
			cout << "Err en " << i << endl;*/

		KalmanPredict( i, input.gyros[i], dt );
	//	cout << "Euler\n" << util::quatToEuler(X.start<4>()) << endl;
	//	cout << "Bias\n"  << X.block<3,1>(4,0) << endl;
	}
//);
	for (i=0 ; i < N ; i++ )
	{
		output.angles[i]	= util::quatToEuler( output.state[i].start<4>() );
	}

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
