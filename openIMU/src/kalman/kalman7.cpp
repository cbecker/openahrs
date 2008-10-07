/*
 *  7-state Kalman Filter for gyro and accelerometer processing
 *	Position and gyro bias tracking.
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



#include <openIMU/kalman/kalman7.h>


#include <Eigen/Core>
#include <Eigen/LU>

USING_PART_OF_NAMESPACE_EIGEN


using namespace std;	//for debugging

namespace openIMU { namespace kalman7
{

	Matrix<FT,7,7>	A;	/* transition matrix */
	Matrix<FT,7,7>	P;	

	Matrix<FT,3,7>	H;	/* observation matrix */
	Matrix<FT,7,3>	K;	/* kalman gain */
	Matrix<FT,7,7>	W;	/* process noise matrix */
	Matrix<FT,3,3>	R;	/* measurement noise matrix */

	Matrix<FT,7,1>	X;	/* state vector */
			/**
			 * X(0..3)	=> quaternion
			 * X(4..6)	=> gyro bias estimate
			 */

	Matrix<FT,4,1>	q;	/* temp quaternion */

	Matrix<FT,3,1>	angleErr;	/* for angle error calculation */
	
	Matrix<FT,7,7>	I;	/* identity */

	/** measurement variance */
	FT	meas_variance	= 0.01;


	FT		calcError( FT plus, FT minus )
	{
		if ( (minus > C_PI/2) && (plus < -C_PI/2) ) {
			return	2*C_PI + plus - minus;
		} else if ( (minus < -C_PI/2) && (plus > C_PI/2) ) {
			return	-2*C_PI + plus - minus;
		}
		else
			return plus - minus;
	}


	void	KalmanInit( Matrix<FT,3,1> &startAngle, 
						Matrix<FT,3,1> &startBias, FT meas_var,
						FT process_bias_var, FT process_quat_var )
	{
		meas_variance	= meas_var;

		I.setIdentity();
		P.setIdentity();

		A.setIdentity();

		R.setIdentity();
		R	*= meas_variance;

		/** noise model covariance matrix  **/
		W.setIdentity();
		
		W		*= process_quat_var;
		W(4,4)	 = process_bias_var;
		W(5,5)	 = process_bias_var;
		W(6,6)	 = process_bias_var;


		H.setZero();

		/** initial estimate and bias **/
		X.block<4,1>(0,0)	= util::eulerToQuat( startAngle );
		X.block<3,1>(4,0)	= startBias;
	}


	/** 
	* Calculate discrete transition matrix
	*
	* @param A			Destination matrix
	* @param gyros		Gyro data, including bias
	* @param q			Current state in quaternion format
	* @param track_bias	if bias should be tracked or held constant
	* @param dt			Delta between updates
	*/
	void	calcA( Matrix<FT,7,7> &A, 
					const Matrix<FT,3,1> &gyros,
					const Matrix<FT,4,1> &q, bool track_bias, FT dt )
	{
		A.setIdentity();
		if ( track_bias )
		{
			/**
			 * Here biases are substracted 'twice',
			 * Anyway that won't be a problem since only <4,4> will be used to update the state
			 * vector (see below in kalman predict) */
			A.block<4,4>(0,0)	= Matrix<FT,4,4>::Identity() +
					dt * util::calcQOmega( gyros[0] - X(4), gyros[1] - X(5), gyros[2] - X(6) )/2;

			A.block<1,3>(0,4)	<<	 dt*q[1]/2,  dt*q[2]/2,  dt*q[3]/2;
			A.block<1,3>(1,4)	<<	-dt*q[0]/2,  dt*q[3]/2, -dt*q[2]/2;
			A.block<1,3>(2,4)	<<	-dt*q[3]/2, -dt*q[0]/2,  dt*q[1]/2;
			A.block<1,3>(3,4)	<<	 dt*q[2]/2, -dt*q[1]/2, -dt*q[0]/2;
		}
		else {
			A.block<4,4>(0,0)	= Matrix<FT,4,4>::Identity() +
				dt * util::calcQOmega( gyros(0) - X(4) , gyros(1) - X(5) , gyros(2) - X(6)) /2;
		}
	}

	void	KalmanUpdate( int iter, const Matrix<FT,3,1> &angles, FT dt )
	{
		/*if ( iter == 0 )
		{
			cout << "A\n" << A << endl;
			cout << "X\n" << X << endl;
			cout << "R\n" << R << endl;
			cout << "W\n" << W << endl;
			cout << "K\n" << K << endl;
			cout << "P\n" << P << endl;

		}*/

		/** Renormalize quaternion **/
		q	= X.start<4>();
		q.normalize();
		X.start<4>()	= q;

		/*** KALMAN UPDATE **/

		/*- R should be weighted depending on angle,
		 * since real inputs are accels */

		H.block<3,4>(0,0)	= util::calcQMeas( q );


		Matrix<FT,7,3>	Ht	= H.transpose();
		Matrix<FT,3,3> inv;
		( H*P*Ht + R ).computeInverse( &inv );

		if ( isnan( inv(0,0) ) )
			cout << "NAN" << endl;
		
		K	= P*Ht * inv;	
		/*if ( iter == 0 ) {
			cout << "P\n" << P << endl;
			cout << "R\n" << R << endl;
			cout << "K\n" << K << endl;
			cout << "H\n" << H <<endl;
			cout << "Ht\n" << Ht <<endl;
		}*/

		/** predicted quaternion to euler for error calculation **/
		Matrix<FT,3,1>	predAngles	= util::quatToEuler( q );
		angleErr(0)	= calcError( angles(0), predAngles(0) );
		angleErr(1)	= calcError( angles(1), predAngles(1) );
		angleErr(2)	= calcError( angles(2), predAngles(2) );

		//X	= X + K*( angles - predAngles  );
		X	= X + K*angleErr;

/*		if ( iter == 0 )
			cout << "X\n" << X << endl;*/


	
		/** Renormalize Quaternion **/
		q	= X.start<4>();
		q.normalize();
		X.start<4>()	= q;

		#if 0
			P	= ( I - K*H ) * P;	/* Using this might not be right if 
									calculation problems arise */
		#else
			P	= ( I - K*H ) * P * (( I - K*H ).transpose()) + K*R*(K.transpose());
		#endif
		
	}


	void	KalmanPredict( int iter, const Matrix<FT,3,1> &gyros, bool track_bias, FT dt )
	{
		/** Predict **/
		calcA( A, gyros, q, track_bias, dt );

		/** only update quaternion-relevant data in our state vector **/
		//X = A*X;
		
		X.start<4>()	= A.block<4,4>(0,0) * X.start<4>();
		
		P	= A*P*(A.transpose()) + W;



/*		if ( iter == 0 )
		{
			cout << "A\n" << A << endl;
			cout << "X\n" << X << endl;
			cout << "P\n" << P << endl;
		}*/

	}


}};

