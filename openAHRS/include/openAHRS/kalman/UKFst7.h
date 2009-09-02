/*
 *  7-state Kalman Filter for gyro and accelerometer processing
 *	Position and gyro bias tracking. Unscented Kalman Filter version.
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

#ifndef __ukf_st7_h_
#define	__ukf_st7_h_

#include <openAHRS/kalman/UKF.h>
#include <openAHRS/util/util.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace openAHRS { 

struct UKFst7_Funcs
{
	/* Predict next state based on current state and gyro measurements
	*
	* @param i			number of state to predict
	* @param gyros		gyro data
	* @param dt			delta between updates
	*/
	static inline Matrix<FT,7,1>  predictState( const Matrix<FT,7,1> &state, const Matrix<FT,3,1> &gyros, FT dt )
	{
		// TODO: optimize
		Matrix<FT,4,1>	quat = state.block<4,1>(0,0);
		Matrix<FT,7,1>	ret;
		
		
	//	quat.normalize();	//=> this shouldn't be neccessary
		
		FT	p = gyros(0) - state(4);
		FT	q = gyros(1) - state(5);
		FT	r = gyros(2) - state(6);

		/* New quaternion estimate */
		ret.block<4,1>(0,0) = quat + util::calcQOmega( p, q, r )*quat*(dt/2);
		ret(4) = state(4);
		ret(5) = state(5);
		ret(6) = state(6);

		//renormalize quaternion
		ret.block<4,1>(0,0) /= ret.block<4,1>(0,0).norm();

		return ret;
	}

	static inline Matrix<FT,3,1>	measure( const Matrix<FT, 7, 1> &state, FT dt )
	{
		Matrix<FT,4,1>	temp = state.block<4,1>(0,0);
		//return	util::quatToEuler(temp);
		return	util::quatToEulerNorm( temp );
	}
};

class UKFst7
{
public:
	enum {
		L = 7,	/* order, states */
		M = 3,	/* inputs */
		N = 3,	/* intermediate for prediction */
	};
private:
	UKF< UKFst7_Funcs, L, M, N, false >	estimator;

public:
		void	getStateVector( Matrix<FT,L,1> &v ) {
			estimator.getStateVector(v);
		}

public:
	UKFst7() 
	{	/* simple, nearly do-nothing constructor */
	}


	/**
	 * Init kalman variables and state.
	 *
	 * @param startAngle	Initial angle estimate		[roll,pitch,yaw]'
	 * @param startBias		Initial gyro bias estimate	[biasP,biasQ,biasR]'
	 * @param meas_variance	Measurement variance.
	 *							TODO: should be more descriptive, and there
	 *							should be two variances, one for gyros and one for accels.
	 *
	 * @param process_bias_var	variance for the process bias estimate
	 * @param process_quat_var	variance for the quaternion estimate
	 */
	void	KalmanInit( Matrix<FT,3,1> &startAngle, 
						Matrix<FT,3,1> &startBias, FT meas_var,
						FT process_bias_var, FT process_quat_var )
	{
		/* temp matrices */
		Matrix<FT,L,L>	Q;
		Matrix<FT,M,M>	R;
		Matrix<FT,L,1>	X;

		estimator.KalmanInit();

		R.setIdentity();
		R	*= meas_var;

		/** noise model covariance matrix  **/
		Q.setIdentity();
		
		Q		*= process_quat_var;
		Q(4,4)	 = process_bias_var;
		Q(5,5)	 = process_bias_var;
		Q(6,6)	 = process_bias_var;


		/** initial estimate and bias **/
		X.block<4,1>(0,0)	= util::eulerToQuat( startAngle );
		X.block<3,1>(4,0)	= startBias;

		estimator.setStateVector( X );
		estimator.setProcessCovariance( Q );
		estimator.setMeasurementCovariance( R );
	}

	/**
	 * Kalman - Update state 
	 *
	 * @param iter		Current iteration number, only for testing purposes.
	 * @param angles	Current measured angles (calculated from accel data).
	 * @param dt		Time between consecutive kalman updates.
	 *
	 */
	void	KalmanUpdate( int iter, const Matrix<FT,3,1> &angles, FT dt )
	{
		estimator.KalmanUpdate( iter, angles, dt );
		if ( (iter % 100) == 0 )
			estimator.printMatrices();
	}

	/** 
	 * Kalman - Predict state 
	 *
	 * @param iter			Current iteration number, only for testing purposes.
	 * @param gyros			Current measured gyro rate, including biases
	 * @param dt			Time between consecutive kalman updates
	 *
	 */
	void	KalmanPredict( int iter, const Matrix<FT,3,1> &gyros, FT dt )
	{
		estimator.KalmanPredict( iter, gyros, dt );
	}


	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

};

#endif	/* __ukf_st7_h_ */

