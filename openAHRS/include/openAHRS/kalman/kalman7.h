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

#ifndef __kalman7_h_
#define	__kalman7_h_

#include <openAHRS/util/util.h>

namespace openAHRS { 

class kalman7
{

public:
	kalman7();	/* simple, nearly do-nothing constructor */

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
						FT process_bias_var, FT process_quat_var );

	/**
	 * Kalman - Update state 
	 *
	 * @param iter		Current iteration number, only for testing purposes.
	 * @param angles	Current measured angles (calculated from accel data).
	 * @param dt		Time between consecutive kalman updates.
	 *
	 */
	void	KalmanUpdate( int iter, const Matrix<FT,3,1> &angles, FT dt );

	/** 
	 * Kalman - Predict state 
	 *
	 * @param iter			Current iteration number, only for testing purposes.
	 * @param gyros			Current measured gyro rate, including biases
	 * @param track_bias	if bias should be tracked
	 * @param dt			Time between consecutive kalman updates
	 *
	 */
	void	KalmanPredict( int iter, const Matrix<FT,3,1> &gyros, 
							FT dt );

public:
	inline void	getStateVector( Matrix<FT,7,1>	&x ) { x = X; }
	/**
	 * Public access to state vector
	 */
private:
	Matrix<FT,7,1>	X;	/* state vector */
			/**
			 * X(0..3)	=> quaternion
			 * X(4..6)	=> gyro bias estimate
			 */

private:
	Matrix<FT,7,7>	A;	/* transition matrix */
	Matrix<FT,7,7>	P;	

	Matrix<FT,3,7>	H;	/* observation matrix */
	Matrix<FT,7,3>	K;	/* kalman gain */
	Matrix<FT,7,7>	W;	/* process noise matrix */
	Matrix<FT,3,3>	R;	/* measurement noise matrix */

			
	Matrix<FT,4,1>	q;	/* temp quaternion */

	Matrix<FT,3,1>	angleErr;	/* for angle error calculation */
	
	Matrix<FT,7,7>	I;	/* identity */

	/** measurement variance */
	FT	meas_variance;


private:
	/** 
	* Calculate transition jacobian
	*
	* @param A			Destination matrix
	* @param gyros		Gyro data, including bias
	* @param q			Current state in quaternion format
	* @param track_bias	if bias should be tracked or held constant
	* @param dt			Delta between updates
	*/
	void	calcA( Matrix<FT,7,7> &A, 
					const Matrix<FT,3,1> &gyros,
					const Matrix<FT,4,1> &q, FT dt );

	/**
	* Predict next state based on current state and gyro measurements
	*
	* @param X			Source/destination state vector
	* @param gyros		gyro data
	* @param dt			delta between updates
	*/
	void	predictState( Matrix<FT,7,1> &X, 
					const Matrix<FT,3,1> &gyros, FT dt );

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

};

#endif	/* __kalman7_h_ */

