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

namespace openAHRS { namespace kalman7
{
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
							bool track_bias, FT dt );


	/**
	 * Public access to state vector
	 */
	extern Matrix<FT,7,1>	X;	/* state vector */

}};

#endif	/* __kalman7_h_ */

