/*
 *  Math / quaternion utilities
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



#ifndef	__util_h_
#define	__util_h_

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

/** OUR OWN PI **/
#define C_PI    3.14159265358979323846264338327950288419716939937511

namespace openIMU { namespace util
{
	/**
	 * Calculate Quaternion Omega Matrix
	 * p,q,r are rotation speeds
	 * 
	 * @param p		Roll
	 * @param q		Pitch
	 * @param r		Yaw
	 *
	 * @return		Quaternion omega matrix
	 */
	Matrix<FT,4,4>		calcQOmega( double p, double q, double r );

	/** 
	 * Calculates measurement matrix (H)
	 * for kalman filter, such that
	 * [ droll\de0	droll\dex	droll\dey	droll\dez	]
	 * [ dpitch\de0	...			...			...			]
	 * [ dyaw\de0	...			...			...			]
	 *
	 * @param q		[e0 ex ey ez]'
	 *
	 * @return		Measurement matrix regarding quaternion state
	 */
	Matrix<FT,3,4>	calcQMeas( const Matrix<FT,4,1>	&q );


	/**
	 * Convert quaternion to euler angles.
	 *
	 * @param q		Input quaternion
	 *
	 * @return		[roll, pitch, heading]'
	 */
	Matrix<FT,3,1>		quatToEuler( const Matrix<FT,4,1> &q );

	/**
	 * Euler to quaternion
	 *
	 * @param e		Euler angles [ roll, pitch, yaw ]'
	 *
	 * @return		Quaternion [ e0 ex ey ez ]'
	 */
	Matrix<FT,4,1>		eulerToQuat( const Matrix<FT,3,1> &e );

	/**
	 * Accelerometer data to pitch and roll.
	 * Yaw is not calculated.
	 *
	 * @param accels	Accelerometer data, input
	 * @param angles	Output angles, only roll and pitch are calculated
	 */
	void	accelToPR( const Matrix<FT,3,1> &accels, Matrix<FT,3,1> &angles );

	/**
	 * Calculate current heading based on magnetic measurements
	 * and pitch and roll.
	 *
	 * @param magn		Magnetometer data in body coordinates
	 * @param attitude	Attitude [roll;pitch;yaw], yaw is not used
	 */
	float	calcHeading( const Matrix<FT,3,1> &magn, const Matrix<FT,3,1> &attitude );

	/**
	 * Normalize quaternion
	 *
	 * NOT USED
	 */
	void	quatNormalize( Matrix<FT,4,1> &q );

	/** Return random number,
	 * normal distribution, 0..1
	 */
	double	randomNormal(void);

	/**
	 * Creates a random vector with requested mean and variance
	 *
	 * @param mean		Desired mean
	 * @param stddev	Desired standard deviation
	 *
	 * @return			Vector3 (row)
	 */
	Matrix<FT,3,1>	randomVector3( FT mean, FT stddev );

}};

#endif /* __util_h_ */

