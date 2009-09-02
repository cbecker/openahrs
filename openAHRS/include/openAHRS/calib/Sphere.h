#ifndef _calib_sphere_h_
#define _calib_sphere_h_
/*
 *  Spherical calibration class and routines. Uses EKF
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



/**
 * Spherical calibration routines,
 * based on Sean D'Epagnier's calibration for openmag: http://repo.or.cz/w/openmag.git
 *
 * There is no predict iteration since it's not a time-variant system.
 */

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN


namespace openAHRS { namespace calib 
{

	class	Sphere
	{
	public:
		Sphere() {}	//default constructor

		/**
		* Init default matrices.
		*
		* @param meas_noise		Measurement noise variance
		* @param estAmplitude	estimated amplitude
		* @param initialMeas	Initial measurement, so that
		*						some states can be approximated before startup
		*/
		void		init( FT meas_noise, FT estAmplitude, const Matrix<FT,3,1> &estBias )
		{
			R = meas_noise;
			P.setIdentity();
			Q.setIdentity();
			Q *= 0;		/* not used now */
			I.setIdentity();
			X.setIdentity();
	
			X(0) = estAmplitude;
			X(1) = estBias(0);
			X(2) = estBias(1);
			X(3) = estBias(2);
		}
			Matrix<FT,4,1>	Ht;
			Matrix<FT,4,1>	K ;
			Matrix<FT,1,4>	H;
			Matrix<FT,4,4>	IKH;


		/** 
		* Perform kalman update/predict
		* 
		* @param meas			Measurement to process for parameter estimation
		*/
		void		estimateParams( const Matrix<FT,3,1> &meas )
		{
			FT	xCent	= meas(0) - X(stateOffsetX);
			FT	yCent	= meas(1) - X(stateOffsetY);
			FT	zCent	= meas(2) - X(stateOffsetZ);

			P = P + Q;

			H << -2*X(0),
				-2*xCent,
				-2*yCent,
				-2*zCent;
			

			Ht = H.transpose();

			FT	inv	= 1/( (H*P*Ht)(0) + R );
			K = P*Ht*inv;

			FT Z = X(0)*X(0) - xCent*xCent - yCent*yCent - zCent*zCent;

			/* update X */
			X = X + K*Z;
			
			IKH = I- K*H;

			P = IKH*P*IKH.transpose() + K*R*K.transpose();
			
			return;
		}

		/**
		* Process input data using the previous estimated states.
		*
		* @param meas	Measurement to process
		* @param out	Where to write the calibrated output
		*/
		void	processInput( const Matrix<FT,3,1> &meas, Matrix<FT,3,1> &out )
		{
			FT	xCent	= meas(0) - X(stateOffsetX);
			FT	yCent	= meas(1) - X(stateOffsetY);
			FT	zCent	= meas(2) - X(stateOffsetZ);

			out(0)	=	xCent/X(0);
			out(1)	=	(yCent )/X(0);
			out(2)	=	( zCent )/X(0);
		}

		/** Just for debugging **/
		void	getStateVector(Matrix<FT,4,1> &y) { y = X; };

	private:
		Matrix<FT,4,4>	Q;
		Matrix<FT,4,4>	P;	/* state covariance matrix */
		Matrix<FT,4,1>	X;	/* state Vector */

		Matrix<FT,4,4>	I;	/* identity matrix */
		
		FT				R;	/* measurement noise variance */

		/* what each element in X means
		 * only some of them listed */
		static const int	stateScaleFactor	= 0;
		static const int	stateOffsetX		= 1;
		static const int	stateOffsetY		= 2;
		static const int	stateOffsetZ		= 3;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};

}}; /* namespace calib, namespace openAHRS */


#endif /* _calib_ellipsoid_h_ */

