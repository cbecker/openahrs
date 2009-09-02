#ifndef _calib_ellipsoid_h_
#define _calib_ellipsoid_h_
/*
 *  Ellipsoid calibration class and routines. Uses EKF.
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
 * Ellipsoid calibration routines,
 * based on Sean D'Epagnier's calibration for openmag: http://repo.or.cz/w/openmag.git
 *
 * There is no predict iteration since it's not a time-variant system.
 * However the process noise matrix is used to help convergence since 
 * there are 9 parameters to estimate.
 *
 */

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN
#include <openAHRS/util/timing.h>

namespace openAHRS { namespace calib 
{

	class	Ellipsoid
	{
	public:
		Ellipsoid() {}	//default constructor

		/**
		* Init default matrices.
		*
		* @param meas_noise		Measurement noise variance
		* @param estBias		Estimated bias for X,Y,Z. Should be quite precise
		*						or the filter may not converge
		* @param estAmpl		Estimated axis amplitude, same comments as for estBias
		* @param variance1		Variance for radius and center estimates' noise process matrix
		* @param variance2		Variance for sphere distortion estimates' noise process matrix
		*
		*/
		void		init( FT meas_noise, const Matrix<FT,3,1> &estBias, FT estAmpl,
					 FT variance1, FT variance2	)
		{
			R = meas_noise;
			P.setIdentity();
			I.setIdentity();

			/* if distortions from a perfect sphere are small this helps for
			 * filter convergence */
			X(1) = X(2) = 1.0;
			X(3) = X(4) = X(5) = 0.0;

			// 1/radius estimate
			X(0) = 1/estAmpl;
			X(stateOffsetX) = estBias(0);
			X(stateOffsetY) = estBias(1);
			X(stateOffsetZ) = estBias(2);

			/** This tells the filter to believe the initial estimates for
			 * the states that need only small corrections */
			Q.setIdentity();
			Q *= variance1;			//these factors depend on how 'distorted'
								// we expect the sphere to be
			Q(1,1) = Q(2,2) = Q(3,3) = Q(4,4) = Q(5,5) = variance2;
			//P(1,1) = P(2,2) = P(3,3) = P(4,4) = P(5,5) = 0;
		}

		/** 
		* Perform kalman update/predict
		* 
		* @param meas			Measurement to process for parameter estimation
		*/
		void		estimateParams( const Matrix<FT,3,1> &meas )
		{
			Matrix<FT,9,9>	T1;
			Matrix<FT,1,9>	H;
			Matrix<FT,9,1>	Ht;

			/* Q 'trick'. System evolution matrix is identity */
			P = P + Q;

			FT	xCent	= meas(0) - X(stateOffsetX);
			FT	yCent	= meas(1) - X(stateOffsetY);
			FT	zCent	= meas(2) - X(stateOffsetZ);

#if 0
		FT X022 = 2*X(0)*X(0);
		FT sum1 = X(2)*zCent + X(5)*yCent + X(4)*xCent;
		FT sum2 = X(1)*yCent + X(3)*xCent;

		FT xx = X022*xCent;
		FT yy = X022*yCent;
		FT zz = X022*zCent;

		H <<  2*X(0)*( sum1*sum1 + sum2*sum2 + xCent*xCent ),
			yy*sum2,
			zz*sum1,
			xx*sum2,
			xx*sum1,
			yy*sum1,
			X022*( X(4)*sum1 - X(3)*sum2 - xCent),
			X022*( X(5)*sum1 - X(1)*sum2),
			X022*X(2)*sum1;
#else
			H <<  2*(X(2)*(zCent)+X(5)*(yCent)+X(4)*(xCent))*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent))+2*(X(1)*(yCent)+X(3)*(xCent))*(X(0)*X(1)*(yCent)+X(0)*X(3)*(xCent))+2*X(0)*(xCent*xCent),
			2*X(0)*(X(0)*X(1)*(yCent)+X(0)*X(3)*(xCent))*(yCent),
			2*X(0)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent))*(zCent),
			2*X(0)*(xCent)*(X(0)*X(1)*(yCent)+X(0)*X(3)*(xCent)),
			2*X(0)*(xCent)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent)),
			2*X(0)*(yCent)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent)),
			-2*X(0)*X(4)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent))-2*X(0)*X(3)*(X(0)*X(1)*(yCent)+X(0)*X(3)*(xCent))-2*X(0)*X(0)*(xCent),
			-2*X(0)*X(5)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent))-2*X(0)*X(1)*(X(0)*X(1)*(yCent)+X(0)*X(3)*(xCent)),
			-2*X(0)*X(2)*(X(0)*X(2)*(zCent)+X(0)*X(5)*(yCent)+X(0)*X(4)*(xCent));
#endif

			Ht = H.transpose();

			FT	inv	= 1/( (H*P*Ht)(0) + R );
			Matrix<FT,9,1>	K = P*Ht*inv;
			Matrix<FT,1,9>	Kt = K.transpose();
			
			FT Z = X(0)*X(0);
			{
				FT	mx = xCent;
				FT	my = X(1)*yCent + X(3)*xCent;
				FT	mz = X(2)*zCent + X(4)*xCent + X(5)*yCent;
				Z *= mx*mx + my*my + mz*mz;
			}
			Z = 1 - Z;

			/* update X */
			X = X + K*Z;

			/** let's check the vector **/
		#if 0
			#define	truncHigh(a)	if ( (a) < (1-maxSphereDistortion) )	\
										a = 1-maxSphereDistortion;			\
									if ( (a) > (1+maxSphereDistortion) )	\
										a = 1+maxSphereDistortion;

			#define	truncLow(a)		if ( (a) < -maxSphereDistortion )		\
										a = -maxSphereDistortion;			\
									if ( (a) > maxSphereDistortion )			\
										a = maxSphereDistortion;

			truncHigh( X(1) );
			truncHigh( X(2) );
			truncLow( X(3) );
			truncLow( X(4) );
			truncLow( X(5) );
		#endif

			T1 = I-K*H;
			P = T1*P*T1.transpose();

			P += K*R*K.transpose();
			
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

			out(0)	=	X(0)*xCent;
			out(1)	=	X(0)*( X(1)*yCent + X(3)*xCent );
			out(2)	=	X(0)*( X(2)*zCent + X(4)*xCent + X(5)*yCent );
		}

		/** Just for debugging **/
		void	getStateVector( Matrix<FT,9,1> &x ) { x = X; };

	private:
		Matrix<FT,9,9>	Q;	/* process noise cov matrix */
		FT				R;	/* measurement noise variance */

		Matrix<FT,9,9>	P;	/* state covariance matrix */
		Matrix<FT,9,1>	X;	/* state Vector */

		Matrix<FT,9,9>	I;	/* identity matrix */

		/* what each element in X means
		 * only some of them listed */
		static const int	stateScaleFactor	= 0;
		static const int	stateOffsetX		= 6;
		static const int	stateOffsetY		= 7;
		static const int	stateOffsetZ		= 8;

		//static const int	maxSphereDistortion	= 2;	// If some coeffs are bigger than this they'll be truncated

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};

}}; /* namespace calib, namespace openAHRS */


#endif /* _calib_ellipsoid_h_ */

