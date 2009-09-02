#ifndef _ukf_ellipsoid_h_
#define _ukf_ellipsoid_h_

/*
 *  Ellipsoid calibration class and routines using UKF (Unscented Kalman Filter)
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
#include <Eigen/Cholesky>
USING_PART_OF_NAMESPACE_EIGEN


namespace openAHRS { namespace calib 
{

	class	UKFEllipsoid
	{
	public:
		static const int	L			= 9;	/* order */

		void	getCovarianceMatrix( Matrix<FT,L,L>	&cm ) {
			cm	= P;
		}
		
		void	setCovarianceMatrix( const Matrix<FT,L,L>	&cm ) {
			P = cm;
		}

	public:
		UKFEllipsoid() {
			UKF_lambda	= UKF_alpha*UKF_alpha*(L + UKF_kappa) - L;
			UKF_Ws0		= UKF_lambda/(L+UKF_lambda);
			UKF_Wc0		= UKF_Ws0 + ( 1- UKF_alpha*UKF_alpha + UKF_beta );
			UKF_Wsi		= 1.0/(2.0*(L+UKF_lambda));
			UKF_Wci		= UKF_Wsi;
			sqWeigth	= sqrt( L + UKF_lambda );
		}	//default constructor

		/**
		* Init default matrices.
		*
		* @param meas_noise		Measurement noise variance
		* @param estBias		Estimated bias for X,Y,Z. Should be quite precise
		*						or the filter may not converge
		* @param estAmpl		Estimated axis amplitude, same comments as for estBias
		*/
		void		init( FT meas_noise, const Matrix<FT,3,1> &estBias, FT estAmpl,
						FT variance1, FT variance2, FT PstartVariance )
		{
			R = meas_noise;
			P.setIdentity();
			P *= PstartVariance;
			I.setIdentity();

			/* if distortions from a perfect sphere are small this helps for
			 * filter convergence */
			X(1) = X(2) = 1.0;
			X(3) = X(4) = X(5) = 0.0;

			// 1/radius estimate0
			X(0) = estAmpl;
			X(stateOffsetX) = estBias(0);
			X(stateOffsetY) = estBias(1);
			X(stateOffsetZ) = estBias(2);

			/** This tells the filter to believe the initial estimates for
			 * the states that need only small corrections */
			Q.setIdentity();
			Q *= variance1;			//these factors depend on how 'distorted'
								// we expect the sphere to be
			Q(1,1) = Q(2,2) = Q(3,3) = Q(4,4) = Q(5,5) = variance2;

			//P(0,0)=0;
			//P(2,2) = P(3,3) = P(4,4) = P(5,5) = P(6,6) = P(7,7) = P(8,8) = 1e-6;
//			P(1,1) = P(2,2) = P(3,3) = P(4,4) = P(5,5) = 1;

			/*std::cout << " P: " << P << "\n";
			getchar();*/
		}

		/** 
		* Perform kalman update/predict
		* 
		* @param meas			Measurement to process for parameter estimation
		*/
		void		estimateParams( const Matrix<FT,3,1> &meas )
		{
			
			/** Calculate sigma points **/
			SP.block<L,1>(0,0)	= X;

			#if 1
				Matrix<FT,L,L>	temp;
				temp	= 16384*P;
				sqMatrix	 = temp.llt().matrixL()*sqWeigth/128;
			#else
				sqMatrix	= P;
				sqMatrix	*= L + UKF_lambda;
				sqMatrix	= sqMatrix.llt().matrixL();
			#endif

			if ( !P.llt().isPositiveDefinite() ) {
				printf("ERR POSITIVE DEF\n");
				exit(-1);
			}

			for (int i=1; i < L+1; i++)
				SP.block<L,1>(0,i) = X + sqMatrix.block<L,1>(0,i-1);
			for (int i=L+1; i < 2*L + 1; i++)
				SP.block<L,1>(0,i) = X - sqMatrix.block<L,1>(0,i-L-1);

			/* Q 'trick'. System evolution matrix is identity */
			P = P + Q;

			/* Project sigma points through h */
			FT	gamma[2*L + 1];
			for (int i=0; i < 2*L+1; i++) {
				FT	xCent = meas(0) - SP(stateOffsetX, i);
				FT	yCent = meas(1) - SP(stateOffsetY, i);
				FT	zCent = meas(2) - SP(stateOffsetZ, i);

				FT	mx = xCent;
				FT	my = SP(1,i)*yCent + SP(3,i)*xCent;
				FT	mz = SP(2,i)*zCent + SP(4,i)*xCent + SP(5,i)*yCent;

				gamma[i] = -1 + (mx*mx + my*my + mz*mz)/(SP(0,i)*SP(0,i));
				
			}

			/* Weight sigma points */
			FT	Z = UKF_Ws0*gamma[0];
			for (int i=1; i < 2*L+1; i++)
				Z += UKF_Wsi*gamma[i];

			/* Weight sigma points for P */
			FT	Pzz	= UKF_Wc0*( gamma[0] - Z)*(gamma[0] - Z) + R;
			Matrix<FT,L,1>	Pxz;
			Pxz	= UKF_Wc0*( SP.block<L,1>(0,0) - X ) * (gamma[0] - Z);
			for (int i=1; i < 2*L+1; i++) {
				Pzz	+= UKF_Wci*( gamma[i] - Z ) * (gamma[i]-Z);
				Pxz	+= UKF_Wci*( SP.block<L,1>(0,i) - X ) * (gamma[i]-Z);
			}

			Matrix<FT,L,1> K = Pxz * (1/Pzz);
			X += K*(-Z);

		//	printf("Error %f\n", (float)Z);
/*			if ( Z > 0.5 )
				printf("Error muy alto: %f\n" , (float)Z);*/

			P = P - K*Pzz*K.transpose();
			
			return;

			/* update X */
			//X = X + K*Z;

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


			//P = (I - K*H)*P*(I - K*H).transpose() + K*R*K.transpose();
			
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
			out(1)	=	( X(1)*yCent + X(3)*xCent )/X(0);
			out(2)	=	( X(2)*zCent + X(4)*xCent + X(5)*yCent )/X(0);
		}

		/** Just for debugging **/
		void	getStateVector( Matrix<FT,L,1> &x ) { x = X; };
		void	setStateVector( const Matrix<FT,L,1>	&x ) { X = x; }

	private:

		Matrix<FT,L,L>	Q;	/* process noise cov matrix */
		FT				R;	/* measurement noise variance */

		Matrix<FT,L,L>	P;	/* state covariance matrix */
		Matrix<FT,L,1>	X;	/* state Vector */

		Matrix<FT,L,L>	I;	/* identity matrix */

		/* what each element in X means
		 * only some of them listed */
		static const int	stateScaleFactor	= 0;
		static const int	stateOffsetX		= 6;
		static const int	stateOffsetY		= 7;
		static const int	stateOffsetZ		= 8;

		/** UKF constants **/
		static const FT	UKF_alpha	= 1e-3;
		static const FT	UKF_beta	= 2.0;
		static const FT	UKF_kappa	= 3 - L;
		
		FT	UKF_lambda;
		FT	UKF_Ws0;
		FT	UKF_Wc0;
		FT	UKF_Wsi;
		FT	UKF_Wci;
		FT	sqWeigth;

		Matrix<FT,L,L>			sqMatrix;	/* preallocate square root matrix */
		Matrix<FT,L,2*L + 1>	SP;	/* preallocate sigma points */

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

}}; /* namespace calib, namespace openAHRS */


#endif /* _calib_ellipsoid_h_ */



