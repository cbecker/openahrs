#ifndef _oukf_h_
#define	_oukf_h_

/*
 *  Unscented Kalman Filter template, using Cholesky decomposition.
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

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

USING_PART_OF_NAMESPACE_EIGEN

/**
 * T is a class/struct that must provide the customization functions for
 *	observation and prediction:
 *
 *	Matrix<FT,numInputs,1> measure( const Matrix<FT,numStates,1> &state, FT dt )
 *		state is the state vector, dt is delta time as given to the UKF Update function.
 *		The function must return the measurement corresponding to the given state.
 *
 *	Matrix<FT,numStates,1> predictState( const Matrix<FT,numStates,1> &state, const Matrix<FT,numPredInputs,1> &data, FT dt )
 *		state is the state vector, data is the custom input data to the UKF Predict function, dt is delta time.
 *		The function must return the predicted state according to the arguments passed.
 *
 */

template <class T, 
	const int numStates,		/* num states */
	const int numInputs,		/* num inputs */
	const int numPredInputs,	/* num inputs for prediction */
	bool checkForPositiveDefinite = true /* if true then check for positive definite matrix before Cholesky() */
	>
class UKF
{

private:
	T	filterData;	/* needs to be instantiated */

	enum { 
		L = numStates,	/* number of states to track */
		M = numInputs,  //numInputs;	/* number of inputs to KF */
		N = numPredInputs
	};

	Matrix<FT,L,L>	Q;	/* process noise cov matrix */
	Matrix<FT,M,M>	R;	/* measurement noise variance */

	Matrix<FT,L,L>	P;	/* state covariance matrix */

	/** UKF constants **/
	static const FT	UKF_alpha	= 1e-3;
	static const FT	UKF_beta	= 2.0;
	static const FT	UKF_kappa	= 0.0;
		
	FT	sqWeight;	/* sqrt(L+lambda) */
	FT	UKF_lambda;
	FT	UKF_Ws0;
	FT	UKF_Wc0;
	FT	UKF_Wsi;
	FT	UKF_Wci;

	Matrix<FT,L,L>			sqMatrix;	/* preallocate square root matrix */
	Matrix<FT,L,2*L + 1>	SP;	/* preallocate sigma points */
	Matrix<FT,M,2*L + 1>	Ysp;
	Matrix<FT,L,2*L + 1>	Xsp;

	/** measurement variance */
	Matrix<FT,L,1>	X;	/* state Vector */

public:
	inline	UKF() {
		/* nothing */
	}

	inline void	getStateVector( Matrix<FT,L,1> &v ) {
		v = X;
	}

	void	printMatrices() {
		std::cout << "X:\n" << X << std::endl;
		std::cout << "P:\n" << P << std::endl;
		std::cout << "Q:\n" << Q << std::endl;
		std::cout << "R:\n" << R << std::endl;
	}

	/**
	 * Init kalman variables and state.
	 * Call setStateVector(), setProcessCovariance() / setProcessCovariance() to set start values -after- this.
	 */
	void	KalmanInit()
	{

			UKF_lambda	= UKF_alpha*UKF_alpha*(L + UKF_kappa) - L;
			UKF_Ws0		= UKF_lambda/(L+UKF_lambda);
			UKF_Wc0		= UKF_Ws0 + ( 1.0 - UKF_alpha*UKF_alpha + UKF_beta );
			UKF_Wsi		= 1.0/((double)2.0*(L+UKF_lambda));
			UKF_Wci		= UKF_Wsi;
			sqWeight	= sqrt( L + UKF_lambda );
		

		sqMatrix.setZero();
		Ysp.setZero();
		Xsp.setZero();
		SP.setZero();

		P.setIdentity();

		R.setIdentity();
		Q.setIdentity();
	}

	inline void	setStateVector( Matrix<FT,L,1>	&st ) {
		X = st;
	}

	inline void	setProcessCovariance( Matrix<FT,L,L> &sQ ) {
		Q = sQ;
	}

	inline void	setMeasurementCovariance( Matrix<FT,M,M> &sR ) {
		R = sR;
	}


	/**
	 * Kalman - Update state 
	 *
	 * @param iter		Current iteration number, only for testing purposes.
	 * @param inData	Current measured input
	 * @param dt		Time between consecutive kalman updates.
	 *
	 */
	void	KalmanUpdate( int iter, const Matrix<FT,M,1> &inData, FT dt )
	{
		Matrix<FT,M,1>	angleError;
		Matrix<FT,M,M>	Pzz;
		Matrix<FT,L,M>	Pxz;
		Matrix<FT,M,1>	Y;
		Matrix<FT,M,M>	Pzz_inv;
		Matrix<FT,L,M> K;

		recalculateSigmaPoints();

			/* Project sigma points through h */
			for (int i=0; i < 2*L+1; i++) {
				Ysp.template block<M,1>(0,i) = filterData.measure( SP.template block<L,1>(0,i), dt );
			}
			
			/* Weight sigma points */
			Y.setZero();
			for (int i=1; i < 2*L+1; i++)
				Y += Ysp.template block<M,1>(0,i);
			
			Y *= UKF_Wsi;
			Y += UKF_Ws0 * Ysp.template block<M,1>(0,0);

			/* Weight sigma points for P */

			/* TODO: optimize */
			Pzz.setZero();
			Pxz.setZero();
			for (int i=1; i < 2*L+1; i++) {
				Pzz	+= ( Ysp.template block<M,1>(0,i) - Y ) * (Ysp.template block<M,1>(0,i) - Y).transpose();
				Pxz	+= ( SP.template block<L,1>(0,i) - X ) * ( Ysp.template block<M,1>(0,i) - Y ).transpose();
			}
			
			Pzz *= UKF_Wci;
			Pxz *= UKF_Wci;

			Pzz += UKF_Wc0*( Ysp.template block<M,1>(0,0) - Y)*(Ysp.template block<M,1>(0,0) - Y).transpose() ;
			Pzz += R;
			
			Pxz	+= UKF_Wc0*( SP.template block<L,1>(0,0) - X ) * (Ysp. template block<M,1>(0,0) - Y).transpose();

			Pzz.computeInverse( &Pzz_inv );
			K = Pxz * Pzz_inv;

			X += K*( inData - Y );

			Matrix<FT,M,L>	Kt;
			Kt = K.transpose();

			P = P - K*Pzz*Kt;

	}

	/** 
	 * Kalman - Predict state 
	 *
	 * @param iter			Current iteration number, only for testing purposes.
	 * @param inPred		Measurements for prediction
	 * @param dt			Time between consecutive kalman updates
	 *
	 */
	void	KalmanPredict( int iter, const Matrix<FT,N,1> &inPred, FT dt )
	{
		recalculateSigmaPoints();
		
		/* Project sigma points through h */
		for (int i=0; i < 2*L+1; i++) {
			Xsp.template block<L,1>(0,i) = filterData.predictState( SP.template block<L,1>(0,i), inPred, dt );
		}

		/* Weight sigma points */
		X = UKF_Ws0 * Xsp.template block<L,1>(0,0);
		for (int i=1; i < 2*L+1; i++) {
			X += UKF_Wsi * Xsp.template block<L,1>(0,i);
		}

		/** Calculate P **/
		/** TODO: optimize */
		P.setZero();
		
		for (int i=1; i < 2*L + 1; i++)
			P += UKF_Wci*( Xsp.template block<L,1>(0,i) - X ) * ( Xsp.template block<L,1>(0,i) - X ).transpose();

		//P *= UKF_Wci;
		P += UKF_Wc0 * ( Xsp.template block<L,1>(0,0) - X ) * ( Xsp.template block<L,1>(0,0) - X ).transpose();
		
		P += Q;


	}



private:
	/** Recalcs sigma points for filtering */
	void	recalculateSigmaPoints() 
	{
		
	#if 0
		Matrix<FT,L,L>	temp;
		temp = (L+UKF_lambda)*P;
		
		sqMatrix = temp.llt().matrixL();
	#else
		#if 1
			Matrix<FT,L,L> temp;
			temp = 16384*P;			// scaling - is this neccesary?
			sqMatrix = temp.llt().matrixL()*sqWeight/128;
		#else
			sqMatrix	= sqWeight*P.llt().matrixL();
		#endif
	#endif

			/** Calculate sigma points **/
			SP.template block<L,1>(0,0)	= X;
			
			if ( checkForPositiveDefinite )
			{
				if ( !P.llt().isPositiveDefinite() ) {
					printf("Err positive def\n");
					std::cout << P << std::endl;
					exit(-1);
				}
			}

			for (int i=1; i < L+1; i++) {
				SP.template block<L,1>(0,i) = X + sqMatrix.template block<L,1>(0,i-1);
			}
			for (int i=L+1; i < 2*L + 1; i++) {
				SP.template block<L,1>(0,i) = X - sqMatrix.template block<L,1>(0,i-L-1);
			}
	}
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW



};

#endif /* _oukf_h_ */
