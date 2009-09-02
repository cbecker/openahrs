#include <iostream>
#include <stdio.h>

#include <openAHRS/kalman/UKF.h>

#define	NUMINPUTS	3
#define	NUMSTATES	4
#define	NUMPREDINPUTS	5

template<class T>
struct	TestFuncs
{
	inline Matrix<FT,NUMINPUTS,1> measure( Matrix<FT,NUMSTATES,1> state, FT dt )
	{
		return	state.block<3,3>(0,0);
	}

	inline Matrix<FT,NUMSTATES,1> predictState( Matrix<FT,NUMSTATES,1> state, Matrix<FT,NUMPREDINPUTS,1> data, FT dt )
	{
		return	state;
	}
};

UKF< TestFuncs<int>, NUMSTATES, NUMINPUTS, NUMPREDINPUTS, false >	ukfTest;

int main()
{

	Matrix<FT,NUMINPUTS,1>	in;
	Matrix<FT,NUMPREDINPUTS,1>	inPred;

	ukfTest.KalmanInit();
	ukfTest.KalmanUpdate( 1, in, 20e-3 );
	ukfTest.KalmanPredict( 1, inPred, 20e-3 );

	return 0;
}

