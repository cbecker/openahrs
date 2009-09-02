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



#include <openAHRS/util/util.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>
#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN


namespace openAHRS { namespace util 
{
	/** Calculates DCM fro roll, pitch and yaw **/
	Matrix<FT,3,3>	calcDCM( FT roll, FT pitch, FT yaw )
	{
		Matrix<FT,3,3>	dcm;

		FT	cr,cp,cy;
		FT	sr,sp,sy;

		cr	= cos(roll); cp = cos(pitch); cy = cos(yaw);
		sr	= sin(roll); sp = sin(pitch); sy = sin(yaw);

		dcm(0,0) = cp*cy;				dcm(0,1) = cp*sy;				dcm(0,2) = -sp;
		dcm(1,0) = sr*sp*cy - cr*sy;	dcm(1,1) = sr*sp*cy + cp*cy;	dcm(1,2) = sr*cp;
		dcm(2,0) = cr*sp*cy - sr*sy;	dcm(2,1) = cr*sp*sy - sr*cy;	dcm(2,2) = cr*cp;

		return	dcm;
	}


	int kbhit(void)
	{
	  struct termios oldt, newt;
	  int ch;
	  int oldf;

	  tcgetattr(STDIN_FILENO, &oldt);
	  newt = oldt;
	  newt.c_lflag &= ~(ICANON | ECHO);
	  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	  ch = getchar();

	  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	  fcntl(STDIN_FILENO, F_SETFL, oldf);

	  if(ch != EOF)
	  {
		ungetc(ch, stdin);
		return 1;
	  }

	  return 0;
	}

	Matrix<FT,4,4>	calcQOmega( double p, double q, double r )
	{
		Matrix<FT,4,4>	ret;

		ret	<<	0, -p, -q, -r, \
				p,  0,  r, -q, \
				q, -r,  0,  p, \
				r,  q, -p,  0;

		return ret;
	}


	Matrix<FT,3,4>	calcQMeas( const Matrix<FT,4,1>	&q )
	{
		Matrix<FT,3,4>	ret;
		
		#define	e0	q[0]
		#define	ex	q[1]
		#define	ey	q[2]
		#define	ez	q[3]

		/******
		 * DROLL/DQ 
		 * ***/
		{
			FT	K1	= 1 - 2*ex*ex - 2*ey*ey;
			FT	K1K1	= K1*K1;

			FT	K	= 2*(e0*ex + ey*ez);
			FT	K2	= 1 + K*K/(K1K1);
			
			FT	K1K2	= K1*K2;

			/** droll/dq **/
			ret(0,0)	= 2*ex/K1K2;
			ret(0,1)	= (2*e0/K1 + 4*ex*K/K1K1)/K2;
			ret(0,2)	= (2*ez/K1 + 4*ey*K/K1K1)/K2;
			ret(0,3)	= 2*ey/K1K2;
		}

		
		/****
		 * DPITCH / DQ **
		 *
		 */
		{
			FT	_T	= 2*ex*ez - 2*e0*ey;

			_T	= -(_T+1)*(_T-1);
			if ( _T < 0 )
				std::cout << "Err NAN Sqrt\n";
			
			/** TODO: check for NaN **/
			FT	D	= sqrt(_T);

			ret(1,0)	=  2*ey/D;
			ret(1,1)	= -2*ez/D;
			ret(1,2)	=  2*e0/D;
			ret(1,3)	= -2*ex/D;
		}

		/****
		 * DYAW / DQ **
		 *
		 */
		{
			FT	K1	= 1 - 2*ey*ey - 2*ez*ez;
			FT	K1K1	= K1*K1;

			FT	K	= 2*(e0*ez + ex*ey);
			FT	K2	= 1 + K*K/(K1K1);
			
			FT	K1K2	= K1*K2;

			/** dyaw/dq **/
			ret(2,0)	= 2*ez/K1K2;
			ret(2,1)	= 2*ey/K1K2;
			ret(2,2)	= (2*ex/K1 + 4*ey*K/K1K1)/K2;
			ret(2,3)	= (2*e0/K1 + 4*ez*K/K1K1)/K2;
		}
		
		#undef	e0
		#undef	ex
		#undef	ey
		#undef	ez

		return ret;
	}
	
	Matrix<FT,3,1>		quatToEulerNorm( Matrix<FT,4,1> q )
	{
		q.normalize();
		Matrix<FT,3,1>	ret;
		
		/** ROLL **/
		ret[0]	=	atan2( 2*(q[0]*q[1] + q[2]*q[3]),
							1 - 2*(q[1]*q[1] + q[2]*q[2]) );
		/** PITCH **/
		FT	temp	= 2*(q[1]*q[3] - q[0]*q[2]);
		if ( temp > 1.0 )
			temp  = 1.0;
		if ( temp < -1.0 )
			temp = -1.0;

		ret[1]	=	-asin( temp );

		/** YAW **/
		ret[2]	=	atan2( 2*(q[0]*q[3] + q[1]*q[2]),
							1 - 2*(q[2]*q[2] + q[3]*q[3]) );

		return ret;
	}

	Matrix<FT,3,1>		quatToEuler( const Matrix<FT,4,1> &q )
	{
		Matrix<FT,3,1>	ret;
		
		/** ROLL **/
		ret[0]	=	atan2( 2*(q[0]*q[1] + q[2]*q[3]),
							1 - 2*(q[1]*q[1] + q[2]*q[2]) );
		/** PITCH **/
		FT	temp	= 2*(q[1]*q[3] - q[0]*q[2]);
		if ( temp > 1.0 )
			temp  = 1.0;
		if ( temp < -1.0 )
			temp = -1.0;

		ret[1]	=	-asin( temp );

		/** YAW **/
		ret[2]	=	atan2( 2*(q[0]*q[3] + q[1]*q[2]),
							1 - 2*(q[2]*q[2] + q[3]*q[3]) );

		return ret;
	}

	/**
	 * Euler to quaternion
	 *
	 * input: [ roll, pitch, yaw ]'
	 * output: [ e0 ex ey ez ]'
	 */
	Matrix<FT,4,1>		eulerToQuat( const Matrix<FT,3,1> &e )
	{
		Matrix<FT,4,1>	ret;

		FT	r	= e[0]/2;
		FT	p	= e[1]/2;
		FT	y	= e[2]/2;

		FT	Cr	= cos(r);
		FT	Cp	= cos(p);
		FT	Cy	= cos(y);

		FT	Sr	= sin(r);
		FT	Sp	= sin(p);
		FT	Sy	= sin(y);

		ret[0]	=	Cr*Cp*Cy + Sr*Sp*Sy;
		ret[1]	=	Sr*Cp*Cy - Cr*Sp*Sy;
		ret[2]	=	Cr*Sp*Cy + Sr*Cp*Sy;
		ret[3]	=	Cr*Cp*Sy - Sr*Sp*Cy;

		return ret;
	}

	void	accelToPR( const Matrix<FT,3,1> &accels, Matrix<FT,3,1> &angles )
	{
		angles(0)	= atan2( -accels[1], accels[2] );
		angles(1)	= asin( -accels[0]/accels.norm() );
	}


	float	calcHeading( const Matrix<FT,3,1> &magn, const Matrix<FT,3,1> &attitude )
	{
		//precalculate
		float	Cp	= cos( attitude[1] );
		float	Sp	= sin( attitude[1] );
		float	Cr	= cos( attitude[0] );
		float	Sr	= sin( attitude[0] );

		/**
		 * Convert to earth fixed frame
		 */
		float	Hx	= magn[0]*Cp + magn[1]*Sp*Sr - magn[2]*Sp*Cr;
		float	Hy	= magn[1]*Cr  + magn[2]*Sr;

	//	cout << "Hx: " << Hx << endl;
	//	cout << "Hy: " << Hy << endl;

		return	atan2(-Hy,Hx);
	}


	void	quatNormalize( Matrix<FT,4,1> &q )
	{
		FT	norm	= sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] +
						q[3]*q[3] );
		q = q / norm;
/*		q[0]	/= norm;
		q[1]	/= norm;
		q[2]	/= norm;
		q[3]	/= norm;*/
	}


	/**
	 * Taken from Numerical Recipes http://finance.bi.no/~bernt/gcc_prog/recipes/recipes/node23.html
	*/
	static double	random_uniform_0_1(void) 
	{
		return	double(rand())/double(RAND_MAX);
	}

	/**
	 * Taken from Numerical Recipes http://finance.bi.no/~bernt/gcc_prog/recipes/recipes/node23.html 
	*/
	double	randomNormal(void)
	{
		double	U1,U2,V1,V2;
		double	S = 2;

		while ( S >= 1 )
		{
			U1	= random_uniform_0_1();
			U2	= random_uniform_0_1();
			V1	= 2.0*U1 - 1.0;
			V2	= 2.0*U2 - 1.0;
			S	= pow(V1,2) + pow(V2,2);
		}

		double	X1	= V1*sqrt((-2.0*log(S))/S);

		return X1;
	}

	Matrix<FT,3,1>	randomVector3( FT mean, FT sqrt_var )
	{
		Matrix<FT,3,1>	temp;
		temp	<<	mean + randomNormal()*sqrt_var ,
					mean + randomNormal()*sqrt_var ,
					mean + randomNormal()*sqrt_var;

		return temp;
	}

}};
