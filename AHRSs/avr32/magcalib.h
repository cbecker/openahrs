/*
 *  AHRS for AVR32.
 *  Works on the ATNGW100 reference board, see schematics on the hardware folder
 *
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


#ifndef	_magcalib_h_
#define	_magcalib_h_

#include <openAHRS/calib/UKFEllipsoid.h>
#include <openAHRS/util/util.h>
#include <openAHRS/util/matrixserializer.h>

/** Magnetometer calibration **/
class	MagCalib
{
private:
	openAHRS::calib::UKFEllipsoid	SP;
	
public:
	static const int	numParams = 9;

	MagCalib() {
		Matrix<FT,3,1>	estBias;
		estBias << 0,0,0;
		SP.init( 1e-4, estBias, /*est amplitude*/ 0.11e-3, 0, 0, 1e-12 );
	}

	

	void	getParams( Matrix<FT,numParams,1>	&p ){
		SP.getStateVector(p);
	}

	//processes input without modifying calibration parameters
	void	processInput( Matrix<FT,3,1> &meas, Matrix<FT,3,1> &out )
	{
		SP.processInput(meas,out);
	}

	//estimate parameters from measurements
	void	estimateParams( Matrix<FT,3,1>	&meas )
	{
		SP.estimateParams(meas);
	}



	bool	saveParameters( const char *fileName )
	{
		Matrix<FT,numParams,numParams>	P;
		Matrix<FT,numParams,1>	S;
		Matrix<FT, numParams, numParams+1> sm;
		
		SP.getCovarianceMatrix(P);
		SP.getStateVector(S);

		sm.block<numParams,numParams>(0,0)	= P;
		sm.block<numParams,1>(0,numParams)	= S;

		return util::MatrixSerializer::save(sm, fileName);
	}

	bool	loadParameters( const char *fileName )
	{
		Matrix<FT, numParams, numParams+1> sm;
		
		if ( !util::MatrixSerializer::load(sm, fileName) )
			return false;

		SP.setCovarianceMatrix( sm.block<numParams,numParams>(0,0) );
		SP.setStateVector( sm.block<numParams,1>(0,numParams) );

		return true;
	}
};

#endif

