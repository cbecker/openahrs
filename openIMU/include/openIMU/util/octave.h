/*
 *  Octave file format writer/reader
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

#ifndef __octave_h_
#define	__octave_h_

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

namespace openIMU { namespace octave
{
	/** 
	* @brief		Write array of row-vectors to octave file
	* 
	* @param file		Destination file
	* @param varname	New variable name
	* @param m			Pointer to array of row-vectors
	* @param n			Number of row-vectors in *m
	* 
	* TODO: there is no error checking,
	*		beware of exceptions.
	*/
	template < class T >
	void	writeVectors( ofstream &file, const string &varname, const T *m, int n )
	{
		/** variable start **/
		file	<< "# name: " << varname << "\n";
		file	<< "# type: matrix\n";
		file	<< "# rows: " << m[0].rows() << "\n";
		file	<< "# columns: " << n << "\n";

		file.precision(20);

		for (int i=0; i < m[0].rows(); i++ )
		{
			for (int j=0; j < n; j++ )
				file << m[j](i,0) << " ";

			file << "\n";
		}

		file << "\n";
	}


	/** 
	* @brief			Reads veactor list from file
	* 
	* @param file		File to read
	* @param varname	variable name in file 'file'
	* @param num_read	number of rows read
	* 
	* @return			Requested matrix or NULL on error
	*					Caller should delete the returned matrix after use.
	*
	* TODO:	should be more error prone.
	*		Just for testing purposes now.
	*/
	MatrixXd *	readVectors( ifstream &file, const char *varname, int *num_read )
	{
		int		n;		// number of elements
		int		rows;	// number of rows

		char	temp[120];
		char	temp2[120];

		/* nothing read yet */
		*num_read = 0;

		strcpy( temp2, "# name: ");
		strcat( temp2, varname );

		/** find start line **/
		file.seekg( 0, ios::beg );	//go to start of file

		while ( !file.eof() )
		{
			file.getline( temp, sizeof(temp) );
			if ( file.fail() )
				return NULL;

			if ( strcmp( temp, temp2 ) == 0 )
				break;
		}
		if ( file.eof() )
			return NULL;

//		std::cout << "FOUND!\n";

		/****** VAR TYPE *****/
		file.getline( temp, sizeof(temp) );
		if ( strcmp( temp, "# type: matrix" ) != 0 )
			return NULL;	//error

		/****	NUM ROWS ***/
		file.getline( temp, sizeof(temp) );
		if ( sscanf( temp, "# rows: %d", &rows ) != 1 )
			return NULL;

		/****	NUM COLS ***/
		file.getline( temp, sizeof(temp) );
		if ( sscanf( temp, "# columns: %d", &n ) != 1 )
			return NULL;


	//	std::cout << "good\n" << rows << " " << n << "\n";

		MatrixXd	*ret	= new MatrixXd[n];

		for (int i=0; i < n; i++)
			ret[i]	= MatrixXd(rows,1);

		for (int i=0; i < rows; i++)
			for (int j=0; j < n; j++)
				file >> ret[j](i,0);

		*num_read	= n;
		return ret;
	}
}};

#endif	/* __octave_h_ */

