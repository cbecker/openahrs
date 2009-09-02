/*
 *  Matrix serializer
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


#ifndef _matserializer_h_
#define	_matserializer_h_

#include <stdio.h>

namespace	openAHRS	{
namespace	util		{
namespace	MatrixSerializer
{
	template <class T>
	bool	save( const T &m, const char *fileName )
	{
		FILE	*f = fopen(fileName,"wb");
		if ( f == NULL )
			return false;

		bool	ret = true;

		if ( fwrite( m.data(), m.rows()*m.cols(), sizeof(m.data()[0]), f ) != sizeof(m.data()[0]) ) {
		//if ( fwrite( m.data(), m.rows()*m.cols(), sizeof(1), f ) != sizeof(1) ) {
			ret = false; goto out;
		}
	
	out:
		fclose(f);
		return ret;
	}

	template <class T>
	bool	load( T &m, const char *fileName )
	{
		FILE	*f	= fopen(fileName, "rb");
		if ( f == NULL )
			return false;

		bool	ret = true;
		if ( fread( m.data(), m.rows()*m.cols(), sizeof(m.data()[0]), f ) != sizeof(m.data()[0]) ) {
			ret = false; goto out;
		}

	out:
		fclose(f);
		return ret;
	}
}}};

#endif

