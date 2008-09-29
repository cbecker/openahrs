/*
 *	Eigen2 library test program
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

//import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

int main()
{
	Matrix3d	m3;
	Matrix3d	invm3;

	m3	<< 1,2,3,4,5,0,7,8,9;

	m3.computeInverse(&invm3);

	cout	<< "------ M3 is " << endl;
	cout << m3 << endl;

	std::cout << "------ Inverse is: \n" << invm3 << std::endl;

	cout	<< endl;
	cout	<< "----> This should be the identity Matrix:\n";
	cout	<< invm3*m3 << endl;

	return 0;
}

