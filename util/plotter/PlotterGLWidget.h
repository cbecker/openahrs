/*
 *  This is part of openAHRS
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


#ifndef _plotterglwidget_h_
#define	_plotterglwidget_h_

#include <QGLWidget>

class	PlotterGLWidget : public QGLWidget
{
Q_OBJECT

public:
	PlotterGLWidget( QWidget *parent = 0 );

	//redraws using new pitch/roll/yaw
	// angles in radians
	void	setPRY( double p, double r, double y );

protected:
	double pitch,roll,yaw;

	void	initializeGL();
	void	resizeGL(int w, int h);
	void	paintGL();
};


#endif

