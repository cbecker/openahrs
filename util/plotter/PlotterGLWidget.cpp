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


#include "PlotterGLWidget.h"

#include <QtOpenGL>

GLfloat	LightAmbient[] = { 0.5,0.5,0.5,1.0 };
GLfloat	LightDiffuse[]	= { 1,1,1,1};
GLfloat	LightPosition[] = {2,2,2,1};

PlotterGLWidget::PlotterGLWidget( QWidget *parent ) : QGLWidget(parent)
{
	pitch = roll = yaw = 0;
}

void	PlotterGLWidget::initializeGL()
{
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	glClearColor(0,0,0,0);

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );

	glLightfv( GL_LIGHT1, GL_AMBIENT, LightAmbient );
	glLightfv( GL_LIGHT1, GL_POSITION, LightPosition );
	glLightfv( GL_LIGHT1, GL_DIFFUSE, LightDiffuse );
	glEnable(GL_LIGHT1);

	glEnable(GL_LIGHTING);
}

void	PlotterGLWidget::resizeGL(int width, int height)
{
	glViewport(0,0,(GLsizei)width, (GLsizei)height);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0f, (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void	PlotterGLWidget::setPRY( double p, double r, double y )
{
	pitch = p; roll = r; yaw = y;
	updateGL();
}

//angles in radians
static void	createRotMatrix( GLdouble m[16], GLdouble roll, GLdouble pitch, GLdouble yaw )
{
	double Cp	= cos(pitch);
	double Sp	= sin(pitch);
	double Cy	= cos(yaw);
	double Sy	= sin(yaw);
	double Sr	= sin(roll);
	double Cr	= cos(roll);

#define	M(col,row)	m[4*(row) + (col)]
//#define	M(col,row)	m[(row) + 4*(col)]

	M(0,0)	= Cp*Cy;
	M(0,1)	= Sp*Sr*Cy - Cr*Sy;
	M(0,2)	= Cr*Sp*Cy + Sr*Sy;
	M(0,3)	= 0;

	M(1,0)	= Cp*Sy;
	M(1,1)	= Sp*Sr*Cy + Cr*Cy;
	M(1,2)	= Cr*Sp*Sy - Sr*Cy;
	M(1,3)	= 0;

	M(2,0)	= -Sp;
	M(2,1)	= Sr*Cp;
	M(2,2) = Cr*Cp;
	M(2,3)	= 0;

	//no translation
	M(3,0) = 0;
	M(3,1) = 0;
	M(3,2) = 0;
	M(3,3) = 1;
}

void	PlotterGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

//	glRotatef( roll, 0,0,1 );
//	glRotatef( pitch, 1,0,0 );
//	glRotatef( yaw, 0,0,1 );

	glTranslatef( 0,0,-6);

	GLdouble	matrix[16];

	createRotMatrix( matrix, roll,pitch,yaw );
	glMultMatrixd( matrix );

	createRotMatrix( matrix, 0, 3.14/2, 0 );
	glMultMatrixd( matrix );

	glColor3f( 0.5, 0.5, 1.0 );

	GLUquadric	*q = gluNewQuadric();

	const	GLfloat	material[] = {0.6,0,0.3,0};
	#if 1
		glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, material );
	#else
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_black);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_black);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_black);
		glMultMatrixf(m); /* apply shading matrix */	
	#endif

	#if 1
		gluCylinder( q, 1, 0, 2, 4, 20 );
	#else
		glutSolidTeapot(2);
	#endif
	printf("Paint\n");
}

