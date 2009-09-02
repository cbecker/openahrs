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


#include "Plotter.h"

//not quite tidy but quicker
#include "../../openAHRS/src/util/util.cpp"
#include <Eigen/Geometry>

#define	TO_DEG(x)	((x)*180.0/3.14)

Plotter::Plotter(QWidget *parent) : QMainWindow(parent)
{
	ui.setupUi(this);

	plotTimer	= new QTimer(this);


	//setup colors
	QPen	bluePen(Qt::blue), redPen(Qt::red);
	cP1[0].setPen( bluePen );
	cP2[0].setPen( bluePen );
	cP3[0].setPen( bluePen );
	cP1[1].setPen( redPen );
	cP2[1].setPen( redPen );
	cP3[1].setPen( redPen );

	//setup graphics
	cP1[0].attach( ui.qwtPlot1 );
	cP1[1].attach( ui.qwtPlot1 );
	cP2[0].attach( ui.qwtPlot2 );
	cP2[1].attach( ui.qwtPlot2 );
	cP3[0].attach( ui.qwtPlot3 );
	cP3[1].attach( ui.qwtPlot3 );

	glFrame	= new GLFrame(0);
	glFrame->setVisible(true);

	connect( ui.butClear, SIGNAL(clicked()), this, SLOT(clearPlots()) );

	udpSocket	= new QUdpSocket(this);
	udpSocket->bind( QHostAddress::Any, 4444 );
	connect( udpSocket, SIGNAL(readyRead()),
		this, SLOT(readPendingDatagrams()) );

	//connect( plotTimer, SIGNAL(timeout()), this, SLOT(plotTimerTimeout()) );
	//plotTimer->start(10);
}

//takes quaternion, returns quaternion
static	Matrix<FT,4,1>	correct45Deg( Matrix<FT,4,1>	quat )
{
	static Eigen::Quaternion<FT>	qRot;	//quaternion used to rotate the measurements 45 degrees
	static bool	firstTime = true;
	if	(firstTime) {
		qRot.coeffs()	= openAHRS::util::eulerToQuat(  Matrix<FT,3,1>(0,0,-45*M_PI/180) );
		firstTime = false;
	}


	return (qRot * Eigen::Quaternion<FT>( quat ) ).coeffs();
}

void	Plotter::processDatagram( QByteArray &data )
{
	static int i = 0;
	QString	s(data);

	double r,p,y,b1,b2,b3,ax,ay,az,rh;
	int numArgs = sscanf( qPrintable(s), "Roll:%lf Pitch:%lf Yaw:%lf Bias1:%lf Bias2:%lf Bias3:%lf Ax:%lf Ay:%lf Az:%lf RH:%lf",
					&r,&p,&y,&b1,&b2,&b3, &ax, &ay, &az, &rh );
	if ( numArgs != 10 ) { printf("Err numargs\n"); return; }

	dP1[0].x.push_back( i );
	dP1[1].x.push_back( i );
	dP2[0].x.push_back( i );
	dP2[1].x.push_back( i );
	dP3[0].x.push_back( i );
	dP3[1].x.push_back( i );

	dP1[0].y.push_back( TO_DEG(r) );
	dP2[0].y.push_back( TO_DEG(p) );
	dP3[0].y.push_back( TO_DEG(y) );

	//plot pitch and roll from pure accel measurements for comparison
	Matrix<FT,3,1>	raw;
	raw << atan2(-ay,az),  -ax/sqrt(ax*ax+ay*ay+az*az), rh;
	raw = openAHRS::util::quatToEuler( correct45Deg( openAHRS::util::eulerToQuat(raw) ) );
	

	dP1[1].y.push_back( TO_DEG( raw(0) ) );
	dP2[1].y.push_back( TO_DEG( raw(1) ) );
	dP3[1].y.push_back( TO_DEG( raw(2) ) );


	if ( (i % 100) == 0 ) {
		printf("Bias1: %lf\nBias2:%lf\nBias3:%lf\n\n", b1,b2,b3 );
	}

	i++;

	//only once upon a while
	if ( (i % 20) == 0 )
		glFrame->setPRY( p, r, y );
	refreshPlots();
}

void	Plotter::readPendingDatagrams()
{
	while( udpSocket->hasPendingDatagrams() )
	{
		QByteArray	datagram;
		datagram.resize( udpSocket->pendingDatagramSize() );
		QHostAddress	sender;
		quint16	senderPort;

		udpSocket->readDatagram( datagram.data(), datagram.size(),
									&sender, &senderPort );

		processDatagram(datagram);
	}
}

void	Plotter::refreshPlots()
{
	cP1[0].setData( dP1[0].x, dP1[0].y );
	cP1[1].setData( dP1[1].x, dP1[1].y );
	cP2[0].setData( dP2[0].x, dP2[0].y );
	cP2[1].setData( dP2[1].x, dP2[1].y );
	cP3[0].setData( dP3[0].x, dP3[0].y );
	cP3[1].setData( dP3[1].x, dP3[1].y );


	ui.qwtPlot1->replot();
	ui.qwtPlot2->replot();
	ui.qwtPlot3->replot();
}

void	Plotter::plotTimerTimeout()
{
	static int i = 3;
	printf("ee\n");

	dP1[0].x.push_back( i );
	dP1[1].x.push_back( i );
	dP2[0].x.push_back( i );
	dP2[1].x.push_back( i );
	dP3[0].x.push_back( i );
	dP3[1].x.push_back( i );

	dP1[0].y.push_back( i );
	dP1[1].y.push_back( -i );
	dP2[0].y.push_back( i );
	dP2[1].y.push_back( -i );
	dP3[0].y.push_back( i );
	dP3[1].y.push_back( -i );

	refreshPlots();
	i++;
}

void	Plotter::clearPlots()
{
	dP1[0].x.clear();
	dP1[1].x.clear();
	dP1[0].y.clear();
	dP1[1].y.clear();

	dP2[0].x.clear();
	dP2[1].x.clear();
	dP2[0].y.clear();
	dP2[1].y.clear();

	dP3[0].x.clear();
	dP3[1].x.clear();
	dP3[0].y.clear();
	dP3[1].y.clear();

	refreshPlots();
}

