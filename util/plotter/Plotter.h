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


#ifndef _plotter_h_
#define	_plotter_h_

#include <QMainWindow>
#include <QTimer>
#include <QtNetwork/QUdpSocket>
#include "GLFrame.h"

#include "ui_plotter.h"

#include <qwt_plot_curve.h>
#include <qwt_array.h>

//static QwtPlotCurve	*curve1, *curve2;
//static QwtArray<double>	xData, yData;

struct	PlotterData
{
	QwtArray<double>	x,y;
};

class	Plotter : public QMainWindow
{
	Q_OBJECT

public:
	Plotter(QWidget *parent = NULL);

private:
	GLFrame			*glFrame;

	QwtPlotCurve	cP1[2];
	QwtPlotCurve	cP2[2];
	QwtPlotCurve	cP3[2];

	PlotterData		dP1[2];
	PlotterData		dP2[2];
	PlotterData		dP3[2];

	QUdpSocket		*udpSocket;

private:
	Ui_plotter	ui;

	QTimer	*plotTimer;
	void	refreshPlots();

	void	processDatagram( QByteArray &datagram );

public slots:
	void	plotTimerTimeout();
	void	readPendingDatagrams();
	void	clearPlots();
};

#endif
