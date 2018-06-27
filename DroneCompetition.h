#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DroneCompetition.h"
#include "painterWiget.h"
#include <QTime>
#include <QTimer>
#include <QLabel>
#include <QPainter>
#include <QBitmap>
#include <QMessageBox>
#include <QDir>
#include <QImage>  

class DroneCompetition : public QMainWindow
{
	Q_OBJECT

public:
	DroneCompetition(QWidget *parent = Q_NULLPTR);

	StatusPainter *status_painter; //For panel

	QTime system_time; // System time

private:
	Ui::DroneCompetitionClass ui;

	void get_Painter_Address(StatusPainter *painter);

	void panel_update();

	void image_update();

	/* Some drone variables to use */
	double baro_height;
	double test1;
	double test2;
	double test3;
	double acc[3];
	double gyo[3];
	double mag[3];
	double gps_n, gps_e, gps_h;
	double baro;
	double zest[2], xest[2], yest[2];

	QImage img_front_rgb;
	QImage img_front_dep;
	QImage img_down_rgb;

	bool save_img_front_rgb;
	bool save_img_front_dep;
	bool save_img_down_rgb;

	int timer_counter;

private slots:
	void timer_update();

	void pushButton_FrontImgRGB_Cap_Clicked();
	void pushButton_FrontImgDep_Cap_Clicked();
	void pushButton_DownImgRGB_Cap_Clicked();
};


