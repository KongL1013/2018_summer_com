#include "DroneCompetition.h"
#include "estimator.h"
#include "droneInfo.h"
#include "qout.h"

extern Estimator estimator_thread;
extern DroneInfo drone_info;
extern QString text_output;
extern QMutex qout_mutex;
extern QOUT qout_thread;

DroneCompetition::DroneCompetition(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	StatusPainter *painter = new StatusPainter();
	ui.tabWidget_PaintArea->addTab(painter, "PANEL");
	ui.tabWidget_PaintArea->setCurrentIndex(0);
	get_Painter_Address(painter);
	
	//Timer for display, 20Hz
	timer_counter = 0;
	QTimer *timer = new QTimer(this);	
    connect(timer, SIGNAL(timeout()), this, SLOT(timer_update()));
	timer->start(50);

	//Connect items
	connect(ui.pushButton_FrontImgRGB_Cap, SIGNAL(clicked()), this, SLOT(pushButton_FrontImgRGB_Cap_Clicked()));
	connect(ui.pushButton_FrontImgDep_Cap, SIGNAL(clicked()), this, SLOT(pushButton_FrontImgDep_Cap_Clicked()));
	connect(ui.pushButton_DownImgRGB_Cap, SIGNAL(clicked()), this, SLOT(pushButton_DownImgRGB_Cap_Clicked()));

	//Settings
	ui.pushButton_FrontImgRGB_Cap->setEnabled(false);
	ui.pushButton_FrontImgDep_Cap->setEnabled(false);
	ui.pushButton_DownImgRGB_Cap->setEnabled(false);

	save_img_front_rgb = false;
	save_img_front_dep = false;
	save_img_down_rgb = false;
}

void DroneCompetition::get_Painter_Address(StatusPainter *painter)
{
	status_painter = painter;
}

void  DroneCompetition::panel_update()
{
	// Update attitude data
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		status_painter->pitchd = drone_info.attitude.angle_d.pitch_d;
		status_painter->rolld = drone_info.attitude.angle_d.roll_d;
		status_painter->pitch = drone_info.attitude.angle.pitch;
		status_painter->roll = drone_info.attitude.angle.roll;
		status_painter->compassd = -drone_info.attitude.angle_d.yaw_d + 180;
		
		test1 = drone_info.test_value.test1;
		test2 = drone_info.test_value.test2;
		test3 = drone_info.test_value.test3;
	}
	// Draw
	status_painter->update();

	ui.label_Test1->setText(QString::number(test1));
	ui.label_Test2->setText(QString::number(test2));
	ui.label_Test3->setText(QString::number(test3));

}

void DroneCompetition::image_update()
{
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		img_front_rgb = drone_info.images.front_rgb;
		img_front_dep = drone_info.images.front_depth;
		img_down_rgb = drone_info.images.down_rgb;
	}

	if (img_front_rgb.height() > 0)
	{
		ui.pushButton_FrontImgRGB_Cap->setEnabled(true);
		QPainter painter(&img_front_rgb);
		QImage resultImg = img_front_rgb.scaled(ui.label_FrontImage->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui.label_FrontImage->setPixmap(QPixmap::fromImage(resultImg));
		ui.label_FrontImage->show();

		if (save_img_front_rgb)
		{
			system_time = QTime::currentTime();
		    char name[60];
			sprintf(name, "E:\\competition2018\\Images\\FrontRGB\\fr-%d-%d-%d.png", system_time.hour(), system_time.minute(), system_time.second());
			img_front_rgb.save(name, "PNG", 100);
			save_img_front_rgb = false;
		}
		
	}
	else
		ui.pushButton_FrontImgRGB_Cap->setEnabled(false);

	if (img_front_dep.height() > 0)
	{
		ui.pushButton_FrontImgDep_Cap->setEnabled(true);
		QPainter painter(&img_front_dep);
		QImage resultImg = img_front_dep.scaled(ui.label_FrontImageDep->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui.label_FrontImageDep->setPixmap(QPixmap::fromImage(resultImg));
		ui.label_FrontImageDep->show();

		if (save_img_front_dep)
		{
			system_time = QTime::currentTime();
			char name[60];
			sprintf(name, "E:\\competition2018\\Images\\FrontDepth\\fd-%d-%d-%d.png", system_time.hour(), system_time.minute(), system_time.second());
			img_front_dep.save(name, "PNG", 100);

			save_img_front_dep = false;
		}

	}
	else
		ui.pushButton_FrontImgDep_Cap->setEnabled(false);

	if (img_down_rgb.height() > 0)
	{
		ui.pushButton_DownImgRGB_Cap->setEnabled(true);
		QPainter painter(&img_down_rgb);
		QImage resultImg = img_down_rgb.scaled(ui.label_DownImage->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui.label_DownImage->setPixmap(QPixmap::fromImage(resultImg));
		ui.label_DownImage->show();

		if (save_img_down_rgb)
		{
			system_time = QTime::currentTime();
			char name[60];
			sprintf(name, "E:\\competition2018\\Images\\DownRGB\\dr-%d-%d-%d.png", system_time.hour(), system_time.minute(), system_time.second());
			img_down_rgb.save(name, "PNG", 100);
			
			save_img_down_rgb = false;
		}

	}
	else
		ui.pushButton_DownImgRGB_Cap->setEnabled(false);
}

void DroneCompetition::timer_update()
{
	timer_counter++;
	if (timer_counter > 100) timer_counter = 0;

	panel_update(); 
	
	if (timer_counter % 2 == 0)  // 10Hz
	{
		image_update();
	}

	if (timer_counter % 4 == 0)
	{
		QString show_data;
		// qout
		{
			QMutexLocker data_locker(&qout_thread.qout_mutex);
			show_data = qout_thread.text_output;
		}
		qout_thread.clear();
		ui.textBrowser->append(show_data);
	}
}

void  DroneCompetition::pushButton_FrontImgRGB_Cap_Clicked()
{
	save_img_front_rgb = true;
}

void DroneCompetition::pushButton_DownImgRGB_Cap_Clicked()
{
	save_img_down_rgb = true;
}
void DroneCompetition::pushButton_FrontImgDep_Cap_Clicked()
{
	save_img_front_dep = true;
}