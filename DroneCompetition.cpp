#include "DroneCompetition.h"
#include "estimator.h"
#include "droneInfo.h"
#include "qout.h"

#include "imageProcess.h"

#include <qevent.h>


#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include "positionController.h"
#include "qout.h"
#include "imageProcess.h"
#include <cmath>
#include <QImage> 
#include <qimagereader.h>
#include <algorithm>
#include <QFile>

#include "projectOne.h"

extern Estimator estimator_thread;
extern DroneInfo drone_info;
extern QString text_output;
extern QMutex qout_mutex;
extern QOUT qout_thread;

////TEMP ��
//extern ImageProcess image_process_thread;
//cv::Mat target_mat;
//cv::Mat result_down;
//cv::Rect num_rect_down;
//std::vector<int> ids;
//std::vector<std::vector<cv::Point2f>> corners;
//bool res;
//
//msr::airlib::MultirotorRpcLibClient client123;
//
//ProjectOne pj1("fuck you");
//
//void getImg(cv::Mat & img, int type) {
//	using namespace msr::airlib;
//	ImageCaptureBase::ImageRequest req;
//	std::vector<ImageCaptureBase::ImageRequest> requests;
//	switch (type) {
//	default:
//	case 0:
//		req.camera_id = 0;
//		req.image_type = ImageCaptureBase::ImageType::Scene;
//		req.pixels_as_float = false;
//		requests.push_back(req);
//		break;
//	case 1:
//		req.camera_id = 3;
//		req.image_type = ImageCaptureBase::ImageType::Scene;
//		req.pixels_as_float = false;
//		requests.push_back(req);
//		break;
//	}
//
//	auto images = client123.simGetImages(requests);
//	auto picData = images[0].image_data_uint8;
//	QByteArray dataArray = QByteArray::fromRawData(reinterpret_cast<const char*>(picData.data()), picData.size());
//	QBuffer buffer(&dataArray);
//	QImageReader reader(&buffer);
//	static QImage Qimg;
//	Qimg = reader.read();
//	{
//		QMutexLocker data_locker(&drone_info.data_mutex);
//		if (type == 1) {
//			drone_info.images.down_rgb = Qimg;
//		}
//		else if (type == 0) {
//			drone_info.images.front_rgb = Qimg;
//		}
//	}
//	img = cv::Mat(Qimg.height(), Qimg.width(), CV_8UC4, (void*)Qimg.constBits(), Qimg.bytesPerLine());
//}

DroneCompetition::DroneCompetition(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	StatusPainter *painter = new StatusPainter();
	ui.tabWidget_PaintArea->addTab(painter, "PANEL");
	ui.tabWidget_PaintArea->setCurrentIndex(1);
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
	connect(ui.pushButton_FrontImg_Cap, SIGNAL(clicked()), this, SLOT(pushButton_FrontImg_Cap_Clicked()));

	//Settings
	ui.pushButton_FrontImgRGB_Cap->setEnabled(false);
	ui.pushButton_FrontImgDep_Cap->setEnabled(false);
	ui.pushButton_DownImgRGB_Cap->setEnabled(false);
	ui.pushButton_FrontImg_Cap->setEnabled(false);

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
		status_painter->roll = -drone_info.attitude.angle.roll; //gcy_changed
		status_painter->compassd = -drone_info.attitude.angle_d.yaw_d;
		//gcy changed
		
		test1 = drone_info.test_value.test1;
		test2 = drone_info.test_value.test2;
		test3 = drone_info.test_value.test3;
		for (int i = 0; i < 3; i++) {
			gyo[i] = drone_info.test_value.gyo[i];
			acc[i] = drone_info.test_value.acc[i];
			mag[i] = drone_info.test_value.mag[i];
		}
		gps_e = drone_info.test_value.gps_e;
		gps_n = drone_info.test_value.gps_n;
		gps_h = drone_info.test_value.gps_h;
		baro = drone_info.test_value.baro;
		xest[0] = drone_info.local_position.position.x;
		yest[0] = drone_info.local_position.position.y;
		zest[0] = drone_info.local_position.position.z;
		xest[1] = drone_info.local_position.velocity.vx;
		yest[1] = drone_info.local_position.velocity.vy;
		zest[1] = drone_info.local_position.velocity.vz;

	}
	// Draw
	status_painter->update();

	ui.label_Test1->setText(QString::number(test1));
	ui.label_Test2->setText(QString::number(test2));
	ui.label_Test3->setText(QString::number(test3));
	ui.acc_x->setText(QString::number(acc[0]));
	ui.acc_y->setText(QString::number(acc[1]));
	ui.acc_z->setText(QString::number(acc[2]));
	ui.gyo_x->setText(QString::number(gyo[0]));
	ui.gyo_y->setText(QString::number(gyo[1]));
	ui.gyo_z->setText(QString::number(gyo[2]));
	ui.mag_x->setText(QString::number(mag[0]));
	ui.mag_y->setText(QString::number(mag[1]));
	ui.mag_z->setText(QString::number(mag[2]));
	ui.gps_e->setText(QString::number(gps_e));
	ui.gps_n->setText(QString::number(gps_n));
	ui.gps_h->setText(QString::number(gps_h));
	ui.baro->setText(QString::number(baro));

	ui.px->setText(QString::number(xest[0]));
	ui.py->setText(QString::number(yest[0]));
	ui.pz->setText(QString::number(zest[0]));

	ui.vx->setText(QString::number(xest[1]));
	ui.vy->setText(QString::number(yest[1]));
	ui.vz->setText(QString::number(zest[1]));
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
		ui.pushButton_FrontImg_Cap->setEnabled(true);
		QPainter painter(&img_front_rgb);
		QImage resultImg = img_front_rgb.scaled(ui.label_FrontImage->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui.label_FrontImage->setPixmap(QPixmap::fromImage(resultImg));
		ui.label_FrontImage->show();

		if (save_img_front_rgb)
		{
			//gcy changed
			system_time = QTime::currentTime();
			QString floder = ui.etSavePath->text();
			if (floder.isEmpty()) {
				floder = "FrontRGB";
			}
			QDir dir("E:\\competition2018\\Images\\" + floder);
			if (!dir.exists()) {
				dir.mkpath("E:\\competition2018\\Images\\" + floder);
			}
			char name[80];
			QString savePath = "E:\\competition2018\\Images\\" + floder + "\\fr-%d-%d-%d.png";
			QByteArray pathTemp = savePath.toLatin1();
			sprintf(name, pathTemp.data(), system_time.hour(), system_time.minute(), system_time.second());
			img_front_rgb.save(name, "PNG", 100);
			save_img_front_rgb = false;
		}
		
	}
	else
		ui.pushButton_FrontImgRGB_Cap->setEnabled(false);

	if (img_front_dep.height() > 0)
	{
		ui.pushButton_FrontImgDep_Cap->setEnabled(true);
		ui.pushButton_FrontImg_Cap->setEnabled(true);

		QPainter painter(&img_front_dep);
		QImage resultImg = img_front_dep.scaled(ui.label_FrontImageDep->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		ui.label_FrontImageDep->setPixmap(QPixmap::fromImage(resultImg));
		ui.label_FrontImageDep->show();

		if (save_img_front_dep)
		{
			//gcy changed
			system_time = QTime::currentTime();
			QString floder = ui.etSavePath->text();
			if (floder.isEmpty()) {
				floder = "FrontDepth";
			}
			QDir dir("E:\\competition2018\\Images\\" + floder);
			if (!dir.exists()) {
				dir.mkpath("E:\\competition2018\\Images\\" + floder);
			}
			char name[80];
			QString savePath = "E:\\competition2018\\Images\\" + floder + "\\fd-%d-%d-%d.png";
			QByteArray pathTemp = savePath.toLatin1();
			sprintf(name, pathTemp.data(), system_time.hour(), system_time.minute(), system_time.second());
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
			//gcy changed
			double h;
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				h = drone_info.test_value.test3 + 2.355;
			}
			QString height = QString::number(h);
			system_time = QTime::currentTime();
			QString floder = ui.etSavePath->text();
			if (floder.isEmpty()) {
				floder = "DownRGB";
			}
			QDir dir("E:\\competition2018\\Images\\" + floder);
			if (!dir.exists()) {
				dir.mkpath("E:\\competition2018\\Images\\" + floder);
			}
			char name[80];
			QString savePath = "E:\\competition2018\\Images\\" + floder + "\\dr-" + height + "-%d-%d-%d.png";
			QByteArray pathTemp = savePath.toLatin1();
			sprintf(name, pathTemp.data(), system_time.hour(), system_time.minute(), system_time.second());
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

	if (timer_counter % 5 == 0)
	{
		QString show_data;
		// qout
		{
			QMutexLocker data_locker(&qout_thread.qout_mutex);
			show_data = qout_thread.text_output;
		}
		qout_thread.clear();
		if(show_data.size() > 0)
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

void DroneCompetition::pushButton_FrontImg_Cap_Clicked()
{
	save_img_front_rgb = true;
	save_img_front_dep = true;
}

void DroneCompetition::keyPressEvent(QKeyEvent  *event) {
	return;
	//bool res;
	//switch (event->key()) {
	////case Qt::Key_D:
	////	getImg(target_mat, 1);
	////	show_string(QString::number(target_mat.rows));
	////	res = image_process_thread.downFindRectangle(target_mat, result_down, num_rect_down);
	//case Qt::Key_1:
	//	ui.etSavePath->setText("1");
	//	break;
	//case Qt::Key_2:
	//	ui.etSavePath->setText("2");
	//	break;
	//case Qt::Key_3:
	//	ui.etSavePath->setText("3");
	//	break;
	//case Qt::Key_4:
	//	ui.etSavePath->setText("4");
	//	break;
	//case Qt::Key_5:
	//	ui.etSavePath->setText("5");
	//	break;
	//case Qt::Key_6:
	//	ui.etSavePath->setText("6");
	//	break;
	//case Qt::Key_7:
	//	ui.etSavePath->setText("7");
	//	break;
	//case Qt::Key_8:
	//	ui.etSavePath->setText("8");
	//	break;
	//case Qt::Key_9:
	//	ui.etSavePath->setText("9");
	//	break;
	//case Qt::Key_0:
	//	ui.etSavePath->setText("0");
	//	break;
	//case Qt::Key_P:
	//	ui.etSavePath->setText("10");
	//	break;
	//case Qt::Key_M:

	//case Qt::Key_Up:
	//	client.enableApiControl(true);
	//	return;
	//case Qt::Key_Left:
	//	client.enableApiControl(true);
	//	pj1.MoveToOpenLoop(-2, 0);
	//	return;
	//case Qt::Key_Right:
	//	client.enableApiControl(true);
	//	pj1.MoveToOpenLoop(2, 0);
	//	return;
	//case Qt::Key_Down:
	//	client.enableApiControl(false);
	//	return;
	//case Qt::Key_H:
	//	client.enableApiControl(true);
	//	client.moveByAngleThrottle(0, 0, 0.58, 0, 50);
	//	client.hover();
	//	return;
	//default:
	//	return;
	//}
	//if (ui.cbIsFront->isChecked()) {
	//	save_img_front_rgb = true;
	//}
	//else {
	//	save_img_down_rgb = true;
	//}
}