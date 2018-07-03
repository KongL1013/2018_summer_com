#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "DroneCompetition.h"
#include <QtWidgets/QApplication>
#include "estimator.h"
#include "droneInfo.h"
#include "infoUpdate.h"
#include "projectOne.h"
#include "positionController.h"
#include "imageProcess.h"
#include "qout.h"
#include <iostream>

DroneInfo drone_info;
Estimator estimator_thread("estimator_thread");
ImageProcess  image_process_thread("image_process_thread");
ProjectOne project_one_thread("project_one_thread");

IMURetriever imu_thread("imu_thread");
GPSRetriever gps_thread("gps_thread");
BaroRetriever baro_thread("baro_thread");
MagRetriever mag_thread("mag_thread");
ImgRetriever img_thread("img_thread");

QOUT qout_thread("qout_thread");

msr::airlib::MultirotorRpcLibClient client;

int main(int argc, char *argv[])
{
	
	estimator_thread.start();
	image_process_thread.start();
	
	project_one_thread.start();
	imu_thread.start();
	gps_thread.start();
	baro_thread.start();
	mag_thread.start();
	img_thread.start();
	qout_thread.start();

	QApplication a(argc, argv);
	DroneCompetition w;
	w.show();
	return a.exec();
}
