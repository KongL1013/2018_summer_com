#include "imageProcess.h"
#include "droneInfo.h"
#include <iostream>

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
//extern msr::airlib::MultirotorRpcLibClient client;


ImageProcess::ImageProcess(QString name) :b_stopped(false)
{

}

ImageProcess::~ImageProcess()
{
	stop();
	quit();
	wait();
}

void ImageProcess::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void ImageProcess::run()
{
	msr::airlib::MultirotorRpcLibClient client;

	while (true)
	{
		//msleep(100); // 10Hz Max

		auto mag = client.getMagnetometerdata(0.03);

		/* Add your code here */
		// Create temperary variables to continue data process and then give the value to "drone_info"
		// Don't forget to add lock
		// e.g.
		// {
		//    QMutexLocker data_locker(&drone_info.data_mutex);
		//    drone_info.attitude.anglar_velocity.pitch_rate = pitch_rate;
		// }



		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}