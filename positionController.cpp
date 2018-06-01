#include "positionController.h"
#include "droneInfo.h"
#include <iostream>

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;


Controller::Controller(QString name) :b_stopped(false)
{

}

Controller::~Controller()
{
	stop();
	quit();
	wait();
}

void Controller::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void Controller::run()
{
	while (true)
	{
		msleep(20); // 50Hz Max

		/* Add your controller code here */
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