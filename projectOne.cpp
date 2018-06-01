#include "projectOne.h"
#include "droneInfo.h"
#include <iostream>
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;

ProjectOne::ProjectOne(QString name) :b_stopped(false)
{

}

ProjectOne::~ProjectOne()
{
	stop();
	quit();
	wait();
}

void ProjectOne::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void ProjectOne::run()
{
	/* This is about strategy :D */

	//client.enableApiControl(true);
	/*while (! client.armDisarm(false))
	{
		msleep(10);
	}
	msleep(10);
	while (! client.takeoff(50))
	{
		msleep(100);
	}
	msleep(2000);*/
	

	while (true)
	{
		
		
		//client.moveByAngleThrottle(0.1f, 0.f, 0.8f, 0.f, 1.f); //Paras unknown meaning
		msleep(2000); 
		//client.moveByAngleThrottle(-0.1f, 0.f, 0.5f, 0.f, 1.f); //Paras unknown meaning
		msleep(2000);

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