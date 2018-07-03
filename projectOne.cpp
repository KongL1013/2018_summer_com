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
#include "positionController.h"

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

	sleep(2); //连接后姿态估计线程会标定当前零漂

	/*while (1)
	{
		sleep(1);
	}*///for manual control

	/* This is about strategy :D */

	client.enableApiControl(true);

	while (! client.armDisarm(true))
	{
		msleep(10);
	}
	while (! client.takeoff(50))
	{
		msleep(100);
	}
	sleep(1);
	
	float pitch, roll, throttle, yaw_rate, duration;

	pitch = 0.0f;
	roll = 0.0f;
	throttle = 1.f;
	yaw_rate = 0.f;
	duration = 4.f;

	  

	/*give velocity setpoints
	controller.giveVelSp(0.0f, 0.0f, 1.0f, 0.0f);
	*/
	//Controller controller_thread("controller_thread");
	///*give position setpoints*/
	//controller_thread.givePosSp(0.0f, 0.0f, -5.0f, 0.f);
	//controller_thread.start();


	while (true)
	{
		/* Direct angular control flight for test */
		client.moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration); //Paras unknown meaning

		msleep(duration * 1000);

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}