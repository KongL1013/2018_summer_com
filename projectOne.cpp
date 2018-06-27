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

	sleep(2); //连接后姿态估计线程会标定当前零漂

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
	msleep(500);
	
	float pitch, roll, throttle, yaw_rate, duration;

	pitch = 0.0f;
	roll = 0.f;
	throttle = 0.9f;
	yaw_rate = 0.f;
	duration = 8.f;

	while (true)
	{
		client.enableApiControl(false);

		/* Set angluar control values here */
		
		/* Direct angular control flight for test */
		client.moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration); //Paras unknown meaning
		msleep(8000);  
		
		/* Assign values */
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.angluar_setpoint.pitch = pitch;
			drone_info.angluar_setpoint.roll = roll;
			drone_info.angluar_setpoint.throttle = throttle;
			drone_info.angluar_setpoint.yaw_rate = yaw_rate;
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}