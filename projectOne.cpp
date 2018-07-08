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
#include "qout.h"

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

	/**不控制*/
	/*while (1) 
	{
		sleep(1);
	}*///for manual control

	/* This is about strategy :D */

	client.enableApiControl(true);

	/**悬停模式*/
	//client.moveByAngleThrottle(0, 0, 5.88, 0, 0.1);
	//client.hover();
	//while (1);

	while (! client.armDisarm(true))
	{
		msleep(10);
	}
	while (! client.takeoff(50))
	{
		msleep(50);
	}
	sleep(1);
	
	Controller controller_thread("controller_thread");
	/*give position setpoints*/
	float yaw_est;
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		yaw_est = drone_info.attitude.angle.yaw;		
		//controller_thread.setmode(controller_thread.POSCTRL);		//显性改变接口	
	}
	controller_thread.start();

	//int counter = 0;
	while (true)
	{
		/*give velocity setpoints
		controller.giveVelSp(0.0f, 0.0f, 1.0f, 0.0f);
		*/


		controller_thread.givePosSp(-11.0f, -62.0f, -5.0f, yaw_est);   //隐性改变接口

		msleep(5000);
		controller_thread.hover();
		msleep(5000);


		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}