
#include  "estimator.h"
#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"
#include <functional>
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "droneInfo.h"
#include <QTime>
#include "qout.h"
#include "attitude_estimator.h"


using namespace std;
using namespace msr::airlib;
using namespace stateestimation;


extern DroneInfo drone_info;
//extern msr::airlib::MultirotorRpcLibClient client;

const unsigned long preferedClockTime = 100;	//in ms
const double pi = 3.1415926;
const double pi_half = 3.1415926 / 2.0;

Estimator::Estimator(QString name):b_stopped(false)
{
	
}

Estimator::~Estimator()
{
	stop();
	quit();
	wait();
}

void Estimator::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void Estimator::run()
{
	msr::airlib::MultirotorRpcLibClient client;	
	
	/* Define variables here */
	float pitch = 0.f, roll = 0.f, yaw = 0.f;
	float pitch_rate = 0.f, roll_rate = 0.f, yaw_rate = 0.f;
	
	QTime clockTime;
	QTime calTime;

	AttitudeEstimator est(false);
	//est.setAttitude(1.0, 0.0, 0.0, 0.0);

	clockTime.start();
	calTime.start();


	while (true)
	{	
		while (clockTime.elapsed() < preferedClockTime) {
			msleep(1);
		}
		msleep(100);
		show_string("HAHA");
		clockTime.restart();

		/* !!!!!! NOTE: Drone Infomation update has been moved to "infoUpdate.h/.cpp" with multithread working !!!!!! */

		//auto baro = client.getBarometerdata(0.03); //NOTE: Time out value need to be tuned !!! Seems 0.03 is min val.
		//auto mag = client.getMagnetometerdata(0.03);
		//auto imu = client.getImudata(0.03);
		//auto gps = client.getGpsLocation();

		/*double dt = calTime.elapsed() * 0.001;
		calTime.restart();
		est.update(dt, imu.angular_velocity.x(), imu.angular_velocity.y(), imu.angular_velocity.z(),
			imu.linear_acceleration.x(), imu.linear_acceleration.y(), imu.linear_acceleration.z(),
			mag.magnetic_field_body.x(), mag.magnetic_field_body.y(), mag.magnetic_field_body.z());
		double q[4];
		est.getAttitude(q);
		Quaternionr quat(q[0], q[1], q[2], q[3]);
		auto rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);
		for (int i = 0; i < 3; i++) {
			if (rpy[i] > pi_half) {
				rpy[i] -= pi;
			}
			else if (rpy[i] < -pi_half) {
				rpy[i] += pi;
			}
		}*/

		/* Add attitude estimator here */
		// Read original data
		//static QTime time;
		//static bool startTimer;
		//if (startTimer) {
		//	int circleTime = time.elapsed();
		//	std::cout << "circle time:" << circleTime << endl;
		//	drone_info.test_value.test1 = circleTime;
		//}
		//time.start();
		//startTimer = true;
		//
		//int timeOnReading = time.elapsed();
		//std::cout << "read time:" << timeOnReading << endl;

		//eg for test
		//if (yaw < 1.8) yaw += 0.001;


		/* Attitude Update */
		{
			QMutexLocker data_locker(&drone_info.data_mutex);

			//drone_info.test_value.test1 = gps.latitude;

			
			//drone_info.test_value.test1 = rpy[2] / pi * 180; //test
			/*double x = mag.magnetic_field_body.x();
			double y = mag.magnetic_field_body.y();
			auto s = sqrt(x * x + y * y);

			drone_info.test_value.test1 = gps.latitude;
			drone_info.test_value.test2 = gps.longitude;
			drone_info.test_value.test3 = gps.altitude;

			drone_info.attitude.anglar_velocity.pitch_rate = imu.angular_velocity.x();
			drone_info.attitude.anglar_velocity.roll_rate = imu.angular_velocity.y();
			drone_info.attitude.anglar_velocity.yaw_rate = imu.angular_velocity.z();

			drone_info.attitude.angle.pitch = rpy[1];
			drone_info.attitude.angle.roll = rpy[0];
			drone_info.attitude.angle.yaw = rpy[2];

			drone_info.degree_values_cal();*/
		}
		

		
	

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}	
	}
	
}

