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
#include "imageProcess.h"
#include <cmath>

#define IMGWIDTH 640
#define IMGHEIGHT 480

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;

Controller controller_thread("controller_thread");
ImageProcess  image_process_thread("image_process_thread");

ProjectOne::ProjectOne(QString name) :
	b_stopped(false)
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

	while (! client.armDisarm(true))
	{
		msleep(10);
	}
	/*while (! client.takeoff(50))
	{
		msleep(50);
	}
	sleep(1);*/

	//Controller controller_thread("controller_thread");
	//ImageProcess  image_process_thread("image_process_thread");

	/*Get Yaw*/
	float headward;
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		headward = drone_info.attitude.angle.yaw;		
	}
	float backward = headward + 3.1415926;

	controller_thread.start();

	show_string("yaw " + QString::number(headward) + "\tback:" + QString::number(backward));

	//int counter = 0;
	double px, py, pz;
	double vx, vy, vz;
	double spx, spy, spz;
	double svx, svy, svz;

	while (true)
	{

		controller_thread.takeoff();
		msleep(1000);

		controller_thread.hover();
		msleep(1000);

		/* Number 1 -> 3*/
		//if(0)
		for (int j = 1; j < 4; j++)
		{
			switch (j)
			{
			case 1:
				spx = 9.5;	spy = -5.2;	spz = -4.0;
				break;
			case 2:
				spx = 10.5;	spy = -11.6;	spz = -4.0;
				break;
			case 3:
				spx = 2.5;	spy = -19.5;	spz = -4.0;
				break;
			}

			// Position control first
			controller_thread.givePosSp(spx, spy, spz, headward);   //隐性改变接口
			while (true)
			{
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					px = drone_info.local_position.position.x;
					py = drone_info.local_position.position.y;
					pz = drone_info.local_position.position.z;
				}

				if (points_close(px, py, pz, spx, spy, spz, 0.8)) break;
			}
			controller_thread.hover();
			msleep(1000);

			/* Land on target board by image*/

			/* Calculate position error by Downside Image */
			controller_thread.hover();
			msleep(250);

			AimBoardType1();

			show_string("going down...");
			while (true) {
				cv::Mat down_mat;
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					drone_info.images.mat_down_rgb.copyTo(down_mat);
				}
				cv::Mat result_down;
				cv::Rect num_rect_down;

				image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);

				cv::imshow("down_mat", down_mat);
				cv::waitKey(1);

				cv::imshow("cut_board", result_down);
				cv::waitKey(1);

				show_string(QString::number(num_rect_down.area()) + "\n");
				if (num_rect_down.area() < 32000) {
					controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.5f, 0.1);
					msleep(500);
					controller_thread.hover();
					msleep(500);
				}
				else {
					break;
				}
			}
			show_string("low enough!");
			AimBoardType1();

			controller_thread.land();
			msleep(3000);

			controller_thread.takeoff();
			msleep(1200);

			controller_thread.hover();
			msleep(1000);

		}

		/* Correct Yaw to headward */
		yawTurnTo(headward);

		/* TODO: Number 3 -> 10*/
		for (int j = 4; j < 11; j++)
		{
			switch (j)
			{
			case 4:
				spx = 15.0;	spy = -30.0;	spz = -7.0;
				break;
			case 5:
				spx = 15.0;	spy = -40.0;	spz = -7.0;
				break;
			case 6:
				spx = 15.0;	spy = -50.0;	spz = -7.0;
				break;
			case 7:
				spx = 15.0;	spy = -60.0;	spz = -5.0;
				break;
			case 8:
				spx = 15.0;	spy = -70.0;	spz = -5.0;
				break;
			case 9:
				spx = 15.0;	spy = -80.0;	spz = -5.0;
				break;
			case 10:
				spx = 15.0;	spy = -90.0;	spz = -5.0;
				break;
			}
			show_string("go to #" + QString::number(j));

			// Position control first
			controller_thread.givePosSp(spx, spy, spz, headward);   //隐性改变接口
			while (true)
			{
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					px = drone_info.local_position.position.x;
					py = drone_info.local_position.position.y;
					pz = drone_info.local_position.position.z;
				}

				if (points_close(px, py, pz, spx, spy, spz, 0.6)) {
					show_string("arrive the line");
					break;
				}
			}
			controller_thread.hover();
			msleep(1000);

			/* TODO: Find and land on board or fly through circle*/
			bool bool_board = true; // otherwise circle

			while (true)
			{
				controller_thread.hover();
				msleep(500);

				cv::Mat down_mat;
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					drone_info.images.mat_down_rgb.copyTo(down_mat);
					px = drone_info.local_position.position.x;
					py = drone_info.local_position.position.y;
					pz = drone_info.local_position.position.z;
				}
				cv::Mat result_down;
				cv::Rect circle_rect;
				cv::Rect board_rect;

				bool found_board =  image_process_thread.downFindRectangle(down_mat, result_down, board_rect);
				bool found_circle = image_process_thread.downFindRedCircle(down_mat, circle_rect);

				if (found_circle) {
					show_string("found circle!");

					//controller_thread.giveAttSp(0.0f, 0.0f, 1.0f, 0.588f, 3.5);
					//sleep(3.8);
					//controller_thread.hover();

					yawTurnTo(backward);

					/*double currentYaw = yaw_est;
					while (true) {
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							currentYaw = drone_info.attitude.angle.yaw;
						}
						msleep(10);
						if (abs(backward - yaw_est) < 0.1) {
							msleep(200);
							controller_thread.hover();
							break;
						}
					}*/

					/* Go to the front of the circle by image*/
					while (true)
					{
						/* Calculate position error by Downside Image */
						controller_thread.hover();
						msleep(500);

						cv::Mat down_mat;
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							drone_info.images.mat_down_rgb.copyTo(down_mat);
							px = drone_info.local_position.position.x;
							py = drone_info.local_position.position.y;
							pz = drone_info.local_position.position.z;
						}
						cv::Mat result_down;
						cv::Rect num_rect_down;

						image_process_thread.downFindRedCircle(down_mat, num_rect_down);

						show_string("CIRC\t" + QString::number(num_rect_down.tl().x) + "\t" + QString::number(num_rect_down.br().x) + "\n");
						if (num_rect_down.br().x - num_rect_down.tl().x > 2 || num_rect_down.br().y - num_rect_down.tl().y > 2) // ??? Why don't work
						{
							int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
							int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

							int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
							int delt_y = center_y - (IMGHEIGHT / 9 * 2); //IMG coordinate

							float pitch_set = 0.f, roll_set = 0.f;
							if (abs(delt_x) < 40 && abs(delt_y) < 40) {
								show_string("close enough, move to front...");
								break;
							}
							if (abs(delt_x) > 40) roll_set = 0.08 * delt_x / abs(delt_x);
							if (abs(delt_y) > 40) pitch_set = 0.08 * delt_y / abs(delt_y);

							
							controller_thread.giveAttSp(pitch_set, roll_set, 0.f, 0.61f, 0.5);
							msleep(500);
							controller_thread.hover();
							msleep(500);

						}
						else
						{
							msleep(500);
							controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.56f, 0.1);
							msleep(200);
							controller_thread.hover();
						}
					}

					msleep(500);
					controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.5f, 5.0);

					while (true)
					{
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							px = drone_info.local_position.position.x;
							py = drone_info.local_position.position.y;
							pz = drone_info.local_position.position.z;
						}

						if (pz > -3.0) {
							break;
							show_string("down to the good level");
						}
						msleep(100);
					}

					controller_thread.hover();
					msleep(1000);

					while (true) {
						/* Fly through circle */
						cv::Mat front_mat;
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							drone_info.images.mat_front_rgb.copyTo(front_mat);
							px = drone_info.local_position.position.x;
							py = drone_info.local_position.position.y;
							pz = drone_info.local_position.position.z;
						}

						Vec3f circle;
						Rect cirRect;
						image_process_thread.frontFindRectangle(front_mat, cirRect);

						cv::imshow("front_mat", front_mat);
						cv::waitKey(10);

						int center_x = cirRect.x + cirRect.width / 2;
						int center_y = cirRect.y + cirRect.height / 2;

						int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
						int delt_y = center_y - (IMGHEIGHT / 2); //IMG coordinate

						float throttle = 0.62f, roll_set = 0.f;
						if (abs(delt_x) < 30 && abs(delt_y) < 30) break;
						/* Control throttle and roll seperately */
						if (abs(delt_x) > 30) {
							roll_set = 0.025 * delt_x / abs(delt_x);
							show_string("adjust roll");
						}
						else if (abs(delt_y) > 30) {
							throttle += -0.05 * delt_y / abs(delt_y);
							show_string("adjust height");
						}

						
						controller_thread.giveAttSp(0.f, roll_set, 0.f, throttle, 0.5);
						msleep(300);
						controller_thread.hover();
						msleep(1000);
					}
					controller_thread.hover();
					show_string("ready to go!");
					controller_thread.giveAttSp(-0.2f, 0.0f, 0.f, 0.6f, 1);
					msleep(1400);
					controller_thread.hover();
					msleep(1000);
					show_string("pass complete");
					yawTurnTo(headward, 0.6);
					controller_thread.hover();
					msleep(100);
					break;
				}
				else if (found_board)
				{
					show_string("found board!");
					/* Land on target board by image*/
					while (true)
					{
						/* Calculate position error by Downside Image */
						controller_thread.hover();
						msleep(500);

						cv::Mat down_mat;
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							drone_info.images.mat_down_rgb.copyTo(down_mat);
							px = drone_info.local_position.position.x;
							py = drone_info.local_position.position.y;
							pz = drone_info.local_position.position.z;
						}
						cv::Mat result_down;
						cv::Rect num_rect_down;

						cv::imshow("down_mat", down_mat);
						cv::waitKey(10);

						image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);

						show_string("RECT\t" + QString::number(num_rect_down.tl().x) + "\t" + QString::number(num_rect_down.br().x) + "\n");
						if (num_rect_down.br().x - num_rect_down.tl().x > 100 || num_rect_down.br().y - num_rect_down.tl().y > 80) // ??? Why don't work
						{
							int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
							int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

							int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
							int delt_y = center_y - IMGHEIGHT / 2; //IMG coordinate

							float pitch_set = 0.f, roll_set = 0.f;
							if (abs(delt_x) < 40 && abs(delt_y) < 40) break;
							if (abs(delt_x) > 40) roll_set = 0.08 * delt_x / abs(delt_x);
							if (abs(delt_y) > 40) pitch_set = 0.08 * delt_y / abs(delt_y);

							msleep(500);
							controller_thread.giveAttSp(pitch_set, roll_set, 0.f, 0.64f, 0.5);
							msleep(500);
							controller_thread.hover();

						}
						else
						{
							msleep(500);
							controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.56f, 0.1);
							msleep(200);
							controller_thread.hover();
						}
					}

					controller_thread.land();
					msleep(3000);

					controller_thread.takeoff();
					msleep(1200);

					controller_thread.hover();
					msleep(1000);

					break;
				}
		
				controller_thread.giveAttSp(0.0f, -0.1f, 0.f, 0.64f, 0.5);
				msleep(1000);
				controller_thread.hover();
				msleep(100);
			}

		}
		
		/* TODO: Find QR codes */

		break;
		

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

bool ProjectOne::points_close(double x1, double y1, double z1, double x2, double y2, double z2, double limit)
{
	if (fabs(x1 - x2) < limit && fabs(y1 - y2) < limit && fabs(z1 - z2) < limit)
		return true;
	else
		return false;
}

//thr 不输入则默认不改变高度 0.587
void ProjectOne::yawTurnTo(double target, double thr)
{
	while (true)
	{
		float yaw_now;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			yaw_now = drone_info.attitude.angle.yaw;
		}
		show_string(QString::number(yaw_now));

		double errorYaw = target - yaw_now;
		if (abs(errorYaw) < 0.0333) {
			controller_thread.hover();
			msleep(1000);
			return;
		}
		else
		{
			double adjustValue = errorYaw / 1.8;
			if (adjustValue > 1.3) {
				adjustValue = 1.3;
			}
			else if (adjustValue < -1.3) {
				adjustValue = -1.3;
			}
			else if (adjustValue > 0 && adjustValue < 0.1) {
				adjustValue = 0.1;
			}
			else if (adjustValue < 0 && adjustValue > -0.1) {
				adjustValue = -0.1;
			}
			controller_thread.giveAttSp(0.f, 0.f, adjustValue, thr, 0.1);
		}
		msleep(200);
	}
}

void ProjectOne::AimBoardType1() {

	show_string("start aiming");

	while (true) {
		cv::Mat down_mat;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.images.mat_down_rgb.copyTo(down_mat);
		}
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);

		cv::imshow("down_mat", down_mat);
		cv::waitKey(1);

		cv::imshow("cut_board", result_down);
		cv::waitKey(1);

		double diagSize = distOfTwoPoint(num_rect_down.tl().x, num_rect_down.tl().y, num_rect_down.br().x, num_rect_down.br().y);

		int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
		int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

		int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
		int delt_y = center_y - IMGHEIGHT / 2; //IMG coordinate
		double rx = delt_x / diagSize;
		double ry = delt_y / diagSize;
		double rdis = distOfTwoPoint(rx, 0, 0, ry);
		//show_string("center_dist: " + QString::number(rdis) + "\n");
		show_string("delt: " + QString::number(rx) + "\t" + QString::number(ry) + "\n");

		float pitch_set = 0.f, roll_set = 0.f;
		if (abs(rx) <= 0.2 && abs(ry) <= 0.2)
		{
			return;
		}
		if (abs(rx) > 0.2)
		{
			roll_set = 0.2 * rx / abs(rx);
		}
		if (abs(ry) > 0.2)
		{
			pitch_set = 0.2 * ry / abs(ry);
		}
		
		double tempTho = 0.58 / cos(pitch_set) / cos(roll_set);
		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, tempTho, 0.5);
		msleep(400);
		controller_thread.hover();
		msleep(600);
	}
}