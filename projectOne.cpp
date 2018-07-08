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
	/*while (! client.takeoff(50))
	{
		msleep(50);
	}
	sleep(1);*/
	
	Controller controller_thread("controller_thread");
	ImageProcess  image_process_thread("image_process_thread");

	/*Get Yaw*/
	float yaw_est;
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		yaw_est = drone_info.attitude.angle.yaw;		
	}
	controller_thread.start();

	show_string("yaw " + QString::number(yaw_est));

	//int counter = 0;
	double px, py, pz;
	double vx, vy, vz;
	double spx, spy, spz;
	double svx, svy, svz;

	while (true)
	{
		/*give velocity setpoints
		controller.giveVelSp(0.0f, 0.0f, 1.0f, 0.0f);
		*/

		controller_thread.takeoff();
		msleep(1000);

		controller_thread.hover();
		msleep(1000);

		/* Number 1 -> 3*/
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
			controller_thread.givePosSp(spx, spy, spz, yaw_est);   //隐性改变接口
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
					controller_thread.giveAttSp(pitch_set, roll_set, 0.f, 0.65f, 0.5);
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

		}

		/* Correct Yaw to 0 */
		float yaw_expt = yaw_est;
		while (true)
		{
			float yaw_now;
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				yaw_now = drone_info.attitude.angle.yaw;
			}

			if (yaw_now - yaw_est > 0.1)
				controller_thread.giveAttSp(0.f, 0.f, -0.1f, 0.61f, 0.1);
			else if (yaw_now - yaw_est < -0.1)
				controller_thread.giveAttSp(0.f, 0.f, 0.1f, 0.61f, 0.1);
			else
				break;

			msleep(200);
		}

		/* TODO: Number 3 -> 10*/
		for (int j = 4; j < 11; j++)
		{
			switch (j)
			{
			case 4:
				spx = 15.0;	spy = -30.0;	spz = -5.0;
				break;
			case 5:
				spx = 15.0;	spy = -40.0;	spz = -5.0;
				break;
			case 6:
				spx = 15.0;	spy = -50.0;	spz = -5.0;
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

			// Position control first
			controller_thread.givePosSp(spx, spy, spz, yaw_est);   //隐性改变接口
			while (true)
			{
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					px = drone_info.local_position.position.x;
					py = drone_info.local_position.position.y;
					pz = drone_info.local_position.position.z;
				}

				if (points_close(px, py, pz, spx, spy, spz, 0.6)) break;
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

				if (found_board)
				{
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
				else // No board found
				{
					bool found_circle = image_process_thread.downFindRedCircle(result_down, circle_rect);

					if (found_circle)
					{
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

							cv::imshow("down_mat", down_mat);
							cv::waitKey(10);

							image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);

							show_string("RECT\t" + QString::number(num_rect_down.tl().x) + "\t" + QString::number(num_rect_down.br().x) + "\n");
							if (num_rect_down.br().x - num_rect_down.tl().x > 2 || num_rect_down.br().y - num_rect_down.tl().y > 2) // ??? Why don't work
							{
								int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
								int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

								int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
								int delt_y = center_y - (IMGHEIGHT / 9 * 4 ); //IMG coordinate

								float pitch_set = 0.f, roll_set = 0.f;
								if (abs(delt_x) < 40 && abs(delt_y) < 40) break;
								if (abs(delt_x) > 40) roll_set = 0.08 * delt_x / abs(delt_x);
								if (abs(delt_y) > 40) pitch_set = 0.08 * delt_y / abs(delt_y);

								msleep(500);
								controller_thread.giveAttSp(pitch_set, roll_set, 0.f, 0.61f, 0.5);
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
							
							if (pz > -2.0)
								break;
							msleep(100);
						}

						controller_thread.hover();
						msleep(1000);

						/* Fly through circle */
						cv::Mat front_mat;
						{
							QMutexLocker data_locker(&drone_info.data_mutex);
							drone_info.images.mat_front_rgb.copyTo(front_mat);
							px = drone_info.local_position.position.x;
							py = drone_info.local_position.position.y;
							pz = drone_info.local_position.position.z;
						}

						std::vector<Vec3f> circles;
						image_process_thread.frontFindCircle(front_mat, circles);

						cv::imshow("front_mat", front_mat);
						cv::waitKey(10);

						int center_x = circles[0][0];
						int center_y = circles[0][1];

						int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
						int delt_y = center_y - (IMGHEIGHT / 2); //IMG coordinate

						float throttle = 6.f, roll_set = 0.f;
						if (abs(delt_x) < 40 && abs(delt_y) < 40) break;
						/* Control throttle and roll seperately */
						if (abs(delt_x) > 40) roll_set = 0.08 * delt_x / abs(delt_x);
						else if (abs(delt_y) > 40) throttle += -0.01 * delt_y / abs(delt_y);

						msleep(500);
						controller_thread.giveAttSp(0.f, roll_set, 0.f, throttle, 0.5);
						msleep(500);
						controller_thread.hover();
					}
				}
		
				controller_thread.giveAttSp(0.f, -0.1f, 0.f, 0.64f, 0.5);
				msleep(1000);
				controller_thread.hover();
				msleep(1000);
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