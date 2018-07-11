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

#define STARTNUM 0
#define MANUAL_CONTROL true

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;

Controller controller_thread("controller_thread");
ImageProcess  image_process_thread("image_process_thread");

ProjectOne::ProjectOne(QString name) :
	b_stopped(false),
	stone_area(false)
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

	while (MANUAL_CONTROL)
	{
		sleep(1);
	}//for manual control

	client.enableApiControl(true);

	while (! client.armDisarm(true))
	{
		msleep(10);
	}

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
	double spx, spy, spz, spx1, spx2, spy1, spy2, spz1, spz2;
	double svx, svy, svz;
	

	/** start */
	while (true)
	{
		controller_thread.takeoff();
		msleep(1000);

		controller_thread.hover();
		msleep(1000);

		/* Number 1 -> 3*/
		if(STARTNUM == 0)
		for (int j = 1; j < 4; j++)
		{
			switch (j)
			{
			case 1:
				spx = 9.5;	spy = -5.2;	spz = -3.5;
				break;
			case 2:
				spx = 10.5;	spy = -11.6;	spz = -3.5;
				break;
			case 3:
				spx = 2.5;	spy = -19.5;	spz = -3.5;
				break;
			}

			MoveTo(spx, spy, spz, headward, 0.8);

			LandOnBoard();

			controller_thread.takeoff();
			msleep(1200);

			controller_thread.hover();
			msleep(1000);

		}

		/* Correct Yaw to headward */
		
		/* TODO: Number 3 -> 10*/
		int targetNum = 4;
		if (STARTNUM != 0) {
			targetNum = STARTNUM;
		}
		for (; targetNum < 11; targetNum++)
		{
			yawTurnTo(headward);
			int searchMode = 1; //1 or 2
			switch (targetNum)
			{
			case 4:
				spx1 = 13.5; spy1 = -29; spz1 = -7.0;
				spx2 = -1; spy2 = -30.0; spz2 = -6.0;
				stone_area = false;
				break;
			case 5:
				spx1 = 11.7; spy1 = -40.0; spz1 = -6.0;
				spx2 = 3; spy2 = -40.0; spz2 = -6.0;
				stone_area = false;
				break;
			case 6:
				spx1 = 1; spy1 = -50.0; spz1 = -6.0;
				spx2 = -8; spy2 = -50.0; spz2 = -6.0;
				stone_area = false;
				break;
			case 7:
				spx1 = -1.2; spy1 = -61.0; spz1 = -6.0;
				spx2 = -10; spy2 = -61.0; spz2 = -6.0;
				stone_area = true;
				break;
			case 8:
				spx1 = 2; spy1 = -73.0; spz1 = -6.0;
				spx2 = -10; spy2 = -73.0; spz2 = -6.0;
				stone_area = true;
				break;
			case 9:
				spx1 = -9.5; spy1 = -82.3; spz1 = -6.0;
				spx2 = -9.5; spy2 = -83.5; spz2 = -6.0;
				stone_area = false;
				break;
			case 10:
				spx1 = 8; spy1 = -93.0; spz1 = -5.0;
				spx2 = 8; spy2 = -93.0; spz2 = -5.0;
				stone_area = false;
				break;
			}
			show_string("go to #" + QString::number(targetNum) + "\n");

			while (true)
			{
				if (searchMode == 1) {
					MoveTo(spx1, spy1, spz2, headward, 0.8);
				}
				else if (searchMode == 2){
					MoveTo(spx2, spy1, spz2, headward, 0.8);
				}
				else if (searchMode == 3) {
					controller_thread.giveAttSp(0.0f, 0.1f, 0.f, 0.60f, 0.5);
					msleep(500);
				}

				controller_thread.hover();
				if (searchMode == 3) {
					msleep(500);
				}
				else {
					msleep(1000);
				}

				cv::Mat down_mat;
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					drone_info.images.mat_down_rgb.copyTo(down_mat);
				}
				cv::Mat result_down;
				cv::Rect circle_rect;
				cv::Rect board_rect;

				bool found_board;
				if (stone_area) {
					found_board = image_process_thread.downFindRectangle_stone(down_mat, result_down, board_rect);
				}
				else {
					found_board = image_process_thread.downFindRectangle(down_mat, result_down, board_rect);
				}
				
				bool found_circle = image_process_thread.downFindRedCircle(down_mat, circle_rect);

				if (found_circle) {
					show_string("found circle!");

					yawTurnTo(backward);

					/* Go to the back of the circle by image*/
					AimCircleDown();

					msleep(500);
					QuickDownTo(-3.0, 0.5);

					AimCircleFront();
					
					controller_thread.hover();
					show_string("ready to go!");
					controller_thread.giveAttSp(-0.2f, 0.0f, 0.f, 0.6f, 1);
					msleep(1700);
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
					LandOnBoard();
					controller_thread.takeoff();
					msleep(1200);
					controller_thread.hover();
					msleep(1000);
					break;
				}

				show_string("found nothing!\n");

				if (searchMode == 1) {
					searchMode = 2;
				}
				else if (searchMode == 2) {
					show_string("misfound in 2 places!");
					searchMode = 3;
				}
				else if (searchMode == 3) {
					{
						QMutexLocker data_locker(&drone_info.data_mutex);
						px = drone_info.local_position.position.x;
					}
					if (abs(spx1 - px) < 1) {
						searchMode = 1;
						show_string("Mode 3 FAIL, do it again\n");
					}
				}
			}
		}
		
		controller_thread.hover();
		msleep(2000);

		/* TODO: Find QR codes */
		show_string("going to QR areas...\n");

		vec3f_t QRPoses[13] = {

			{7.5, -107.0, -10.5},
			{16.3, -107.1, -10.5 },
			{14.0, -113.1, -10.5},
			{18.8, -118.6, -10.5},
			{16.7, -129.5, -10.5},
			{10.4, -122.0, -10.5},
			{7.4, -129.8, -10.5},
			{3.3, -121.2, -10.5},
			{- 4.0, -129.1, -11.4},
			{- 16.1, -123.7, -10.5},
			{- 16.0, -116.3, -10.5},
			{- 5.4, -116.6, -10.5},
			{- 3.6, -107.1, -10.5},
		};
		
		yawTurnTo(backward);

		for (int i = 0; i < 13; i++) {
			while (true) {
				MoveTo(QRPoses[i].x, QRPoses[i].y, QRPoses[i].z, backward, 0.5);
				cv::Mat down_mat;
				{
					QMutexLocker data_locker(&drone_info.data_mutex);
					drone_info.images.mat_down_rgb.copyTo(down_mat);
				}
				cv::Mat result_down;
				cv::Rect num_rect_down;

				if (image_process_thread.downFindTree(down_mat, result_down, num_rect_down)) {
					break;
				}
			}

			AimStubDown();

			StubToQR();

			QuickUpTo(-10, 0.65);
		}

		MoveTo(0, -5, -9, backward, 0.8);
		controller_thread.hover();
		msleep(2000);
		MoveTo(0, 0, -7, backward, 0.8);
		LandOnBoard();

		show_string("mission completed");

		
		

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
		//show_string(QString::number(yaw_now));

		double errorYaw = target - yaw_now;
		if (abs(errorYaw) < 0.0333) {
			controller_thread.hover();
			msleep(1000);
			return;
		}
		else
		{
			double adjustValue = errorYaw / 1.5;
			adjustValue = constrainNum(adjustValue, 0.1, 1.5);
			controller_thread.giveAttSp(0.f, 0.f, adjustValue, thr, 0.1);
		}
		msleep(200);
	}
}

void ProjectOne::LandOnBoard() {
	/* Land on target board by image*/

	/* Calculate position error by Downside Image */
	controller_thread.hover();
	msleep(250);

	AimBoardHigh();

	show_string("going down to proper height...\n");
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
			controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.475f, 0.1);
			msleep(500);
			controller_thread.hover();
			msleep(500);
		}
		else {
			break;
		}
	}
	show_string("low enough!\n");
	AimBoardLow();

	controller_thread.land();
	msleep(3000);
}

void ProjectOne::AimBoardHigh() {

	show_string("start aiming at high level\n");

	while (true) {
		cv::Mat down_mat;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.images.mat_down_rgb.copyTo(down_mat);
		}
		cv::Mat result_down;
		cv::Rect num_rect_down;

		if (!stone_area) {
			image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);
		}
		else {
			image_process_thread.downFindRectangle_stone(down_mat, result_down, num_rect_down);
		}

		cv::imshow("down_mat", down_mat);
		cv::waitKey(1);

		cv::imshow("cut_board", result_down);
		cv::waitKey(1);

		int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
		int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

		int delt_x = center_x - IMGWIDTH / 2; // + = need go right
		int delt_y = center_y - IMGHEIGHT / 2; // + = need go back

		show_string("center_dist: " + QString::number(delt_x) + "\t" + QString::number(delt_y) + "\n");

		float pitch_set = 0.f, roll_set = 0.f;
		if (abs(delt_x) <= 30 && abs(delt_y) <= 30)
		{
			return;
		}
		if (abs(delt_x) > 30)
		{
			roll_set = 0.002 * delt_x;
			roll_set = constrainNum(roll_set, 0.05, 0.15);
		}
		if (abs(delt_y) > 30)
		{
			pitch_set = 0.002 * delt_y;
			pitch_set = constrainNum(pitch_set, 0.05, 0.15);
		}

		show_string("coor num: " + QString::number(roll_set) + "\t" + QString::number(pitch_set) + "\n");
		
		double tempTho = 0.58 / cos(pitch_set) / cos(roll_set);
		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, tempTho, 0.5);
		msleep(400);
		controller_thread.hover();
		msleep(1000);
	}
}

void ProjectOne::AimBoardLow() {

	show_string("start aiming at low level\n");

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
		double tollerance = diagSize / 9.0;

		int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
		int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

		int delt_x = center_x - IMGWIDTH / 2; // + = need go right
		int delt_y = center_y - IMGHEIGHT / 2; // + = need go back

		show_string("center_dist: " + QString::number(delt_x) + "\t" + QString::number(delt_y) + "\t" + 
			QString::number(tollerance) + "\n");

		float pitch_set = 0.f, roll_set = 0.f;
		if (abs(delt_x) <= tollerance && abs(delt_y) <= tollerance)
		{
			return;
		}
		if (abs(delt_x) > tollerance)
		{
			roll_set = 0.002 * delt_x;
			roll_set = constrainNum(roll_set, 0.035, 0.15);
		}
		if (abs(delt_y) > tollerance)
		{
			pitch_set = 0.002 * delt_y;
			pitch_set = constrainNum(pitch_set, 0.035, 0.15);
		}

		show_string("coor num: " + QString::number(roll_set) + "\t" + QString::number(pitch_set) + "\n");

		double tempTho = 0.58 / cos(pitch_set) / cos(roll_set);
		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, tempTho, 0.5);
		msleep(400);
		controller_thread.hover();
		msleep(1000);
	}
}

void ProjectOne::AimCircleDown() {
	while (true)
	{
		/* Calculate position error by Downside Image */
		controller_thread.hover();
		msleep(500);

		cv::Mat down_mat;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.images.mat_down_rgb.copyTo(down_mat);
		}
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindRedCircle(down_mat, num_rect_down);

		if (num_rect_down.br().x - num_rect_down.tl().x > 2 || num_rect_down.br().y - num_rect_down.tl().y > 2) // ??? Why don't work
		{
			int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
			int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

			int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
			int delt_y = center_y - (IMGHEIGHT / 9 * 2); //IMG coordinate

			float pitch_set = 0.f, roll_set = 0.f;
			if (abs(delt_x) < 40 && abs(delt_y) < 40) {
				controller_thread.hover();
				msleep(500);
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
}

void ProjectOne::AimCircleFront() {
	show_string("start aimming circle\n");
	while (true) {
		/* Fly through circle */
		cv::Mat front_mat;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.images.mat_front_rgb.copyTo(front_mat);
		}

		Vec3f circle;
		Rect cirRect;
		image_process_thread.frontFindRectangle(front_mat, cirRect);

		cv::imshow("front_mat", front_mat);
		cv::waitKey(10);

		int center_x = cirRect.x + cirRect.width / 2;
		int center_y = cirRect.y + cirRect.height / 2;

		//show_string(QString::number(cirRect.area()) + "\n");
		int circSize = cirRect.area();

		int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
		int delt_y = center_y - (IMGHEIGHT / 2); //IMG coordinate

		float throttle = 0.588f, roll_set = 0.f, pitch_set = 0.f;
		if (abs(delt_x) < 45 && abs(delt_y) < 45 && abs(circSize) > 80000 && abs(circSize) < 180000) {
			break;
		}
		/* Control throttle and roll seperately */
		if (abs(delt_x) > 45) { // 20
			roll_set = 0.025 * delt_x / abs(delt_x);
			show_string("adjust roll\t");
		}
		if (abs(delt_y) > 45) { // 17
			float adjustV = -4e-3 * delt_y;
			adjustV = constrainNum(adjustV, 0.08, 0.2);
			throttle += adjustV;
			show_string("adjust height\t");
		}
		if (abs(delt_x) < 120 && abs(delt_y) < 100) {
			if (abs(circSize) < 80000) {
				show_string("too far!\t");
				pitch_set = -0.05;
			}
		}
		if (abs(circSize) > 180000) {
			show_string("too close!\t");
			pitch_set = 0.05;
		}

		throttle = throttle / cos(pitch_set) / cos(roll_set);

		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, throttle, 0.5);
		msleep(300);
		controller_thread.hover();
		msleep(1000);
		show_string("\n");
	}
}

void ProjectOne::MoveTo(double spx, double spy, double spz, double yaw, double tollerance) {
	show_string("moving to: " + QString::number(spx) + " " + QString::number(spy)
		+ " " + QString::number(spz) + "\n");
	double px, py, pz;
	controller_thread.givePosSp(spx, spy, spz, yaw);
	while (true)
	{
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			px = drone_info.local_position.position.x;
			py = drone_info.local_position.position.y;
			pz = drone_info.local_position.position.z;
		}
		msleep(1);
		if (points_close(px, py, pz, spx, spy, spz, tollerance)) break;
	}
	show_string("arrive!\n");
	controller_thread.hover();
	msleep(1000);
}

void ProjectOne::QuickDownTo(double height, double thr) {
	controller_thread.giveAttSp(0.f, 0.f, 0.f, thr, 5.0);
	show_string("going down to" + QString::number(height) + "...\n");

	double pz;

	while (true)
	{
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			pz = drone_info.local_position.position.z;
		}

		if (pz > height) {
			break;
		}
		msleep(100);
	}

	controller_thread.hover();
	msleep(1000);
}

void ProjectOne::QuickUpTo(double height, double thr) {
	controller_thread.giveAttSp(0.f, 0.f, 0.f, thr, 5.0);
	show_string("going down to" + QString::number(height) + "...\n");

	double pz;

	while (true)
	{
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			pz = drone_info.local_position.position.z;
		}

		if (pz < height) {
			break;
		}
		msleep(100);
	}

	controller_thread.hover();
	msleep(1000);
}

void ProjectOne::StubToQR() {
	show_string("moving to QR...\n");
	controller_thread.hover();
	msleep(400);
	controller_thread.giveAttSp(0.25, 0.0, 0, 0.6, 1);
	msleep(1000);
	controller_thread.giveAttSp(-0.25, 0.0, 0, 0.6, 0.8);
	msleep(800);
	controller_thread.hover();
	msleep(500);
	QuickDownTo(-5.7, 0.5);
	//TODO take picture
	controller_thread.hover();
	msleep(1000);
	show_string("arrive QR...\n");
}

//E:\\competition2018\\Images\\

void ProjectOne::SaveImg(QString path, QString fileName, QImage & image) {
	QDir dir(path);
	if (!dir.exists()) {
		dir.mkpath(path);
	}
	QString savePath = path + fileName;
	image.save(savePath, "PNG", 100);
}

void ProjectOne::AimStubDown() {
	show_string("start aiming at stub\n");

	while (true) {
		cv::Mat down_mat;
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.images.mat_down_rgb.copyTo(down_mat);
		}
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindTree(down_mat, result_down, num_rect_down);

		cv::imshow("down_mat", down_mat);
		cv::waitKey(1);

		cv::imshow("cut_board", result_down);
		cv::waitKey(1);

		double diagSize = distOfTwoPoint(num_rect_down.tl().x, num_rect_down.tl().y, num_rect_down.br().x, num_rect_down.br().y);
		double tollerance = 40;

		int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
		int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

		int delt_x = center_x - IMGWIDTH / 2; // + = need go right
		int delt_y = center_y - IMGHEIGHT / 2; // + = need go back

		show_string("center_dist: " + QString::number(delt_x) + "\t" + QString::number(delt_y) + "\n");

		float pitch_set = 0.f, roll_set = 0.f;
		if (abs(delt_x) <= tollerance && abs(delt_y) <= tollerance)
		{
			controller_thread.hover();
			msleep(500);
			return;
		}
		if (abs(delt_x) > tollerance)
		{
			roll_set = 0.002 * delt_x;
			roll_set = constrainNum(roll_set, 0.035, 0.15);
		}
		if (abs(delt_y) > tollerance)
		{
			pitch_set = 0.002 * delt_y;
			pitch_set = constrainNum(pitch_set, 0.035, 0.15);
		}

		show_string("correct num: " + QString::number(roll_set) + "\t" + QString::number(pitch_set) + "\n");

		double tempTho = 0.588 / cos(pitch_set) / cos(roll_set);
		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, tempTho, 0.5);
		msleep(400);
		controller_thread.hover();
		msleep(1000);
	}
}

double ProjectOne::constrainNum(double x, double down, double up)
{
	up = abs(up);
	down = abs(down);
	if (x > 0) {
		if (x > up) {
			return up;
		}
		else if (x < down) {
			return down;
		}
	}
	else {
		if (x < -up) {
			return -up;
		}
		else if (x > -down) {
			return -down;
		}
	}
	return x;
}
