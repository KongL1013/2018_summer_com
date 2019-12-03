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
#include <QImage> 
#include <qimagereader.h>
#include <algorithm>
#include <QFile>

#include "common/common_utils/FileSystem.hpp"

#define IMGWIDTH 640
#define IMGHEIGHT 480

#define STARTNUM 0
#define MANUAL_CONTROL false
#define SMALLTEST false

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;

Controller controller_thread("controller_thread");
ImageProcess  image_process_thread("image_process_thread");

ProjectOne::ProjectOne(QString name) :
	b_stopped(false),
	stone_area(false),
	resultFile("D:\\result.txt")
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

static int countScan_x, countScan_y;
void ProjectOne::scanMap()
{
	
	for (int i = 0; i < 15; ++i)
	{
		if (i == 0)
		{
			MoveToOpenLoop(17, -10);
		}
		else {
			if (i % 2 != 0) {
				MoveToOpenLoop(-34, -10);
			}
			else{
				MoveToOpenLoop(34, -10);
			}
		}
	}
}

void ProjectOne::gotoVerticleHeight(double goal)
{
	double pz;
	while (true)
	{
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			pz = drone_info.local_position.position.z;
		}
		if (pz < goal) {
			if (goal - pz < 0.8) {
				QuickDownTo(goal, 0.525);

			}
			else {
				QuickDownTo(goal, 0.4);
			}
			break;
		}
		else if (pz >= goal)
		{
			if (pz - goal < 0.8) {
				QuickUpTo(goal, 0.65);
			}
			else {
				QuickUpTo(goal, 0.8);
			}
			break;
		}
		else {
			break;
		}
		
	}
}

void ProjectOne::run()
{

	sleep(2); //连接后姿态估计线程会标定当前零漂

	FileInit();

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
	double spx, spy, spz, spx1, spx2, spy1, spy2, spz1, spz2, spx3, spy3, spz3;
	double svx, svy, svz;
	
	std::vector<vec3f_double> boardPosMap[11];

	boardPosMap[1].push_back({-21.5,	-8,	 -3.2 });
	boardPosMap[1].push_back({0.5,		-10.0,	-3.2 });
	boardPosMap[1].push_back({18.0,		-5.2,	-3.2 });

	boardPosMap[2].push_back({ -15.5,	-17,	-3.2 });
	boardPosMap[2].push_back({ -2,		-19,	-3.2 });
	boardPosMap[2].push_back({10.5,		-16.0,	-3.2 });

	boardPosMap[3].push_back({ -20.5,	-25.5,	-3.2 });
	boardPosMap[3].push_back({ -6.0,	-28.0,	-3.2 });
	boardPosMap[3].push_back({ 13.0,	-25.0,	-3.2 });

	//boardPosMap[4].push_back({ -15.0,	-35.5,	-6.0 });
	boardPosMap[4].push_back({ -18.0,	-35.5,	-6.0 });	//35.5
	boardPosMap[4].push_back({ 0.0,		-35.0,	-6.0 });	//35.5
	//boardPosMap[4].push_back({ 17.0,	-32.0,	-6.0 });
	boardPosMap[4].push_back({ 14.0,	-32.0,	-6.0 });

	boardPosMap[5].push_back({ -16.5,	-44.0,	-6.0 });	//-44
	boardPosMap[5].push_back({ -2.0,	-44.0,	-6.0 });	//--44
	boardPosMap[5].push_back({ 12.0,	-44.0,	-6.0 });

	boardPosMap[6].push_back({ -20.0,	-56.0,	-6.0 });
	boardPosMap[6].push_back({ -2.0,	-56.0,	-6.0 });
	boardPosMap[6].push_back({ 16.0,	-54.0,	-6.0 });

	// boardPosMap[7].push_back({ -14.0,	-67.25,	-6.0 }); // chg removed
	// boardPosMap[7].push_back({ 0.0,		-68.0,	-6.0 });
	// boardPosMap[7].push_back({ 18.0,	-68.0,	-6.0 });
	boardPosMap[7].push_back({ -14.0,	-67.0,	-6.0 });
	boardPosMap[7].push_back({ 2.0,		-67.0,	-6.0 });
	boardPosMap[7].push_back({ 18.0,	-65.0,	-6.0 });

	boardPosMap[8].push_back({ -20,	-77.0,	-6.0 });
	boardPosMap[8].push_back({ -6.5,	-77.0,	-6.0 });
	boardPosMap[8].push_back({ 12.5,	-78.0,	-6.0 });

	boardPosMap[9].push_back({ -20,	-92.5,	-6.0 });
	boardPosMap[9].push_back({ -2.0,	-87.0,	-6.0 });	//-89
	boardPosMap[9].push_back({ 13.0,	-92.5,	-6.0 });

	/*boardPosMap[10].push_back({ -14.0,	-97.0,	-6.0 });
	boardPosMap[10].push_back({ 4.0,	-97.0,	-6.0 });
	boardPosMap[10].push_back({ 17.0,	-97.0,	-6.0 });*/

	boardPosMap[10].push_back({ -12.0,	-104.0,	-6.0 });	//all + 1
	boardPosMap[10].push_back({ 6.0,	-102.0,	-6.0 });
	boardPosMap[10].push_back({ 22.0,	-105.0,	-6.0 });


	std::map<int, int> numQRmap;
	numQRmap[106] = 14;
	numQRmap[219] = 10;
	numQRmap[391] = 11;
	numQRmap[83] = 8;
	numQRmap[999] = 7;
	numQRmap[579] = 3;
	numQRmap[615] = 4;
	numQRmap[759] = 1;
	numQRmap[482] = 0;
	numQRmap[845] = 9;

	vec3f_double woodPositions[16];
	float gcy_x[16] = { 141,137,132,92,98,110,74,60,48,50,9,15,20,-11,0 };
	float gcy_y[16] = { 19,11.4,-1.5,-1.8,9.6,19.2,24,16.2,24,-0.6,24.2,18,6.6,12,0 };
	woodPositions[0] = { -23,	 -132,	-4.0 };
	woodPositions[1] = { -22,	 -126,	-4.0 };
	woodPositions[2] = { -21,	 -115,	-4.0 };
	woodPositions[3] = { -10,	 -111,	-4.0 };
	woodPositions[4] = { -10.5,	 -126,	-4.0 };
	woodPositions[5] = { -13.5,  -134,	-4.0 };
	woodPositions[6] = { -5,	 -138,	-4.0 };
	woodPositions[7] = { 0,		 -129,	-4.0 };
	woodPositions[8] = { 4.5,	 -138,	-4.0 };
	woodPositions[9] = { 2.5,	 -115,	-4.0 };
	woodPositions[10] = { 14,	 -140,	-4.0 };
	woodPositions[11] = { 12,	 -131,	-4.0 };
	woodPositions[12] = { 11,	 -121,	-4.0 };
	woodPositions[13] = { 19,	 -127,	-4.0 };
	woodPositions[14] = { 16,	 -115,	-4.0 };
	woodPositions[15] = { 9,	 -12,	-4.0 };

	for (int i = 0; i < 15; i++) {
		woodPositions[i].x = gcy_x[i] * -5.0 / 18.0;
		woodPositions[i].y = gcy_y[i] * -23.0 / 24;
		show_string(QString::number(woodPositions[i].x) + "\t" + QString::number(woodPositions[i].y) + "\n");
	}

	std::set<int> woods2go;
	bool QRNoDefine = false;
	for (const auto QRnumber : targetQR) {
		auto pos = numQRmap.find(QRnumber);
		if (pos == numQRmap.end()) {
			QRNoDefine = true;
			show_string("not on the map!");
			break;
		}
		woods2go.insert((*pos).second);
	}

	if (QRNoDefine) {
		woods2go.clear();
		for (int i = 0; i < 15; i++) {
			woods2go.insert(i);
		}
	}

	/** start */
	while (true)
	{
		controller_thread.takeoff();
		msleep(1500);
		//gotoVerticleHeight(-4.5);
		controller_thread.hover();
		msleep(1000);	//gcy 2000
		if (SMALLTEST) {
			flyMeToTheStar();
			controller_thread.hover();
			msleep(200000);
			cv::Mat im;
			getImg(im, 1);
			imshow("hahaha", im);
			waitKey(0);
			MoveToOpenLoop(0, -2);
			controller_thread.hover();
			msleep(200000);
		}
		

		double cur_xabs, cur_yabs;

		/* Number 1 -> 3*/
		//if (STARTNUM == 0) {

		//	show_string("start moving to #1!\n");
		//	
		//	for (int j = 1; j < 2; j++)
		//	{
		//		switch (j)
		//		{
		//		case 1:
		//			//spx = 9.5;	spy = -5.2;	spz = -3.2;
		//			spx = 18;	spy = -5.2;	spz = -3.2;
		//			break;
		//		}

		//		while (true) {
		//			MoveToOpenLoop(spx, spy);

		//			gotoVerticleHeight(-3);

		//			cv::Mat down_mat;
		//			getImg(down_mat, 1);
		//			cv::Mat result_down;
		//			cv::Rect num_rect_down;
		//			bool found = image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);
		//			if (found) {
		//				cur_xabs = spx;
		//				cur_yabs = spy;
		//				break;
		//			}
		//		}
		//		LandOnBoard();

		//		controller_thread.takeoff();
		//		msleep(1200);
		//		//gotoVerticleHeight(-4.5);

		//		controller_thread.hover();
		//		msleep(800);

		//	}
		//}
		/* Correct Yaw to headward */
		
		/* TODO: Number 2 -> 10*/
		int targetNum = 1;
		if (STARTNUM != 0) {
			targetNum = STARTNUM;
			show_string("[DEBUG] fly to" + QString::number(STARTNUM) + "directly\n");
		}
		int searchRes = 1;
		bool isCircle = false;
		int searchMode = 1; //1 or 2 or 3

		for (; targetNum < 11; targetNum++)
		{
			if (STARTNUM == 10) {
				flyMeToTheMoon();
				cur_xabs = 0;
				cur_yabs = boardPosMap[10][1].y;
			}
			searchMode = 1;
			show_string(QString("--------------- new target begins! ----------------\n"));
			show_string(QString("TargetNum: ")	+ "\t" + QString::number(targetNum) + "\n"
					  + QString("curPos_x: ")	+ "\t" + QString::number(cur_xabs)  + "\n"
				      + QString("curPos_y: ")	+ "\t" + QString::number(cur_yabs)  + "\n");

			//yawTurnTo(headward);
			
			double goalxabs1, goalxabs2, goalxabs3;
			double goalyabs1, goalyabs2, goalyabs3;

			std::vector<double> x_pos;
			int indexs[3];
			for (int i = 0; i < 3; ++i) {
				indexs[i] = -1;
			}
			for (int j = 0; j < boardPosMap[targetNum].size();++j)
			{
				x_pos.push_back(boardPosMap[targetNum][j].x);
			}
			sort(x_pos.begin(), x_pos.end());
			//if (targetNum == 2) {
			//	for (int j = 0; j < boardPosMap[targetNum].size(); ++j)
			//	{
			//		if (boardPosMap[targetNum][j].x == x_pos[1])
			//		{
			//			goalxabs1 = boardPosMap[targetNum][j].x;
			//			goalyabs1 = boardPosMap[targetNum][j].y;
			//			indexs[0] = j;
			//		}
			//		else if (boardPosMap[targetNum][j].x == x_pos[0])
			//		{
			//			goalxabs2 = boardPosMap[targetNum][j].x;
			//			goalyabs2 = boardPosMap[targetNum][j].y;
			//			indexs[1] = j;
			//		}
			//	}
			//	indexs[2] = -1;
			//}
			//else if (targetNum == 3) {
			//	show_string(QString("-------------minimun_x:") + QString::number(x_pos[0]) + "\t" + \
			//				QString("-------------maximum_x:") + QString::number(x_pos[1]));

			//	for (int j = 0; j < boardPosMap[targetNum].size(); ++j)
			//	{
			//		if (boardPosMap[targetNum][j].x == x_pos[0])
			//		{
			//			goalxabs1 = boardPosMap[targetNum][j].x;
			//			goalyabs1 = boardPosMap[targetNum][j].y;	//gcyggg
			//			indexs[0] = j;
			//		}
			//		else if (boardPosMap[targetNum][j].x == x_pos[1])
			//		{
			//			goalxabs2 = boardPosMap[targetNum][j].x;
			//			goalyabs2 = boardPosMap[targetNum][j].y;
			//			indexs[1] = j;
			//		}
			//	}
			//	indexs[2] = -1;
			//}
			
			float disx1 = cur_xabs - boardPosMap[targetNum].front().x;
			float disx2 = boardPosMap[targetNum].back().x - cur_xabs;
			float delta = disx1 - disx2;
			show_string("\nf:" + QString::number(boardPosMap[targetNum].front().x));
			show_string("\nb:" + QString::number(boardPosMap[targetNum].back().x));
			show_string("\ndisx1:" + QString::number(disx1) + "\t" + \
						"disx2" + QString::number(disx2) + "\n");
			show_string("delta:" + QString::number(delta) + "\n");
			if(delta>0)
			{
				show_string(QString("-------------minimun_x:   ") + QString::number(x_pos[0]) + "\t" + \
							QString("-------------maximum_x:   ") + QString::number(x_pos[2]));
				//double x_max = x_pos[x_pos.size()-1];
				for (int j = 0; j < boardPosMap[targetNum].size(); ++j)
				{
					if (boardPosMap[targetNum][j].x == x_pos[2])
					{
						goalxabs1 = boardPosMap[targetNum][j].x;
						goalyabs1 = boardPosMap[targetNum][j].y;
						indexs[0] = j;
					}
					else if (boardPosMap[targetNum][j].x == x_pos[1])
					{
						goalxabs2 = boardPosMap[targetNum][j].x;
						goalyabs2 = boardPosMap[targetNum][j].y;
						indexs[1] = j;
					}
					else if (boardPosMap[targetNum][j].x == x_pos[0])
					{
						goalxabs3 = boardPosMap[targetNum][j].x;
						goalyabs3 = boardPosMap[targetNum][j].y;
						indexs[2] = j;
					}

				}
			}
			else if(delta <=0)
			{
				for (int j = 0; j < boardPosMap[targetNum].size(); ++j)
				{
					if (boardPosMap[targetNum][j].x == x_pos[0])
					{
						goalxabs1 = boardPosMap[targetNum][j].x;
						goalyabs1 = boardPosMap[targetNum][j].y;
						indexs[0] = j;
					}
					else if (boardPosMap[targetNum][j].x == x_pos[1])
					{
						goalxabs2 = boardPosMap[targetNum][j].x;
						goalyabs2 = boardPosMap[targetNum][j].y;
						indexs[1] = j;
					}
					else if (boardPosMap[targetNum][j].x == x_pos[2])
					{
						goalxabs3 = boardPosMap[targetNum][j].x;
						goalyabs3 = boardPosMap[targetNum][j].y;
						indexs[2] = j;
					}
				}
			}

			float xTarget;
			float yTarget;
			float search_offSet = 0.0;

			while (true)
			{
				if (targetNum > 3)
				{
					gotoVerticleHeight(-4.50 - search_offSet);
				}

				if (searchMode == 1) {
					xTarget = goalxabs1;
					yTarget = goalyabs1;
				} else if (searchMode == 2) {
					xTarget = goalxabs2;
					yTarget = goalyabs2;
				} else if (searchMode == 3) {
					xTarget = goalxabs3;
					yTarget = goalyabs3;
				}

				MoveToOpenLoop(xTarget - cur_xabs, yTarget - cur_yabs);
				show_string("target:" + QString::number(xTarget) + "\t" + QString::number(yTarget) + 
							"\ncurrent:" + QString::number(cur_xabs) + "\t" + QString::number(cur_yabs));
				if (STARTNUM != 0) {
					gotoVerticleHeight(-6);
				}

				if (targetNum > 3)
				{
					gotoVerticleHeight(-6 - search_offSet);
				}

				controller_thread.hover();
				msleep(500);

				cv::Mat down_mat;
				getImg(down_mat, 1);
				cv::Mat result_down;
				cv::Rect circle_rect;
				cv::Rect board_rect;

				bool found_circle = image_process_thread.downFindRedCircle(down_mat, circle_rect);

				if (found_circle && targetNum > 3) 
				{
					searchRes = searchMode;
					cur_xabs = boardPosMap[targetNum][indexs[searchMode - 1]].x;
					cur_yabs = boardPosMap[targetNum][indexs[searchMode - 1]].y - 1.1;
					show_string("found circle!");

					/* Go to the back of the circle by image*/
					AimCircleDown();

					msleep(500);
					QuickDownTo(-3.25, 0.5);
					controller_thread.hover();
					msleep(500);

					AimCircleFront();

					controller_thread.hover();
					msleep(500);
					controller_thread.giveAttSp(-0.2f, 0.0f, 0.f, 0.6f, 1);
					msleep(1700);
					controller_thread.hover();
					msleep(1000);
					break;
				}
				else {
					if (targetNum <= 3) {
						gotoVerticleHeight(-4.2 - search_offSet);
					}
					else {
						// gotoVerticleHeight(-3.5 - search_offSet); //chg
						gotoVerticleHeight(-4.0 - search_offSet);
					}
					bool found_board;
					getImg(down_mat, 1);
					found_board = image_process_thread.downFindRectangle(down_mat, result_down, board_rect);

					if (found_board)
					{
						cur_xabs = boardPosMap[targetNum][indexs[searchMode - 1]].x;
						cur_yabs = boardPosMap[targetNum][indexs[searchMode - 1]].y;

						searchRes = searchMode;
						cv::imshow("down_mat", down_mat);
						cv::imshow("cut_board", result_down);
						cv::waitKey(1);
						LandOnBoard();
						controller_thread.takeoff();
						msleep(1800);
						//gotoVerticleHeight(-4.5);
						controller_thread.hover();
						msleep(1000);
						break;
					}
				}

				cur_xabs = xTarget;
				cur_yabs = yTarget;

				show_string("found nothing!\n");

				if (searchMode == 1) {
					searchMode = 2;
				}
				else if (searchMode == 2) {
					show_string("misfound in 2 places!\n");
					searchMode = 3;
				}
				else if(searchMode == 3)
				{
					show_string("misfound in 3 places! go to place 1\n");
					searchMode = 1;
					search_offSet += 0.5;
				}
			}
		}

		/* QR codes init */
		show_string("going to QR areas...\n");

		bool first2QR = true;

		gotoVerticleHeight(-7);
		float cur2x = boardPosMap[10][1].x - cur_xabs;
		float cur2y = boardPosMap[10][1].y - cur_yabs;
		MoveToOpenLoop(woodPositions[15].x + cur2x, woodPositions[15].y + cur2y);
		cur_xabs = woodPositions[14].x;
		cur_yabs = woodPositions[14].y;
		QuickUpTo(-10, 0.61);
		msleep(500);
		QuickDownTo(-8.25, 0.55);
		controller_thread.hover();
		msleep(500);

		while (true) {
			cv::Mat down_mat;
			getImg(down_mat, 1);
			cv::Mat result_down;
			cv::Rect num_rect_down;

			bool foundTree;
			foundTree = image_process_thread.downFindTree(down_mat, result_down, num_rect_down);
			if (foundTree) {
				break;
			}
			controller_thread.giveAttSp(0, 0, 0, 0.65, 0.5);
			msleep(500);
			controller_thread.hover();
			msleep(400);
		}
		AimStubDown();

		yawTurnTo(backward);
		
		for (auto woodTargetIt = woods2go.rbegin(); woodTargetIt != woods2go.rend(); woodTargetIt++) {
			int woodTarget = * woodTargetIt;
			show_string("next tree target: " + QString::number(woodTarget) + "\n" 
						+ "curent:\t" + QString::number(cur_xabs) + " :\t" + QString::number(cur_yabs) + "\n"
						+ "target:\t" + QString::number(woodPositions[woodTarget].x) 
						+ " :\t" + QString::number(woodPositions[woodTarget].y) + "\n");
			
			MoveToOpenLoop(cur_xabs - woodPositions[woodTarget].x , cur_yabs - woodPositions[woodTarget].y);
			cur_xabs = woodPositions[woodTarget].x;
			cur_yabs = woodPositions[woodTarget].y - 2.75;
			QuickDownTo(-9.1, 0.55);
			while (true) {
				cv::Mat down_mat;
				getImg(down_mat, 1);
				cv::Mat result_down;
				cv::Rect num_rect_down;

				bool foundTree;
				foundTree = image_process_thread.downFindTree(down_mat, result_down, num_rect_down);
				if (foundTree) {
					break;
				}
				controller_thread.giveAttSp(0, 0, 0, 0.65, 0.5);
				msleep(500);
				controller_thread.hover();
				msleep(400);
			}
			AimStubDown();
			StubToQR();
			QuickUpTo(-7, 0.65);
			msleep(500);
		}

		MoveToOpenLoop(cur_xabs - woodPositions[9].x, cur_yabs - woodPositions[9].y);
		AimStubDown();
		MoveToOpenLoop(2, 0);
		QuickUpTo(-10, 0.6);
		flyMeToTheStar();
		QuickDownTo(-5.5, 0.55);
		LandOnBoard();

		show_string("mission complete");

		while (1);

		
		

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
		double pz;
		cv::Mat down_mat;
		getImg(down_mat, 1);
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			pz = drone_info.local_position.position.z;
		}

		if (pz > -1) {
			show_string("drop to 1m, too low! force to land\n");
			break;
		}
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindRectangle(down_mat, result_down, num_rect_down);

		cv::imshow("down_mat", down_mat);
		cv::waitKey(1);

		cv::imshow("cut_board", result_down);
		cv::waitKey(1);

		if (num_rect_down.area() < 32000) {
			controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.42f, 0.5);	//gcy 0.45
			msleep(500);	//gcy 500
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
	msleep(2000); //gcy 2000
}

void ProjectOne::AimBoardHigh() {

	show_string("start aiming at high level\n");

	while (true) {
		cv::Mat down_mat;
		//{
		//	QMutexLocker data_locker(&drone_info.data_mutex);
		//	drone_info.images.mat_down_rgb.copyTo(down_mat);
		//}
		getImg(down_mat, 1);
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
		//{
		//	QMutexLocker data_locker(&drone_info.data_mutex);
		//	drone_info.images.mat_down_rgb.copyTo(down_mat);
		//}
		getImg(down_mat, 1);
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

		float pitch_set = 0.f, roll_set = 0.f;
		if (abs(delt_x) <= tollerance && abs(delt_y) <= tollerance)
		{
			return;
		}
		if (abs(delt_x) > tollerance)
		{
			roll_set = 0.0015 * delt_x;
			roll_set = constrainNum(roll_set, 0.035, 0.15);
		}
		if (abs(delt_y) > tollerance)
		{
			pitch_set = 0.0015 * delt_y;
			pitch_set = constrainNum(pitch_set, 0.035, 0.15);
		}

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
		//{
		//	QMutexLocker data_locker(&drone_info.data_mutex);
		//	drone_info.images.mat_down_rgb.copyTo(down_mat);
		//}
		getImg(down_mat, 1);
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindRedCircle(down_mat, num_rect_down);

		if (num_rect_down.br().x - num_rect_down.tl().x > 2 || num_rect_down.br().y - num_rect_down.tl().y > 2) // ??? Why don't work
		{
			int center_x = (num_rect_down.tl().x + num_rect_down.br().x) / 2;
			int center_y = (num_rect_down.tl().y + num_rect_down.br().y) / 2;

			int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
			int delt_y = center_y - (IMGHEIGHT * 0.15); //IMG coordinate

			float pitch_set = 0.f, roll_set = 0.f;
			if (abs(delt_x) < 40 && abs(delt_y) < 40) {
				controller_thread.hover();
				msleep(500);
				break;
			}
			if (abs(delt_x) > 40) roll_set = 0.1 * delt_x / abs(delt_x);
			if (abs(delt_y) > 40) pitch_set = 0.1 * delt_y / abs(delt_y);


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
		cv::Mat front_mat;
		getImg(front_mat, 0);
		Vec3f circle;
		Rect cirRect;
		image_process_thread.frontFindRectangle(front_mat, cirRect);

		cv::imshow("front_mat", front_mat);
		cv::waitKey(10);

		int center_x = cirRect.x + cirRect.width / 2;
		int center_y = cirRect.y + cirRect.height / 2; //中心坐标

		//show_string(QString::number(cirRect.area()) + "\n");
		int circSize = cirRect.area();

		int delt_x = center_x - IMGWIDTH / 2; //IMG coordinate
		int delt_y = center_y - (IMGHEIGHT / 2); //IMG coordinate

		float throttle = 0.588f, roll_set = 0.f, pitch_set = 0.f;
		if (abs(delt_x) < 45 && abs(delt_y) < 45 && abs(circSize) > 80000 && abs(circSize) < 180000) {
			break;
		}
		/* Control throttle and roll seperately */
		if (abs(delt_x) > 23) { // 20
			roll_set = 0.05 * delt_x / abs(delt_x);
			show_string("adjust roll\t" + QString::number(delt_x) + "\t");
		}
		if (abs(delt_y) > 23) { // 17
			float adjustV = -5e-3 * delt_y;
			adjustV = constrainNum(adjustV, 0.08, 0.2);
			throttle += adjustV;
			show_string("adjust height\t" + QString::number(delt_y) + "\t");
		}
		if (abs(delt_x) < 120 && abs(delt_y) < 100) {
			if (abs(circSize) < 100000) {
				show_string("too far!\t");
				pitch_set = -0.05;
			}
		}
		if (abs(circSize) > 140000) {
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

		//if (pz - height < 1) {
		//	controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.525, 5.0);
		//}
		msleep(100);
	}

	controller_thread.hover();
	msleep(800);
}

void ProjectOne::QuickUpTo(double height, double thr) {
	controller_thread.giveAttSp(0.f, 0.f, 0.f, thr, 5.0);
	show_string("going up to" + QString::number(height) + "...\n");

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

		//if (height - pz  < 1) {
		//	controller_thread.giveAttSp(0.f, 0.f, 0.f, 0.65, 5.0);
		//}
		msleep(100);
	}

	controller_thread.hover();
	msleep(800);
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
	QuickDownTo(-3.9, 0.51);
	//TODO take picture

	cv::Mat front_img_qr;
	cv::Mat qr_roi;

	//{
	//	QMutexLocker data_locker(&drone_info.data_mutex);
	//	drone_info.images.mat_front_rgb.copyTo(front_img_qr);
	//}

	getImg(front_img_qr, 0);

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;

	if (image_process_thread.detect_2d_coder(front_img_qr, qr_roi, ids, corners))
	{
		show_string("ID: " + QString::number(ids[0]) + "Left Top:" + QString::number(corners[0][0].x) + "," +
				QString::number(corners[0][0].y) + "Right Bottom:" + QString::number(corners[0][1].x) + "," +
				QString::number(corners[0][1].y) + "\n");

		if (targetQR.find(ids[0]) != targetQR.end()) {
			targetQR.erase(ids[0]);
			show_string("it's a target!!!\n");
			static QString s(" ");
			QString recordStr(QString::number(ids[0])
					+ s + QString::number(corners[0][0].x)
					+ s + QString::number(corners[0][0].y)
					+ s + QString::number(corners[0][1].x)
					+ s + QString::number(corners[0][1].y));
			QTextStream targetInfoOut(&resultFile);
			targetInfoOut << recordStr << endl;

			saveQRImages(front_img_qr, ids[0]);
		}
	}

	controller_thread.hover();
	msleep(500);
}

void ProjectOne::AimStubDown() {
	show_string("start aiming at stub\n");

	while (true) {
		cv::Mat down_mat;
		//{
		//	QMutexLocker data_locker(&drone_info.data_mutex);
		//	drone_info.images.mat_down_rgb.copyTo(down_mat);
		//}
		getImg(down_mat, 1);
		cv::Mat result_down;
		cv::Rect num_rect_down;

		image_process_thread.downFindTree(down_mat, result_down, num_rect_down);

		cv::imshow("down_mat", down_mat);
		cv::waitKey(1);

		cv::imshow("cut_board", result_down);
		cv::waitKey(1);

		double diagSize = distOfTwoPoint(num_rect_down.tl().x, num_rect_down.tl().y, num_rect_down.br().x, num_rect_down.br().y);
		double tollerance = 20;

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
			roll_set = constrainNum(roll_set, 0.03, 0.1);
		}
		if (abs(delt_y) > tollerance)
		{
			pitch_set = 0.002 * delt_y;
			pitch_set = constrainNum(pitch_set, 0.03, 0.1);
		}

		show_string("correct num: " + QString::number(roll_set) + "\t" + QString::number(pitch_set) + "\n");

		double tempTho = 0.588 / cos(pitch_set) / cos(roll_set);
		controller_thread.giveAttSp(pitch_set, roll_set, 0.f, tempTho, 0.4);
		msleep(400);
		controller_thread.hover();
		msleep(500);
	}
}

void ProjectOne::FileInit()
{
	QFile targetFile("D:/aruco.txt");
	if (!targetFile.open(QIODevice::ReadOnly)) {
		show_string("aruco.txt open FAIL!!\n");
	}
	else {
		show_string("The QR Target:\n");
		while (!targetFile.atEnd()) {
			QByteArray line = targetFile.readLine();
			QString str(line);
			int target = str.toInt();
			targetQR.insert(target);
			show_string("#" + QString::number(targetQR.size()) + ":  " + QString::number(target) + "\n");
		}
		show_string("we have" + QString::number(targetQR.size()) + "targets\n\n");
	}

	if (resultFile.exists()) {
		resultFile.remove();
	}

	msleep(1);

	if (!resultFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		show_string("result.txt open FAIL!!\n");
	}
	else {
		show_string("result.txt open SUCCESS\n");
	}

	QDir dir1("D:\\images\\");
	if (!dir1.exists()) {
		dir1.mkpath("D:\\images\\");
	}
	else {
		dir1.setFilter(QDir::Files);
		int count = dir1.count();
		for (int i = 0; i < count; i++) {
			dir1.remove(dir1[i]);
		}
	}
	QDir dir2("D:\\depth\\");
	if (!dir2.exists()) {
		dir1.mkpath("D:\\depth\\");
	}
	else {
		dir2.setFilter(QDir::Files);
		int count = dir2.count();
		for (int i = 0; i < count; i++) {
			dir2.remove(dir2[i]);
		}
	}
}

//front 0
//down 1
void ProjectOne::getImg(cv::Mat & img, int type) {
	ImageCaptureBase::ImageRequest req;
	std::vector<ImageCaptureBase::ImageRequest> requests;
	switch (type) {
	default:
	case 0:
		req.camera_id = 0;
		req.image_type = ImageCaptureBase::ImageType::Scene;
		req.pixels_as_float = false;
		requests.push_back(req);
		break;
	case 1:
		req.camera_id = 3;
		req.image_type = ImageCaptureBase::ImageType::Scene;
		req.pixels_as_float = false;
		requests.push_back(req);
		break;
	}

	auto images = client.simGetImages(requests);
	auto picData = images[0].image_data_uint8;
	QByteArray dataArray = QByteArray::fromRawData(reinterpret_cast<const char*>(picData.data()), picData.size());
	QBuffer buffer(&dataArray);
	QImageReader reader(&buffer);
	static QImage Qimg;
	Qimg = reader.read();
	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		if (type == 1) {
			drone_info.images.down_rgb = Qimg;
		}
		else if (type == 0) {
			drone_info.images.front_rgb = Qimg;
		}
	}
	img = cv::Mat(Qimg.height(), Qimg.width(), CV_8UC4, (void*)Qimg.constBits(), Qimg.bytesPerLine());
}

void ProjectOne::flyMeToTheMoon()
{
	gotoVerticleHeight(-10);
	double thur = 0.588 / cos(0.3);
	controller_thread.giveAttSp(-0.3, 0, 0, thur, 4.35);
	msleep(4350);
	controller_thread.giveAttSp(-0.2, 0, 0, thur, 2.90);
	msleep(2900);
	controller_thread.giveAttSp(0.3, 0, 0, thur, 2);
	msleep(2000);
	controller_thread.hover();
	msleep(1000);
}

void ProjectOne::flyMeToTheStar()
{
	gotoVerticleHeight(-10);
	double thur = 0.588 / cos(0.3);
	controller_thread.giveAttSp(-0.3, 0, 0, thur, 4.80);
	msleep(4800);
	controller_thread.giveAttSp(-0.2, 0, 0, thur, 3.10);
	msleep(3100);
	controller_thread.giveAttSp(0.3, 0, 0, thur, 2);
	msleep(2000);
	controller_thread.hover();
	msleep(1000);
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

void ProjectOne::saveQRImages(cv::Mat & rgb, int id) {
	ImageCaptureBase::ImageRequest req;
	std::vector<ImageCaptureBase::ImageRequest> request;
	req.camera_id = 0;
	req.image_type = ImageCaptureBase::ImageType::DepthPerspective;
	req.pixels_as_float = true;
	request.push_back(req);
	auto images = client.simGetImages(request);
	Utils::writePfmFile(images[0].image_data_float.data(), images[0].width, images[0].height,
		"D:\\depth\\" + to_string(id) + ".pfm");
	imwrite("D:\\images\\" + to_string(id) + ".png", rgb);
}

void ProjectOne::SmallMove(int distance, bool isxDirection)
{
	double duration, thrust;
	thrust = 0.599;
	if (abs(distance) > 0.02)
	{
		if (abs(distance) == 5)
		{
			thrust = 0.605;
			if (isxDirection) {
				duration = 1.35;
				
			}
			else {
				duration = 1.35;
			}	
		}
		else if (abs(distance) == 2) {
			if (isxDirection) {
				//duration = 1.3;
				duration = 1.05;
			}
			else
				//duration = 1.5;
				duration = 1.05;
		}
		else if (0.02 < abs(distance) && abs(distance) < 2) {
			duration = 1.05 * fabs(distance) / 2.0;
			show_string(QString("small distance " + QString::number(duration)));
		}
		
		double pitch, roll;
		if (isxDirection) {
			if (distance > 0)
			{
				pitch = 0.1;
				roll = 0.;
			}
			else {
				pitch = -0.1;
				roll = 0.;
			}
		}
		else {
			if (distance > 0)
			{
				roll = 0.1;
				pitch = 0.;
			}
			else {
				roll = -0.1;
				pitch = 0.;
			}
		}
		auto starttime = QTime::currentTime();
		//client.moveByAngleThrottle(roll, pitch, 0.599, 0, duration);

		controller_thread.giveAttSp(roll, pitch, 0, thrust, duration);
		//show_string("thur:\t" + QString::number(thrust) + "\n");
		if (thrust == 0.599) {
			msleep(duration * 1000);
		}
		else
		{
			msleep(duration * 1500);
		}
		auto stopTime = QTime::currentTime();
		int elapsed = starttime.msecsTo(stopTime);
		double elapsed_s = elapsed / 1000.;
		controller_thread.hover();
		msleep(1000);//等飞机稳定hover

	}
	else if (abs(distance) < 0.02) {
		controller_thread.hover();
		msleep(500);
		return;
	}
	return;
}

void ProjectOne::show_pos()
{
}

void ProjectOne::MoveToOpenLoop(double spx, double spy)
{
	int sign_x;
	spx > 0 ? sign_x = 1 : sign_x = -1;
	int sign_y;
	spy > 0 ? sign_y = 1 : sign_y = -1;

	show_string("moved by: " + QString::number(spx) + "\t" + QString::number(spy)+ "\n");
	int xnum_fiveMeter = abs(spx) / 5;
	int ynum_fiveMeter = abs(spy) / 5;
	if (spx > 0) {
		spx = spx - xnum_fiveMeter * 5;
	}
	else {
		spx = spx + xnum_fiveMeter * 5;
	}
	if (spy > 0) {
		spy = spy - ynum_fiveMeter * 5;
	}
	else {
		spy = spy + ynum_fiveMeter * 5;
	}
	
	int xnum_twoMeter = abs(spx) / 2;
	int ynum_twoMeter = abs(spy) / 2;
	
	double rest_x, rest_y;
	show_pos();
	if (spx > 0) {
		rest_x = spx - xnum_twoMeter * 2;
	}
	else {
		rest_x = spx + xnum_twoMeter * 2;
	}
	if (spy > 0) {
		rest_y = spy - ynum_twoMeter * 2;
	}
	else {
		rest_y = spy + ynum_twoMeter * 2;
	}

	//x-direction
	if (xnum_fiveMeter != 0)
	{
		for (int i = 0; i < xnum_fiveMeter; ++i) {
			SmallMove(sign_x * 5, true);
			countScan_x++;
			//show_pos();
		}
		countScan_x++;
		//show_pos();
	}
	if (xnum_twoMeter != 0)
	{
		for (int i = 0; i < xnum_twoMeter; ++i) {
			SmallMove(sign_x * 2, true);
			countScan_x++;
			//show_pos();
		}
		
		//show_pos();
	}
	SmallMove(rest_x, true);
	countScan_x++;

	controller_thread.hover();
	msleep(500);

	//y-direction
	if (ynum_fiveMeter != 0)
	{
		for (int i = 0; i < ynum_fiveMeter; ++i) {
			SmallMove(sign_y * 5, false);
			countScan_x++;
			//show_pos();
		}
		countScan_x++;
		//show_pos();
	}
	if (ynum_twoMeter != 0)
	{
		for (int i = 0; i < ynum_twoMeter; ++i) {
			SmallMove(sign_y * 2, false);
			countScan_y++;
			//show_pos();
		}
		
		//show_pos();
	}

	SmallMove(rest_y, false);
	countScan_y++;

	controller_thread.hover();
	msleep(500);
	return;
}

/*for (; targetNum < 11; targetNum++)
{
yawTurnTo(headward);
int searchMode = 1; //1 or 2 or 3

switch (targetNum)
{
case 2:
//spx = 10.5;	spy = -14;	spz = -3.2;
//sp2: (0, -17)
spx1 = -6.5;	spy1 = -9.5;	spz1 = -3.2;
spx2 = -10.5,	spy2 = -3;		spz2 = -3.2;
break;
case 3:
//spx1 = -5;	spy1 = -26.5;	spz1 = -3.5;
//sp2: (13, -24);  //3
spx1 = -15.5;	spy1 = -12.5;	spz1 = -3.2;
spx2 = 28.5;	spy2 = -2.5;
break;
case 4:
//spx1 = -15; spy1 = -36.5; spz1 = -8.0;//abs juesai1
//spx2 = 17;  spy2 = -32);//abs 2
//spx3: (0, -35);//5
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = -10; spy1 = -10.5; spz1 = -3.5;
}
else if (searchRes == 2) {
spx1 = -28; spy1 = -12.5; spz1 = -3.5;
}
spx2 = 32.0; spy2 = 4.5; spz2 = -7.0;

stone_area = false;
break;
case 5:
//spx1 = -16; spy1 = -45.0; spz1 = -7.0;//abs juesai
//sp2: (0, -45);//5
//sp3: (12, -44);//5
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = -1; spy1 = -10.0; spz1 = -7.0;
}
else if (searchRes == 2) {
spx1 = -33; spy1 = -13.0; spz1 = -7.0;
}
spx2 = 16.0;	spy2 = 1.0; spz2 = -3.2;
stone_area = false;
break;
case 6: //circle
//spx1 = -20, spy1 = -56//abs juesai
//sp2: (-2, -56)
//sp3: (16, -54) //6
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = -4; spy1 = -8.0; spz1 = -6.0;
}
else if (searchRes == 2) {
spx1 = -20; spy1 = -11; spz1 = -6.0;
}
spx2 = 18; spy2 = 0; spz2 = -6.0;
stone_area = false;
break;
case 7:
//spx1 = -14, spy1 = -68//abs juesai
//sp2: (0, -68);//7
//(18, -68);//7
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = 6; spy1 = -12; spz1 = -6.0;//add distance to the ring
}
else if (searchRes == 2) {
spx1 = -12; spy1 = -12; spz1 = -6.0;
}
spx2 = 14; spy2 = 0; spz2 = -6.0;
stone_area = false;
break;
case 8:
QuickUpTo(-6.0, 0.8);
//spx1 = -6, spy1 = -79 //abs juesai
//sp2: (-19, -77);//8
//sp3: (12, -77);//8
if (searchRes == 1) {
spx1 = 8.0; spy1 = -8.0; spz1 = -6.0;
}
else if (searchRes == 2) {
spx1 = -6; spy1 = -9; spz1 = -6.0;
}
spx2 = -13; spy2 = 2; spz2 = -6.0;
stone_area = false;
break;
case 9:
//spx1 = -2, spy1 = -92//abs juesai
//sp2: (-19, -94);//9
//sp3:	(13, -94);//9
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = 4; spy1 = -9; spz1 = -7.0;
}
else if (searchRes == 2) {
spx1 = -6; spy1 = -9; spz1 = -6.0;
}

spx2 = -9.5; spy2 = -83.5; spz2 = -7.0;
stone_area = false;
break;
case 10:
//17,-110 abs juesai
//sp2: (4, -107);//10
//sp3(-14, -107);//10
QuickUpTo(-6.0, 0.8);
if (searchRes == 1) {
spx1 = 19; spy1 = -18; spz1 = -6.0;
}
else if (searchRes == 2) {
spx1 = 36; spy1 = -13; spz1 = -6.0;
}

spx2 = -13; spy2 = 3.0; spz2 = -6.0;
stone_area = false;
break;
}
show_string("go to #" + QString::number(targetNum) + "\n");

while (true)
{
if (searchMode == 1) {
MoveToOpenLoop(spx1, spy1);
}
else if (searchMode == 2){
MoveToOpenLoop(spx2, spy2);
//MoveTo(spx2, spy1, spz2, headward, 0.8);
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
//{
//	QMutexLocker data_locker(&drone_info.data_mutex);
//	drone_info.images.mat_down_rgb.copyTo(down_mat);
//}
getImg(down_mat, 1);
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
searchRes = searchMode;
show_string("found circle!");

//yawTurnTo(backward);

// Go to the back of the circle by image
AimCircleDown();

msleep(500);
QuickDownTo(-3.0, 0.5);

AimCircleFront();

controller_thread.hover();
show_string("ready to go!");
controller_thread.giveAttSp(-0.2f, 0.0f, 0.f, 0.6f, 1);
msleep(1700);
controller_thread.hover();
msleep(2000);

show_string("pass complete");
yawTurnTo(headward, 0.6);
controller_thread.hover();
msleep(1000);
break;
}
else if (found_board)
{
searchRes = searchMode;
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
}*/