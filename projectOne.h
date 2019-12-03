#pragma once
#pragma once
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"
#include <functional>
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <QtCore>  
#include <QMutex>  
#include <QApplication>
#include <QThread>
#include <vector>

#include <opencv2\opencv.hpp> 
#include <string>
#include <set>


/* You may need openCV header files here */

class ProjectOne :public QThread
{
public:
	explicit ProjectOne(QString name);
	~ProjectOne();

	void run();
	void stop();

	void MoveToOpenLoop(double spx, double spy);
private:
	void LandOnBoard();
	void AimBoardHigh();
	void AimBoardLow();
	void AimCircleDown();
	void AimCircleFront();
	void QuickDownTo(double height, double thr);
	void QuickUpTo(double height, double thr);
	void StubToQR();
	void AimStubDown();

	void FileInit();

	void saveQRImages(cv::Mat & rgb, int id);

	void getImg(cv::Mat & img, int type);

	//bool isShorterXVec3f_double(const vec3f_double & a, const vec3f_double & b);
	void flyMeToTheMoon();
	void flyMeToTheStar();
	bool b_stopped;
	QMutex m_mutex;
	bool points_close(double x1, double y1, double z1, double x2, double y2, double z2, double limit = 0.2);
	void yawTurnTo(double target, double thr = 0.587);
	void MoveTo(double px, double py, double pz, double yaw, double tollerance);
	void MoveToOneMeterOpenloop(double px,double py, double pz);
	double constrainNum(double x, double up, double down);
	bool stone_area;
	void SmallMove(int distance, bool isxDirection);
	void show_pos();
	void scanMap();
	void gotoVerticleHeight(double goal);
	std::set<int> targetQR;
	QFile resultFile;
};