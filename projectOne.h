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


/* You may need openCV header files here */

class ProjectOne :public QThread
{
public:
	explicit ProjectOne(QString name);
	~ProjectOne();

	void run();
	void stop();


private:
	void LandOnBoard();
	void AimBoardHigh();
	void AimBoardLow();
	void AimCircleDown();
	void AimCircleFront();
	void QuickDownTo(double height, double thr);
	void QuickUpTo(double height, double thr);
	void StubToQR();
	void SaveImg(QString path, QString name, QImage & image);
	void AimStubDown();

	bool b_stopped;
	QMutex m_mutex;
	bool points_close(double x1, double y1, double z1, double x2, double y2, double z2, double limit = 0.2);
	void yawTurnTo(double target, double thr = 0.587);
	void MoveTo(double px, double py, double pz, double yaw, double tollerance);
	double constrainNum(double x, double up, double down);
	bool stone_area;
};