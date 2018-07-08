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
	bool b_stopped;
	QMutex m_mutex;
	bool points_close(double x1, double y1, double z1, double x2, double y2, double z2, double limit = 0.2);
};