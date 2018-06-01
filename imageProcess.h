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

class ImageProcess :public QThread
{
public:
	explicit ImageProcess(QString name);
	~ImageProcess();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;
};