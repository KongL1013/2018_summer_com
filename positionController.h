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

class Controller :public QThread
{
public:
	explicit Controller(QString name);
	~Controller();

	void run();
	void stop();

private:
	bool b_stopped;
	QMutex m_mutex;
};