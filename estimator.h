#pragma once

#include <QtCore>  
#include <QMutex>  
#include <QApplication>
#include <QThread>
#include <vector>
#include "common/CommonStructs.hpp"
#include <QImage> 

class Estimator:public QThread
{
public:
	explicit Estimator(QString name);
	~Estimator();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;

	
};