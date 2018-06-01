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

class IMURetriever :public QThread
{
public:
	explicit IMURetriever(QString name);
	~IMURetriever();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;
};


class GPSRetriever :public QThread
{
public:
	explicit GPSRetriever(QString name);
	~GPSRetriever();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;
};

class BaroRetriever :public QThread
{
public:
	explicit BaroRetriever(QString name);
	~BaroRetriever();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;
};

class MagRetriever :public QThread
{
public:
	explicit MagRetriever(QString name);
	~MagRetriever();

	void run();
	void stop();


private:
	bool b_stopped;
	QMutex m_mutex;
};

class ImgRetriever :public QThread
{
public:
	explicit ImgRetriever(QString name);
	~ImgRetriever();

	void run();
	void stop();

	void vector_to_qimage(msr::airlib::vector<uint8_t> &img_vec, QImage &img);

private:
	bool b_stopped;
	QMutex m_mutex;
};
