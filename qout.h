#pragma once
#include <qstring.h>
#include <QMutex>  
#include <QtCore>  
#include <QApplication>
#include <QThread>

void show_string(QString data);

class QOUT :public QThread
{
public:
	explicit QOUT(QString name);
	~QOUT();

	void run();
	void stop();
	void qout(QString value);
	void clear();

	QString text_output;
	QMutex qout_mutex;

private:
	bool b_stopped;
	QMutex m_mutex;
	
};
