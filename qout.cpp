#include "qout.h"

QOUT::QOUT(QString name) :b_stopped(false)
{

}

QOUT::~QOUT()
{
	stop();
	quit();
	wait();
}

void QOUT::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void QOUT::run()
{
	while (true)
	{
		sleep(200);

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

void QOUT::qout(QString value)
{
	QMutexLocker data_locker(&qout_mutex);
	text_output.append(value);
	text_output.append(" ");
}

void QOUT::clear()
{
	QMutexLocker data_locker(&qout_mutex);
	text_output.clear();
}


extern QOUT qout_thread;
void show_string(QString data)
{
	qout_thread.qout(data);
}