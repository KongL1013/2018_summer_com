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

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp> 
#include <iostream>

#define IMGWIDTH 640
#define IMGHEIGHT 480

using namespace std;
using namespace cv;

/* You may need openCV header files here */

class ImageProcess :public QThread
{
public:
	explicit ImageProcess(QString name);
	~ImageProcess();

	void run();
	void stop();

	void downFindAllRectangle(Mat &input_img, std::vector<RotatedRect> &all_location);
	cv::Rect downFindAllRedCircle(Mat &input_img);
	void downFindRectangle(Mat &input_img, Mat &output_img, Rect &rect);
	void frontFindCircle(Mat &input_img, std::vector<Vec3f> &circles);
	Rect frontFindRectangle(Mat &input_img);
	float pixelToLength(int pixel_num, float distance);


private:

	void undistort_img(Mat &input_img, Mat &output_img);	

	bool b_stopped;

	//���ͼ���λ��
	bool down_sacn_rect;	//����ͼɨ��ͣ��ƺ
	bool down_scan_cycle;	//����ͼɨ��ԲȦ
	bool front_rect;		//������ɨ��Ȧ�����֣������������ڷ���
	bool front_cycle;		//������ɨ��Ȧ�����ģ����ڶ�λ
	bool down_rect;			//������ɨ��ͣ��ƺ�����֣������������ڷ���,ͬʱ���Է��ؾ���(����)

	QMutex m_mutex;
};