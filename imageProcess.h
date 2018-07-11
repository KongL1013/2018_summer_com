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
#include <opencv2/aruco.hpp>


#define IMGWIDTH 640
#define IMGHEIGHT 480

using namespace std;
using namespace cv;
using namespace aruco;

/* You may need openCV header files here */

class ImageProcess :public QThread
{
public:
	explicit ImageProcess(QString name);
	~ImageProcess();

	void run();
	void stop();

	void downFindAllRectangle(Mat &input_img, std::vector<RotatedRect> &all_location);
	bool downFindRedCircle(Mat &input_img, cv::Rect& maxRect);
	bool downFindRectangle(Mat &input_img, Mat &output_img, Rect &rect);
	bool downFindRectangle_stone(Mat &input_img, Mat &output_img, Rect &rect);
	bool frontFindCircle(Mat &input_img, Vec3f &bigCircle);
	bool frontFindRectangle(Mat &input_img, Rect &maxRect);
	bool downFindTree(Mat &input_img, Mat &roi, Rect &rect);
	bool detect_2d_coder(Mat &inputImg, Mat &roi, std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners);
	float pixelToLength(int pixel_num, float distance);


private:

	void undistort_img(Mat &input_img, Mat &output_img);	

	bool b_stopped;

	//检测图像的位置
	bool down_sacn_rect;	//顶视图扫描停机坪
	bool down_scan_cycle;	//顶视图扫描圆圈
	bool front_rect;		//近距离扫描圈的数字，剪切下来用于分类
	bool front_cycle;		//近距离扫描圈的中心，用于定位
	bool down_rect;			//近距离扫描停机坪的数字，剪切下来用于分类,同时可以返回矩形(中心)

	QMutex m_mutex;
};