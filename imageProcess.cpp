#include "imageProcess.h"
#include "droneInfo.h"
#include <iostream>



using namespace std;
using namespace cv;
using namespace msr::airlib;

extern DroneInfo drone_info;
//extern msr::airlib::MultirotorRpcLibClient client;


ImageProcess::ImageProcess(QString name) :b_stopped(false)
{

}

ImageProcess::~ImageProcess()
{
	stop();
	quit();
	wait();
}

void ImageProcess::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void ImageProcess::run()
{
	msr::airlib::MultirotorRpcLibClient client;

	bool weHaveRequest = false;
	DroneInfo::ImageProcess::ImageCommand theCmd;
	DroneInfo::ImageProcess::ImageCommand noCmd;

	while (true)
	{

		/* Add your code here */
		// Create temperary variables to continue data process and then give the value to "drone_info"
		// Don't forget to add lock
		// e.g.
		// {
		//    QMutexLocker data_locker(&drone_info.data_mutex);
		//    drone_info.attitude.anglar_velocity.pitch_rate = pitch_rate;
		// }

		while (!weHaveRequest) {
			//以1ms为间距,查询数据的更新情况
			msleep(1);
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				//查询是否有要求
				theCmd = drone_info.imgProcess.imgCmd;
				//drone_info.imgProcess.imgCmd = noCmd; //是否每次只执行一次?
			}
			weHaveRequest = theCmd.findDownCirc;
			weHaveRequest |= theCmd.findDownRect;
			weHaveRequest |= theCmd.findFrontAxis;
			weHaveRequest |= theCmd.getDownCircs;
			weHaveRequest |= theCmd.getDownRects;
			weHaveRequest |= theCmd.recognizeNum;
		}

		if (theCmd.findDownRect) {
			//TODO downFindAllRectangle()
			std::vector<vec3f_t> poses;
			poses.push_back({0,0,0});
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				drone_info.imgProcess.downRects.updated = true;
				drone_info.imgProcess.downRects.rectPoses = poses;
			}
		}

		if (theCmd.findDownCirc) {

		}

		if (theCmd.findFrontAxis) {

		}

		if (theCmd.getDownCircs) {

		}

		if (theCmd.getDownRects) {

		}

		if (theCmd.recognizeNum) {

		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

//---DownCamera------//


//-1- read image into cache  from AirSim API


//-2- processs distort with warpperspective
void ImageProcess::undistort_img(Mat &input_img, Mat &output_img) {
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 246.33203;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 289.82891;
	cameraMatrix.at<double>(1, 1) = 246.65002;
	cameraMatrix.at<double>(1, 2) = 253.71157;

	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = -0.04114;
	distCoeffs.at<double>(1, 0) = 0.04878;
	distCoeffs.at<double>(2, 0) = -0.02292;
	distCoeffs.at<double>(3, 0) = 0.01071;
	distCoeffs.at<double>(4, 0) = 0;

	//Mat view, rview, map1, map2;
	//Size imageSize;
	//imageSize = input_img.size();
	//cvInitUndistortMap(cameraMatrix, distCoeffs, map1, map2);
	//remap(input_img, output_img, map1, map2, INTER_LINEAR);
	undistort(input_img, output_img, cameraMatrix, distCoeffs);
}

//-3- find rectangle num pitch in down image
//首先飞高定位所有的数字停机坪
//每次调动后需要clear vector 避免冲突   >>c.clear(); 
//此处需要定高巡航来调参数（确定一个高度然后阈值调参），》》后面局部确定一个更小的巡航高度用于定位精确降落
//TODO: 测量像素值对应实际距离
//Output: loading tarmac global locations  (from picture to GPS)   巡航后得到全局停机坪的位置
void ImageProcess::downFindAllRectangle(Mat &input_img, std::vector<RotatedRect> &all_location) {
	namedWindow("down_view");
	imshow("down_view", input_img);
	waitKey();



	//deliate in the region
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
	Mat dilate_img;

	dilate(input_img, dilate_img, element);

	imshow("down_view", dilate_img);
	waitKey();


	//binary process
	Mat output_bin_img;
	threshold(dilate_img, output_bin_img, 200, 255, CV_THRESH_TOZERO);    //BRIGHT REGION
	imshow("down_view", output_bin_img);
	waitKey();


	//find contour and rect it
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;

	findContours(output_bin_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);



	//draw and display， return location
	RotatedRect location;
	Point2f vertex[4];
	location.points(vertex);
	for (int j = 0; j < contours.size(); j++) {
		location = minAreaRect(contours[j]);
		location.points(vertex);
		all_location.push_back(location);

		for (int i = 0; i < 4; i++) {
			line(input_img, vertex[i], vertex[(i + 1) % 4], cv::Scalar(255));
		}
	}
	imshow("down_view", input_img);
	waitKey();
	//TODO: REDLEASE ALL VARIABLES

}

//-4-
//Find the red circle top seview location in top view
//寻找所有的红色圈的位置，同样要确定巡航高度*hight0*，调节阈值
//Output: red circles global locations    (from picture to GPS)  巡航后得到全局红色圈的位置
 bool ImageProcess::downFindRedCircle(Mat &input_img, cv::Rect& maxRect)	 {

	//use hue to find the red region  from   Hue, Saturation, Value  --A. R. Smith1978//
	Mat img_hsv;
	Mat red_out(640, 480, CV_8U);
	cvtColor(input_img, img_hsv, CV_BGR2HSV);    //change color into hsv

	int iLowH = 0;
	int iHighH = 10;

	int iLowS = 42;
	int iHighS = 255;

	int iLowV = 43;
	int iHighV = 255;

	//闭操作 (连接一些连通域)
	inRange(img_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), red_out); //Threshold the image

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(red_out, red_out, element);
	element = getStructuringElement(MORPH_RECT, Size(15, 15));
	dilate(red_out, red_out, element);
	//imshow("Thresholded Image", red_out); //show the thresholded image
	//imshow("Original", input_img); //show the original image
	//waitKey();


	//find max_contours
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;

	// 查找轮廓，对应连通域
	findContours(red_out, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//problems: https://blog.csdn.net/fightingforcv/article/details/78423866

	// 寻找最大连通域
	double maxArea = 0;
	std::vector<cv::Point> maxContour;
	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > maxArea)
		{
			maxArea = area;
			maxContour = contours[i];
		}
	}
	//如果需要返回全部区域，用push vector返回即可


	// 将轮廓转为矩形框
	maxRect = cv::boundingRect(maxContour);
	
	//cout << maxRect << endl;
	// 显示连通域
	cv::Mat result1, result2;

	input_img.copyTo(result1);
	input_img.copyTo(result2);

	for (size_t i = 0; i < contours.size(); i++)
	{
	cv::Rect r = cv::boundingRect(contours[i]);
	cv::rectangle(result1, r, cv::Scalar(255));
	}
	cv::imshow("all regions", result1);
	cv::waitKey(10);

	cv::rectangle(result2, maxRect, cv::Scalar(255));
	cv::imshow("largest region", result2);

	waitKey(10);
	

	bool sign_circle = false;
	if (maxRect.area() >= 200) sign_circle = true;

	return sign_circle;
}


//-5- 
//根据全局坐标，飞到对应停机坪上方固定高度*hight1*处采集图像
//根据图像，抠取相应的图像区域rect，resize后作为SVM/DNN的输入
//Output: number rect region
bool ImageProcess::downFindRectangle(Mat &input_img, Mat &output_img, Rect &rect) {

	// canny 检测 边缘
	bool sign_rect = false;			//check if there is any rect detected
	Mat grey;
	Mat edges;
	int lowThresh = 50; int lowThrestHigh = 100; int apatureszie = 3;

	cvtColor(input_img, grey, COLOR_BGR2GRAY);

	//denoise
	//blur(grey, grey, Size(3, 3));
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	//erode(grey, grey, element);
	morphologyEx(grey, grey, MORPH_OPEN, element);
	blur(grey, grey, Size(2, 2));

	//imshow("down", grey);
	Canny(grey, edges, lowThresh, lowThrestHigh, apatureszie);
	//imshow("edges",edges);
	//waitKey();

	//寻找包络
	std::vector<std::vector<Point>> contours;
	//threshold(edges, edges, 128, 255, THRESH_BINARY);
	findContours(edges, contours , CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	double maxare = 0; int max_i = 0;
	for (size_t i = 0; i < contours.size(); i++) {  //size_t special typedef
		double area = contourArea(contours[i]);
		if (area > maxare) {
			maxare = area;
			max_i = i;
		}
		//cout << contours.size()<<"   "<< max_i << endl;

	}


	//拟合最大矩形
	rect = boundingRect(contours[max_i]);
	
	if (rect.area() >= 5000){
		sign_rect = true;
	}//some threshold
	Mat rect_roi;


	input_img(rect).copyTo(rect_roi);
	//返回矩形片 Mat
	output_img = rect_roi;

	//TODO:delete
	imshow("rect_down", rect_roi);   //check
	waitKey(10);


	return sign_rect;
}

//-6- find center
//If the drone in front of the circle, we could use the houghcircle to detect the cycle and center//if not work use the frontFindRectangle mass center
//根据全局坐标， 飞到对应的圈前面，固定高度*hight2*和前面距离*distance*
//Output: circle center  得到对应圈的中心
void ImageProcess::frontFindCircle(Mat &input_img, std::vector<Vec3f> &circles) {
	Mat grey;
	cvtColor(input_img, grey, COLOR_BGR2GRAY);
	GaussianBlur(grey, grey, Size(9, 9), 2, 2);
	HoughCircles(grey, circles, HOUGH_GRADIENT, 1.5, 10, 200, 50, 100, 200);

	//TODO: Find the biggest one argmax(circles.r)
	/*
	for (size_t i = 0; i < circles.size(); i++)
	{
	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	int radius = cvRound(circles[i][2]);
	// circle center
	circle(input_img, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	// circle outline
	circle(input_img, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}
	imshow("hough", input_img);
	waitKey();
	*/
}




//-7- find rectangle num pitch in font image
//根据全局坐标， 飞到对应的圈前面，固定高度*hight2*和前面距离*distance*
//根据图像，抠取相应的图像区域rect，resize后作为SVM/DNN的输入
//Output: number rect region
Rect ImageProcess::frontFindRectangle(Mat &input_img) {


	//find read region
	int iLowH = 0; int iHighH = 7; int iLowS = 43; int iHighS = 255; int iLowV = 43; int iHighV = 255;


	Mat img_hsv;
	Mat red_out(640, 480, CV_8U);
	cvtColor(input_img, img_hsv, CV_BGR2HSV);    //change color into hsv

	inRange(img_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), red_out); //Threshold the image

																							//imshow("origin", input_img);
																							//imshow("hsv", red_out);



																							//if find circle in the sunny side, hough is enough!
	Mat element1 = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(30, 30));
	erode(red_out, red_out, element1);
	dilate(red_out, red_out, element2);
	imshow("dilate", red_out);

	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	Mat circle_font = input_img.clone();

	// 查找轮廓，对应连通域
	findContours(red_out, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//problems: https://blog.csdn.net/fightingforcv/article/details/78423866

	// 寻找最大连通域
	double maxArea = 0;
	std::vector<cv::Point> maxContour;
	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area > maxArea)
		{
			maxArea = area;
			maxContour = contours[i];
		}
	}

	// 将轮廓转为矩形框
	cv::Rect maxRect = cv::boundingRect(maxContour);
	maxRect.height = maxRect.height * 2;
	cv::rectangle(circle_font, maxRect, cv::Scalar(255));
	//cv::imshow("largest region", circle_font);
	//waitKey();
	//cout << maxRect << endl;
	return maxRect;		//return the corase  X,Y  location of circle
						//根据这个大小推断出下面数字的片元，相同的宽度下高度变成二倍来减出片元，其中包含数字。

}



//由像素转换为实际距离,目前内参为269.5mm,计算出像素单元大小为:
//distance 为距离目标的高度或者深度信息  update/
float ImageProcess::pixelToLength(int pixel_num, float distance) {
	float pixel_scale = 0.001;		//pixel scale in meter  TODO:calibration 单位是m 像素的大小标定结果0.001m -- 1mm
	float focus_lenght = 0.2695;		//fx=fy=269.5mm
	float real_length = (pixel_num*pixel_scale) * fabs(distance) / focus_lenght;  //nx/f=X/z  from camera model

	return real_length;
}