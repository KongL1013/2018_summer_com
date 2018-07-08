#include "droneInfo.h"
#include <iostream>
#include "infoUpdate.h"
#include <QTime>
#include <QImage> 
#include <qbuffer.h>
#include <qimagereader.h>
#include <qbytearray.h>
#include "qout.h"

#include <vector>

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;

/* IMURetriever*/
IMURetriever::IMURetriever(QString name) :b_stopped(false)
{

}

IMURetriever::~IMURetriever()
{
	stop();
	quit();
	wait();
}

void IMURetriever::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void IMURetriever::run()
{
	msr::airlib::MultirotorRpcLibClient client;
	QTime current_time;

	while (true)
	{
		//msleep(10);

		auto imu = client.getImudata();
		current_time.start();
		long int time_stamp = current_time.msec() + current_time.second() * 1000 + (long int)current_time.minute() * 1000 * 60;

		/* Attitude Update */
		{
			//gcy IMU ACC xyz -> back left up I changed it
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.imu.angular_v.v_x = imu.angular_velocity.x();
			drone_info.imu.angular_v.v_y = imu.angular_velocity.y();
			drone_info.imu.angular_v.v_z = imu.angular_velocity.z();
			drone_info.imu.linear_acc.acc_x = imu.linear_acceleration.x();
			drone_info.imu.linear_acc.acc_y = imu.linear_acceleration.y();
			drone_info.imu.linear_acc.acc_z = imu.linear_acceleration.z();
			drone_info.imu.updated = true;
			drone_info.imu.time_stamp = time_stamp;

			//show_string(QString::number(imu.linear_acceleration.x()) + "\n");
			//show_string(QString::number(imu.linear_acceleration.y()) + "\n");
			//show_string(QString::number(imu.linear_acceleration.z()) + "\n");
		}
		

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}


/* GPSRetriever */
GPSRetriever::GPSRetriever(QString name) :b_stopped(false)
{

}

GPSRetriever::~GPSRetriever()
{
	stop();
	quit();
	wait();
}

void GPSRetriever::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void GPSRetriever::run()
{
	msr::airlib::MultirotorRpcLibClient client;
	QTime current_time;

	while (true)
	{
		//msleep(10);

		auto gps = client.getGpsLocation();
		current_time.start();
		long int time_stamp = current_time.msec() + current_time.second() * 1000 + (long int)current_time.minute() * 1000 * 60;
		
		{
			QMutexLocker data_locker(&drone_info.data_mutex);

			drone_info.global_position.lattitude = gps.latitude;
			drone_info.global_position.longtitude = gps.longitude;
			drone_info.global_position.gps_height = -gps.altitude; //gcy changed
			drone_info.global_position.updated = true;
			drone_info.global_position.time_stamp = time_stamp;
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

/*BaroRetriever*/
BaroRetriever::BaroRetriever(QString name) :b_stopped(false)
{

}

BaroRetriever::~BaroRetriever()
{
	stop();
	quit();
	wait();
}

void BaroRetriever::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void BaroRetriever::run()
{
	msr::airlib::MultirotorRpcLibClient client;
	QTime current_time;

	while (true)
	{
		//msleep(10);

		auto baro = client.getBarometerdata();
		current_time.start();
		long int time_stamp = current_time.msec() + current_time.second() * 1000 + (long int)current_time.minute() * 1000 * 60;

		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.baro.altitude = -baro.altitude;	//gcy changed
			drone_info.baro.pressure = baro.pressure;
			drone_info.baro.time_stamp = time_stamp;
			drone_info.baro.updated = true;
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

/*MagRetriever*/
MagRetriever::MagRetriever(QString name) :b_stopped(false)
{

}

MagRetriever::~MagRetriever()
{
	stop();
	quit();
	wait();
}

void MagRetriever::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void MagRetriever::run()
{
	msr::airlib::MultirotorRpcLibClient client;
	QTime current_time;

	while (true)
	{
		//msleep(10);

		auto mag = client.getMagnetometerdata();
		current_time.start();
		long int time_stamp = current_time.msec() + current_time.second() * 1000 + (long int)current_time.minute() * 1000 * 60;

		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.mag.body_x = mag.magnetic_field_body.x();
			drone_info.mag.body_y = mag.magnetic_field_body.y();
			drone_info.mag.body_z = mag.magnetic_field_body.z();
			drone_info.mag.time_stamp = time_stamp;
			drone_info.mag.updated = true;
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

/*ImgRetriever*/
ImgRetriever::ImgRetriever(QString name) :b_stopped(false)
{

}

ImgRetriever::~ImgRetriever()
{
	stop();
	quit();
	wait();
}

void ImgRetriever::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void ImgRetriever::run()
{
	msr::airlib::MultirotorRpcLibClient client;
	QTime current_time;

	sleep(1);

	while (true)
	{
		//msleep(50);

		/* Image Update */
		/* Note Image Id and type need to be checked */
		//auto image_front_rgb = client.simGetImage(0, ImageCaptureBase::ImageType::Scene);

		std::vector<ImageCaptureBase::ImageRequest> requests;
		ImageCaptureBase::ImageRequest req;

		req.camera_id = 0;
		req.image_type = ImageCaptureBase::ImageType::Scene;
		req.pixels_as_float = false;
		requests.push_back(req);

		req.camera_id = 3;
		req.image_type = ImageCaptureBase::ImageType::Scene;                                               
		req.pixels_as_float = false;
		requests.push_back(req);

		std::vector<ImageCaptureBase::ImageRequest> depth_request;
		ImageCaptureBase::ImageRequest req2;

		req2.camera_id = 0;
		req2.image_type = ImageCaptureBase::ImageType::DepthPerspective;
		req2.pixels_as_float = true;
		depth_request.push_back(req2);

		auto depth_image = client.simGetImages(depth_request);

		auto images = client.simGetImages(requests);
		// ImageCaptureBase::ImageResponse
		current_time.start();
		long int time_stamp1 = current_time.msec() + current_time.second() * 1000 + (long int)current_time.minute() * 1000 * 60;

		/* Convert data */
		QImage q_image_front_rgb;
		vector_to_qimage(images[0].image_data_uint8, q_image_front_rgb);
		QImage q_image_down_rgb;
		vector_to_qimage(images[1].image_data_uint8, q_image_down_rgb);

		QImage q_image_front_dep;
		float_vector_to_qimage(depth_image[0].image_data_float, q_image_front_dep);

		cv::Mat mat_front_rgb, mat_front_dep, mat_down_rgb;
		qimage_to_mat(q_image_front_rgb, mat_front_rgb);
		qimage_to_mat(q_image_front_dep, mat_front_dep);
		qimage_to_mat(q_image_down_rgb, mat_down_rgb);
		
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			
			mat_front_rgb.copyTo(drone_info.images.mat_front_rgb);
			mat_front_dep.copyTo(drone_info.images.mat_front_depth);
			mat_down_rgb.copyTo(drone_info.images.mat_down_rgb);
			
			drone_info.images.front_rgb = q_image_front_rgb;
			drone_info.images.front_depth = q_image_front_dep;
			drone_info.images.down_rgb = q_image_down_rgb;
			drone_info.images.time_stamp_front_rgb = time_stamp1;
			//drone_info.images.time_stamp_front_depth = time_stamp2;
			//drone_info.images.time_stamp_down_rgb = time_stamp3;
			drone_info.images.updated = true;
		
			drone_info.ground_truth.px = images[0].camera_position(0);
			drone_info.ground_truth.py = images[0].camera_position(1);
			drone_info.ground_truth.pz = images[0].camera_position(2);
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

void ImgRetriever::vector_to_qimage(msr::airlib::vector<uint8_t> &img_vec, QImage &img)
{
	QByteArray data = QByteArray::fromRawData(reinterpret_cast<const char*>(img_vec.data()), img_vec.size());
	QBuffer buffer(&data);
	QImageReader reader(&buffer);
	img = reader.read();
}

void ImgRetriever::float_vector_to_qimage(msr::airlib::vector<float> &img_vec, QImage &img)
{
	// Store depth from 0 - 5.0m.
	// Resolution: 0.02m

	unsigned char * c_data = new unsigned char[img_vec.size() * 4];
	for (int i = 0, j = 0; i < img_vec.size(); ++i, j += 4)
	{
		unsigned char temp_char;
		if (img_vec[i] < 0.f)
			temp_char = 0;
		else if (img_vec[i] >= 5.f)
			temp_char = 255;
		else
		{
			temp_char = (unsigned char)((10.f - img_vec[i]) / 5.f * 255.f);
		}
		
		c_data[j] = c_data[j + 1] = c_data[j + 2] = temp_char;
		c_data[j + 4] = ~0;
	}
	//QImage image(c_data, 64, 48, QImage::Format_ARGB32_Premultiplied);
	QImage image(c_data, 64, 48, QImage::Format_ARGB32);
	img = image;
}


void ImgRetriever::qimage_to_mat(QImage &image, cv::Mat &mat)
{
	qDebug() << image.format();
	switch (image.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
		cv::cvtColor(mat, mat, CV_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
		break;
	}
}

