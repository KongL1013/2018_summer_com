
#include  "estimator.h"
#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"
#include <functional>
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "droneInfo.h"
#include <QTime>
#include "qout.h"

#include "attitude_estimator.h"

#define MAG_EST_ALL false //flase: 只估计yaw  true: 修正3个角度
#define FIX_GYRO_BIAS false //是否修正零漂
#define USING_ACC_EST_ALT true //是否使用加速度计做修正

using namespace std;
using namespace msr::airlib;
using namespace stateestimation;
using namespace Eigen;

extern DroneInfo drone_info;

const float w_mag = 0.4f;
const float w_acc = 0.01f;
const float w_mag_full = 0.5f;
const double w_acc_pos_vel = 0.8;
const float w_motor_output = 0.4f;	//电机输出的反馈修正权重

const double w_acc_bias_xy = 0.001;

const double w_z_baro_p = 1.0;
const double w_z_baro_v = 1.0;
const double w_xy_gps_p = 2.0;
const double w_xy_gps_v = 1.0;

const double w_gyro_bias = 0.03;

const double bias_max = 0.05;

bool isUsingAPI = false;

double x_est[2] = { 0.0, 0.0 };
double y_est[2] = { 0.0, 0.0 };
double z_est[2] = { 0.0, 0.0 };
double lastX = 0.0;
double lastY = 0.0;
double lastZ = 0.0;

double corr_baro[2] = { 0.0 , 0.0 };
double corr_gps[3][2] = {
	{ 0.0f, 0.0f },		// N (pos, vel)
	{ 0.0f, 0.0f },		// E (pos, vel)
	{ 0.0f, 0.0f },		// D (pos, vel)
};

const unsigned long preferedClockTime = 100;	//in ms
const double pi = 3.1415926;
const double pi_half = 3.1415926 / 2.0;
const float fifty_dps = 0.873f;

const double deg2rad = 0.01745329;

Estimator::Estimator(QString name) :b_stopped(false)
{

}

Estimator::~Estimator()
{
	stop();
	quit();
	wait();
}

void Estimator::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void print3num(double a, double b, double c) {
	show_string(
		QString::number(a) + "\t" +
		QString::number(b) + "\t" +
		QString::number(c) + "\n");
}

void print4num(double a, double b, double c, double d) {
	show_string(
		QString::number(a) + "\t" +
		QString::number(b) + "\t" +
		QString::number(c) + "\t" +
		QString::number(d) + "\n"
		);
}

void Estimator::run()
{
	vec3f_t corr;
	att_t att_local;

	DroneInfo::Barometer baro;
	DroneInfo::GlobalPosition gps;
	DroneInfo::GlobalPosition gps_init;
	DroneInfo::IMU imu;
	DroneInfo::Magnetic mag;
	DroneInfo::AngluarSetpoint motorOutput;

	msr::airlib::MultirotorRpcLibClient client;


	QTime initializationClock;
	QTime dtTimer;

	bool got_data = false;

	dtTimer.start();

	/**初始化采集1s的数据*/
	initializationClock.start();
	bool blInitilaized = false;
	int initCount = 0;
	double lons = 0.0;
	double lats = 0.0;
	double gpsHeights = 0.0;
	double baroAlts = 0.0;
	double gyroBiases[3] = { 0.0, 0.0, 0.0 };
	double accBiases[3] = { 0.0, 0.0, 0.0 };
	double mags[3] = { 0.0, 0.0, 0.0 };


	while (!blInitilaized) {
		
		while (!got_data) {
			{
				QMutexLocker data_locker(&drone_info.data_mutex);

				got_data = drone_info.imu.updated;
				got_data &= drone_info.mag.updated;
				got_data &= drone_info.global_position.updated;
				got_data &= drone_info.baro.updated;
				//只有所有数据更新后才进行估计
			}
			//以1ms为间距,查询数据的更新情况
			msleep(1);
		}
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			//标志位flag置0并读取数据
			drone_info.imu.updated = false;
			drone_info.mag.updated = false;
			drone_info.global_position.updated = false;
			drone_info.baro.updated = false;
			baro = drone_info.baro;
			mag = drone_info.mag;
			imu = drone_info.imu;
			gps = drone_info.global_position;
		}

		lats += gps.lattitude;
		lons += gps.longtitude;
		gpsHeights += gps.gps_height;
		baroAlts += baro.altitude;

		gyroBiases[0] += imu.angular_v.v_x;
		gyroBiases[1] += imu.angular_v.v_y;
		gyroBiases[2] += imu.angular_v.v_z;
		accBiases[0] += imu.linear_acc.acc_x;
		accBiases[1] += imu.linear_acc.acc_y;
		accBiases[2] += imu.linear_acc.acc_z;
		mags[0] += mag.body_x;
		mags[1] += mag.body_y;
		mags[2] += mag.body_z;

		initCount++;

		if (initializationClock.elapsed() > 1000) {
			blInitilaized = true;
		}
	}

	lats /= initCount;
	lons /= initCount;
	gpsHeights /= initCount;
	baroAlts /= initCount;
	gyroBiases[0] /= initCount;
	gyroBiases[1] /= initCount;
	gyroBiases[2] /= initCount;
	accBiases[0] /= initCount;
	accBiases[1] /= initCount;
	accBiases[2] /= initCount;
	mags[0] /= initCount;
	mags[1] /= initCount;
	mags[2] /= initCount;

	//经纬度换算
	const double lat2meter = 111133.3333;
	const double lon2meter = abs(cos(deg2rad * lats)) * lat2meter;

	//gps经纬度原点
	const double lon_home = lons * lon2meter;
	const double lat_home = lats * lat2meter;

	//gps高度原点
	const double gps_height_home = gpsHeights;

	//气压计原点
	const double baro_home = baroAlts;

	double gps_last_x = 0.0;
	double gps_last_y = 0.0;
	double baro_last = 0.0;

	//TODO on first start +- wrong

	//陀螺仪原点
	double gyro_bias[3] = { gyroBiases[0] ,gyroBiases[1] ,gyroBiases[2] };

	//加速度计零漂
	double acc_bias[3] = { accBiases[0], accBiases[1], 0 };
	show_string(QString::number(accBiases[0]) + "\n");
	show_string(QString::number(accBiases[1]) + "\n");

	//重力加速度的绝对值
	const double acc_gravity = sqrt(
		accBiases[0] * accBiases[0] +
		accBiases[1] * accBiases[1] +
		accBiases[2] * accBiases[2]);

	//初始姿态角
	double first_yaw = -atan2(mags[1], mags[0]);

	//磁北朝向
	const double mag_earth_x = sqrt(mags[1] * mags[1] + mags[0] * mags[0]);
	vec3f_t fixed_mag_earth = { mag_earth_x, 0, mags[2] };

	//姿态四元数初始化
	att_local.Q.q[0] = cos(first_yaw / 2);
	att_local.Q.q[1] = 0;
	att_local.Q.q[2] = 0;
	att_local.Q.q[3] = sin(first_yaw / 2);
	quaternion2rotation(&att_local.Q, &att_local.R);
	att_local.Euler.z = first_yaw;
	AttitudeEstimator est(false);
	est.setAttitudeEuler(first_yaw, 0, 0);

	// Kalman filter initialize
	PVAKF pva_x;
	PVAKF pva_y;
	PVAKF pva_z;


	pva_x.std_dev(0) = 2.15; //p
	pva_x.std_dev(1) = 1.0; //v 
	pva_x.std_dev(2) = 0.032;//a
	pva_x.spa_weight = 0.0;
	pva_x.p2v_weight = 0.02;

	pva_y.std_dev(0) = 2.0; //p
	pva_y.std_dev(1) = 1.0; //v 
	pva_y.std_dev(2) = 0.005;//a
	pva_y.spa_weight = 0.0;
	pva_y.p2v_weight = 0.015;

	pva_z.std_dev(0) = 0.2; //p
	pva_z.std_dev(1) = 1.0; //v 
	pva_z.std_dev(2) = 0.02;//a
	pva_z.spa_weight = 0.0;
	pva_z.p2v_weight = 0.005;

	// Parameters to sovle delay issue
	const int gps_delay_ms = 200;
	const int acc_delay_ms = 30;
	const int buf_length = gps_delay_ms / acc_delay_ms;
	double acc_buffer[buf_length][2]; // x, y only
	double time_buffer[buf_length];
	int delay_counter = 0;

	double gps_buffer[5][3];
	double gps_last[3];

	double ground_x;
	double ground_y;
	double ground_z;

	double fake_vx = 0.0;
	double fake_vy = 0.0;
	double fake_vz = 0.0;

	// Initialize gps buffer & last
	for (int i = 0; i < 5; i++)
	{
		gps_buffer[i][0] = 0.0;
		gps_buffer[i][1] = 0.0;
		gps_buffer[i][2] = 0.0;
	}
	gps_last[0] = 0.0;
	gps_last[1] = 0.0;
	gps_last[2] = 0.0;


	while (true)
	{
		//查询数据是否更新
		got_data = false;
		while (!got_data) {
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				isUsingAPI = drone_info.angluar_setpoint.usingAPI;

				got_data = drone_info.imu.updated;
				got_data &= drone_info.mag.updated;
				got_data &= drone_info.global_position.updated;
				got_data &= drone_info.baro.updated;
			}
			msleep(1);
		}
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			drone_info.imu.updated = false;
			drone_info.mag.updated = false;
			drone_info.global_position.updated = false;
			drone_info.baro.updated = false;
			baro = drone_info.baro;
			mag = drone_info.mag;
			imu = drone_info.imu;
			gps = drone_info.global_position;
			motorOutput = drone_info.angluar_setpoint;

			ground_x = drone_info.ground_truth.px;
			ground_y = drone_info.ground_truth.py;
			ground_z = drone_info.ground_truth.pz;
		}

		//时间流逝
		double dt = dtTimer.elapsed() * 0.001;
		dtTimer.restart();

		/** 姿态估计*/
		//清零修正值
		for (int i = 0; i < 3; i++) {
			corr.v[i] = 0;
		}

		vec3f_t mag_err_earth;
		vec3f_t mag_err_body;
		vec3f_t mag_earth;
		vec3f_t mag_body = { mag.body_x, mag.body_y, mag.body_z };
		vec3f_t gyro = { imu.angular_v.v_x, imu.angular_v.v_y , imu.angular_v.v_z };

		float spinRate = vec3f_length(&gyro);

		//mag
		if (MAG_EST_ALL) {
			//使用磁力计纠正所有姿态角
			vec3f_t fixed_mag_body;
			vec3f_t mag_err;
			for (int i = 0; i < 3; i++) {
				double sum = 0;
				for (int j = 0; j < 3; j++) {
					sum += fixed_mag_earth.v[j] * att_local.R.R[j][i];
				}
				fixed_mag_body.v[i] = sum;
			}
			vec3f_cross(&mag_body, &fixed_mag_body, &mag_err);
			for (int i = 0; i < 3; i++) {
				corr.v[i] += mag_err.v[i] * w_mag_full;
			}
		}
		else {
			//仅纠正yaw
			body2earth(&(att_local.R), &mag_body, &mag_earth, 3);

			mag_err_earth.z = -atan2(mag_earth.y, mag_earth.x);
			mag_err_earth.x = 0;
			mag_err_earth.y = 0;

			float gainMult = 1.0f;
			if (spinRate > fifty_dps) {
				gainMult = min(100.0f, spinRate / fifty_dps);
			}
			earth2body(&(att_local.R), &mag_err_earth, &mag_err_body, 3);
			for (int i = 0; i < 3; i++) {
				corr.v[i] += mag_err_body.v[i] * gainMult * w_mag;
			}
		}

		//使用加速度估计姿态(铅锤方向)
		vec3f_t up_body;
		up_body.x = -(2.0f * (att_local.Q.q1 * att_local.Q.q3 - att_local.Q.q0 * att_local.Q.q2));
		up_body.y = -(2.0f * (att_local.Q.q2 * att_local.Q.q3 + att_local.Q.q0 * att_local.Q.q1));
		up_body.z = -(1.0f - ((att_local.Q.q1 * att_local.Q.q1 + att_local.Q.q2 * att_local.Q.q2) * 2.0f));

		vec3f_t grav_acc = { imu.linear_acc.acc_x - acc_bias[0], imu.linear_acc.acc_y - acc_bias[1], imu.linear_acc.acc_z };
		vec3f_t test_grav_no_motor;
		//添加电机加速度
		//if (isUsingAPI) {
		//	vec3f_t motor_acc_body = { 0, 0, -motorOutput.throttle * 20 };
		//	vec3f_t motor_acc_earth;
		//	body2earth(&att_local.R, &motor_acc_body, &motor_acc_earth, 3);
		//	motor_acc_earth.z += acc_gravity;
		//	earth2body(&att_local.R, &motor_acc_earth, &motor_acc_body, 3);
		//	for (int i = 0; i < 3; i++) {
		//		test_grav_no_motor.v[i] = grav_acc.v[i] - motor_acc_body.v[i];
		//	}
		//	print3num(test_grav_no_motor.x, test_grav_no_motor.y, test_grav_no_motor.z);
		//}
		
		vec3f_normalize(&grav_acc);
		vec3f_t acc_err;
		vec3f_cross(&grav_acc, &up_body, &acc_err);
		if (USING_ACC_EST_ALT) {
			for (int i = 0; i < 3; i++) {
				corr.v[i] += acc_err.v[i] * w_acc;
				//if (isUsingAPI) {
				//	corr.v[i] += acc_err.v[i] * w_acc * 10;
				//}
				//else {
				//	corr.v[i] += acc_err.v[i] * w_acc;
				//}
			}
		}
		

		//使用电机输出进行修正
		if (isUsingAPI) {
			corr.P += (motorOutput.pitch - att_local.Euler.P) * w_motor_output;
			corr.R += (motorOutput.roll - att_local.Euler.R) * w_motor_output;
			// print3num(corr.R, motorOutput.roll, att_local.Euler.R);
		}

		if (spinRate < 0.175 && FIX_GYRO_BIAS) {
			for (int i = 0; i < 3; i++) {
				gyro_bias[i] += corr.v[i] * dt * w_gyro_bias;
				if (gyro_bias[i] > bias_max) {
					gyro_bias[i] = bias_max;
				} 
				else if (gyro_bias[i] < -bias_max) {
					gyro_bias[i] = -bias_max;
				}
			}
		}
		//陀螺仪动态估计
		for (int i = 0; i < 3; i++) {
			gyro.v[i] -= gyro_bias[i];
		}
		att_local.rate = gyro;
		for (int i = 0; i < 3; i++) {
			corr.v[i] += att_local.rate.v[i];
		}

		//通过修正量计算四元数的差分
		quaternion_t derQ;
		quaternion_derivative(&att_local.Q, &derQ, &corr);
		for (int i = 0; i < 4; i++) {
			att_local.Q.q[i] += derQ.q[i] * dt;
		}
		//将结果转化为四元数,姿态,等等
		quaternion_normalize(&att_local.Q);
		quaternion2rotation(&att_local.Q, &att_local.R);
		rotation2euler(&att_local.R, &att_local.Euler);

		/**开始位置估计*/
		vec3f_t acc_earth;
		vec3f_t acc_body = { imu.linear_acc.acc_x - acc_bias[0], imu.linear_acc.acc_y - acc_bias[1], imu.linear_acc.acc_z };
		body2earth(&att_local.R, &acc_body, &acc_earth, 3);
		acc_earth.v[2] += acc_gravity;	//去掉重力加速度得到运动加速度

		double baro_now = baro.altitude - baro_home;
		corr_baro[0] = baro_now - z_est[0];
		corr_baro[1] = (baro_now - baro_last) / dt - z_est[1];	//TODO 直接差分得到的速度,要弃用
		baro_last = baro_now;

		//gps原始数据
		double gps_x = gps.lattitude * lat2meter - lat_home;
		double gps_y = gps.longtitude * lon2meter - lon_home;

		/* New position estimator using KF */
		/* GPS position NED and acc NED are used */
		double ax, vx, px;
		double ay, vy, py;
		double az, vz, pz;

		for (int i = 0; i < 2; i++) {
			if (abs(acc_earth.v[i]) < 0.05) {
				acc_earth.v[i] = 0.0;
			}
		}

		// Data buffer initialize
		if (delay_counter < buf_length) 
		{
			px = 0.0; py = 0.0; pz = 0.0;
			vx = 0.0; vy = 0.0; vz = 0.0;
			ax = 0.0; ay = 0.0; az = 0.0;

			// Fill acc buffer
			acc_buffer[delay_counter][0] = (double)acc_earth.v[0];
			acc_buffer[delay_counter][1] = (double)acc_earth.v[1];
			time_buffer[delay_counter] = dt;

			delay_counter++;

			// Update GPS buffer
			for (int i = 0; i < 4; i++)
			{
				gps_buffer[i][0] = gps_buffer[i + 1][0];
				gps_buffer[i][1] = gps_buffer[i + 1][1];
				gps_buffer[i][2] = gps_buffer[i + 1][2];
			}
			gps_buffer[4][0] = gps_x;
			gps_buffer[4][1] = gps_y;
			gps_buffer[4][2] = gps.gps_height;
		}
		else
		{
			// Update acc buffer
			for (int i = 0; i < buf_length - 1; i++)
			{
				acc_buffer[i][0] = acc_buffer[i + 1][0];
				acc_buffer[i][1] = acc_buffer[i + 1][1];
				time_buffer[i] = time_buffer[i + 1];
			}
			acc_buffer[buf_length - 1][0] = (double)acc_earth.v[0];
			acc_buffer[buf_length - 1][1] = (double)acc_earth.v[1];
			time_buffer[buf_length - 1] = dt;

			// Update GPS buffer
			for (int i = 0; i < 4; i++)
			{
				gps_buffer[i][0] = gps_buffer[i + 1][0];
				gps_buffer[i][1] = gps_buffer[i + 1][1];
				gps_buffer[i][2] = gps_buffer[i + 1][2];
			}
			gps_buffer[4][0] = gps_x;
			gps_buffer[4][1] = gps_y;
			gps_buffer[4][2] = gps.gps_height;

			// Generate a fake velocity 
			double gps_avg_x = 0.0;
			double gps_avg_y = 0.0;
			double gps_avg_z = 0.0;

			for (int i = 0; i < 5; i++)
			{
				gps_avg_x += gps_buffer[i][0];
				gps_avg_y += gps_buffer[i][1];
				gps_avg_z += gps_buffer[i][2];
			}
			gps_avg_x = gps_avg_x / 5.0;
			gps_avg_y = gps_avg_y / 5.0;
			gps_avg_z = gps_avg_z / 5.0;

			fake_vx = (fake_vx + acc_buffer[0][0] * dt) * 0.998 + (gps_avg_x - gps_last[0]) / dt * 0.002; // delayed 200ms
			fake_vy = (fake_vy + acc_buffer[0][1] * dt) * 0.998 + (gps_avg_y - gps_last[1]) / dt * 0.002; // delayed 200ms
			fake_vz = (fake_vz + acc_buffer[0][2] * dt) * 0.995 + (gps_avg_z - gps_last[2]) / dt * 0.005; // delayed 200ms

			gps_last[0] = gps_avg_x;
			gps_last[1] = gps_avg_y;
			gps_last[2] = gps_avg_z;

			// Kalman filter
			pva_x.get_predict_value(gps_x, fake_vx, acc_buffer[0][0], 0.0, dt, px, vx, ax);
			pva_y.get_predict_value(gps_y, fake_vy, acc_buffer[0][1], 0.0, dt, py, vy, ay);
			pva_z.get_predict_value(baro_now / 2.0, fake_vz, (double)acc_earth.v[2], 0.0, dt, pz, vz, az);

			// Using acc to make up delayed data
			vx = fake_vx;
			vy = fake_vy;
			vz = fake_vz;

			for (int i = 1; i < buf_length; i++)
			{
				px += 0.5 * acc_buffer[i][0] * time_buffer[i] * time_buffer[i];
				py += 0.5 * acc_buffer[i][1] * time_buffer[i] * time_buffer[i];

				vx += acc_buffer[i][0] * time_buffer[i];
				vy += acc_buffer[i][1] * time_buffer[i];
				vz += acc_buffer[i][2] * time_buffer[i];
			}
			print4num(fake_vx, vx, px, gps_x);
		}
		// print3num(fake_vx, vx, px);
		
		

		/*vec3f_t coor_acc_bias_earth = { px - lastX ,py - lastY ,pz - lastZ };
		vec3f_t coor_acc_bias_body;
		earth2body(&att_local.R, &coor_acc_bias_earth, &coor_acc_bias_body, 3);*/
		//acc_bias[0] -= coor_acc_bias_body.v[0] * w_acc_bias_xy * dt;
		//acc_bias[1] -= coor_acc_bias_body.v[1] * w_acc_bias_xy * dt;
		//acc_bias[2] -= coor_acc_bias_body.v[2] * w_acc_bias_xy * dt;

		//print3num(gps_x, px, ground_x);
		//print3num(imu.linear_acc.acc_x, imu.linear_acc.acc_y, imu.linear_acc.acc_z);
		/*print3num(acc_bias[0], acc_bias[1], acc_bias[2]);
		print3num(acc_body.v[0], acc_body.v[1], acc_body.v[2]);
		print3num(acc_earth.x, acc_earth.y, acc_earth.z);*/

		//print3num(gps_x, gps_y, gps.gps_height - gps_height_home);

		{
			QMutexLocker data_locker(&drone_info.data_mutex);

			//体坐标系的加速度值
			drone_info.test_value.acc[0] = imu.linear_acc.acc_x;
			drone_info.test_value.acc[1] = imu.linear_acc.acc_y;
			drone_info.test_value.acc[2] = imu.linear_acc.acc_z;
			//drone_info.test_value.acc[0] = acc_earth.v[0];
			//drone_info.test_value.acc[1] = acc_earth.v[1];
			//drone_info.test_value.acc[2] = acc_earth.v[2];
			//陀螺仪值,去除初始偏差
			drone_info.test_value.gyo[0] = imu.angular_v.v_x - gyro_bias[0];
			drone_info.test_value.gyo[1] = imu.angular_v.v_y - gyro_bias[1];
			drone_info.test_value.gyo[2] = imu.angular_v.v_z - gyro_bias[2];
			//体坐标系的磁力计值
			drone_info.test_value.mag[0] = mag.body_x;
			drone_info.test_value.mag[1] = mag.body_y;
			drone_info.test_value.mag[2] = mag.body_z;
			//gps的位置和高度
			drone_info.test_value.gps_e = gps_y;
			drone_info.test_value.gps_n = gps_x;
			drone_info.test_value.gps_h = gps.gps_height - gps_height_home;
			//气压计值(减去初始值）
			drone_info.test_value.baro = baro_now;

			//估计的位置
			drone_info.local_position.position.x = 0.0; //px;
			drone_info.local_position.position.y = 0.0; //py;
			drone_info.local_position.position.z = 0.0; //pz;

			//估计的速度
			drone_info.local_position.velocity.vx = 0.0;// vx;
			drone_info.local_position.velocity.vy = 0.0; //vy;
			drone_info.local_position.velocity.vz = 0.0; //  vz;

			//估计的姿态
			drone_info.attitude.angle.roll = att_local.Euler.x;
			drone_info.attitude.angle.pitch = att_local.Euler.y;
			drone_info.attitude.angle.yaw = att_local.Euler.z;


			drone_info.test_value.test1 = att_local.Euler.x;
			drone_info.test_value.test2 = att_local.Euler.y;
			drone_info.test_value.test3 = att_local.Euler.z;

			drone_info.degree_values_cal();
		}

		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}

}


void body2earth(const rotation_t* R, const vec3f_t* body, vec3f_t* earth, short dimension)
{
	if (dimension == 2) {
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		earth->x = body->x*cos(yaw) + body->y*sin(-yaw);
		earth->y = body->x*sin(yaw) + body->y*cos(yaw);
	}
	else if (dimension == 3) {
		earth->x = (body->x*R->R[0][0] + body->y*R->R[0][1] + body->z*R->R[0][2]);
		earth->y = (body->x*R->R[1][0] + body->y*R->R[1][1] + body->z*R->R[1][2]);
		earth->z = (body->x*R->R[2][0] + body->y*R->R[2][1] + body->z*R->R[2][2]);
	}
}

void earth2body(const rotation_t* R, const vec3f_t* earth, vec3f_t* body, short dimension)//body=inv(R)*earth
{
	if (dimension == 2) {
		float yaw = -atan2(R->R[0][1], R->R[1][1]);
		body->x = earth->x*cos(yaw) + earth->y*sin(yaw);
		body->y = earth->x*sin(-yaw) + earth->y*cos(yaw);
	}
	else if (dimension == 3) {
		body->x = (earth->x*R->R[0][0] + earth->y*R->R[1][0] + earth->z*R->R[2][0]);
		body->y = (earth->x*R->R[0][1] + earth->y*R->R[1][1] + earth->z*R->R[2][1]);
		body->z = (earth->x*R->R[0][2] + earth->y*R->R[1][2] + earth->z*R->R[2][2]);
	}
}

float vec3f_length(const vec3f_t * v)
{
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void quaternion_normalize(quaternion_t* Q)
{
	float q0, q1, q2, q3;
	float inv_norm;
	q0 = Q->q0;
	q1 = Q->q1;
	q2 = Q->q2;
	q3 = Q->q3;
	inv_norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= inv_norm;
	q1 /= inv_norm;
	q2 /= inv_norm;
	q3 /= inv_norm;
	Q->q0 = q0;
	Q->q1 = q1;
	Q->q2 = q2;
	Q->q3 = q3;
}

void vec3f_normalize(vec3f_t * v)
{
	float inv_norm;
	inv_norm = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	v->x /= inv_norm;
	v->y /= inv_norm;
	v->z /= inv_norm;
}

void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d)
{
	/*
	a X b = | i		j		k	|
	| ax	ay		az	|
	| bx	by		bz	|
	where |.| is the determinant
	*/
	d->x = a->y * b->z - a->z * b->y;
	d->y = a->z * b->x - a->x * b->z;
	d->z = a->x * b->y - a->y * b->x;
}

void quaternion_derivative(const quaternion_t* Q, quaternion_t* derQ, const vec3f_t* w)
{
	int i;
	float dataQ[] = {
		Q->q[0], -Q->q[1], -Q->q[2], -Q->q[3],
		Q->q[1],  Q->q[0], -Q->q[3],  Q->q[2],
		Q->q[2],  Q->q[3],  Q->q[0], -Q->q[1],
		Q->q[3], -Q->q[2],  Q->q[1],  Q->q[0]
	};
	float V[4] = { 0,w->x,w->y,w->z };
	float result[4];

	for (int i = 0; i < 4; i++) {
		float sum = 0;
		for (int j = 0; j < 4; j++) {
			sum += dataQ[i * 4 + j] * V[j];
		}
		result[i] = sum / 2;
	}

	for (i = 0; i<4; i++) {
		derQ->q[i] = result[i];
	}
}

void quaternion2rotation(const quaternion_t* Q, rotation_t* R)
{
	float q0, q1, q2, q3;
	q0 = Q->q0;
	q1 = Q->q1;
	q2 = Q->q2;
	q3 = Q->q3;
	R->R[0][0] = 1.0f - ((q2 * q2 + q3 * q3) * 2.0f);
	R->R[0][1] = (q1 * q2 - q0 * q3) * 2.0f;
	R->R[0][2] = (q1 * q3 + q0 * q2) * 2.0f;
	R->R[1][0] = (q1 * q2 + q0 * q3) * 2.0f;
	R->R[1][1] = 1.0f - ((q1 * q1 + q3 * q3) * 2.0f);
	R->R[1][2] = (q2 * q3 - q0 * q1) * 2.0f;
	R->R[2][0] = (q1 * q3 - q0 * q2) * 2.0f;
	R->R[2][1] = (q2 * q3 + q0 * q1) * 2.0f;
	R->R[2][2] = 1.0f - ((q1 * q1 + q2 * q2) * 2.0f);
}

void rotation2euler(const rotation_t* R, vec3f_t* Euler)
{
	Euler->P = -asin(R->R[2][0]);
	Euler->R = atan2(R->R[2][1], R->R[2][2]);
	Euler->Y = -atan2(R->R[0][1], R->R[1][1]);
}

void inertial_filter_predict(double dt, double x[2], double acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		x[0] += (x[1] * dt + acc * dt * dt / 2.0f) * w_acc_pos_vel;
		x[1] += (acc * dt) * w_acc_pos_vel;
	}
}

void inertial_filter_correct(double e, double dt, double x[2], int i, double w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;
		if (w > 1.0f)
			w = 1.0f;
		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}


PVAKF::PVAKF()
{
	// Initialize
	P = Matrix3d::Identity();
	Q = Matrix3d::Zero();
	F = Matrix3d::Identity();

	H = Matrix3d::Zero();
	H(0, 0) = 1.0; // Enable gps
	H(1, 1) = 1.0; // Disable velocity sensor
	H(2, 2) = 1.0; // Enable acc

	std_dev = Vector3d::Zero();
	spa_weight = 0.0;
	p2v_weight = 0.0;

	x = Vector3d::Zero();
	z = Vector3d::Zero();
	K = Vector3d::Zero();

	last_p = 0.0;
}

void PVAKF::get_predict_value(double p, double v, double a, double spa, double dt, double &rp, double &rv, double &ra)
{
	// Kalman Book P143, P147
	// Predict
	F(0, 1) = dt;
	F(0, 2) = 0.5 * dt * dt;
	F(1, 2) = dt;


	Vector3d sp_influence = Vector3d::Zero();
	if (spa_weight > 0.01)
	{
		double delt_a = spa - a;	
		sp_influence(0) = 0.5 * dt * dt * spa_weight * delt_a;
		sp_influence(1) = dt * spa_weight * delt_a;
		sp_influence(2) = spa_weight * delt_a;	
	}
	x = F * x + sp_influence; // KF 1
	//x(1) = x(1) * 0.9 + 0.1 * v;  

	double dif_v = (p - last_p) / dt;

	if (p2v_weight > 0.0)
	{
		x(1) = (1.0 - p2v_weight) * x(1) + p2v_weight * dif_v;
	}

	
	P = F * P * F.transpose() + Q; // KF 2

	// Correct
	z(0) = p;
	z(1) = dif_v; //v;
	z(2) = a;
	last_p = p;

	for (int i = 0; i < 3; i++)
	{
		double temp_d = H.row(i) * P * H.row(i).transpose() + std_dev(i);
		if (temp_d == 0.0) continue;
		K = (1 / temp_d) * (P * H.row(i).transpose());
		P = P - K * H.row(i) * P;
		x = x + K * (z(i) - H.row(i)*x);
	}

	rp = x(0);
	rv = x(1);
	ra = x(2);
}