
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

#define MAG_EST_ALL false //flase: ֻ����yaw  true: ����3���Ƕ�
#define FIX_GYRO_BIAS false //�Ƿ�������Ư
#define USING_ACC_EST_ALT true //�Ƿ�ʹ�ü��ٶȼ�������

using namespace std;
using namespace msr::airlib;
using namespace stateestimation;
using namespace Eigen;

extern DroneInfo drone_info;

const float w_mag = 0.4f;
const float w_acc = 0.01f;
const float w_mag_full = 0.5f;
const double w_acc_pos_vel = 0.8;
const float w_motor_output = 0.2f;

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

	/**��ʼ���ɼ�1s������*/

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
				//ֻ���������ݸ��º�Ž��й���
			}
			//��1msΪ���,��ѯ���ݵĸ������
			msleep(1);
		}
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			//��־λflag��0����ȡ����
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

	//��γ�Ȼ���
	const double lat2meter = 111133.3333;
	const double lon2meter = abs(cos(deg2rad * lats)) * lat2meter;

	//gps��γ��ԭ��
	const double lon_home = lons * lon2meter;
	const double lat_home = lats * lat2meter;

	//gps�߶�ԭ��
	const double gps_height_home = gpsHeights;

	//��ѹ��ԭ��
	const double baro_home = baroAlts;

	double gps_last_x = 0.0;
	double gps_last_y = 0.0;
	double baro_last = 0.0;

	//TODO on first start +- wrong

	//������ԭ��
	double gyro_bias[3] = { gyroBiases[0] ,gyroBiases[1] ,gyroBiases[2] };

	//�������ٶȵľ���ֵ
	const double acc_gravity = sqrt(
		accBiases[0] * accBiases[0] +
		accBiases[1] * accBiases[1] +
		accBiases[2] * accBiases[2]);

	//��ʼ��̬��
	double first_yaw = -atan2(mags[1], mags[0]);

	//�ű�����
	const double mag_earth_x = sqrt(mags[1] * mags[1] + mags[0] * mags[0]);
	vec3f_t fixed_mag_earth = { mag_earth_x, 0, mags[2] };

	//��̬��Ԫ����ʼ��
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

	pva_x.std_dev(0) = 1.15; //p
	pva_x.std_dev(1) = 0.5; //v 
	pva_x.std_dev(2) = 0.032;//p
	pva_x.spa_weight = 0.0;

	pva_y.std_dev(0) = 1.0; //p
	pva_y.std_dev(1) = 0.5; //v 
	pva_y.std_dev(2) = 0.005;//a
	pva_y.spa_weight = 0.0;

	// int gps_delay_ms = 200;

	
	while (true)
	{
		//��ѯ�����Ƿ����
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
		}

		//ʱ������
		double dt = dtTimer.elapsed() * 0.001;
		dtTimer.restart();

		/** ��̬����*/
		//��������ֵ
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
			//ʹ�ô����ƾ���������̬��
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
			//������yaw
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

		//ʹ�ü��ٶȹ�����̬(Ǧ������)
		vec3f_t up_body;
		up_body.x = -(2.0f * (att_local.Q.q1 * att_local.Q.q3 - att_local.Q.q0 * att_local.Q.q2));
		up_body.y = -(2.0f * (att_local.Q.q2 * att_local.Q.q3 + att_local.Q.q0 * att_local.Q.q1));
		up_body.z = -(1.0f - ((att_local.Q.q1 * att_local.Q.q1 + att_local.Q.q2 * att_local.Q.q2) * 2.0f));

		vec3f_t grav_acc = { imu.linear_acc.acc_x, imu.linear_acc.acc_y, imu.linear_acc.acc_z };
		//��ӵ�����ٶ�
		if (isUsingAPI) {
			vec3f_t motor_acc_body = { 0, 0, -motorOutput.throttle * 20 };
			vec3f_t motor_acc_earth;
			body2earth(&att_local.R, &motor_acc_body, &motor_acc_earth, 3);
			motor_acc_earth.z += acc_gravity;
			earth2body(&att_local.R, &motor_acc_earth, &motor_acc_body, 3);
			for (int i = 0; i < 3; i++) {
				grav_acc.v[i] -= motor_acc_body.v[i];
			}
		}
		
		vec3f_normalize(&grav_acc);
		vec3f_t acc_err;
		vec3f_cross(&grav_acc, &up_body, &acc_err);
		if (USING_ACC_EST_ALT) {
			for (int i = 0; i < 3; i++) {
				if (isUsingAPI) {
					corr.v[i] += acc_err.v[i] * w_acc * 10;
				}
				else {
					corr.v[i] += acc_err.v[i] * w_acc;
				}
			}
		}
		

		//ʹ�õ�������������
		if (isUsingAPI) {
			corr.P += (motorOutput.pitch - att_local.Euler.P) * w_motor_output;
			corr.R += (motorOutput.roll - att_local.Euler.R) * w_motor_output;
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
		//�����Ƕ�̬����
		for (int i = 0; i < 3; i++) {
			gyro.v[i] -= gyro_bias[i];
		}
		att_local.rate = gyro;
		for (int i = 0; i < 3; i++) {
			corr.v[i] += att_local.rate.v[i];
		}

		//ͨ��������������Ԫ���Ĳ��
		quaternion_t derQ;
		quaternion_derivative(&att_local.Q, &derQ, &corr);
		for (int i = 0; i < 4; i++) {
			att_local.Q.q[i] += derQ.q[i] * dt;
		}
		//�����ת��Ϊ��Ԫ��,��̬,�ȵ�
		quaternion_normalize(&att_local.Q);
		quaternion2rotation(&att_local.Q, &att_local.R);
		rotation2euler(&att_local.R, &att_local.Euler);

		/**��ʼλ�ù���*/
		vec3f_t acc_earth;
		vec3f_t acc_body = { imu.linear_acc.acc_x, imu.linear_acc.acc_y, imu.linear_acc.acc_z };
		body2earth(&att_local.R, &acc_body, &acc_earth, 3);
		acc_earth.v[2] += acc_gravity;	//ȥ���������ٶȵõ��˶����ٶ�

		double baro_now = baro.altitude - baro_home;
		corr_baro[0] = baro_now - z_est[0];
		corr_baro[1] = (baro_now - baro_last) / dt - z_est[1];	//TODO ֱ�Ӳ�ֵõ����ٶ�,Ҫ����
		baro_last = baro_now;

		//gpsԭʼ����
		double gps_x = gps.lattitude * lat2meter - lat_home;
		double gps_y = gps.longtitude * lon2meter - lon_home;

		/* New position estimator using KF */
		/* GPS position NED and acc NED are used */
		
		// X, a,v,p Kalman filter
		double ax, vx, px;
		double ay, vy, py;
		pva_x.get_predict_value(gps_x, 0.0, (double)acc_earth.v[0], 0.0, dt, px, vx, ax);
		pva_y.get_predict_value(gps_y, 0.0, (double)acc_earth.v[1], 0.0, dt, py, vy, ay);

		print3num(gps_x, px, (double)acc_earth.v[0]);

		//print3num(gps_x, gps_y, gps.gps_height - gps_height_home);


		/* The following is the old position estimator*/
		//corr_gps[0][0] = gps_x - x_est[0];
		//corr_gps[1][0] = gps_y - y_est[0];
		//corr_gps[0][1] = (gps_x - gps_last_x) / dt - x_est[1]; //TODO ֱ�Ӳ�ֵõ����ٶ�,Ҫ����
		//corr_gps[1][1] = (gps_y - gps_last_y) / dt - y_est[1]; //TODO ֱ�Ӳ�ֵõ����ٶ�,Ҫ����
		//gps_last_x = gps_x;
		//gps_last_y = gps_y;

		//ʹ�ü��ٶȹ��Ƶõ�λ�ú��ٶ�
		//inertial_filter_predict(dt, x_est, acc_earth.x);
		//inertial_filter_predict(dt, y_est, acc_earth.y);
		//inertial_filter_predict(dt, z_est, acc_earth.z);

		/*inertial_filter_correct(corr_baro[0], dt, z_est, 0, w_z_baro_p);
		inertial_filter_correct(corr_baro[1], dt, z_est, 1, w_z_baro_v);
		inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
		inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);
		inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
		inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);*/
		/* End of the old position estimator */

		QString temp = QString::number(dt) + "\n";
		//show_string(std::move(temp));
		// print3num(up_body.x, up_body.y, up_body.z);

		{
			QMutexLocker data_locker(&drone_info.data_mutex);

			//������ϵ�ļ��ٶ�ֵ
			drone_info.test_value.acc[0] = imu.linear_acc.acc_x;
			drone_info.test_value.acc[1] = imu.linear_acc.acc_y;
			drone_info.test_value.acc[2] = imu.linear_acc.acc_z;
			//drone_info.test_value.acc[0] = acc_earth.v[0];
			//drone_info.test_value.acc[1] = acc_earth.v[1];
			//drone_info.test_value.acc[2] = acc_earth.v[2];
			//������ֵ,ȥ����ʼƫ��
			drone_info.test_value.gyo[0] = imu.angular_v.v_x - gyro_bias[0];
			drone_info.test_value.gyo[1] = imu.angular_v.v_y - gyro_bias[1];
			drone_info.test_value.gyo[2] = imu.angular_v.v_z - gyro_bias[2];
			//������ϵ�Ĵ�����ֵ
			drone_info.test_value.mag[0] = mag.body_x;
			drone_info.test_value.mag[1] = mag.body_y;
			drone_info.test_value.mag[2] = mag.body_z;
			//gps��λ�ú͸߶�
			drone_info.test_value.gps_e = gps_y;
			drone_info.test_value.gps_n = gps_x;
			drone_info.test_value.gps_h = gps.gps_height - gps_height_home;
			//��ѹ��ֵ(��ȥ��ʼֵ��
			drone_info.test_value.baro = baro_now;

			//���Ƶ�λ��
			drone_info.local_position.position.x = x_est[0];
			drone_info.local_position.position.y = y_est[0];
			drone_info.local_position.position.z = z_est[0];

			//���Ƶ��ٶ�
			drone_info.local_position.velocity.vx = x_est[1];
			drone_info.local_position.velocity.vy = y_est[1];
			drone_info.local_position.velocity.vz = z_est[1];

			//���Ƶ���̬
			drone_info.attitude.angle.roll = att_local.Euler.x;
			drone_info.attitude.angle.pitch = att_local.Euler.y;
			drone_info.attitude.angle.yaw = att_local.Euler.z;


			drone_info.test_value.test1 = acc_earth.v[0];
			drone_info.test_value.test2 = acc_earth.v[1];
			drone_info.test_value.test3 = acc_earth.v[2];

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
	H(1, 1) = 0.0; // Disable velocity sensor
	H(2, 2) = 1.0; // Enable acc

	std_dev = Vector3d::Zero();
	spa_weight = 0.0;

	x = Vector3d::Zero();
	z = Vector3d::Zero();
	K = Vector3d::Zero();
}

void PVAKF::get_predict_value(double p, double v, double a, double spa, double dt, double &rp, double &rv, double &ra)
{
	// Kalman Book P143, P147
	// Predict
	F(0, 1) = dt;
	F(0, 2) = 0.5 * dt * dt;
	F(1, 2) = dt;

	if (spa_weight > 0.0001)
	{
		double delt_a = spa - a;
		Vector3d sp_influence;
		sp_influence(0) = 0.5 * dt * dt * spa_weight * delt_a;
		sp_influence(1) = dt * spa_weight * delt_a;
		sp_influence(2) = spa_weight * delt_a;
		x = F * x + sp_influence; // KF 1
	}
	else
	{
		x = F * x; // KF 1
	}
	
	P = F * P * F.transpose() + Q; // KF 2

	// Correct
	z(0) = p;
	z(1) = v;
	z(2) = a;

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