#pragma once

#include <QtCore>  
#include <QMutex>  
#include <QApplication>
#include <QThread>
#include <vector>
#include "common/CommonStructs.hpp"
#include <QImage> 
#include <Eigen\Dense>

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

class PVAKF
{
public:

	PVAKF();

	void get_predict_value(double p, double v, double a, double spa, double dt, double &rp, double &rv, double &ra);

	Eigen::Matrix3d Q; // Noise matrix
	Eigen::Matrix3d H; // p, v, a sensibility. Normally, if you use a sensor, set the corresponding diagonal element to 1, otherwise 0.
	Eigen::Vector3d std_dev; // Sensor Standard Deviation, for R
	double spa_weight; // Weight of acc setpoint
	
private:
	Eigen::Matrix3d P; // X covariance matrix
	Eigen::Vector3d x; // State. p, v, a
	Eigen::Vector3d z; // Sensor value. p, v, a
	Eigen::Vector3d K; // Kalman factor
	Eigen::Matrix3d F; // State transition matrix
};

typedef struct vec3f_s {
	union {
		struct {
			float R;
			float P;
			float Y;
		};
		struct {
			float x;
			float y;
			float z;
		};
		float v[3];
	};
}vec3f_t;

typedef struct quaternion_s {
	union {
		struct {
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float q[4];
	};
} quaternion_t;

typedef struct rotation_s {
	float R[3][3];
} rotation_t;

typedef struct att_s {
	vec3f_t Euler;
	quaternion_t Q;
	rotation_t R;
	vec3f_t rate;
} att_t;//used in queue

void earth2body(const rotation_t* R, const vec3f_t* earth, vec3f_t* body, short dimension);
void body2earth(const rotation_t* R, const vec3f_t* body, vec3f_t* earth, short dimension);
float vec3f_length(const vec3f_t * v);
void quaternion_normalize(quaternion_t* Q);
void vec3f_normalize(vec3f_t * v);
void vec3f_cross(const vec3f_t * a, const vec3f_t * b, vec3f_t * d);
void quaternion_derivative(const quaternion_t* Q, quaternion_t* derQ, const vec3f_t* w);
void quaternion2rotation(const quaternion_t* Q, rotation_t* R);
void rotation2euler(const rotation_t* R, vec3f_t* Euler);
void inertial_filter_correct(double e, double dt, double x[2], int i, double w);
void inertial_filter_predict(double dt, double x[2], double acc);