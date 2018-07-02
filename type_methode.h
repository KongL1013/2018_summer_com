#ifndef TYPE_METHODS_H
#define TYPE_METHODS_H
//#include <math.h>
#include "basic_types.h"
#include "droneInfo.h"
//#include "Eigen/Eigen/Eigen"
//#include "Eigen/Eigen/Geometry"
//using namespace Eigen;

void body2earth(const Eigen::Matrix3f* R, const Eigen::Vector3f* body, Eigen::Vector3f* earth, short dimension);

void earth2body(const Eigen::Matrix3f* R, const Eigen::Vector3f* earth, Eigen::Vector3f* body, short dimension);//body=inv(R)*earth

void rotation2euler(const Eigen::Matrix3f* R, Eigen::Vector3f* Euler);

void euler2rotation(const Eigen::Vector3f* Euler, Eigen::Matrix3f* R);

float data_2_angle(float x, float y, float z);	 //in rad

void vec3f_norm(const Eigen::Vector3f* a, float* anwser);

float deriv_f(const	float f_now, const float f_past , const float dt);

void vec3f_normalize(Eigen::Vector3f* v);

void vec3f_passnorm(const Eigen::Vector3f* v, Eigen::Vector3f* vec_des);

float vec3f_length(const Eigen::Vector3f* v);

float vec3f_dot(const Eigen::Vector3f* a, const Eigen::Vector3f* b);

void vec3f_cross(const Eigen::Vector3f* a, const Eigen::Vector3f* b, Eigen::Vector3f* d);

void vec3f_angle(Eigen::Vector3f* a, Eigen::Vector3f* b, float* angleInRad);

float degToRad(float deg);

float radToDeg(float deg);

void number_times_vec3f(const float& a, Eigen::Vector3f* v);

void vec3f_devide_number(Eigen::Vector3f* v, const float& a);

Eigen::Vector3f vec3f_minus(Eigen::Vector4f* a, Eigen::Vector3f* b);

void droneInfo2Vector3f(const DroneInfo& droneinfo, Eigen::Vector3f* posEst, Eigen::Vector3f* velEst);

void vec3f_add(Eigen::Vector4f* a, Eigen::Vector3f* b);
#endif