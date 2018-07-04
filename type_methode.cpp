#include "type_methode.h"

float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float deriv_f(const	float f_now, const float f_past , const float dt)
{
	return (f_now - f_past)/dt;
}
void vec3f_normalize(Eigen::Vector3f* v)
{
	if((*v)(0)==0&&(*v)(1)==0&&(*v)(2)==0)
	{}
	else{
	float inv_norm;
	inv_norm = sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
	(*v)(0) /= inv_norm;
	(*v)(1) /= inv_norm;
	(*v)(2) /= inv_norm;
	}
}
void vec3f_norm(const Eigen::Vector3f* a, float* anwser)
{
	*anwser = sqrtf((*a)(0)*(*a)(0) + (*a)(1)*(*a)(1) + (*a)(2)*(*a)(2));
}
void vec3f_cross(const Eigen::Vector3f* a, const Eigen::Vector3f* b, Eigen::Vector3f* d)
{
/*
a X b = | i		j		k	|
		| ax	ay		az	|
		| bx	by		bz	|
	where |.| is the determinant
*/	
	(*d)(0) = (*a)(1) * (*b)(2) - (*a)(2) * (*b)(1);
	(*d)(1) = (*a)(2) * (*b)(0) - (*a)(0) * (*b)(2);
	(*d)(2) = (*a)(0) * (*b)(1) - (*a)(1) * (*b)(0);
}
float vec3f_dot(const Eigen::Vector3f* a, const Eigen::Vector3f* b)
{
	return ((*a)(0) * (*b)(0) + (*a)(1) * (*b)(1) + (*a)(2) * (*b)(2));
}
void vec3f_passnorm(const Eigen::Vector3f* v, Eigen::Vector3f* vec_des)
{
	float inv_norm;
	inv_norm = sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
	(*vec_des)(0) = (*v)(0) / inv_norm;
	(*vec_des)(1) = (*v)(1) / inv_norm;
	(*vec_des)(2) = (*v)(2) / inv_norm;
}



float vec3f_length(const Eigen::Vector3f* v)
{
	return sqrtf((*v)(0) * (*v)(0) + (*v)(1) * (*v)(1) + (*v)(2) * (*v)(2));
}
void body2earth(const Eigen::Matrix3f* R, const Eigen::Vector3f* body, Eigen::Vector3f* earth, short dimension)
{	
	if(dimension == 2){
		float yaw = -atan2((*R)(0,1), (*R)(1,1));
		(*earth)(0) = (*body)(0)*cos(yaw) + (*body)(1)*sin(-yaw);
		(*earth)(1) = (*body)(0)*sin(yaw) + (*body)(1)*cos(yaw); 
	}
	else if(dimension == 3){
		(*earth)(0) = ((*body)(0)*(*R)(0,0) + (*body)(1)*(*R)(0,1) + (*body)(2)*(*R)(0,2));
		(*earth)(1) = ((*body)(0)*(*R)(1,0) + (*body)(1)*(*R)(1,1) + (*body)(2)*(*R)(1,2));
		(*earth)(2) = ((*body)(0)*(*R)(2,0) + (*body)(1)*(*R)(2,1) + (*body)(2)*(*R)(2,2));
	}
}
void earth2body(const Eigen::Matrix3f* R, const Eigen::Vector3f* earth, Eigen::Vector3f* body, short dimension)//body=inv(R)*earth
{
	if(dimension == 2){
		float yaw = -atan2((*R)(0,1), (*R)(1,1));
		(*body)(0) = (*earth)(0)*cos(yaw) + (*earth)(1)*sin(yaw);
		(*body)(1) = (*earth)(0)*sin(-yaw) + (*earth)(1)*cos(yaw); 
	}
	else if(dimension == 3){
		(*body)(0) = ((*earth)(0)*(*R)(0,0) + (*earth)(1)*(*R)(1,0) + (*earth)(2)*(*R)(2,0));
		(*body)(1) = ((*earth)(0)*(*R)(0,1) + (*earth)(1)*(*R)(1,1) + (*earth)(2)*(*R)(2,1));
		(*body)(2) = ((*earth)(0)*(*R)(0,2) + (*earth)(1)*(*R)(1,2) + (*earth)(2)*(*R)(2,2));
	}
}
void rotation2euler(const Eigen::Matrix3f* R, Eigen::Vector3f* Euler)
{
	////hongzhe
	////(*Euler)(1) = -asin((*R)(2,0));

	//(*Euler)(0) = atan2((*R)(2,1), (*R)(2,2));
	//float x = (*R)(2, 1)*(*R)(2, 1);
	//float y = (*R)(2, 2)*(*R)(2, 2);
	//(*Euler)(1) = atan2(-(*R)(2, 0), sqrtf(x+y));
	//(*Euler)(2) = atan2((*R)(1,0), (*R)(0,0));
	////(*Euler)(2) = -atan2((*R)(0,1), (*R)(1,1));
	
	//(*Euler)(0) = atan2((*R)(2, 1), (*R)(2, 2));
	(*Euler)(0) = atan((*R)(2, 1)/(*R)(2, 2));
	(*Euler)(1) = -asin((*R)(2, 0));
	(*Euler)(2) = -atan2((*R)(0, 1), (*R)(1, 1));
}
void euler2rotation(const Eigen::Vector3f* Euler, Eigen::Matrix3f* R)
{
	float cp = cos((*Euler)(1));
	float sp = sin((*Euler)(1));
	float sr = sin((*Euler)(0));
	float cr = cos((*Euler)(0));
	float sy = sin((*Euler)(2));
	float cy = cos((*Euler)(2));
	(*R)(0,0) = cp * cy;
	(*R)(0,1) = -((sr * sp * cy) - (cr * sy));
	(*R)(0,2) = ((cr * sp * cy) + (sr * sy));
	(*R)(1,0) = cp * sy;
	(*R)(1,1) = ((sr * sp * sy) + (cr * cy));
	(*R)(1,2) = ((cr * sp * sy) - (sr * cy));
	(*R)(2,0) = -sp;
	(*R)(2,1) = sr * cp;
	(*R)(2,2) = cr * cp;	
}


float data_2_angle(float x, float y, float z)	 //in rad
{
	float res;
	res = atan2(x,sqrtf(y*y+z*z));
	return res;
}
float degToRad(float deg) {
    return deg / 180.0f * 3.1415926;
}

float radToDeg(float rad) {
    return rad * 180.0f / 3.1415926;
}


void vec3f_angle(Eigen::Vector3f* a, Eigen::Vector3f* b, float* angleInRad)
{
	float norm_a, norm_b;
	vec3f_norm(a,&norm_a);
	vec3f_norm(b,&norm_b);
	*angleInRad = acos(vec3f_dot(a,b)/(norm_a*norm_b)); //range--[0:PI]
}

void number_times_vec3f(const float& a, Eigen::Vector3f* v)
{
	(*v)(0) = a * (*v)(0);
	(*v)(1) = a * (*v)(1); 
	(*v)(2) = a * (*v)(2);
}
void vec3f_devide_number(Eigen::Vector3f* v, const float& a)
{
	(*v)(0) =  (*v)(0) / a;
	(*v)(1) =  (*v)(1) / a;
	(*v)(2) =  (*v)(2) / a;
}

Eigen::Vector3f vec3f_minus(Eigen::Vector4f* a, Eigen::Vector3f* b)
{
	Eigen::Vector3f result;
	result.setZero();
	for(int i=0;i<3;++i)
	{
		result(i) = (*a)(i) - (*b)(i);
	}
	return result;
}

void vec3f_add(Eigen::Vector4f* a, Eigen::Vector3f* b)
{
	for (int i = 0; i<3; ++i)
	{
		(*a)(i) += (*b)(i);
	}
}

void droneInfo2Vector3f(const DroneInfo& droneinfo, Eigen::Vector3f* posEst, Eigen::Vector3f* velEst)
{
	(*posEst)(0) = droneinfo.local_position.position.x;
	(*posEst)(1) = droneinfo.local_position.position.y;
	(*posEst)(2) = droneinfo.local_position.position.z;
	(*velEst)(0) = droneinfo.local_position.velocity.vx;
	(*velEst)(1) = droneinfo.local_position.velocity.vy;
	(*velEst)(2) = droneinfo.local_position.velocity.vz;
	
}

