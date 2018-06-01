#include "droneInfo.h"

DroneInfo::DroneInfo()
{
	/*If you want some initialization*/
	attitude.angle.pitch = 0.f;
	attitude.angle.roll = 0.f;
	attitude.angle.yaw = 0.f;

	degree_values_cal();
}

void DroneInfo::rad_to_degree(float &rad, float &degree)
{
	degree = rad * 57.325;
}

void DroneInfo::degree_values_cal()
{
	rad_to_degree(attitude.angle.pitch, attitude.angle_d.pitch_d);
	rad_to_degree(attitude.angle.roll, attitude.angle_d.roll_d);
	rad_to_degree(attitude.angle.yaw, attitude.angle_d.yaw_d);
}