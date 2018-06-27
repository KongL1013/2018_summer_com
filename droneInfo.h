#pragma once

#include <QMutex>  
#include <QImage>  

class DroneInfo
{
public:
	
	DroneInfo();

	void rad_to_degree(float &rad, float &degree);
	void degree_values_cal();
	
	/* Useful Data*/
	QMutex data_mutex;

	struct Images
	{
		QImage front_rgb;
		QImage front_depth;
		QImage down_rgb;
		 
		long int time_stamp_front_rgb;  //MS
		long int time_stamp_front_depth;
		long int time_stamp_down_rgb;
		bool updated;
	}images;

	struct Barometer
	{
		double altitude;
		double pressure;

		long int time_stamp; //MS
		bool updated;
	}baro;

	struct Magnetic
	{
		double body_x;
		double body_y;
		double body_z;

		long int time_stamp; //MS
		bool updated;
	}mag;

	struct IMU
	{
		struct AngularVelocity
		{
			double v_x;
			double v_y;
			double v_z;
		}angular_v;
		struct LinearAcc
		{
			double acc_x;
			double acc_y;
			double acc_z;
		}linear_acc;

		long int time_stamp; //MS
		bool updated;
	}imu;

	struct Attitude
	{
		struct Angle  // rad
		{
			float pitch;
			float roll;
			float yaw;
		}angle;

		struct Angle_d  // degree
		{
			float pitch_d;
			float roll_d;
			float yaw_d;
		}angle_d;

		struct AngularVelocity
		{
			float pitch_rate;  // rad/s
			float roll_rate;
			float yaw_rate;
		}anglar_velocity;

	}attitude;

	struct LocalPosition
	{
		struct Position  // m
		{
			double x;
			double y;
			double z;
		}position;

		struct Velocity  // m /s
		{
			double vx;
			double vy;
			double vz;
		}velocity;

	}local_position;

	struct GlobalPosition
	{
		double lattitude;
		double longtitude;
		double gps_height;

		long int time_stamp;
		bool updated;

	}global_position;

	struct AngluarSetpoint
	{
		bool usingAPI = false;
		float pitch;
		float roll;
		float yaw_rate;
		float throttle;
	}angluar_setpoint;

	struct LocalPositionSetpoint
	{
		struct Position  // m
		{
			double x;
			double y;
			double z;
		}position;

		struct Velocity  // m /s
		{
			double vx;
			double vy;
			double vz;
		}velocity;

	}local_position_setpoint;

	struct TestValue
	{
		double test1;
		double test2;
		double test3;
		double gyo[3];
		double acc[3];
		double mag[3];
		double gps_n, gps_e, gps_h;
		double baro;
	}test_value;
};

