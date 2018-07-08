#include "positionController.h"
#include <iostream>
#include "qout.h"

using namespace std;
using namespace msr::airlib;

extern DroneInfo drone_info;
extern msr::airlib::MultirotorRpcLibClient client;


Controller::Controller(QString name) :
	  b_stopped(false)
	//, isPosControl(true)
	, m_mode(HOVER)
	, m_pidX(1.7f, 0.03, 0.07, 1.f, 0.0, -2.f, 5.f, -0.3, 0.3) //kp, kd, ki, kpp, ff, minOutput, maxOutput, integratorMin, integratorMax;
	, m_pidY(1.7f, 0.03, 0.07, 1.f, 0.0, -2.f, 5.f, -0.3, 0.3)//kp 22 kd 1.8 ki 2.0 kpp 7
	, m_pidZ(2.f, 0.01, 0.05, 1.f, 0.0, -1.f, 10.f, -2, 2)//kpp 3
	{
		pos_Sp.setZero();
		vel_Sp.setZero();
		vel_est.setZero();

		acc_IMU_wd.setZero();
		_acc_Sp_W.setZero();
		_acc_Sp_B.setZero();

		_Zb_des.setZero();
		_Xc_des.setZero();
		_Xb_des.setZero();
		_Yb_des.setZero();
		_R_des.setZero();

		m_pidX.reset();
		m_pidY.reset();
		m_pidZ.reset();

		printf("hello!! control.h\n");

}

Controller::~Controller()
{
	stop();
	quit();
	wait();
}

void Controller::stop()
{
	QMutexLocker stop_locker(&m_mutex);
	b_stopped = true;
}

void Controller::run()
{
	
	Eigen::Vector3f posEst, velEst, vel_ff, posDiff;
	Eigen::Vector4f Output, posSptmp, velSptmp;
	vel_ff.setZero();
	Output.setZero();
	posSptmp.setZero();
	velSptmp.setZero();
	posDiff.setZero();
	m_duration = 4.f;
	int control_cnt = 0;
	float control_thres = 0.1;

	{
		QMutexLocker data_locker(&drone_info.data_mutex);
		drone_info.angluar_setpoint.usingAPI = true;

	}
	
	while (true)
	{
		msleep(20); // 50Hz Max
		/* Add your controller code here */
		// Create temperary variables to continue data process and then give the value to "drone_info"
		// Don't forget to add lock
		// e.g.
		
		{
			QMutexLocker data_locker(&drone_info.data_mutex);
			droneInfo2Vector3f(drone_info, &posEst, &velEst);  //类型转换，得到Eigen向量方便运算
		}
			
		//if (control_cnt == 0)
		//{
		//	for (int i = 0; i < 3; ++i) {
		//		posSptmp[i] = m_posSp[i];
		//	}
		//	posDiff = vec3f_minus(&posSptmp, &posEst);
		//	vec3f_devide_number(&posDiff, m_duration);	//控制点之间的间隔
		//	for (int i = 0; i < 3; ++i) {
		//		posSptmp[i] = posEst[i];
		//	}
		//}

		if (vec3f_length(&posDiff) <= control_thres)  //接近（认为到达）目标，hover
		{
			/*hover*/
		}
		if (m_mode == ATTCTRL)             //姿态控制接口
		{
			client.moveByAngleThrottle(m_pitch, m_roll, m_throttle, m_yaw_rate, m_duration);
		}
		else if(m_mode == HOVER)		   //悬停接口
		{
			client.hover();
		}	
		else {							  //位置、速度控制接口
			std::vector<Eigen::Vector3f> rvel_acc;
			rvel_acc = control(posEst, velEst, m_posSp, m_velSp, vel_ff, 0.02f, &Output); //控制过程
			control_cnt++;
			/*控制器输出结果*/

			m_roll = Output(0);
			m_pitch = Output(1);
			m_yaw_rate = 0.0f;
			m_throttle = Output(3);

			/*show_string("x- p\t" + QString::number(posEst(0)) + "\tv " + QString::number(velEst(0)) + "\tth " +  QString::number(throttle) + "\trs " + QString::number(m_roll) + "\tyaws " + QString::number(m_posSp(3)) + "\n");
			show_string("y- p\t" + QString::number(posEst(1)) + "\tv " + QString::number(velEst(1)) + "\tth " +  QString::number(throttle) + "\tps " + QString::number(m_pitch) + "\n");
			show_string("z- p\t" + QString::number(posEst(2)) + "\tv " + QString::number(velEst(2)) + "\tth " +  QString::number(throttle) + "\tys " + QString::number(yaw_rate) + "\n");*/


			show_string(QString::number(posEst(0)) + "\t" + QString::number(posEst(1)) + "\t" + QString::number(posEst(2)) + "\n");

			client.moveByAngleThrottle(m_pitch, m_roll, m_throttle, m_yaw_rate, m_duration);

			/* Assign values */
			{
				QMutexLocker data_locker(&drone_info.data_mutex);
				drone_info.angluar_setpoint.pitch = m_pitch;
				drone_info.angluar_setpoint.roll = m_roll;
				drone_info.angluar_setpoint.yaw_rate = 0.0f;
				drone_info.angluar_setpoint.throttle = m_throttle;

			}
		}
		
		/* Stop watch dog */
		{
			QMutexLocker stop_locker(&m_mutex);
			if (b_stopped)
				break;
		}
	}
}

std::vector<Eigen::Vector3f> Controller::control(const Eigen::Vector3f& pos_est, const Eigen::Vector3f& vel_est, Eigen::Vector4f&  posSp,
	Eigen::Vector4f&  velSp, Eigen::Vector3f& Vel_ff, float dt, Eigen::Vector4f* Output)
{
	//    Eigen::Vector4f* Output;
	if (dt<0.1f)
	{
		float x_temp_est = pos_est(0);
		float y_temp_est = pos_est(1);
		float z_temp_est = pos_est(2);

		float x_sp = pos_Sp(0);
		float y_sp = pos_Sp(1);
		float z_sp = pos_Sp(2);
		switch (m_mode)
		{
		case POSCTRL:							//位置控制接口 
			for (int i = 0; i<3; i++) {
				pos_Sp(i) = posSp(i);
			}
			
			if (Vel_ff[0] == 0.0f && Vel_ff[1] == 0.0f && Vel_ff[2] == 0.0f) //without vel_ff
			{
				vel_Sp(0) = m_pidX.pp_update(x_temp_est, x_sp);
				vel_Sp(1) = m_pidY.pp_update(y_temp_est, y_sp);
				vel_Sp(2) = m_pidZ.pp_update(z_temp_est, z_sp);
			}
			else {											//with vel_ff
				vel_Sp(0) = m_pidX.pp_update(x_temp_est, x_sp)*0.7f + Vel_ff(0)*0.3f; //+ff
				vel_Sp(1) = m_pidY.pp_update(y_temp_est, y_sp)*0.7f + Vel_ff(1)*0.3f;
				vel_Sp(2) = m_pidZ.pp_update(z_temp_est, z_sp)*0.7f + Vel_ff(2)*0.3f;
			}
			break;
		case VELCTRL:										//速度控制接口
			for (int i = 0; i < 3; i++) {
				vel_Sp(i) = velSp(i);
			}
			break;
		default:
			
			break;
		}
		float vx_temp_est = vel_est(0);
		float vy_temp_est = vel_est(1);
		float vz_temp_est = vel_est(2);

		float vx_sp = vel_Sp(0);
		float vy_sp = vel_Sp(1);
		float vz_sp = vel_Sp(2);

		_acc_Sp_W(0) = m_pidX.pid_update(vx_temp_est, vel_Sp(0), dt);
		_acc_Sp_W(1) = m_pidY.pid_update(vy_temp_est, vel_Sp(1), dt);
		_acc_Sp_W(2) = m_pidZ.pid_update(vz_temp_est, vel_Sp(2), dt);
		_acc_Sp_W(2) = _acc_Sp_W(2) - GRAVITY / 1000.0f;// *(float)VEHICLE_MASS;

		vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

		for (int i = 0; i<3; i++)
			_R_des(i, 2) = _Zb_des(i);

		_Xc_des(0) = cos(posSp(3));
		_Xc_des(1) = sin(posSp(3));

		_Xc_des(2) = 0;

		vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
		vec3f_normalize(&_Yb_des);
		vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);

		for (int i = 0; i<3; i++)
		{
			_R_des(i, 0) = _Xb_des(i);
			_R_des(i, 1) = _Yb_des(i);
		}

		rotation2euler(&_R_des, &RPY_des);

		for (int i = 0; i<2; i++) {
			(*Output)(i) = RPY_des(i);
		}

		(*Output)(2) = 0.0f;  //yaw rate

							  //(*Output)(0) = -(*Output)(0);

		Eigen::Vector3f temp;
		temp.setZero();
		for (int i = 0; i<3; i++) {
			temp(i) = _R_des(i, 2);
		}

		//_acc_Sp_W(2)
		float thrust_force = vec3f_dot(&_acc_Sp_W, &temp)*VEHICLE_MASS / 1000.f;

		thrust_force = std::min(thrust_force, max_thrust);
		(*Output)(3) = thrust_force;
	}
	std::vector<Eigen::Vector3f> res;
	res.push_back(vel_Sp);
	res.push_back(_acc_Sp_W);
	return res;
}