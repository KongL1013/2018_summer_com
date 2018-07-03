#pragma once

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/Common.hpp"
#include <functional>
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <QtCore>  
#include <QMutex>  
#include <QApplication>
#include <QThread>
#include <vector>
#include "type_methode.h"
#include "commons.h"
#include "pid.h"


class Controller :public QThread
{
public:
	explicit Controller(QString name);
	~Controller();

	void run();
	void stop();
	Eigen::Vector3f  control(const Eigen::Vector3f& pos_est, const Eigen::Vector3f& vel_est, Eigen::Vector4f&  posSp,
		Eigen::Vector4f&  velSp, Eigen::Vector3f& Vel_ff,float dt, Eigen::Vector4f* Output);
	void usePosControl()
	{
		isPosControl = true;
	}
	void useVelControl()
	{
		isPosControl = false;
	}
	void givePosSp(const float& x, const float& y, const float& z, const float& yaw)
	{
		m_posSp(0) = x;
		m_posSp(1) = y;
		m_posSp(2) = z;
		m_posSp(3) = yaw;
		usePosControl();
		m_velSp.setZero();
	}
	void giveVelSp(const float& vx, const float& vy, const float& vz, const float& yaw)
	{
		m_velSp(0) = vx;
		m_velSp(1) = vy;
		m_velSp(2) = vz;
		m_velSp(3) = yaw;
		useVelControl();
	}
	const float max_thrust = 0.5827*1.3;  //to modify 
private:
	bool b_stopped, isPosControl;//是否用位置控制
	QMutex m_mutex;
	Eigen::Vector3f _Zb_des, _Xc_des, _Yb_des, _Xb_des;
	Eigen::Matrix3f _R_des;

	Eigen::Vector3f vel_Sp, _acc_Sp_W, _acc_Sp_B, vel_est, acc_IMU_wd;
	Eigen::Vector3f RPY_des;
	Eigen::Vector4f m_posSp, m_velSp;
	Eigen::Vector3f pos_Sp;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;


};