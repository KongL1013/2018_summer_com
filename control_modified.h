#include <math.h>
#include "type_methode.h"
#include "commons.h"
#include "pid.h"
#include "droneInfo.h"

//using namespace Eigen;
class ControlCalculator
{
public:
	ControlCalculator()
	:m_pidX(160.0, 0.5, 2.0, 2.5, 0.0, -1e6, 1e6, -0.3, 0.3) //kp, kd, ki, kpp, ff, minOutput, maxOutput, integratorMin, integratorMax;
	,m_pidY(160.0, 0.5, 2.0, 2.5, 0.0, -1e6, 1e6, -0.3, 0.3)//kp 22 kd 1.8 ki 2.0 kpp 7
	,m_pidZ(170.0, 1.65, 5.0, 5.0, 0.0, -1e6, 1e6, -2, 2)//kpp 3
	{

	 vel_Sp.setZero();
	 acc_IMU_wd.setZero();
	 _acc_Sp_W.setZero();
	 _acc_Sp_B.setZero();

	 _Zb_des.setZero();
	 _Xc_des.setZero();
	 _Xb_des.setZero();
	 _Yb_des.setZero();
	 _R_des.setZero();
	 e_R.setZero();

	 pos_Sp.setZero();


	 acc_Sp_net.setZero();

	 m_pidX.reset();
	 m_pidY.reset();
	 m_pidZ.reset();
	 
	 printf("hello!! control.h\n");
	};

	const float w_Vicon = 1.0f;
	const float w_IMU = 0.0f;
	const float wv_Vicon = 1;
	const float wv_IMU = 0;
	const float m_cutoff_freq = 2.0f; //cutoff f
    const float m_sample_freq = 300.0f; //sampling f
    const float m_fc_gyro = 0.5f;
    const float max_thrust = 0.5827*1.3;
    float Pitch_Sp;
	float Roll_Sp;
protected:
	//att:
	Eigen::Vector3f  e_R;
	Eigen::Vector3f _Zb_des, _Xc_des, _Yb_des, _Xb_des, Thrust_des;
	Eigen::Matrix3f _R_des;

	Eigen::Vector3f vel_Sp, _acc_Sp_W, _acc_Sp_B, acc_IMU_wd, acc_deriv, acc_Sp_net;
	Eigen::Vector3f RPY_des;

	Eigen::Vector3f l_pos, pos_Sp;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;

	void resetposController(Eigen::Vector3f* pos_est)
	{
		l_pos = *pos_est;
	}

public:
    void control(bool isPosControl, const DroneInfo::local_position& pos_est, Eigen::Vector4f& Sp, Eigen::Vector3f& Vel_ff,
                                  Eigen::Vector3f& acc_Sp, Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output);
	
};