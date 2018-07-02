#include "control_modified.h"

void ControlCalculator::control(bool isPosControl, const drone_info.local_position& pos_est, Eigen::Vector4f& Sp, Eigen::Vector3f& Vel_ff,
                                          Eigen::Vector3f& acc_Sp, Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output)
{
//    Eigen::Vector4f* Output;
    if(dt<0.1f)
    {
        _acc_Sp_W = acc_Sp;
        //euler2rotation(&Euler,&R_est);
		if (isPosControl)
		{
			for (int i = 0; i<3; i++) {
				pos_Sp(i) = Sp(i);
			}
			float x_temp_est = pos_est(0);
			float y_temp_est = pos_est(1);
			float z_temp_est = pos_est(2);

			float x_sp = pos_Sp(0);
			float y_sp = pos_Sp(1);
			float z_sp = pos_Sp(2);

			vel_est(0) = (pos_est(0) - l_pos(0)) / dt;
			vel_est(1) = (pos_est(1) - l_pos(1)) / dt;
			vel_est(2) = (pos_est(2) - l_pos(2)) / dt;

			l_pos(0) = x_temp_est;
			l_pos(1) = y_temp_est;
			l_pos(2) = z_temp_est;


			if (Vel_ff[0] == Vel_ff[1] == Vel_ff[2] == 0.0f) //without vel_ff
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
			
		}
        
		float vx_temp_est = vel_est(0);
		float vy_temp_est = vel_est(1);
		float vz_temp_est = vel_est(2);

        float vx_sp = vel_Sp(0);
        float vy_sp = vel_Sp(1);
        float vz_sp = vel_Sp(2);

        l_velsp = vel_Sp;

        _acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
        _acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
        _acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);


        acc_Sp_net = _acc_Sp_W;

        _acc_Sp_W(2) = _acc_Sp_W(2) + GRAVITY/1000.0f * (float)VEHICLE_MASS;


        vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

        for (int i=0; i<3; i++)
            _R_des(i,2) = _Zb_des(i);

        _Xc_des(0) = cos(Sp(3));
        _Xc_des(1) = sin(Sp(3));
        _Xc_des(2) = 0;

        vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
        vec3f_normalize(&_Yb_des);
        vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);

        for (int i=0; i<3; i++)
        {
            _R_des(i,0) = _Xb_des(i);
            _R_des(i,1) = _Yb_des(i);
        }

        rotation2euler(&_R_des,&RPY_des);

       /* x_sp = RPY_des(0);
        y_sp = RPY_des(1);
        z_sp = RPY_des(2);*/

		for(int i=0;i<2;i++){
            (*Output)(i) = RPY_des(i);
        }

        (*Output)(2) = 0;  //yaw rate
        (*Output)(0) = -(*Output)(0);

        Eigen::Vector3f temp;
        temp.setZero();
        for(int i=0;i<3;i++){
            temp(i) = _R_des(i,2);
        }

        float thrust_force = vec3f_dot(&_acc_Sp_W,&temp);

//        thrust_force /= 470.0f;
        thrust_force = std::min(thrust_force,max_thrust);
        //thrust_force = 2500.0f;
        (*Output)(3) = thrust_force;

    }
}