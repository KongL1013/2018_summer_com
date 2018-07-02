#include "commander.h"

const float max_thrust = 0.5827*1.3;

#define _USE_MATH_DEFINES //PI

//MODE_RAW 0
//MODE_POS 1
//MODE_TRJ 2

void Commander::posspReset(const float& x, const float& y, const float& z)
{
    (*m_pos_sp)[0] = x;
    (*m_pos_sp)[1] = y;
    (*m_pos_sp)[2] = z;

    (*m_vel_ff)[0] = 0.0f;
    (*m_vel_ff)[1] = 0.0f;
    (*m_vel_ff)[2] = 0.0f;
    m_last_rateSp.setZero();

    (*m_acc_sp)[0] = 0.0f;
    (*m_acc_sp)[1] = 0.0f;
    (*m_acc_sp)[2] = 0.0f;
}
void Commander::initSps( Eigen::Vector4f& posSp,  Eigen::Vector3f& velSp,  Eigen::Vector3f& accSp)
{
    m_pos_sp = &posSp;
    m_vel_ff = &velSp;
    m_acc_sp = &accSp;
}
void Commander::yawspReset(const float& yaw)
{
    (*m_pos_sp)[3] = yaw;
}
void Commander::set_hover(const float& hovering_time, const FlightState & next_flight_state)
{
    m_hover_duration = hovering_time;
    isHovering = true;
    m_flight_state = Hovering;
    m_fl_state_after_hover = next_flight_state;
}
void Commander::init_sp(Eigen::Vector4f* pos_sp,Eigen::Vector3f* vel_ff,Eigen::Vector3f* acc_sp)
{
    m_pos_sp = pos_sp;
    m_vel_ff = vel_ff;
    m_acc_sp = acc_sp;
}
void Commander::setTakeOffPos(const float& x,const float& y,const float& z)
{
    _takeoff_Pos[0] = x;
    _takeoff_Pos[1] = y;
    _takeoff_Pos[2] = z;
}
void Commander::iteration()
{
    static float time_elapse = 0;
    float _dt_deriv = 0.01f;
    switch(m_flight_mode){

        case MODE_POS:{
            switch(m_flight_state){
                case Idle:{
                    //all motors off
                    break;
                }
                case Automatic:{

                    break;
                }
                case TakingOff:{
                    printf("takeof!\n");
                            command_takeoff(_dt_deriv);
                            if((*m_curPos)(2) > takeoff_objective_height - m_takeoff_switch_Hover)
                            {
                                set_hover(-1,Hovering);
                            }

                    break;
                }
                case Landing:{
//                        control_landing(_dt_deriv);
                    break;
                }

                case Hovering:
                {
                    if(m_hover_duration!=-1) // hover for a given duration
                    {
                        if(m_hovering_time>=m_hover_duration)
                        {
                            m_flight_state = m_fl_state_after_hover;
                            m_hovering_time = 0.0f;
                            m_hover_duration = -1;
                        }
                        posspReset( m_hover_pos(0),m_hover_pos(1),m_hover_pos(2));

                        m_hovering_time += _dt_deriv;
                    }
                    else{ //hover without a change of state
                        posspReset(m_hover_pos(0),m_hover_pos(1),m_hover_pos(2));
                    }
                    break;
                }

                case Circling:{
                    printf("begin circling!!\n");
                        //command_circling(_dt_deriv);
                    break;
                }
                case Funny:{
                    break;
                }
            }//end switch state
        }//end case posctrl mode
            break;
        case MODE_TRJ:{
        }
            break;
        default:
            break;
    } //end switch mode

}
void Commander::hover_toujour()
{
    isHovering = true;
    m_hover_duration = -1;
    m_flight_state = Hovering;
}

void Commander::setFlighState(const FlightState& state)
{
    m_flight_state = state;
}

void Commander::command_takeoff(const float& dt) {
    reset_takeoff = false;
    (*m_pos_sp)[0] = _takeoff_Pos(0);
    (*m_pos_sp)[1] = _takeoff_Pos(1);
    yawspReset(0.0f);

    if ((*m_pos_sp)[2] >= _takeoff_Pos(2) + takeoff_objective_height) {
        takeoff_switch = false;

        (*m_pos_sp)[2] = _takeoff_Pos(2) + takeoff_objective_height;
    } else {
        if (takeoff_condition) //in case of a sudden thrust at the beggining..
        {
            (*m_pos_sp)[2] = _takeoff_Pos(2) - takeoff_safe_distance;
            takeoff_condition = false;
        } else {
            if ((*m_pos_sp)[2] < _takeoff_Pos[2] + 0.2f) {
                (*m_pos_sp)[2] += takeoff_low_rate * dt;

                (*m_vel_ff)[2] = takeoff_low_rate;

                (*m_acc_sp)[2] = deriv_f(takeoff_low_rate, m_last_rateSp(2), dt);
                m_last_rateSp(2) = takeoff_low_rate;
            } else {
                (*m_pos_sp)[2] += takeoff_high_rate * dt;

                (*m_vel_ff)[2] = takeoff_high_rate;

                (*m_acc_sp)[2] = deriv_f(takeoff_high_rate, m_last_rateSp(2), dt);
                m_last_rateSp(2) = takeoff_high_rate;
            }
        }
    }
}