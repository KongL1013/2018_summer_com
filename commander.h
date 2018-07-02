//
// Created by lucasyu on 18-6-22.
//
#include <stdio.h>
#include <iostream>
#include <vector>
#include "type_methode.h"

#ifndef PROJECT_COMMANDER_H
#define PROJECT_COMMANDER_H

#endif //PROJECT_COMMANDER_H
enum FlightState {
	Idle = 0,
	Hovering = 1,
	Circling = 2,
	Funny = 3,
	Landing = 4,
	Automatic = 5,
	TakingOff = 6,
	GoTo = 7
};
enum FlightMode{
    MODE_RAW = 0,
    MODE_POS = 1,
    MODE_TRJ = 2,
};

class Commander
{
public:
    Commander():
            takeoff_objective_height(1.2f)
            ,takeoff_safe_distance(0.0f)
            ,takeoff_low_rate(0.2f)
            ,takeoff_high_rate(0.3f)
            ,reset_takeoff(false)
            ,takeoff_condition(true)
            ,m_landspeed(0.2f)
            ,m_land_switch_idle(0.1f)
            ,m_takeoff_switch_Auto(0.2f)
            ,m_takeoff_switch_Hover(0.2f)
            ,m_hovering_time(0)
            ,m_hover_duration(-1)
            ,m_pos_sp(nullptr)
            ,m_vel_ff(nullptr)
            ,m_acc_sp(nullptr)

    {
        _takeoff_Pos.setZero();
        m_last_rateSp.setZero();
        m_flight_state = Idle;
        m_fl_state_after_hover = Hovering;

    }

    void setFlighState(const FlightState& state);
    void initSps( Eigen::Vector4f& posSp,  Eigen::Vector3f& velSp,  Eigen::Vector3f& accSp);
    void posspReset(const float& x, const float& y, const float& z);
    void yawspReset(const float& yaw);
    void set_hover(const float& hovering_time, const FlightState& next_flight_state);
    void iteration();
    void hover_toujour();
    void command_takeoff(const float& dt);
    float m_yaw_sp;
    void init_sp(Eigen::Vector4f* pos_sp,Eigen::Vector3f* vel_ff,Eigen::Vector3f* acc_sp);
    void set_initPos(const Eigen::Vector3f& initPos);
    void setTakeOffPos(const float& x,const float& y,const float& z);
private:
    int m_id;
    FlightState m_flight_state,m_fl_state_after_hover;
    FlightMode m_flight_mode;
    Eigen::Vector3f* m_curPos;
    bool takeoff_switch, takeoff_condition;
    float takeoff_objective_height, takeoff_safe_distance, takeoff_low_rate, takeoff_high_rate, m_landspeed, m_landsafe;
    bool reset_takeoff;
    float m_takeoff_switch_Auto, m_takeoff_switch_Hover, m_land_switch_idle;
    float m_hovering_time,m_hover_duration;
    bool isFirstCircling, isHovering;

    Eigen::Vector4f* m_pos_sp;
    Eigen::Vector3f* m_vel_ff;
    Eigen::Vector3f* m_acc_sp;
    Eigen::Vector3f _takeoff_Pos, m_last_rateSp, m_hover_pos;

};