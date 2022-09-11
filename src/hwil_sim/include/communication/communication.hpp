#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <serial/serial.hpp>

class HWILCommunication
{
private:
    ros::NodeHandle nh_;
    ros::Rate m_rate;

    ros::Publisher p_pwm;
    ros::Subscriber s_imu, s_altSonar, s_pos;

    //Member Variable
    sensor_msgs::Imu m_imu;
    sensor_msgs::Range m_altSonar;
    sensor_msgs::NavSatFix m_pos;
    hector_uav_msgs::MotorPWM m_pwm;

    double m_gyroX, m_gyroY, m_gyroZ, m_accX, m_accY, m_accZ;
    float m_relativeAlt;
    double m_latitude, m_longitude;

    //Member Methods
    void initSub();
    void initPub();

    //Member Methods Callback
    void cb_imu(const sensor_msgs::Imu &data);
    void cb_altSonar(const sensor_msgs::Range &data);
    void cb_pos(const sensor_msgs::NavSatFix &data);


public:
    HWILCommunication(ros::NodeHandle &nodehandle);
    ~HWILCommunication();

    void sendMessage();
    void receiveMessage();

    Serial ser;
    
};