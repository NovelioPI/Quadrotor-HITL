#include <ros/ros.h>
#include <communication/communication.hpp>
#include "nanopb/pb_encode.h"
#include "nanopb/pb_decode.h"
#include "nanopb/hwil.pb.h"

const int RATE = 500;

HWILCommunication::HWILCommunication(ros::NodeHandle &nodehandle)
    :nh_(nodehandle), m_rate(RATE), ser("/dev/ttyACM0", 115200)
{
    ROS_INFO("Instantiating Communication");
    static ros::AsyncSpinner spinner(0);
    spinner.start();

    initSub();
    
    while(ros::ok && !ser.isOpen()){
        ROS_INFO("CONNECTING TO MCU.......");
    }

    initPub();
    ROS_INFO("Controller instantiated!");
}

HWILCommunication::~HWILCommunication()
{
    ros::shutdown();
    ROS_INFO("Shutting down controller...");
}

void HWILCommunication::cb_imu(const sensor_msgs::Imu &data)
{
    m_gyroX = data.angular_velocity.x;
    m_gyroY = data.angular_velocity.y;
    m_gyroZ = data.angular_velocity.z;

    m_accX = data.linear_acceleration.x;
    m_accY = data.linear_acceleration.y;
    m_accZ = data.linear_acceleration.z;
}

void HWILCommunication::cb_altSonar(const sensor_msgs::Range &data)
{
    m_relativeAlt = data.range;
}

void HWILCommunication::cb_pos(const sensor_msgs::NavSatFix &data)
{
    m_latitude = data.latitude;
    m_longitude = data.longitude;
}

void HWILCommunication::initSub()
{
    ROS_INFO("Initializing Subscribers!");
    s_imu = nh_.subscribe("/raw_imu", 1, &HWILCommunication::cb_imu, this);
    s_altSonar = nh_.subscribe("/sonar_height", 1, &HWILCommunication::cb_altSonar, this);
    s_pos = nh_.subscribe("/fix", 1, &HWILCommunication::cb_pos, this);
}

void HWILCommunication::initPub()
{
    ROS_INFO("Initializing Publisher");
    p_pwm = nh_.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 1);
}


void HWILCommunication::sendMessage()
{
    uint8_t buffer[100];
    HWIL msg = HWIL_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    msg.gyroX = m_gyroX;
    msg.gyroY = m_gyroY;
    msg.gyroZ = m_gyroZ;
    msg.accX = m_accX;
    msg.accY = m_accY;
    msg.accZ = m_accZ;
    msg.relativeAlt = m_relativeAlt;
    msg.latitude = m_latitude;
    msg.longitude = m_longitude;

    msg.has_gyroX = true;
    msg.has_gyroY = true;
    msg.has_gyroZ = true;
    msg.has_accX = true;
    msg.has_accY = true;
    msg.has_accZ = true;
    msg.has_relativeAlt = true;
    msg.has_latitude = true;
    msg.has_longitude = true;

    bool status = pb_encode(&stream, HWIL_fields, &msg);
    size_t message_length = stream.bytes_written;

    if (!status)
    {
        ROS_ERROR("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }
    
    unsigned char c = '&';
    ser.sendChar(c);

    c = message_length;
    ser.sendChar(c);

    for(uint8_t i=0; i<message_length; i++){
        ser.sendChar(buffer[i]);
    }

    receiveMessage();
}

void HWILCommunication::receiveMessage()
{
    uint8_t buffer[100];
    HWIL msg = HWIL_init_zero;
    
    size_t message_length = ser.readData(buffer, sizeof(buffer));

    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);

    bool status = pb_decode(&stream, HWIL_fields, &msg);

    if (!status)
    {
        ROS_ERROR("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        return;
    }
}