#include <ros/ros.h>
#include <communication/communication.hpp>

const float RATE = 500;

void startSimulation(HWILCommunication& communication)
{
    ros::Rate rate(RATE);

    while(ros::ok())
    {
        communication.sendMessage();
        rate.sleep();
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "communication_node");
    ros::NodeHandle nh;
    HWILCommunication communication(nh);

    startSimulation(communication);

    return 0;
}