/**
 * @file Node_ForceSensor.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include <serial/serial.h>

#include "rs485/Class_ForceSensor.h"
#include "msgs_continuumrobot/Msg_Force.h"

#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<msgs_continuumrobot::Msg_Force>("Topic_Force",1000);
    //ros::Publisher pub = nh.advertise<std_msgs::String>("Topic_Test",1000);
    
    NS_ForceSensor::ForceSensor* FS = new NS_ForceSensor::ForceSensor();
    ros::Rate Collect(30);
    double* ForceShow = new double[3];    

    while (ros::ok()){
	    msgs_continuumrobot::Msg_Force ForceMsg;	
        ForceShow = FS->GetForce();
        ForceMsg.F1 = ForceShow[0];
        ForceMsg.F2 = ForceShow[1];
        ForceMsg.F3 = ForceShow[2];
        pub.publish(ForceMsg);

        // std_msgs::String msg;
        // std::stringstream ss;
        // ss<<"hello";
        // msg.data = ss.str();
        // pub.publish(msg);
        ROS_INFO_STREAM("[RS485]F1 = "<<ForceShow[0]<<"N  F2 = "<<ForceShow[1]<<"N  F3 = "<<ForceShow[2]<<"N");
        Collect.sleep();
    }    
    
    return 0;
}
