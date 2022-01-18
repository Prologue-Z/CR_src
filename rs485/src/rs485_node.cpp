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
#include "rs485/Msg_Force.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh("~");

    // //test serial
    // serial::Serial ser;

    // try { 
    // //设置串口属性，并打开串口 
    //     ser.setPort("/dev/ttyCH341USB0");
    //     ser.setBaudrate(9600); 
    //     serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
    //     ser.setTimeout(to); 
    //     ser.open(); 
    // } 
    // catch (serial::IOException& e) { 
    //     ROS_ERROR_STREAM("Unable to open port "); 
    //     return -1; 
    // } 
    // if(ser.isOpen()) 
    // { 
    //     ROS_INFO_STREAM("Serial Port initialized"); 
    // } 
    // else{ 
    //     return -1; 
    // }

    // ros::Publisher pub = nh.advertise<rs485::Msg_Force>("Topic_Force",1);
    
    NS_ForceSensor::ForceSensor* FS = new NS_ForceSensor::ForceSensor();
    ros::Rate Collect(1);
    double* ForceShow = new double[3];
    rs485::Msg_Force ForceMsg;
    while (ros::ok())
    {
        ForceShow = FS->GetForce();
        ForceMsg.F1 = ForceShow[0];
        ForceMsg.F2 = ForceShow[1];
        ForceMsg.F3 = ForceShow[2];
        //pub.publish(ForceMsg);
        ROS_INFO_STREAM("[RS485]F1 = "<<ForceShow[0]<<"N  F2 = "<<ForceShow[1]<<"N  F3 = "<<ForceShow[2]<<"N");
        Collect.sleep();
    }    
    
    return 0;
}