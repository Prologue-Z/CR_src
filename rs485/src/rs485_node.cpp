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

#include <rs485/Class_ForceSensor.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh("~");

    NS_ForceSensor::ForceSensor FS;

    
    return 0;
}