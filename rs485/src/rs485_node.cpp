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
    
    NS_ForceSensor::ForceSensor* FS = new NS_ForceSensor::ForceSensor();
    ros::Rate Collect(1);
    double* ForceShow = new double[3];
    while (1)
    {
        ForceShow = FS->GetForce();
        ROS_INFO_STREAM("[RS485]F1 = "<<ForceShow[0]<<"N  F2 = "<<ForceShow[1]<<"N  F3 = "<<ForceShow[2]<<"N");
        Collect.sleep();
    }    
    
    return 0;
}