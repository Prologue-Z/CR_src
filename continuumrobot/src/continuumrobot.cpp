/**
 * @file continuumrobot.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-23
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include "ECanVci.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh("~");    

    int nDeviceType = 3;
	int nDeviceInd = 0;
	int nReserved = 0;    
	DWORD dwRel; 
	 
	dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved); 
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail open dev"); 
	    return -1; 
	} 
	ROS_INFO("open succ");   


    
	CloseDevice(nDeviceType, nDeviceInd);

}
