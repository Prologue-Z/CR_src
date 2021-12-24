/**
 * @file continuumrobot_node.cpp
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

	DWORD dwRel;
    int nDeviceType = 3;
	int nDeviceInd = 0;
	int nReserved = 0;
	int nCANInd =0;
	
	//open 
	dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved); 
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail open dev"); 
	    return -1; 
	} 
	ROS_INFO("open dev succ"); 

	//init
	INIT_CONFIG init_config;
	init_config.AccCode = 0; 
	init_config.AccMask =0xffffff; 
	init_config.Filter = 0;
	init_config.Timing0 = 0; 
	init_config.Timing1 = 0x1c; 
	init_config.Mode = 0;
	dwRel = InitCAN(nDeviceType, nDeviceInd,nCANInd,&init_config);
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail init dev"); 
		CloseDevice(nDeviceType, nDeviceInd);
	    return -1; 
	}
	ROS_INFO("init dev succ"); 

	//star
	dwRel = StartCAN(nDeviceType, nDeviceInd, nCANInd);  
	if (dwRel == STATUS_ERR){  
		ROS_ERROR("fail start can");
		CloseDevice(nDeviceType, nDeviceInd);  
		return -1; 
	}
	ROS_INFO("start can succ"); 
    
	CloseDevice(nDeviceType, nDeviceInd);
}
