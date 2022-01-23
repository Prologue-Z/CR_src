/**
 * @file continuumrobot_node.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-23
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <signal.h>
#include <math.h>
#define PI M_PI
#include <ros/ros.h>
#include "ECanVci.h"

#include "continuumrobot/Class_ContinuumRobot.h"

void MySigintHandler(int sig)
{
	ROS_ERROR_STREAM("[continuumrobot] ros node shutting down!");
	ros::shutdown();
}
int main(int argc, char **argv){
    ros::init(argc, argv, "datacollection_node");
	signal(SIGINT, MySigintHandler);
    ros::NodeHandle nh; 

	NS_ContinuumRobot::ContinuumRobot CR(nh);
        
	int flag = CR.InitRobot();
	if(flag == 0){
		ROS_ERROR_STREAM("[continuuumrobot_node] init robot fail");
		return 0;
	}

	flag = CR.ClearMotorPosition();
	if(flag == 0){
		ROS_ERROR_STREAM("[continuuumrobot_node] clear position of motors fail");
		return 0;
	}


	//data collection control
	int Num_Data = 300;
	CR.DataCollection(Num_Data);
	
	return 1;
}


