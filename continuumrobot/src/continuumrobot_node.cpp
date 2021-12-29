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
#include "continuumrobot/Class_Motor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh("~"); 

	DWORD dwRel;
	NS_Motor::Motor Motors;
	dwRel = Motors.InitMotors();
	if(dwRel != STATUS_OK){
		return dwRel;
	}

	dwRel = Motors.EnableMotors();
	if(dwRel != STATUS_OK){
		return dwRel;
	}
	Motors.SetSpeedMode();
	double Speed[3] = {-60,-30,-15};
	Motors.SetSpeed(Speed);
	sleep(10);
	for(int i=0;i<3;i++){Speed[i] = 0;}
	Motors.SetSpeed(Speed);
	double* P = new double[3];
	P = Motors.GetPosition();
	ROS_INFO_STREAM("p1="<<P[0]<<"r, p2="<<P[1]<<"r, p3="<<P[2]<<"r");
	Motors.CloseMotors();

	return 1;
}
