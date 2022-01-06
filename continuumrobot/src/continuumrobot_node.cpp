/**
 * @file continuumrobot_node.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-23
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <math.h>
#define PI M_PI
#include <ros/ros.h>
#include "ECanVci.h"
#include "continuumrobot/Class_Motor.h"
#include "continuumrobot/Class_ContinuumRobot.h"
//#include "rs485/Msg_Force.h"

// void CallBack(const rs485::Msg_Force::ConstPtr& Force){
// 	ROS_INFO_STREAM("[CR]Sub get msg.F1 = "<<Force->F1<<"N,F2 = "<<Force->F2<<"N,F3 = "<<Force->F3<<"N.");
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh("~"); 

	int T = 5;
	int F = 10;//10-20

	NS_ContinuumRobot::ContinuumRobot CR;
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

	// double L_D[3] = {0.3,0.3,0.3};
	// CR.ToLength_DrivingWire(L_D,T,F);

	double C_D[2] = {PI/3,0};
	CR.ToConfiguration(C_D,T,F);

	// ros::Rate Collect(1);
	// DWORD dwRel;
	// NS_Motor::Motor Motors;

	// dwRel = Motors.InitMotors();
	// if(dwRel != STATUS_OK){
	// 	return dwRel;
	// }
	// dwRel = Motors.EnableMotors();
	// if(dwRel != STATUS_OK){
	// 	return dwRel;
	// }
	// Motors.SetSpeedMode();
	

	// int flag = 1;
	// while(ros::ok()&&flag<=10){
	// 	double* P = new double[3];
	// 	P = Motors.GetPosition();
	// 	ROS_INFO_STREAM("p1="<<P[0]<<"r, p2="<<P[1]<<"r, p3="<<P[2]<<"r");
	// 	ros::Subscriber sub = nh.subscribe("Topic_Force",1,CallBack);
		
	// 	double Speed[3] = {-60*flag*0.1,-30*flag*0.1,-15*flag*0.1};
	// 	Motors.SetSpeed(Speed);

	// 	flag++;

	// 	ros::spinOnce();
	// 	Collect.sleep();
	// }

	// Motors.CloseMotors();

	return 1;
}
