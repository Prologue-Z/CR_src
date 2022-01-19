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
//#include "continuumrobot/Class_Motor.h"
#include "continuumrobot/Class_ContinuumRobot.h"
//#include "rs485/Msg_Force.h"

// void CallBack(const rs485::Msg_Force::ConstPtr& Force){
// 	ROS_INFO_STREAM("[CR]Sub get msg.F1 = "<<Force->F1<<"N,F2 = "<<Force->F2<<"N,F3 = "<<Force->F3<<"N.");
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh("~"); 

	NS_ContinuumRobot::ContinuumRobot CR(nh);
	ros::Subscriber Sub_Force = nh.subscribe<msgs_continuumrobot::Msg_Force>("Topic_Force",1,&NS_ContinuumRobot::ContinuumRobot::Force_CallBack,&CR);;
        
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
	

	//CR.To0Position();

	//data collection control
	int Num_Data = 300;
	CR.DataCollection(Num_Data);

	return 1;
}
