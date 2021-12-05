/*
*  CR_DataCollection_Node.cpp
*
*  Saved on:2021.11.25
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:RS485 to get force sensors signal
*/

//ROS
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ForceSensor_Node");	//init node named ForceSensor_Node
    ros::NodeHandle nh("~");


}