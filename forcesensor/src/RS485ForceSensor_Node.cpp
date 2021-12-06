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

//
#include <forcesensor/RS485_ForceSensor.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ForceSensor_Node");	//init node named ForceSensor_Node
    ros::NodeHandle nh("~");

    ros::Rate Force_T(1);//hz

    ForceSensor_NS::RS485_ForceSensor *FS = new ForceSensor_NS::RS485_ForceSensor();
    double* ForceShow;
    while(1){        
        ForceShow = FS->GetForce();
        ROS_INFO("[ForceSensor]:%lf %lf %lf",ForceShow[0],ForceShow[1],ForceShow[2]);
        Force_T.sleep();
    }
    return 0;
}
