#include <ros/ros.h>
#include "msgs_continuumrobot/Msg_Force.h"

#include "std_msgs/String.h"

void CallBack(const msgs_continuumrobot::Msg_Force::ConstPtr& Forcemsg){
	ROS_INFO("I heard: [F = %f]", Forcemsg->F1);
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("node_b is receiving [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh; 

	//ros::Subscriber Sub_Force = nh.subscribe("Topic_Test",1000,chatterCallback);
    ros::Subscriber Sub_Force = nh.subscribe("Topic_Force",1000,CallBack);
	ROS_INFO_STREAM("[test] sub success??");
	ros::spin();

	
	return 1;
}
