#include "ros/ros.h"
#include "std_msgs/String.h"
//话题回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("node_b is receiving [%s]", msg->data.c_str());
}
 
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_b");	//初始化ROS，节点命名为node_b，节点名必须唯一。
	ros::NodeHandle n;	//节点句柄实例化
	ros::Subscriber sub = n.subscribe("str_message", 1000, chatterCallback);	//向话题“str_message”订阅，一旦发布节点（node_a）在该话题上发布消息，本节点就会调用chatterCallbck函数。
 
	ros::spin();	//程序进入循环，直到ros::ok()返回false，进程结束。
 
	return 0;
}