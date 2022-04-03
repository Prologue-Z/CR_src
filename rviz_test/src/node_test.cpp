/**
 * @file node_test.cpp
 * @brief 测试rviz显示连续体机器人
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-03-31
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <rviz_test/class_continuumrobot.h>
/*
int main( int argc, char** argv){
    ros::init(argc, argv, "node_test");

    continuumrobot CR;

    ros::Rate F(30);

    VectorXd C(6);
    C << M_PI/3,M_PI/2,
         M_PI/4,M_PI/3,
         M_PI/2,M_PI/4;
    VectorXd C1(6);
    C1<<0,0,0,0,0,0;

    while(ros::ok()){
        CR.updataRobot(C1);
        ROS_INFO_STREAM("updata successfully");
        F.sleep();
    }    

    //return 1;

}
*/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 0;
  laser_point.point.y = 0;
  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "node_test");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
