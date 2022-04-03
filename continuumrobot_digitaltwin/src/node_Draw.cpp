/**
 * @file node_Draw.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <tf/transform_listener.h>
/*
void transformPoint(const tf::TransformListener& listener){
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  laser_point.header.stamp = ros::Time();

  laser_point.point.x = 0.0;
  laser_point.point.y = 0.0;
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
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle nh;
    bool FLAG;

    int F;
    double *L;
    int Num_Seg;

    FLAG = nh.getParam("structure/length_backbone",*L);
    FLAG *= nh.getParam("rviz/frequency_refresh",F);

    if(!FLAG){
        ROS_ERROR_STREAM("[node_Draw] 参数服务器中获取参数失败，检查yaml配置文件!!!");
        return 0;
    }
    Num_Seg = sizeof(L)/sizeof(L[0]);

    ros::Rate R(F);

    tf::TransformListener listener;

    while(nh.ok()){
        tf::StampedTransform *T;
        
        try{
            char *base = "base_Seg";            
            for(int i=0;i<Num_Seg;i++){
                listener.lookupTransform("base", base+char(i+1),ros::Time(0),T[i]);
            }
        }
        catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
        }

        for(int i=0;i<Num_Seg;i++){
            ROS_INFO_STREAM("[node_Draw]Tx"<<i<<T[i].getOrigin().getX());
            ROS_INFO_STREAM("[node_Draw]Ty"<<i<<T[i].getOrigin().getY());
            ROS_INFO_STREAM("[node_Draw]Tz"<<i<<T[i].getOrigin().getZ());
        }
    }
    return 1;
}