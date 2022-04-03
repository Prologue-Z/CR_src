/**
 * @file class_continuumrobot.h
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-03-31
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#define NUM_MAX_POINTS 200//rviz上每段的点的数量


class continuumrobot{
    public:
    continuumrobot();

    void updataRobot(VectorXd C);

    private:
    const int num_seg = 3;//关节数量
    const double L[3] = {0.28,0.28,0.28};//各关节长度

    VectorXd Configuration;//构型空间坐标

    ros::NodeHandle nh_;
    ros::Publisher Pub_marker[3];
    visualization_msgs::MarkerArray Marker_Backbone[3];

    tf::Transform T[4];//基坐标和三个关节末端与基坐标系的变换


    void InitMaker();

    void RenewTransformMatrixs();

    void RenewShape();

    tf::Transform GetTransformMatrix(double theta,double psi,double L);


};

