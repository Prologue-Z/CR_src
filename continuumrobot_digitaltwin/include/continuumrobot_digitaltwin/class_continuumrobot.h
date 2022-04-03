/**
 * @file class_continuumrobot.h
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

class continuumrobot{
public:
    /**
     * @brief Construct a new continuumrobot object
     * 
     * @param nh ros句柄，用于类内部调用ros库
     */
    continuumrobot(ros::NodeHandle nh);

    /**
     * @brief Destroy the continuumrobot object
     * 
     */
    ~continuumrobot();

    /**
     * @brief 根据给定构型坐标更新机器人数据
     * 
     * @param C 给定构型坐标
     */
    void updateRobot(Eigen::VectorXd C);
    
    /**
     * @brief 返回各关节末端Transform
     * 
     * @return tf::Transform* 各关节末端Transform
     */
    tf::Transform * getTransform();

    int getNum_Seg();

private:
    int Num_Seg;//关节数量
    double *L;//各关节长度
    tf::Transform *T;//各关节末端的坐标系

    ros::NodeHandle nh_;

    /**
     * @brief 输入一段关节的构型参数，返回末端到这一关节基座的transform
     * 
     * @param theta 圆心角
     * @param psi 相位角
     * @param L backbone长度
     * @return Transform 
     */
    tf::Transform getTransform_OneSeg(double theta,double psi,double L);

    /**
     * @brief 给定构型坐标更新机器人的T
     * 
     * @param C 给定构型坐标
     */
    void updateTransform(Eigen::VectorXd C);
};