/**
 * @file class_continuumrobot.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <continuumrobot_digitaltwin/class_continuumrobot.h>


continuumrobot::continuumrobot(ros::NodeHandle nh) : nh_(nh) {
    bool FLAG;
    FLAG = this->nh_.getParam("structure/length_backbone",*L);
    if(!FLAG){
        ROS_ERROR_STREAM("[class continuumrobot] 参数服务器中获取参数失败，检查yaml配置文件!!!");
    }
    this->Num_Seg = sizeof(this->L)/sizeof(this->L[0]);
}

int continuumrobot::getNum_Seg(){
    return this->Num_Seg;
}

void continuumrobot::updateRobot(Eigen::VectorXd C){
    //避免奇异构型造成雅各比不可计算
    for(int i=0;i<Num_Seg;i++){
        if(!C[2*i]) C[2*i]=1e-8;
    }    
    updateTransform(C);
}

tf::Transform * continuumrobot::getTransform(){
    return this->T;
}

tf::Transform continuumrobot::getTransform_OneSeg(double theta,double psi,double L){
    tf::Transform T,T_Temp;
    double R = L/theta;
    T_Temp.setRotation(tf::createQuaternionFromRPY(0,0,psi));
    T_Temp.setOrigin(tf::Vector3(0,0,0));
    T *= T_Temp;
    T_Temp.setRotation(tf::createQuaternionFromRPY(0,0,0));
    T_Temp.setOrigin(tf::Vector3(R,0,0));
    T *= T_Temp;
    T_Temp.setRotation(tf::createQuaternionFromRPY(0,theta,0));
    T_Temp.setOrigin(tf::Vector3(0,0,0));
    T *= T_Temp;
    T_Temp.setRotation(tf::createQuaternionFromRPY(0,0,0));
    T_Temp.setOrigin(tf::Vector3(-R,0,0));
    T *= T_Temp;
    T_Temp.setRotation(tf::createQuaternionFromRPY(0,0,-psi));
    T_Temp.setOrigin(tf::Vector3(0,0,0));
    T *= T_Temp;

    return T;
}

void continuumrobot::updateTransform(Eigen::VectorXd C){
    //更新各transform
    for(int i=0;i<Num_Seg;i++){
        double theta = C[2*i];
        double psi = C[2*i-1];
        this->T[i] = getTransform_OneSeg(theta,psi,this->L[i]);
        if(i) this->T[i] = this->T[i-1]*this->T[i];//i!=0
    }
}



















