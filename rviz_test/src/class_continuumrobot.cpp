/**
 * @file class_continuumrobot.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-03-31
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */


#include <rviz_test/class_continuumrobot.h>


continuumrobot::continuumrobot(){
    char Topic_rivz_Backbone[30];

    for(int i=0;i<this->num_seg;i++){
        sprintf(Topic_rivz_Backbone, "Backbone_%d", i);
        this->Marker_Backbone[i].markers.resize(NUM_MAX_POINTS);
        this->Pub_marker[i] = this->nh_.advertise<visualization_msgs::MarkerArray>(Topic_rivz_Backbone,1);
    }
    this->InitMaker();
}

void continuumrobot::updataRobot(VectorXd C){

    this->Configuration = C;    
    if(this->Configuration[0] == 0) Configuration[0]=0.0001;
    if(this->Configuration[1] == 0) Configuration[1]=0.0001;
    if(this->Configuration[2] == 0) Configuration[2]=0.0001;    
    
    this->RenewTransformMatrixs();    
    this->RenewShape();

    for(int i=0;i<this->num_seg;i++){
        this->Pub_marker[i].publish(Marker_Backbone[i]);   
        
    }

}

void continuumrobot::InitMaker(){
    for(int i=0;i<this->num_seg;i++){
        for(int j=0;j<NUM_MAX_POINTS;j++){
            
            //名称属性用于区分或后期查看
            this->Marker_Backbone[i].markers[j].header.frame_id = "Backbone";//这里header更偏向于标题的意义
            this->Marker_Backbone[i].markers[j].header.stamp = ros::Time::now();
            this->Marker_Backbone[i].markers[j].ns = "base_shape";//工作空间
            this->Marker_Backbone[i].markers[j].id = j;

            //增加/删除
            this->Marker_Backbone[i].markers[j].action = visualization_msgs::Marker::ADD;

            //定义姿态四元数的w
            this->Marker_Backbone[i].markers[j].pose.orientation.w = 1.0;
            this->Marker_Backbone[i].markers[j].pose.orientation.x = 0;
            this->Marker_Backbone[i].markers[j].pose.orientation.y = 0;
            this->Marker_Backbone[i].markers[j].pose.orientation.z = 0;


            //类型定义，LINE_STRIP指将各点连成线
            this->Marker_Backbone[i].markers[j].type = visualization_msgs::Marker::LINE_STRIP;

            //定义线的粗细
            this->Marker_Backbone[i].markers[j].scale.x = 0.1*(4-i);

            this->Marker_Backbone[i].markers[j].lifetime = ros::Duration();//????

        }  
    }
    for(int j=0;j<NUM_MAX_POINTS;j++){
            //颜色rgba
            this->Marker_Backbone[0].markers[j].color.r = 1.0;
            this->Marker_Backbone[0].markers[j].color.a = 1.0;
            this->Marker_Backbone[1].markers[j].color.g = 1.0;
            this->Marker_Backbone[1].markers[j].color.a = 1.0;
            this->Marker_Backbone[2].markers[j].color.b = 1.0;
            this->Marker_Backbone[2].markers[j].color.a = 1.0;
    }
}


void continuumrobot::RenewTransformMatrixs(){
    double theta1 = this->Configuration[0];
    double psi1 = this->Configuration[1];
    double theta2 = this->Configuration[2];
    double psi2 = this->Configuration[3];
    double theta3 = this->Configuration[4];
    double psi3 = this->Configuration[5];
    //ROS_INFO_STREAM("start");
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[0]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[1]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[2]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin());
    this->T[1] = this->GetTransformMatrix(theta1,psi1,this->L[0]);
    this->T[2] = this->GetTransformMatrix(theta2,psi2,this->L[1]);
    this->T[2] = this->T[1]*this->T[2];
    this->T[3] = this->GetTransformMatrix(theta3,psi3,this->L[2]);
    this->T[3] = this->T[2]*this->T[3];
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[0]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[1]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin()[2]);
    //ROS_INFO_STREAM("T1" << T[1].getOrigin());
    //ROS_INFO_STREAM("end");//?????

}

tf::Transform continuumrobot::GetTransformMatrix(double theta,double psi,double L){
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

void continuumrobot::RenewShape(){
    tf::Transform T_Temp;
    T_Temp.setIdentity();
    for(int i=1;i<=this->num_seg;i++){
        for(int j=0;j<NUM_MAX_POINTS;j++){
            double theta = this->Configuration[2*i-2]/NUM_MAX_POINTS*j;
            double psi = this->Configuration[2*i-1];
            double L = this->L[i-1]/NUM_MAX_POINTS*j;
            T_Temp = T[i];
            //ROS_INFO_STREAM("T o" << T_Temp.getOrigin());
            T_Temp = T_Temp*this->GetTransformMatrix(theta,psi,L);
            //ROS_INFO_STREAM("theta" << theta);
            //ROS_INFO_STREAM("psi" << psi);
            //ROS_INFO_STREAM("L" << L);
            //ROS_INFO_STREAM("T o" << T_Temp.getOrigin());

            this->Marker_Backbone[i-1].markers[j].pose.position.x = T_Temp.getOrigin()[0];
            this->Marker_Backbone[i-1].markers[j].pose.position.y = T_Temp.getOrigin()[1];
            this->Marker_Backbone[i-1].markers[j].pose.position.z = T_Temp.getOrigin()[2];
            //ROS_INFO_STREAM("position = " << this->Marker_Backbone[i-1].markers[j].pose.position);
        }
    }
}








