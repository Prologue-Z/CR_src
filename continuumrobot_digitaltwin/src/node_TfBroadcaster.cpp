/**
 * @file node_TfBroadcaster.cpp
 * @brief 广播连续体机器人的各Transform
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <continuumrobot_digitaltwin/class_continuumrobot.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "node_TfBroadcaster");
    ros::NodeHandle nh;
    int F;
    nh.getParam("rviz/frequency_refresh",F);
    ros::Rate r(F);

    continuumrobot CR(nh);
    int num_seg = CR.getNum_Seg();

    Eigen::VectorXd C(6);
    C << 0,0,0,0,0,0;
    CR.updateRobot(C);

    tf::TransformBroadcaster broadcaster[num_seg];
    while(nh.ok()){
        tf::Transform *T = CR.getTransform();
        char *base = "base_Seg";
        for(int i=0;i<num_seg;i++){            
            broadcaster[i].sendTransform(
                tf::StampedTransform(T[i],ros::Time::now(),"base", base+char(i+1))
            );
        }
        r.sleep();
    }
}