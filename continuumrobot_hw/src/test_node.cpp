/**
 * @file test_node.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <continuumrobot_hw/Kinematic_CX.h>
#include <ros/ros.h>

template<typename _Matrix_Type_> 
_Matrix_Type_ pinv(const _Matrix_Type_ &a, double epsilon = 
    std::numeric_limits<double>::epsilon()){  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh; 


    double L[3] = {0.28,0.28,0.28};
    Kinematic_CX K_CX(L);
/*
    MatrixXd Mt(4,5);
    MatrixXd Mp(5,4);

    Mt << 1,2,3,4,1,
          5,6,7,8,1,
          9,8,7,6,1,
          5,4,3,2,1;
    Mp << K_CX.pinv(Mt);
    ROS_INFO_STREAM("Mp = "<< Mp);

*/

    VectorXd C1(6);
    C1 << 1.2,1.8,1.2,1.6,1.6,0.8;
    VectorXd C_S(6);
    C_S = 0.9*C1;

    VectorXd X1 = K_CX.GetX(C1);
    ROS_INFO_STREAM("X1 = "<< X1);

    VectorXd C2 = K_CX.GetConfiguration(X1,C_S);
    ROS_INFO_STREAM("C2 = "<< C2);
    ROS_INFO_STREAM("X2 = "<< K_CX.GetX(C2));
    return 1;
}