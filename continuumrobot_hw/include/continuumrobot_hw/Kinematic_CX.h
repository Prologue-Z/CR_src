/**
 * @file Kinematic_CX.h
 * @brief 连续体机器人运动学：构型空间和末端工作空间
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-03-29
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <eigen3/Eigen/Dense>
using namespace Eigen;

class Kinematic_CX{
    public:
    /**
     * @brief Construct a new Kinematic_CX object
     * 
     * @param L 各关节backbone长度
     */
    Kinematic_CX(double L[3]);

    /**
     * @brief 输入构型空间坐标，返回末端位姿
     * 
     * @param Configuration 构型空间坐标-[theta1;psi1;theta2;psi2;theta3;psi3]
     * @return VectorXd 末端位姿-9维向量，齐次变换矩阵内元素组成 [T(1,1);T(2,1);T(3,1);T(1,2);T(2,2);T(3,2);T(1,4);T(2,4);T(3,4);]
     */
    VectorXd GetX(VectorXd Configuration);

    /**
     * @brief 输入末端位姿，返回构型空间坐标
     * 
     * @param X 末端位姿-定义同GetX()
     * @param Configuration_Start 迭代初始值
     * @return VectorXd 构型空间坐标-定义同GetX()
     */
    VectorXd GetConfiguration(VectorXd X,VectorXd Configuration_Start);


    private:
    //各关节长度
    double L1;
    double L2;
    double L3;

    int num_max_iteration = 2000;//逆运动学最大迭代次数

    /**
     * @brief 输入构型空间坐标，获得对应齐次变换矩阵
     * 
     * @param Configuration 构型空间坐标-定义同GetX()
     * @return Matrix4d 齐次变换矩阵
     */
    Matrix4d GetTransformMatrix(VectorXd Configuration);

    /**
     * @brief 输入构型空间坐标，获得对应雅各比矩阵
     * 
     * @param Configuration 构型空间坐标-定义同GetX()
     * @return Matrix<double,9,6> 雅各比矩阵
     */
    Matrix<double,9,6> GetJacobianCX(VectorXd Configuration);


    /**
     * @brief 求矩阵的伪逆矩阵
     * @ref https://www.cnblogs.com/wxl845235800/p/8892681.html
     * 
     * @param J 
     * @return MatrixXd 
     */
    MatrixXd pinv(const MatrixXd a);


};