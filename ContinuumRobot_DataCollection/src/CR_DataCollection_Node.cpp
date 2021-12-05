/*
*  CR_DataCollection_Node.cpp
*
*  Saved on:2021.11.18
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:Random motion to collect motion parameters
*      SI:m,Hz
*/

//ROS
#include <ros/ros.h>

//Continuum Robot
#include <ContinuumRobot_DataCollection/ContinuumRobot.h>

//Eign3
#include <eigen3/Eigen/Dense>
using namespace Eigen;

//CANMotor
#include <CANMotor/CANMotor.h>

//Function
Vector2d RandConfiguration();//Random position in configuration space

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CRKinematicControl_Node");	//init node named CANMotor_Node (unique)
    ros::NodeHandle nh("~");

    int Collect_Num = 1000;//Data number

    double Backbone_Length = 0.3;//length of backbone;
    double Radius = 0.015;//distribution radius driving wires 

    CANMotor_NS::CANMotor CM(nh);    

    ContinuumRobot_NS::ContinuumRobot CR(Backbone_Length,Radius);//New continuum robot

    CM.SetModeID(3);//Torque mode

    CR.InitRobot();

    Vector2d Configuration_Now;//Configuration now
    Vector2d Configuration_Des;//Configuration desired from random
    Vector2d Configuration_Delta;//Delta C-motion direction in configuration space
    Vector2d Configuration_Direction;//unit vector of Configuration_Delta    

    Vector3d Driving_Velocity;
    
    ros::Rate Collect_T(10);//Desired path changed path/circle

    while(!CR.Boundary_Flag&&(Collect_Num--)){
        CR.ResetRobot(CM.GetPosition());

        //To make desired random derection in workspace by configuration space
        Configuration_Now = CR.Configuration;
        Configuration_Des = RandConfiguration();

        Configuration_Delta = Configuration_Des-Configuration_Now;
        Configuration_Direction = Configuration_Delta/Configuration_Delta.norm();//unit motion direction

        Driving_Velocity = CR.JacobianLC*Configuration_Direction*(PI/2)/20;//control law???     speedsize-20s:Theta 0 -> PI/2

        CM.SetVelocity(CR.TransVelocity(Driving_Velocity));

        //tongxun jilu lichuanganqidaxiao ----jilu kongjianweizhi 



        
        
        Collect_T.sleep();
    }

    return 0;
}



Vector2d RandConfiguration(){
    Vector2d Configuration_Rand;

    //rand Theta&Psi
    double Theta_Rand = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/PI*2));
    double Psi_Rand = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/PI/2));  

    Configuration_Rand(0) = Theta_Rand;
    Configuration_Rand(1) = Psi_Rand;

    return Configuration_Rand;
}