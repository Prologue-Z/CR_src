/**
 * @file Class_ContinuumRobot.h
 * @brief Unit:SI-m,s,kg
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2022-01-03
 * 
 * @copyright Copyright (c) 2022 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <math.h>
#define PI M_PI

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include "continuumrobot/Class_Motor.h"


namespace NS_ContinuumRobot {
    class ContinuumRobot{
        public:
        /**
         * @brief Construct a new Continuum Robot object-init robot
         * 
         */
        ContinuumRobot();

        /**
         * @brief Destroy the Continuum Robot object-close motor
         * 
         */
        ~ContinuumRobot();

        int ToConfiguration(double Configuration_Desired[2],int T,int F);

        /**
         * @brief init Length_DrivingWire__Startup,DrivingWire_Length; set Motor->reset robot
         * 
         * @return int 1-success 0-failure
         */
        int InitRobot();

        private:
        const double Length_Backbone = 0.3;
        const double Radius = 0.015;
        const double Beta = PI/3;
        const double ScrewLead = -0.001;

        //const char* FileAddress = "../doc/Length_Record.txt";
        const char* FileAddress = "/home/zhangxu/catkin_ws/src/continuumrobot/doc/Length_Record.txt";//???lujingwenti 

        Vector3d Length_DrivingWire__Startup;//DrivingWire_Length at startup robot 
        Vector3d Length_DrivingWire;//Position in actuation space-Length of three driving wires
        Vector2d Configuration;//Position in configuration space-Configuration[0]-Center angle of arc  Configuration[1]-Angle between bending plane and X-Z plane
        Vector3d X;//Position in operating space

        Matrix<double,2,3> JacobianCX;// Jacobian dC1dX
        Matrix<double,3,2> JacobianLC;// Jacobian dL1dC
        Matrix<double,3,3> JacobianLX;// Jacobian dL1dX

        NS_Motor::Motor Motor;

        /**
         * @brief 
         * 
         */
        void ResetRobot();

        int SetVelocity(Vector3d Velocity);

        void ResetLength_DrivingWire();
        void ResetConfiguration();
        void ResetX();

        void ResetJacobianCX();
        void ResetJacobianLC();
        void ResetJacobianLX();

        MatrixXd pinv(MatrixXd J);

        MatrixXd TrajectoryGeneration(double Theta_Desired, double Psi_Desired,int T,int F);



    };
    
}