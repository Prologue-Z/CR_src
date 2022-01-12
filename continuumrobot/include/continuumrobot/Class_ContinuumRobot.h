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

        /**
         * @brief init Length_DrivingWire__Startup,DrivingWire_Length; set Motor->reset robot
         * 
         * @return int 1-success 0-failure
         */
        int InitRobot();

        /**
         * @brief clear position of motors to zero
         * 
         * @return int 1-success 0- fail
         */
        int ClearMotorPosition();

        /**
         * @brief configuration to Configuration_Desired
         * 
         * @param Configuration_Desired 
         * @param T motion time
         * @param F control frequency
         * @return int 0-fail 1-success
         */
        int ToConfiguration(double Configuration_Desired[2],int T,int F);

        /**
         * @brief Length_DrivingWire to Length_DrivingWire_Desired 
         * 
         * @param Length_DrivingWire_Desired 
         * @param T motion time
         * @param F control frequency
         * @return int 0-fail 1-success
         */
        int ToLength_DrivingWire(double Length_DrivingWire_Desired[3],int T,int F);

        int MotorTest();

        private:
        const double Length_Backbone = 0.3;
        const double Radius = 0.015;
        const double Beta = 2*PI/3;
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
         * @brief Set the Velocity of each lead screw
         * 
         * @param Velocity 
         * @return int 0-fail 1-success
         */
        int SetVelocity(Vector3d Velocity);

        /**
         * @brief Reset every paramters of the continuum robot:ResetLength_DrivingWire->ResetConfiguration
         * 
         */
        void ResetRobot();


        void ResetLength_DrivingWire();
        void ResetConfiguration();
        void ResetX();

        void ResetJacobianCX();
        void ResetJacobianLC();
        void ResetJacobianLX();

        /**
         * @brief kinematic:Configuration -> Length_DrivingWire
         * 
         * @param Configuration_Desired configuraition
         * @return double* 
         */
        double* ConfigurationToLength_DrivingWire(double Configuration_Desired[2]);
        
        /**
         * @brief get the pseudo inverse of a matrix
         * 
         * @param J 
         * @return MatrixXd 
         */
        MatrixXd pinv(MatrixXd J);

        /**
         * @brief  trajectory planning of velocity
         * @ref Li，Minhan - 机器人控制系统与程序技术文档.PDF
         * 
         * @param T motion time
         * @param F control frequency
         * @return MatrixXd desired velocity matrix at each time point
         */
        MatrixXd TrajectoryGeneration(double Configuration_Desired_0,double Configuration_Desired_1,int T,int F);
        MatrixXd TrajectoryGeneration(double Length_DrivingWire_Desired_0,double Length_DrivingWire_Desired_1,double Length_DrivingWire_Desired_2,int T,int F);

        /**
         * @brief Reset Length_record.txt from @param Length_DrivingWire
         * 
         */
        void ResetDoc_Length();

    };
    
}