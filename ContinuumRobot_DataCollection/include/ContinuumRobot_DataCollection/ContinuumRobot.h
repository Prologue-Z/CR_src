/*
*  ContinuumRobot.h
*
*  Saved on:2021.11.11
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:class ContinuumRobot------kinematics model
* Unit:SI-m,s,kg
* Reference: Zhang X., Liu Y., Branson D.T., Yang C., Dai J.S., Kang R. , Variable-gain control for continuum robots based on velocity sensitivity [J], Mechanism and Machine Theory, 2022, 168: 104618.
*/

//PI
#include <math.h>
#define PI M_PI

//Eigen3
#include <eigen3/Eigen/Dense>
using namespace Eigen;

namespace ContinuumRobot_NS {

    class ContinuumRobot {
        public:
        /*!
        * Constructor.
        * L - length of backbone;R - distribution radius driving wires 
        */
        ContinuumRobot(double L, double R);        

        /*!
        * Init robot-driving wires length
        * data from: scanf txt/backbone length
        */
        void InitRobot();
        
        /*!
        * Reset member variables of continuum robot
        * R - Radians of retarder output shaft
        */
        void ResetRobot(double R[3]);


        /*!
        * Transform velocity of drivingwire to speed of reducer output shaft
        * Driving_Velocity-velocity of drivingwire    return-speed of reducer output shaft
        */
        signed short * TransVelocity(Vector3d Driving_Velocity);

        bool Boundary_Flag = false;//false-not to boundary;true-out boundry

        Vector3d DrivingWire_Length_Init;//DrivingWire_Length at startup robot        
        Vector3d DrivingWire_Length;//Position in actuation space-Lengthree driv of thing wires
        Vector2d Configuration;//Position in configuration space-Configuration[0]-Center angle of arc  Configuration[1]-Angle between bending plane and X-Z plane
        Vector3d X;//Position in operating space

        Matrix<double,2,3> JacobianCX;// Jacobian dC1dX
        Matrix<double,3,2> JacobianLC;// Jacobian dL1dC
        Matrix<double,3,3> JacobianLX;// Jacobian dL1dX

        private:
        double Backbone_Length;//Length of backbone
        double Radius;//Distribution radius driving wires
        double Beta;//Driving wire distribution phase difference

        double ScrewLead = 0.001;//Lead of ball screw

        /*!
        * Reset DrivingWire_Length
        */
        void ResetDrivingWire_Length(double L[3]);
        
        /*!
        * Reset Configuration
        * Configuration[0]-Center angle of arc  Configuration[1]-Angle between bending plane and X-Z plane
        */
        void ResetConfiguration();

        /*!
        * Reset X
        */
        void ResetX();    

        /*!
        * Reset JacobianCX
        */
       void ResetJacobianCX();

       /*!
        * Reset JacobianLC
        */
       void ResetJacobianLC();

       /*!
        * Reset JacobianLX
        */
       void ResetJacobianLX();


       /*!
        * Reset Boundry_Flag
        * if desired path out of boundry,change Boundry_Flag->true;
        * else Boundry_Flag->false
        */
       void ResetBoundry_Flag();


        /*!
        * Reset matrix's pseudoInverse
        */
       MatrixXd pinv(MatrixXd J);

    };
}