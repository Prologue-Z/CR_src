/*
*  CANmotor.h
*
*  Saved on:2021.11.11
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:class CANMotor------Communication function between CAN-Motors
*/

//ECAN
#include <CANMotor/ECanVci.h>

//ROS
#include <ros/ros.h>

//PI
#include <math.h>
#define PI M_PI

namespace CANMotor_NS {

    class CANMotor {
    public:   
    /*!
    * Constructor.
    */
    CANMotor(ros::NodeHandle nh);
    /*!
    * Destructor.
    */
    ~CANMotor();

    /*!
    * Set motors mode 1-Position 2- Speed 3-Torque  and ID
    */   
    DWORD SetModeID(int Mode);

    /*!
    * Set motors speed(rpm)
    */
    DWORD SetVelocity(signed short Speed[3]);

    /*!
    * Get motors position in radian
    */   
    double * GetPosition();


    /*!
    * Close Motor
    */
    DWORD ClMotors();
    

    private:
    int DeviceType; // USBCAN-I 3;USBCAN-I 4
    int DeviceIndex; // DevIndex  0 1 2...
    int CANIndex;// CANIndex 0 1 2 ...

    int Encoder_PPR = 1024;
    double ReductionRatio = 42.3;//Reduction ratio of servo motors


    CAN_OBJ SendMotor[3];//Data to Motor

    CAN_OBJ RequestCommand_PositionFeedback[3];//Position feedback request command from pc to driver

    /*!
    * Open ECAN device
    */
    DWORD OpCAN();

    /*!
    * Initialize ECAN Device
    */
    DWORD InCAN();

    /*!
    * Start ECAN Device
    */
    DWORD StCAN();

    /*!
    * Clear buffer of the CAN channel.
    */
   DWORD ClCAN();

   /*!
    * Initialize data for SendMotor
    */
   void InData();

   /*!
    * Open Motors
    */
   DWORD OpMotors();


   

   

   


    };

}