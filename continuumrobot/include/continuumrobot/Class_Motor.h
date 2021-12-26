/**
 * @file Class_Motor.h
 * @brief driver model:IDS-306
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#ifndef CLASS_MOTOR_H
#define CLASS_MOTOR_H

#include "continuumrobot/Class_USBCAN.h"

namespace NS_Motor{
    class Motor{
    public:
    /**
     * @brief Construct a new Motor object
     * 
     */
    Motor();

    /**
     * @brief open CAN -> clear buffer ->init SendData
     * 
     * @return DWORD 1-success 0-failure
     */
    DWORD InitMotors();

    /**
     * @brief Enable motors:send data->receive data
     * 
     * @return DWORD 1-success 0-failure
     */
    DWORD EnableMotors();

    /**
     * @brief Set Speed Mode
     * 
     * @return DWORD 1-success 0-failure
     */
    DWORD SetSpeedMode();

    /**
     * @brief Set the Speed object
     * 
     * @param Speed 
     * @return DWORD DWORD 1-success 0-failure
     */
    DWORD SetSpeed(double Speed[3]);

    /**
     * @brief Get the Position of motors(Phase angle relative to power on)
     * 
     * @return double*radian measure
     */
    double * GetPosition();

    DWORD CloseMotors();

    private:
    NS_USBCAN::USBCAN CAN;
    CAN_OBJ SendData[3];
    CAN_OBJ ReceiveData[3];
    const double ReductionRatio = 42.3;
    const int Encoder_PPR = 1024;
    double Position[3]={0,0,0};

    /**
     * @brief initialize SendData
     * 
     */
    void InitSendData();



    };

}

#endif