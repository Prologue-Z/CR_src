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
     * @param Speed rpm
     * @return DWORD DWORD 1-success 0-failure
     */
    DWORD SetSpeed(double Speed[3]);

    /**
     * @brief Get the Position of motors(number of circles relative to power on)
     * 
     * @return double* number of circles
     */
    double * GetPosition();

    /**
     * @brief clear position of motors to 0
     * 
     * @return DWORD 1-success 0-fail
     */
    DWORD ClearPosition();

    /**
     * @brief close Motors:disenable motors->close CAN
     * 
     * @return DWORD 1-success 0-fail
     */
    DWORD CloseMotors();

    /**
     * @brief Forced shutdown when wrong->close CAN
     * 
     */
    void ForcedShutdown();

    private:
    NS_USBCAN::USBCAN CAN;
    int WaitTime_CAN = 2000;//ms
    CAN_OBJ SendData[3];
    CAN_OBJ ReceiveData[3];
    const double ReductionRatio = 42.3;
    const int Encoder_PPR = 1024*4;//1024-ppr 4-frequency doubling
    double Position[3]={0,0,0};

    /**
     * @brief initialize SendData
     * 
     */
    void InitSendData();

    /**
     * @brief check receivedate is right
     * 
     * @param ReadWrite 0-read 1-write
     * @return DWORD 
     */
    DWORD DataCheck(int ReadWrite);//0-read 1-write

    };

}

#endif