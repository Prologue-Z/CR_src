/**
 * @file Class_ForceSensor.cpp
 * @brief Transmitter:ZNLBSQ-TS3-485
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */


#include <rs485/Class_ForceSensor.h>
#include <ros/ros.h>

namespace NS_ForceSensor{
    //public
    ForceSensor::ForceSensor(){
        InitSer();
    }

    //private
    int ForceSensor::InitSer(){
        Ser.setPort(SerName);
        Ser.setBaudrate(BaudRate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);

        try{
            Ser.open();
        }
        catch (serial::IOException&e){ 
            ROS_ERROR_STREAM("[RS485 Error] Unable to open port "); 
            return -1; 
        }

        if(Ser.isOpen()){ 
            ROS_INFO_STREAM("[RS485] Serial Port initialized"); 
            return 0;
        } 
        else{ 
            return -1; 
        }
    }
}