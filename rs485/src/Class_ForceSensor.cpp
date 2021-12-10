/**
 * @file Class_ForceSensor.cpp
 * @brief Transmitter:ZNLBSQ-TS3-485
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include <rs485/Class_ForceSensor.h>
#include <rs485/Function_Common.h>
#include <rs485/Class_Serial.h>


namespace NS_ForceSensor{
    //public
    ForceSensor::ForceSensor(){
        InitTrans();
    }

    //private

    void ForceSensor::RenewSFrame(MSG_CMD cmd,unsigned short data){
        unsigned short ts;//test
        SendFrame[SFI_Add] = AddBit;
        SendFrame[SFI_Cmd] = cmd;
        NS_CommonFunction::ShortToChar(data, &SendFrame[SFI_Data]);
        SendFrame[SFI_Check] = NS_CommonFunction::BCC(SendFrame,7);
    }

    size_t ForceSensor::SendMsg(){
    size_t size = 0;
    size = Ser.swrite(SendFrame,8);  
    ROS_INFO_STREAM("[test]ssize = "<<size);
    return size;
    }

    int ForceSensor::InitTrans(){
        size_t flag = 0;

        //set baud rate
        RenewSFrame(MC_BaudRate,3);//set baud rate:1-2400 2-4800 3-9600 4-19200 5-38400
        ROS_INFO_STREAM("[TEST]get new frame success");
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set baud rate of transmitter,flag = "<<flag);
            Ser.sclose();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set baud rate of transmitter successfully");

        return 0;
    }

}