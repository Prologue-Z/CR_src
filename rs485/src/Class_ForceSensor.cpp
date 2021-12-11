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

    double* ForceSensor::GetForce(){
        for(int i=1;i<=3;i++){
            RenewSFrame(1,MC_SingleOutput,0);
            SendMsg();
            usleep(50000);//50ms
            ReadMsg();
        }        
        return Force;
    }

    //private

    void ForceSensor::RenewSFrame(unsigned short add,MSG_CMD cmd,unsigned short data){
        SendFrame[SFI_Add] = add;
        SendFrame[SFI_Cmd] = cmd;
        NS_CommonFunction::ShortToChar(data, &SendFrame[SFI_Data]);
        SendFrame[SFI_Check] = NS_CommonFunction::BCC(SendFrame,7);
    }

    size_t ForceSensor::SendMsg(){
    size_t size = Ser.swrite(SendFrame,8);  //8-a send frame of transmitter
    return size;
    }

    size_t ForceSensor::ReadMsg(){
        unsigned char* ReadBuff = new unsigned char[30];//30=10*3 10-a receive frame of transmitter;3-three forcesensors
        size_t size = Ser.sread(ReadBuff,30);
        Handledata(ReadBuff);
        return size;
    }

    int ForceSensor::InitTrans(){
        size_t flag = 0;
        //set baud rate
        for(int i=1;i<=3;i++){
            RenewSFrame(1,MC_BaudRate,3);//set baud rate:1-2400 2-4800 3-9600 4-19200 5-38400
            flag += SendMsg();
        }      
        if(flag<24){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set baud rate of transmitter,flag = "<<flag);
            Ser.sclose();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set baud rate of transmitter successfully");

        return 0;
    }

    void ForceSensor::Handledata(unsigned char *ReBuff){
        short s[3];
        for(int i=0;i<3;i++){
            s[i] = NS_CommonFunction::CharToShort(&ReBuff[RFI_Data+i*RFI_FrameNum]);
            Force[i] = s[i]*pow(10,-ReBuff[RFI_Decimal+i*RFI_FrameNum]);//unit -kg
            Force[i] *= 9.8;//unit-N
        }
    }

}