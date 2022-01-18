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
#include "rs485/Class_ForceSensor.h"
#include "rs485/Function_Common.h"


namespace NS_ForceSensor{
    //public
    ForceSensor::ForceSensor(){
        InitTrans();
    }

    double* ForceSensor::GetForce(){
        for(int i=1;i<=3;i++){
            RenewSFrame(i,MC_SingleOutput,0);
            SendMsg();
            //usleep(100000);
            int Num = 0;
            while(Num < 10){Num = Ser.available();};
            ROS_INFO_STREAM("[test] Num = "<<Num);
            ReadMsg(i);
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
        size_t size = Ser.write(SendFrame,8);  //8-a send frame of transmitter
        if(size<8) {        
            ROS_ERROR_STREAM("[RS485]Send data failue");
            Ser.close();
        }
        return size;
    }

    size_t ForceSensor::ReadMsg(int i){
        unsigned char* ReadBuff = new unsigned char[10];//10-a receive frame of transmitter
        size_t size = Ser.read(ReadBuff,10);
        if(size<10){
            ROS_ERROR_STREAM("[RS485]Read data failure");
        }
        //sROS_INFO_STREAM("[test]rf[3] = "<<short(ReadBuff[RFI_Add]));
        Force[i-1] = Handledata(ReadBuff);
        return size;
    }

    int ForceSensor::InitTrans(){
        size_t flag = 0;
        //Set Ser
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        Ser.setPort("/dev/ttyCH341USB0");
        Ser.setBaudrate(9600);
        Ser.setTimeout(to);
        //open Ser
        try{
            Ser.open();
        }
        catch(serial::IOException& e){
            ROS_ERROR_STREAM("[RS485] Unable to open port.");
            return -1;
        }        
        if(Ser.isOpen())
        {
            ROS_INFO_STREAM("[RS485] /dev/ttyCH341USB0 is opened.");
        }
        else{
            return -1;
        }

        //set baud rate
        for(int i=1;i<=3;i++){
            RenewSFrame(1,MC_BaudRate,3);//set baud rate:1-2400 2-4800 3-9600 4-19200 5-38400
            flag += SendMsg();
        }      
        if(flag<24){
            ROS_ERROR_STREAM("[RS485] Failed to set baud rate of transmitter,flag = "<<flag);
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485] Set baud rate of transmitter successfully");

        //clear buffer
        Ser.flush();

        return 0;
    }

    double ForceSensor::Handledata(unsigned char *ReBuff){
        short s;
        s = NS_CommonFunction::CharToShort(&ReBuff[RFI_Data]);
        double F = s*pow(10,-int(ReBuff[RFI_Decimal])+1);//unit -kg
        F *= 9.8;//unit-N
        return F;
    }

}