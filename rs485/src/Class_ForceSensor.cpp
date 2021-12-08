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


namespace NS_ForceSensor{
    //public
    ForceSensor::ForceSensor(){
        InitSer();
        InitTrans();
    }

    //private
    int ForceSensor::InitSer(){
        Ser.setPort(SerName);
        Ser.setBaudrate(BaudRate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);//ms

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

    void ForceSensor::AddFrame(MSG_CMD cmd,int data){
        unsigned char *pNode = NULL;
        unsigned char NewFrame[8];
        NewFrame[0] = NewFrame[1] = NewFrame[2] = 0xAA;
        NewFrame[SFI_Add] = AddBit;
        
        NewFrame[SFI_Cmd] = cmd;
        NS_CommonFunction::ShortToChar(data, &NewFrame[SFI_Data]);

        NewFrame[SFI_Check] = NS_CommonFunction::BCC(NewFrame,7);
        unsigned char test = 0x01;
        ROS_INFO_STREAM("[test]"<<test);
        pNode = (unsigned char *)malloc(sizeof(NewFrame)); /* 发送线程会free掉它 */
        memcpy(pNode, NewFrame, sizeof(NewFrame));
        

        SendDataList.push_back(pNode);
    }

    size_t ForceSensor::SendMsg(){
    unsigned char *SenBufNode = NULL;
    size_t size = 0;
    SenBufNode = SendDataList.front();
    SendDataList.pop_front();

    size = Ser.write(SenBufNode,8);

    delete SenBufNode;
    return size;
    }

    int ForceSensor::InitTrans(){
        short flag = 0;

        //set baud rate
        AddFrame(MC_BaudRate,3);//set baud rate:1-2400 2-4800 3-9600 4-19200 5-38400
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set baud rate of transmitter,flag = "<<flag);
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set baud rate of transmitter successfully");

        //set range
        AddFrame(MC_Range,10);//set range
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set range of transmitter");
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set range of transmitter successfully");

        //set unit
        AddFrame(MC_Unit,2);//set unit:1-MPa 2-Kg 3-T
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set unit of transmitter");
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set unit of transmitter successfully");

        //set polarity
        AddFrame(MC_Polarity,2);//1-Unipolarity 2-Bipolarity  -10-+10
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set polarity of transmitter");
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set polarity of transmitter successfully");

        //reset zero point
        AddFrame(MC_ZeroPoint,0);
        flag = SendMsg();
        if(flag<8){
            ROS_ERROR_STREAM("[RS485 Error]Failed to set zero point of transmitter");
            Ser.close();
            return -1;
        }
        else ROS_INFO_STREAM("[RS485]Set zero point of transmitter successfully");

        return 0;
    }

}