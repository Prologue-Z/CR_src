/**
 * @file Class_CANMotor.cpp
 * @brief driver model:IDS-306  usbCAN model:GCAN USBCAN-Mini
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include "ECanVci.h"
#include "continuumrobot/Class_Motor.h"
#include "continuumrobot/Function_Common.h"

namespace NS_Motor{
    //public
    DWORD Motor::InitMotors(){
        DWORD dwRel;
        dwRel = CAN.OpenCAN();
        dwRel = CAN.ClearCAN();
        InitSendData();
        return dwRel;
    }

    DWORD Motor::EnableMotors(){
        DWORD dwRel;
        InitSendData();

        for(int i=0;i<3;i++){
            SendData[i].Data[2] = 0x00;
            SendData[i].Data[3] = 0x00;
            SendData[i].Data[4] = 0x01;
        }
        
        dwRel = CAN.SendData(SendData,3);
        dwRel = CAN.ReceiveData(ReceiveData,3,1000);

        if(dwRel < 3&&ReceiveData[0].Data[1]!=0x1b&&ReceiveData[1].Data[1]!=0x1b&&ReceiveData[2].Data[1]!=0x1b){
            ROS_ERROR_STREAM("[Motor]Send data fail,failed to enable motors");
            CAN.CloseCAN(); 
            return 0; 
        }

        ROS_INFO_STREAM("[Motor]Enable motors successfully");
        return 1;
    }

    DWORD Motor::CloseMotors(){
        DWORD dwRel;
        InitSendData();

        for(int i=0;i<3;i++){
            SendData[i].Data[2] = 0x00;
            SendData[i].Data[3] = 0x00;
            SendData[i].Data[4] = 0x00;
        }
        
        dwRel = CAN.SendData(SendData,3);
        dwRel = CAN.ReceiveData(ReceiveData,3,1000);

        if(dwRel < 3&&ReceiveData[0].Data[1]!=0x1b&&ReceiveData[1].Data[1]!=0x1b&&ReceiveData[2].Data[1]!=0x1b){
            ROS_ERROR_STREAM("[Motor]Send data fail,failed to disenable motors");
            CAN.CloseCAN(); 
            return 0; 
        }

        ROS_INFO_STREAM("[Motor]Disenable motors successfully");

        sleep(1);
        CAN.CloseCAN();
        return 1;
    }

    DWORD Motor::SetSpeedMode(){
        DWORD dwRel;
        InitSendData();

        for(int i=0;i<3;i++){
            SendData[i].Data[2] = 0x02;
            SendData[i].Data[3] = 0x00;
            SendData[i].Data[4] = 0xc4;
        }

        dwRel = CAN.SendData(SendData,3);
        dwRel = CAN.ReceiveData(ReceiveData,3,1000);

        if(dwRel < 3&&ReceiveData[0].Data[1]!=0x1b&&ReceiveData[1].Data[1]!=0x1b&&ReceiveData[2].Data[1]!=0x1b){
            ROS_ERROR_STREAM("[Motor]Send data fail,failed to set speed mode");
            CAN.CloseCAN(); 
            return 0; 
        }

        ROS_INFO_STREAM("[Motor]Set speed mode successfully");
        return dwRel;
    }

    DWORD Motor::SetSpeed(double Speed[3]){
        DWORD dwRel;
        InitSendData();

        for(int i=0;i<3;i++){
            SendData[i].Data[2] = 0x06;
            NS_CommonFunction::IntToBYTE(int(Speed[i]*ReductionRatio),&SendData[i].Data[3]);
        }

        dwRel = CAN.SendData(SendData,3);        
        dwRel = CAN.ReceiveData(ReceiveData,3,1000);

        if(dwRel < 3&&ReceiveData[0].Data[1]!=0x1b&&ReceiveData[1].Data[1]!=0x1b&&ReceiveData[2].Data[1]!=0x1b){
            ROS_ERROR_STREAM("[Motor]Send data fail,failed to set speed");
            CAN.CloseCAN(); 
            return 0; 
        }

        ROS_INFO_STREAM("[Motor]Set speed successfully");
        return dwRel;        
    }

    double * Motor::GetPosition(){
        DWORD dwRel;
        double Position[3];
        InitSendData();

        for(int i=0;i<3;i++){
            SendData[i].Data[1] = 0x2a;//read
            SendData[i].Data[2] = 0xe8;
            SendData[i].Data[2] = 0xe9;         
        }

        dwRel = CAN.SendData(SendData,3);
        dwRel = CAN.ReceiveData(ReceiveData,3,1000);

        if(dwRel < 3){
            ROS_ERROR_STREAM("[Motor]Receive data fail,failed to get position");
            CAN.CloseCAN(); 
            return 0; 
        }

        for(int i=0;i<3;i++){
            Position[i] = NS_CommonFunction::ByteToInt(ReceiveData[i])/Encoder_PPR*2*M_PI;
        }

        ROS_INFO_STREAM("[Motor]Get position successfully");
        return Position;
    }




    //private
    void Motor::InitSendData(){
        BYTE initData[8] = {0x00,0x1a,0xff,0x00,0x00,0xff,0x00,0x00};
        for(int i=0;i<3;i++){
            SendData[i].ID = i+1;
            SendData[i].DataLen = 8;
            SendData[i].SendType = 0;
            SendData[i].RemoteFlag = 0;
            SendData[i].ExternFlag = 0;
            memcpy(SendData[i].Data,initData,sizeof(initData));
        }
    }

}