/**
 * @file Class_USBCAN.cpp
 * @brief USBCAN model:GCAN USBCAN-Mini
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include "ECanVci.h"
#include "continuumrobot/Class_USBCAN.h"

namespace NS_USBCAN{
    //public
    USBCAN::USBCAN(){}

    DWORD USBCAN::OpenCAN(){
        //open device
        DWORD dwRel = 0;
        for(int i=0;i<3;i++){
            dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved);
            if (dwRel != STATUS_OK){ 
                ROS_ERROR_STREAM("[CAN]Failed to open device,try "<<i+1<<" times");                
            } 
            else{
                break;
            }
        }
        if (dwRel != STATUS_OK){ 
            return 0; 
        } 

        ROS_INFO_STREAM("[CAN]Open device successfully");

        //initializate CAN
        INIT_CONFIG init_config;
        init_config.AccCode = 0; 
        init_config.AccMask =0xffff; 
        init_config.Filter = 0;
        init_config.Timing0 = 0; 
        init_config.Timing1 = 0x14; 
        init_config.Mode = 0;
        dwRel = InitCAN(nDeviceType, nDeviceInd,nCANInd,&init_config);
        if (dwRel == STATUS_ERR) 
        { 
            ROS_ERROR_STREAM("[CAN]Failed to initializate CAN"); 
            CloseDevice(nDeviceType, nDeviceInd);
            return STATUS_ERR; 
        }
        ROS_INFO_STREAM("[CAN]Initializate CAN successfully");

        //start CAN
        dwRel = StartCAN(nDeviceType, nDeviceInd, nCANInd);  
        if (dwRel == STATUS_ERR){  
            ROS_ERROR_STREAM("[CAN]Failed to start CAN");
            CloseDevice(nDeviceType, nDeviceInd);  
            return STATUS_ERR; 
        }
        ROS_INFO_STREAM("[CAN]Start CAN successfully");
        return dwRel;
    }

    BOOL USBCAN::CloseCAN(){
        BOOL bRel;
        for(int i=0;i<5;i++){
            bRel = CloseDevice(nDeviceType, nDeviceInd);
            if (bRel != STATUS_OK){ 
                ROS_ERROR_STREAM("[CAN]Failed to close device,try "<<i+1<<" times");                
            } 
            else{
                break;
            }
        }
        if (bRel != STATUS_OK){ 
            return 0; 
        } 
         ROS_INFO_STREAM("[CAN]Close CAN successfully");
        return bRel;
    }

    DWORD USBCAN::ClearCAN(){
        DWORD dwRel;
        dwRel = ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
        if (dwRel == STATUS_ERR){  
            ROS_ERROR_STREAM("[CAN]Failed to clear buffer ");
            CloseDevice(nDeviceType, nDeviceInd);  
            return 0; 
        }
        ROS_INFO_STREAM("[CAN]Clear buffer successfully");
        return dwRel;
    }

    DWORD USBCAN::SendData(CAN_OBJ *Data,int Num){
        DWORD dwRel;
        dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,Data,Num);
        return dwRel;
    }

    DWORD USBCAN::ReceiveData(CAN_OBJ *Data,int Num,int WaitTime){
        DWORD dwRel;
        int Receivenum=0;
        while(Receivenum < Num){Receivenum = GetReceiveNum(nDeviceType, nDeviceInd, nCANInd);};
        dwRel = Receive(nDeviceType, nDeviceInd, nCANInd,Data,Num,WaitTime);
        return dwRel;
    }

    void USBCAN::PrintCAN_OBJ(CAN_OBJ Data){
        ROS_INFO_STREAM("ID="<<int(Data.ID));
        ROS_INFO_STREAM("Data="<<int(Data.Data[0])<<" "<<int(Data.Data[1])<<" "<<int(Data.Data[2])<<" "<<int(Data.Data[3])<<" "<<int(Data.Data[4])<<" "<<int(Data.Data[5])<<" "<<int(Data.Data[6])<<" "<<int(Data.Data[7]));
    }


}