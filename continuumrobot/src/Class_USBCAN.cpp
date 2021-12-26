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
    DWORD USBCAN::OpenCAN(){
        //open device
        DWORD dwRel;
        dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved); 
        if (dwRel != STATUS_OK) 
        { 
            ROS_ERROR_STREAM("[CAN]Failed to open device");
            return STATUS_ERR; 
        } 
        ROS_INFO_STREAM("[CAN]Open device successfully");

        //initializate CAN
        INIT_CONFIG init_config;
        init_config.AccCode = 0; 
        init_config.AccMask =0xffffff; 
        init_config.Filter = 0;
        init_config.Timing0 = 0; 
        init_config.Timing1 = 0x1c; 
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

        


        }

}