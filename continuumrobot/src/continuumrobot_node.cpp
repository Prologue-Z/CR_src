/**
 * @file continuumrobot_node.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-23
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>
#include "ECanVci.h"
#include "continuumrobot/Function_Common.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "continuumrobot_node");
    ros::NodeHandle nh("~");    

	DWORD dwRel;
    int nDeviceType = 3;
	int nDeviceInd = 0;
	int nReserved = 0;
	int nCANInd =0;
	
	//open 
	dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved); 
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail open dev"); 
	    return -1; 
	} 
	ROS_INFO("open dev succ");

	//init
	INIT_CONFIG init_config;
	init_config.AccCode = 0; 
	init_config.AccMask =0xffff; 
	init_config.Filter = 0;
	init_config.Timing0 = 0; 
	init_config.Timing1 = 0x14; 
	init_config.Mode = 0;
	dwRel = InitCAN(nDeviceType, nDeviceInd,nCANInd,&init_config);
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail init dev"); 
		CloseDevice(nDeviceType, nDeviceInd);
	    return -1; 
	}
	ROS_INFO("init dev succ"); 

	//star
	dwRel = StartCAN(nDeviceType, nDeviceInd, nCANInd);  
	if (dwRel == STATUS_ERR){  
		ROS_ERROR("fail start can");
		CloseDevice(nDeviceType, nDeviceInd);  
		return -1; 
	}
	ROS_INFO("start can succ");

	//clear buffer
	dwRel = ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	if (dwRel == STATUS_ERR){  
		ROS_ERROR("fail clear buffer");
		CloseDevice(nDeviceType, nDeviceInd);  
		return -1; 
	}
	ROS_INFO("clear buffer successfully");

	//init canobj
	CAN_OBJ SendMotor[3];
	BYTE initData[8] = {0x00,0x1a,0xff,0x00,0x00,0xff,0x00,0x00};
	enum DATA_INDEX{
		DI_Head = 0,
		DI_FunctionCode = 1,
		DI_Register1 = 2,
		DI_Data1 = 3,
		DI_Register2 = 5,
		DI_Data2 = 6,
    };
	enum FC_TYPE{
		FT_Write = 0x1a,//1 to 1
		FT_Read = 0x2a,//1 to 1
	};


	for(int i=0;i<3;i++){
		SendMotor[i].ID = i+1;
		SendMotor[i].DataLen = 8;
		SendMotor[i].SendType = 0;
		SendMotor[i].RemoteFlag = 0;
		SendMotor[i].ExternFlag = 0;
		memcpy(SendMotor[i].Data,initData,sizeof(initData));
	}

	//enable
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x1a;
		SendMotor[i].Data[DI_Register1] = 0x00;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0x01;
		SendMotor[i].Data[DI_Register2] = 0xff;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,enable motors fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("enable motor success");

	//set speed
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x1a;
		SendMotor[i].Data[DI_Register1] = 0x02;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0xc4;
		SendMotor[i].Data[DI_Register2] = 0xff;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,set mode fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("set speed mode success");

	//clear buffer
	dwRel = ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
	if (dwRel == STATUS_ERR){  
		ROS_ERROR("fail clear buffer");
		CloseDevice(nDeviceType, nDeviceInd);  
		return -1; 
	}
	ROS_INFO("clear buffer successfully");

	//receive position
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x2a;
		SendMotor[i].Data[DI_Register1] = 0xe8;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0x00;
		SendMotor[i].Data[DI_Register2] = 0xe9;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,receive position fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("receive position");

	CAN_OBJ ReceiveMotor[3];
	dwRel = Receive(nDeviceType, nDeviceInd, nCANInd,ReceiveMotor,3,1000);
	int position[3] = {0,0,0};
	for(int i=0;i<3;i++){
		position[i] = NS_CommonFunction::ByteToInt(ReceiveMotor[i]);
	}
	ROS_INFO_STREAM("dw="<<dwRel);
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Register1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data1+1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data2]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data2+1]));
	ROS_INFO_STREAM("p1="<<position[0]<<"p2="<<position[1]<<"p3="<<position[2]);


	//speed
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x1a;
		SendMotor[i].Data[DI_Register1] = 0x06;
		SendMotor[i].Data[DI_Data1] = 0x02;
		SendMotor[i].Data[DI_Data1+1] = 0x58;//600rpm
		SendMotor[i].Data[DI_Register2] = 0xff;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,set speed fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("set speed success");

	
	dwRel = GetReceiveNum(nDeviceType, nDeviceInd, nCANInd);
	ROS_INFO_STREAM("rnum = "<<dwRel);
	
	sleep(10);

	//set o speed
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x1a;
		SendMotor[i].Data[DI_Register1] = 0x06;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0x00;//0rpm
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,set speed fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("set speed success");

	//receive position
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x2a;
		SendMotor[i].Data[DI_Register1] = 0xe8;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0x00;
		SendMotor[i].Data[DI_Register2] = 0xe9;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,receive position fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("receive position");
	dwRel = Receive(nDeviceType, nDeviceInd, nCANInd,ReceiveMotor,3,1000);
	ROS_INFO_STREAM("dw="<<dwRel);
	for(int i=0;i<3;i++){
		position[i] = ReceiveMotor[i].Data[DI_Data1]<<24|ReceiveMotor[i].Data[DI_Data1+1]<<16|ReceiveMotor[i].Data[DI_Data2]<<8|ReceiveMotor[i].Data[DI_Data2+1];
	}
	ROS_INFO_STREAM("p1="<<position[0]<<"p2="<<position[1]<<"p3="<<position[2]);
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Register1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data1+1]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data2]));
	ROS_INFO_STREAM(int(ReceiveMotor[2].Data[DI_Data2+1]));


	//dis enable
	for(int i=0;i<3;i++){
		SendMotor[i].Data[DI_FunctionCode] = 0x1a;
		SendMotor[i].Data[DI_Register1] = 0x00;
		SendMotor[i].Data[DI_Data1] = 0x00;
		SendMotor[i].Data[DI_Data1+1] = 0x00;
		SendMotor[i].Data[DI_Register2] = 0xff;
		SendMotor[i].Data[DI_Data2] = 0x00;
		SendMotor[i].Data[DI_Data2+1] = 0x00;
	}
	dwRel = Transmit(nDeviceType, nDeviceInd, nCANInd,SendMotor,3);
	if(dwRel < 3){
		ROS_ERROR("send data fail,disable motors fail");
		CloseDevice(nDeviceType, nDeviceInd);  
		return 0; 
	}
	ROS_INFO_STREAM("disenable motor success");

	sleep(1);//关太快最后一条会失效
    
	CloseDevice(nDeviceType, nDeviceInd);//fanhuizhikanyixia 
	ROS_INFO("close can");

	return 1;
}
