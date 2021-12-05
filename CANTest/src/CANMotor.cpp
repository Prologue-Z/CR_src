/*
*  CANmotor.cpp
*
*  Saved on:2021.11.11
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
* 
* PS: ???---undefined to modify
*/

//CANNotor
#include <CANTest/CANMotor.h>

//ECAN
#include <CANTest/ECanVci.h>

//c++
#include <unistd.h>

namespace CANMotor_NS {
//public
CANMotor::CANMotor(ros::NodeHandle nh)
    :DeviceType(3),DeviceIndex(0),CANIndex(0){
   ROS_INFO("[CANMotor] Node started.");
   DWORD dwRel;
   dwRel = OpCAN();
   dwRel = InCAN();
   dwRel = StCAN();
   dwRel = ClCAN();
   dwRel = OpMotors();
}

DWORD CANMotor::SetModeID(int Mode){
    if(Mode==1) ROS_INFO("[CANMotor] Use position control mode");
    else if(Mode==2) ROS_INFO("[CANMotor] Use speed control mode");
    else ROS_INFO("[CANMotor] Use torque control mode");    

    DWORD dwRel;

    // Data setting of motors mode
    SendMotor[0].ID = 0x00;//To all motors in the group???
    SendMotor[0].Data[0] = 0x00;//Number of Group-0
    SendMotor[0].Data[1] = 0x9A;//To One-to-many write data operation without response

    // Set mode
    if(Mode==1) {//Position mode 
    SendMotor[0].Data[2] = 0x02;
    SendMotor[0].Data[3] = 0x00;
    SendMotor[0].Data[4] = 0xd0;
    //Acceleration setting
    SendMotor[0].Data[5] = 0x09;
    SendMotor[0].Data[6] = 0x00;//10-100ms 0-3000rpm 0-max response
    SendMotor[0].Data[7] = 0x00;
    }
    else{//Speed mode and torque mode (torque control by changing rated current(ox2c))
    SendMotor[0].Data[2] = 0x02;
    SendMotor[0].Data[3] = 0x00;
    SendMotor[0].Data[4] = 0xc4;
    //Acceleration setting
    SendMotor[0].Data[5] = 0x0a;
    SendMotor[0].Data[6] = 0x00;//100ms 0-3000rpm 0-max response
    SendMotor[0].Data[7] = 0x00;
    }        

    dwRel = Transmit(DeviceType, DeviceIndex, CANIndex,SendMotor,1);//1-SendData[0] only

    InData();//Initialize SendMotor ID&Data-0xFF

    return dwRel;
}

DWORD CANMotor::SetVelocity(signed int Speed[3]){
    DWORD dwRel;
    signed short WrValue[3];

    //Communication setting
    SendMotor[0].Data[0] = 0x00;//Number of Group-0
    SendMotor[0].Data[1] = 0x2A;//To One-to-one write data operation without response
    SendMotor[1].Data[0] = 0x00;//Number of Group-0
    SendMotor[1].Data[1] = 0x2A;//To One-to-one write data operation without response
    SendMotor[2].Data[0] = 0x00;//Number of Group-0
    SendMotor[2].Data[1] = 0x2A;//To One-to-one write data operation without response

    //Calculate write value-IDS306 Servo driver user manual
    WrValue[0] = Speed[0]/3000*8192;
    WrValue[1] = Speed[1]/3000*8192;
    WrValue[2] = Speed[2]/3000*8192;    

    //setting speed???
    SendMotor[0].Data[2] = (unsigned char)((WrValue[0]>>8)&0xff); 
    SendMotor[0].Data[3] = (unsigned char)((WrValue[0])&0xff);//Velocity of motor1
    SendMotor[1].Data[2] = (unsigned char)((WrValue[1]>>8)&0xff); 
    SendMotor[1].Data[3] = (unsigned char)((WrValue[1])&0xff);//Velocity of motor2
    SendMotor[2].Data[2] = (unsigned char)((WrValue[2]>>8)&0xff); 
    SendMotor[2].Data[3] = (unsigned char)((WrValue[2])&0xff);//Velocity of motor3

    dwRel = Transmit(DeviceType, DeviceIndex, CANIndex,SendMotor,3);//3-SendData[0-2]

    return dwRel;
}

DWORD CANMotor::ClMotors(){
    ROS_INFO("[CANMotor] Close motors");

    DWORD dwRel;

    // Data setting of closing motors
    SendMotor[0].ID = 0x00;//To all motors in the group???
    SendMotor[0].Data[0] = 0x00;//Number of Group-0
    SendMotor[0].Data[1] = 0x9A;//To One-to-many write data operation without response
    // Open motors
    SendMotor[0].Data[2] = 0x00;
    SendMotor[0].Data[3] = 0x00;
    SendMotor[0].Data[4] = 0x00;
    //Null data
    SendMotor[0].Data[5] = 0xFF;
    SendMotor[0].Data[6] = 0xFF;
    SendMotor[0].Data[7] = 0xFF;

    dwRel = Transmit(DeviceType, DeviceIndex, CANIndex,SendMotor,1);//1-SendData[0] only
    /*if (dwRel == STATUS_ERR)
	{
		ROS_INFO("Error:Failed to close motors");
	}
    */
    return dwRel;
}

double * CANMotor::GetPosition(){
    DWORD ReceiveNum;//Data number in CAN
    DWORD dwRel;//Read data number
    CAN_OBJ DataFromCAN[3];
    CAN_OBJ Position_CAN[3];

    int Position_Encoder[3];//Encoder data
    double Position[3];//Return position in radian

    UINT Count = 0;
    int mi = 0;
    int i = 0;

    ReceiveNum = GetReceiveNum(DeviceType, DeviceIndex, CANIndex);
    dwRel = Receive(DeviceType, DeviceIndex, CANIndex,DataFromCAN,ReceiveNum,1000);

    //???
    if(dwRel!=0xFFFFFFFF){
        while (Count!=7){//Count!=7 i<2^3 for three motors
            if (mi<3){
                if ((Count>>((DataFromCAN[dwRel-1-i].ID>>4)-1))&1) // if motor have been read
                {
                    i++;
                    continue;
                }
                else
                {
                    Position_CAN[(DataFromCAN[dwRel-1-i].ID>>4)-1] = DataFromCAN[dwRel-1-i]; // Read the data in order
                    Count=(((Count>>((DataFromCAN[dwRel-1-i].ID>>4)-1))+1)<<((DataFromCAN[dwRel-1-i].ID>>4)-1))|Count; // Mark read motors
                    mi++;
                    i++;
                    continue;
                }
            }
        }
        for(i=0;i<3;i++){// i<3 for three motors       
            Position_Encoder[i] = Position_CAN[i].Data[4]<<24|Position_CAN[i].Data[5]<<16|Position_CAN[i].Data[6]<<8|Position_CAN[i].Data[7];
            Position[i] = Position_Encoder[i]*2*PI/2000;//Encoder data to radian
        }
    }
    //???
    else{
        ROS_INFO("[CANMotor] Error:Failed to get position of motors");
        ClMotors();
        CloseDevice(DeviceType,DeviceIndex);
    }
    return Position;
}

//private
DWORD CANMotor::OpCAN(){
    ROS_INFO("[CANMotor] Open device.");

    int nReserved =0;//meaningless
    DWORD dwRel;

    dwRel = OpenDevice(DeviceType, DeviceIndex, nReserved);
    if (dwRel != STATUS_OK)
    {
    ROS_INFO("Error:Failed to open device");
    }    
    return dwRel;
}

DWORD CANMotor::InCAN(){
    ROS_INFO("[CANMotor] Initialize CAN.");

    int nReserved =0;//meaningless
    DWORD dwRel;
    INIT_CONFIG InitConfig;
    
    InitConfig.Filter=0; /// Inable Filter
	InitConfig.AccCode=0x0000;
	InitConfig.AccMask=0xffff;
	InitConfig.Mode=0;   /// Normal Mode
    InitConfig.Timing0=0x00;/// CAN BaudRate 1000Kbps
	InitConfig.Timing1=0x14;/// CAN BaudRate 1000Kbps

    dwRel=InitCAN(DeviceType,DeviceIndex,CANIndex,&InitConfig);
    if(dwRel!=STATUS_OK)
    {
    ROS_INFO("Error:Failed to initialize CAN");
    }
    return dwRel;
}

DWORD CANMotor::StCAN(){
    ROS_INFO("[CANMotor] Start CAN.");

    DWORD dwRel;

    dwRel = StartCAN (DeviceType,DeviceIndex,CANIndex);

    if (dwRel == STATUS_ERR)
	{
		ROS_INFO("Error:Failed to start CAN");
   		CloseDevice(DeviceType,DeviceIndex);
	}

    sleep(1);// 1-1s    Transmit after StartCAN 10ms  
    return dwRel;
}

DWORD CANMotor::ClCAN(){
    ROS_INFO("[CANMotor] Clear buffer.");

    DWORD dwRel;

    dwRel = ClearBuffer(DeviceType,DeviceIndex,CANIndex);

    if (dwRel == STATUS_ERR)
	{
		ROS_INFO("[CANMotor] Error:Failed to clear buffer");
		CloseDevice(DeviceType,DeviceIndex);
	}	

    return dwRel;
}

void CANMotor::InData(){
    ROS_INFO("[CANMotor] initialize motors.");

    DWORD dwRel;

    BYTE Motor_InitData[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; //Data used in initialization for motor

    //Set data type of SendMotor(CAN_OBJ)
    SendMotor[0].DataLen=8;SendMotor[1].DataLen=8;SendMotor[2].DataLen=8; // Datalength 8
    SendMotor[0].SendType=0;SendMotor[1].SendType=0;SendMotor[2].SendType=0;// Transmission frame type 0-Normal 1-Omce 2-Self sending for test
    SendMotor[0].RemoteFlag=0;SendMotor[1].RemoteFlag=0;SendMotor[2].RemoteFlag=0;//Is remote 0-Data 1-Remote
    SendMotor[0].ExternFlag=0;SendMotor[1].ExternFlag=0;SendMotor[2].ExternFlag=0;//IS extern 0-Standard(11) 1-Extern(29)

    //Initial data for motor
    memcpy(SendMotor[0].Data,Motor_InitData,sizeof(Motor_InitData));
    memcpy(SendMotor[1].Data,Motor_InitData,sizeof(Motor_InitData));
    memcpy(SendMotor[2].Data,Motor_InitData,sizeof(Motor_InitData));

    //reset Motor???
    SendMotor[0].ID=0x00;
    SendMotor[1].ID=0x00;
    SendMotor[2].ID=0x00;
 
    sleep(1);
}

DWORD CANMotor::OpMotors(){
    ROS_INFO("[CANMotor] Open motors");

    DWORD dwRel;

    // Data setting of opening motors
    SendMotor[0].ID = 0x00;//To all motors in the group
    SendMotor[0].Data[0] = 0x00;//Number of Group-0
    SendMotor[0].Data[1] = 0x9A;//To One-to-many write data operation without response
    // Open motors
    SendMotor[0].Data[2] = 0x00;
    SendMotor[0].Data[3] = 0x00;
    SendMotor[0].Data[4] = 0x01;
    //Null data
    SendMotor[0].Data[5] = 0xFF;
    SendMotor[0].Data[6] = 0xFF;
    SendMotor[0].Data[7] = 0xFF;

    dwRel = Transmit(DeviceType, DeviceIndex, CANIndex,SendMotor,1);//1-SendData[0] only
    /*if (dwRel == STATUS_ERR)
	{
		ROS_INFO("Error:Failed to open motors");
	}
    */

   //Close motors if communication interrupt
    SendMotor[0].Data[2] = 0x1c;
    SendMotor[0].Data[3] = 0x00;
    SendMotor[0].Data[4] = 0x07;

    dwRel = Transmit(DeviceType, DeviceIndex, CANIndex,SendMotor,1);//1-SendData[0] only

    return dwRel;
}



}
