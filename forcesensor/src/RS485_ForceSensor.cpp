/*
*  RS485_ForceSensor.cpp
*
*  Saved on:2021.12.04
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Transmitter model:ZNBSQ-TS3-485
*/

#include <forcesensor/RS485_ForceSensor.h>

#include <forcesensor/RS485_UartOperator.h>

#include <forcesensor/CommonFuncition.h>

#include <stdlib.h>
#include <string.h>
using namespace UartOperator;

namespace ForceSensor_NS{
    //public
    RS485_ForceSensor::RS485_ForceSensor()
    :SetPara({(BAUD_RATE)3,8,'1',0}),Uart(&SetPara) {
        SetBaudRate();
        SetRange();
        SetUnit();
        SetPolarity();
        SetZeroPoint();
    }

    double* RS485_ForceSensor::GetForce(){
        ReadForce();
        return Force;
    }
    
    //private
    int RS485_ForceSensor::AddFrame(MSG_CMD cmd,int data){
        unsigned char *pNode = NULL;
        unsigned char NewFrame[8] = {0xAA,0xAA,0xAA};
        NewFrame[SFI_Add] = Trans_Add;
        
        NewFrame[SFI_Cmd] = cmd;
        CommonFunction::ShortToChar(data, &NewFrame[SFI_Data]);

        NewFrame[SFI_Check] = CommonFunction::BCC(NewFrame,7);

        pNode = (unsigned char *)malloc(sizeof(NewFrame)); /* 发送线程会free掉它 */
        memcpy(pNode, NewFrame, sizeof(NewFrame));

        SendDataList.push_back(NewFrame);
        return 0;
    }

    void RS485_ForceSensor::SendMsg(){
        unsigned char *SenBufNode = NULL;
        SenBufNode = SendDataList.front();
        SendDataList.pop_front();

        Uart.SendData(SenBufNode,8);
        delete SenBufNode;
    }

    void RS485_ForceSensor::ReadForce(){
        AddFrame(MC_SingleOutput,0);
        SendMsg();

        ReadBuf = new unsigned char[ReadMsgSize];
        Uart.ReadData(ReadBuf,ReadMsgSize);

        HandleReMsg();
    }
    

    void RS485_ForceSensor::SetBaudRate(){
        AddFrame(MC_BaudRate,BaudRate);
        SendMsg();
    }

    void RS485_ForceSensor::SetRange(){
        AddFrame(MC_Range,Range);
        SendMsg();
    }

    void RS485_ForceSensor::SetUnit(){
        AddFrame(MC_Unit,Unit);
        SendMsg();
    }

    void RS485_ForceSensor::SetPolarity(){
        AddFrame(MC_Polarity,Polarity);
        SendMsg();
    }

    void RS485_ForceSensor::SetZeroPoint(){
        AddFrame(MC_ZeroPoint,0);
        SendMsg();
    }

    void RS485_ForceSensor::HandleReMsg(){
        unsigned short Temp[3];
        for(int i=0;i<3;i++){
            CommonFunction::CharToShort(Temp[i],&ReadBuf[RFI_Data+i*RFI_FrameNum]);
            Force[i] = Temp[i]*10^ReadBuf[RFI_Decimal+i*RFI_FrameNum];
        }
    }

}
