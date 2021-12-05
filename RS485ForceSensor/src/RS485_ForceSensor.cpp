/*
*  RS485_ForceSensor.cpp
*
*  Saved on:2021.12.04
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Transmitter model:ZNBSQ-TS3-485
*/

#include <RS485ForceSensor/RS485_ForceSensor.h>

#include <RS485ForceSensor/CommonFuncition.h>

#include <stdlib.h>
#include <string.h>

namespace ForceSensor{

    //private
    int RS485_ForceSensor::AddFrame(MSG_CMD cmd,int data){
        unsigned char *pNode = NULL;
        unsigned char NewFrame[8] = {0xAA,0xAA,0xAA,0x01};
        
        NewFrame[SFI_Cmd] = cmd;
        CommonFunction::ShortToChar(data, &NewFrame[SFI_Data]);

        NewFrame[SFI_Check] = CommonFunction::BCC(NewFrame,7);

        pNode = (unsigned char *)malloc(sizeof(NewFrame)); /* 发送线程会free掉它 */
        memcpy(pNode, NewFrame, sizeof(NewFrame));

        SendDataList.push_back(NewFrame);
        return 0;
    }
}
