/**
 * @file Function_Common.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include "continuumrobot/Function_Common.h"
#include "ECanVci.h"

namespace NS_CommonFunction{
    int ByteToInt(CAN_OBJ ReceiveMotor){
        int Int = 0;
        Int |= ReceiveMotor.Data[7];
        Int |= ReceiveMotor.Data[6]<<8;
        Int |= ReceiveMotor.Data[4]<<16;
        Int |= ReceiveMotor.Data[3]<<24;
        return Int;
    }

    void IntToBYTE(int Int,unsigned char *byte){
        byte[0] = (Int >> 8) & 0xFF;
        byte[1] = Int & 0xFF;
    }
}