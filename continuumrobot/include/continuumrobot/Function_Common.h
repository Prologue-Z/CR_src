/**
 * @file Function_Common.h
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include "ECanVci.h"

namespace NS_CommonFunction{
    /**
     * @brief 
     * 
     * @param ReceiveMotor 
     * @return int 
     */
    int ByteToInt(CAN_OBJ ReceiveMotor);

    /**
     * @brief int->BYTE[2]
     * 
     */
    void IntToBYTE(int Int,unsigned char *byte);
}