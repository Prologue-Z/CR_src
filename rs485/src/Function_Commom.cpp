/**
 * @file Function_Commom.cpp
 * @brief common function
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <rs485/Function_Common.h>

namespace NS_CommonFunction{
    void ShortToChar(unsigned short Short,unsigned char *byte){
        byte[0] = (Short >> 8) & 0xFF;
        byte[1] = Short & 0xFF;
    }

    void CharToShort(unsigned short Short,unsigned char *byte){
        Short |= byte[1];
        Short |= byte[0] << 8;
    }

    unsigned char BCC(unsigned char* data,int length){
        unsigned char retval = 0;    
        for (int i = 0;i < length;i++) {
            retval ^= *data++;
        }
        return retval;
    }
}