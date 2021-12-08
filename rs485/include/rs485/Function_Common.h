/**
 * @file Function_Common.h
 * @brief common function
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

namespace CommonFunction{
    void ShortToChar(unsigned short Short,unsigned char *byte);

    void CharToShort(unsigned short Short,unsigned char *byte);

    unsigned char BCC(unsigned char* data,int length);
}