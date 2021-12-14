/**
 * @file Function_Common.h
 * @brief common function
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

namespace NS_CommonFunction{
     /**
     * @brief unsigned short->unsigned char[2]
     * 
     * @param Short input data
     * @param byte outputdata
     */
    void ShortToChar(unsigned short Short,unsigned char *byte);

    /**
     * @brief unsigned char[2]->short
     * 
     * @param byte input data
     * @return output data
     */
    short CharToShort(unsigned char* byte);

    /**
     * @brief BCC check
     * 
     * @param data 
     * @param length BCC data length
     * @return unsigned char 
     */
    unsigned char BCC(unsigned char* data,int length);
}