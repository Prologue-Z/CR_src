/*
*  CommonFuncition.cpp
*
*  Saved on:2021.12.05
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
*/



namespace CommonFunction{
    void ShortToChar(unsigned short Short,unsigned char *byte){
        byte[0] = (Short >> 8) & 0xFF;
        byte[1] = Short & 0xFF;
    }

    unsigned char BCC(unsigned char* data,int length){
        unsigned char retval = 0;    
        for (int i = 0;i < length;i++) {
            retval ^= *data++;
        }
        return retval;
    }
}