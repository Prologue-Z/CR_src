/**
 * @file Class_Serial.cpp
 * @brief serial open write read
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <rs485/Class_Serial.h>
#include <fcntl.h>

namespace NS_Serial{
    //public
    Serial::Serial(){
        //O_RDWR-read&write   O_NOCTTY-can not controlled by the terminal
        fd = open(port,O_RDWR | O_NOCTTY);
        tcflush(DevFd, TCIFLUSH);//clear buffer
    }

    size_t Serial::swrite(unsigned char* data,size_t size){
        size_t ws = write(fd,data,size);
        return ws;
    }

    size_t Serial::sread(unsigned char* buffer,size_t size){
        size_t rs = read(fd,buffer,size);
        return rs;
    }

    void Serial::sclose(){
        close(fd);
    }
}