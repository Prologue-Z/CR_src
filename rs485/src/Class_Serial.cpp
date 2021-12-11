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
#include <termios.h>

namespace NS_Serial{
    //public
    Serial::Serial(){
        //O_RDWR-read&write   O_NOCTTY-can not controlled by the terminal
        fd = open(port,O_RDWR | O_NOCTTY);
        ROS_INFO_STREAM("[RS485]Open port successfully,fd = "<<fd);
        set_opt(fd,BaudRate,8,'N',1);
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

    int Serial::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
    {
        struct termios newtio,oldtio;
        /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
        if ( tcgetattr(fd,&oldtio) != 0)
        {
            perror("SetupSerial 1");
            return -1;
        }
        bzero( &newtio, sizeof( newtio ) );
        /*步骤一，设置字符大小*/
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        /*设置停止位*/
        switch( nBits )
        {
            case 7:
                newtio.c_cflag |= CS7;
                break;
            case 8:
                newtio.c_cflag |= CS8;
                break;
        }
    /*设置奇偶校验位*/
        switch( nEvent )
        {
            case 'O': //奇数
                newtio.c_cflag |= PARENB;
                newtio.c_cflag |= PARODD;
                newtio.c_iflag |= (INPCK | ISTRIP);
            break;
            case 'E': //偶数
                newtio.c_iflag |= (INPCK | ISTRIP);
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
            break;
                case 'N': //无奇偶校验位
                newtio.c_cflag &= ~PARENB;
            break;
        }
        /*设置波特率*/
        switch( nSpeed )
        {
            case 2400:
                cfsetispeed(&newtio, B2400);
                cfsetospeed(&newtio, B2400);
            break;
            case 4800:
                cfsetispeed(&newtio, B4800);
                cfsetospeed(&newtio, B4800);
            break;
            case 9600:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
            break;
            case 115200:
                cfsetispeed(&newtio, B115200);
                cfsetospeed(&newtio, B115200);
            break;
            case 460800:
                cfsetispeed(&newtio, B460800);
                cfsetospeed(&newtio, B460800);
            break;
            default:
                cfsetispeed(&newtio, B9600);
                cfsetospeed(&newtio, B9600);
            break;
        }
        /*设置停止位*/
        if( nStop == 1 )
            newtio.c_cflag &= ~CSTOPB;
        else if ( nStop == 2 )
            newtio.c_cflag |= CSTOPB;
        /*设置等待时间和最小接收字符*/ 
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;
        /*处理未接收字符*/
        tcflush(fd,TCIFLUSH);
        /*激活新配置*/
        if((tcsetattr(fd,TCSANOW,&newtio))!=0)
        {
            perror("com set error");
            return -1;
        }
        printf("[RS485]port set done!\n");
        return 0;
    } 
}