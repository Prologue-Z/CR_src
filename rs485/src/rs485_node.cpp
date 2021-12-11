/**
 * @file Node_ForceSensor.cpp
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#include <ros/ros.h>

#include <rs485/Class_ForceSensor.h>

//#include <serial/serial.h>
#include <fcntl.h>
#include <termios.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh("~");
    
    NS_ForceSensor::ForceSensor FS;
    ros::Rate Collect(1);
    double* ForceShow;
    while (1)
    {
        ForceShow = FS.GetForce();
        ROS_INFO_STREAM("[RS485]F1 = "<<ForceShow[0]<<"  F2 = "<<ForceShow[1]<<"  F3 = "<<ForceShow[2]);
        Collect.sleep();
    }    
    
    

    //test1
    /*
    serial::Serial ser;
    ser.setPort("/dev/ttyCH341USB0");
    ser.setBaudrate(9600);
    ser.open();
    if(ser.isOpen()){ 
            ROS_INFO_STREAM("[test] Serial Port initialized"); 
    } 
    unsigned char buff[8] = {0xaa,0xaa,0xaa,0x01,0xb1,0x00,0x00,0x1a};
    size_t ss = ser.write(buff,8);
    ROS_INFO_STREAM("[test]ss="<<ss);
    unsigned char *rbuff;
    size_t rs = ser.read(rbuff,30);
    ROS_INFO_STREAM("[test]rs="<<rs);
    */
    
   //test2
   /*
    int fd = open("/dev/ttyCH341USB0",O_RDWR | O_NOCTTY);
    set_opt(fd,9600,8,'N',1);
    unsigned char buff[8]={0xaa,0xaa,0xaa,0x01,0xb1,0x00,0x00,0x1a};
    size_t ss = write(fd,buff,8);
    ROS_INFO_STREAM("[test]ss="<<ss);
    usleep(500000);
    size_t rs = read(fd,buff,30);
    ROS_INFO_STREAM("[test]rs="<<rs);
    */
    
    return 0;
}

//test
/*
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    //保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
    if ( tcgetattr(fd,&oldtio) != 0)
    {
        perror("SetupSerial 1");
         return -1;
     }
    bzero( &newtio, sizeof( newtio ) );
    //步骤一，设置字符大小
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    //设置停止位
    switch( nBits )
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }
    //设置奇偶校验位
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
    //设置波特率
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
    //设置停止位
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    //设置等待时间和最小接收字符
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    //处理未接收字符
    tcflush(fd,TCIFLUSH);
    //激活新配置
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
} 
*/