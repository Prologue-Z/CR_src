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
//#include <fcntl.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs485_node");
    ros::NodeHandle nh("~");

    NS_ForceSensor::ForceSensor FS;

    //test1
    /*
    serial::Serial ser;
    ser.setPort("/dev/ttyCH341USB0");
    //ser.setBaudrate(9600);
    ser.open();
    if(ser.isOpen()){ 
            ROS_INFO_STREAM("[test] Serial Port initialized"); 
    } 
    unsigned char buff[8]={0xaa,0xaa,0xaa,0x01,0xb1,0x00,0x00,0x1a};
    size_t ss = ser.write(buff,8);
    ROS_INFO_STREAM("[test]ss="<<ss);
    */
    
   //test2
   /*
    int fd = open("/dev/ttyCH341USB0",O_RDWR | O_NOCTTY);
    unsigned char buff[8]={0xaa,0xaa,0xaa,0x01,0xb1,0x00,0x00,0x1a};
    size_t ss = write(fd,buff,8);
    ROS_INFO_STREAM("[test]ss="<<ss);
    */
    
    return 0;
}