/*
*  RS485_UartOperator.cpp
*
*  Saved on:2021.12.02
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:RS485 uart operator
* without lock.h
*/

//ROS
#include <ros/ros.h>

//RS485_UartOperator
#include <RS485ForceSensor/RS485_UartOperator.h>

//open
#include <fcntl.h>

//
#include <termios.h>

namespace UartOperator{    
    //public
    RS485_UartOperator::RS485_UartOperator(const struct UART_SETPARA *SetPara){
        InitUart(SetPara); 
        
        
    }

    int RS485_UartOperator::SendData(unsigned char *Data,int DataSize){
        if (NULL == Data){
            ROS_INFO("Get Null pointer, Check!!!");
            return -1;
        }

        int WriteCount = 0;
        WriteCount = write(FD,Data,DataSize);
        
        if (WriteCount != DataSize){
            ROS_INFO("[RS485 Error]:write err, writeCount=%d, writeSize=%d \n", WriteCount, DataSize);
            return -1;
        }
        return 0;
    }

    int RS485_UartOperator::ReadData(unsigned char *Data,int DataSize){
        if(NULL==Data){
            ROS_INFO("[RS485 Error]:Get Null pointer, Check!!!");
            return -1;
        }

        int ReadCount = 0;
        ReadCount = read(FD,Data,DataSize);

        if (ReadCount < 0)    {
            ROS_INFO("[RS485 Error]:read serial err, fd=%d  errno= %s Check!!!\n", FD, strerror(errno));
            return -1;
        }
        return ReadCount;
    }

    bool RS485_UartOperator::dataAvailable(int TimeOutMsec){
        struct timeval timeout;
        fd_set readfds;

        if (TimeOutMsec == 0){
            // no waiting
            timeout.tv_sec = 0;
            timeout.tv_usec = 0;
        } 
        else{
            timeout.tv_sec = TimeOutMsec / 1000;
            timeout.tv_usec = (TimeOutMsec % 1000) * 1000;
        }

        FD_ZERO(&readfds);
        FD_SET(FD, &readfds);

        if (select(FD + 1, &readfds, NULL, NULL, &timeout) > 0){
            return true; // data is ready
        } 
        else{
            return false; // no data
        }
    }

    //private    
    int RS485_UartOperator::InitUart(const struct UART_SETPARA *SetPara){
        //open uart
        FD = OpenUart();
        if(FD<0) return -1;

        //Clear old setting
        struct termios OldParm;
	    bzero(&OldParm, sizeof(OldParm));
        tcsetattr(FD, TCSANOW, &OldParm);

        //Set baud rate
        if(SetBaudRate(SetPara->BaudRate) !=0){
            ROS_INFO("[RS485 Error]:SetBaudRate(%d) fail! \n", SetPara->BaudRate);
            close(FD);
            return -1;
        }

        //Set message format
        if(SetDataBit(SetPara->DataBit) !=0){//Set data bit
            ROS_INFO("[RS485 Error]:SetDataBit fail!");
            close(FD);
            return -1;
        }
        if(SetCheck(SetPara->CheckBit) != 0){//Set check bit
            ROS_INFO("[RS485 Error]:SetCheck fail! \n");
            close(FD);
            return -1;
        }
        if (SetStopBit(SetPara->StopBit) != 0){//Set stop bit
            ROS_INFO("[RS485 Error]:SetStopBit fail! \n");
            close(FD);
            return -1;
        }

        return 0;
    }

    int RS485_UartOperator::OpenUart(){
        //Open uart O_RDWR-read&write   O_NOCTTY-can not controlled by the terminal
        FD = open(UartName,O_RDWR | O_NOCTTY);
        if(FD<0){
            ROS_INFO("[RS485 Error]:Open dev %s fail! \n",UartName);
            return -1;
        }
        ROS_INFO("[RS485]:Open dev %s success! \n",UartName);
        return 0;
    }

    int RS485_UartOperator::SetBaudRate(BAUD_RATE BR){
        int s32Speed = 0;    
        int s32SpeedPrint = 0;  
        
        switch(BR)
        {
            case BR_1200:
            {
                s32Speed = B1200;
                s32SpeedPrint = 1200;
                break;
            }
            case BR_2400:
            {
                s32Speed = B2400;
                s32SpeedPrint = 2400;
                break;
            }
            case BR_4800:
            {
                s32Speed = B4800;
                s32SpeedPrint = 4800;
                break;
            }
            case BR_9600:
            {
                s32Speed = B9600;
                s32SpeedPrint = 9600;
                break;
            }
            case BR_19200:
            {
                s32Speed = B19200;
                s32SpeedPrint = 19200;
                break;
            }
            case BR_38400:
            {
                s32Speed = B38400;
                s32SpeedPrint = 38400;
                break;
            }
            case BR_57600:
            {
                s32Speed = B57600;
                s32SpeedPrint = 57600;
                break;
            }
            case BR_115200:
            {
                s32Speed = B115200;
                s32SpeedPrint = 115200;
                break;
            }
            case BR_230400:
            {
                s32Speed = B230400;
                s32SpeedPrint = 230400;
                break;
            }
            case BR_460800:
            {
                s32Speed = B460800;
                s32SpeedPrint = 460800;
                break;
            }
            case BR_921600:
            {
                s32Speed = B921600;
                s32SpeedPrint = 921600;
                break;
            }
            case BR_14400:
            case BR_380400:
            default:
            {
                ROS_INFO("[RS485 Error]:Unsupport Baud Rate %d \n",BR);
                return -1;
            }      
        }
        
        struct termios stTermios;
        tcgetattr(FD,&stTermios);                      /* 获取串口参数 */
        
        tcflush(FD,TCIOFLUSH);                         /* 先清空输入、输出缓冲区 */
        
        cfsetispeed(&stTermios, s32Speed);             /* 设置波特率 */
        cfsetospeed(&stTermios, s32Speed);
        stTermios.c_cflag |= (CLOCAL | CREAD);
        
        if (tcsetattr(FD, TCSANOW, &stTermios) != 0)   /* 设置串口参数 */
        {
            ROS_INFO("[RS485 Error]:tc set attr error!!!\n ");
            return -1;
        }
        tcflush(FD,TCIOFLUSH);                        /* 再次清空输入、输出缓冲区 */

        ROS_INFO("[RS485]:Set Baud Rate %d \n", s32SpeedPrint);
        return 0;
    }

    int RS485_UartOperator::SetDataBit(unsigned char DataBit){
        struct termios stTermios;
        tcgetattr(FD, &stTermios); /* 获取串口参数 */

        stTermios.c_cflag &= ~CSIZE;  /* 屏蔽字符大小位 */
        int DataBitPrint = 0;
        switch(DataBit)
        {
            case 8:
            {
                stTermios.c_cflag |= CS8;
                DataBitPrint = 8;
                break;
            }
            case 7:
            {
                stTermios.c_cflag |= CS7;
                DataBitPrint = 7;
                break;
            }
            case 6:
            {
                stTermios.c_cflag |= CS6;
                DataBitPrint = 6;
                break;
            }
            case 5:
            {
                stTermios.c_cflag |= CS5;
                DataBitPrint = 5;
                break;
            }
            default:
            {
                ROS_INFO("[RS485 Error]:Unknow Data Bit %d ,Check!!! \n", DataBit);
                return -1;
            }       
        }
        
        tcflush(FD, TCIFLUSH); /* 先清空输入缓冲区 */
        if (tcsetattr(FD, TCSANOW, &stTermios) != 0)  /* 设置串口参数 */
        {
            ROS_INFO("[RS485 Error]:Set data bit error!!!");
            return -1;
        }
        
        ROS_INFO("[RS485]:Set Data Bit = %d\n", DataBitPrint);
        return 0;   
    }

    int RS485_UartOperator::SetCheck(unsigned char CheckBit){
        unsigned char aCheckPrint[10] = {0};
        struct termios stTermios;
        tcgetattr(FD, &stTermios); /* 获取串口参数 */
        
        stTermios.c_cflag &= ~PARENB;    /* Clear parity enable */
        stTermios.c_iflag &= ~INPCK;	 /* Enable parity checking */
        stTermios.c_iflag &= ~CMSPAR;
        
        switch(CheckBit)
        {
            case 0: // None
            {
                stTermios.c_cflag &= ~PARENB;  /* Clear parity enable */
                stTermios.c_iflag &= ~INPCK;   /* Enable parity checking */
                memcpy(aCheckPrint, "None", sizeof(aCheckPrint));
                break;
            }
            case 1: // Odd
            {
                stTermios.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/ 
                stTermios.c_iflag |= INPCK;              /* Disnable parity checking */ 
                memcpy(aCheckPrint, "Odd", sizeof(aCheckPrint));
                break;
            }
            case 2: // Even
            {
                stTermios.c_cflag |= PARENB;     /* Enable parity */    
                stTermios.c_cflag &= ~PARODD;    /* 转换为偶效验*/     
                stTermios.c_iflag |= INPCK;      /* Disnable parity checking */
                memcpy(aCheckPrint, "Even", sizeof(aCheckPrint));
                break;
            }
            case 3: // Space
            {
                stTermios.c_cflag &= ~PARENB;  
                stTermios.c_cflag &= ~CSTOPB;
                memcpy(aCheckPrint, "Space", sizeof(aCheckPrint));
                break;
            }
            default:
            {
                ROS_INFO("[RS485 Error]:Unknow Check Param %d , Check!!! \n", CheckBit);
                return -1;
            }       
        }
        
        tcflush(FD, TCIFLUSH); /* 先清空输入缓冲区 */
        if (tcsetattr(FD, TCSANOW, &stTermios) != 0)  /* 设置串口参数 */
        {
            ROS_INFO("[RS485 Error]:tc set attr error!!!");
            return -1;
        }
        
        ROS_INFO("[RS485]:Set Check(%s)\n", aCheckPrint);
        return 0;
    }

    int RS485_UartOperator::SetStopBit(unsigned char StopBit){
        int StopBitPrint = 0;
        struct termios stTermios;
        tcgetattr(FD, &stTermios); /* 获取串口参数 */
        switch(StopBit)
        {
            case '1':
            {
                stTermios.c_cflag &= ~CSTOPB;
                StopBitPrint = 1;
                break;
            }
            case '2':
            {
                stTermios.c_cflag |= CSTOPB;
                StopBitPrint = 2;
                break;
            }
            case '1.5':
            default:
            {
                ROS_INFO("[RS485 Error]:Unknow Stop Bit %d ,Check!!! \n",StopBit);
                return -1;
            }       
        }
        
        tcflush(FD, TCIFLUSH); /* 先清空输入缓冲区 */
        if (tcsetattr(FD, TCSANOW, &stTermios) != 0)  /* 设置串口参数 */
        {
            ROS_INFO("[RS485 Error]:tc set attr error!!!");
            return -1;
        }
        
        ROS_INFO("[RS485]:Set Stop Bit %d \n",StopBitPrint);
        return 0;
    }
}



