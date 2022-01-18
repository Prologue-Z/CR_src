/**
 * @file Class_ForceSensor.h
 * @brief Transmitter:ZNLBSQ-TS3-485
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-07
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#ifndef CLASS_FORCESENSOR_H
#define CLASS_FORCESENSOR_H

#include <serial/serial.h>

namespace NS_ForceSensor{
    class ForceSensor{
        public:

        /**
         * @brief Construct a new Force Sensor object and init it
         * 
         */
        ForceSensor();

        /**
         * @brief Get the Force object
         * 
         * @return double* Forece[3]
         */
        double* GetForce();

        private:
        unsigned char SendFrame[8] = {0xaa,0xaa,0xaa};
        double Force[3] = {0,0,0};
        
        serial::Serial Ser;

        enum MSG_CMD{
            MC_BaudRate = 0xA2,
            MC_Range = 0xA3,
            MC_Unit = 0xA5,
            MC_Polarity = 0xA6,
            MC_ZeroPoint = 0xA7,
            MC_SingleOutput = 0xB1,
        };
        //Send frame index
        enum SEND_FRAME_INDEX{
            SFI_Head = 0,
            SFI_Add = 3,
            SFI_Cmd = 4,
            SFI_Data = 5,
            SFI_Check = 7,
        };
        //Receive frame index
        enum RECEIVE_FRAME_INDEX{
            RFI_Head = 0,
            RFI_Add = 3,
            RFI_Cmd = 4,
            RFI_Data = 5,
            RFI_Decimal = 7,
            RFI_Unit = 8,
            RFI_Check = 9,
            RFI_FrameNum = 10,
        };
        
        /**
         * @brief initialize transmitter:Set Ser->open Ser->set baud rate of sensor
         * 
         * @return int 0-success -1-failure
         */
        int InitTrans();

        /**
         * @brief get a frame date to renew Sendframe
         * 
         * @param add add index
         * @param cmd - command type
         * @param data - parameters
         */
        void RenewSFrame(unsigned short add,MSG_CMD cmd,unsigned short data);

        /**
         * @brief send SendFrame
         * 
         * @return Bytes read successfully
         */
        size_t SendMsg();

        /**
         * @brief read 3 frame(10 byte) from buffer and handle it to Force
         * 
         * @param i address of data
         * @return size_t Bytes sent successfully
         */
        size_t ReadMsg(int i);

        /**
         * @brief handle data from buffer to force
         * 
         * @param ReBuff a frame data(unsigned char)
         * @return data(unsigned char) to F(N)
         */
        double Handledata(unsigned char *ReBuff);
    };
}

#endif