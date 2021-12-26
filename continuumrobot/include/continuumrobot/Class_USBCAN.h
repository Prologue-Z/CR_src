/**
 * @file Class_USBCAN.h
 * @brief USBCAN model:GCAN USBCAN-Mini
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#ifndef CLASS_USBCAN_H
#define CLASS_USBCAN_H

#include "ECanVci.h"

namespace NS_USBCAN{
    class USBCAN{
        public:
        /**
         * @brief Construct a new USBCAN object
         * 
         */
        USBCAN();

        /**
         * @brief open CAN:open device -> initialization CAN -> start CAN
         * 
         * @return DWORD 1-success 0-failure
         */
        DWORD OpenCAN();

        /**
         * @brief close CAN:failure->close(while)
         * 
         * @return BOOL 1-success 0-failure
         */
        BOOL CloseCAN();

        /**
         * @brief clear buffer
         * 
         * @return DWORD 1-success 0-failure
         */
        DWORD ClearCAN();

        /**
         * @brief Send CAN_OBJ
         * 
         * @param Data CAN_OBJ[]
         * @param Num number of CAN_OBJ
         * @return DWORD number of CAN_OBJ  successfully sent
         */
        DWORD SendData(CAN_OBJ *Data,int Num);

        /**
         * @brief receive CAN_OBJ while GetReceiveNum = Num
         * 
         * @param Data CAN_OBJ[]
         * @param Num number of CAN_OBJ received
         * @param WaitTime unit ms
         * @return DWORD DWORD number of CAN_OBJ  successfully receive
         */
        DWORD ReceiveData(CAN_OBJ *Data,int Num,int WaitTime);

        /**
         * @brief print CAN_OBJ int(Data)
         * 
         * @param Data CAN_OBJ
         */
        void PrintCAN_OBJ(CAN_OBJ Data);

        private:
        int nDeviceType = 3;//3-USBCAN I  4-USBCAN II
        int nDeviceInd = 0;//Device index
        int nReserved = 0;
        int nCANInd =0;//CAN index

    };
}

#endif