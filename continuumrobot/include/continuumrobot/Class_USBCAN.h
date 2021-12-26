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
         * @brief open CAN:open device-> initialization CAN->start CAN->clear buffer
         * 
         * @return DWORD 1-success 0-failure
         */
        DWORD OpenCAN();



        private:
        int nDeviceType = 3;//3-USBCAN I  4-USBCAN II
        int nDeviceInd = 0;//Device index
        int nReserved = 0;
        int nCANInd =0;//CAN index


    };
}

#endif