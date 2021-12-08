/**
 * @file Class_ForceSensro.h
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

#include <list>

namespace NS_ForceSensor{
    class ForceSensor{
        public:

        /**
         * @brief Construct a new Force Sensor object
         * 
         */
        ForceSensor();

        private:
        const char* SerName = "/dev/ttyCH341USB0";
        const short BaudRate = 9600;
        
        serial::Serial Ser;
        
        /**
         * @brief set port->set baud rate->set time out->open serial
         * 
         * @return int 0-success -1-fail
         */
        int InitSer();



    };
}

#endif