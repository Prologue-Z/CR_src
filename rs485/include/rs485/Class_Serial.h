/**
 * @file Class_Serial.h
 * @brief 
 * 
 * @author Zhang Xu (tjuzx2016@126.com)
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021 Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
 */

#ifndef CLASS_SERIAL_H
#define CLASS_SERIAL_H
#include <ros/ros.h>
#include <string>

namespace NS_Serial{
    class Serial{
        public:
        /**
         * @brief Construct a new Serial object
         * 
         */
        Serial();        

        /**
         * @brief send data to serial port
         * 
         * @param data A const reference containing the data to be written to the serial port.
         * @param size 	A size_t that indicates how many bytes should be written from the given data buffer.

         * @return size_t representing the number of bytes actually written to the serial port.
         */
        size_t swrite(unsigned char* data,size_t size);

        /**
         * @brief read data from serial poet
         * 
         * @param buffer An unsigned char array of at least the requested size.
         * @param size 	A size_t defining how many bytes to be read
         * @return size_t representing the number of bytes read as a result of the call to read.
         */
        size_t sread(unsigned char* buffer,size_t size);

        /**
         * @brief close the port
         * 
         */
        void sclose();

        private:
        int fd;
        const char* port = "/dev/ttyCH341USB0";       
    };
}

#endif