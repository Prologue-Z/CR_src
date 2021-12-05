/*
*  RS485_ForceSensor.h
*
*  Saved on:2021.12.04
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Transmitter model:ZNBSQ-TS3-485
*/

//Uart operator
#include <RS485ForceSensor/RS485_UartOperator.h>

#include <list>

namespace ForceSensor{
    


    class RS485_ForceSensor{
        public:
        enum MSG_CMD{
            MC_BaudRate = 0xA2,
            MC_Range = 0xA3,
            MC_Unit = 0xA5,
            MC_Polarity = 0xA6,
            MC_ZeroPoint = 0xA7,
            MC_SingleOutput = 0xB1,
        };
        enum SEND_FRAME_INDEX{
            SFI_Head = 0,
            SFI_Add = 3,
            SFI_Cmd = 4,
            SFI_Data = 5,
            SFI_Check = 7,
        };


        



        private:
        double Force[3];
        unsigned char Trans_Add = 0x01;
        std::list<unsigned char *> SendDataList;
        short BaudRate = 3;//1-2400 2-4800 3-9600 4-19200 5-38400

        UartOperator::RS485_UartOperator *Uart;

        int AddFrame(MSG_CMD cmd,int data);

        void SetBaudRate();

    };
}