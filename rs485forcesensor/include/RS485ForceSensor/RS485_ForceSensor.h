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

namespace ForceSensor_NS{
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

        /*!
        * Constructor.
        * Set bautrate->Set range->Set unit->Set polarity->Set zero point
        */
        RS485_ForceSensor();
        
        double* GetForce();


        private:
        double Force[3];        
        std::list<unsigned char *> SendDataList;//Data list to be sent
        unsigned char *ReadBuf;
        const unsigned char Trans_Add = 0x01;//Transmitter address
        const short ReadMsgSize = sizeof(unsigned char)*10*3;
        const short BaudRate = 3;//1-2400 2-4800 3-9600 4-19200 5-38400
        const short Range = 10;//Sensor range
        const short Unit = 2;//1-MPa 2-Kg 3-T
        const short Polarity = 2;//1-Unipolarity 2-Bipolarity

        UartOperator::RS485_UartOperator *Uart;
        
        /*!
        * AddFrame
        * Add a frame to the data to be transmitted
        * @param cmd descriptCommand bye
        * @param data date byte
        */
        int AddFrame(MSG_CMD cmd,int data);

        /*!
        * SendMsg
        * Send the first frame from SendDataList and delete it
        */
        void SendMsg();

        /*!
        * ReadForce
        * Send Single output ccommand ->Read force from uart -> HandleReMsg
        */
        void ReadForce();

        /*!
        * SetBaudRate
        */
        void SetBaudRate();

        /*!
        * SetRange
        */
        void SetRange();

        /*!
        * SetUnit
        */
        void SetUnit();

        /*!
        * SetPolarity
        */
        void SetPolarity();

        /*!
        * SetZeroPoint
        */
        void SetZeroPoint();

        /*!
        * HandleReMsg
        * Read buff->Force
        */
        void HandleReMsg();

    };
}