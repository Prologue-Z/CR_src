/*
*  RS485_UartOperator.h
*
*  Saved on:2021.12.02
*    Author:Xu Zhang
* Institute:Key Laboratory of Mechanism Theory and Equipment Design of the Ministry of Education, School of Mechanical Engineering, Tianjin University, China
*
* Note:RS485 uart operator
*/

namespace UartOperator{
    struct UART_SETPARA{
        //reference-ttps://blog.csdn.net/weixin_43319854/article/details/109844860?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1.no_search_link&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1.no_search_link
        BAUD_RATE BaudRate;            
        unsigned char DataBit; 
        unsigned char StopBit; 
        unsigned char CheckBit; 
        unsigned char reserve[16];
    };
    
    enum BAUD_RATE{
        BR_1200,
        BR_2400,
        BR_4800,
        BR_9600,
        BR_14400,
        BR_19200,
        BR_38400,
        BR_57600,
        BR_115200,
        BR_230400,
        BR_380400,
        BR_460800,
        BR_921600,
        BR_BUTT,
    };

    class RS485_UartOperator{
        public:
        /*!
        * Constructor.
        * Open uart&Inituart
        */
        RS485_UartOperator(const struct UART_SETPARA *SetPara);

        ~RS485_UartOperator();

        /*!
        * SendData
        * @param Data Data used send
        * @param DataSize Send data size
        * @return Number of bits successfully sent
        */
        int SendData(unsigned char *Data,int DataSize);

        /*!
        * ReadData
        * @param Data Data used read
        * @param DataSize Read data size
        * @return Number of bits successfully read
        */
        int ReadData(unsigned char *Data,int DataSize);


        bool dataAvailable(int TimeOutMsec);

        private:
        const char* UartName = "/dev/ttyCH341USB0"; // Uart name
        int FD = 0;//File descriptor
        

        /**
         * InitUart - init tty device    
         */
        int InitUart(const struct UART_SETPARA *SetPara);

        /**
         * OpenUart - open tty device    
         */
        int OpenUart();
        
        /**
         * Set message format
         * 
         * @param BR       SetPara->BaudRate
         * @param DataBit  SetPara->DataBit
         * @param CheckBit SetPara->CheckBit
         * @param StopBit  SetPara->StopBit
         */
        int SetBaudRate(BAUD_RATE BR);
        int SetDataBit(unsigned char DataBit);//8，7，6，5
        int SetCheck(unsigned char CheckBit);// 0-None，1-Odd，2-Even,3-Space
        int SetStopBit(unsigned char StopBit);// '1'  '1.5'   '2'



        

    };
}