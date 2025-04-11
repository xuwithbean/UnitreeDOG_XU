
#include "Sensor.h"
#include <unistd.h>
#include "termios.h"
#include <stdlib.h>
#include <fcntl.h>
#include <iostream>
#include <strings.h>

using namespace std;

void IMU::__doMoveBuffer(uint8_t *buffer,int totalLen,int offset)
{
    for(int ftPtr = 0,mvPtr = offset;mvPtr < totalLen;++mvPtr,++ftPtr)
        buffer[ftPtr] = buffer[mvPtr];
}

int IMU::__doParser(uint8_t* parserBuffer,int& parserLen,uint8_t* msgBuffer)
{
    uint8_t readTest[512];
    int parserPtr = 0;
    uint8_t CK1 = 0,CK2 = 0;
    int headPtr = -1;
    uint8_t len;
    while(parserPtr < parserLen)
    {
        if(headPtr == -1)
        {
            if(parserBuffer[parserPtr++] == 0x59)
            {
                headPtr = parserPtr-1;
                if(parserPtr >= parserLen)break;
                if(parserBuffer[parserPtr++] != 0x53)
                {headPtr = -1;continue;}
            }else continue;

        }else
        {
            if(parserLen-parserPtr <= 3)break;
            CK1 =CK2= 0;
            CK1 += parserBuffer[parserPtr++];
            CK2 += CK1;
            CK1 += parserBuffer[parserPtr++];
            CK2 += CK1;
            len = parserBuffer[parserPtr++];
            CK1 += len;
            CK2 += CK1;
            if(parserLen-parserPtr < len+2)
                break;
            for(int ll = len;ll>0;ll--)
            {
                CK1 += parserBuffer[parserPtr++];
                CK2 += CK1;
            }
            if(CK1 == parserBuffer[parserPtr] && CK2 == parserBuffer[parserPtr+1])
            {
            //cout<<"recv msg"<<endl;
                for(int i = 0;i<len;i++)
                    msgBuffer[i] = parserBuffer[parserPtr-len+i];
                __doMoveBuffer(parserBuffer,parserLen,parserPtr);
                parserLen -= parserPtr;
                return len;
            }else
            {
                //int cc1 = 0,cc2 = 0;
                //for(int i = 0;i<parserPtr+2-headPtr;++i)
                //{
                    //printf("0x%02x",parserBuffer[i+headPtr]);
                  //  if(i > 1 && i < parserPtr-headPtr)
                  // {
                 //       cc1 = cc1+parserBuffer[i+headPtr];
                  //      printf("+");
//
                 //       cc2 += cc1;
                 //   }else printf(" ");
                //}

                //printf("\nCK1=0x%02x,CK2=0x%02x\n",CK1,CK2);
                //printf("\ncc1=0x%02x,cc2=0x%02x\n",cc1%256,cc2%256);
                parserPtr -= len+4;
                headPtr = -1;
            }
        }

    }
    if(headPtr == -1)parserLen = 0;
    else
    {
        __doMoveBuffer(parserBuffer,parserLen,headPtr);
        parserLen -= headPtr;
    }
    return 0;
}

void IMU::__doParserMsg(uint8_t *msgBuffer,int msgLen,IMUReading& reading)
{
    int i = 0;
    //cout<<"msg Len:"<<msgLen<<endl;
    //printf("msg:\n");
    //for(int j = 0;j<msgLen;++j)
     //  printf("0x%02x ",msgBuffer[j]);
    //printf("\n");

    while(i<msgLen)
    {
        uint8_t type = msgBuffer[i++];

        switch(type)
        {
            case 0x10://acceleration
            {
                //cout<<"acc"<<endl;
                if(msgBuffer[i] != 0x0c)break;
                i++;
                int number = 0;
                uint8_t* ptr = (uint8_t*)(&number);
                //printf("\1=0x%02x,2=0x%02x,3=0x%02x,4=0x%02x\n",msgBuffer[i],msgBuffer[i+1],msgBuffer[i+2],msgBuffer[i+3]);
                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];

                //if(number<0)number = 0;
                reading.accX = (float)number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.accY = number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.accZ = number * 1e-6;
            }break;
            case 0x20://angular velocity
            {
                if(msgBuffer[i] != 0x0c)break;
                i++;
                int number = 0;
                uint8_t* ptr = (uint8_t*)&number;
                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.pitchVel = number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.rollVel = number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.yawVel = number * 1e-6;
            }break;
            case 0x40://angle
            {
                if(msgBuffer[i] != 0x0c)break;
                i++;
                int number = 0;
                uint8_t* ptr = (uint8_t*)&number;
                //printf("\1=0x%02x,2=0x%02x,3=0x%02x,4=0x%02x\n",msgBuffer[i],msgBuffer[i+1],msgBuffer[i+2],msgBuffer[i+3]);
                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.pitch = number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.roll = number * 1e-6;

                ptr[0]= msgBuffer[i++];
                ptr[1]= msgBuffer[i++];
                ptr[2]= msgBuffer[i++];
                ptr[3]= msgBuffer[i++];
                reading.yaw = number * 1e-6;
            }break;
        }
    }
}

uint8_t rxBuffer[10240];
uint8_t parserBuffer[20480];
uint8_t msgBuffer[10240];

void* IMU::_recvFunc(void* arg)
{
    IMU* pThis = static_cast<IMU*>(arg);

    int parserLen = 0;
    int readLen = 0;
    while(!pThis->m_sysExit)
    {
        int rd = read(pThis->m_serialFd,rxBuffer,10240);
        //receive data from gyroscope
        if(parserLen)//if parser buffer is not empty ,parse the remain data first
        {
            int ret = __doParser(parserBuffer,parserLen,msgBuffer);
            //parsing the remain data
            if(ret != 0)
            {//if the parser has parsed a packet,parse the message segment
                //cout<<"do msg"<<endl;
                mutex_lock(pThis->m_mutexDesc);
                //cout<<"msg in "<<endl;
                __doParserMsg(msgBuffer,ret,pThis->m_reading);
                //cout <<"msg out"<<endl;
                mutex_unlock(pThis->m_mutexDesc);
            }
        }
        //if(rd+parserLen > 20480)cout<<"full"<<endl;
        for(int i = 0;i<rd;++i)
            parserBuffer[parserLen + i] = rxBuffer[i];
        parserLen += rd;
    }
    close(pThis->m_serialFd);
    pThis->m_serialFd = -1;
    return 0;
}

IMUReading IMU::GetIMUData()
{
    mutex_lock(m_mutexDesc);
    IMUReading reading = m_reading;
    mutex_unlock(m_mutexDesc);
    return reading;
}

void IMU::OpenIMU(std::string serialName)
{
    if(m_serialFd != -1)return;
    m_serialFd = open(serialName.c_str(),O_RDONLY);
    termios config;
    tcgetattr(m_serialFd,&config);
    bzero(&config,sizeof(config));
    config.c_cflag |= CLOCAL | CREAD;

    cfsetospeed(&config,B460800);
    cfsetispeed(&config,B460800);

    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_iflag &= ~INPCK;
    tcsetattr(m_serialFd,TCSANOW,&config);
    tcflush(m_serialFd, TCIOFLUSH);
    if(m_serialFd < 0)
    {
        cout<<"[IMU]open failed"<<endl;
        return;
    }
    printf("[IMU]open successfully\n");
    m_sysExit = false;

    thread_create(IMU::_recvFunc,this,m_threadDesc);

}

void IMU::CloseIMU()
{
if(m_sysExit)return;
    m_sysExit = true;
    if(m_serialFd != -1)
        close(m_serialFd);

    thread_join(m_threadDesc);
}

#include <string.h>

IMU::IMU(float fwdX, float fwdY, float fwdZ)
{
m_serialFd = -1;
    m_sysExit = true;
    m_threadDesc = -1;
    mutex_create(m_mutexDesc);
    memset(&m_reading,0,sizeof(IMUReading));

}

IMU::~IMU()
{
    if(m_serialFd != -1)
        close(m_serialFd);
    cout<<"closed"<<endl;
}
