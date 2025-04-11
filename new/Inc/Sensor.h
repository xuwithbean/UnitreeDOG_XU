#pragma once
#include <stdint.h>
#include <string>
#include "MultiThread.h"

class IMUReading
{
    public:
    float yaw,pitch,roll;
    float yawVel,pitchVel,rollVel;
    float accX,accY,accZ;
};

class IMU
{
    static void* _recvFunc(void* arg);
    protected:
    static int __doParser(uint8_t* parserBuffer,int& parserLen,uint8_t* msgBuffer);
    static void __doMoveBuffer(uint8_t* buffer,int totalLen,int offset);
    static void __doParserMsg(uint8_t* msgBuffer,int msgLen,IMUReading& reading);
public:
    IMUReading GetIMUData();
    IMU(float fwdX,float fwdY,float fwdZ);
    ~IMU();
    void OpenIMU(std::string serialName);
    void CloseIMU();

protected:
    THREAD m_threadDesc;
    MUTEX m_mutexDesc;
    IMUReading m_reading;
    int m_serialFd;
    bool m_sysExit;
    //Eigen::Matrix3f m_transform;
};
