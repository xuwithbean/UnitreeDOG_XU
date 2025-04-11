#ifndef __INITSERIAL_H
#define __INITSERIAL_H
#include "SerialPort.h"
#include "motor_control.h"
#include "Kinematics.h"
class InitAngle
{
public:
    InitAngle(  std::string portName1,
                std::string portName2,
                std::string portName3,
                std::string portName4);
    AxisMovement m_initangle[4];
    MOTOR_recv initrecv;
protected:
    SerialPort m_serialport0;
    SerialPort m_serialport1;
    SerialPort m_serialport2;
    SerialPort m_serialport3;
};
#endif