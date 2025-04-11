#include "InitAngle.h"
InitAngle::InitAngle 
    (   std::string portName0,
        std::string portName1,
        std::string portName2,
        std::string portName3)
:
    m_serialport0(portName0),
    m_serialport1(portName1),
    m_serialport2(portName2),
    m_serialport3(portName3)
{
    MOTOR_recv initrecv;
    initrecv = position_get(m_serialport0,0);
    m_initangle[0].shoulderHorizontal=initrecv.Pos;
    initrecv = position_get(m_serialport0,1);
    m_initangle[0].armRotation=initrecv.Pos;
    initrecv = position_get(m_serialport0,2);
    m_initangle[0].armFeetIntersect=initrecv.Pos;
    initrecv = position_get(m_serialport1,0);
    m_initangle[1].shoulderHorizontal=initrecv.Pos;
    initrecv = position_get(m_serialport1,1);
    m_initangle[1].armRotation=initrecv.Pos;
    initrecv = position_get(m_serialport1,2);
    m_initangle[1].armFeetIntersect=initrecv.Pos;
    initrecv = position_get(m_serialport2,0);
    m_initangle[2].shoulderHorizontal=initrecv.Pos;
    initrecv = position_get(m_serialport2,1);
    m_initangle[2].armRotation=initrecv.Pos;
    initrecv = position_get(m_serialport2,2);
    m_initangle[2].armFeetIntersect=initrecv.Pos;
    initrecv = position_get(m_serialport3,0);
    m_initangle[3].shoulderHorizontal=initrecv.Pos;
    initrecv = position_get(m_serialport3,1);
    m_initangle[3].armRotation=initrecv.Pos;
    initrecv = position_get(m_serialport3,2);
    m_initangle[3].armFeetIntersect=initrecv.Pos;
    printf("**********initanglestart************\n");
    for (int i = 0; i < 4; i++)
    {
        if(i<=2)
        {
            printf("AxisMovement(%f,%f,%f),\n",
            m_initangle[i].shoulderHorizontal,
            m_initangle[i].armRotation,
            m_initangle[i].armFeetIntersect);/* code */
        }
        if(i==3)
        {
            printf("AxisMovement(%f,%f,%f)\n",
            m_initangle[i].shoulderHorizontal,
            m_initangle[i].armRotation,
            m_initangle[i].armFeetIntersect);/* code */
        }
    }
    printf("**********initangleend************\n");
}