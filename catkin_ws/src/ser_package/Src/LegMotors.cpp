#include "LegMotors.h"
#include <iostream>
#include <fstream>
#include "Debug.h"
using namespace std;
//void __init();

float LegMotors::ms_motorScalar;
bool LegMotors::ms_systemSafe = true;
std::vector<LegMotors*> LegMotors::ms_registerMotor;

#define DEG(deg)    deg*3.141592653589/180
#define RAD(rad)    rad*180/3.141592653589

#define SAFE_TRAP(msg,info)      {\
                                ms_systemSafe = false;\
                                int motorID;\
                                for(motorID = 0;motorID < ms_registerMotor.size();motorID++)\
                                {if(this == ms_registerMotor[motorID])break;}\
                                while(true){cout<<"[LegMotor]:system unsafe!!!motor:"<<m_name<<msg<<"@"<<info<<endl;usleep(10000000);}\
                            }

void LegMotors::_checkParam(float kp,float kw)
{
    while(!ms_systemSafe)usleep(10000000);
    const float maxKP = 3;
    const float maxKW = 20;

    if(kp <0)SAFE_TRAP("[LegMotor]:kp小于0！",kp);
    if(kp > maxKP)SAFE_TRAP("[LegMotor]:kp过大！",kp);
    if(!(kp>=0 && kp<maxKP))
        SAFE_TRAP("[LegMotor]:kp!",kp);

    if(kw<0)SAFE_TRAP("[LegMotor]:kw小于0！",kw);
    if(kw>maxKW)SAFE_TRAP("[LegMotor]:kw过大!",kw);
    if(!(kw>=0 && kw<=maxKW))
        SAFE_TRAP("[LegMotor]:kw!",kw);
}

void LegMotors::_checkPos(float shoulderPos,float armPos,float feetPos)
{
    const float safetyAngle = DEG(120);
    while(!ms_systemSafe)usleep(10000000);

    float deltaAngle = fabsf(m_currentAngle.shoulderHorizontal-shoulderPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:1电机控制角度相差过大!",deltaAngle);
    deltaAngle = fabsf(m_currentAngle.armRotation-armPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:2电机控制角度相差过大!",deltaAngle);
    deltaAngle = fabsf(m_currentAngle.armFeetIntersect - feetPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:3电机控制角度相差过大!",deltaAngle);
}
void LegMotors::_checkRange(float shoulderAngle,float armAngle,float feetAngle)
{
    const float safetyShoulder[2] = {DEG(-40),DEG(40)};
    const float safetyArm[2] = {DEG(30),DEG(240)};
    const float safetyFeet[2] = {DEG(15),DEG(140)};
    while(!ms_systemSafe);

    if(shoulderAngle < safetyShoulder[0])
        SAFE_TRAP("[LegMotor]:shoudler angle, too small!",shoulderAngle);
    if(shoulderAngle > safetyShoulder[1])
        SAFE_TRAP("[LegMotor]:shoudler angle, too large!",shoulderAngle);
    if(!(shoulderAngle > safetyShoulder[0] && shoulderAngle < safetyShoulder[1]))
        SAFE_TRAP("[LegMotor]:shoudler angle!",shoulderAngle);

    if(armAngle < safetyArm[0])
        SAFE_TRAP("[LegMotor]:arm angle, too small!",armAngle);
    if(armAngle > safetyArm[1])
        SAFE_TRAP("[LegMotor]:arm angle, too large!",armAngle);
    if(!(armAngle > safetyArm[0] && armAngle < safetyArm[1]))
        SAFE_TRAP("[LegMotor]:arm angle!",armAngle);

    if(feetAngle < safetyFeet[0])
        SAFE_TRAP("[LegMotor]:feet angle, too small!",feetAngle);
    if(feetAngle > safetyFeet[1])
        SAFE_TRAP("[LegMotor]:feet angle, too large!",feetAngle);
    if(!(feetAngle > safetyFeet[0] && feetAngle < safetyFeet[1]))
        SAFE_TRAP("[LegMotor]:feet angle!",feetAngle);
}

LegMotors::LegMotors(AxisMovement zeroPos,std::vector<float> motorSign,float motorScalar,std::string portName,int id)
:m_serial(portName),
m_motorSign(motorSign),
m_zeros(zeroPos),
m_name(portName),
m_params(0.2,6),
m_id(id)
{
    MOTOR_recv recv = position_get(m_serial, 0);
    m_currentAngle.shoulderHorizontal = m_motorSign[0] * (recv.Pos - m_zeros.shoulderHorizontal) / ms_motorScalar;
    m_currentTorque.shoulderTorque = m_motorSign[0]*recv.T;
    printf("0:%f,",recv.Pos);
    recv = position_get(m_serial, 1);
    m_currentAngle.armRotation = m_motorSign[1] * (recv.Pos - m_zeros.armRotation) / ms_motorScalar;
    m_currentTorque.armTorque = m_motorSign[1]*recv.T;
    printf("1:%f,",recv.Pos);
    recv = position_get(m_serial, 2);
    m_currentAngle.armFeetIntersect = m_motorSign[2] * (recv.Pos - m_zeros.armFeetIntersect) / ms_motorScalar;
    m_currentTorque.armTorque = m_motorSign[2]*recv.T;
    printf("2:%f\n",recv.Pos);
    m_params.k_p = 0.05;
    m_params.k_w = 4;
    
}


// #include <Windows.h>
// HANDLE log_mutex;
// fstream file;
// bool init = false;

// void __init()
// {
//     if (init)return;
//     file.open("./leg.txt", ios::out);
//     log_mutex = CreateMutex(NULL, FALSE, NULL);
//     init = true;
// }
void LegMotors::PositionCtrl(float shoulderAngle,float armAngle,float armFeetInterAngle)
{
    //_checkPos(shoulderAngle, armAngle, armFeetInterAngle);
    //_checkRange(shoulderAngle, armAngle, armFeetInterAngle);
    //_checkParam(m_params.k_p,m_params.k_w);
    //OUT("kp:%.3f,kw:%.3f\n",m_params.k_p,m_params.k_w);
    //usleep(12000);//
    //Debug::Record(m_id,shoulderAngle,armAngle,armFeetInterAngle);
    //printf("%f,%f,%f\n",shoulderAngle,armAngle,armFeetInterAngle);
    //return;
    _checkPos(shoulderAngle, armAngle, armFeetInterAngle);
    _checkRange(shoulderAngle, armAngle, armFeetInterAngle);
    _checkParam(m_params.k_p,m_params.k_w);

    /*
    MOTOR_recv recv;
    recv = postion_control(m_serial,0,m_motorSign[0]*shoulderAngle*ms_motorScalar+m_zeros.shoulderHorizontal);
    m_currentAngle.shoulderHorizontal = m_motorSign[0]*(recv.Pos-m_zeros.shoulderHorizontal)/ms_motorScalar;

    recv = postion_control(m_serial,1,m_motorSign[1]*armAngle*ms_motorScalar+m_zeros.armRotation);
    m_currentAngle.armRotation = m_motorSign[1]*(recv.Pos-m_zeros.armRotation)/ms_motorScalar;

    recv = postion_control(m_serial,2,m_motorSign[2]*armFeetInterAngle*ms_motorScalar+m_zeros.armFeetIntersect);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;
    */
    MOTOR_recv recv;
    recv = position_control_custom(m_serial,0,m_motorSign[0]*shoulderAngle*ms_motorScalar+m_zeros.shoulderHorizontal,m_params.k_p,m_params.k_w);
    m_currentAngle.shoulderHorizontal = m_motorSign[0]*(recv.Pos-m_zeros.shoulderHorizontal)/ms_motorScalar;

    recv = position_control_custom(m_serial,1,m_motorSign[1]*armAngle*ms_motorScalar+m_zeros.armRotation,m_params.k_p,m_params.k_w);
    m_currentAngle.armRotation = m_motorSign[1]*(recv.Pos-m_zeros.armRotation)/ms_motorScalar;

    recv = position_control_custom(m_serial,2,m_motorSign[2]*armFeetInterAngle*ms_motorScalar+m_zeros.armFeetIntersect,m_params.k_p,m_params.k_w);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}
void LegMotors::TorqueCtrl(float shoulderTorque,float armTorque,float armFeetInterTorque)
{
        // cout << "torque control:\n" << shoulderTorque << ", " << armTorque << ", " << armFeetInterTorque << endl;
    //return;
    MOTOR_recv recv;
    recv = torque_control(m_serial,0,shoulderTorque);
    m_currentAngle.shoulderHorizontal = m_motorSign[0]*(recv.Pos-m_zeros.shoulderHorizontal)/ms_motorScalar;

    recv = postion_control(m_serial,1,armTorque);
    m_currentAngle.armRotation = m_motorSign[1]*(recv.Pos-m_zeros.armRotation)/ms_motorScalar;

    recv = postion_control(m_serial,2,armFeetInterTorque);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}
void LegMotors::HopsTorqueCtrl(float armFeetInterTorque)
{
        // cout << "torque control:\n" << shoulderTorque << ", " << armTorque << ", " << armFeetInterTorque << endl;
    //return;
    MOTOR_recv recv;
    recv = torque_control(m_serial,2,armFeetInterTorque);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}
void LegMotors::HopsVelCtrl(float armFeetInterTorque)
{
        // cout << "torque control:\n" << shoulderTorque << ", " << armTorque << ", " << armFeetInterTorque << endl;
    //return;
    MOTOR_recv recv;
    recv = velocity_control(m_serial,2,armFeetInterTorque);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}

void BlendCtrl(AxisMovement angle,AxisTorque torque)
{

}

AxisMovement LegMotors::GetCurrentMotorAngle()
{
    return m_currentAngle;
}
