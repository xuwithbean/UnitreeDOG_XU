#pragma once
#include "SerialPort.h"
#include "motor_control.h"
#include "Kinematics.h"

class LegMotors
{
    public:
    struct MotorParams
    {
        float k_p;
        float k_w;
        MotorParams(float kp = 0,float kw = 0):k_p(kp),k_w(kw){}
    };
public:
    static void SetMotorScalar(float motorScalar) { ms_motorScalar = motorScalar; }
    static float GetMotorScalar() { return ms_motorScalar; }
    static void MotorInitCheck();
public:
    LegMotors(AxisMovement zeroPos,std::vector<float> motorSign,float motorScalar,std::string portName,int id);

    void PositionCtrl(float shoulderAngle,float armAngle,float armFeetInterAngle);
    void TorqueCtrl(float shoulderTorque,float armTorque,float armFeetInterTorque);
    void HopsTorqueCtrl(float armFeetInterTorque);
    void HopsVelCtrl(float armFeetInterTorque);
    //void BlendCtrl(AxisMovement angle,AxisTorque torque);
    AxisMovement GetCurrentMotorAngle();
    void SetMotorParams(MotorParams params){m_params = params;}
    MotorParams GetMotorParams(){return m_params;}

protected:
    void _initCheck();

    void _checkPos(float shoulderPos,float armPos,float feetPos);
    void _checkRange(float shoudlerAngle,float armAngle,float feetAngle);
    void _checkParam(float kp,float kw);
protected:
    static std::vector<LegMotors*> ms_registerMotor;
    AxisMovement m_zeros;
    SerialPort m_serial;
    AxisMovement m_currentAngle;
    AxisTorque m_currentTorque;
    static float ms_motorScalar;
    std::vector<float> m_motorSign;
    static bool ms_systemSafe;
    std::string m_name;
    MotorParams m_params;
    int m_id;
};
