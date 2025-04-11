#pragma once
#include "MultiThread.h"
#include "Eigen/Eigen"
#include "LegMotors.h"
#include "Kinematics.h"
#include <time.h>
#include "Sensor.h"
#include "motor_control.h"



class LegController
{
protected:
    static void* _threadFunc(void* arg);
public:
    enum CtrlMode
    {
        VMC,
        Position,
        Blend
    };
    enum Mask
    {
        topSpeed = 0x01,
        feetPos = 0x02,
        orientation = 0x04
    };
    struct VMCParam
    {
        Eigen::Vector3f feetSpringStrength;
        Eigen::Vector3f feetAbsorbStrength;
        Eigen::Vector3f cornerAbsorbStrength;
        float cornerSpringZ;
        float rollSpring;
        float rollAbsorb;
        float yawSpring;
        float yawAbsorb;
        float dogWidth;
        float dogLength;
    };
    struct ForceParam
    {
        float Froceshoulder;
        float Froceleg;
        float Frocefeet;
    };
    struct CtrlParam
    {
        unsigned char ctrlMask;
        float topSpeedX;
        float topSpeedY;
        float topSpeedZ;
        float feetPosX;
        float feetPosY;
        float feetPosZ;
        float feetSpeedX;
        float feetSpeedY;
        float feetSpeedZ;
        float roll;
        float rollVel;
        float yaw;
        float yawVel;
        bool feetTouchDown;
    public:
        CtrlParam() {
            feetTouchDown = false;
            topSpeedX =
                topSpeedY =
                topSpeedZ =
                feetPosX =
                feetPosY =
                feetPosZ =
                feetSpeedX =
                feetSpeedY =
                feetSpeedZ =
                roll =
                rollVel =
                yaw =
                yawVel = 0;
        }
    };
    LegController(VMCParam vmcParam,int legId,std::string serialName,AxisMovement motorZeros,std::vector<float> motorSign);
    ~LegController();
    void Start(float startUpTime);
    void Exit();
    void ApplyCtrlParam(CtrlParam& param);
    void ApplyFroceCtrlParam(float Frocefeet);
    void ApplyVelCtrlParam(float Velfeet);
    //void ApplyImmediate(CtrlParam& param);
    void SetCtrlMode(CtrlMode mode);
    void SetIMUReading(IMUReading& reading);
    bool Ready();
    FeetMovement GetCurrentPosition(){return m_currentPos;}
    LegMotors* GetMotors(){return &m_motors;}
protected:
    void _updateParam(LegController::CtrlParam& currentParam,
        LegController::CtrlParam& lastParam,
        IMUReading& reading,
        float invDeltaTime);

    void _doStartUp();
protected:
    FeetMovement m_currentPos;
    float m_startUpTime;

    bool ms_systemSafe;
    bool m_sysExit;
    bool m_ctrlUpdated;
    THREAD m_threadDesc;
    MUTEX m_mutexDesc;
    LegMotors m_motors;
    bool m_bVMCCtrl;
    VMCParam m_vmcParam;
    CtrlParam m_ctrlParam;
    ForceParam m_forceparam;
    LegKinematicsSolver m_solver;
    clock_t m_updateTime;
    float m_compVelX;
    float m_dogWidthInv;
    float m_compVelY;
    int m_legID;
    IMUReading m_imuReading;
    CtrlMode m_mode;
};
