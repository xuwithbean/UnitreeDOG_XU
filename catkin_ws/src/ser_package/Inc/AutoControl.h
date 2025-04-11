#pragma once
#include "fcntl.h"
#include "MultiThread.h"
#include <queue>
#include "Controller.h"
#include "Sensor.h"
#include "ser.h"

class AutoCtrl
{
    public:
    enum ActionType
    {
        run,
        back,
        stop,
        turnR,
        turnL,
        rotateR,
        rotateL,
        hold,
        climb,
        moveR,
        moveL,
        hop,
        hoponstair,
        hopdownstair,
        hopair,
        getbalance,
        hopforce,
        stepAndSpan,
        stepAndRestore,
        record,
        autoRotateTo,
        autoRotateWith,
        ClawDown,
        clawForward,
        clawRight,
        clawLeft,
        straight,
        straight1,
        straight2,
        straight3,
        straight4,
        sback,
        climbstraight
    };
    struct Action
    {
        int actionCnt;
        double init;//
        ActionType action;
        float x,y,r;
        int actionnum;
        double wanty;
        double wantx;
        double firsty;
        double firstx;
        double lasty;
        double lastx;
    };
public:

    struct AutoCtrlParam
    {
        float x,y,r;
        bool hop;
        Controller::HopType hopType;
        bool stop;
        bool climb = false;
        float climbangle = 0;
        float yawangle = 0;
    };

public:
    AutoCtrl(IMU* pIMU,Lazer* pLazer);
    ~AutoCtrl();
    void UpdateStep();
    void GetAutoCtrlParam(AutoCtrlParam& param){param = m_param;}
    void AddAction(Action action);
    void AddAction(ActionType type,int count);
    void AddRecord(float x,float y,float r);
    void AddActions(std::vector<Action> actions);
    void ClearActions();
    bool IsEmpty();
    IMU* GetIMUSensor(){return m_pIMU;}
protected:
    MUTEX m_mutex;
    AutoCtrlParam m_param;
    std::queue<Action> m_actions;
    bool m_startRotate;
    bool m_startAuto;
    float m_threshold;
    float m_targetAngle;
    float m_kp;
    float m_kp_str;
    bool m_meetThres;
    float m_lastDiff;
    IMU *m_pIMU;
    Lazer *m_pLazer;
};