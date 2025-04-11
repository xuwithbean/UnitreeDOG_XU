#include "AutoControl.h"
#include "PacePlanner.h"
AutoCtrl::AutoCtrl(IMU* pIMU)
{
    m_param.hop = false;
    m_param.r = m_param.x = m_param.y = 0;
    m_startAuto = false;
    m_startRotate = false;
    m_meetThres = false;
    m_threshold = 4;
    m_kp = 0.05;
    m_kp_str = 0.02;
    m_lastDiff = 0;
    m_pIMU = pIMU;
    mutex_create(m_mutex);
}

AutoCtrl::~AutoCtrl()
{
    mutex_destroy(m_mutex);
}
#include <stdio.h>
void AutoCtrl::UpdateStep()
{

    mutex_lock(m_mutex);
    if(m_actions.empty())
    {
        m_param.hop = false;
        m_param.r = 0;
        m_param.x = 0;
        m_param.y = 0;
    }else
    {
    printf("[Auto]:update steps!\n");
        ActionType type = m_actions.front().action;
        if(type == autoRotateTo || type == autoRotateWith)
        {
            a=1;
            m_threshold = 4;
            float yaw = m_pIMU->GetIMUData().yaw;
            float delta = yaw-m_targetAngle;
            if(!m_startRotate)
            {
                m_startRotate = true;
                m_meetThres = false;
                if(type == autoRotateTo)m_targetAngle = m_actions.front().r;
                else m_targetAngle = m_pIMU->GetIMUData().yaw+m_actions.front().r;
                if(m_targetAngle>180)m_targetAngle-= 360.0;
                else if(m_targetAngle < -180)m_targetAngle+=360.0;
                m_lastDiff = delta = yaw-m_targetAngle;
            }else
            {
                if(delta*m_lastDiff <= 0)m_meetThres = true;
            }
            printf("[AutoRotate]:cur:%.3f,tar:%.3f,delta:%.3f\n",yaw,m_targetAngle,delta);

            if(delta > 180.0)delta -= 360.0;
            else if(delta < -180.0)delta += 360.0;


            float p = -m_kp* delta;
            if(p >= 1)p = 1;
            if(p<=-1)p = -1;
            m_param.hop = false;
            m_param.x = m_param.y = 0;

            if(!m_meetThres)
                m_param.r = 0.7*p;
            else
                m_param.r = 0.3*p;
            printf("[Auto] r=%.3f,delta=%.3f,over=%d\n",m_param.r,delta,m_meetThres);

            if(fabsf(yaw-m_targetAngle) <= m_threshold)
            {
                if(m_meetThres)
                {
                    printf("[AutoRotate]ok!\n");
                    m_actions.pop();
                    m_startRotate = false;
                    m_meetThres = false;
                    m_param.r = 0;
                }
            }
            m_lastDiff = delta;

        }else
        {
            switch(type)
            {
                case run:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = 1;}break;
                case back:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = -1;}break;
                case stop:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.x = m_param.y = 0;}break;
                case turnR:{m_param.climb = false;m_param.hop = false;m_param.r = -0.5;m_param.x = 0;m_param.y = 0.8;}break;
                case turnL:{m_param.climb = false;m_param.hop = false;m_param.r = 0.5;m_param.x = 0;m_param.y = 0.8;}break;
                case rotateR:{m_param.climb = false;m_param.hop = false;m_param.r = -0.5;m_param.x = m_param.y = 0;}break;
                case rotateL:{m_param.climb = false;m_param.hop = false;m_param.r = 0.5;m_param.x = m_param.y = 0;}break;
                case moveR:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = 1;}break;
                case moveL:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = -1;}break;
                case hop:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopForward;}break;
                case hoponstair:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopForwardonStair;}break;
                case hopdownstair:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopForwarddownStair;}break;
                case hopair:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopForwardAir;}break;
                case getbalance:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::GetBalance;}break;
                case hopforce:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::ForceHopForward;}break;
                case stepAndSpan:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::StepAndSpan;}break;
                case stepAndRestore:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::StepToRestore;}break;
                case clawForward:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::Claw;}break;
                case clawRight:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::ClawRight;}break;
                case clawLeft:{m_param.climb = false;m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::ClawLeft;}break;
                case hold:{m_param.climb = false;m_param.hop = false;m_param.r = m_param.x = m_param.y = 0;}break;
                case record:{
                    m_param.climb = false;
                    m_param.hop = false;
                    Action &act = m_actions.front();
                    m_param.x = act.x;
                    m_param.y = act.y;
                    m_param.r = act.r;
                }break;
                case climbstraight:
                {
                    a=0;
                    m_param.climb = true;
                    m_param.hop = false;
                    m_param.climbangle = m_pIMU->GetIMUData().pitch-0;
                    Action &act = m_actions.front();
                    if(act.action == climbstraight)
                        {m_param.y = 1;}
                    printf("[AutoClimbStrainght]");
                }break;
                case sback:
                case straight:
                {
                    a=0;
                    m_param.climb = false;
                    Action &act = m_actions.front();
                    if(!m_startAuto)
                    {
                        while(fabsf(act.r) >= 180.0)
                        {
                            if(act.r > 0)act.r -= 360.0;
                            else act.r += 360.0;
                        }
                        m_startAuto = true;
                        m_targetAngle = act.r;
                    }
                    float delta = m_pIMU->GetIMUData().yaw-m_targetAngle;

                    if(delta > 180.0)delta -= 360.0;
                    else if(delta < -180.0)delta += 360.0;
                    m_param.hop = false;
                    m_param.x = delta*0.5*m_kp_str;
                    m_param.r = -m_kp_str*delta;

                    if(m_param.x >= 0.2)m_param.x = 0.2;
                    else if(m_param.x <= -0.2)m_param.x = -0.2;

                    if(m_param.r >= 0.2)m_param.r = 0.2;
                    else if(m_param.r <= -0.2)m_param.r = -0.2;

                    if(act.action == straight)
                        {m_param.x-=0.0;m_param.y = 1;}
                    else {m_param.y = -1;m_param.x *= -1;m_param.r *= -1;}

                    printf("[AutoStrainght] delta = %.3f,r = %.3f,x=%.3f\n",delta,m_param.r,m_param.x);
                }
                
            }
            switch(type)
            {
                case hop:
                case hoponstair:
                case hopdownstair:
                case hopair:
                case getbalance:
                case hopforce:
                case stepAndSpan:
                case stepAndRestore:
                case clawForward:
                case clawRight:
                case clawLeft:m_actions.pop();break;
                default:
                {
                    m_param.climbangle = m_pIMU->GetIMUData().pitch-0;
                    m_actions.front().actionCnt--;
                    if(m_actions.front().actionCnt <= 0)
                    {
                        m_startAuto = false;
                        m_actions.pop();
                    }
                }break;
            }
        }

    }
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddRecord(float x,float y,float r)
{
    mutex_lock(m_mutex);
    Action act;
    act.action = record;
    act.actionCnt = 1;
    act.x = x;
    act.y = y;
    act.r = r;
    m_actions.push(act);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddAction(Action action)
{
    mutex_lock(m_mutex);
    m_actions.push(action);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddAction(ActionType type,int count)
{
    mutex_lock(m_mutex);
    Action act;
    act.action = type;
    act.actionCnt = count;
    m_actions.push(act);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddActions(std::vector<Action> actions)
{
    mutex_lock(m_mutex);
    for(int i = 0;i<actions.size();++i)
        this->m_actions.push(actions[i]);
    mutex_unlock(m_mutex);
}

void AutoCtrl::ClearActions()
{
    mutex_lock(m_mutex);
    while(!m_actions.empty())m_actions.pop();
    m_param.hop = false;
    m_param.r = 0;
    m_param.x = 0;
    m_param.y = 0;
    mutex_unlock(m_mutex);
}

bool AutoCtrl::IsEmpty()
{
mutex_lock(m_mutex);
    bool empty = m_actions.empty();
    mutex_unlock(m_mutex);
    return empty;
}