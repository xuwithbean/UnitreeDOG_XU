#include "PacePlanner.h"
#include <iostream>
using namespace std;
int a=0;
PacePlanner::PacePlanner(float maxStepFwd,float maxStepVrt,float maxRotation,float dogWidth,float dogLength)
    :m_maxStepForward(maxStepFwd),
    m_maxStepVertical(maxStepVrt),
    m_maxRotation(maxRotation),
    m_gait({0,0,0,0},{1,1,1,1},1),
    m_lastGait({0,0,0,0},{1,1,1,1},1)
{
    m_dogRadius = sqrtf(dogWidth*dogWidth+dogLength*dogLength);
    m_dogDirX =  dogWidth/m_dogRadius;
    m_dogDirY = dogLength/m_dogRadius;
    m_dogRadius /= 2.0f;
    m_climbAngle = 0;
    m_offsetX = 9.41;
    m_offsetY = 0;
    for(int i = 0;i<4;++i)
    {
        m_offset[i] = m_lastOffset[i] = {0.0f,0.0f};
        m_legSwinging[i] = false;

    }
    Reset();
}

float PacePlanner::_cycloidCurve(float time,float height)
{
    const float pi_2 = 3.141592653589f*2.0f;
    float phi = time*pi_2;
    return 0.5f*height*(1.0f-cos(phi));
}

void PacePlanner::SetPaceType(PaceType type)
{
    m_type = type;
}
void PacePlanner::SetGait(Gait gait,float transitionTime)
{
    m_lastGait = m_gait;
    m_gait = gait;
    m_gaitLerpTime = transitionTime;
}
void PacePlanner::SetDogOffset(float offsetX,float offsetY)
{
    m_offsetX = offsetX;
    m_offsetY = offsetY;
}

void PacePlanner::Reset()
{
    m_totalTime = 0;
    for (int i = 0; i < 4; ++i)
        m_offset[i] = m_lastOffset[i] = { 0,0 };
}

void PacePlanner::DebugShow()
{
    printf("[PacePlanner]Debug Information:\n");
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",m_dogHeight,m_offsetX,m_climbAngle,m_offset[0].first,m_offset[0].second,m_totalTime);
    printf("[PacePlanner]End of the section.\n");
}

void PacePlanner::SetVelocity(float forward,float vertical,float rotation)
{
    if(m_bMVCCtrl)return;
    forward += 0.10;
    //rotation += 0.1;
    vertical += 0.01;

    if (forward > 1.0f)forward = 1.0f;
    else if (forward < -1.0f)forward = -1.0f;

    if (vertical > 1.0f)vertical = 1.0f;
    else if (vertical < -1.0f)vertical = -1.0f;

    if (rotation > 1.0f)rotation = 1.0f;
    else if (rotation < -1.0f)rotation = -1.0f;


    float stepFwd = forward * m_maxStepForward;
    float stepVert = vertical * m_maxStepVertical;
    float stepRot = rotation  * m_maxRotation * m_dogRadius;
    if(a == 1)
    {
    stepRot = rotation  * 2 * m_maxRotation * m_dogRadius;
    }
    else
    {
    stepRot = rotation  * m_maxRotation * m_dogRadius;
    }

    m_offset[RF] = { stepVert - stepRot * m_dogDirY,stepFwd + stepRot * m_dogDirX };
    m_offset[LF] = { stepVert - stepRot * m_dogDirY,stepFwd - stepRot * m_dogDirX };
    m_offset[LB] = { stepVert + stepRot * m_dogDirY,stepFwd - stepRot * m_dogDirX };
    m_offset[RB] = { stepVert + stepRot * m_dogDirY,stepFwd + stepRot * m_dogDirX };

    //for(int i = 0;i<4;++i)
     //   std::cout<<"applay motor:"<<i<<"pos:"<<m_offset[i].first<<","<<m_offset[i].second<<endl;

}

void PacePlanner::SetClimbAngle(float angle)
{
    m_climbAngle = angle;
}

bool PacePlanner::Update(float deltaTime,std::vector<FeetMovement>& move,std::vector<bool>& touch,bool climb,float climbangle,float yawangle)
{
    float centerY = -2;
    bool loopOver = false;
    m_totalTime += deltaTime;
    m_climbAngle = climbangle/180.0f*3.141592653589f;
    m_yawAngle = yawangle/180.0f*3.141592653589f;
    if(climb == false)
    {
        SetCurveHeight(10.0f);
        m_climbAngle = 0;
        m_yawAngle = 0;
    }
    if(climb == true)
    {
        SetCurveHeight(8.0f);
    }
    //static int cnt = 0;
    //if(cnt++ == 1000)
     //   cout<<"totalTime:"<<m_totalTime<<endl,cnt = 0;
    if(m_totalTime >= m_gait.GetTotalDuration())
    {
       // cout<<"1s"<<endl;
        loopOver = true;
        m_totalTime -= m_gait.GetTotalDuration();
    }
    std::vector<float> legTime = m_gait.GetMovingStatus(m_totalTime);

    float signX = 1,signY = 1;
    if(m_bMVCCtrl)
    {

        for(int i = 0;i<4;++i)
        {
            if (i == LF || i == LB)signX = -1;
            else signX = 1;
            if(i == LF || i == RF)signY = 1;
            else signY = -1;
            move[i].x = move[i].y=0;
            move[i].x += signX * m_offsetX;
            if (legTime[i] <= 1.0f)
            {
                move[i].z = -m_dogHeight + _cycloidCurve(legTime[i], m_curveHeight);
                touch[i] = false;
            }
            else
            {
                move[i].z = -m_dogHeight;
                touch[i] = true;
            }
        }
    }else
    {

        for(int i = 0;i<4;++i)
        {
            if (i == LF || i == LB)signX = -1;
            else signX = 1;
            if(i == LF || i == RF)signY = 1;
            else signY = -1;
            if(legTime[i]<=1.0f)
            {
                if(!m_legSwinging[i])
                {
                    m_legSwinging[i] = true;
                    m_lastOffset[i] = m_offset[i];
                }
                move[i].x = signX*m_offsetX+(1.0f-legTime[i])*-m_lastOffset[i].first+legTime[i]*m_offset[i].first;
                move[i].y = signY*m_offsetY+(1.0f-legTime[i])*-m_lastOffset[i].second+legTime[i]*m_offset[i].second+centerY;
                move[i].z = -m_dogHeight+_cycloidCurve(legTime[i],m_curveHeight);
                m_legSwinging[i] = true;
                touch[i] = false;
            }else{
                if(m_legSwinging[i])
                {
                    m_legSwinging[i] = false;
                    m_lastOffset[i] = m_offset[i];
                }
                move[i].x = signX * m_offsetX+(2.0f-legTime[i])*m_lastOffset[i].first-(legTime[i]-1.0f)*m_offset[i].first;
                move[i].y = signY*m_offsetY+(2.0f-legTime[i])*m_lastOffset[i].second-(legTime[i]-1.0f)*m_offset[i].second+centerY;
                move[i].z = -m_dogHeight;
                touch[i] = true;
            }
            move[i].y += sin(m_climbAngle)*move[i].z;
            move[i].z *= cos(m_climbAngle);
            /*
            if(climb == true)
            {
                float a = atan(sin(m_climbAngle)*tan(m_yawAngle));
                move[i].x -= move[i].z*cos(a)*sin(m_climbAngle);
                move[i].y += move[i].z*sin(a);
                move[i].z *= cos(a)*cos(m_climbAngle);

            }
            
            */
        }
    }
    return loopOver;
}

void PacePlanner::EnableVMC(bool vmcEnable)
{
    m_bMVCCtrl = vmcEnable;
}

void PacePlanner::SetDogHeight(float height)
{
    m_dogHeight = height;
}
void PacePlanner::SetCurveHeight(float height)
{
    m_curveHeight = height;
}
