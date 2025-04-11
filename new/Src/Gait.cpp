#include "Gait.h"
#include <assert.h>

Gait Gait::LinerInterpl(Gait& g1,Gait& g2,float lambda)
{
    return Gait({0,0,0,0},{1,1,1,1},1);
}

Gait Gait::Trot(float duration)
{
    return Gait({0*duration,0.5f*duration,0*duration,0.5f*duration},{0.5f*duration,0.5f*duration,0.5f*duration,0.5f*duration},duration);
}

Gait::Gait(std::vector<float> phase,std::vector<float> duration,float totalDuration)
{
    m_duration = totalDuration;
    assert(phase.size() == 4);assert(duration.size() == 4);assert(totalDuration>0);

    for(int i = 0;i<4;++i)
    {
        m_desc[i].duration = duration[i]/m_duration;
        m_desc[i].phase = phase[i]/m_duration;
    }
}

Gait Gait::Pace(float duration,float standingTime)
{
    float swingTime = (duration-2.0f*standingTime)/2.0f;
    return Gait(
        {0,swingTime+standingTime,0,swingTime+standingTime},
        {swingTime,swingTime,swingTime,swingTime},
        duration);
}

Gait::Gait(GaitDesc desc[4],float duration)
{
    m_duration = duration;
    for(int i = 0;i<4;++i)
    {
        m_desc[i].duration = desc[i].duration/m_duration;
        m_desc[i].phase = desc[i].phase/m_duration;
    }
}

#include <stdio.h>

std::vector<float> Gait::GetMovingStatus(float time)
{
    std::vector<float> phase(4);
    int period = int(time/m_duration);
    time -= period*m_duration;
    time /= m_duration;
    //static int cnt = 0;
    //if(cnt++ == 1000)
     //   printf("time:%f\n",time*100),cnt = 0;
    for(int i = 0;i<4;++i)
    {
        float dt = time-m_desc[i].phase;
        float swingDur = m_desc[i].duration;
        float touchDur = 1.0f-m_desc[i].duration;
        if(swingDur < 1e-5){phase[i] = 0;continue;}
        if(touchDur<1e-5){phase[i] = 1+1e-5;continue;}

        if(dt >=0)
        {
            dt -= m_desc[i].duration;
            if(dt>=0)
                phase[i] = 1.0f+dt/touchDur;
            else
                phase[i] = 1.0f+dt/swingDur;
        }else
        {
            dt += 1.0f-m_desc[i].duration;
            if(dt >=0)
                phase[i] = 1.0f+dt/touchDur;
            else
                phase[i] = 1.0f+dt/touchDur;
        }
    }
    return phase;
}

float Gait::GetSwingDuration()
{
    return m_desc->duration*m_duration;
}
float Gait::GetTouchDuration()
{
    return (1.0f-m_desc->duration)*m_duration;
}

float Gait::GetTotalDuration()
{
    return m_duration;
}
