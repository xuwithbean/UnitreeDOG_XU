#pragma once
#include <vector>

///some conventions///
///摆动项(0~1)
///工作项(1~2)

class Gait
{
public:
    struct GaitDesc//描述的是摆动项的相位和时长
    {
        float phase;
        float duration;
    };
public:
    static Gait LinerInterpl(Gait& g1,Gait& g2,float lambda);
    static Gait Trot(float duration);
    static Gait Pace(float duration,float standingTime);
public:
    Gait(GaitDesc desc[4],float duration);
    Gait(std::vector<float> phase,std::vector<float> duration,float totalDuration);
    std::vector<float> GetMovingStatus(float time);
    float GetSwingDuration();
    float GetTouchDuration();
    float GetTotalDuration();
protected:
    GaitDesc m_desc[4];
    float m_duration;
};
