#ifndef __SER_H
#define __SER_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include "MultiThread.h"
class Lazer
{
    public:
    struct l_param
    {
        double lazery;
        double lazerx;
        double yaw;
        double firsty;
        double firstx;
        double firstyaw;
        double wanty;
        double wantx;
        double init;
    };
    nav_msgs::Odometry::ConstPtr latest_msg;
    l_param lazer_param;
    l_param m_lazer_param;
    l_param init_param;
    l_param dog_param;
    void transfer();
    void transferinit(double yaw);
    void transfertw(double yaw);
    void getlazerinit();
    void uplazerfirst();
    void getlazerparaminitfirst();
    void tranferinitpoint(double x,double y,double yaw);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void timeCallback(const ros::TimerEvent&);
    void getlazerparaminit();
    void getlazerparam();
    static void* _Lazerinit(void *arg);
    protected:
    bool m_first;
    MUTEX ml_mutex;
    
};
class LazerStart
{
    public:
    struct lazer_params
    {

        int argc;
        char **argv;
        Lazer lazer;
    };
    void Start(int argc, char **argv,Lazer lazer);
    void Exit();
    protected:
    THREAD ml_threadDesc;
};
#endif