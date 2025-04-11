 #include <iostream>
#include <stdio.h>
#include "Console.h"
#include "Controller.h"
#include "AutoControl.h"
#include "Connector.h"
#include "Debug.h"
#include "Sensor.h"
#include "InitAngle.h"
#include "ser.h"
#include <signal.h>
using namespace std;
//step 16 1.2 0.16 6finish double load
#define DEG(deg)    (deg)/180.0f*3.141592653589f
//printf
int main(int argc, char **argv)
{
    
    /*
    IMU imuSensor(0,0,0);
    imuSensor.OpenIMU("/dev/ttyUSB4");
    float vmcparam_a=0.01;
    float vmcparam_b=0.002;
    int cnt=0;
    while(1)
    {
        if(cnt%1000000==0)
        {
            float f=vmcparam_a*imuSensor.GetIMUData().roll+vmcparam_b*imuSensor.GetIMUData().rollVel;
            printf("r:%g,rv:%g,f:%g\n",imuSensor.GetIMUData().roll,imuSensor.GetIMUData().rollVel,f);
        }
        cnt++;
    }
    */
   
    Debug::Initialize(NULL,"../../../src/ser_package/log/log.txt","../../../src/ser_package/log/pipe.txt");//"./log/pipe.txt"
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    IMU imuSensor(0,0,0);
    imuSensor.OpenIMU("/dev/imu1");
    Lazer lazer;
    LazerStart lazerstart;
    //lazerstart.Start(argc,argv,lazer);
    ros::init(argc, argv, "odometry_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/Odometry", 10, &Lazer::odometryCallback,&lazer);
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), &Lazer::timeCallback,&lazer); 
    for(int i=0;i<=100000;i++)
    {
    ros::spinOnce();
    ros::spinOnce();
    lazer.uplazerfirst();
    lazer.transfer();
    lazer.getlazerparaminitfirst();
    lazer.transferinit(lazer.lazer_param.yaw);   
    }
    printf("%.3f,%.3f,%.3f\n",lazer.lazer_param.firsty,lazer.lazer_param.firstx,lazer.lazer_param.firstyaw); 
             
    AutoCtrl autoCtrl(&imuSensor,&lazer);
    InitAngle doginitangle("/dev/num1","/dev/num2","/dev/num3","/dev/num4");
    Connector connector(Connector::speed,&autoCtrl);
    
    Controller controller(
        {"/dev/num1","/dev/num2","/dev/num3","/dev/num4"},
        {
    AxisMovement(5.493198,1.364863,2.951769),
    AxisMovement(0.898531,1.326896,0.572560),
    AxisMovement(0.387331,3.678494,3.731034),
    AxisMovement(3.925466,4.911435,5.386586)


            /*
            doginitangle.m_initangle[0],
            doginitangle.m_initangle[1],
            doginitangle.m_initangle[2],
            doginitangle.m_initangle[3]//uud
            */
        },

        {
            //AxisMovement(DEG(90),DEG(90),DEG(90)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(21.95)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(21.95)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(21.95)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(21.95)),
        },
        {
            {-1,-1,-1},
            {1,1,1},
            {-1,1,1},
            {1,-1,-1},
        },
        LegController::VMCParam(),
        Controller::CtrlInitParam(0.5,5,20,14,0.3,0.01),//24//16////20//14
        Controller::MechParam(15+9.41f+9.41f,41)
    );
    Console con("/dev/key2",&autoCtrl,&controller,&lazer);
    OUT("[main]:system ready!\n");




    PacePlanner& planner = controller.GetPacePlanner();
    planner.SetCurveHeight(10.0f);
    planner.SetDogHeight(30.0f);

    planner.SetGait(Gait::Pace(0.6f,0.08f),0);//0.6,0.08//0.5.0.08
    controller.EnableVMC(false);
    OUT("[main]:params ready!starting up ...\n");

    controller.Start(3.0f);
    OUT("[main]:finish start up procedure!\n");

    OUT("[main]:start loop\n");
    con.Start();
    Console::ConsoleRequest req;
    Console::ConsoleStatus status;
    AutoCtrl::AutoCtrlParam autoPar;
    //while(1);
    bool sysQuit = false;
    bool connectStart = false;
    bool stepOver = false;

    long long debugCnt = 0;
        lazer.uplazerfirst();
    
    for(int i;i<=100000;i++)
    {lazer.transfer();}
    for(int i;i<=100000;i++)
    {lazer.getlazerparaminitfirst();}
    for(int i;i<=100000;i++)
    {lazer.transferinit(lazer.lazer_param.yaw);}
    lazer.lazer_param.wanty=20;//left is good;
    while(!sysQuit)
    {
        lazer.transfer();
        //lazer.transferinit(lazer.lazer_param.yaw);  
        //printf("ix:%.3f,iy:%.3f\n",lazer.init_param.lazerx,lazer.init_param.lazery);
        //Lazer::l_param a;
        //lazer.getlazerparam();
        //printf("%.3f\n",lazer.lazer_param.yaw);
        //printf("%.3f,y:%.3f\n",lazer.lazer_param.lazerx,lazer.lazer_param.lazery);
        //lazer.getlazerparaminit();
        //lazer.transferinit(lazer.init_param.yaw);
        //printf("%.3f,y:%.3f\n",lazer.init_param.lazerx,lazer.init_param.lazery);
        /////////////test area//////////////////////
        //controlleri.Update(0,0,0,true);
        //continue;
        /////////////end test area//////////////////

        con.GetConsoleRequest(req);
        con.GetConsoleStatus(status);
        if(status.auto_ || status.test)
        {
            //TODO
            if(status.auto_)
            {
                if(!connectStart && false)
                {
                    connector.Start();
                    connectStart = true;
                }
            }
            if(autoCtrl.IsEmpty())
                controller.StopMoving();
            autoCtrl.GetAutoCtrlParam(autoPar);
            //printf("%f,%f,%f\n",autoPar.x,autoPar.y,autoPar.r);
            //printf("%f\n",autoPar.climbangle);
            //printf("%d\n",autoPar.climb);
            if(controller.Update(autoPar.x,autoPar.y,autoPar.r,autoPar.hop,autoPar.hopType,true,autoPar.climb,autoPar.climbangle,autoPar.yawangle))
                autoCtrl.UpdateStep();
            stepOver = false;
        }else
            stepOver = controller.Update(req.x,req.y,req.r,req.reqHop,req.hopType,true,autoPar.climb,autoPar.climbangle,autoPar.yawangle);
        //printf("update\n");
        if(req.reqStop)
        {
            controller.StopMoving();
            //printf("req stop\n");
            }
        con.UpdateEvent(controller.IsStop(),stepOver);
        sysQuit = status.quit;
        ros::spinOnce();
        ros::spinOnce();

    }
    OUT("[main]:system quitting...\n");
    controller.Exit();
    OUT("[main]:Controller quit!\n");
    connector.Exit();
    OUT("[Auto]:Connector quit!\n");
    con.Exit();
    OUT("[Console]:Console quit!\n");
    imuSensor.CloseIMU();
    Debug::Exit();
    OUT("[main]:system quit!\n");
    return 0;
}
