 #include <iostream>
#include <stdio.h>
#include "Console.h"
#include "Controller.h"
#include "AutoControl.h"
#include "Connector.h"
#include "Debug.h"
#include "Sensor.h"
#include "InitAngle.h"
using namespace std;
//step 16 1.2 0.16 6finish double load
#define DEG(deg)    (deg)/180.0f*3.141592653589f
//printf
int main()
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
   
    Debug::Initialize(NULL,"../log/log.txt","../log/pipe.txt");//"./log/pipe.txt"
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    IMU imuSensor(0,0,0);
    imuSensor.OpenIMU("/dev/imu1");
    AutoCtrl autoCtrl(&imuSensor);
    InitAngle doginitangle("/dev/num1","/dev/num2","/dev/num3","/dev/num4");
    Connector connector(Connector::speed,&autoCtrl);
    Controller controller(
        {"/dev/num1","/dev/num2","/dev/num3","/dev/num4"},
        {
AxisMovement(2.475084,2.680254,5.834893),
AxisMovement(1.577319,2.512666,0.803808),
AxisMovement(6.310045,2.090054,3.308804),
AxisMovement(2.348914,1.182702,5.464819)




            /*
            doginitangle.m_initangle[0],
            doginitangle.m_initangle[1],
            doginitangle.m_initangle[2],
            doginitangle.m_initangle[3]//uud
            */
        },

        {
            //AxisMovement(DEG(90),DEG(90),DEG(90)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(19.95)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(19.95)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(19.95)),
            AxisMovement(DEG(21.66),DEG(180-21.45),DEG(19.95)),
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
    Console con("/dev/input/event11",&autoCtrl,&controller);
    OUT("[main]:system ready!\n");




    PacePlanner& planner = controller.GetPacePlanner();
    planner.SetCurveHeight(8.0f);
    planner.SetDogHeight(30.0f);

    planner.SetGait(Gait::Pace(0.6f,0.06f),0);//0.6,0.08
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
    while(!sysQuit)
    {
        /////////////test area//////////////////////
        //controller.Update(0,0,0,true);
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
            if(controller.Update(autoPar.x,autoPar.y,autoPar.r,autoPar.hop,autoPar.hopType,true,autoPar.climb,autoPar.climbangle))
                autoCtrl.UpdateStep();
            stepOver = false;
        }else
            stepOver = controller.Update(req.x,req.y,req.r,req.reqHop,req.hopType,true,autoPar.climb,autoPar.climbangle);
        //printf("update\n");
        if(req.reqStop)
        {
            controller.StopMoving();
            //printf("req stop\n");
            }
        con.UpdateEvent(controller.IsStop(),stepOver);
        sysQuit = status.quit;

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
