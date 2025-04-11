#include "Controller.h"
#include<iostream>
#include "Debug.h"
using  namespace std;

#define DEBUG_MODE
#ifdef DEBUG_MODE
const bool enableMap[4] = {true,true,true,true};
#endif // DEBUG_MODE

#define X	0
#define Y	1
#define R	2

#define CLAMP(num,low,up)	{if(num > up)num = up;\
							else if(num < low)num = low;}

Controller::Controller(
    std::vector<std::string> serialName,
	std::vector<AxisMovement> motorAngle,
	std::vector<AxisMovement> realAngle,
	std::vector<std::vector<float>> motorSign,
	LegController::VMCParam param,
	CtrlInitParam initParam,MechParam mcParam)
	:m_planner(initParam.maxVelFw,initParam.maxVelVt,initParam.maxVelRt,mcParam.dogWidth,mcParam.dogLength),
	m_pos(4,FeetMovement(0,0,0)),
	m_touchStatus(4,true),
	m_kp(initParam.kp),m_kw(initParam.kw),
	m_movingThres(initParam.movingThreshold)
	//m_usingGlobalRecord(initParam.usingGlobal)
{
	m_maxVel[X] = initParam.maxVelFw;
	m_maxVel[Y] = initParam.maxVelVt;
	m_maxVel[R] = initParam.maxVelRt;

	for(int i = 0;i<3;++i)
		m_incVel[i] = m_hisVel[i] = m_outVel[i] = 0;
	m_time = -1;
	AxisMovement angle;
	float scalar = LegMotors::GetMotorScalar();
	for (int i = 0; i < 4; ++i)
	{
		angle.shoulderHorizontal = motorAngle[i].shoulderHorizontal - motorSign[i][0]*realAngle[i].shoulderHorizontal * scalar;
		angle.armRotation = motorAngle[i].armRotation - motorSign[i][1]*realAngle[i].armRotation * scalar;
		angle.armFeetIntersect = motorAngle[i].armFeetIntersect - motorSign[i][2]*realAngle[i].armFeetIntersect * scalar;
        cout<<"init leg controller["<<i<<"] zeros:"<<angle.shoulderHorizontal<<","<<angle.armRotation<<","<<angle.armFeetIntersect<<endl;
        #ifdef DEBUG_MODE
        if(enableMap[i])
        #endif // DEBUG_MODE
            m_pControllers[i] = new LegController(param, i, serialName[i],angle,motorSign[i]);
	}
	m_smthCtrl = true;
	m_needHop = m_needStop = false;
	m_stop = true;
    m_planner.SetDogOffset(9.41f);
    m_clawDown = false;
    m_startClaw = false;
}

Controller::~Controller()
{
	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
		delete m_pControllers[i];
    }
}

void Controller::Start(float startUpTime)
{
    m_planner.SetVelocity(0,0,0);
	LegController::CtrlParam param;
	m_planner.Update(0, m_pos, m_touchStatus,false,0);
    m_planner.DebugShow();
    for (int i = 0; i < 4; ++i)
    {
        param.ctrlMask = LegController::feetPos;
        param.feetPosX = m_pos[i].x;
        param.feetPosY = m_pos[i].y;
        param.feetPosZ = m_pos[i].z;
        #ifdef DEBUG_MODE
        if(enableMap[i])
        #endif // DEBUG_MODE
        {m_pControllers[i]->ApplyCtrlParam(param);}
        printf("leg[%d],x:%.3f,y:%.3f,z:%.3f\n",m_pos[i].x,m_pos[i].y,m_pos[i].z);
    }




	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
        {m_pControllers[i]->Start(startUpTime);}
	}

	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
        if(enableMap[i])
	#endif // DEBUG_MODE
        {
            while (!m_pControllers[i]->Ready());
		}
	}
}

void Controller::EnableSmoothCtrl(bool enable)
{
	m_smthCtrl = enable;
}

void Controller::Exit()
{
	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
		{m_pControllers[i]->Exit();}
	}
}

void Controller::_updateVel(float x,float y,float r,float dt)
{
	float ctrlVel[3] = {x,y,r};
    float err_x = ctrlVel[X] - m_outVel[X],err_y = ctrlVel[Y]-m_outVel[Y],err_r = ctrlVel[R]-m_outVel[R];

	if(m_smthCtrl)
	{
		m_incVel[X] += m_kp*err_x*dt+m_kw*(err_x-m_hisVel[X]);
		m_incVel[Y] += m_kp*err_y*dt+m_kw*(err_y-m_hisVel[Y]);
		m_incVel[R] += m_kp*err_r*dt+m_kw*(err_r-m_hisVel[R]);

		m_outVel[X] += m_incVel[X]*dt;
		m_outVel[Y] += m_incVel[Y]*dt;
		m_outVel[R] += m_incVel[R]*dt;

		m_hisVel[X] = err_x;
		m_hisVel[Y] = err_y;
		m_hisVel[R] = err_r;
	}else
	{
		m_outVel[X] = ctrlVel[X];
		m_outVel[Y] = ctrlVel[Y];
		m_outVel[R] = ctrlVel[R];
	}
	if(fabsf(m_outVel[X])<m_movingThres && x == 0)m_outVel[X] = 0;
	if(fabsf(m_outVel[Y])<m_movingThres && y == 0)m_outVel[Y] = 0;
	if(fabsf(m_outVel[R])<m_movingThres && r == 0)m_outVel[R] = 0;


	m_moving = fabsf(m_outVel[X])>m_movingThres ||
				fabsf(m_outVel[Y])>m_movingThres ||
				fabsf(m_outVel[R])>m_movingThres;

    CLAMP(m_outVel[X],-1,1);
    CLAMP(m_outVel[Y],-1,1);
    CLAMP(m_outVel[R],-1,1);
}

bool Controller::Update(float velX, float velY, float velYaw,bool Hop,HopType type,bool restrictHop,bool climb,float climbangle)
{
	if(climb == false)
	{
		climbangle = 0;
	}
	m_balanceangle = climbangle;
	CLAMP(velX,-1,1);
	CLAMP(velY,-1,1);
	CLAMP(velYaw,-1,1);

	if(m_needStop)
		velX = velY = velYaw = 0;


	if(m_stop)
	{
		if(m_needHop || Hop)
		{
            if(Hop)
                _doHop(type);
            else _doHop(m_hopType);
			m_needHop =false;
			return true;
		}
		if(velX != 0 || velY != 0 || velYaw != 0)StartMoving();
		else return true;
	}else
	{
        if(Hop)
        {
            if(restrictHop)
            {
                m_needHop = true;
                m_needStop = true;
                m_hopType = type;
            }
        }
	}


    if(m_time == -1)
        m_time = clock();
	clock_t time = clock();
	float dt = (float)(time - m_time)/CLOCKS_PER_SEC;
	m_time = time;
	_updateVel(velX,velY,velYaw,dt);
    //printf("x:%f,y:%f,r:%f,dt:%f\n",m_outVel[Y], m_outVel[X], m_outVel[R],dt);
	m_planner.SetVelocity(m_outVel[Y], m_outVel[X], m_outVel[R]);
	LegController::CtrlParam param;
	bool status = m_planner.Update(dt, m_pos, m_touchStatus,climb,climbangle);
	if(status)
	{
		if(m_needStop)
		{
			if(!m_moving)
			{
				m_stop = true;
				m_needStop = false;
				m_planner.Reset();
				m_planner.Update(0,m_pos, m_touchStatus,climb,climbangle);
			}
		}
	}
	if (m_bVMCCtrl)
	{
		for (int i = 0; i < 4; ++i)
		{
			param.ctrlMask = LegController::feetPos|LegController::orientation|LegController::topSpeed;
			param.feetPosX = m_pos[i].x;
			param.feetPosY = m_pos[i].y;
			param.feetPosZ = m_pos[i].z;
			param.feetSpeedX = 0;
			param.feetSpeedY = 0;
			param.feetSpeedZ = 0;
			param.feetTouchDown = m_touchStatus[i];
			//param.roll
			param.yaw = 0;
			param.yawVel = velYaw;
			param.topSpeedX = velX;
			param.topSpeedY = velY;
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
			{m_pControllers[i]->ApplyCtrlParam(param);}

		}
	}
	else
	{

		for (int i = 0; i < 4; ++i)
		{
			param.ctrlMask = LegController::feetPos;
			param.feetPosX = m_pos[i].x;
			param.feetPosY = m_pos[i].y;
			param.feetPosZ = m_pos[i].z;
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
			{m_pControllers[i]->ApplyCtrlParam(param);}
			//cout<<"applay motor:"<<i<<endl;
			//if(i == LB)
			//cout<<"applay motor:"<<i<<"pos:"<<m_pos[i].x<<","<<m_pos[i].y<<","<<m_pos[i].z<<endl;
		}
	}
	return status;
}

void Controller::GetCurrentVelocity(float &x,float &y,float &r)
{
    x = m_outVel[X];
    y = m_outVel[Y];
    r = m_outVel[R];
}

void Controller::StartMoving()
{
    //printf("sm\n");
	m_time = -1;
	m_planner.Reset();
	m_stop = false;
}

void Controller::StopMoving()
{
	if(m_stop)return;
	m_needStop = true;
}

void Controller::EnableVMC(bool enable)
{
	m_bVMCCtrl = enable;
	m_planner.EnableVMC(enable);
	for (int i = 0; i < 4; ++i){
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
        {m_pControllers[i]->SetCtrlMode(LegController::Position);}
	}
}

PacePlanner& Controller::GetPacePlanner()
{
	return m_planner;
}



