#include "LegController.h"
#include <string>
#include <sstream>
#include "Eigen/Eigen"


#define PI          3.141592653589f
#define MAX(a,b)        (a>b?a:b)

float __measureMaxDiff(AxisMovement& angle1,AxisMovement& angle2)
{
    float dt[3] = {
        fabsf(angle1.shoulderHorizontal-angle2.shoulderHorizontal),
        fabsf(angle1.armRotation-angle2.armRotation),
        fabsf(angle1.armFeetIntersect-angle2.armFeetIntersect)};
    return MAX(MAX(dt[0],dt[1]),dt[2]);
}

void LegController::_updateParam(LegController::CtrlParam& currentParam,
    LegController::CtrlParam& lastParam,
    IMUReading& reading,
    float invDeltaTime)
{
    currentParam.yaw = reading.yaw;
    currentParam.yawVel = reading.yawVel;
    currentParam.roll = reading.roll;
    currentParam.rollVel = reading.rollVel;

    currentParam.feetSpeedX = (currentParam.feetPosX - lastParam.feetPosX) * invDeltaTime;
    currentParam.feetSpeedY = (currentParam.feetPosY - lastParam.feetPosY) * invDeltaTime;
    currentParam.feetSpeedZ = (currentParam.feetPosZ - lastParam.feetPosZ) * invDeltaTime;

    currentParam.topSpeedX = -currentParam.feetSpeedX - currentParam.yawVel * m_compVelX;
    currentParam.topSpeedY = -currentParam.feetSpeedY - currentParam.yawVel * m_compVelY;
    currentParam.topSpeedZ = -currentParam.feetSpeedZ;
}

void LegController::_doStartUp()
{
    if(m_ctrlParam.ctrlMask & feetPos == 0){while(1){printf("[LegController]:start up without an initial position!\n");}}
    clock_t start = clock(), end = start;
    float progress = 0;
    AxisMovement initAngle = m_motors.GetCurrentMotorAngle();
    LegKinematicsSolver::SolverCache cache;
    FeetMovement move(m_ctrlParam.feetPosX, m_ctrlParam.feetPosY, m_ctrlParam.feetPosZ);
    AxisMovement targetAngle = m_solver.Solve(move, cache);
    int idx = 0;
    do{
        //printf("%d\n",idx++);
        end = clock();
        progress = (float)(end - start) / CLOCKS_PER_SEC/m_startUpTime;
        m_motors.PositionCtrl(
            (1.0f-progress)*initAngle.shoulderHorizontal+progress*targetAngle.shoulderHorizontal,
            (1.0f-progress)*initAngle.armRotation+progress*targetAngle.armRotation,
            (1.0f-progress)*initAngle.armFeetIntersect+progress*targetAngle.armFeetIntersect
        );
    } while (progress <= 1.0f);
    this->m_currentPos = move;
}


void* LegController::_threadFunc(void* arg)
{
    LegController *pThis = static_cast<LegController*>(arg);
    LegMotors& motor = pThis->m_motors;
    LegKinematicsSolver& solver = pThis->m_solver;

    AxisMovement currentAngle,lastAngle;
    LegKinematicsSolver::SolverCache cache;
    bool cacheValid = false;
    const float validThreshold = 2.0/180.0*PI;
    Eigen::Matrix3f jacobMat;
    Eigen::Vector3f force,torque;
    LegController::VMCParam& vmc = pThis->m_vmcParam;
    LegController::CtrlParam ctrl = pThis->m_ctrlParam;
    LegController::CtrlParam real = ctrl,lastReal = ctrl;
    IMUReading reading;

    clock_t start = clock(), end = start;

    pThis->_doStartUp();
    pThis->m_startUpTime = 0;

    while(!pThis->m_sysExit)
    {
        if(pThis->m_ctrlUpdated)
        {
            mutex_lock(pThis->m_mutexDesc);

            ctrl = pThis->m_ctrlParam;
            pThis->m_ctrlUpdated = false;
            reading = pThis->m_imuReading;
            mutex_unlock(pThis->m_mutexDesc);
        }

        if(pThis->m_mode != VMC)
        {
            FeetMovement move(ctrl.feetPosX, ctrl.feetPosY, ctrl.feetPosZ);
            currentAngle = solver.Solve(move,cache);
            //if(pThis->m_legID == 1)
             //   std::cout<<currentAngle.shoulderHorizontal<<" "<<currentAngle.armRotation<<" "<<currentAngle.armFeetIntersect<<std::endl;
             if(pThis->m_mode == Position)
             {
                motor.PositionCtrl(currentAngle.shoulderHorizontal,currentAngle.armRotation,currentAngle.armFeetIntersect);
                pThis->m_currentPos = move;
             }else
            {
                //motor.BlendCtrl(currentAngle,AxisTorque(0,0,0));
            }
            cacheValid = false;
        }else if(pThis->m_mode == VMC){
            ////////calc jacobian matrix/////////

            if(ctrl.feetTouchDown)
            {
                if(cacheValid)
                    jacobMat = solver.FastCalcTopForceJacbMat(currentAngle,cache,lastAngle);
                else{
                    cacheValid = true;
                    lastAngle = currentAngle;
                    jacobMat = solver.CalcTopForceJacbMat(currentAngle,cache);
                }
                ///////// do torque control ////////////
                ///calc joint force//////
                force.x() = (ctrl.topSpeedX-real.topSpeedX)*vmc.cornerAbsorbStrength.x();
                force.y() = (ctrl.topSpeedY-real.topSpeedY)*vmc.cornerAbsorbStrength.y()
                +pThis->m_dogWidthInv*(vmc.yawSpring*(ctrl.yaw-real.yaw)+vmc.yawAbsorb*(ctrl.yawVel-real.yawVel));
                force.z() = (ctrl.topSpeedZ-real.topSpeedZ)*vmc.cornerAbsorbStrength.z()
                 +(ctrl.feetPosZ-real.feetPosZ)*vmc.cornerSpringZ;
                torque = jacobMat*force;
                torque.x()-=vmc.rollSpring*real.roll+vmc.rollAbsorb*real.rollVel;
            }else
            {
                if(cacheValid)
                    jacobMat = solver.FastCalcFeetForceJacbMat(currentAngle,cache,lastAngle);
                else{
                    cacheValid = true;
                    lastAngle = currentAngle;
                    jacobMat = solver.CalcFeetForceJacbMat(currentAngle,cache);
                }
                ///////// do torque control ////////////
                ///calc joint force//////
                force.x() = (ctrl.feetSpeedX-real.feetSpeedX)*vmc.feetAbsorbStrength.x()
                +(ctrl.feetPosX-real.feetPosX)*vmc.feetSpringStrength.x();
                force.y() = (ctrl.feetSpeedY-real.feetSpeedY)*vmc.feetAbsorbStrength.y()
                +(ctrl.feetPosY-real.feetPosY)*vmc.feetSpringStrength.y();
                force.z() = (ctrl.feetSpeedZ-real.feetSpeedZ)*vmc.feetAbsorbStrength.z()
                +(ctrl.feetPosZ-real.feetPosZ)*vmc.feetSpringStrength.z();
                torque = jacobMat*force;

            }
            ///// apply force////
            motor.TorqueCtrl(torque.x(),torque.y(),torque.z());
            lastAngle = currentAngle;
            currentAngle = motor.GetCurrentMotorAngle();
            if(cacheValid)
            {
                if(__measureMaxDiff(currentAngle,lastAngle) >= validThreshold)
                    cacheValid = false;
            }
            if(cacheValid)
            {
                FeetMovement move = solver.FastSolve(currentAngle,cache,lastAngle);
                real.feetPosX = move.x;
                real.feetPosY = move.y;
                real.feetPosZ = move.z;
            }else
            {
                FeetMovement move = solver.Solve(currentAngle,cache);
                cacheValid = true;
                real.feetPosX = move.x;
                real.feetPosY = move.y;
                real.feetPosZ = move.z;
            }
            float invDt;
            end = clock();
            invDt = 1.0f / (end - start+1e-3f) * CLOCKS_PER_SEC;
            start = end;
            pThis->_updateParam(real, lastReal, reading, invDt);

        }
    }
    return 0;
}

bool __isRight(int id)
{
    switch(id)
    {
        case RF:
        case RB:return true;
        case LF:
        case LB:return false;
    }
    return false;
}

LegController::LegController(
    VMCParam vmcParam,
    int legId,
    std::string serialName,
    AxisMovement zeroAngle,
    std::vector<float> motorSign):
        m_motors(zeroAngle,motorSign,9.1f,serialName,legId),
        m_vmcParam(vmcParam),
        m_solver(__isRight(legId), LegStructure::theStruct),
        m_updateTime(-1),
        m_legID(legId),
        m_startUpTime(0)
{
    m_ctrlUpdated = false;
    m_mode = Position;
    m_bVMCCtrl = false;

    float norm = sqrtf(vmcParam.dogLength * vmcParam.dogLength + vmcParam.dogWidth * vmcParam.dogWidth);
    float xdr = vmcParam.dogWidth/norm;
    float ydr = vmcParam.dogLength/norm;

    switch(legId)
    {
        case RF:
        {
            m_dogWidthInv = 1.0f/vmcParam.dogWidth;
            m_compVelX = -norm*ydr;
            m_compVelY = norm*xdr;
        }break;
        case RB:
        {
            m_dogWidthInv = 1.0f/vmcParam.dogWidth;
            m_compVelX = norm*ydr;
            m_compVelY = norm*xdr;
        }break;
        case LF:
        {
            m_dogWidthInv = -1.0f/vmcParam.dogWidth;
            m_compVelX = -norm*ydr;
            m_compVelY = -norm*xdr;
        }break;
        case LB:
        {
            m_dogWidthInv = -1.0f/vmcParam.dogWidth;
            m_compVelX = norm*ydr;
            m_compVelY = -norm*xdr;
        }break;
    }

    mutex_create(m_mutexDesc);

}

LegController::~LegController()
{
    mutex_destroy(m_mutexDesc);
//    thread_destroy(m_threadDesc);
}

void LegController::Start(float startUpTime)
{
    m_startUpTime = startUpTime;
    thread_create(LegController::_threadFunc, this,m_threadDesc);
}

void LegController::Exit()
{
    m_sysExit = true;
    thread_join(m_threadDesc);

}

void LegController::SetIMUReading(IMUReading& reading)
{
    mutex_lock(m_mutexDesc);
    m_imuReading = reading;
    m_ctrlUpdated = true;
    mutex_unlock(m_mutexDesc);
}

bool LegController::Ready()
{
    return m_startUpTime == 0;
}

void LegController::ApplyCtrlParam(LegController::CtrlParam &param)
{
    //printf("OK");
    static long long cnt = 0;
    mutex_lock(m_mutexDesc);
    if(param.ctrlMask & topSpeed)
    {
        m_ctrlParam.topSpeedX = param.topSpeedX;
        m_ctrlParam.topSpeedY = param.topSpeedY;
    }
    if(param.ctrlMask & feetPos)
    {
        m_ctrlParam.feetPosX = param.feetPosX;
        m_ctrlParam.feetPosY = param.feetPosY;
        m_ctrlParam.feetPosZ = param.feetPosZ;

        if(cnt % 10000 == 0)
        {cnt = 0;
            //printf("pos:%.3f,%.3f,%.3f\n",param.feetPosX,param.feetPosY,param.feetPosZ);
        }
        cnt++;
    }
    if(param.ctrlMask & orientation)
    {
        m_ctrlParam.yaw = param.yaw;
        m_ctrlParam.yawVel = param.yawVel;
    }
    m_ctrlParam.feetTouchDown = param.feetTouchDown;
    m_ctrlUpdated = true;
    mutex_unlock(m_mutexDesc);
}

void LegController::SetCtrlMode(CtrlMode mode)
{
    m_mode = mode;
}
void LegController::ApplyFroceCtrlParam(float Frocefeet)
{
    mutex_lock(m_mutexDesc);
    m_motors.HopsTorqueCtrl(Frocefeet);
    m_ctrlUpdated = true;
    mutex_unlock(m_mutexDesc);
}
void LegController::ApplyVelCtrlParam(float Velfeet)
{
    mutex_lock(m_mutexDesc);
    m_motors.HopsTorqueCtrl(Velfeet);
    m_ctrlUpdated = true;
    mutex_unlock(m_mutexDesc);
}