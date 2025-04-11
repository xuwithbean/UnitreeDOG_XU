
#pragma once
#include "Eigen/Eigen"

/////////////some coventions//////////////
//1.world coordiante:the world coordiante is a right handed coordiante,whose y axis is pointing forward,x axis pointing right
//
//        --| y
//     z^ /
//      |/
//      o----->x
//
//////////////////////////////////////////

#define RF  0//right-front
#define LF  1//left-front
#define LB  2//right-back
#define RB  3//left-back


class LegStructure
{
public:
    static void RegisterStructure(LegStructure strc) { theStruct = strc; }
public:
    LegStructure(float shoulder,float arm,float feet):
    shoulderLen(shoulder),
    armLen(arm),
    feetLen(feet){}

    float shoulderLen;
    float armLen;
    float feetLen;

    static LegStructure theStruct;
};


class AxisMovement
{
    public:
    AxisMovement(float shoulderHorizontal =0,float armRotation = 0,float armFeetIntersect = 0,bool valid = true)
    :shoulderHorizontal(shoulderHorizontal),
    armRotation(armRotation),armFeetIntersect(armFeetIntersect),valid(valid){}
    float shoulderHorizontal;
    float armRotation;
    float armFeetIntersect;
    bool valid;
};

class AxisTorque
{
    public:
    AxisTorque(float shoulderTorque = 0,float armTorque = 0,float feetTroque = 0):shoulderTorque(shoulderTorque),armTorque(armTorque),feetTorque(feetTorque){}
    float shoulderTorque;
    float armTorque;
    float feetTorque;
};

class FeetMovement
{
    public:
    FeetMovement(float x = 0,float y = 0,float z = 0):x(x),y(y),z(z){}
    float x,y,z;
};

class LegKinematicsSolver
{
public:
    struct SolverCache
    {
        float sinVal[3],cosVal[3];
    };
protected:
    LegStructure m_legStruct;
    bool m_bRight;
public:
    LegKinematicsSolver(bool right,LegStructure structure):
    m_legStruct(structure),m_bRight(right){}

    FeetMovement Solve(AxisMovement axisAngle,SolverCache& cache);
    AxisMovement Solve(FeetMovement movement,SolverCache& cache);

    FeetMovement FastSolve(
        AxisMovement axisAngle,
        SolverCache& cache,
        AxisMovement lastAngle);

    AxisMovement FastSolve(
        FeetMovement movement,
        SolverCache& cache,
        FeetMovement lastMove);

    Eigen::Matrix3f CalcTopForceJacbMat(AxisMovement angle,SolverCache& cache);
    Eigen::Matrix3f CalcFeetForceJacbMat(AxisMovement angle,SolverCache& cache);

    Eigen::Matrix3f FastCalcTopForceJacbMat(
        AxisMovement angle,
        SolverCache& cache,
        AxisMovement lastAngle);

    Eigen::Matrix3f FastCalcFeetForceJacbMat(
        AxisMovement angle,
        SolverCache& cache,
        AxisMovement lastAngle);
};

