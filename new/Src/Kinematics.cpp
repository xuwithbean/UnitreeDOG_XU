#include "Kinematics.h"

    //TODO:更改腿型，重新求值
LegStructure LegStructure::theStruct(0,0,0);
AxisMovement LegKinematicsSolver::Solve(FeetMovement movement,SolverCache& cache)//do not support acceleration!
{

    if(m_bRight)
    {
        float delta = (2.0f*movement.x*m_legStruct.shoulderLen)*(2.0f*movement.x*m_legStruct.shoulderLen)
            - 4.0f*(movement.x*movement.x + movement.z*movement.z)*
            (m_legStruct.shoulderLen*m_legStruct.shoulderLen - movement.z*movement.z);
        if (delta < 0)
            return AxisMovement(0,0,0,false);
        delta = (sqrtf(delta) + 2.0f*movement.x*m_legStruct.shoulderLen) / 2.0f
         / (movement.x*movement.x + movement.z*movement.z);
        if (delta > 1)
            delta = 1;
        float shoudler = acosf(delta);
        if (movement.x < m_legStruct.shoulderLen)
            shoudler *= -1;
        float projPos_x=movement.y,
        projPos_y = -m_legStruct.shoulderLen * tanf(shoudler) + movement.z / cosf(shoudler);

        float Lf = sqrtf(projPos_x*projPos_x + projPos_y*projPos_y);
        float arm = acosf(projPos_x / Lf);
        arm = arm + acosf((m_legStruct.armLen *m_legStruct.armLen + Lf * Lf - m_legStruct.feetLen * m_legStruct.feetLen)
         / (2.0f*m_legStruct.armLen*Lf));
        float angz = (m_legStruct.armLen * m_legStruct.armLen + m_legStruct.feetLen * m_legStruct.feetLen - Lf * Lf)/(2.0 * m_legStruct.armLen * m_legStruct.feetLen);
        if(angz > 1)
            angz = 1;
        else if(angz <= -1)
            angz = -1;
        float feet = acosf(angz);
        return AxisMovement(shoudler,arm,feet);
    }else
    {
        movement.x = -movement.x;
        float delta = (2.0f*movement.x*m_legStruct.shoulderLen)*(2.0f*movement.x*m_legStruct.shoulderLen)
            - 4.0f*(movement.x*movement.x + movement.z*movement.z)*
            (m_legStruct.shoulderLen*m_legStruct.shoulderLen - movement.z*movement.z);
        if (delta < 0)
            return AxisMovement(0,0,0,false);
        delta = (sqrtf(delta) + 2.0f*movement.x*m_legStruct.shoulderLen) / 2.0f
         / (movement.x*movement.x + movement.z*movement.z);
        if (delta > 1)
            delta = 1;
        float shoulder = acosf(delta);
        if (movement.x < m_legStruct.shoulderLen)
            shoulder *= -1;
        float projPos_x =  movement.y,
        projPos_y=-m_legStruct.shoulderLen * tanf(shoulder) + movement.z / cosf(shoulder);
        double Lf = sqrtf(projPos_x*projPos_x + projPos_y*projPos_y);
        float arm = acosf(projPos_x / Lf);
        arm = arm + acosf((m_legStruct.armLen*m_legStruct.armLen + Lf * Lf - m_legStruct.feetLen * m_legStruct.feetLen)
        / (2.0f*m_legStruct.armLen*Lf));
        double angz = (m_legStruct.armLen * m_legStruct.armLen + m_legStruct.feetLen * m_legStruct.feetLen - Lf * Lf)/(2.0 * m_legStruct.armLen * m_legStruct.feetLen);
        if(angz > 1)
            angz = 1;
        else if(angz <= -1)
            angz = -1;
        float feet = acos(angz);
        return AxisMovement(shoulder,arm,feet);
    }
}

FeetMovement LegKinematicsSolver::Solve(AxisMovement movement,SolverCache& cache)
{
    cache.sinVal[0] = sinf(movement.shoulderHorizontal);
    cache.cosVal[0] = cosf(movement.shoulderHorizontal);
    cache.sinVal[1] = sinf(movement.armRotation);
    cache.cosVal[1] = cosf(movement.armRotation);
    cache.sinVal[2] = sinf(movement.armFeetIntersect+movement.armRotation);
    cache.cosVal[2] = cosf(movement.armFeetIntersect+movement.armRotation);

    if(m_bRight)
    {
        Eigen::Vector3f pos(m_legStruct.shoulderLen* cache.cosVal[0],
        0,m_legStruct.shoulderLen*cache.sinVal[0]);
        Eigen::Matrix3f shoulderTrans;
        shoulderTrans << cache.cosVal[0],0,-cache.sinVal[0],
                         0              ,1,0               ,
                         cache.sinVal[0],0, cache.cosVal[0];

        Eigen::Vector2f feetPos_rel(
        m_legStruct.armLen*cache.cosVal[1]-m_legStruct.feetLen*cache.cosVal[2],
        -m_legStruct.armLen*cache.sinVal[1]+m_legStruct.feetLen*cache.sinVal[2]);

        Eigen::Vector3f feetPos(0,feetPos_rel.x(),feetPos_rel.y());
        pos += shoulderTrans*feetPos;
        return FeetMovement(pos.x(),pos.y(),pos.z());
    }else{
        Eigen::Vector3f pos(-m_legStruct.shoulderLen* cache.cosVal[0],
        m_legStruct.shoulderLen*cache.sinVal[0],0);
        Eigen::Matrix3f shoulderTrans;
        shoulderTrans << cache.cosVal[0],0,cache.sinVal[0],
                         0              ,1,0              ,
                        -cache.sinVal[0],0,cache.cosVal[0];

        Eigen::Vector2f feetPos_rel(
        m_legStruct.armLen*cache.cosVal[1]-m_legStruct.feetLen*cache.cosVal[2],
        -m_legStruct.armLen*cache.sinVal[1]+m_legStruct.feetLen*cache.sinVal[2]);
        Eigen::Vector3f feetPos(0,feetPos_rel.x(),feetPos_rel.y());
        pos += shoulderTrans*feetPos;
        return FeetMovement(pos.x(),pos.y(),pos.z());
    }
}

FeetMovement LegKinematicsSolver::FastSolve(AxisMovement axisAngle,SolverCache& solverCache,AxisMovement lastAngle)
{
    SolverCache cache;
    cache.sinVal[0] = solverCache.sinVal[0]+solverCache.cosVal[0]*(axisAngle.shoulderHorizontal-lastAngle.shoulderHorizontal);
    cache.cosVal[0] = solverCache.cosVal[0]-solverCache.sinVal[0]*(axisAngle.shoulderHorizontal-lastAngle.shoulderHorizontal);
    cache.sinVal[1] = solverCache.sinVal[1]+solverCache.cosVal[1]*(axisAngle.armRotation-lastAngle.armRotation);
    cache.cosVal[1] = solverCache.cosVal[1]-solverCache.sinVal[1]*(axisAngle.armRotation-lastAngle.armRotation);
    cache.sinVal[2] = solverCache.sinVal[2]+solverCache.cosVal[2]*(axisAngle.armRotation-lastAngle.armRotation+axisAngle.armFeetIntersect-lastAngle.armFeetIntersect);
    cache.cosVal[2] = solverCache.cosVal[2]-solverCache.sinVal[2]*(axisAngle.armRotation-lastAngle.armRotation+axisAngle.armFeetIntersect-lastAngle.armFeetIntersect);
    if(m_bRight)
    {
        Eigen::Vector3f pos(m_legStruct.shoulderLen* cache.cosVal[0],
        0,m_legStruct.shoulderLen*cache.sinVal[0]);
        Eigen::Matrix3f shoulderTrans;
        shoulderTrans << cache.cosVal[0],0,-cache.sinVal[0],
                         0              ,1,0               ,
                         cache.sinVal[0],0, cache.cosVal[0];

        Eigen::Vector2f feetPos_rel(
        m_legStruct.armLen*cache.cosVal[1]-m_legStruct.feetLen*cache.cosVal[2],
        -m_legStruct.armLen*cache.sinVal[1]+m_legStruct.feetLen*cache.sinVal[2]);

        Eigen::Vector3f feetPos(0,feetPos_rel.x(),feetPos_rel.y());
        pos += shoulderTrans*feetPos;
        return FeetMovement(pos.x(),pos.y(),pos.z());
    }else{
        Eigen::Vector3f pos(-m_legStruct.shoulderLen* cache.cosVal[0],
        m_legStruct.shoulderLen*cache.sinVal[0],0);
        Eigen::Matrix3f shoulderTrans;
        shoulderTrans << cache.cosVal[0],0,cache.sinVal[0],
                         0              ,1,0              ,
                        -cache.sinVal[0],0,cache.cosVal[0];

        Eigen::Vector2f feetPos_rel(
        m_legStruct.armLen*cache.cosVal[1]-m_legStruct.feetLen*cache.cosVal[2],
        -m_legStruct.armLen*cache.sinVal[1]+m_legStruct.feetLen*cache.sinVal[2]);
        Eigen::Vector3f feetPos(0,feetPos_rel.x(),feetPos_rel.y());
        pos += shoulderTrans*feetPos;
        return FeetMovement(pos.x(),pos.y(),pos.z());
    }
}
AxisMovement LegKinematicsSolver::FastSolve(FeetMovement movement,SolverCache& cache,FeetMovement lastMove)
{
    return Solve(movement,cache);//do not support!
}

Eigen::Matrix3f LegKinematicsSolver::CalcTopForceJacbMat(AxisMovement angle,SolverCache& cache)
{
    cache.sinVal[0] = sinf(angle.shoulderHorizontal);
    cache.cosVal[0] = cosf(angle.shoulderHorizontal);
    cache.sinVal[1] = sinf(angle.armRotation);
    cache.cosVal[1] = cosf(angle.armRotation);
    cache.sinVal[2] = sinf(angle.armFeetIntersect+ angle.armRotation);
    cache.cosVal[2] = cosf(angle.armFeetIntersect+ angle.armRotation);

    Eigen::Matrix3f jacobian;
    if(m_bRight)
    {
        jacobian(0, 0) = -cache.sinVal[0] * m_legStruct.shoulderLen
            - cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen);//m11 dx/d_theta
        jacobian(0, 1) = -cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen);//m12 dx/d_phi
        jacobian(0, 2) = -cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m13 dx/d_psi
        jacobian(1, 0) = 0;//m21 dy/d_theta
        jacobian(1, 1) = -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen;//m22 dy/d_phi
        jacobian(1, 2) = cache.sinVal[2] * m_legStruct.feetLen;//m23 dy/d_psi
        jacobian(2, 0) = cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen);//m31 dz/d_theta
        jacobian(2, 1) = cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen);//m32 dz/d_phi
        jacobian(2, 2) = cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }else
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            + cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }
    return -1*jacobian;

}
Eigen::Matrix3f LegKinematicsSolver::CalcFeetForceJacbMat(AxisMovement angle,SolverCache& cache)
{
    cache.sinVal[0] = sinf(angle.shoulderHorizontal);
    cache.cosVal[0] = cosf(angle.shoulderHorizontal);
    cache.sinVal[1] = sinf(angle.armRotation);
    cache.cosVal[1] = cosf(angle.armRotation);
    cache.sinVal[2] = sinf(angle.armFeetIntersect+ angle.armRotation);
    cache.cosVal[2] = cosf(angle.armFeetIntersect+ angle.armRotation);
    Eigen::Matrix3f jacobian;
    if(m_bRight)
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            - cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            -cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            -cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }else
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            + cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }
    return jacobian;

}

Eigen::Matrix3f LegKinematicsSolver::FastCalcTopForceJacbMat(AxisMovement angle,SolverCache& solverCache,AxisMovement lastAngle)
{
    SolverCache cache;
    cache.sinVal[0] = solverCache.sinVal[0] + solverCache.cosVal[0] * (angle.shoulderHorizontal - lastAngle.shoulderHorizontal);
    cache.cosVal[0] = solverCache.cosVal[0] - solverCache.sinVal[0] * (angle.shoulderHorizontal - lastAngle.shoulderHorizontal);
    cache.sinVal[1] = solverCache.sinVal[1] + solverCache.cosVal[1] * (angle.armRotation - lastAngle.armRotation);
    cache.cosVal[1] = solverCache.cosVal[1] - solverCache.sinVal[1] * (angle.armRotation - lastAngle.armRotation);
    cache.sinVal[2] = solverCache.sinVal[2] + solverCache.cosVal[2] * (angle.armRotation - lastAngle.armRotation + angle.armFeetIntersect - lastAngle.armFeetIntersect);
    cache.cosVal[2] = solverCache.cosVal[2] - solverCache.sinVal[2] * (angle.armRotation - lastAngle.armRotation + angle.armFeetIntersect - lastAngle.armFeetIntersect);
    Eigen::Matrix3f jacobian;
    if(m_bRight)
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            - cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            -cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            -cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }else
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            + cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }
    return -1*jacobian;

}
Eigen::Matrix3f LegKinematicsSolver::FastCalcFeetForceJacbMat(AxisMovement angle,SolverCache& solverCache,AxisMovement lastAngle)
{
    SolverCache cache;
    cache.sinVal[0] = solverCache.sinVal[0] + solverCache.cosVal[0] * (angle.shoulderHorizontal - lastAngle.shoulderHorizontal);
    cache.cosVal[0] = solverCache.cosVal[0] - solverCache.sinVal[0] * (angle.shoulderHorizontal - lastAngle.shoulderHorizontal);
    cache.sinVal[1] = solverCache.sinVal[1] + solverCache.cosVal[1] * (angle.armRotation - lastAngle.armRotation);
    cache.cosVal[1] = solverCache.cosVal[1] - solverCache.sinVal[1] * (angle.armRotation - lastAngle.armRotation);
    cache.sinVal[2] = solverCache.sinVal[2] + solverCache.cosVal[2] * (angle.armRotation - lastAngle.armRotation + angle.armFeetIntersect - lastAngle.armFeetIntersect);
    cache.cosVal[2] = solverCache.cosVal[2] - solverCache.sinVal[2] * (angle.armRotation - lastAngle.armRotation + angle.armFeetIntersect - lastAngle.armFeetIntersect);
    Eigen::Matrix3f jacobian;
    if(m_bRight)
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            - cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            -cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            -cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }else
    {
        jacobian <<
            -cache.sinVal[0] * m_legStruct.shoulderLen
            + cache.cosVal[0] * (
                -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m11 dx/d_theta
            cache.sinVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m12 dx/d_phi
            cache.sinVal[0] * (cache.cosVal[2] * m_legStruct.feetLen),//m13 dx/d_psi
            0,//m21 dy/d_theta
            -cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen,//m22 dy/d_phi
            cache.sinVal[2] * m_legStruct.feetLen,//m23 dy/d_psi
            cache.cosVal[0] * m_legStruct.shoulderLen
            - cache.sinVal[0] * (-cache.sinVal[1] * m_legStruct.armLen + cache.sinVal[2] * m_legStruct.feetLen),//m31 dz/d_theta
            cache.cosVal[0] * (-cache.cosVal[1] * m_legStruct.armLen + cache.cosVal[2] * m_legStruct.feetLen),//m32 dz/d_phi
            cache.cosVal[0] * (cache.cosVal[2] * m_legStruct.feetLen);//m33 dz/d_psi
    }
    return jacobian;
}
