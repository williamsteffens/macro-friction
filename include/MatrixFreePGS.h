#pragma once

#include <Eigen/Dense>

class RigidBodySystem;

// Matrix-free Projected Gauss-Seidel.
//
class MatrixFreePGS
{
public:

    MatrixFreePGS(RigidBodySystem* _rigidBodySystem);

    void setTimestep(float _dt) { m_dt = _dt; }

    void setMaxIter(int _maxIter) { m_maxIter = _maxIter; }

    // Implement solve method that solves for the constraint forces in @a m_rigidBodySystem.
    //
    virtual void solve();

protected:

    float m_dt;
    int m_maxIter;
    RigidBodySystem* m_rigidBodySystem;

};
