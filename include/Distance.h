#pragma once

#include "Joint.h"

// Distance joint class.
//
class Distance : public Joint
{
public:

    // Constructor with all parameters.
    Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, float _d01);

    virtual eConstraintType getType() const { return kDistance; }

    virtual void computeJacobian() override;

    Eigen::Vector3f r0;
    Eigen::Vector3f r1;

    float d0;   // neutral distance

protected:
    // Default constructor (hidden).
    Distance();

};
