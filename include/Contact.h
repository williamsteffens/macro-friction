#pragma once

#include "Joint.h"
#include "CollisionUtils.hpp"

#include <Eigen/Dense>

typedef float CoeffsType[4];

// Non-interpenetration contact constraint.
// This class stores the contact normal @a n and contact point @a p,
// as well as the Jacobians @a J0 and @a J1 for each body,
// which are computed by computeJacobian().
//
class Contact : public Joint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static float patchSize;  // physical extent of the patch (one side)
    static int patchDim;     // resolution of the contact patch

public:
    static float kA, kB;

    // Constructor with all parameters.
    Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _phi);

    Eigen::Vector3f p;          // The contact point.
    Eigen::Vector3f n;          // The contact normal.
    Eigen::Vector3f t1, t2;     // Tangent directions.
    MeshCollisionData meshData0;
    MeshCollisionData meshData1;
    CoeffsType coeffs;
    QRect contactPatch0;
    QRect contactPatch1;
    float mu;                   // Coefficient of friction.
    float pene;                 // Penetration
    bool frictionless;          // Flag for friction/no friction

    void computeCoeffs(CoeffsType& coeffs, const QImage& fboImage) const;

    virtual eConstraintType getType() const override { return kContact; }

    virtual void computeJacobian() override;

    void computeContactFrame(const Eigen::Vector3f& dir)
    {
        t1 = dir - (dir.dot(n))*n;
        // Compute tangent directions.
        //t2 = n.cross( dir );
        if( t1.norm() < 1e-3f )
        {
            t1 = -n.cross( Eigen::Vector3f(0,0,-1) );
        }
        t1.normalize();
        t2 = n.cross(t1);
        t2.normalize();
    }

protected:
    // Default constructor (hidden).
    Contact();

};
