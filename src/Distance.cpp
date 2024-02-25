#include "Distance.h"
#include "RigidBody.h"

namespace
{
    static inline Eigen::Matrix3f hat(const Eigen::Vector3f& v)
    {
        Eigen::Matrix3f vhat;
        vhat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return vhat;
    }
}


Distance::Distance() : Joint(), r0(), r1(), d0(1.0f)
{

}

Distance::Distance(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _r0, const Eigen::Vector3f& _r1, float _d0) : Joint(_body0, _body1),
    r0(_r0), r1(_r1), d0(_d0)
{
    J0.setZero(1, 6);
    J1.setZero(1, 6);
    J0Minv.setZero(1, 6);
    J1Minv.setZero(1, 6);
    phi.setZero(1);
    lambda.setZero(1);
}

void Distance::computeJacobian()
{
    const Eigen::Vector3f rr0 = body0->R * r0;
    const Eigen::Vector3f rr1 = body1->R * r1;
    const Eigen::Vector3f p0 = body0->x + rr0;
    const Eigen::Vector3f p1 = body1->x + rr1;

    const Eigen::Vector3f d01 = p1 - p0;
    const float d = d01.norm();
    const Eigen::Vector3f n01 = (d > 1e-6f) ? d01 / d : Eigen::Vector3f(1,0,0);

    // compute constraint error
    phi(0) = d - d0;

    const Eigen::Vector3f a0 = p0.cross(n01);
    const Eigen::Vector3f a1 = n01.cross(p1);

    J0.block(0, 0, 1, 3) = n01.transpose();
    J0.block(0, 3, 1, 3) = a0.transpose();
    J1.block(0, 0, 1, 3) = -n01.transpose();
    J1.block(0, 3, 1, 3) = a1.transpose();
    J0Minv.block(0,0,1,3) = (1.0f/body0->mass) * J0.block(0, 0, 1, 3);
    J0Minv.block(0,3,1,3) = J0.block(0, 3, 1, 3) * body0->Iinv;
    J1Minv.block(0,0,1,3) = (1.0f/body1->mass) * J1.block(0, 0, 1, 3);
    J1Minv.block(0,3,1,3) = J1.block(0, 3, 1, 3) * body1->Iinv;

}

