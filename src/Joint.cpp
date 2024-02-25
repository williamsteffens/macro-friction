#include "Joint.h"
#include "RigidBody.h"

Joint::Joint() :
    body0(nullptr), body1(nullptr),
    k(1e5f), b(1e4f)
{

}


Joint::Joint(RigidBody* _body0, RigidBody* _body1) :
    body0(_body0), body1(_body1), k(1e5f), b(1e4f)
{

}

