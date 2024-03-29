#include "RigidBody.h"

#include "Contact.h"
#include "MeshAssets.h"
#include <Discregrid/All>

#include <iostream>

RigidBody::RigidBody(float _mass, Geometry* _geometry, const std::string& meshFilename) :
    fixed(false),
    mass(_mass),
    x(0,0,0),
    R(Eigen::Matrix3f::Identity()),
    xdot(0,0,0),
    omega(0,0,0),
    q(1,0,0,0),
    Ibody(Eigen::Matrix3f::Identity()),
    IbodyInv(Eigen::Matrix3f::Identity()),
    Iinv(Eigen::Matrix3f::Zero()),
    f(0,0,0),
    tau(0,0,0),
    f_c(0,0,0),
    tau_c(0,0,0),
    geometry(_geometry),
    contacts(),
    joints(),
    materialId(kCoeffA),
    mesh(nullptr)
{
    Ibody = geometry->computeInertia(mass);
    IbodyInv = Ibody.inverse();

    if( !meshFilename.empty() )
    {
        mesh = MeshAssetRegistry::loadObj(meshFilename);
    }
    contacts.clear();
    joints.clear();
}


void RigidBody::updateInertiaMatrix()
{
    if (!fixed)
    {
        I = R * Ibody * R.transpose();
        Iinv = R * IbodyInv * R.transpose();
    }
    else
    {
        Iinv.setZero();
    }
}

void RigidBody::addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force)
{
    const Eigen::Vector3f r = pos - x;
    f += force;
    tau += r.cross(force);
}

void RigidBody::getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel)
{
    const Eigen::Vector3f r = pos - x;
    vel = xdot + r.cross(omega);
}
