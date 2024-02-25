#pragma once

#include "Geometry.hpp"
#include <Eigen/Dense>

#include <vector>

class Contact;
class Joint;
struct Mesh;

// Rigid body class.
// Stores properties for rigid body simulation, including
// the geometry and list of contact constraints.
//
class RigidBody
{
public:
    enum eMaterialId { kCoeffA = 0, kCoeffB };

    RigidBody(float _mass, Geometry* _geometry, const std::string& meshFilename = "");

    void updateInertiaMatrix();
    void addForceAtPos(const Eigen::Vector3f& pos, const Eigen::Vector3f& force);
    void getVelocityAtPos(const Eigen::Vector3f& pos, Eigen::Vector3f& vel);

    bool fixed;                         ///< Flag for a fixed rigid body. Default = false.
    float mass;                         ///< Mass.
    Eigen::Matrix3f I, Iinv;            ///< Inertia and inverse inertia matrix (world space)
    Eigen::Matrix3f Ibody, IbodyInv;    ///< Inertia and inverse inertia in the local body frame.
    Eigen::Vector3f x;                  ///< Position.
    Eigen::Quaternionf q;               ///< Orientation.

    Eigen::Matrix3f R;                  ///< Rotation matrix (auxiliary).
    Eigen::Vector3f xdot;               ///< Linear velocity.
    Eigen::Vector3f omega;              ///< Angular velocity.
    Eigen::Vector3f f;                  ///< Linear force.
    Eigen::Vector3f tau;                ///< Angular force (torque).

    Eigen::Vector3f f_c;                ///< Linear force (constraints)
    Eigen::Vector3f tau_c;              ///< Angular force (constraints).

    std::unique_ptr<Geometry> geometry; ///< The geometry of the rigid body.
    std::vector<Contact*> contacts;     ///< Pointer array of contact constraints involving this body.
    std::vector<Joint*> joints;         ///< Pointer array of joints involving this body.

    Mesh* mesh;                         ///< Used for rendering
    eMaterialId materialId;             ///< Material id used for friction texture

};
