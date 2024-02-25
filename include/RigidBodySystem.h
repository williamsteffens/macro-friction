#pragma once

#include <Eigen/Dense>

#include <vector>
#include <unordered_map>

class Contact;
class Joint;
class MatrixFreePGS;
class RigidBody;

typedef std::function<void(std::vector<Contact*>&)> ContactFilterFunc;
typedef std::function<void(std::vector<RigidBody*>&)> PreStepFunc;
typedef std::function<void()> ResetFunc;

class RigidBodySystem
{
public:

    RigidBodySystem();

    virtual ~RigidBodySystem();

    void step(float _dt);

    // Remove all rigid bodies and cleanup the memory.
    void clear();

    // Add rigid body @a _b to the system. The rigid body is owned and managed by this RigidBodySystem.
    void addBody(RigidBody* _b);

    // Add joint @a _j to the system. The joint is owned and managed by this RigidBodySystem.
    void addJoint(Joint* _j);

    // Accessors for the body array.
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    std::vector<RigidBody*>& getBodies() { return m_bodies; }

    // Accessors for the contact constraint array.
    const std::vector<Contact*>& getContacts() const { return m_contacts; }
    std::vector<Contact*>& getContacts() { return m_contacts; }
    const std::vector<Joint*>& getJoints() const { return m_joints; }
    std::vector<Joint*>& getJoints() { return m_joints; }

    int getFrameCount() const { return m_frame; }

    // Contact solver params
    void setContactStiffness(float _stiffness) { m_contactStiffness = _stiffness; }
    void setContactDamping(float _damping) { m_contactDamping = _damping; }
    void setFrictionCoefficient(float _mu) { m_mu = _mu; }
    void setPGSIterations(int _pgsIter) { m_pgsIter = _pgsIter; }
    void setUseContactFrameAlign(bool _contactFrameAlign) { m_contactFrameAlign = _contactFrameAlign; }

    // Callbacks
    void setContactFilterFunc(ContactFilterFunc _func) { m_contactFilterFunc = _func; }
    void setPreStepFunc(PreStepFunc _func) { m_preStepFunc = _func; }
    void setResetFunc(ResetFunc _func) { m_resetFunc = _func; }

    // Collision detection
    void ignoreCollision(unsigned int _bodyIndex0, unsigned int _bodyIndex1);

private:

    std::vector<RigidBody*> m_bodies;
    std::vector<Contact*> m_contacts;
    std::vector<Joint*> m_joints;
    float m_contactStiffness, m_contactDamping;
    float m_mu;
    int m_pgsIter;
    int m_frame;
    bool m_contactFrameAlign;

    std::unordered_map<unsigned int, bool> m_collisionIgnore;

    // Compute the Jacobians for bilateral constraints (no contacts)
    void computeJointJacobians();

    // Compute the Jacobians for contacts
    void computeContactJacobians();

    // Compute the world-space inverse inertia matrices for all bodies.
    //  This function also updates the rotation matrices using the quaternions.
    void computeInertias();

    // Compute the constraint forces.
    void calcConstraintForces(float dt);

    // Perform collision detection. Only sphere-sphere and sphere-box collision are supported.
    void detectCollisions();

    // Cleanup contact constraints
    void cleanupContacts();

    // Cleanup joints
    void cleanupJoints();

    // Cleanup rigid bodies
    void cleanupBodies();

    ContactFilterFunc m_contactFilterFunc;
    PreStepFunc m_preStepFunc;
    ResetFunc m_resetFunc;

    MatrixFreePGS* m_solver;
};


template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator*(ScalarType a, const Eigen::Quaternion<ScalarType, Options>& q)
{
    return Eigen::Quaternion<ScalarType, Options>(a*q.w(),a*q.x(),a*q.y(),a*q.z());
}

template<typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator+(const Eigen::Quaternion<ScalarType, Options>& q1, const Eigen::Quaternion<ScalarType, Options>& q2)
{
    return Eigen::Quaternion<ScalarType, Options>(q1.w()+q2.w(),q1.x()+q2.x(),q1.y()+q2.y(),q1.z()+q2.z());
}
