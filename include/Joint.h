#pragma once

#include <Eigen/Dense>

typedef Eigen::Matrix<float, -1, 6, Eigen::RowMajor> JBlock;
typedef Eigen::Matrix<float, 6, -1, Eigen::ColMajor> JBlockTranspose;

class RigidBody;

enum eConstraintType { kContact = 0, kSpherical, kDistance };

// Joint class.
//
class Joint
{
public:

    // Constructor with all parameters.
    Joint(RigidBody* _body0, RigidBody* _body1);

    RigidBody* body0;           //< The first body
    RigidBody* body1;           //< The second boy
    JBlock J0;                  //< The Jacobian of body0
    JBlock J1;                  //< The Jacobian of body1
    JBlock J0Minv;
    JBlock J1Minv;
    Eigen::VectorXf phi;        //< Contraint error
    Eigen::VectorXf lambda;     //< Constraint impulse
    float k;                    //< Contact stiffness
    float b;                    //< Contact damping

    virtual eConstraintType getType() const = 0;

    virtual void computeJacobian() = 0;

protected:

    // Default constructor (hidden).
    Joint();

};
