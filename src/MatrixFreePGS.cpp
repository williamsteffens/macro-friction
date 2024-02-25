#include "MatrixFreePGS.h"

#include "Contact.h"
#include "CollisionUtils.hpp"
#include "Joint.h"
#include "RigidBody.h"
#include "RigidBodySystem.h"

namespace
{
    static inline void multAndSub(const JBlock& G, const Eigen::Vector3f& x, const Eigen::Vector3f& y, const float a, Eigen::VectorXf& b)
    {
            b -= a * G.col(0) * x(0);
            b -= a * G.col(1) * x(1);
            b -= a * G.col(2) * x(2);
            b -= a * G.col(3) * y(0);
            b -= a * G.col(4) * y(1);
            b -= a * G.col(5) * y(2);
    }

    static inline void buildRHS(Joint* j, float dt, Eigen::VectorXf& b)
    {
        const float hinv = 1.0f / dt;
        const float gamma = j->k / (j->k + hinv * j->b); // error reduction parameter
        const int dim = j->lambda.rows();
        b = hinv * hinv * gamma * j->phi;
        if( !j->body0->fixed )
        {
            multAndSub(j->J0Minv, j->body0->f, j->body0->tau, 1.0f, b);
            multAndSub(j->J0, j->body0->xdot, j->body0->omega, hinv, b);
        }
        if( !j->body1->fixed )
        {
            multAndSub(j->J1Minv, j->body1->f, j->body1->tau, 1.0f, b);
            multAndSub(j->J1, j->body1->xdot, j->body1->omega, hinv, b);
        }
    }

    // Loop over all other contacts involving _body
    //      and compute : x -= (J0*Minv0*Jother^T) * lambda_other
    //
    static inline void accumulateCoupledContactsAndJoints(Joint* j, const JBlock& JMinv, RigidBody* body, Eigen::VectorXf& x)
    {
        if( body->fixed )
            return;

        const int dim = j->lambda.rows();
        for(Contact* cc : body->contacts)
        {
            if( cc != j )
            {
                if( body == cc->body0 )
                {
                    x.noalias() = x - JMinv * (cc->J0.transpose() * cc->lambda);
                }
                else
                {
                    x.noalias() = x - JMinv * (cc->J1.transpose() * cc->lambda);
                }
            }
        }
        for(Joint* jj : body->joints)
        {
            if( jj != j )
            {
                const int otherDim = jj->lambda.rows();
                if( body == jj->body0 )
                {
                    x.noalias() = x - JMinv * (jj->J0.transpose() * jj->lambda);
                }
                else
                {
                    x.noalias() = x - JMinv * (jj->J1.transpose() * jj->lambda);
                }
            }
        }

    }

    static inline void solveContact(const Eigen::Matrix3f& A, const Eigen::VectorXf& b, Eigen::VectorXf& x, const CoeffsType& coeffs)
    {
        x(0) = std::max(0.0f, (b(0) - A(0,1) * x(1) - A(0,2) * x(2) ) / A(0,0) );
        x(1) = std::max(-coeffs[2]*x(0), std::min(coeffs[0]*x(0), ( b(1) - A(1,0) * x(0) - A(1,2) * x(2) ) / A(1,1) ));
        x(2) = std::max(-coeffs[3]*x(0), std::min(coeffs[1]*x(0), ( b(2) - A(2,0) * x(0) - A(2,1) * x(1) ) / A(2,2) ));
    }

    static inline void solveFrictionlessContact(const float aii, const float b, float& x)
    {
        x = std::max(0.0f, b / aii );
    }

    static inline void solveJoint(const Eigen::LLT<Eigen::MatrixXf>& LLT, const Eigen::VectorXf& b, Eigen::VectorXf& x)
    {
        x = LLT.solve(b);
    }
}

MatrixFreePGS::MatrixFreePGS(RigidBodySystem* _rigidBodySystem) : m_rigidBodySystem(_rigidBodySystem), m_dt(0.01f), m_maxIter(10)
{

}

void MatrixFreePGS::solve()
{
    const float hinv = 1.0f / m_dt;
    std::vector<Contact*>& contacts = m_rigidBodySystem->getContacts();
    std::vector<Joint*>& joints = m_rigidBodySystem->getJoints();
    const int numContacts = contacts.size();
    const int numJoints = joints.size();
    const int N = numJoints + numContacts;

    // Build diagonal matrices of bilateral joints
    std::vector<Eigen::LLT<Eigen::MatrixXf>> LLTjointii;
    if( numJoints > 0 )
    {
        // Build diagonal matrices
        LLTjointii.resize(numJoints);
        for(int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            const int dim = j->lambda.rows();
            const float eps = 1.0f / (m_dt * m_dt * j->k + m_dt * j->b);    // constraint force mixing

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            //
            Eigen::MatrixXf A = eps * Eigen::MatrixXf::Identity(dim, dim);

            if( !j->body0->fixed )
            {
                JBlockTranspose JT = j->J0.transpose();
                A += j->J0Minv * JT;
            }
            if( !j->body1->fixed )
            {
                JBlockTranspose JT = j->J1.transpose();
                A += j->J1Minv * JT;
            }

            LLTjointii[i] = A.llt();
        }
    }

    // Build diagonal matrices of contacts
    std::vector<Eigen::MatrixXf> Acontactii;
    if( numContacts > 0)
    {
        // Build diagonal matrices
        Acontactii.resize(numContacts);
        for(int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            const float eps = 1.0f / (m_dt * m_dt * c->k + m_dt * c->b);    // constraint force mixing

            // Compute the diagonal term : Aii = J0*Minv0*J0^T + J1*Minv1*J1^T
            //
            if( c->frictionless )
                Acontactii[i].setZero(1,1);
            else
                Acontactii[i].setZero(3,3);

            Acontactii[i](0,0) += eps;

            if( !c->body0->fixed )
            {
                JBlockTranspose JT = c->J0.transpose();
                Acontactii[i] += c->J0Minv * JT;
            }
            if( !c->body1->fixed )
            {
                JBlockTranspose JT = c->J1.transpose();
                Acontactii[i] += c->J1Minv * JT;
            }
        }
    }

    if( N > 0 )
    {
        std::vector<Eigen::VectorXf> b;
        b.resize(N);

        // Compute the right-hand side vector : b = phi - J*vel - dt*JMinvJT*force
        //
        for(int i = 0; i < numJoints; ++i)
        {
            Joint* j = joints[i];
            buildRHS(j, m_dt, b[i]);
        }
        for(int i = 0; i < numContacts; ++i)
        {
            Contact* c = contacts[i];
            buildRHS(c, m_dt, b[i+numJoints]);
            c->lambda.setZero();
        }

        // PGS main loop.
        // There is no convergence test here.
        // Stop after @a maxIter iterations.
        //
        for(int iter = 0; iter < m_maxIter; ++iter)
        {
            // For each joints, compute an updated value of joints[i]->lambda.
            //
            for(int i = 0; i < numJoints; ++i)
            {
                Joint* j = joints[i];
                const int dim = j->lambda.rows();
                const float eps = 1.0f / (m_dt * m_dt * j->k + m_dt * j->b);    // constraint force mixing

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[i];

                accumulateCoupledContactsAndJoints(j, j->J0Minv, j->body0, x);
                accumulateCoupledContactsAndJoints(j, j->J1Minv, j->body1, x);
                solveJoint(LLTjointii[i], x, j->lambda);
            }

            // For each contact, compute an updated value of contacts[i]->lambda
            //      using matrix-free pseudo-code provided in the course notes.
            //
            for(int i = 0; i < numContacts; ++i)
            {
                Contact* c = contacts[i];
                const float eps = 1.0f / (m_dt * m_dt * c->k + m_dt * c->b);    // constraint force mixing

                // Initialize current solution as x = b[i]
                Eigen::VectorXf x = b[numJoints+i];

                accumulateCoupledContactsAndJoints(c, c->J0Minv, c->body0, x);
                accumulateCoupledContactsAndJoints(c, c->J1Minv, c->body1, x);
                if( c->frictionless )
                {
                    solveFrictionlessContact(Acontactii[i](0,0), x(0), c->lambda(0));
                }
                else
                {
                    solveContact(Acontactii[i], x, c->lambda, c->coeffs);
                }
            }
        }
    }
}

