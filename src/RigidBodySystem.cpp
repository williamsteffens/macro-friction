#include "RigidBodySystem.h"

#include "CollisionUtils.hpp"
#include "Contact.h"
#include "RigidBody.h"
#include "MatrixFreePGS.h"
#include "MeshAssets.h"

#include <QGLFramebufferObject>

#include <iostream>

namespace Eigen
{
    typedef Matrix<float, 6, 1, ColMajor> Vector6f;
}

namespace
{
    // Compute key used for the collision filter
    static inline unsigned int collisionKey(unsigned int _bodyIndex0, unsigned int _bodyIndex1)
    {
        return (0x0000FFFF & _bodyIndex0) + (0xFFFF0000 & (_bodyIndex1 << 16));
    }

    // Sphere-sphere collision test.
    // Assume that if this function is called, both @a body0 and @a body1
    // have a sphere collision geometry.
    //
    //  Returns:
    //       n - The contact normal.
    //       p - The contact point.
    //       pene - The penetration depth.
    //
    static inline bool collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1, Eigen::Vector3f& p, Eigen::Vector3f& n, float& pene)
    {
        Sphere* sphere0 = dynamic_cast<Sphere*>(body0->geometry.get());
        Sphere* sphere1 = dynamic_cast<Sphere*>(body1->geometry.get());

        // Implement sphere-sphere collision detection.
        // The function should check if a collision exists, and if it does
        // compute the contact normal, contact point, and penetration depth.
        //
        Eigen::Vector3f vec = body0->x - body1->x;

        const float rsum = (sphere0->radius + sphere1->radius);
        const float dist = vec.norm();
        if( dist < rsum )
        {
            n = vec / dist;
            p = 0.5f * ((body0->x - sphere0->radius*n) + (body1->x + sphere1->radius*n));
            pene = rsum-dist;
            return true;
        }

        return false;
    }

    // Plane-vertex collision test.
    static inline bool collisionDetectPointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n, float& pene)
    {
        const float dp = (p - plane_p).dot(plane_n);
        if (dp < 1e-5f)
        {
            pene = std::max(0.0f, -dp);
            return true;
        }
        return false;
    }

    // Plane-vertex collision test.
    static inline bool collisionDetectPointCylinder(const Eigen::Vector3f& p, Eigen::Vector3f& n, Eigen::Vector3f& t1, Eigen::Vector3f& t2, float& pene)
    {
        float dist = sqrt(p.x()*p.x()+p.z()*p.z()) - 0.041/2.0;
        if ( dist < 0 )
        {
            pene = -dist;
            n[0] = p[0];
            n[1] = 0;
            n[2] = p[2];
            n.normalize();
            // first build an aligned frame
            t1 = Eigen::Vector3f(0,1,0);
            t2 = n.cross(t1);

            double P = 0.7; // remove the 5 to get back to a normal pitch... try to make this visible?
            double width = 4 * 3.1415926535;
            double hyp = sqrt(P*P + width*width);
            double c = P/hyp;
            double s = width/hyp;

            t2[0] = s * t1[0] + c * t2[0];
            t2[1] = s * t1[1] + c * t2[1];
            t2[2] = s * t1[2] + c * t2[2];
            t1 = -n.cross(t2);

            return true;
        }
        return false;
    }

    static inline void collisionDetectBoxPlane(RigidBody* body0, RigidBody* body1, std::vector<Contact*>& contacts)
    {
        Box* box = dynamic_cast<Box*>(body0->geometry.get());
        Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());
        const Eigen::Vector3f pplane = body1->x;
        const Eigen::Vector3f nplane = body1->R * plane->normal;
        const Eigen::Vector3f plocal[8] = {
            0.5f*Eigen::Vector3f(-box->dim(0), -box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0), -box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0),  box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0),  box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0), -box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0), -box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0),  box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0),  box->dim(1),  box->dim(2))
        };

        for (int i = 0; i < 8; ++i)
        {
            const Eigen::Vector3f pbox = body0->R * plocal[i] + body0->x;
            float phi;
            if  ( collisionDetectPointPlane(pbox, pplane, nplane, phi) )
            {
                const int index = contacts.size();
                contacts.push_back( new Contact(body0, body1, pbox, nplane, phi) );
                body0->contacts.push_back( contacts[index] );
                body1->contacts.push_back( contacts[index] );
            }
        }
    }

    // Sphere-box collision test.
    //
    static inline bool collisionDetectSphereBox(RigidBody* body0, RigidBody* body1, Eigen::Vector3f& p, Eigen::Vector3f& n, float& pene)
    {
        Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
        Box* box = dynamic_cast<Box*>(body1->geometry.get());

        const Eigen::Vector3f clocal = body1->R.transpose() * (body0->x - body1->x);

        Eigen::Vector3f q(0,0,0);
        for(int i = 0; i < 3; ++i)
        {
            q[i] = std::max(-box->dim[i]/2.0f, std::min(box->dim[i]/2.0f, clocal[i]));
        }

        const Eigen::Vector3f dx = clocal - q;
        const float dist = dx.norm();
        if( dist < sphere->radius )
        {
            n = body1->R * (dx/dist);
            p = body1->R * q + body1->x;
            pene = sphere->radius - dist;
            return true;
        }

        return false;
    }


    static inline bool collisionDetectPointSDF(const Discregrid::CubicLagrangeDiscreteGrid& sdf, const Eigen::Vector3f& p, Eigen::Vector3f& n, float& phi)
    {
        if( !sdf.domain().contains(p) ) return false;

        // Interpolate to find the SD value
        Eigen::Vector3f grad;
        const double sdist = sdf.interpolate(0, p, &grad);

        if ( sdist < 1e-5 )
        {
            n = grad;
            phi = (float)-sdist;
            return true;
        }

        return false;
    }


    // Collision between a box (body0) and an SDF geometry (body1)
    static inline void collisionDetectSDFBox(RigidBody* body0, RigidBody* body1, std::vector<Contact*>& contacts)
    {
        SDFGeometry* geomSDF = dynamic_cast<SDFGeometry*>(body1->geometry.get());
        Box* box = dynamic_cast<Box*>(body0->geometry.get());
        const Eigen::Vector3f plocal[8] = {
            0.5f*Eigen::Vector3f(-box->dim(0), -box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0), -box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0),  box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f(-box->dim(0),  box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0), -box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0), -box->dim(1),  box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0),  box->dim(1), -box->dim(2)),
            0.5f*Eigen::Vector3f( box->dim(0),  box->dim(1),  box->dim(2))
        };

        for (int i = 0; i < 8; ++i)
        {
            // Compute box point in world coordinates
            const Eigen::Vector3f pbox = body0->R * plocal[i] + body0->x;

            // Transform the box point into local SDF coordinate
            const Eigen::Vector3f psdf = body1->R.transpose() * (pbox - body1->x);

            Eigen::Vector3f n;
            float phi;
            if( collisionDetectPointSDF(geomSDF->sdf, psdf, n, phi) )
            {
                Eigen::Vector3f nworld = body1->R * n;
                nworld.normalize();
                const int index = contacts.size();
                contacts.push_back( new Contact(body0, body1, pbox, nworld, phi) );
                body0->contacts.push_back( contacts[index] );
                body1->contacts.push_back( contacts[index] );
            }
        }
    }

    static inline void collisionDetectSdfCylinder(RigidBody* body0, RigidBody* body1, std::vector<Contact*>& contacts)
    {
        SDFGeometry* geomSDF = dynamic_cast<SDFGeometry*>(body0->geometry.get());
        //Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

        float phi;
        // Check : body0 vertices vs. body 1 cylinder
        for (std::size_t i = 0; i < geomSDF->mesh.nVertices(); ++i)
        {
            const Eigen::Vector3f& vert = geomSDF->mesh.vertex(i);
            const Eigen::Vector3f p0 = body0->R * vert + body0->x;
            const Eigen::Vector3f pplane = body1->x;

            Eigen::Vector3f n, t1, t2;
            if( collisionDetectPointCylinder(p0, n, t1, t2, phi) )
            {
                Contact* c = new Contact(body0, body1, p0, n, phi);
                c->t1 = t1;
                c->t2 = t2;
                contacts.push_back( c );
                body0->contacts.push_back( c );
                body1->contacts.push_back( c );
            }
        }
    }

    static inline void collisionDetectSdfPlane(RigidBody* body0, RigidBody* body1, std::vector<Contact*>& contacts)
    {
        SDFGeometry* geomSDF = dynamic_cast<SDFGeometry*>(body0->geometry.get());
        Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

        float phi;
        // Check : body0 vertices vs. body 1 SDF
        for(int i = 0; i < geomSDF->mesh.nVertices(); ++i)
        {
            const Eigen::Vector3f& vert = geomSDF->mesh.vertex(i);
            const Eigen::Vector3f p0 = body0->R * vert + body0->x;
            const Eigen::Vector3f pplane = body1->x;
            const Eigen::Vector3f nplane = body1->R * plane->normal;

            if( collisionDetectPointPlane(p0, pplane, nplane, phi) )
            {
                const int index = contacts.size();
                contacts.push_back( new Contact(body0, body1, p0, nplane, phi) );
                body0->contacts.push_back( contacts[index] );
                body1->contacts.push_back( contacts[index] );
            }
        }

    }

    static inline void collisionDetectSdfSdf(RigidBody* body0, RigidBody* body1, std::vector<Contact*>& contacts)
    {
        SDFGeometry* geomSDF0 = dynamic_cast<SDFGeometry*>(body0->geometry.get());
        SDFGeometry* geomSDF1 = dynamic_cast<SDFGeometry*>(body1->geometry.get());

        Eigen::Vector3f n;
        float phi;
        // Check : body0 vertices vs. body 1 SDF
        for(int i = 0; i < geomSDF0->mesh.nVertices(); ++i)
        {
            const Eigen::Vector3f& vert0 = geomSDF0->mesh.vertex(i);
            const Eigen::Vector3f p0 = body0->R * vert0 + body0->x;
            const Eigen::Vector3f p0_1 = body1->R.transpose() * (p0 - body1->x);

            if( collisionDetectPointSDF(geomSDF1->sdf, p0_1, n, phi) )
            {
                Eigen::Vector3f nworld = body1->R * n;
                nworld.normalize();
                const int index = contacts.size();
                contacts.push_back( new Contact(body0, body1, p0, nworld, phi) );
                body0->contacts.push_back( contacts[index] );
                body1->contacts.push_back( contacts[index] );
            }
        }
        // Check : body1 vertices vs. body 0 SDF
        for(int i = 0; i < geomSDF1->mesh.nVertices(); ++i)
        {
            const Eigen::Vector3f& vert1 = geomSDF1->mesh.vertex(i);
            const Eigen::Vector3f p1 = body1->R * vert1 + body1->x;
            const Eigen::Vector3f p1_0 = body0->R.transpose() * (p1 - body0->x);

            if( collisionDetectPointSDF(geomSDF0->sdf, p1_0, n, phi) )
            {
                Eigen::Vector3f nworld = body0->R * n;
                nworld.normalize();
                const int index = contacts.size();
                contacts.push_back( new Contact(body1, body0, p1, nworld, phi) );
                body0->contacts.push_back( contacts[index] );
                body1->contacts.push_back( contacts[index] );
            }
        }
    }

    static inline void computeVelocityAtPoint(const Eigen::Vector3f& v, const Eigen::Vector3f& omega, const Eigen::Vector3f& r, Eigen::Vector3f& vout)
    {
        vout = v + omega.cross(r);
    }
}


RigidBodySystem::RigidBodySystem() :
    m_contacts(),
    m_joints(),
    m_collisionIgnore(),
    m_frame(0),
    m_contactFrameAlign(false),
    m_contactStiffness(1e9f), m_contactDamping(1e8f),
    m_mu(0.4f), m_pgsIter(10), m_contactFilterFunc(nullptr), m_preStepFunc(nullptr), m_resetFunc(nullptr)
{
    m_solver = new MatrixFreePGS(this);
}

RigidBodySystem::~RigidBodySystem()
{
    clear();

    delete m_solver;
}

void RigidBodySystem::addBody(RigidBody *_b)
{
    _b->R = _b->q.toRotationMatrix();
    m_bodies.push_back(_b);
}

void RigidBodySystem::ignoreCollision(unsigned int _bodyIndex0, unsigned int _bodyIndex1)
{
    const unsigned int key = collisionKey(_bodyIndex0, _bodyIndex1);
    m_collisionIgnore[key] = true;
}

void RigidBodySystem::step(float dt)
{
    // Initialize the system.
    // Apply gravitional forces and reset angular forces.
    // Cleanup contacts from the previous time step.
    //
    cleanupContacts();
    for(RigidBody* b : m_bodies)
    {
        //b->f.setZero();
        b->f = b->mass * Eigen::Vector3f(0.f, -9.81f, 0.f);
        b->tau.setZero();
        b->f_c.setZero();
        b->tau_c.setZero();
        b->contacts.clear();
    }

    // Standard simulation pipeline.
    computeInertias();
    computeJointJacobians();

    if( m_preStepFunc )
    {
        m_preStepFunc(m_bodies);
    }

    detectCollisions();
    computeContactJacobians();

    // Update contact stiffness and damping
    //
    for(auto c : m_contacts)
    {
        c->k = m_contactStiffness;
        c->b = m_contactDamping;
        c->mu = m_mu;

        // Uncomment the block below to compute the barycentric coordinates of the contact point
        // on the mesh geometry.
        //
#if 0
        {
            const Eigen::Matrix3f RT = c->body0->R.transpose();
            const Eigen::Vector3f plocal = RT * (c->p - c->body0->x);
            const Eigen::Vector3f nlocal = RT * c->n;
            barycentricClosestPoint(*(c->body0->mesh), plocal, c->meshData0, &nlocal);
        }
        {
            const Eigen::Matrix3f RT = c->body1->R.transpose();
            const Eigen::Vector3f plocal = RT * (c->p - c->body1->x);
            const Eigen::Vector3f nlocal = RT * c->n;
            barycentricClosestPoint(*(c->body1->mesh), plocal, c->meshData1, &nlocal);
        }
#endif
    }

    if( m_contactFilterFunc )
    {
        m_contactFilterFunc(m_contacts);
    }

    for(RigidBody* b : m_bodies)
    {
        b->f_c.setZero();
        b->tau_c.setZero();
    }

    calcConstraintForces(dt);


    // Perform numerical integration of each rigid body in @a m_bodies
    // using the equations provided in the course notes and
    // the Baraff 2001 SIGGRAPH course.
    //
    for(RigidBody* b : m_bodies)
    {
        if( !b->fixed )
        {
            b->xdot += dt * (1.0f/b->mass) * (b->f + b->f_c);
            b->omega += dt * b->Iinv * (b->tau + b->tau_c - b->omega.cross(b->I * b->omega));
        }
        else
        {
            b->xdot.setZero();
            b->omega.setZero();
        }

        b->x += dt * b->xdot;
        b->q = b->q + 0.5f * dt * Eigen::Quaternionf(0, b->omega[0], b->omega[1], b->omega[2])*b->q;
        b->q.normalize();
    }

    ++m_frame;
}

void RigidBodySystem::clear()
{
    if( m_resetFunc )
    {
        m_resetFunc();
    }

    cleanupContacts();
    cleanupJoints();
    cleanupBodies();
    m_collisionIgnore.clear();
    m_frame = 0;
}

void RigidBodySystem::computeJointJacobians()
{
    for(auto j : m_joints)
    {
        j->computeJacobian();
    }


}

void RigidBodySystem::computeContactJacobians()
{
    static const float dt = 0.01f;

    if( m_contactFrameAlign )
    {
        // Do a frictionless solve
        for(auto c : m_contacts)
        {
            c->frictionless = true;
            c->computeJacobian();
        }
        calcConstraintForces(dt);

        // Computed updated body velocities based on frictionless solve
        for(auto c : m_contacts)
        {
            Eigen::Vector3f v0, v1;
            if( !c->body0->fixed )
            {
                const Eigen::Vector3f xdot = c->body0->xdot + dt * (1.0f/c->body0->mass) * (c->body0->f + c->body0->f_c);
                const Eigen::Vector3f omega = c->body0->omega + dt * c->body0->Iinv * (c->body0->tau + c->body0->tau_c);
                const Eigen::Vector3f r = c->p - c->body0->x;
                computeVelocityAtPoint(xdot, omega, r, v0);
            }
            else
            {
                v0.setZero();
            }

            if( !c->body1->fixed )
            {
                const Eigen::Vector3f xdot = c->body1->xdot + dt * (1.0f/c->body1->mass) * (c->body1->f + c->body1->f_c);
                const Eigen::Vector3f omega = c->body1->omega + dt * c->body1->Iinv * (c->body1->tau + c->body1->tau_c);
                const Eigen::Vector3f r = c->p - c->body1->x;
                computeVelocityAtPoint(xdot, omega, r, v1);
            }
            else
            {
                v1.setZero();
            }

            c->computeContactFrame(v0-v1);
            c->frictionless = false;
            c->computeJacobian();
        }
    }
    else
    {
        for(auto c : m_contacts)
        {
            // This could already have been computed by collision detection to align with special features... :/
            // TODO: THIS MUST BE PUT INTO COMMENTS FOR THE NUT AND BOLT EXAMPLE
            c->computeContactFrame( Eigen::Vector3f(1,0,0) );

            c->frictionless = false;
            c->computeJacobian();
        }
    }
}

void RigidBodySystem::computeInertias()
{
    for(RigidBody* b : m_bodies)
    {
        b->R = b->q.toRotationMatrix();
        b->updateInertiaMatrix();
    }
}

void RigidBodySystem::detectCollisions()
{
    Eigen::Vector3f p, n;
    float phi;
    for(unsigned int i = 0; i < m_bodies.size(); ++i)
    {
        for(unsigned int j = i+1; j < m_bodies.size(); ++j)
        {
            // Check if we should ignore collision between body i and j
            if( m_collisionIgnore.find(collisionKey(i,j)) != m_collisionIgnore.end() ||
                m_collisionIgnore.find(collisionKey(j,i)) != m_collisionIgnore.end() ) continue;

            RigidBody* body0 = m_bodies[i];
            RigidBody* body1 = m_bodies[j];

            // Test for sphere-sphere collision.
            if( body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere &&
                collisionDetectSphereSphere(body0, body1, p, n, phi) )
            {
                const int index = m_contacts.size();
                m_contacts.push_back( new Contact(body0, body1, p, n, phi) );
                body0->contacts.push_back( m_contacts[index] );
                body1->contacts.push_back( m_contacts[index] );
            }
            // Test for sphere-box collision
            else if( body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kBox &&
                collisionDetectSphereBox(body0, body1, p, n, phi) )
            {
                const int index = m_contacts.size();
                m_contacts.push_back( new Contact(body0, body1, p, n, phi) );
                body0->contacts.push_back( m_contacts[index] );
                body1->contacts.push_back( m_contacts[index] );
            }
            // Test for box-sphere collision (order swap)
            else if( body1->geometry->getType() == kSphere &&
                body0->geometry->getType() == kBox &&
                collisionDetectSphereBox(body1, body0, p, n, phi) )
            {
                const int index = m_contacts.size();
                m_contacts.push_back( new Contact(body1, body0, p, n, phi) );
                body0->contacts.push_back( m_contacts[index] );
                body1->contacts.push_back( m_contacts[index] );
            }
            // Test for plane-box collision
            else if( body1->geometry->getType() == kPlane &&
                     body0->geometry->getType() == kBox)
            {
                collisionDetectBoxPlane(body0, body1, m_contacts);
            }
            // Test for plane-box collision (order swap)
            else if ( body0->geometry->getType() == kPlane &&
                      body1->geometry->getType() == kBox)
            {
                collisionDetectBoxPlane(body1, body0, m_contacts);
            }
            // Test for SDF-box collision
            else if (body0->geometry->getType() == kSDF &&
                     body1->geometry->getType() == kBox)
            {
                collisionDetectSDFBox(body1, body0, m_contacts);
            }
            // Test for SDF-box collision (order swap)
            else if (body1->geometry->getType() == kSDF &&
                     body0->geometry->getType() == kBox)
            {
                collisionDetectSDFBox(body0, body1, m_contacts);
            }
            // Test for SDF-SDF collision
            else if( body0->geometry->getType() == kSDF &&
                     body1->geometry->getType() == kSDF )
            {
                collisionDetectSdfSdf(body0, body1, m_contacts);
            }
            // Test of SDF-plane collision
            else if( body0->geometry->getType() == kSDF &&
                     body1->geometry->getType() == kPlane )
            {
                collisionDetectSdfPlane(body0, body1, m_contacts);
            }
            // Test of SDF-plane collision (order swap)
            else if( body0->geometry->getType() == kPlane &&
                     body1->geometry->getType() == kSDF )
            {
                collisionDetectSdfPlane(body1, body0, m_contacts);
            }
            // SDF Cylinder
            else if( body0->geometry->getType() == kSDF && body1->geometry->getType() == kCylinder ) {
                collisionDetectSdfCylinder( body0, body1, m_contacts );
            } else if( body1->geometry->getType() == kSDF && body0->geometry->getType() == kCylinder ) {
                collisionDetectSdfCylinder( body1, body0, m_contacts );
            }

        }
    }
}

void RigidBodySystem::calcConstraintForces(float dt)
{
    // Solve for the constraint forces lambda
    //
    m_solver->setMaxIter(m_pgsIter);
    m_solver->solve();

    // Apply the constraint forces as forces and torques acting on each body.
    // Essentially, we compute contact forces by computing :
    //
    //       f_contact = J^T * lambda
    //
    // for each contact constraint.
    //
    for(const auto j : m_joints)
    {
        const Eigen::Vector6f f0 = j->J0.transpose() * j->lambda;
        const Eigen::Vector6f f1 = j->J1.transpose() * j->lambda;
        j->body0->f_c += f0.head<3>();
        j->body0->tau_c += f0.tail<3>();
        j->body1->f_c += f1.head<3>();
        j->body1->tau_c += f1.tail<3>();
    }
    for(const auto c : m_contacts)
    {
        const Eigen::Vector6f f0 = c->J0.transpose() * c->lambda;
        const Eigen::Vector6f f1 = c->J1.transpose() * c->lambda;
        c->body0->f_c += f0.head<3>();
        c->body0->tau_c += f0.tail<3>();
        c->body1->f_c += f1.head<3>();
        c->body1->tau_c += f1.tail<3>();
    }
}

void RigidBodySystem::cleanupContacts()
{
    for(auto c : m_contacts)
    {
        delete c;
    }
    m_contacts.clear();

    for(auto b : m_bodies)
    {
        b->contacts.clear();
    }
}

void RigidBodySystem::cleanupJoints()
{
    for(auto j : m_joints)
    {
        delete j;
    }
    m_joints.clear();
}

void RigidBodySystem::cleanupBodies()
{
    for(auto b : m_bodies)
    {
        delete b;
    }
    m_bodies.clear();
}

void RigidBodySystem::addJoint(Joint* _j)
{
    m_joints.push_back(_j);
    _j->body0->joints.push_back(_j);
    _j->body1->joints.push_back(_j);
}
