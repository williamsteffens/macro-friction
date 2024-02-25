#pragma once

#include "Distance.h"
#include "Spherical.h"
#include "MeshAssets.h"
#include<cstdlib>
#include <iostream>
#include "RigidBody.h"
#include "RigidBodySystem.h"

#include <Eigen/Dense>

class Scenarios
{
public:

    // Create box sliding on a plane
    static void createBoxOnSawtoothPlane(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_sawtooth.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.2f, 0.2f, 0.2f)), "resources/box_textured.obj");

        body0->fixed = true;
        //body0->q = Eigen::AngleAxisf(0.7f, Eigen::Vector3f(0, 1, 0));
        body0->materialId = RigidBody::kCoeffA;

        body1->materialId = RigidBody::kCoeffB;
        body1->x.y() = 0.12f;
        //body1->x.x() = 2.0f;
        //body1->xdot.x() = -4.0f;
        //body1->omega.y() = 10.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
    }

    // Create box on inclined plane
    static void createBoxOnInclinedPlane(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_checkerboard.obj");

        body0->fixed = true;
        body0->q = Eigen::AngleAxisf(0.24f, Eigen::Vector3f(0, 0, 1)) ;//* Eigen::AngleAxisf(1.57f, Eigen::Vector3f(0, 1, 0));

        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.2f, 0.2f, 0.2f)), "resources/box_textured.obj");
        body1->x = Eigen::Vector3f(1.5f, 1.0f, 0.1f);
        body1->q = Eigen::AngleAxisf(0.24f, Eigen::Vector3f(0, 0, 1));

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
    }

    // Create box on inclined plane
    static void createBoxOnRoughPlane(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_concrete.obj");


        body0->fixed = true;

        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(0.2f, 0.2f, 0.2f)), "resources/box_textured.obj");
        body1->x.y() = 0.15f;
        //body1->x.x() = 2.0f;
        //body1->xdot.x() = -4.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
    }

    // Create box on inclined plane
    static void createNutAndBolt(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* nut = new RigidBody(0.1f, new SDFGeometry("resources/nut.obj", {8,8,8}), "resources/nut.obj");
        RigidBody* bolt = new RigidBody(0.1f, new Cylinder(), "resources/bolt.obj");

        bolt->fixed = true;
//        nut->x = Eigen::Vector3f(0.0f, 0.199f, 0.0f);
//        nut->omega = Eigen::Vector3f(0.0f, -20.0f, 0.0f);
        nut->x = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        nut->q = Eigen::AngleAxisf(0.1f, Eigen::Vector3f(0, 1, 0));

        rigidBodySystem.addBody(nut);
        rigidBodySystem.addBody(bolt);
    }

    // Create box on inclined plane
    static void createSiggraphBump(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_teapot.obj");

        body0->materialId = RigidBody::kCoeffA;
        body0->fixed = true;
        body0->q = Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0)) * Eigen::AngleAxisf(-0.32f, Eigen::Vector3f(0, 0, 1));
        //body0->q = Eigen::AngleAxisf(0.32f, Eigen::Vector3f(0, 0, 1));

        RigidBody* body1 = new RigidBody(5.0f, new SDFGeometry("resources/bunny_smooth.obj", {8,8,8}), "resources/bunny_smooth.obj");
        body1->materialId = RigidBody::kCoeffB;
        body1->x = Eigen::Vector3f(1.5f, 1.0f, -0.6f);
        body1->xdot = Eigen::Vector3f(-1.0f, 0.0f, 0.0f);
        body1->q = Eigen::AngleAxisf(0.32f, Eigen::Vector3f(0, 0, 1));

        RigidBody* body2 = new RigidBody(10.0f, new SDFGeometry("resources/bunny_metal.obj", {8,8,8}), "resources/bunny_metal.obj");
        body2->materialId = RigidBody::kCoeffB;
        body2->x = Eigen::Vector3f(1.5f, 1.0f, 0.0f);
        body2->xdot = Eigen::Vector3f(-1.0f, 0.0f, 0.0f);
        body2->q = Eigen::AngleAxisf(0.32f, Eigen::Vector3f(0, 0, 1));

        RigidBody* body3 = new RigidBody(20.0f, new SDFGeometry("resources/bunny_rock.obj", {8,8,8}), "resources/bunny_rock.obj");
        body3->materialId = RigidBody::kCoeffB;
        body3->x = Eigen::Vector3f(1.5f, 1.0f, 0.6f);
        body3->xdot = Eigen::Vector3f(-1.0f, 0.0f, 0.0f);
        body3->q = Eigen::AngleAxisf(0.32f, Eigen::Vector3f(0, 0, 1));

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
    }

    // Two sphere colliding (horizontal)
    static void createTwoSpheresSimple(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Sphere(1.0f), "resources/sphere.obj");
        body0->x.x() = -3.0f;
        body0->xdot.x() = 4.0f;
        RigidBody* body1 = new RigidBody(1.0f, new Sphere(1.0f), "resources/sphere.obj");
        body1->x.x() = 3.0f;
        body1->xdot.x() = -4.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
    }

    // SDF example of box on incline.
    static void createCapsulesAndFunnel(RigidBodySystem& rigidBodySystem) {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/big_funnel.obj", {32,32,32}), "resources/big_funnel.obj");
        body0->fixed = true;
        rigidBodySystem.addBody(body0);

        float yoff = 0.2f;
        for(int i = 0; i < 3; ++i)
        {
            RigidBody* body1 = new RigidBody(0.1f, new SDFGeometry("resources/capsule.obj", {4,4,4}), "resources/capsule.obj");
            RigidBody* body2 = new RigidBody(0.1f, new SDFGeometry("resources/capsule.obj", {4,4,4}), "resources/capsule.obj");
            RigidBody* body3 = new RigidBody(0.1f, new SDFGeometry("resources/capsule.obj", {4,4,4}), "resources/capsule.obj");
            body1->x = Eigen::Vector3f(0.04f, yoff, -0.04f);
            body2->x = Eigen::Vector3f(-0.04f, yoff, -0.04f);
            body3->x = Eigen::Vector3f(0.00f, yoff, -0.02f);
            rigidBodySystem.addBody(body1);
            rigidBodySystem.addBody(body2);
            rigidBodySystem.addBody(body3);
            yoff += 0.1f;
        }
    }

    // Two spheres colliding (vertical). Bottom sphere is fixed.
    static void createTwoSpheresVertical(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Sphere(1.0f), "resources/sphere.obj");
        body0->x.y() = 2.0f;
        RigidBody* body1 = new RigidBody(1.0f, new Sphere(1.0f), "resources/sphere.obj");
        body1->x.y() = -2.0f;
        body1->fixed = true;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
    }

    // Create a stack of boxes and spheres.
    static void createStack(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* floor = new RigidBody(1.0f, new SDFGeometry("resources/thin_box.obj", {16,4,12}), "resources/thin_box.obj");
        floor->fixed = true;
        floor->x = Eigen::Vector3f(0.0f, -0.55f, 0.0f);
        rigidBodySystem.addBody(floor);

        const Eigen::Vector3f impulse(2.0f, 0.0f, 0.0f);
        {
            RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/pvc_cube.obj", {8,8,8}), "resources/pvc_cube.obj");
            RigidBody* body1 = new RigidBody(1.0f, new SDFGeometry("resources/pvc_cube.obj", {8,8,8}), "resources/pvc_cube.obj");
            RigidBody* body2 = new RigidBody(1.0f, new SDFGeometry("resources/pvc_cube.obj", {8,8,8}), "resources/pvc_cube.obj");
            RigidBody* body3 = new RigidBody(1.0f, new SDFGeometry("resources/pvc_cube.obj", {8,8,8}), "resources/pvc_cube.obj");
            RigidBody* body4 = new RigidBody(1.0f, new SDFGeometry("resources/pvc_cube.obj", {8,8,8}), "resources/pvc_cube.obj");
            //body0->fixed = true;
            body0->x = Eigen::Vector3f(0.0, -0.4f, 0.0f);
            body1->x = Eigen::Vector3f(0.0, -0.2f, 0.0f);
            body1->xdot = impulse;
            body2->x = Eigen::Vector3f(0.0, 0.0f, 0.0f);
            body3->x = Eigen::Vector3f(0.0, 0.2f, 0.0f);
            body4->x = Eigen::Vector3f(0.0, 0.4f, 0.0f);
            rigidBodySystem.addBody(body0);
            rigidBodySystem.addBody(body1);
            rigidBodySystem.addBody(body2);
            rigidBodySystem.addBody(body3);
            rigidBodySystem.addBody(body4);
        }

        {
            RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/oak_cube.obj", {8,8,8}), "resources/oak_cube.obj");
            RigidBody* body1 = new RigidBody(1.0f, new SDFGeometry("resources/oak_cube.obj", {8,8,8}), "resources/oak_cube.obj");
            RigidBody* body2 = new RigidBody(1.0f, new SDFGeometry("resources/oak_cube.obj", {8,8,8}), "resources/oak_cube.obj");
            RigidBody* body3 = new RigidBody(1.0f, new SDFGeometry("resources/oak_cube.obj", {8,8,8}), "resources/oak_cube.obj");
            RigidBody* body4 = new RigidBody(1.0f, new SDFGeometry("resources/oak_cube.obj", {8,8,8}), "resources/oak_cube.obj");
            //body0->fixed = true;
            body0->x = Eigen::Vector3f(0.5f, -0.4f, 0.0f);
            body1->x = Eigen::Vector3f(0.5f, -0.2f, 0.0f);
            body1->xdot = impulse;
            body2->x = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
            body3->x = Eigen::Vector3f(0.5f, 0.2f, 0.0f);
            body4->x = Eigen::Vector3f(0.5f, 0.4f, 0.0f);
            rigidBodySystem.addBody(body0);
            rigidBodySystem.addBody(body1);
            rigidBodySystem.addBody(body2);
            rigidBodySystem.addBody(body3);
            rigidBodySystem.addBody(body4);
        }

        {
            RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/acrylic_cube.obj", {8,8,8}), "resources/acrylic_cube.obj");
            RigidBody* body1 = new RigidBody(1.0f, new SDFGeometry("resources/acrylic_cube.obj", {8,8,8}), "resources/acrylic_cube.obj");
            RigidBody* body2 = new RigidBody(1.0f, new SDFGeometry("resources/acrylic_cube.obj", {8,8,8}), "resources/acrylic_cube.obj");
            RigidBody* body3 = new RigidBody(1.0f, new SDFGeometry("resources/acrylic_cube.obj", {8,8,8}), "resources/acrylic_cube.obj");
            RigidBody* body4 = new RigidBody(1.0f, new SDFGeometry("resources/acrylic_cube.obj", {8,8,8}), "resources/acrylic_cube.obj");
            //body0->fixed = true;
            body0->x = Eigen::Vector3f(-0.5f, -0.4f, 0.0f);
            body1->x = Eigen::Vector3f(-0.5f, -0.2f, 0.0f);
            body1->xdot = impulse;
            body2->x = Eigen::Vector3f(-0.5f, 0.0f, 0.0f);
            body3->x = Eigen::Vector3f(-0.5f, 0.2f, 0.0f);
            body4->x = Eigen::Vector3f(-0.5f, 0.4f, 0.0f);
            rigidBodySystem.addBody(body0);
            rigidBodySystem.addBody(body1);
            rigidBodySystem.addBody(body2);
            rigidBodySystem.addBody(body3);
            rigidBodySystem.addBody(body4);
        }
    }


    // Create box sliding on a plane
    static void createThreeBoxesSliding(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_steel.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box.obj");

        RigidBody* body2 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_castiron.obj");
        RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box.obj");

        RigidBody* body4 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_concrete.obj");
        RigidBody* body5 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box.obj");

        RigidBody* body6 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_smooth.obj");
        RigidBody* body7 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box.obj");

        const float smooth_zoff = 2.0f;
        const float light_zoff = 0.0f;
        const float med_zoff = -2.0f;
        const float heavy_zoff = -4.0f;

        body0->materialId = RigidBody::kCoeffA;
        body0->fixed = true;
        body0->x.y() = 0.0f;
        body0->x.z() = light_zoff;
        body0->q = Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0));
        body1->materialId = RigidBody::kCoeffB;
        body1->x.y() = 0.5f;
        body1->x.x() = 2.0f;
        body1->x.z() = light_zoff;
        body1->xdot.x() = -4.0f;

        body2->materialId = RigidBody::kCoeffA;
        body2->fixed = true;
        body2->x.y() = 0.0f;
        body2->x.z() = med_zoff;
        body2->q = Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0));
        body3->materialId = RigidBody::kCoeffB;
        body3->x.y() = 0.5f;
        body3->x.x() = 2.0f;
        body3->x.z() = med_zoff;
        body3->xdot.x() = -4.0f;

        body4->materialId = RigidBody::kCoeffA;
        body4->fixed = true;
        body4->x.y() = 0.0f;
        body4->x.z() = heavy_zoff;
        body4->q = Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0));
        body5->materialId = RigidBody::kCoeffB;
        body5->x.y() = 0.5f;
        body5->x.x() = 2.0f;
        body5->x.z() = heavy_zoff;
        body5->xdot.x() = -4.0f;

        body6->materialId = RigidBody::kCoeffA;
        body6->fixed = true;
        body6->x.y() = 0.0f;
        body6->x.z() = smooth_zoff;
        body6->q = Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0));
        body7->materialId = RigidBody::kCoeffB;
        body7->x.y() = 0.5f;
        body7->x.x() = 2.0f;
        body7->x.z() = smooth_zoff;
        body7->xdot.x() = -4.0f;

        rigidBodySystem.addBody(body4);
        rigidBodySystem.addBody(body5);
        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.addBody(body6);
        rigidBodySystem.addBody(body7);
        rigidBodySystem.ignoreCollision(1, 2);
        rigidBodySystem.ignoreCollision(1, 4);
        rigidBodySystem.ignoreCollision(1, 6);
        rigidBodySystem.ignoreCollision(3, 0);
        rigidBodySystem.ignoreCollision(3, 4);
        rigidBodySystem.ignoreCollision(3, 6);
        rigidBodySystem.ignoreCollision(5, 2);
        rigidBodySystem.ignoreCollision(5, 0);
        rigidBodySystem.ignoreCollision(5, 6);
        rigidBodySystem.ignoreCollision(7, 0);
        rigidBodySystem.ignoreCollision(7, 2);
        rigidBodySystem.ignoreCollision(7, 4);
    }

    // Create box sliding on a plane
    static void createTwoBoxesAnisotropic(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_wavy.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box_textured.obj");
        RigidBody* body2 = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_wavy_90deg.obj");
        RigidBody* body3 = new RigidBody(1.0f, new Box(Eigen::Vector3f(.2f, .2f, .2f)), "resources/box_textured.obj");

        body0->fixed = true;
        body0->x.z() = 1.0f;
        body1->x.y() = 0.2f;
        body1->x.x() = 2.0f;
        body1->x.z() = 1.0f;
        body1->xdot.x() = -4.0f;

        body2->fixed = true;
        body2->x.z() = -1.0f;
        body3->x.y() = 0.2f;
        body3->x.x() = 2.0f;
        body3->x.z() = -1.0f;
        body3->xdot.x() = -4.0f;

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addBody(body2);
        rigidBodySystem.addBody(body3);
        rigidBodySystem.ignoreCollision(1, 2);
        rigidBodySystem.ignoreCollision(3, 0);
    }


    static void createTwoBodySpherical(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(2.0f, 2.0f, 2.0f)),  "resources/box_textured.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Sphere(1.0f),  "resources/sphere.obj");

        body0->fixed = true;
        body0->x = Eigen::Vector3f(0.f, 4.f, 0.f);

        Spherical* ballJoint = new Spherical(body0, body1, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 4.f, 0.f));

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addJoint(ballJoint);

    }

    static void createTwoBodyDistance(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        RigidBody* body0 = new RigidBody(1.0f, new Box(Eigen::Vector3f(2.0f, 2.0f, 2.0f)),  "resources/box_textured.obj");
        RigidBody* body1 = new RigidBody(1.0f, new Box(Eigen::Vector3f(2.0f, 2.0f, 2.0f)),  "resources/box_textured.obj");

        body0->fixed = true;
        body0->x = Eigen::Vector3f(0.f, 1.f, 0.f);

        Distance* distanceJoint = new Distance(body0, body1, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, 0.f), 2.0f);

        rigidBodySystem.addBody(body0);
        rigidBodySystem.addBody(body1);
        rigidBodySystem.addJoint(distanceJoint);

    }

    static void createBolaRough(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/rod_rough.obj", {16,16,16}), "resources/rod_rough.obj");
        body0->fixed = true;
        body0->x.y() = 0.7f;
        body0->x.x() = 0.1f;
        body0->q = Eigen::AngleAxisf(0.5f, Eigen::Vector3f(1, 0, 0));
        rigidBodySystem.addBody(body0);

        RigidBody* prevBody = nullptr;
        RigidBody* body;

        for(int i = 0; i < 25; ++i)
        {
            if (i == 0)
            {
                body = new RigidBody(2.0f, new SDFGeometry("resources/small_sphere.obj", {4,4,4}), "resources/small_sphere.obj");
                body->xdot = Eigen::Vector3f(1.0f, 0.0f, -0.5f);
            }
            else if (i == 24)
            {
                body = new RigidBody(2.0f, new SDFGeometry("resources/small_sphere.obj", {4,4,4}), "resources/small_sphere.obj");
                body->xdot = Eigen::Vector3f(1.0f, 0.0f, 0.5f);
            }
            else
            {
                body = new RigidBody(0.08f, new SDFGeometry("resources/chain_link.obj", {2,8,4}), "resources/chain_link.obj");
                body->xdot = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
            }

            body->x = Eigen::Vector3f(0.f, 1.0f - (float)i * 0.036f, 0.f);
            if (i % 2 == 0)
                body->q = Eigen::AngleAxisf(1.0f, Eigen::Vector3f(0, 1, 0));

            rigidBodySystem.addBody(body);

            if( prevBody != nullptr )
            {
                Spherical* joint = new Spherical(prevBody, body, Eigen::Vector3f(0.0f, -0.02f, 0.f), Eigen::Vector3f(0.0f, 0.02f, 0.f));
                rigidBodySystem.addJoint(joint);

                rigidBodySystem.ignoreCollision(i, i+1);
            }

            prevBody = body;
        }
    }

    static void createBolaSmooth(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();
        RigidBody* body0 = new RigidBody(1.0f, new SDFGeometry("resources/rod_smooth.obj", {16,16,16}), "resources/rod_smooth.obj");
        body0->fixed = true;
        body0->x.y() = 0.7f;
        body0->x.x() = 0.1f;
        body0->q = Eigen::AngleAxisf(0.5f, Eigen::Vector3f(1, 0, 0));
        rigidBodySystem.addBody(body0);

        RigidBody* prevBody = nullptr;
        RigidBody* body;

        for(int i = 0; i < 25; ++i)
        {
            if (i == 0)
            {
                body = new RigidBody(2.0f, new SDFGeometry("resources/small_sphere.obj", {4,4,4}), "resources/small_sphere.obj");
                body->xdot = Eigen::Vector3f(8.0f, 0.0f, -0.5f);
            }
            else if (i == 24)
            {
                body = new RigidBody(2.0f, new SDFGeometry("resources/small_sphere.obj", {4,4,4}), "resources/small_sphere.obj");
                body->xdot = Eigen::Vector3f(8.0f, 0.0f, 0.5f);
            }
            else
            {
                body = new RigidBody(0.08f, new SDFGeometry("resources/chain_link.obj", {2,8,4}), "resources/chain_link.obj");
                body->xdot = Eigen::Vector3f(6.0f, 0.0f, 0.0f);
            }

            body->x = Eigen::Vector3f(0.f, 1.0f - (float)i * 0.036f, 0.f);
            if (i % 2 == 0)
                body->q = Eigen::AngleAxisf(1.0f, Eigen::Vector3f(0, 1, 0));

            rigidBodySystem.addBody(body);

            if( prevBody != nullptr )
            {
                Spherical* joint = new Spherical(prevBody, body, Eigen::Vector3f(0.0f, -0.02f, 0.f), Eigen::Vector3f(0.0f, 0.02f, 0.f));
                rigidBodySystem.addJoint(joint);

                rigidBodySystem.ignoreCollision(i, i+1);
            }

            prevBody = body;
        }
    }


    static void createCrawlingWorm(RigidBodySystem& rigidBodySystem)
    {
        rigidBodySystem.clear();

        const float rad = 0;
        const float dx = std::cos(rad);
        const float dy = std::sin(rad);
        RigidBody* floor = new RigidBody(1.0f, new Plane(Eigen::Vector3f(0.0f, 1.0f, 0.0f)), "resources/plane_steel.obj");
        floor->fixed = true;
        floor->q = Eigen::AngleAxisf(rad, Eigen::Vector3f(0, 0, 1));
        rigidBodySystem.addBody(floor);

        RigidBody* prevBody = nullptr;
        RigidBody* body;
        for(int i = 0; i < 5; ++i)
        {
            body = new RigidBody(0.1f,  new SDFGeometry("resources/worm_body.obj", {4,4,4}), "resources/worm_body.obj");
            body->x = Eigen::Vector3f(0.3f*dx*(float)i, 0.1f+0.3f*dy*(float)i, 0.0f);
            body->q =  Eigen::AngleAxisf(rad, Eigen::Vector3f(0, 0, 1)) * Eigen::AngleAxisf(3.14f, Eigen::Vector3f(0, 1, 0));

            rigidBodySystem.addBody(body);

            if( prevBody != nullptr )
            {
               rigidBodySystem.addJoint( new Distance(body, prevBody, Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, 0.f), 0.3f) );
               rigidBodySystem.ignoreCollision(i, i+1);
            }

            prevBody = body;
        }

    }

};
