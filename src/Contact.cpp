#include "Contact.h"
#include "RigidBody.h"

namespace
{

    static inline QImage createSubImage(const QImage* image, const QRect & rect)
    {
        const size_t offset = rect.x() * image->depth() / 8
                + rect.y() * image->bytesPerLine();
        return QImage(image->bits() + offset, rect.width(), rect.height(),
                      image->bytesPerLine(), image->format());
    }

    static inline Eigen::Vector3f computeNormalFromMesh(RigidBody* body, const MeshCollisionData& meshData)
    {
        const Eigen::Vector3f t = computeTangent(body->mesh, meshData);
        const Eigen::Vector3f n = computeNormal(body->mesh, meshData);
        const Eigen::Vector3f b = n.cross(t);
        const Eigen::Vector2f uv = computeUV(body->mesh, meshData);

        Eigen::Vector3f nworld;
        if( body->mesh->material.map_bump != nullptr )
        {
            const Eigen::Vector3f textureN = computeTextureNormal(body->mesh->material.img_bump, uv[0], uv[1]);

            Eigen::Matrix3f TBN;
            TBN.col(0) = t;
            TBN.col(1) = b;
            TBN.col(2) = n;

            nworld = body->R * TBN * textureN;
        }
        else
        {
            nworld = body->R * n;
        }

        nworld.normalize();
        return nworld;
    }


    static inline float ploughFunc(const Eigen::Vector3f& n0, const Eigen::Vector3f& n1, const Eigen::Vector3f& nmean, const Eigen::Vector3f& v)
    {
        static const float s_2_pi = (2.0f / 3.14159f);

        const float dp0 = -nmean.dot(n0);
        const float dp1 = nmean.dot(n1);
        const float ang0 = std::acos( std::max(-1.0f, std::min(1.0f, dp0)) );
        const float ang1 = std::acos( std::max(-1.0f, std::min(1.0f, dp1)) );
        const float mu1 = s_2_pi * std::max(0.0f, -n0.dot(v)) * std::tan( std::min(1.57f, std::max(-1.57f, ang0)) );
        const float mu2 = s_2_pi * std::max(0.0f, n1.dot(v)) * std::tan( std::min(1.57f, std::max(-1.57f, ang1)) );

        return (mu1 + mu2);
    }

    static inline float plasticFunc(const Eigen::Vector3f& n0, const Eigen::Vector3f& n1, const Eigen::Vector3f& nmean, const Eigen::Vector3f& v)
    {
        static const float pi_2 = 1.570796f;
        const float dp0 = -nmean.dot(n0);
        const float dp1 = nmean.dot(n1);
        const float ang0 = pi_2 + std::acos( std::max(-1.0f, std::min(1.0f, dp0)) );
        const float ang1 = pi_2 + std::acos( std::max(-1.0f, std::min(1.0f, dp1)) );
        const float mu1 = std::max(0.0f, -n0.dot(v)) * std::tan( std::asin(0.3536f*(2.0f+ang0)/(1.0f+ang0)) );
        const float mu2 = std::max(0.0f, n1.dot(v)) * std::tan( std::asin(0.3536f*(2.0f+ang1)/(1.0f+ang1)) );
        return (mu1 + mu2);
    }

    static inline void convolve(const QImage& patch0, const QImage& patch1, CoeffsType& ploughCoeffsA, CoeffsType& ploughCoeffsB)
    {
        assert(patch0.width() == patch1.width());
        assert(patch0.height() == patch1.height());

        const int width = patch0.width();
        const int height = patch0.height();

        memset(ploughCoeffsA, 0, sizeof(CoeffsType));
        memset(ploughCoeffsB, 0, sizeof(CoeffsType));
        int counter = 0;
        for(int yi = 0; yi < height; ++yi)
        {
            for(int xi = 0; xi < width; ++xi)
            {
                const QRgb p0 = patch0.pixel(xi, yi);
                const QRgb p1 = patch1.pixel(xi, yi);

                if( p0 < 1 || p1 < 1 ) continue;

                ploughCoeffsA[0] += ((float)qRed(p0) / 255.0f);
                ploughCoeffsA[1] += ((float)qGreen(p0) / 255.0f);
                ploughCoeffsA[2] += ((float)qBlue(p0) / 255.0f);
                ploughCoeffsA[3] += ((float)qAlpha(p0) / 255.0f);
                ploughCoeffsB[0] += ((float)qRed(p1) / 255.0f);
                ploughCoeffsB[1] += ((float)qGreen(p1) / 255.0f);
                ploughCoeffsB[2] += ((float)qBlue(p1) / 255.0f);
                ploughCoeffsB[3] += ((float)qAlpha(p1) / 255.0f);
                ++counter;
            }
        }

        if( counter > 0 )
        {
            ploughCoeffsA[0] /= (float)counter;
            ploughCoeffsA[1] /= (float)counter;
            ploughCoeffsA[2] /= (float)counter;
            ploughCoeffsA[3] /= (float)counter;
            ploughCoeffsB[0] /= (float)counter;
            ploughCoeffsB[1] /= (float)counter;
            ploughCoeffsB[2] /= (float)counter;
            ploughCoeffsB[3] /= (float)counter;
        }

    }
}

float Contact::kA = 1.0f;
float Contact::kB = 1.0f;

float Contact::patchSize = 0.02f;
int Contact::patchDim = 16;

Contact::Contact() : Joint(),
    p(), n(), t1(), t2(), mu(0.4f), frictionless(false),
    contactPatch0(0,0,Contact::patchDim,Contact::patchDim),
    contactPatch1(0,0,Contact::patchDim,Contact::patchDim)
{
    memset(coeffs, 0, sizeof(coeffs));
}

Contact::Contact(RigidBody* _body0, RigidBody* _body1, const Eigen::Vector3f& _p, const Eigen::Vector3f& _n, float _pene) : Joint(_body0, _body1),
    p(_p), n(_n), t1(), t2(), mu(0.4f), pene(_pene), frictionless(false)
{
    J0.setZero(3, 6);
    J1.setZero(3, 6);
    J0Minv.setZero(3, 6);
    J1Minv.setZero(3, 6);
    lambda.setZero(3);
    phi.setZero(3);
    phi(0) = _pene;
    memset(coeffs, 0, sizeof(coeffs));
}

void Contact::computeJacobian()
{
    // Compute the Jacobians J0 and J1
    //
    const Eigen::Vector3f r0 = p - body0->x;
    const Eigen::Vector3f r1 = p - body1->x;

    if( frictionless )
    {
        J0.setZero(1, 6);
        J1.setZero(1, 6);
        J0Minv.setZero(1, 6);
        J1Minv.setZero(1, 6);
        lambda.setZero(1);
        phi.setZero(1);
        phi(0) = pene;

        // normal
        J0.block(0, 0, 1, 3) = n.transpose();
        J0.block(0, 3, 1, 3) = r0.cross(n).transpose();
        J1.block(0, 0, 1, 3) = -n.transpose();
        J1.block(0, 3, 1, 3) = -r1.cross(n).transpose();

        J0Minv.block(0,0,1,3) = (1.0f/body0->mass) * J0.block(0, 0, 1, 3);
        J0Minv.block(0,3,1,3) = J0.block(0, 3, 1, 3) * body0->Iinv;
        J1Minv.block(0,0,1,3) = (1.0f/body1->mass) * J1.block(0, 0, 1, 3);
        J1Minv.block(0,3,1,3) = J1.block(0, 3, 1, 3) * body1->Iinv;
    }
    else
    {
        J0.setZero(3, 6);
        J1.setZero(3, 6);
        J0Minv.setZero(3, 6);
        J1Minv.setZero(3, 6);
        lambda.setZero(3);
        phi.setZero(3);
        phi(0) = pene;

        // normal
        J0.block(0, 0, 1, 3) = n.transpose();
        J0.block(0, 3, 1, 3) = r0.cross(n).transpose();
        J1.block(0, 0, 1, 3) = -n.transpose();
        J1.block(0, 3, 1, 3) = -r1.cross(n).transpose();
        // tangent 1
        J0.block(1, 0, 1, 3) = t1.transpose();
        J0.block(1, 3, 1, 3) = r0.cross(t1).transpose();
        J1.block(1, 0, 1, 3) = -t1.transpose();
        J1.block(1, 3, 1, 3) = -r1.cross(t1).transpose();
        // tangent 2
        J0.block(2, 0, 1, 3) = t2.transpose();
        J0.block(2, 3, 1, 3) = r0.cross(t2).transpose();
        J1.block(2, 0, 1, 3) = -t2.transpose();
        J1.block(2, 3, 1, 3) = -r1.cross(t2).transpose();

        J0Minv.block(0,0,3,3) = (1.0f/body0->mass) * J0.block(0, 0, 3, 3);
        J0Minv.block(0,3,3,3) = J0.block(0, 3, 3, 3) * body0->Iinv;
        J1Minv.block(0,0,3,3) = (1.0f/body1->mass) * J1.block(0, 0, 3, 3);
        J1Minv.block(0,3,3,3) = J1.block(0, 3, 3, 3) * body1->Iinv;
    }
}

void Contact::computeCoeffs(CoeffsType& coeffs, const QImage& fboImage) const
{
    CoeffsType ploughCoeffs0, ploughCoeffs1;

    convolve( createSubImage(&fboImage, contactPatch0), createSubImage(&fboImage, contactPatch1), ploughCoeffs0, ploughCoeffs1);

    const float k0 = (body0->materialId == RigidBody::kCoeffA) ? kA : kB;
    const float k1 = (body1->materialId == RigidBody::kCoeffA) ? kA : kB;
    coeffs[0] = mu + k0*ploughCoeffs0[0] + k1*ploughCoeffs1[0];
    coeffs[1] = mu + k0*ploughCoeffs0[1] + k1*ploughCoeffs1[1];
    coeffs[2] = mu + k0*ploughCoeffs0[2] + k1*ploughCoeffs1[2];
    coeffs[3] = mu + k0*ploughCoeffs0[3] + k1*ploughCoeffs1[3];
}
