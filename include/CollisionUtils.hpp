#pragma once

#include "MeshAssets.h"

#include <Eigen/Dense>
#include <QColor>

// Type for storing a tuple of vertex indices
struct MeshCollisionData
{
    MeshCollisionData() : i0(0), i1(0), i2(0), a(0.0f), b(0.0f), c(0.0f), p(), distance(FLT_MAX) {}

    unsigned int i0, i1, i2;    ///< Vertex indices
    float a, b, c;              ///< Barycentric coordinates
    Eigen::Vector3f p;          ///< Collision point (local coordinates)
    float distance;
};


// Compute barycentric coordinates of point p for triangle {v1,v2,v3}. The barycentric coordinates are stored in {u,v,w}.
//
static inline void barycentric(const Eigen::Vector3f& p, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, float& u, float& v, float& w, Eigen::Vector3f& pout)
{
    Eigen::Vector3f diff = p - v0;
    const Eigen::Vector3f edge0 = v1 - v0;
    const Eigen::Vector3f edge1 = v2 - v0;
    float a00 = edge0.dot(edge0);
    float a01 = edge0.dot(edge1);
    float a11 = edge1.dot(edge1);
    float b0 = -diff.dot(edge0);
    float b1 = -diff.dot(edge1);
    float const zero = 0.0;
    float const one = 1.0;
    float det = a00 * a11 - a01 * a01;
    float t0 = a01 * b1 - a11 * b0;
    float t1 = a01 * b0 - a00 * b1;

    if (t0 + t1 <= det)
    {
        if (t0 < zero)
        {
            if (t1 < zero)  // region 4
            {
                if (b0 < zero)
                {
                    t1 = zero;
                    if (-b0 >= a00)  // V1
                    {
                        t0 = one;
                    }
                    else  // E01
                    {
                        t0 = -b0 / a00;
                    }
                }
                else
                {
                    t0 = zero;
                    if (b1 >= zero)  // V0
                    {
                        t1 = zero;
                    }
                    else if (-b1 >= a11)  // V2
                    {
                        t1 = one;
                    }
                    else  // E20
                    {
                        t1 = -b1 / a11;
                    }
                }
            }
            else  // region 3
            {
                t0 = zero;
                if (b1 >= zero)  // V0
                {
                    t1 = zero;
                }
                else if (-b1 >= a11)  // V2
                {
                    t1 = one;
                }
                else  // E20
                {
                    t1 = -b1 / a11;
                }
            }
        }
        else if (t1 < zero)  // region 5
        {
            t1 = zero;
            if (b0 >= zero)  // V0
            {
                t0 = zero;
            }
            else if (-b0 >= a00)  // V1
            {
                t0 = one;
            }
            else  // E01
            {
                t0 = -b0 / a00;
            }
        }
        else  // region 0, interior
        {
            const float invDet = one / det;
            t0 *= invDet;
            t1 *= invDet;
        }
    }
    else
    {
        float tmp0, tmp1, numer, denom;

        if (t0 < zero)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - (2.0) * a01 + a11;
                if (numer >= denom)  // V1
                {
                    t0 = one;
                    t1 = zero;
                }
                else  // E12
                {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            }
            else
            {
                t0 = zero;
                if (tmp1 <= zero)  // V2
                {
                    t1 = one;
                }
                else if (b1 >= zero)  // V0
                {
                    t1 = zero;
                }
                else  // E20
                {
                    t1 = -b1 / a11;
                }
            }
        }
        else if (t1 < zero)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - (2.0) * a01 + a11;
                if (numer >= denom)  // V2
                {
                    t1 = one;
                    t0 = zero;
                }
                else  // E12
                {
                    t1 = numer / denom;
                    t0 = one - t1;
                }
            }
            else
            {
                t1 = zero;
                if (tmp1 <= zero)  // V1
                {
                    t0 = one;
                }
                else if (b0 >= zero)  // V0
                {
                    t0 = zero;
                }
                else  // E01
                {
                    t0 = -b0 / a00;
                }
            }
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            if (numer <= zero)  // V2
            {
                t0 = zero;
                t1 = one;
            }
            else
            {
                denom = a00 - (2.0) * a01 + a11;
                if (numer >= denom)  // V1
                {
                    t0 = one;
                    t1 = zero;
                }
                else  // 12
                {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            }
        }
    }

    u = one - t0 - t1;
    v = t0;
    w = t1;
    pout = v0 + t0 * edge0 + t1 * edge1;
}

// Return the barycentric coordinates of the closest point to @a p in the given VxGraphics::Mesh.
//
static inline bool barycentricClosestPoint(const Mesh& _mesh, const Eigen::Vector3f& _p, MeshCollisionData& _meshData, const Eigen::Vector3f* nfilter)
{
    float minDistance = FLT_MAX;
    _meshData.a = _meshData.b = _meshData.c = 0.0f;
    const unsigned int numVertices = _mesh.vertices.size();
    for(unsigned int i = 0; i < numVertices; i += 3)
    {

        const Eigen::Map<const Eigen::Vector3f> v0(_mesh.vertices[i].position);
        const Eigen::Map<const Eigen::Vector3f> v1(_mesh.vertices[i+1].position);
        const Eigen::Map<const Eigen::Vector3f> v2(_mesh.vertices[i+2].position);

        float a, b, c;
        Eigen::Vector3f tmpVec;
        barycentric(_p, v0, v1, v2, a, b, c, tmpVec);

        const Eigen::Vector3f n = Eigen::Vector3f(
                    a*_mesh.vertices[i].normal[0] + b*_mesh.vertices[i+1].normal[0] + c*_mesh.vertices[i+2].normal[0],
                    a*_mesh.vertices[i].normal[1] + b*_mesh.vertices[i+1].normal[1] + c*_mesh.vertices[i+2].normal[1],
                    a*_mesh.vertices[i].normal[2] + b*_mesh.vertices[i+1].normal[2] + c*_mesh.vertices[i+2].normal[2]);

        const float distance = (tmpVec - _p).norm();
        if (distance < minDistance && (nfilter == nullptr || std::abs(n.dot(*nfilter)) > 0.8f) )
        {
            minDistance = distance;
            _meshData.i0 = i;
            _meshData.i1 = i+1;
            _meshData.i2 = i+2;
            _meshData.a = a;
            _meshData.b = b;
            _meshData.c = c;
            _meshData.p = tmpVec;
            _meshData.distance = distance;
        }
    }

    return (minDistance < FLT_MAX);
}

static inline Eigen::Vector3f computeTextureNormal(const QImage& _image, float _u, float _v)
{
    const float i = (float)(_image.width()-1) * _u;
    const float j = (float)(_image.height()-1) * _v;
    QRgb rgb = _image.pixel((int)i, (int)j);

    const float x = (float) qRed(rgb) / 255.0f;
    const float y = (float) qGreen(rgb) / 255.0f;
    const float z = (float) qBlue(rgb) / 255.0f;

    return Eigen::Vector3f(2.0f*x-1.0f, 2.0f*y-1.0f, 2.0f*z-1.0f);
}

static inline Eigen::Vector3f computeTangent(const Mesh* _mesh, const MeshCollisionData& _meshData)
{
    return Eigen::Vector3f(
                _meshData.a * _mesh->vertices[_meshData.i0].tangent[0] + _meshData.b * _mesh->vertices[_meshData.i1].tangent[0] + _meshData.c * _mesh->vertices[_meshData.i2].tangent[0],
                _meshData.a * _mesh->vertices[_meshData.i0].tangent[1] + _meshData.b * _mesh->vertices[_meshData.i1].tangent[1] + _meshData.c * _mesh->vertices[_meshData.i2].tangent[1],
                _meshData.a * _mesh->vertices[_meshData.i0].tangent[2] + _meshData.b * _mesh->vertices[_meshData.i1].tangent[2] + _meshData.c * _mesh->vertices[_meshData.i2].tangent[2]
                );
}

static inline Eigen::Vector3f computePosition(const Mesh* _mesh, const MeshCollisionData& _meshData)
{
    return Eigen::Vector3f(
                _meshData.a * _mesh->vertices[_meshData.i0].position[0] + _meshData.b * _mesh->vertices[_meshData.i1].position[0] + _meshData.c * _mesh->vertices[_meshData.i2].position[0],
                _meshData.a * _mesh->vertices[_meshData.i0].position[1] + _meshData.b * _mesh->vertices[_meshData.i1].position[1] + _meshData.c * _mesh->vertices[_meshData.i2].position[1],
                _meshData.a * _mesh->vertices[_meshData.i0].position[2] + _meshData.b * _mesh->vertices[_meshData.i1].position[2] + _meshData.c * _mesh->vertices[_meshData.i2].position[2]
                );
}

static inline Eigen::Vector3f computeNormal(const Mesh* _mesh, const MeshCollisionData& _meshData)
{
    return Eigen::Vector3f(
                _meshData.a * _mesh->vertices[_meshData.i0].normal[0] + _meshData.b * _mesh->vertices[_meshData.i1].normal[0] + _meshData.c * _mesh->vertices[_meshData.i2].normal[0],
                _meshData.a * _mesh->vertices[_meshData.i0].normal[1] + _meshData.b * _mesh->vertices[_meshData.i1].normal[1] + _meshData.c * _mesh->vertices[_meshData.i2].normal[1],
                _meshData.a * _mesh->vertices[_meshData.i0].normal[2] + _meshData.b * _mesh->vertices[_meshData.i1].normal[2] + _meshData.c * _mesh->vertices[_meshData.i2].normal[2]
                );
}


static inline Eigen::Vector2f computeUV(const Mesh* _mesh, const MeshCollisionData& _meshData)
{
    Eigen::Vector2f uv = Eigen::Vector2f(
                _meshData.a * _mesh->vertices[_meshData.i0].uv[0] + _meshData.b * _mesh->vertices[_meshData.i1].uv[0] + _meshData.c * _mesh->vertices[_meshData.i2].uv[0],
                _meshData.a * _mesh->vertices[_meshData.i0].uv[1] + _meshData.b * _mesh->vertices[_meshData.i1].uv[1] + _meshData.c * _mesh->vertices[_meshData.i2].uv[1]
                );
    if( uv.x() < 0.0f || uv.x() > 1.0f ) uv.x() -= std::floor(uv.x());
    if( uv.y() < 0.0f || uv.y() > 1.0f ) uv.y() -= std::floor(uv.y());
    return uv;
}
