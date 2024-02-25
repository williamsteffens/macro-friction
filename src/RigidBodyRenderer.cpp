#include "RigidBodyRenderer.h"

#include "CollisionUtils.hpp"
#include "Contact.h"
#include "RigidBody.h"
#include "RigidBodySystem.h"
#include "OBJLoader.h"

#include <QOpenGLFunctions_4_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLFramebufferObject>

#include <cassert>

#define BUFFER_OFFSET(i) ((char *)NULL + (i))



namespace
{
    static const GLsizeiptr positionOffset = 0;
    static const GLsizeiptr normalOffset = sizeof(Vertex::position);
    static const GLsizeiptr tangentOffset = sizeof(Vertex::position)+ sizeof(Vertex::normal);
    static const GLsizeiptr uvOffset = sizeof(Vertex::position)+ sizeof(Vertex::normal) + sizeof(Vertex::tangent);
    static const GLsizei stride = sizeof(Vertex);
    static const unsigned int MAX_CONTACTS = 1024;

    static inline void computeContactMVP(Contact* c, QMatrix4x4& modelViewMatrix, QMatrix4x4& projMatrix)
    {
        Eigen::Vector3f t = c->p;

        Eigen::Vector3f forward = c->n;
        forward.normalize();
        Eigen::Vector3f right = c->t1;
        right.normalize();
        Eigen::Vector3f up = c->t2;
        up.normalize();

        modelViewMatrix.setToIdentity();
        modelViewMatrix.setRow(0, QVector4D(right[0], right[1], right[2], -right[0]*t[0] - right[1]*t[1] - right[2]*t[2]) );
        modelViewMatrix.setRow(1, QVector4D(up[0], up[1], up[2], -up[0]*t[0] - up[1]*t[1] - up[2]*t[2]));
        modelViewMatrix.setRow(2, QVector4D(forward[0], forward[1], forward[2], -forward[0]*t[0] - forward[1]*t[1] - forward[2]*t[2]));
        modelViewMatrix.setRow(3, QVector4D(0.0f, 0.0f, 0.0f, 1.0f));

        const float dim = Contact::patchSize/2.0f;
        const float phi = c->pene + 0.01f * dim;
        projMatrix.setToIdentity();
        projMatrix.ortho(-dim, dim, -dim, dim, -phi, phi);
    }

    static inline void drawTextureNormals(RigidBody* body, const MeshCollisionData& meshData, GLfloat* contactVerts)
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

        const Eigen::Vector3f p = body->R * computePosition(body->mesh, meshData) + body->x;
        const Eigen::Vector3f p2 = p + 0.5f*nworld;
        memcpy(contactVerts, p.data(), sizeof(GLfloat)*3);
        memcpy(contactVerts+3, p2.data(), sizeof(GLfloat)*3);
    }

    static inline void drawBody(QOpenGLFunctions* gl, ShaderVars& shader, const QMatrix4x4& modelViewMatrix, RigidBody* body, bool useLighting)
    {
        const Eigen::Vector3f& t = body->x;
        const Eigen::Matrix3f& R = body->R;

        QMatrix4x4 bodyTM;
        bodyTM(0,0) = R(0,0); bodyTM(1,0) = R(1,0); bodyTM(2,0) = R(2,0);
        bodyTM(0,1) = R(0,1); bodyTM(1,1) = R(1,1); bodyTM(2,1) = R(2,1);
        bodyTM(0,2) = R(0,2); bodyTM(1,2) = R(1,2); bodyTM(2,2) = R(2,2);
        bodyTM(0,3) = t(0);
        bodyTM(1,3) = t(1);
        bodyTM(2,3) = t(2);

        if( body->mesh != nullptr )
        {
            shader.program->bind();

            gl->glUniform1i(shader.useLightingLoc, (useLighting ? 1 : 0));
            gl->glUniform3f(shader.KdLoc, body->mesh->material.Kd[0], body->mesh->material.Kd[1], body->mesh->material.Kd[2]);
            gl->glUniform3f(shader.KsLoc, body->mesh->material.Ks[0], body->mesh->material.Ks[1], body->mesh->material.Ks[2]);
            gl->glUniform1f(shader.KnLoc, body->mesh->material.Kn);
            if( body->mesh->material.map_diffuse )
            {
                gl->glUniform1i(shader.useTextureLoc, 1);
                gl->glActiveTexture(GL_TEXTURE0);
                body->mesh->material.map_diffuse->bind();
            }
            else
            {
                gl->glUniform1i(shader.useTextureLoc, 0);
            }

            if( body->mesh->material.map_bump )
            {
                gl->glUniform1i(shader.useNormalMapLoc, 1);
                gl->glActiveTexture(GL_TEXTURE1);
                body->mesh->material.map_bump->bind();
            }
            else
            {
                gl->glUniform1i(shader.useNormalMapLoc, 0);
            }

            const GLuint first = body->mesh->vboOffset / sizeof(Vertex);
            const GLsizei count = body->mesh->vertices.size();

            const QMatrix4x4 compoundMatrix = modelViewMatrix * bodyTM;
            shader.program->setUniformValue(shader.mvMatrixLoc, compoundMatrix);
            shader.program->setUniformValue(shader.normalMatrixLoc, compoundMatrix.normalMatrix());

            gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            gl->glDrawArrays(GL_TRIANGLES, first, count);

        }
    }

    static inline void drawBodyNormals(QOpenGLFunctions* gl, ShaderVars& shader, const QMatrix4x4& modelViewMatrix, RigidBody* body)
    {
        const Eigen::Vector3f& t = body->x;
        const Eigen::Matrix3f& R = body->R;

        QMatrix4x4 bodyTM;
        bodyTM(0,0) = R(0,0); bodyTM(1,0) = R(1,0); bodyTM(2,0) = R(2,0);
        bodyTM(0,1) = R(0,1); bodyTM(1,1) = R(1,1); bodyTM(2,1) = R(2,1);
        bodyTM(0,2) = R(0,2); bodyTM(1,2) = R(1,2); bodyTM(2,2) = R(2,2);
        bodyTM(0,3) = t(0); bodyTM(1,3) = t(1); bodyTM(2,3) = t(2); bodyTM(3,3) = 1.0f;

        if( body->mesh != nullptr )
        {
            if( !body->mesh->material.map_bump )
                return;

            shader.program->bind();

            if( body->mesh->material.map_bump )
            {
                gl->glActiveTexture(GL_TEXTURE1);
                body->mesh->material.map_bump->bind();
            }

            const GLuint first = body->mesh->vboOffset / sizeof(Vertex);
            const GLsizei count = body->mesh->vertices.size();

            const QMatrix4x4 compoundMatrix = modelViewMatrix * bodyTM;
            shader.program->setUniformValue(shader.mvMatrixLoc, compoundMatrix);
            shader.program->setUniformValue(shader.normalMatrixLoc, compoundMatrix.normalMatrix());

            const float texWidth = (float)body->mesh->material.img_bump.width();
            const float texHeight = (float)body->mesh->material.img_bump.height();
            QVector4D texIncrement(1.0f / texWidth, 1.0f / texHeight, 0.0f, -1.0f / texHeight);
            shader.program->setUniformValue(shader.userParams[4], texIncrement);

            gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            gl->glDrawArrays(GL_TRIANGLES, first, count);
        }
    }
}

RigidBodyRenderer::RigidBodyRenderer(QOpenGLFunctions_4_3_Core* _gl, RigidBodySystem* _system) :
    m_gl(_gl),
    m_drawBodies(true),
    m_drawContacts(true),
    m_system(_system),
    m_lighting()
{
    init();
}

RigidBodyRenderer::~RigidBodyRenderer()
{
    delete m_objectShader.program;
}

void RigidBodyRenderer::setDrawBodiesEnabled(bool _drawBodies)
{
    m_drawBodies = _drawBodies;
}

void RigidBodyRenderer::setDrawContactsEnabled(bool _drawContacts)
{
    m_drawContacts = _drawContacts;
}

void RigidBodyRenderer::updateMeshVBOs()
{
    // Here, two passes are performed over the rigid body meshes to update the mesh VBO.
    //
    // First pass: estimate buffer size
    unsigned int dataSize = 0;
    auto& meshCache = MeshAssetRegistry::cachedMeshes();
    for(MeshCache::iterator meshItr = meshCache.begin(); meshItr != meshCache.end(); ++meshItr)
    {
        const unsigned int numVerts = meshItr->second.vertices.size();
        dataSize += numVerts * sizeof(Vertex);
    }
    m_objectShader.program->bind();
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);
    m_gl->glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW); // was STATIC... but wants to get updated?

    // Second pass: copy data
    unsigned int vboOffset =0;
    for(MeshCache::iterator meshItr = meshCache.begin(); meshItr != meshCache.end(); ++meshItr)
    {
        const unsigned int numVerts = meshItr->second.vertices.size();
        const unsigned int size = numVerts * sizeof(Vertex);

        meshItr->second.vboOffset = vboOffset;
        m_gl->glBufferSubData(GL_ARRAY_BUFFER, vboOffset, size, &meshItr->second.vertices[0]);
        vboOffset += size;
    }
}

void RigidBodyRenderer::init()
{
    assert(m_system != nullptr);

    // Body shader.
    {
        m_gl->glGenVertexArrays(1, &m_meshVAO);
        m_gl->glBindVertexArray(m_meshVAO);

        // Init object shader.
        //
        m_objectShader.program = new QOpenGLShaderProgram;
        if (!m_objectShader.program->addShaderFromSourceFile(QOpenGLShader::Vertex, "glsl/basicShader.vert")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() << m_objectShader.program->log();
        }
        if (!m_objectShader.program->addShaderFromSourceFile(QOpenGLShader::Fragment, "glsl/basicShader.frag")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() << m_objectShader.program->log();
        }
        m_objectShader.program->link();
        m_objectShader.program->bind();

        // The strings "vPosition", "mvMatrix", etc. have to match an attribute name in the vertex shader.
        QString shaderParameter;
        shaderParameter = "vPosition";
        if ((m_objectShader.vPositionLoc = m_objectShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vNormal";
        if ((m_objectShader.vNormalLoc = m_objectShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vTangent";
        if ((m_objectShader.vTangentLoc = m_objectShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vUV";
        if ((m_objectShader.vUvLoc = m_objectShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "projMatrix";
        if ((m_objectShader.projMatrixLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "mvMatrix";
        if ((m_objectShader.mvMatrixLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "normalMatrix";
        if ((m_objectShader.normalMatrixLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "lPosition";
        if ((m_objectShader.lPositionLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "Kd";
        if ((m_objectShader.KdLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "Ks";
        if ((m_objectShader.KsLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "Kn";
        if ((m_objectShader.KnLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "lKd";
        if ((m_objectShader.lKdLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "lKs";
        if ((m_objectShader.lKsLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "lKa";
        if ((m_objectShader.lKaLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "useTexture";
        if ((m_objectShader.useTextureLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "useLighting";
        if ((m_objectShader.useLightingLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "useNormalMap";
        if ((m_objectShader.useNormalMapLoc = m_objectShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        // Create mesh VBO object
        //
        GLsizei dataSize = 1;       // arbitrary size, we'll fill it with data later
        m_gl->glGenBuffers(1, &m_meshVBO);
        m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);
        m_gl->glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW); // was static, but the contact info needs to get updated! ??

        m_gl->glVertexAttribPointer(m_objectShader.vPositionLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(positionOffset));
        m_gl->glEnableVertexAttribArray(m_objectShader.vPositionLoc);
        m_gl->glVertexAttribPointer(m_objectShader.vNormalLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(normalOffset));
        m_gl->glEnableVertexAttribArray(m_objectShader.vNormalLoc);
        m_gl->glVertexAttribPointer(m_objectShader.vTangentLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(tangentOffset));
        m_gl->glEnableVertexAttribArray(m_objectShader.vTangentLoc);
        m_gl->glVertexAttribPointer(m_objectShader.vUvLoc, 2, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(uvOffset));
        m_gl->glEnableVertexAttribArray(m_objectShader.vUvLoc);
        m_objectShader.program->bind();
    }
    // Setup contact shader.
    {
        // Fill VBO with vertices data
        //  vert positions + normals
        contactVerts.resize(16*MAX_CONTACTS);
        std::fill(contactVerts.begin(), contactVerts.end(), 0.0f);

        // Init shaders
        m_contactShader.program = new QOpenGLShaderProgram;
        if (! m_contactShader.program->addShaderFromSourceFile(QOpenGLShader::Vertex, "glsl/spring.vert")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactShader.program->log();
        }
        if (! m_contactShader.program->addShaderFromSourceFile(QOpenGLShader::Fragment, "glsl/spring.frag")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactShader.program->log();
        }
        m_contactShader.program->link();
        m_contactShader.program->bind();

        // The strings "vPosition", "mvMatrix", etc. have to match an attribute name in the vertex shader.
        QString shaderParameter;
        shaderParameter = "vPosition";
        if ((m_contactShader.vPositionLoc = m_contactShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "projMatrix";
        if ((m_contactShader.projMatrixLoc = m_contactShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "mvMatrix";
        if ((m_contactShader.mvMatrixLoc = m_contactShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "Kd";
        if ((m_contactShader.KdLoc = m_contactShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        m_gl->glGenVertexArrays(1, &m_contactVAO);
        m_gl->glGenBuffers(1, &m_contactVBO);

        // Set VAO that binds the shader vertices inputs to the buffer data
        m_gl->glBindVertexArray(m_contactVAO);
        m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_contactVBO);

        const GLsizei vertsSize = contactVerts.size() * sizeof(GLfloat);
        const GLsizei dataSize = vertsSize;
        const GLsizei stride = 0;
        const GLsizeiptr positionOffset = 0;

        // Initialize a buffer of versSize+normalsSize
        //
        m_gl->glBufferData(GL_ARRAY_BUFFER, vertsSize, NULL, GL_DYNAMIC_DRAW);
        // Copy vertex positions into first part...
        //
        m_gl->glBufferSubData(GL_ARRAY_BUFFER, 0, vertsSize, contactVerts.data());

        // Setup shader pointers to position and normal data
        //
        m_gl->glVertexAttribPointer(m_contactShader.vPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
        m_gl->glEnableVertexAttribArray(m_contactShader.vPositionLoc);
    }

    // Setup depth shader.
    {
        // Init shaders
        m_depthShader.program = new QOpenGLShaderProgram;
        if (! m_depthShader.program->addShaderFromSourceFile(QOpenGLShader::Vertex, "glsl/depthShader.vert")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_depthShader.program->log();
        }
        if (! m_depthShader.program->addShaderFromSourceFile(QOpenGLShader::Fragment, "glsl/depthShader.frag")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_depthShader.program->log();
        }
        m_depthShader.program->link();
        m_depthShader.program->bind();

        // The strings "vPosition", "mvMatrix", etc. have to match an attribute name in the vertex shader.
        QString shaderParameter;
        shaderParameter = "vPosition";
        if ((m_depthShader.vPositionLoc = m_depthShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "projMatrix";
        if ((m_depthShader.projMatrixLoc = m_depthShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "mvMatrix";
        if ((m_depthShader.mvMatrixLoc = m_depthShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        // Set VAO that binds the shader vertices inputs to the buffer data
        m_gl->glBindVertexArray(m_meshVAO);
        m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);

        m_gl->glVertexAttribPointer(m_depthShader.vPositionLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(positionOffset));
        m_gl->glEnableVertexAttribArray(m_depthShader.vPositionLoc);
    }

    // Setup contact coefficient shader.
    {
        // Init shaders
        m_contactCoeffShader.program = new QOpenGLShaderProgram;
        if (! m_contactCoeffShader.program->addShaderFromSourceFile(QOpenGLShader::Vertex, "glsl/contactTextureShader.vert")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactCoeffShader.program->log();
        }
        if (! m_contactCoeffShader.program->addShaderFromSourceFile(QOpenGLShader::Fragment, "glsl/contactTextureShader.frag")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactCoeffShader.program->log();
        }
        m_contactCoeffShader.program->link();
        m_contactCoeffShader.program->bind();

        // The strings "vPosition", "mvMatrix", etc. have to match an attribute name in the vertex shader.
        QString shaderParameter;
        shaderParameter = "vPosition";
        if ((m_contactCoeffShader.vPositionLoc = m_contactCoeffShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vNormal";
        if ((m_contactCoeffShader.vNormalLoc = m_contactCoeffShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vTangent";
        if ((m_contactCoeffShader.vTangentLoc = m_contactCoeffShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vUV";
        if ((m_contactCoeffShader.vUvLoc = m_contactCoeffShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "projMatrix";
        if ((m_contactCoeffShader.projMatrixLoc = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "mvMatrix";
        if ((m_contactCoeffShader.mvMatrixLoc = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "normalMatrix";
        if ((m_contactCoeffShader.normalMatrixLoc = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "ncontact";
        if ((m_contactCoeffShader.userParams[0] = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;
        shaderParameter = "d0";
        if ((m_contactCoeffShader.userParams[1] = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;
        shaderParameter = "d1";
        if ((m_contactCoeffShader.userParams[2] = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;
        shaderParameter = "sigma";
        if ((m_contactCoeffShader.userParams[3] = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;
        shaderParameter = "texIncrement";
        if ((m_contactCoeffShader.userParams[4] = m_contactCoeffShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;


        // Set VAO that binds the shader vertices inputs to the buffer data
        m_gl->glBindVertexArray(m_meshVAO);
        m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);

        m_gl->glVertexAttribPointer(m_contactCoeffShader.vPositionLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(positionOffset));
        m_gl->glEnableVertexAttribArray(m_contactCoeffShader.vPositionLoc);
        m_gl->glVertexAttribPointer(m_contactCoeffShader.vNormalLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(normalOffset));
        m_gl->glEnableVertexAttribArray(m_contactCoeffShader.vNormalLoc);
        m_gl->glVertexAttribPointer(m_contactCoeffShader.vTangentLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(tangentOffset));
        m_gl->glEnableVertexAttribArray(m_contactCoeffShader.vTangentLoc);
        m_gl->glVertexAttribPointer(m_contactCoeffShader.vUvLoc, 2, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(uvOffset));
        m_gl->glEnableVertexAttribArray(m_contactCoeffShader.vUvLoc);
    }

    // Setup contact weight shader.
    {
        // Init shaders
        m_contactWeightShader.program = new QOpenGLShaderProgram;
        if (! m_contactWeightShader.program->addShaderFromSourceFile(QOpenGLShader::Vertex, "glsl/contactWeightShader.vert")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactWeightShader.program->log();
        }
        if (! m_contactWeightShader.program->addShaderFromSourceFile(QOpenGLShader::Fragment, "glsl/contactWeightShader.frag")) {
            qDebug() << "Unable to load shader" << endl
                     << "Log file:" << endl;
            qDebug() <<  m_contactWeightShader.program->log();
        }
        m_contactWeightShader.program->link();
        m_contactWeightShader.program->bind();

        // The strings "vPosition", "mvMatrix", etc. have to match an attribute name in the vertex shader.
        QString shaderParameter;
        shaderParameter = "vPosition";
        if ((m_contactWeightShader.vPositionLoc = m_contactWeightShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vNormal";
        if ((m_contactWeightShader.vNormalLoc = m_contactWeightShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vTangent";
        if ((m_contactWeightShader.vTangentLoc = m_contactWeightShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "vUV";
        if ((m_contactWeightShader.vUvLoc = m_contactWeightShader.program->attributeLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "projMatrix";
        if ((m_contactWeightShader.projMatrixLoc = m_contactWeightShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "mvMatrix";
        if ((m_contactWeightShader.mvMatrixLoc = m_contactWeightShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "normalMatrix";
        if ((m_contactWeightShader.normalMatrixLoc = m_contactWeightShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;

        shaderParameter = "texIncrement";
        if ((m_contactWeightShader.userParams[4] = m_contactWeightShader.program->uniformLocation(shaderParameter)) < 0)
            qDebug() << "Unable to find shader location for " << shaderParameter;


        // Set VAO that binds the shader vertices inputs to the buffer data
        m_gl->glBindVertexArray(m_meshVAO);
        m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);

        m_gl->glVertexAttribPointer(m_contactWeightShader.vPositionLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(positionOffset));
        m_gl->glEnableVertexAttribArray(m_contactWeightShader.vPositionLoc);
        m_gl->glVertexAttribPointer(m_contactWeightShader.vNormalLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(normalOffset));
        m_gl->glEnableVertexAttribArray(m_contactWeightShader.vNormalLoc);
        m_gl->glVertexAttribPointer(m_contactWeightShader.vTangentLoc, 3, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(tangentOffset));
        m_gl->glEnableVertexAttribArray(m_contactWeightShader.vTangentLoc);
        m_gl->glVertexAttribPointer(m_contactWeightShader.vUvLoc, 2, GL_FLOAT, GL_FALSE, stride, BUFFER_OFFSET(uvOffset));
        m_gl->glEnableVertexAttribArray(m_contactWeightShader.vUvLoc);
    }

    // Bind default shader
    m_objectShader.program->bind();

    // Enable backface culling
    m_gl->glCullFace(GL_BACK);
    m_gl->glEnable(GL_CULL_FACE);
    m_gl->glEnable(GL_DEPTH_TEST);
    m_gl->glDepthFunc(GL_LESS);
}

void RigidBodyRenderer::draw(const QMatrix4x4& projectionMatrix, const QMatrix4x4& modelViewMatrix)
{
    if( m_drawBodies )
    {
        drawBodies(projectionMatrix, modelViewMatrix);
    }

    if( m_drawContacts )
    {
        drawContacts(projectionMatrix, modelViewMatrix);
        //drawNormals(projectionMatrix, modelViewMatrix);
    }
}

void RigidBodyRenderer::drawDepth(const QMatrix4x4 &projectionMatrix, const QMatrix4x4 &modelViewMatrix)
{
    m_depthShader.program->bind();
    m_depthShader.program->setUniformValue(m_depthShader.projMatrixLoc, projectionMatrix);

    // Set VAO that binds the shader vertices inputs to the buffer data.
    m_gl->glBindVertexArray(m_meshVAO);

    QMatrix4x4 bodyTM;
    QMatrix4x4 scaleMatrix;
    bodyTM.setToIdentity();
    scaleMatrix.setToIdentity();
    m_gl->glPolygonMode(GL_FRONT, GL_FILL);
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);

    const auto bodies = m_system->getBodies();
    GLuint first = 0;
    GLsizei count = 0;

    for(RigidBody* b : bodies)
    {
        const Eigen::Vector3f t = b->x;
        const Eigen::Matrix3f R = b->R;
        bodyTM(0,0) = R(0,0); bodyTM(1,0) = R(1,0); bodyTM(2,0) = R(2,0);
        bodyTM(0,1) = R(0,1); bodyTM(1,1) = R(1,1); bodyTM(2,1) = R(2,1);
        bodyTM(0,2) = R(0,2); bodyTM(1,2) = R(1,2); bodyTM(2,2) = R(2,2);
        bodyTM(0,3) = t(0);
        bodyTM(1,3) = t(1);
        bodyTM(2,3) = t(2);

        if( b->mesh != nullptr )
        {
            first = b->mesh->vboOffset / sizeof(Vertex);
            count = b->mesh->vertices.size();
        }

        const QMatrix4x4 compoundMatrix = modelViewMatrix * bodyTM * scaleMatrix;
        m_depthShader.program->setUniformValue(m_depthShader.mvMatrixLoc, compoundMatrix);
        m_gl->glDrawArrays(GL_TRIANGLES, first, count);
    }
}

void RigidBodyRenderer::drawBodies(const QMatrix4x4& projectionMatrix, const QMatrix4x4& modelViewMatrix)
{
    m_objectShader.program->bind();
    m_objectShader.program->setUniformValue(m_objectShader.projMatrixLoc, projectionMatrix);

    // Set VAO that binds the shader vertices inputs to the buffer data.
    m_gl->glBindVertexArray(m_meshVAO);

    QMatrix4x4 bodyTM;
    bodyTM.setToIdentity();
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);

    const auto bodies = m_system->getBodies();
    GLuint first = 0;
    GLsizei count = 0;

    const QVector4D lPosition = modelViewMatrix * m_lighting.pos;
    m_objectShader.program->setUniformValue(m_objectShader.lPositionLoc, lPosition);
    m_objectShader.program->setUniformValue(m_objectShader.lKdLoc, m_lighting.diffuse);
    m_objectShader.program->setUniformValue(m_objectShader.lKsLoc, m_lighting.specular);
    m_objectShader.program->setUniformValue(m_objectShader.lKaLoc, m_lighting.ambient);

    for(RigidBody* b : bodies)
    {
        drawBody(m_gl, m_objectShader, modelViewMatrix, b, true);
    }
}

void RigidBodyRenderer::drawContacts(const QMatrix4x4& projectionMatrix, const QMatrix4x4& modelViewMatrix)
{
    const auto& contacts = m_system->getContacts();
    const GLuint numContacts = (MAX_CONTACTS < contacts.size()) ? MAX_CONTACTS : contacts.size();

    // Normal axes
    int offset = 0;
    for(int i = 0; i < numContacts; ++i)
    {
        const Contact& c = *contacts[i];
        const Eigen::Vector3f p2 = c.p + 0.5f*c.n;
        memcpy(&contactVerts[offset+6*i], c.p.data(), sizeof(GLfloat)*3);
        memcpy(&contactVerts[offset+6*i+3], p2.data(), sizeof(GLfloat)*3);
    }
    // First tangent axes
    offset += 6*numContacts;
    for(int i = 0; i < numContacts; ++i)
    {
        const Contact& c = *contacts[i];
        const Eigen::Vector3f p2 = c.p + 0.5f*c.t1;
        memcpy(&contactVerts[offset+6*i], c.p.data(), sizeof(GLfloat)*3);
        memcpy(&contactVerts[offset+6*i+3], p2.data(), sizeof(GLfloat)*3);
    }
    // Second tangent axes
    offset += 6*numContacts;
    for(int i = 0; i < numContacts; ++i)
    {
        const Contact& c = *contacts[i];
        const Eigen::Vector3f p2 = c.p + 0.5f*c.t2;
        memcpy(&contactVerts[offset+6*i], c.p.data(), sizeof(GLfloat)*3);
        memcpy(&contactVerts[offset+6*i+3], p2.data(), sizeof(GLfloat)*3);
    }

    m_contactShader.program->bind();
    m_contactShader.program->setUniformValue(m_contactShader.projMatrixLoc, projectionMatrix);
    m_contactShader.program->setUniformValue(m_contactShader.mvMatrixLoc, modelViewMatrix);

    // Set VAO that binds the shader vertices inputs to the buffer data
    m_gl->glBindVertexArray(m_contactVAO);
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_contactVBO);

    // Copy vertex positions
    const GLsizei vertsSize = contactVerts.size() * sizeof(GLfloat);
    m_gl->glBufferSubData(GL_ARRAY_BUFFER, 0, vertsSize, contactVerts.data());
    m_gl->glDisable(GL_DEPTH_TEST);
    m_gl->glDisable(GL_CULL_FACE);
    m_gl->glLineWidth(2.0f);
    m_gl->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Draw normal axes
    m_gl->glUniform3f(m_contactShader.KdLoc, 0.1f, 0.1f, 1.0f);
    m_gl->glDrawArrays(GL_LINES, 0, 2*numContacts);
    // Draw t1 axes
    m_gl->glUniform3f(m_contactShader.KdLoc, 1.0f, 0.1f, 0.1f);
    m_gl->glDrawArrays(GL_LINES, 2*numContacts, 2*numContacts);
    // Draw t2 axes
    m_gl->glUniform3f(m_contactShader.KdLoc, 0.1f, 1.0f, 0.1f);
    m_gl->glDrawArrays(GL_LINES, 4*numContacts, 2*numContacts);

    m_gl->glEnable(GL_DEPTH_TEST);
    m_gl->glEnable(GL_CULL_FACE);
}

void RigidBodyRenderer::drawNormals(const QMatrix4x4& projectionMatrix, const QMatrix4x4& modelViewMatrix)
{
    const auto& contacts = m_system->getContacts();
    const GLuint numContacts = (MAX_CONTACTS < contacts.size()) ? MAX_CONTACTS : contacts.size();

    for(int i = 0; i < numContacts; ++i)
    {
        const Contact& c = *contacts[i];
        drawTextureNormals(c.body0, c.meshData0, &contactVerts[12*i]);
        drawTextureNormals(c.body1, c.meshData1, &contactVerts[12*i+6]);
    }

    m_contactShader.program->bind();
    m_contactShader.program->setUniformValue(m_contactShader.projMatrixLoc, projectionMatrix);
    m_contactShader.program->setUniformValue(m_contactShader.mvMatrixLoc, modelViewMatrix);

    // Set VAO that binds the shader vertices inputs to the buffer data
    m_gl->glBindVertexArray(m_contactVAO);
    m_gl->glBindBuffer(GL_ARRAY_BUFFER, m_contactVBO);

    // Copy vertex positions
    const GLsizei vertsSize = contactVerts.size() * sizeof(GLfloat);
    m_gl->glBufferSubData(GL_ARRAY_BUFFER, 0, vertsSize, contactVerts.data());
    // Colour.
    m_gl->glUniform3f(m_contactShader.KdLoc, 1.0f, 1.0f, 0.1f);

    m_gl->glDisable(GL_DEPTH_TEST);
    m_gl->glDisable(GL_CULL_FACE);
    m_gl->glLineWidth(2.0f);
    m_gl->glDrawArrays(GL_LINES, 0, 4*numContacts);
    m_gl->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    m_gl->glEnable(GL_DEPTH_TEST);
    m_gl->glEnable(GL_CULL_FACE);
}

void RigidBodyRenderer::drawContactCoeffs(Contact* c, int bodyIndex)
{
    static const float sSigma[2] = { -1.0f, 1.0f };
    QMatrix4x4 projMatrix, modelViewMatrix;
    computeContactMVP(c, modelViewMatrix, projMatrix);

    QVector3D ncontact = modelViewMatrix.mapVector( QVector3D(c->n[0], c->n[1], c->n[2]) );
    QVector3D d0 = modelViewMatrix.mapVector( -QVector3D(c->t1[0], c->t1[1], c->t1[2]) );
    QVector3D d1 = modelViewMatrix.mapVector( -QVector3D(c->t2[0], c->t2[1], c->t2[2]) );

    m_contactCoeffShader.program->bind();
    m_contactCoeffShader.program->setUniformValue(m_contactCoeffShader.projMatrixLoc, projMatrix);
    m_contactCoeffShader.program->setUniformValue(m_contactCoeffShader.userParams[0], ncontact);
    m_contactCoeffShader.program->setUniformValue(m_contactCoeffShader.userParams[1], d0);
    m_contactCoeffShader.program->setUniformValue(m_contactCoeffShader.userParams[2], d1);

    // Set VAO that binds the shader vertices inputs to the buffer data.
    m_gl->glBindVertexArray(m_meshVAO);

    // Draw the patch of body at the specified index
    RigidBody* bodies[2] = { c->body0, c->body1 };
    m_contactCoeffShader.program->setUniformValue(m_contactCoeffShader.userParams[3], sSigma[bodyIndex]);
    drawBodyNormals(m_gl, m_contactCoeffShader, modelViewMatrix, bodies[bodyIndex]);

}

void RigidBodyRenderer::drawContactWeights(Contact* c, int bodyIndex)
{
    QMatrix4x4 projMatrix, modelViewMatrix;
    computeContactMVP(c, modelViewMatrix, projMatrix);

    m_contactWeightShader.program->bind();
    m_contactWeightShader.program->setUniformValue(m_contactWeightShader.projMatrixLoc, projMatrix);

    // Set VAO that binds the shader vertices inputs to the buffer data.
    m_gl->glBindVertexArray(m_meshVAO);

    // Draw the patch of body at the specified index
    if( bodyIndex == 0 )
    {
        drawBodyNormals(m_gl, m_contactWeightShader, modelViewMatrix, c->body0);
    }
    else
    {
        drawBodyNormals(m_gl, m_contactWeightShader, modelViewMatrix, c->body1);
    }

}
