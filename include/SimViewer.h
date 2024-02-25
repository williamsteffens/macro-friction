#pragma once

/**
 * @file SimViewer.h
 *
 * @brief Viewer for a cloth simulation application.
 *
 */

#include <QOpenGLFunctions_4_3_Core>
#include <qglviewer.h>
#include <QPoint>
#include <Contact.h>
#include <Eigen/Dense>

#include "ShaderVars.h"
#include "SimulationSerializer.h"

QT_FORWARD_DECLARE_CLASS(RigidBodyRenderer)
QT_FORWARD_DECLARE_CLASS(RigidBodySystem)
QT_FORWARD_DECLARE_CLASS(QOpenGLFramebufferObject)
QT_FORWARD_DECLARE_CLASS(FBOWidget)
QT_FORWARD_DECLARE_CLASS(FrictionLimitsWidget)
QT_FORWARD_DECLARE_CLASS(SimulationSerializer)

class SimViewer : public QGLViewer
{
    Q_OBJECT

public:
    SimViewer();
    virtual ~SimViewer();

    struct PickingData
    {
        PickingData() : depth(FLT_MAX), body(nullptr), plocal() {}
        float depth;
        Eigen::Vector3f plocal;
        RigidBody* body;
        qglviewer::Vec cursor;
    };

public slots:
    void cleanup();
    void setDrawMesh(bool);
    void setDrawContacts(bool);
    void setTimestep(double);
    void setSubsteps(int);
    void setMaxIterations(int);
    void setFrictionCoefficient(double);
    void setContactStiffness(double);
    void setContactDamping(double);
    void setPloughingCoeffA(double);
    void setPloughingCoeffB(double);
    void setPaused(bool);
    void stepOnce();
    void setContactFrameAlign(bool);
    void setSaveFrames(bool);

    void setPatchSize(double);
    void setPatchResolution(int);

    void setExtForceMagnitude(double);
    void setExtForceDir(int);
    void setExtForceId(int);
    void applyExtImpulse();

    void setFrameBufferWidget(QWidget* _widget);

    void createBoxOnInclinedPlane();
    void createThreeBoxesSliding();
    void createTwoBoxesAnisotropic();
    void createBoxOnRoughPlane();
    void createStack();
    void createBoxOnSawtoothPlane();
    void createBolaRough();
    void createBolaSmooth();
    void createNutAndBolt();
    void createTestSystem(); // a throw away test system

signals:

    void statusMessageChanged(const QString&);

protected :
    virtual void animate() override;
    virtual void draw() override;
    virtual void init() override;

    virtual void mouseMoveEvent(QMouseEvent* e) override;
    virtual void mousePressEvent(QMouseEvent *e) override;
    virtual void mouseReleaseEvent(QMouseEvent* e) override;

    QOpenGLFunctions_4_3_Core m_gl;

    void renderContactTextures(std::vector<Contact*>&);
    void preStep(std::vector<RigidBody*>&);
    void onReset();

private:

    // Simulation parameters
    float m_dt;                         //< Time step parameter.
    int m_subSteps;
    bool m_paused;                      //< Pause the simulation.
    bool m_stepOnce;                    //< Advance the simulation by one frame and then stop.
    bool m_saveFrames;                  //< Enable writing frames using the SimulationSerializer.

    int m_extImpulseId;
    float m_extImpulseMag;
    int m_extImpulseDir;
    int m_extImpulseCounter;


    GLuint m_mouseVAO;
    GLuint m_mouseVBO;
    ShaderVars m_mouseShader;

    enum ePickType { kPickNone = 0, kPickMouseSpring, kPickFreeze } ;
    ePickType m_pickFlag;               //< Flag indicating if the user tried to pick a particle.
    int m_mouseX, m_mouseY;             //< Mouse x,y position on the screen (in pixels).
    PickingData m_pickingData;          //< Mouse picking data.

    // Offscreen render buffer
    GLuint m_renderBuf;
    QOpenGLFramebufferObject* m_fbo;
    QWidget* m_fboWidget;
    QImage m_fboImage;
    FBOWidget* m_coeffWidgetsA[4];
    FBOWidget* m_coeffWidgetsB[4];
    FrictionLimitsWidget* m_frictionLimitsWidget;

    std::chrono::microseconds m_renderMicros;
    std::chrono::microseconds m_convolveMicros;
    std::chrono::microseconds m_dynamicsMicros;
    int m_numContacts;

    std::unique_ptr<RigidBodySystem> m_rigidBodySystem;
    std::unique_ptr<RigidBodyRenderer> m_rigidBodyRenderer;
    SimulationSerializer m_serializer;
};
